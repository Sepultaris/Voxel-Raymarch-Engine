#include "planet/geodesic_topology.hpp"
#include "planet/geodesic_material.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <numbers>
#include <set>
#include <stdexcept>
#include <tuple>

namespace voxel {
namespace {

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(Vec3 value, double scale) { return {value.x * scale, value.y * scale, value.z * scale}; }

double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

Vec3 normalized(Vec3 value) {
    const double length = std::sqrt(dot(value, value));
    if (length <= std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Cannot normalize a zero geodesic vector");
    }
    return value * (1.0 / length);
}

std::array<float, 4> packed(Vec3 value, float fourth = 0.0F) {
    return {static_cast<float>(value.x), static_cast<float>(value.y),
            static_cast<float>(value.z), fourth};
}

using QuantizedPosition = std::tuple<long long, long long, long long>;

QuantizedPosition quantized(Vec3 position) {
    constexpr double scale = 1'000'000'000.0;
    return {std::llround(position.x * scale),
            std::llround(position.y * scale),
            std::llround(position.z * scale)};
}

constexpr std::array<std::array<std::uint32_t, 3>, 20> kIcosahedronFaces{{
    {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
    {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
    {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
    {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1},
}};

std::uint32_t expandMortonBits(std::uint32_t value) {
    value &= 0x000003ffU;
    value = (value | (value << 16U)) & 0x030000ffU;
    value = (value | (value << 8U)) & 0x0300f00fU;
    value = (value | (value << 4U)) & 0x030c30c3U;
    value = (value | (value << 2U)) & 0x09249249U;
    return value;
}

std::uint32_t mortonCode(Vec3 position) {
    const auto quantize = [](double component) {
        return static_cast<std::uint32_t>(std::clamp(
            std::llround((component * 0.5 + 0.5) * 1023.0), 0LL, 1023LL));
    };
    return expandMortonBits(quantize(position.x)) |
           (expandMortonBits(quantize(position.y)) << 1U) |
           (expandMortonBits(quantize(position.z)) << 2U);
}

std::array<std::uint32_t, 4> activeMask(std::uint32_t count) {
    const std::uint64_t bits = count >= 64U ? ~0ULL : (1ULL << count) - 1ULL;
    return {static_cast<std::uint32_t>(bits), static_cast<std::uint32_t>(bits >> 32U), 0U, 0U};
}

std::array<Vec3, 12> icosahedronVertices() {
    constexpr double phi = 1.6180339887498948482;
    std::array<Vec3, 12> vertices{{
        {-1, phi, 0}, {1, phi, 0}, {-1, -phi, 0}, {1, -phi, 0},
        {0, -1, phi}, {0, 1, phi}, {0, -1, -phi}, {0, 1, -phi},
        {phi, 0, -1}, {phi, 0, 1}, {-phi, 0, -1}, {-phi, 0, 1},
    }};
    for (Vec3& vertex : vertices) {
        vertex = normalized(vertex);
    }
    return vertices;
}

} // namespace

std::uint64_t GeodesicTopology::brickCellCapacity() const noexcept {
    return static_cast<std::uint64_t>(tiles.size()) * brickResolution;
}

std::uint64_t GeodesicTopology::residentBrickCellCapacity() const noexcept {
    return brickCells.size();
}

std::uint32_t GeodesicTopology::residentPageCount() const noexcept {
    return static_cast<std::uint32_t>(brickResolution == 0U ? 0U : brickCells.size() / brickResolution);
}

std::uint32_t GeodesicTopology::remapResidentPage(
    std::uint32_t physicalPageIndex, std::uint32_t tileIndex) {
    if (physicalPageIndex >= residentPageCount() || tileIndex >= tiles.size()) {
        throw std::out_of_range("Sparse geodesic page remap is out of range");
    }
    if ((pageTable[tileIndex].data[2] & kPageResident) != 0U) {
        if (pageTable[tileIndex].data[0] == physicalPageIndex) {
            return tileIndex;
        }
        throw std::invalid_argument("Cannot map a second physical page to a resident tile");
    }

    std::uint32_t evictedTile = kInvalidTile;
    for (std::uint32_t candidate = 0; candidate < pageTable.size(); ++candidate) {
        const auto& entry = pageTable[candidate];
        if ((entry.data[2] & kPageResident) != 0U && entry.data[0] == physicalPageIndex) {
            evictedTile = candidate;
            break;
        }
    }
    if (evictedTile == kInvalidTile) {
        throw std::runtime_error("Sparse geodesic physical page has no current owner");
    }

    return remapResidentPageFromOwner(physicalPageIndex, tileIndex, evictedTile);
}

std::uint32_t GeodesicTopology::remapResidentPageFromOwner(
    std::uint32_t physicalPageIndex, std::uint32_t tileIndex,
    std::uint32_t currentOwnerTile) {
    if (physicalPageIndex >= residentPageCount() || tileIndex >= tiles.size() ||
        currentOwnerTile >= tiles.size()) {
        throw std::out_of_range("Sparse geodesic direct page remap is out of range");
    }
    if ((pageTable[tileIndex].data[2] & kPageResident) != 0U) {
        if (pageTable[tileIndex].data[0] == physicalPageIndex) {
            return tileIndex;
        }
        throw std::invalid_argument("Cannot map a second physical page to a resident tile");
    }
    auto& previousOwner = pageTable[currentOwnerTile];
    if ((previousOwner.data[2] & kPageResident) == 0U ||
        previousOwner.data[0] != physicalPageIndex) {
        throw std::runtime_error("Sparse geodesic inverse page owner is stale");
    }
    const std::uint32_t previousGeneration = previousOwner.data[1];
    previousOwner.data = {kInvalidPage, previousGeneration, 0U, 0U};

    nextPageGeneration = std::max(nextPageGeneration, previousGeneration + 1U);
    pageTable[tileIndex].data = {physicalPageIndex, nextPageGeneration++, kPageResident, 0U};
    const auto& state = columnStates[tileIndex].data;
    const std::size_t physicalOffset =
        static_cast<std::size_t>(physicalPageIndex) * brickResolution;
    for (std::uint32_t layer = 0; layer < brickResolution; ++layer) {
        brickCells[physicalOffset + layer] =
            geodesicMaterialForLayer(state[1], state[2], layer);
    }
    return currentOwnerTile;
}

bool GeodesicTopology::remapResidentPageIfGeneration(
    std::uint32_t physicalPageIndex, std::uint32_t tileIndex,
    std::uint32_t expectedGeneration, std::uint32_t& evictedTile) {
    if (tileIndex >= pageTable.size()) {
        throw std::out_of_range("Sparse geodesic generation check is out of range");
    }
    if (pageTable[tileIndex].data[1] != expectedGeneration) {
        return false;
    }
    evictedTile = remapResidentPage(physicalPageIndex, tileIndex);
    return true;
}

GeodesicTopology GeodesicTopology::build(
    std::uint32_t frequency,
    std::uint32_t brickResolution,
    std::uint32_t lookupWidth,
    std::uint32_t lookupHeight,
    std::uint32_t residentPageCapacity,
    bool buildReferenceTraversal,
    const ProgressCallback& progress) {
    if (frequency == 0 || brickResolution == 0 || brickResolution > 32U ||
        lookupWidth == 0 || lookupHeight == 0) {
        throw std::invalid_argument("Geodesic dimensions are invalid or exceed the 32-layer mask");
    }

    const auto baseVertices = icosahedronVertices();
    std::vector<Vec3> positions;
    std::map<QuantizedPosition, std::uint32_t> vertexByPosition;
    std::set<std::pair<std::uint32_t, std::uint32_t>> edges;
    std::uint32_t triangleCount = 0;

    const auto addVertex = [&](Vec3 position) {
        position = normalized(position);
        const QuantizedPosition key = quantized(position);
        const auto existing = vertexByPosition.find(key);
        if (existing != vertexByPosition.end()) {
            return existing->second;
        }
        const auto index = static_cast<std::uint32_t>(positions.size());
        positions.push_back(position);
        vertexByPosition.emplace(key, index);
        return index;
    };
    const auto addEdge = [&](std::uint32_t a, std::uint32_t b) {
        edges.emplace(std::min(a, b), std::max(a, b));
    };

    if (progress) {
        progress("topology generation: subdividing icosahedron", 0U,
                 kIcosahedronFaces.size());
    }
    for (std::size_t faceIndex = 0; faceIndex < kIcosahedronFaces.size(); ++faceIndex) {
        const auto& face = kIcosahedronFaces[faceIndex];
        std::vector<std::vector<std::uint32_t>> grid(frequency + 1);
        for (std::uint32_t i = 0; i <= frequency; ++i) {
            grid[i].resize(frequency - i + 1);
            for (std::uint32_t j = 0; j <= frequency - i; ++j) {
                const double inverseFrequency = 1.0 / static_cast<double>(frequency);
                const double aWeight = static_cast<double>(frequency - i - j) * inverseFrequency;
                const double bWeight = static_cast<double>(i) * inverseFrequency;
                const double cWeight = static_cast<double>(j) * inverseFrequency;
                grid[i][j] = addVertex(baseVertices[face[0]] * aWeight +
                                       baseVertices[face[1]] * bWeight +
                                       baseVertices[face[2]] * cWeight);
            }
        }

        const auto addTriangle = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
            addEdge(a, b);
            addEdge(b, c);
            addEdge(c, a);
            ++triangleCount;
        };
        for (std::uint32_t i = 0; i < frequency; ++i) {
            for (std::uint32_t j = 0; j < frequency - i; ++j) {
                const std::uint32_t a = grid[i][j];
                const std::uint32_t b = grid[i + 1][j];
                const std::uint32_t c = grid[i][j + 1];
                addTriangle(a, b, c);
                if (i + j + 1 < frequency) {
                    addTriangle(b, grid[i + 1][j + 1], c);
                }
            }
        }
        if (progress) {
            progress("topology generation: subdividing icosahedron",
                     faceIndex + 1U, kIcosahedronFaces.size());
        }
    }

    std::vector<std::vector<std::uint32_t>> neighbors(positions.size());
    for (const auto& [a, b] : edges) {
        neighbors[a].push_back(b);
        neighbors[b].push_back(a);
    }

    GeodesicTopology result;
    result.frequency = frequency;
    result.brickResolution = brickResolution;
    result.triangleCount = triangleCount;
    result.lookupWidth = lookupWidth;
    result.lookupHeight = lookupHeight;
    result.referenceTraversalBuilt = buildReferenceTraversal;
    result.tiles.resize(positions.size());

    const std::uint64_t cellsPerBrick64 = brickResolution;
    if (cellsPerBrick64 > std::numeric_limits<std::uint32_t>::max() ||
        cellsPerBrick64 * positions.size() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::overflow_error("Tile brick address space exceeds 32-bit GPU offsets");
    }
    const auto cellsPerBrick = static_cast<std::uint32_t>(cellsPerBrick64);
    double bedrockInnerRadius = 0.0;
    double bedrockOuterRadius = std::numeric_limits<double>::max();

    if (progress) {
        progress("topology generation: dual cells", 0U, positions.size());
    }
    for (std::uint32_t tileIndex = 0; tileIndex < positions.size(); ++tileIndex) {
        const Vec3 center = positions[tileIndex];
        const Vec3 reference = std::abs(center.y) < 0.9 ? Vec3{0, 1, 0} : Vec3{0, 0, 1};
        const Vec3 tangent = normalized(cross(reference, center));
        const Vec3 bitangent = cross(center, tangent);
        auto& tileNeighbors = neighbors[tileIndex];
        std::ranges::sort(tileNeighbors, [&](std::uint32_t left, std::uint32_t right) {
            const Vec3 leftOffset = positions[left] - center * dot(positions[left], center);
            const Vec3 rightOffset = positions[right] - center * dot(positions[right], center);
            const double leftAngle = std::atan2(dot(leftOffset, bitangent), dot(leftOffset, tangent));
            const double rightAngle = std::atan2(dot(rightOffset, bitangent), dot(rightOffset, tangent));
            return leftAngle < rightAngle;
        });

        GeodesicTileGpu& tile = result.tiles[tileIndex];
        double maximumNeighborAngle = 0.0;
        for (const std::uint32_t neighbor : tileNeighbors) {
            maximumNeighborAngle = std::max(
                maximumNeighborAngle,
                std::acos(std::clamp(dot(center, positions[neighbor]), -1.0, 1.0)));
        }
        const float tangentHalfExtent = static_cast<float>(maximumNeighborAngle * 0.62);
        // One logical voxel spans the complete geodesic surface cell. Give
        // each radial layer half that local width, then stack the configured
        // layer count inward. Very coarse diagnostic topologies are capped so
        // their inner cap cannot pass through the planet center; production
        // frequencies retain the requested 1:2 height-to-width aspect ratio.
        constexpr float maximumRadialDepth = 0.25F;
        const float localSurfaceCellWidth = tangentHalfExtent * 2.0F;
        const float desiredLayerHeight = localSurfaceCellWidth * 0.5F;
        const float radialDepth = std::min(
            desiredLayerHeight * static_cast<float>(brickResolution),
            maximumRadialDepth);
        const double innerHeight = 1.0 - static_cast<double>(radialDepth);
        const double layerHeight = static_cast<double>(radialDepth) /
                                   static_cast<double>(brickResolution);
        // Every dual corner is inside maximumNeighborAngle. A radial sphere
        // above innerHeight/cos(angle) and below the first cap therefore lies
        // inside immutable layer zero for every possible angular owner.
        bedrockInnerRadius = std::max(
            bedrockInnerRadius,
            innerHeight / std::cos(maximumNeighborAngle));
        bedrockOuterRadius = std::min(
            bedrockOuterRadius, innerHeight + layerHeight);
        // Every dual corner lies inside the angular radius of the farthest
        // one-ring neighbor. Intersecting the outer cap plane therefore cannot
        // exceed sec(maximumNeighborAngle). This conservative bound lets the
        // production DDA build skip exact per-prism corner generation.
        result.outerBoundingRadius = std::max(
            result.outerBoundingRadius,
            static_cast<float>(1.0 / std::cos(maximumNeighborAngle)));
        const float tangentVoxelSize = localSurfaceCellWidth;
        tile.center = packed(center, tangentHalfExtent);
        tile.tangent = packed(tangent, radialDepth);
        tile.bitangent = packed(bitangent, tangentVoxelSize);
        tile.neighborsLow.fill(kInvalidTile);
        tile.neighborsHigh.fill(kInvalidTile);
        for (std::uint32_t neighborIndex = 0; neighborIndex < tileNeighbors.size(); ++neighborIndex) {
            if (neighborIndex < 4) {
                tile.neighborsLow[neighborIndex] = tileNeighbors[neighborIndex];
            } else {
                tile.neighborsHigh[neighborIndex - 4] = tileNeighbors[neighborIndex];
            }
        }
        const bool pentagon = tileNeighbors.size() == 5;
        result.pentagonCount += pentagon ? 1U : 0U;
        result.hexagonCount += pentagon ? 0U : 1U;
        tile.brickInfo = {tileIndex * cellsPerBrick, brickResolution,
                          static_cast<std::uint32_t>(tileNeighbors.size()), pentagon ? 1U : 0U};
        if (progress && ((tileIndex & 0xffffU) == 0U || tileIndex + 1U == positions.size())) {
            progress("topology generation: dual cells", tileIndex + 1U, positions.size());
        }
    }
    if (bedrockInnerRadius < bedrockOuterRadius) {
        result.bedrockRadius = static_cast<float>(
            std::midpoint(bedrockInnerRadius, bedrockOuterRadius));
    }

    result.columnStates.resize(result.tiles.size());
    result.pageTable.resize(result.tiles.size());
    if (progress) {
        progress("topology generation: column metadata", 0U, result.tiles.size());
    }
    for (std::uint32_t tileIndex = 0; tileIndex < result.tiles.size(); ++tileIndex) {
        const Vec3 center = positions[tileIndex];
        const double terrain = center.y * 0.9 + center.x * center.z * 1.7 +
                               std::sin(center.x * 13.0 + center.z * 9.0) * 0.32;
        const double normalizedHeight = std::clamp(terrain * 0.18 + 0.68, 0.25, 1.0);
        const auto filledLayers = std::max(
            1U, static_cast<std::uint32_t>(std::round(normalizedHeight * brickResolution)));
        const std::uint32_t material = result.tiles[tileIndex].brickInfo[3] != 0U ? 2U : 1U;
        const std::uint32_t occupancyMask = filledLayers >= 32U
                                                ? 0xffffffffU
                                                : (1U << filledLayers) - 1U;
        result.columnStates[tileIndex].data = {occupancyMask, filledLayers, material, 0U};
        if (progress && ((tileIndex & 0x1ffffU) == 0U || tileIndex + 1U == result.tiles.size())) {
            progress("topology generation: column metadata", tileIndex + 1U,
                     result.tiles.size());
        }
    }

    const std::uint32_t physicalPageCount = std::min(
        residentPageCapacity, static_cast<std::uint32_t>(result.tiles.size()));
    result.brickCells.resize(static_cast<std::size_t>(physicalPageCount) * brickResolution, 0U);
    for (std::uint32_t physicalPage = 0; physicalPage < physicalPageCount; ++physicalPage) {
        result.pageTable[physicalPage].data = {
            physicalPage, result.nextPageGeneration++, kPageResident, 0U};
        const auto& state = result.columnStates[physicalPage].data;
        const std::size_t physicalOffset =
            static_cast<std::size_t>(physicalPage) * brickResolution;
        for (std::uint32_t layer = 0; layer < brickResolution; ++layer) {
            result.brickCells[physicalOffset + layer] =
                geodesicMaterialForLayer(state[1], state[2], layer);
        }
    }

    struct TileBounds {
        Vec3 minimum;
        Vec3 maximum;
        Vec3 centroid;
    };
    std::vector<TileBounds> tileBounds;
    if (buildReferenceTraversal) {
    tileBounds.resize(result.tiles.size());
    for (std::uint32_t tileIndex = 0; tileIndex < result.tiles.size(); ++tileIndex) {
        const Vec3 axis = positions[tileIndex];
        const double radialDepth = static_cast<double>(result.tiles[tileIndex].tangent[3]);
        const double innerHeight = 1.0 - radialDepth;
        Vec3 minimum{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max()};
        Vec3 maximum{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                     std::numeric_limits<double>::lowest()};
        const auto includePoint = [&](Vec3 point) {
            minimum.x = std::min(minimum.x, point.x);
            minimum.y = std::min(minimum.y, point.y);
            minimum.z = std::min(minimum.z, point.z);
            maximum.x = std::max(maximum.x, point.x);
            maximum.y = std::max(maximum.y, point.y);
            maximum.z = std::max(maximum.z, point.z);
        };
        const auto& tileNeighbors = neighbors[tileIndex];
        for (std::size_t cornerIndex = 0; cornerIndex < tileNeighbors.size(); ++cornerIndex) {
            const Vec3 firstPlane = axis - positions[tileNeighbors[cornerIndex]];
            const Vec3 secondPlane =
                axis - positions[tileNeighbors[(cornerIndex + 1U) % tileNeighbors.size()]];
            Vec3 cornerDirection = normalized(cross(firstPlane, secondPlane));
            if (dot(cornerDirection, axis) < 0.0) {
                cornerDirection = cornerDirection * -1.0;
            }
            const double alignment = dot(cornerDirection, axis);
            if (alignment <= 0.0) {
                throw std::runtime_error("A geodesic column corner faces away from its tile");
            }
            const Vec3 outerCorner = cornerDirection * (1.0 / alignment);
            includePoint(cornerDirection * (innerHeight / alignment));
            includePoint(outerCorner);
            result.outerBoundingRadius = std::max(
                result.outerBoundingRadius, static_cast<float>(std::sqrt(dot(outerCorner, outerCorner))));
        }
        // Covers the microscopic shared-plane overlap used by the GPU clipper.
        constexpr double padding = 2e-5;
        minimum = minimum - Vec3{padding, padding, padding};
        maximum = maximum + Vec3{padding, padding, padding};
        tileBounds[tileIndex] = {minimum, maximum, (minimum + maximum) * 0.5};
    }
    }

    if (buildReferenceTraversal) {
    std::vector<std::uint32_t> bvhTiles(result.tiles.size());
    std::iota(bvhTiles.begin(), bvhTiles.end(), 0U);
    result.bvhNodes.reserve(result.tiles.size() * 2U - 1U);
    const auto component = [](Vec3 value, std::uint32_t axis) {
        return axis == 0U ? value.x : axis == 1U ? value.y : value.z;
    };
    std::function<std::uint32_t(std::size_t, std::size_t)> buildBvh =
        [&](std::size_t begin, std::size_t end) -> std::uint32_t {
        Vec3 minimum{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max()};
        Vec3 maximum{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                     std::numeric_limits<double>::lowest()};
        Vec3 centroidMinimum = minimum;
        Vec3 centroidMaximum = maximum;
        for (std::size_t index = begin; index < end; ++index) {
            const TileBounds& bounds = tileBounds[bvhTiles[index]];
            minimum.x = std::min(minimum.x, bounds.minimum.x);
            minimum.y = std::min(minimum.y, bounds.minimum.y);
            minimum.z = std::min(minimum.z, bounds.minimum.z);
            maximum.x = std::max(maximum.x, bounds.maximum.x);
            maximum.y = std::max(maximum.y, bounds.maximum.y);
            maximum.z = std::max(maximum.z, bounds.maximum.z);
            centroidMinimum.x = std::min(centroidMinimum.x, bounds.centroid.x);
            centroidMinimum.y = std::min(centroidMinimum.y, bounds.centroid.y);
            centroidMinimum.z = std::min(centroidMinimum.z, bounds.centroid.z);
            centroidMaximum.x = std::max(centroidMaximum.x, bounds.centroid.x);
            centroidMaximum.y = std::max(centroidMaximum.y, bounds.centroid.y);
            centroidMaximum.z = std::max(centroidMaximum.z, bounds.centroid.z);
        }

        const std::uint32_t nodeIndex = static_cast<std::uint32_t>(result.bvhNodes.size());
        result.bvhNodes.emplace_back();
        GeodesicBvhNodeGpu& node = result.bvhNodes[nodeIndex];
        node.minimum = packed(minimum);
        node.maximum = packed(maximum);
        if (end - begin == 1U) {
            node.data = {bvhTiles[begin], 0U, 1U, 0U};
            return nodeIndex;
        }

        const Vec3 extent = centroidMaximum - centroidMinimum;
        const std::uint32_t splitAxis = extent.x >= extent.y && extent.x >= extent.z ? 0U
                                      : extent.y >= extent.z ? 1U : 2U;
        const std::size_t middle = begin + (end - begin) / 2U;
        std::nth_element(bvhTiles.begin() + static_cast<std::ptrdiff_t>(begin),
                         bvhTiles.begin() + static_cast<std::ptrdiff_t>(middle),
                         bvhTiles.begin() + static_cast<std::ptrdiff_t>(end),
                         [&](std::uint32_t left, std::uint32_t right) {
            return component(tileBounds[left].centroid, splitAxis) <
                   component(tileBounds[right].centroid, splitAxis);
        });
        const std::uint32_t left = buildBvh(begin, middle);
        const std::uint32_t right = buildBvh(middle, end);
        result.bvhNodes[nodeIndex].data = {left, right, 0U, 0U};
        return nodeIndex;
    };
    buildBvh(0, bvhTiles.size());

    // Build a compact, stackless wide hierarchy over Morton-ordered geodesic
    // columns.  Four exact prism leaves amortize node traffic, while each
    // internal node may address up to eight spatial children today.  The
    // 64-bit masks leave room for the 16-surface x 4-radial refinement planned
    // for sparse cellular material without changing the GPU ABI.
    constexpr std::size_t kWideLeafItems = 4U;
    constexpr std::size_t kWideFanout = 8U;
    result.wideItems.resize(result.tiles.size());
    std::iota(result.wideItems.begin(), result.wideItems.end(), 0U);
    std::ranges::sort(result.wideItems, [&](std::uint32_t left, std::uint32_t right) {
        const std::uint32_t leftCode = mortonCode(positions[left]);
        const std::uint32_t rightCode = mortonCode(positions[right]);
        return leftCode != rightCode ? leftCode < rightCode : left < right;
    });
    result.wideNodes.reserve(result.tiles.size() / 3U + 16U);
    std::function<std::uint32_t(std::size_t, std::size_t)> buildWide =
        [&](std::size_t begin, std::size_t end) -> std::uint32_t {
        Vec3 minimum{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                     std::numeric_limits<double>::max()};
        Vec3 maximum{std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                     std::numeric_limits<double>::lowest()};
        for (std::size_t index = begin; index < end; ++index) {
            const TileBounds& bounds = tileBounds[result.wideItems[index]];
            minimum.x = std::min(minimum.x, bounds.minimum.x);
            minimum.y = std::min(minimum.y, bounds.minimum.y);
            minimum.z = std::min(minimum.z, bounds.minimum.z);
            maximum.x = std::max(maximum.x, bounds.maximum.x);
            maximum.y = std::max(maximum.y, bounds.maximum.y);
            maximum.z = std::max(maximum.z, bounds.maximum.z);
        }

        const std::uint32_t nodeIndex = static_cast<std::uint32_t>(result.wideNodes.size());
        result.wideNodes.emplace_back();
        result.wideNodes[nodeIndex].minimum = packed(minimum);
        result.wideNodes[nodeIndex].maximum = packed(maximum);
        const std::size_t itemCount = end - begin;
        if (itemCount <= kWideLeafItems) {
            const auto count = static_cast<std::uint32_t>(itemCount);
            result.wideNodes[nodeIndex].data = {
                static_cast<std::uint32_t>(begin), count, nodeIndex + 1U, 1U};
            result.wideNodes[nodeIndex].mask = activeMask(count);
            return nodeIndex;
        }

        const std::size_t childCount = std::min(kWideFanout,
            (itemCount + kWideLeafItems - 1U) / kWideLeafItems);
        const std::uint32_t firstChild = nodeIndex + 1U;
        for (std::size_t child = 0; child < childCount; ++child) {
            const std::size_t childBegin = begin + itemCount * child / childCount;
            const std::size_t childEnd = begin + itemCount * (child + 1U) / childCount;
            (void)buildWide(childBegin, childEnd);
        }
        const auto childCount32 = static_cast<std::uint32_t>(childCount);
        result.wideNodes[nodeIndex].data = {
            firstChild, childCount32,
            static_cast<std::uint32_t>(result.wideNodes.size()), 0U};
        result.wideNodes[nodeIndex].mask = activeMask(childCount32);
        return nodeIndex;
    };
    buildWide(0U, result.wideItems.size());
    }

    result.directionLookup.resize(static_cast<std::size_t>(lookupWidth) * lookupHeight);
    if (progress) {
        progress("topology generation: direction atlas", 0U, lookupHeight);
    }
    for (std::uint32_t y = 0; y < lookupHeight; ++y) {
        const double latitude = (0.5 - (static_cast<double>(y) + 0.5) / lookupHeight) * std::numbers::pi;
        const double latitudeCosine = std::cos(latitude);
        for (std::uint32_t x = 0; x < lookupWidth; ++x) {
            const double longitude = ((static_cast<double>(x) + 0.5) / lookupWidth * 2.0 - 1.0) *
                                     std::numbers::pi;
            const Vec3 direction{std::cos(longitude) * latitudeCosine,
                                 std::sin(latitude),
                                 std::sin(longitude) * latitudeCosine};
            std::uint32_t bestTile = x > 0U
                                         ? result.directionLookup[static_cast<std::size_t>(x - 1U) +
                                                                  static_cast<std::size_t>(lookupWidth) * y]
                                         : y > 0U
                                               ? result.directionLookup[static_cast<std::size_t>(lookupWidth) *
                                                                        (y - 1U)]
                                               : 0U;
            // Adjacent lookup samples almost always share a tile or one of its
            // neighbors. Walking the convex topology changes this build from
            // O(lookupPixels * tileCount) to approximately O(lookupPixels).
            const std::uint32_t maximumWalk = frequency * 4U + 16U;
            for (std::uint32_t walk = 0; walk < maximumWalk; ++walk) {
                double bestDot = dot(direction, positions[bestTile]);
                std::uint32_t nextTile = bestTile;
                for (const std::uint32_t neighbor : neighbors[bestTile]) {
                    const double candidate = dot(direction, positions[neighbor]);
                    if (candidate > bestDot + 1e-14) {
                        bestDot = candidate;
                        nextTile = neighbor;
                    }
                }
                if (nextTile == bestTile) {
                    break;
                }
                bestTile = nextTile;
            }
            result.directionLookup[static_cast<std::size_t>(x) +
                                   static_cast<std::size_t>(lookupWidth) * y] = bestTile;
        }
        if (progress && ((y & 15U) == 0U || y + 1U == lookupHeight)) {
            progress("topology generation: direction atlas", y + 1U, lookupHeight);
        }
    }

    result.validate();
    if (progress) {
        progress("topology generation: validated", 1U, 1U);
    }
    return result;
}

void GeodesicTopology::validate() const {
    const std::uint64_t expectedTiles = 10ULL * frequency * frequency + 2ULL;
    const std::uint64_t expectedTriangles = 20ULL * frequency * frequency;
    if (tiles.size() != expectedTiles || triangleCount != expectedTriangles) {
        throw std::runtime_error("Geodesic topology violates the icosphere count invariant");
    }
    if (pentagonCount != 12 || hexagonCount + pentagonCount != tiles.size()) {
        throw std::runtime_error("Geodesic dual must contain 12 pentagons and otherwise hexagons");
    }
    if (brickResolution == 0U || brickCells.size() % brickResolution != 0U ||
        residentPageCount() > tiles.size()) {
        throw std::runtime_error("Sparse geodesic physical page pool has an invalid size");
    }
    if (referenceTraversalBuilt) {
        if (bvhNodes.empty() || bvhNodes.size() != tiles.size() * 2U - 1U) {
            throw std::runtime_error("Geodesic column BVH has an invalid node count");
        }
        if (wideNodes.empty() || wideItems.size() != tiles.size() ||
            wideNodes.front().data[2] != wideNodes.size()) {
            throw std::runtime_error("Geodesic wide hierarchy has an invalid root or item count");
        }
    } else if (!bvhNodes.empty() || !wideNodes.empty() || !wideItems.empty()) {
        throw std::runtime_error("Production geodesic topology retained reference traversal data");
    }
    if (columnStates.size() != tiles.size() || pageTable.size() != tiles.size()) {
        throw std::runtime_error("Geodesic column-state or page table has an invalid size");
    }
    std::vector<bool> physicalPageOwned(residentPageCount(), false);
    for (std::uint32_t tileIndex = 0; tileIndex < tiles.size(); ++tileIndex) {
        const GeodesicTileGpu& tile = tiles[tileIndex];
        const std::uint32_t count = tile.brickInfo[2];
        if (count != 5 && count != 6) {
            throw std::runtime_error("A geodesic tile has an invalid neighbor count");
        }
        if (tile.brickInfo[0] != tileIndex * brickResolution ||
            tile.brickInfo[1] != brickResolution ||
            static_cast<std::uint64_t>(tile.brickInfo[0]) + brickResolution > brickCellCapacity()) {
            throw std::runtime_error("A geodesic tile has an invalid virtual radial-brick address");
        }
        const GeodesicColumnStateGpu& columnState = columnStates[tileIndex];
        if (columnState.data[1] == 0U || columnState.data[1] > brickResolution) {
            throw std::runtime_error("A geodesic column has an invalid occupied-layer count");
        }
        const auto& page = pageTable[tileIndex].data;
        const bool resident = (page[2] & kPageResident) != 0U;
        if (!resident) {
            if (page[0] != kInvalidPage) {
                throw std::runtime_error("A nonresident geodesic page has a physical address");
            }
        } else {
            if (page[0] >= residentPageCount() || physicalPageOwned[page[0]]) {
                throw std::runtime_error("A sparse physical page has an invalid or duplicate owner");
            }
            physicalPageOwned[page[0]] = true;
            const std::size_t physicalOffset =
                static_cast<std::size_t>(page[0]) * brickResolution;
            for (std::uint32_t layer = 0; layer < brickResolution; ++layer) {
                const std::uint32_t expectedMaterial = geodesicMaterialForLayer(
                    columnState.data[1], columnState.data[2], layer);
                if (brickCells[physicalOffset + layer] != expectedMaterial) {
                    throw std::runtime_error(
                        "Geodesic column metadata disagrees with resident page material");
                }
            }
        }
        for (std::uint32_t neighborSlot = 0; neighborSlot < count; ++neighborSlot) {
            const std::uint32_t neighbor = neighborSlot < 4
                                               ? tile.neighborsLow[neighborSlot]
                                               : tile.neighborsHigh[neighborSlot - 4];
            if (neighbor >= tiles.size()) {
                throw std::runtime_error("A geodesic neighbor index is out of range");
            }
            const GeodesicTileGpu& reverseTile = tiles[neighbor];
            bool reciprocal = false;
            for (std::uint32_t reverseSlot = 0; reverseSlot < reverseTile.brickInfo[2]; ++reverseSlot) {
                const std::uint32_t reverse = reverseSlot < 4
                                                  ? reverseTile.neighborsLow[reverseSlot]
                                                  : reverseTile.neighborsHigh[reverseSlot - 4];
                reciprocal = reciprocal || reverse == tileIndex;
            }
            if (!reciprocal) {
                throw std::runtime_error("Geodesic adjacency is not reciprocal");
            }
        }
    }
    if (!std::ranges::all_of(physicalPageOwned, [](bool owned) { return owned; })) {
        throw std::runtime_error("A sparse geodesic physical page is unowned");
    }

    if (referenceTraversalBuilt) {
    std::vector<bool> wideTileSeen(tiles.size(), false);
    for (std::uint32_t nodeIndex = 0; nodeIndex < wideNodes.size(); ++nodeIndex) {
        const GeodesicWideNodeGpu& node = wideNodes[nodeIndex];
        const std::uint32_t count = node.data[1];
        const std::uint32_t escape = node.data[2];
        if (count == 0U || count > 64U || escape <= nodeIndex || escape > wideNodes.size()) {
            throw std::runtime_error("A geodesic wide node has an invalid count or escape link");
        }
        const std::uint64_t expectedMask = count == 64U ? ~0ULL : (1ULL << count) - 1ULL;
        const std::uint64_t actualMask = static_cast<std::uint64_t>(node.mask[0]) |
                                         (static_cast<std::uint64_t>(node.mask[1]) << 32U);
        if (actualMask != expectedMask) {
            throw std::runtime_error("A geodesic wide node has an invalid active mask");
        }
        if (node.data[3] != 0U) {
            if (static_cast<std::uint64_t>(node.data[0]) + count > wideItems.size()) {
                throw std::runtime_error("A geodesic wide leaf item range is invalid");
            }
            for (std::uint32_t item = 0; item < count; ++item) {
                const std::uint32_t tile = wideItems[node.data[0] + item];
                if (tile >= tiles.size() || wideTileSeen[tile]) {
                    throw std::runtime_error("A geodesic tile is duplicated or invalid in the wide hierarchy");
                }
                wideTileSeen[tile] = true;
            }
        } else {
            if (node.data[0] != nodeIndex + 1U || count > 8U) {
                throw std::runtime_error("A geodesic wide internal node has invalid children");
            }
            std::uint32_t child = node.data[0];
            for (std::uint32_t slot = 0; slot < count; ++slot) {
                if (child >= escape) {
                    throw std::runtime_error("A geodesic wide child escaped its parent");
                }
                child = wideNodes[child].data[2];
            }
            if (child != escape) {
                throw std::runtime_error("Geodesic wide child escape links do not cover the parent");
            }
        }
    }
    if (!std::ranges::all_of(wideTileSeen, [](bool seen) { return seen; })) {
        throw std::runtime_error("A geodesic tile is missing from the wide hierarchy");
    }
    }
}

} // namespace voxel
