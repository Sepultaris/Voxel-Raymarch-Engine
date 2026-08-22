#include "planet/geodesic_topology.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

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
    if (length <= 1e-15) {
        throw std::runtime_error("zero test vector");
    }
    return value * (1.0 / length);
}
Vec3 unpack(const std::array<float, 4>& value) {
    return {value[0], value[1], value[2]};
}
double distance(Vec3 a, Vec3 b) {
    const Vec3 delta = a - b;
    return std::sqrt(dot(delta, delta));
}
void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::uint32_t neighbor(const voxel::GeodesicTileGpu& tile, std::uint32_t slot) {
    return slot < 4U ? tile.neighborsLow[slot] : tile.neighborsHigh[slot - 4U];
}

std::vector<Vec3> outerCorners(const voxel::GeodesicTopology& topology, std::uint32_t tileIndex) {
    const auto& tile = topology.tiles[tileIndex];
    const Vec3 center = unpack(tile.center);
    std::vector<Vec3> corners;
    corners.reserve(tile.brickInfo[2]);
    for (std::uint32_t slot = 0; slot < tile.brickInfo[2]; ++slot) {
        const Vec3 first = center - unpack(topology.tiles[neighbor(tile, slot)].center);
        const Vec3 second = center - unpack(
            topology.tiles[neighbor(tile, (slot + 1U) % tile.brickInfo[2])].center);
        Vec3 direction = normalized(cross(first, second));
        if (dot(direction, center) < 0.0) {
            direction = direction * -1.0;
        }
        corners.push_back(direction * (1.0 / dot(direction, center)));
    }
    return corners;
}

bool clipPlane(Vec3 normal, double planeDistance, Vec3 origin, Vec3 direction,
               double& nearDistance, double& farDistance,
               Vec3* nearNormal = nullptr) {
    const double originDistance = dot(normal, origin) - planeDistance;
    const double denominator = dot(normal, direction);
    if (denominator == 0.0) {
        return originDistance <= 0.0;
    }
    const double nearSide = std::fma(denominator, nearDistance, originDistance);
    const double farSide = std::fma(denominator, farDistance, originDistance);
    if (nearSide <= 0.0 && farSide <= 0.0) {
        return true;
    }
    if (nearSide > 0.0 && farSide > 0.0) {
        return false;
    }
    const double hit = -originDistance / denominator;
    if (!std::isfinite(hit)) {
        return false;
    }
    const double clampedHit = std::clamp(hit, nearDistance, farDistance);
    if (nearSide > 0.0) {
        if (clampedHit > nearDistance && nearNormal != nullptr) {
            *nearNormal = normal;
        }
        nearDistance = clampedHit;
    } else {
        farDistance = clampedHit;
    }
    return nearDistance <= std::nextafter(
        farDistance, std::numeric_limits<double>::infinity());
}

bool intersectColumn(const voxel::GeodesicTopology& topology, std::uint32_t tileIndex,
                     Vec3 origin, Vec3 direction, double& hitDistance,
                     Vec3* hitNormal = nullptr) {
    const auto& tile = topology.tiles[tileIndex];
    const auto& state = topology.columnStates[tileIndex].data;
    if (state[1] == 0U) {
        return false;
    }
    const Vec3 center = unpack(tile.center);
    const double totalDepth = tile.tangent[3];
    const double layerThickness = totalDepth / tile.brickInfo[1];
    const double innerHeight = 1.0 - totalDepth;
    const double outerHeight = innerHeight + state[1] * layerThickness;
    double nearDistance = 0.0;
    double farDistance = 100.0;
    Vec3 nearNormal{};
    if (!clipPlane(center, outerHeight, origin, direction,
                   nearDistance, farDistance, &nearNormal) ||
        !clipPlane(center * -1.0, -innerHeight, origin, direction,
                   nearDistance, farDistance, &nearNormal)) {
        return false;
    }
    for (std::uint32_t slot = 0; slot < tile.brickInfo[2]; ++slot) {
        const Vec3 neighborCenter = unpack(topology.tiles[neighbor(tile, slot)].center);
        const Vec3 inward = center - neighborCenter;
        const double inwardLength = std::sqrt(dot(inward, inward));
        if (!clipPlane(inward * -1.0, 2e-6 * inwardLength, origin, direction,
                       nearDistance, farDistance, &nearNormal)) {
            return false;
        }
    }
    hitDistance = nearDistance;
    if (hitNormal != nullptr) {
        *hitNormal = nearNormal;
    }
    return hitDistance >= 0.0 && hitDistance < 100.0;
}

double aabbNear(const voxel::GeodesicBvhNodeGpu& node, Vec3 origin, Vec3 direction,
                double maximumDistance) {
    double nearDistance = 0.0;
    double farDistance = maximumDistance;
    for (std::uint32_t axis = 0; axis < 3U; ++axis) {
        const double component = axis == 0U ? direction.x : axis == 1U ? direction.y : direction.z;
        const double originComponent = axis == 0U ? origin.x : axis == 1U ? origin.y : origin.z;
        const double minimum = node.minimum[axis];
        const double maximum = node.maximum[axis];
        if (std::abs(component) < 1e-12) {
            if (originComponent < minimum || originComponent > maximum) {
                return 100.0;
            }
            continue;
        }
        double first = (minimum - originComponent) / component;
        double second = (maximum - originComponent) / component;
        if (first > second) {
            std::swap(first, second);
        }
        nearDistance = std::max(nearDistance, first);
        farDistance = std::min(farDistance, second);
        if (farDistance < nearDistance) {
            return 100.0;
        }
    }
    return nearDistance <= maximumDistance ? nearDistance : 100.0;
}

double aabbNear(const voxel::GeodesicWideNodeGpu& node, Vec3 origin, Vec3 direction,
                double maximumDistance) {
    voxel::GeodesicBvhNodeGpu compatible{};
    compatible.minimum = node.minimum;
    compatible.maximum = node.maximum;
    return aabbNear(compatible, origin, direction, maximumDistance);
}

double exhaustiveHit(const voxel::GeodesicTopology& topology, Vec3 origin,
                     Vec3 direction, Vec3* hitNormal = nullptr) {
    double closest = 100.0;
    Vec3 closestNormal{};
    for (std::uint32_t tile = 0; tile < topology.tiles.size(); ++tile) {
        double candidate = 0.0;
        Vec3 candidateNormal{};
        if (intersectColumn(topology, tile, origin, direction, candidate,
                            &candidateNormal) && candidate < closest) {
            closest = candidate;
            closestNormal = candidateNormal;
        }
    }
    if (hitNormal != nullptr) {
        *hitNormal = closestNormal;
    }
    return closest;
}

double acceleratedHit(const voxel::GeodesicTopology& topology, Vec3 origin, Vec3 direction) {
    double closest = 100.0;
    std::vector<std::uint32_t> stack{0U};
    while (!stack.empty()) {
        const std::uint32_t nodeIndex = stack.back();
        stack.pop_back();
        const auto& node = topology.bvhNodes[nodeIndex];
        if (aabbNear(node, origin, direction, closest) >= 100.0) {
            continue;
        }
        if (node.data[2] != 0U) {
            double candidate = 0.0;
            if (intersectColumn(topology, node.data[0], origin, direction, candidate)) {
                closest = std::min(closest, candidate);
            }
        } else {
            stack.push_back(node.data[0]);
            stack.push_back(node.data[1]);
        }
    }
    return closest;
}

double wideAcceleratedHit(const voxel::GeodesicTopology& topology,
                          Vec3 origin, Vec3 direction) {
    double closest = 100.0;
    std::uint32_t nodeIndex = 0U;
    while (nodeIndex < topology.wideNodes.size()) {
        const auto& node = topology.wideNodes[nodeIndex];
        if (aabbNear(node, origin, direction, closest) >= 100.0) {
            nodeIndex = node.data[2];
            continue;
        }
        if (node.data[3] != 0U) {
            for (std::uint32_t item = 0; item < node.data[1]; ++item) {
                double candidate = 0.0;
                const std::uint32_t tile = topology.wideItems[node.data[0] + item];
                if (intersectColumn(topology, tile, origin, direction, candidate)) {
                    closest = std::min(closest, candidate);
                }
            }
            ++nodeIndex;
        } else {
            nodeIndex = node.data[0];
        }
    }
    return closest;
}

std::uint32_t refineTile(const voxel::GeodesicTopology& topology,
                         std::uint32_t tile, Vec3 direction,
                         std::uint32_t maximumWalk = 32U) {
    direction = normalized(direction);
    for (std::uint32_t walk = 0; walk < maximumWalk; ++walk) {
        double bestAlignment = dot(direction, unpack(topology.tiles[tile].center));
        std::uint32_t bestTile = tile;
        for (std::uint32_t slot = 0; slot < topology.tiles[tile].brickInfo[2]; ++slot) {
            const std::uint32_t candidate = neighbor(topology.tiles[tile], slot);
            const double alignment = dot(direction, unpack(topology.tiles[candidate].center));
            if (alignment > bestAlignment) {
                bestAlignment = alignment;
                bestTile = candidate;
            }
        }
        if (bestTile == tile) {
            break;
        }
        tile = bestTile;
    }
    return tile;
}

std::uint32_t locateTile(const voxel::GeodesicTopology& topology, Vec3 direction,
                         std::uint32_t maximumWalk = 32U) {
    direction = normalized(direction);
    const double longitude = std::atan2(direction.z, direction.x);
    const double latitude = std::asin(std::clamp(direction.y, -1.0, 1.0));
    const double u = longitude / (2.0 * 3.14159265358979323846) + 0.5;
    const double v = 0.5 - latitude / 3.14159265358979323846;
    const std::uint32_t x = std::min(
        static_cast<std::uint32_t>(u * topology.lookupWidth), topology.lookupWidth - 1U);
    const std::uint32_t y = std::min(
        static_cast<std::uint32_t>(v * topology.lookupHeight), topology.lookupHeight - 1U);
    return refineTile(topology,
                      topology.directionLookup[x + topology.lookupWidth * y],
                      direction, maximumWalk);
}

bool raySphereInterval(Vec3 origin, Vec3 direction, double radius,
                       double& nearDistance, double& farDistance) {
    const double projection = dot(origin, direction);
    const double discriminant = projection * projection - dot(origin, origin) + radius * radius;
    if (discriminant < 0.0) {
        return false;
    }
    const double root = std::sqrt(discriminant);
    nearDistance = std::max(-projection - root, 0.0);
    farDistance = -projection + root;
    return farDistance >= nearDistance;
}

struct DdaEventTrace {
    std::uint32_t step{};
    std::uint32_t tile{voxel::kInvalidTile};
    std::uint32_t tiedCount{};
    std::uint32_t nextTile{voxel::kInvalidTile};
    double travel{};
    double eventDistance{};
    std::array<std::uint32_t, 6> tiedOwners{};
};

double geodesicDdaHit(const voxel::GeodesicTopology& topology,
                      Vec3 origin, Vec3 direction,
                      std::uint32_t* hitTile = nullptr,
                      std::uint32_t* stepsUsed = nullptr,
                      bool recoverParallelBoundary = true,
                      bool robustBoundaryRecovery = false,
                      std::uint32_t* terminationReason = nullptr,
                      DdaEventTrace* eventTrace = nullptr) {
    (void)recoverParallelBoundary;
    if (terminationReason != nullptr) {
        *terminationReason = 0U;
    }
    double travel = 0.0;
    double farDistance = 0.0;
    if (!raySphereInterval(origin, direction, topology.outerBoundingRadius,
                           travel, farDistance)) {
        return 100.0;
    }
    const auto tiedByUlps = [](double first, double second,
                               std::uint32_t maximumUlps = 4U) {
        if (first == second) {
            return true;
        }
        double lower = std::min(first, second);
        const double upper = std::max(first, second);
        for (std::uint32_t ulp = 0U; ulp < maximumUlps; ++ulp) {
            lower = std::nextafter(lower, std::numeric_limits<double>::infinity());
            if (lower >= upper) {
                return true;
            }
        }
        return false;
    };
    const auto ownsAt = [&](std::uint32_t candidate, double eventTravel) {
        const Vec3 candidateCenter = unpack(topology.tiles[candidate].center);
        for (std::uint32_t slot = 0U;
             slot < topology.tiles[candidate].brickInfo[2]; ++slot) {
            const std::uint32_t adjacent = neighbor(topology.tiles[candidate], slot);
            const Vec3 plane = candidateCenter -
                unpack(topology.tiles[adjacent].center);
            const double signedDistance = dot(plane, origin) +
                dot(plane, direction) * eventTravel;
            if (signedDistance < 0.0 ||
                (signedDistance == 0.0 && candidate > adjacent)) {
                return false;
            }
        }
        return true;
    };
    const auto ownsAfterEvent = [&](std::uint32_t candidate,
                                    double eventTravel) {
        const Vec3 candidateCenter = unpack(topology.tiles[candidate].center);
        double probeTravel = eventTravel;
        for (std::uint32_t ulp = 0U; ulp < 4U; ++ulp) {
            probeTravel = std::nextafter(
                probeTravel, std::numeric_limits<double>::infinity());
        }
        for (std::uint32_t slot = 0U;
             slot < topology.tiles[candidate].brickInfo[2]; ++slot) {
            const std::uint32_t adjacent = neighbor(topology.tiles[candidate], slot);
            const Vec3 plane = candidateCenter -
                unpack(topology.tiles[adjacent].center);
            const double originDistance = dot(plane, origin);
            const double derivative = dot(plane, direction);
            const double signedDistance = std::fma(
                derivative, probeTravel, originDistance);
            if (signedDistance < 0.0 ||
                (signedDistance == 0.0 &&
                 (derivative < 0.0 ||
                  (derivative == 0.0 && candidate > adjacent)))) {
                return false;
            }
        }
        return true;
    };
    const double seededTravel = std::nextafter(
        travel, std::numeric_limits<double>::infinity());
    std::uint32_t tile = locateTile(topology, origin + direction * seededTravel);
    {
        std::uint32_t canonical = ownsAt(tile, seededTravel)
            ? tile : voxel::kInvalidTile;
        for (std::uint32_t slot = 0U;
             slot < topology.tiles[tile].brickInfo[2]; ++slot) {
            const std::uint32_t candidate = neighbor(topology.tiles[tile], slot);
            if (ownsAt(candidate, seededTravel) &&
                (canonical == voxel::kInvalidTile || candidate < canonical)) {
                canonical = candidate;
            }
        }
        if (canonical != voxel::kInvalidTile) {
            tile = canonical;
        }
    }
    double closest = 100.0;
    double previousEventDistance = travel;
    bool hasPreviousEvent = false;
    const std::uint32_t maximumSteps = robustBoundaryRecovery ? 4'096U : 2'048U;
    for (std::uint32_t step = 0; step < maximumSteps && travel <= farDistance; ++step) {
        if (eventTrace != nullptr) {
            eventTrace->step = step;
            eventTrace->tile = tile;
            eventTrace->travel = travel;
            eventTrace->eventDistance = farDistance;
            eventTrace->tiedCount = 0U;
            eventTrace->nextTile = voxel::kInvalidTile;
        }
        if (stepsUsed != nullptr) {
            *stepsUsed = step + 1U;
        }
        const auto consider = [&](std::uint32_t candidateTile) {
            double candidate = 0.0;
            if (intersectColumn(topology, candidateTile, origin, direction, candidate) &&
                (candidate >= travel || tiedByUlps(candidate, travel, 8U)) &&
                candidate < closest) {
                closest = candidate;
                if (hitTile != nullptr) {
                    *hitTile = candidateTile;
                }
            }
        };
        consider(tile);

        const Vec3 center = unpack(topology.tiles[tile].center);
        double eventDistance = farDistance;
        bool hasSideEvent = false;
        for (std::uint32_t slot = 0; slot < topology.tiles[tile].brickInfo[2]; ++slot) {
            const std::uint32_t adjacent = neighbor(topology.tiles[tile], slot);
            const Vec3 inward = center - unpack(topology.tiles[adjacent].center);
            const double denominator = dot(inward, direction);
            const double originPlaneDistance = dot(inward, origin);
            const double intervalPlaneDistance = originPlaneDistance +
                denominator * travel;
            if (intervalPlaneDistance * intervalPlaneDistance <=
                4e-12 * dot(inward, inward)) {
                // This is the exact expanded prism overlap, not a traversal
                // epsilon. Test every geometric owner that contains the
                // half-open event point.
                consider(adjacent);
            }
            if (denominator == 0.0) {
                continue;
            }
            if (denominator > 0.0) {
                continue;
            }
            const double crossing = -originPlaneDistance / denominator;
            if (!std::isfinite(crossing) || crossing <= travel ||
                crossing > farDistance) {
                continue;
            }
            if (!hasSideEvent || crossing < eventDistance) {
                eventDistance = crossing;
                hasSideEvent = true;
            }
        }
        if (!hasSideEvent) {
            if (closest >= 100.0 && hasPreviousEvent) {
                std::uint32_t corrected = ownsAfterEvent(
                    tile, previousEventDistance) ? tile : voxel::kInvalidTile;
                for (std::uint32_t slot = 0U;
                     slot < topology.tiles[tile].brickInfo[2]; ++slot) {
                    const std::uint32_t candidate = neighbor(
                        topology.tiles[tile], slot);
                    if (ownsAfterEvent(candidate, previousEventDistance) &&
                        (corrected == voxel::kInvalidTile || candidate < corrected)) {
                        corrected = candidate;
                    }
                }
                hasPreviousEvent = false;
                if (corrected != voxel::kInvalidTile && corrected != tile) {
                    tile = corrected;
                    continue;
                }
            }
            if (closest < 100.0 && terminationReason != nullptr) {
                *terminationReason = 1U;
            } else if (terminationReason != nullptr) {
                *terminationReason = 2U;
            }
            return closest;
        }
        if (eventTrace != nullptr) {
            eventTrace->eventDistance = eventDistance;
        }

        std::array<std::uint32_t, 6> tiedOwners{};
        std::uint32_t tiedCount = 0U;
        for (std::uint32_t slot = 0U;
             slot < topology.tiles[tile].brickInfo[2]; ++slot) {
            const std::uint32_t adjacent = neighbor(topology.tiles[tile], slot);
            const Vec3 inward = center - unpack(topology.tiles[adjacent].center);
            const double denominator = dot(inward, direction);
            if (denominator >= 0.0) {
                continue;
            }
            const double crossing = -dot(inward, origin) / denominator;
            if (std::isfinite(crossing) &&
                tiedByUlps(crossing, eventDistance)) {
                tiedOwners[tiedCount++] = adjacent;
                consider(adjacent);
            }
        }
        if (eventTrace != nullptr) {
            eventTrace->tiedCount = tiedCount;
            eventTrace->tiedOwners = tiedOwners;
        }
        if (closest < eventDistance || tiedByUlps(closest, eventDistance, 8U)) {
            if (terminationReason != nullptr) {
                *terminationReason = 1U;
            }
            return closest;
        }
        if (tiedCount == 0U) {
            if (terminationReason != nullptr) {
                *terminationReason = 2U;
            }
            return closest;
        }

        const double nextTravel = std::nextafter(
            eventDistance, std::numeric_limits<double>::infinity());
        std::uint32_t nextTile = voxel::kInvalidTile;
        const auto selectOwner = [&](std::uint32_t candidate) {
            if (ownsAfterEvent(candidate, eventDistance) &&
                (nextTile == voxel::kInvalidTile || candidate < nextTile)) {
                nextTile = candidate;
            }
        };
        for (std::uint32_t index = 0U; index < tiedCount; ++index) {
            selectOwner(tiedOwners[index]);
        }
        if (nextTile == voxel::kInvalidTile) {
            // A rejected entered owner is itself an ambiguous event signal.
            // Expand only this event's exact local topology ring.
            for (std::uint32_t index = 0U; index < tiedCount; ++index) {
                const std::uint32_t eventTile = tiedOwners[index];
                for (std::uint32_t slot = 0U;
                     slot < topology.tiles[eventTile].brickInfo[2]; ++slot) {
                    const std::uint32_t candidate = neighbor(
                        topology.tiles[eventTile], slot);
                    if (ownsAfterEvent(candidate, eventDistance) &&
                        (nextTile == voxel::kInvalidTile || candidate < nextTile)) {
                        nextTile = candidate;
                    }
                }
            }
        }
        if (nextTile == voxel::kInvalidTile) {
            nextTile = *std::min_element(
                tiedOwners.begin(), tiedOwners.begin() + tiedCount);
        }
        if (eventTrace != nullptr) {
            eventTrace->nextTile = nextTile;
        }
        travel = nextTravel;
        previousEventDistance = eventDistance;
        hasPreviousEvent = true;
        tile = nextTile;
    }
    if (terminationReason != nullptr) {
        *terminationReason = travel > farDistance ? 3U : 4U;
    }
    return closest;
}

std::vector<Vec3> ulpJitteredDirections(Vec3 direction) {
    std::vector<Vec3> variants{direction};
    for (std::uint32_t axis = 0U; axis < 3U; ++axis) {
        for (double destination : {
                 -std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()}) {
            Vec3 jittered = direction;
            double* component = axis == 0U ? &jittered.x
                              : axis == 1U ? &jittered.y : &jittered.z;
            for (std::uint32_t ulp = 0U; ulp < 4U; ++ulp) {
                *component = std::nextafter(*component, destination);
                variants.push_back(normalized(jittered));
            }
        }
    }
    return variants;
}

void requireDdaMatchesExhaustive(const voxel::GeodesicTopology& topology,
                                 Vec3 origin, Vec3 direction,
                                 const char* scenario) {
    const double expected = exhaustiveHit(topology, origin, direction);
    std::uint32_t hitTile = voxel::kInvalidTile;
    std::uint32_t steps = 0U;
    std::uint32_t termination = 0U;
    DdaEventTrace trace{};
    const double actual = geodesicDdaHit(
        topology, origin, direction, &hitTile, &steps, true, true,
        &termination, &trace);
    const bool sameCoverage = (expected < 100.0) == (actual < 100.0);
    const bool sameDistance = expected >= 100.0 ||
        std::abs(expected - actual) < 2e-5;
    if (sameCoverage && sameDistance) {
        return;
    }

    std::ostringstream message;
    message.precision(17);
    message << scenario << " differential divergence: origin=("
            << origin.x << ',' << origin.y << ',' << origin.z
            << ") ray=(" << direction.x << ',' << direction.y << ','
            << direction.z << ") expected=" << expected
            << " actual=" << actual << " hitTile=" << hitTile
            << " steps=" << steps << " termination=" << termination
            << " lastEvent={step=" << trace.step << ",tile=" << trace.tile
            << ",travel=" << trace.travel << ",distance="
            << trace.eventDistance << ",tied=" << trace.tiedCount
            << ",owners=[";
    for (std::uint32_t index = 0U; index < trace.tiedCount; ++index) {
        message << (index == 0U ? "" : ",") << trace.tiedOwners[index];
    }
    message << "],next=" << trace.nextTile << "}";
    throw std::runtime_error(message.str());
}

void testSharedEdgesAndLeafBounds(const voxel::GeodesicTopology& topology) {
    require(topology.bedrockRadius > 0.0F,
            "Test topology has no guaranteed layer-zero shell");
    std::vector<std::uint32_t> leafByTile(topology.tiles.size(), voxel::kInvalidTile);
    for (std::uint32_t nodeIndex = 0; nodeIndex < topology.bvhNodes.size(); ++nodeIndex) {
        const auto& node = topology.bvhNodes[nodeIndex];
        if (node.data[2] != 0U) {
            require(node.data[0] < topology.tiles.size(), "BVH leaf tile is out of range");
            require(leafByTile[node.data[0]] == voxel::kInvalidTile, "duplicate BVH tile leaf");
            leafByTile[node.data[0]] = nodeIndex;
        } else {
            require(node.data[0] < topology.bvhNodes.size() &&
                    node.data[1] < topology.bvhNodes.size(), "BVH child is out of range");
        }
    }

    for (std::uint32_t tileIndex = 0; tileIndex < topology.tiles.size(); ++tileIndex) {
        require(leafByTile[tileIndex] != voxel::kInvalidTile, "tile is missing a BVH leaf");
        const auto corners = outerCorners(topology, tileIndex);
        const auto& leaf = topology.bvhNodes[leafByTile[tileIndex]];
        const double innerScale = 1.0 - topology.tiles[tileIndex].tangent[3];
        const double firstLayerOuter = innerScale +
            topology.tiles[tileIndex].tangent[3] /
                static_cast<double>(topology.tiles[tileIndex].brickInfo[1]);
        const Vec3 tileCenter = unpack(topology.tiles[tileIndex].center);
        for (const Vec3 outer : corners) {
            const Vec3 bedrockPoint = normalized(outer) * topology.bedrockRadius;
            const double bedrockProjection = dot(tileCenter, bedrockPoint);
            require(bedrockProjection >= innerScale - 1e-6 &&
                    bedrockProjection <= firstLayerOuter + 1e-6,
                    "Guaranteed bedrock radius leaves a layer-zero prism");
            const Vec3 inner = outer * innerScale;
            for (const Vec3 point : {outer, inner}) {
                require(point.x >= leaf.minimum[0] - 1e-5 && point.x <= leaf.maximum[0] + 1e-5 &&
                        point.y >= leaf.minimum[1] - 1e-5 && point.y <= leaf.maximum[1] + 1e-5 &&
                        point.z >= leaf.minimum[2] - 1e-5 && point.z <= leaf.maximum[2] + 1e-5,
                        "BVH leaf does not contain a prism vertex");
            }
        }

        const auto& tile = topology.tiles[tileIndex];
        for (std::uint32_t slot = 0; slot < tile.brickInfo[2]; ++slot) {
            const std::uint32_t adjacent = neighbor(tile, slot);
            if (adjacent <= tileIndex) {
                continue;
            }
            const auto adjacentCorners = outerCorners(topology, adjacent);
            std::uint32_t sharedCorners = 0;
            for (const Vec3 first : corners) {
                for (const Vec3 second : adjacentCorners) {
                    sharedCorners += distance(first, second) < 2e-4 ? 1U : 0U;
                }
            }
            require(sharedCorners == 2U, "neighboring prism caps do not share exactly one edge");
        }
    }
}

void testBvhMatchesExhaustive(const voxel::GeodesicTopology& topology) {
    std::uint32_t raysWithHits = 0;
    constexpr std::uint32_t grid = 28;
    for (std::uint32_t cameraIndex = 0; cameraIndex < 8U; ++cameraIndex) {
        const double yaw = static_cast<double>(cameraIndex) * 0.7853981633974483;
        const double pitch = (static_cast<int>(cameraIndex % 3U) - 1) * 0.31;
        const Vec3 camera = normalized({std::sin(yaw) * std::cos(pitch), std::sin(pitch),
                                        std::cos(yaw) * std::cos(pitch)}) * 3.0;
        const Vec3 forward = normalized(camera * -1.0);
        const Vec3 right = normalized(cross(forward, {0.0, 1.0, 0.0}));
        const Vec3 up = cross(right, forward);
        for (std::uint32_t y = 0; y < grid; ++y) {
            for (std::uint32_t x = 0; x < grid; ++x) {
                const double screenX = (static_cast<double>(x) + 0.5) / grid * 0.9 - 0.45;
                const double screenY = (static_cast<double>(y) + 0.5) / grid * 0.9 - 0.45;
                const Vec3 direction = normalized(forward + right * screenX + up * screenY);
                const double expected = exhaustiveHit(topology, camera, direction);
                const double accelerated = acceleratedHit(topology, camera, direction);
                const double wideAccelerated = wideAcceleratedHit(topology, camera, direction);
                const double geodesicDda = geodesicDdaHit(topology, camera, direction);
                require((expected < 100.0) == (accelerated < 100.0),
                        "BVH and exhaustive ray coverage disagree");
                require((expected < 100.0) == (wideAccelerated < 100.0),
                        "wide hierarchy and exhaustive ray coverage disagree");
                require((expected < 100.0) == (geodesicDda < 100.0),
                        "geodesic DDA and exhaustive ray coverage disagree");
                if (expected < 100.0) {
                    ++raysWithHits;
                    require(std::abs(expected - accelerated) < 2e-5,
                            "BVH and exhaustive nearest hits disagree");
                    require(std::abs(expected - wideAccelerated) < 2e-5,
                            "wide hierarchy and exhaustive nearest hits disagree");
                    if (std::abs(expected - geodesicDda) >= 2e-5) {
                        std::uint32_t expectedTile = voxel::kInvalidTile;
                        for (std::uint32_t tile = 0U; tile < topology.tiles.size(); ++tile) {
                            double candidate = 0.0;
                            if (intersectColumn(topology, tile, camera, direction, candidate) &&
                                std::abs(candidate - expected) < 2e-5) {
                                expectedTile = tile;
                                break;
                            }
                        }
                        std::uint32_t ddaTile = voxel::kInvalidTile;
                        (void)geodesicDdaHit(topology, camera, direction, &ddaTile);
                        const std::uint32_t directionTile = locateTile(
                            topology, camera + direction * expected);
                        throw std::runtime_error(
                            "geodesic DDA and exhaustive nearest hits disagree: expected " +
                            std::to_string(expected) + ", DDA " +
                            std::to_string(geodesicDda) + ", camera " +
                            std::to_string(cameraIndex) + ", pixel " +
                            std::to_string(x) + "," + std::to_string(y) +
                            ", expected tile " + std::to_string(expectedTile) +
                            ", DDA tile " + std::to_string(ddaTile) +
                            ", hit-direction tile " + std::to_string(directionTile));
                    }
                }
            }
        }
    }
    require(raysWithHits > 1'000U, "ray regression did not exercise enough planet hits");
}

void testSurfaceBasinGrazingCoverage() {
    voxel::GeodesicTopology topology = voxel::GeodesicTopology::build(
        32U, 32U, 32U, 16U, 64U, false);

    // A low bowl surrounded by tall radial columns forces shallow player rays
    // to cross many angular cells before their first hit, matching the visual
    // failure mode that produced background-colored spokes across basin caps.
    const Vec3 basinDirection = normalized({0.0, 0.65, 0.76});
    for (std::uint32_t tile = 0U; tile < topology.tiles.size(); ++tile) {
        const double alignment = dot(unpack(topology.tiles[tile].center), basinDirection);
        const double bowlRadius = std::acos(std::clamp(alignment, -1.0, 1.0));
        const double rim = std::clamp((bowlRadius - 0.18) / 0.18, 0.0, 1.0);
        const std::uint32_t filled = static_cast<std::uint32_t>(
            std::lround(std::lerp(5.0, 29.0, rim * rim)));
        const std::uint32_t mask = filled == 32U ? 0xffffffffU : (1U << filled) - 1U;
        topology.columnStates[tile].data = {mask, filled, 2U, 0U};
    }

    const Vec3 radial{0.0, 0.0, 1.0};
    const Vec3 camera = radial * (static_cast<double>(topology.outerBoundingRadius) + 0.025);
    const double lookPitch = -0.35;
    const Vec3 forward = normalized(Vec3{0.0, std::cos(lookPitch), std::sin(lookPitch)});
    const Vec3 right{1.0, 0.0, 0.0};
    const Vec3 up = normalized(cross(right, forward));
    std::uint32_t grazingHits = 0U;
    std::uint32_t longPaths = 0U;
    constexpr std::uint32_t width = 56U;
    constexpr std::uint32_t height = 32U;
    for (std::uint32_t y = 0U; y < height; ++y) {
        for (std::uint32_t x = 0U; x < width; ++x) {
            const double screenX = ((static_cast<double>(x) + 0.5) / width * 2.0 - 1.0) * 1.75;
            const double screenY = -0.15 +
                (static_cast<double>(y) + 0.5) / height * 0.53;
            const Vec3 ray = normalized(forward + right * screenX + up * screenY);
            const double expected = exhaustiveHit(topology, camera, ray);
            std::uint32_t steps = 0U;
            const double dda = geodesicDdaHit(topology, camera, ray, nullptr, &steps);
            const double robust = geodesicDdaHit(
                topology, camera, ray, nullptr, nullptr, true, true);
            require((expected < 100.0) == (dda < 100.0),
                    "Surface basin ray leaked through the geodesic shell");
            require((expected < 100.0) == (robust < 100.0),
                    "Robust basin replay leaked through the geodesic shell");
            if (expected < 100.0) {
                require(std::abs(expected - dda) < 2e-5,
                        "Surface basin grazing ray chose the wrong nearest hit");
                require(std::abs(expected - robust) < 2e-5,
                        "Robust basin replay changed the nearest hit");
                ++grazingHits;
                longPaths += steps > 6U ? 1U : 0U;
            }
            double bedrockNear = 0.0;
            double bedrockFar = 0.0;
            if (raySphereInterval(camera, ray, topology.bedrockRadius,
                                  bedrockNear, bedrockFar)) {
                std::uint32_t terminationReason = 0U;
                const double closedShellHit = geodesicDdaHit(
                    topology, camera, ray, nullptr, nullptr, true, false,
                    &terminationReason);
                require(closedShellHit < 100.0,
                        "Closed-shell camera ray returned background");
                require(terminationReason == 1U,
                        "Closed-shell ray ended through no-exit/far/step termination");
            }

        }
    }
    require(grazingHits > width * height / 10U,
            "Surface basin regression did not cover enough terrain pixels");
    require(longPaths > grazingHits / 20U,
            "Surface basin regression did not exercise long grazing traversal");
}

void testParallelHorizonBoundaryCoverage() {
    voxel::GeodesicTopology topology = voxel::GeodesicTopology::build(
        32U, 32U, 32U, 16U, 64U, false);
    for (auto& state : topology.columnStates) {
        state.data = {0U, 0U, 0U, 0U};
    }

    // Build the exact failure rather than relying on a lucky raster sample:
    // the camera and ray both lie in one shared bisector plane. Floating-point
    // ownership selects one empty column, while the equally valid partner is
    // solid. An ordered DDA never crosses a plane it travels along, so the
    // miss-only parallel-partner probe is required to prevent a sky pixel.
    bool discriminatingRayFound = false;
    for (std::uint32_t tile = 0U;
         tile < topology.tiles.size() && !discriminatingRayFound; ++tile) {
        const Vec3 firstCenter = unpack(topology.tiles[tile].center);
        for (std::uint32_t slot = 0U;
             slot < topology.tiles[tile].brickInfo[2] && !discriminatingRayFound; ++slot) {
            const std::uint32_t adjacent = neighbor(topology.tiles[tile], slot);
            if (adjacent < tile) {
                continue;
            }
            const Vec3 secondCenter = unpack(topology.tiles[adjacent].center);
            const Vec3 boundaryRadial = normalized(firstCenter + secondCenter);
            const Vec3 boundaryNormal = normalized(firstCenter - secondCenter);
            const Vec3 boundaryTangent = normalized(cross(boundaryNormal, boundaryRadial));
            for (double sign : {-1.0, 1.0}) {
                const Vec3 camera = boundaryRadial *
                    (static_cast<double>(topology.outerBoundingRadius) + 0.002);
                const Vec3 ray = normalized(
                    boundaryTangent * sign - boundaryRadial * 0.35);
                double travel = 0.0;
                double farDistance = 0.0;
                if (!raySphereInterval(camera, ray, 1.15, travel, farDistance)) {
                    continue;
                }
                const Vec3 entry = camera + ray * (travel + 1e-7);
                const std::uint32_t owner = locateTile(topology, entry);
                const Vec3 ownerCenter = unpack(topology.tiles[owner].center);
                std::uint32_t partner = voxel::kInvalidTile;
                for (std::uint32_t ownerSlot = 0U;
                     ownerSlot < topology.tiles[owner].brickInfo[2]; ++ownerSlot) {
                    const std::uint32_t candidate = neighbor(
                        topology.tiles[owner], ownerSlot);
                    const Vec3 inward = normalized(
                        ownerCenter - unpack(topology.tiles[candidate].center));
                    if (std::abs(dot(inward, ray)) < 1e-6 &&
                        std::abs(dot(inward, entry)) < 1e-5) {
                        partner = candidate;
                        break;
                    }
                }
                if (partner == voxel::kInvalidTile) {
                    continue;
                }

                topology.columnStates[partner].data = {
                    0xffffffffU, 32U, 2U, 0U};
                const double expected = exhaustiveHit(topology, camera, ray);

                if (expected < 100.0) {
                    for (const Vec3 variant : ulpJitteredDirections(ray)) {
                        requireDdaMatchesExhaustive(
                            topology, camera, variant,
                            "parallel shared-face horizon");
                    }
                    discriminatingRayFound = true;
                }
                topology.columnStates[partner].data = {0U, 0U, 0U, 0U};
                if (discriminatingRayFound) {
                    break;
                }
            }
        }
    }
    require(discriminatingRayFound,
            "Parallel horizon regression did not discriminate the coverage fix");
}

void testMultiPlaneVertexCoverage() {
    voxel::GeodesicTopology topology = voxel::GeodesicTopology::build(
        32U, 32U, 32U, 16U, 64U, false);
    for (auto& state : topology.columnStates) {
        state.data = {0U, 0U, 0U, 0U};
    }

    bool vertexFound = false;
    for (std::uint32_t source = 0U;
         source < topology.tiles.size() && !vertexFound; ++source) {
        for (const Vec3 corner : outerCorners(topology, source)) {
            const Vec3 radial = normalized(corner);
            const Vec3 camera = radial *
                (static_cast<double>(topology.outerBoundingRadius) + 0.002);
            const Vec3 ray = radial * -1.0;
            const Vec3 entry = camera + ray * 1e-7;
            const std::uint32_t owner = locateTile(topology, entry);
            const Vec3 ownerCenter = unpack(topology.tiles[owner].center);
            std::array<std::uint32_t, 2> partners{
                voxel::kInvalidTile, voxel::kInvalidTile};
            std::uint32_t partnerCount = 0U;
            for (std::uint32_t slot = 0U;
                 slot < topology.tiles[owner].brickInfo[2]; ++slot) {
                const std::uint32_t candidate = neighbor(topology.tiles[owner], slot);
                const Vec3 inward = normalized(
                    ownerCenter - unpack(topology.tiles[candidate].center));
                if (std::abs(dot(inward, entry)) < 1e-5 &&
                    std::abs(dot(inward, ray)) < 1e-6 && partnerCount < 2U) {
                    partners[partnerCount++] = candidate;
                }
            }
            if (partnerCount != 2U) {
                continue;
            }

            for (const std::uint32_t partner : partners) {
                topology.columnStates[partner].data = {
                    0xffffffffU, 32U, 4U, 0U};
                const double expected = exhaustiveHit(topology, camera, ray);
                require(expected < 100.0,
                        "Vertex regression did not intersect its solid partner");
                for (const Vec3 variant : ulpJitteredDirections(ray)) {
                    requireDdaMatchesExhaustive(
                        topology, camera, variant,
                        "multi-plane geodesic vertex");
                }
                topology.columnStates[partner].data = {0U, 0U, 0U, 0U};
            }
            vertexFound = true;
            break;
        }
    }
    require(vertexFound,
            "Multi-plane vertex regression did not find a shared triple point");
}

void testMappedContentEntryOrdering(
    const voxel::GeodesicTopology& topology) {
    const double margin = 1.0e-4;
    const double safeOuter = topology.outerBoundingRadius + margin;
    const std::array<Vec3, 8> chartDirections{{
        {1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0},
        {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0},
        normalized({0.43, -0.71, 0.56}),
        normalized({-0.62, 0.31, -0.72})}};
    for (const Vec3 rawRadial : chartDirections) {
        const Vec3 radial = normalized(rawRadial);
        const Vec3 reference = std::abs(radial.y) < 0.8
            ? Vec3{0.0, 1.0, 0.0} : Vec3{1.0, 0.0, 0.0};
        const Vec3 tangent = normalized(cross(reference, radial));
        const Vec3 direction = normalized(radial * -1.0 + tangent * 0.035);
        const Vec3 embeddedOrigin = radial * 0.98 + tangent * 0.015;
        double reverseNear = 0.0;
        double reverseFar = 0.0;
        require(raySphereInterval(embeddedOrigin, direction * -1.0,
                                  safeOuter, reverseNear, reverseFar),
                "mapped content origin could not be repaired to outer shell");
        const Vec3 repairedOrigin =
            embeddedOrigin - direction * (reverseFar + margin);
        require(std::sqrt(dot(repairedOrigin, repairedOrigin)) > safeOuter,
                "mapped content repair remained inside terrain shell");

        Vec3 hitNormal{};
        const double hit = exhaustiveHit(
            topology, repairedOrigin, direction, &hitNormal);
        require(hit < 100.0 && hit > 0.0,
                "repaired mapped ray missed the closed voxel planet");
        require(dot(hitNormal, direction) < 0.0,
                "mapped ray selected an interior/back-facing prism root");

        Vec3 skyNormal{};
        const double sky = exhaustiveHit(
            topology, repairedOrigin, radial, &skyNormal);
        require(sky >= 100.0,
                "ordinary outward content sky ray hit the planet back side");
    }
}

} // namespace

int main() {
    try {
        const voxel::GeodesicTopology topology = voxel::GeodesicTopology::build(8U, 16U, 64U, 32U);
        testSharedEdgesAndLeafBounds(topology);
        testBvhMatchesExhaustive(topology);
        testSurfaceBasinGrazingCoverage();
        testParallelHorizonBoundaryCoverage();
        testMultiPlaneVertexCoverage();
        testMappedContentEntryOrdering(topology);
        std::cout << "Watertight geometry, binary BVH, wide hierarchy, and geodesic DDA ray gates passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Geometry gate failed: " << error.what() << '\n';
        return 1;
    }
}
