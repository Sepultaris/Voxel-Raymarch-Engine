#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <compare>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "planet/geodesic_topology.hpp"

namespace voxel::system_lab {

inline constexpr std::uint32_t kPlanetCount = 8U;
inline constexpr std::uint32_t kPentagonsPerLevel = 12U;
inline constexpr std::uint32_t kSystemExactLodLevel = 9U; // f512
inline constexpr std::uint32_t kSystemCoarseLodLevels = 9U; // f1..f256
inline constexpr std::uint32_t kSystemPageColumns = 32U;
inline constexpr std::uint32_t kSystemResidentPagesPerPlanet = 512U;
inline constexpr std::uint64_t kSystemAuthorityMemoryGuardrail =
    384ULL * 1024ULL * 1024ULL;

struct Settings {
    float starRadius{2.0F};
    float starRadiance{7.0F};
    std::array<float, 3> starColor{1.0F, 0.90F, 0.72F};
    std::array<float, 3> ambientColor{0.14F, 0.18F, 0.24F};
    float ambientStrength{1.0F};
    std::uint32_t shadowSamples{8U};
    std::uint32_t debugView{};
    std::uint32_t selectedPlanet{};
    std::array<float, kPlanetCount> planetRadii{
        0.72F, 0.775F, 0.83F, 0.885F, 0.94F, 0.995F, 1.05F, 1.105F};
    std::array<std::uint32_t, kPlanetCount> planetSeeds{
        2654437106U, 1013905563U, 3668341320U, 2027819777U,
        387299938U, 3041735705U, 1401184162U, 4055629929U};
};

struct Vec3d {
    double x{};
    double y{};
    double z{};
};

[[nodiscard]] constexpr Vec3d operator+(Vec3d a, Vec3d b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
[[nodiscard]] constexpr Vec3d operator-(Vec3d a, Vec3d b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
[[nodiscard]] constexpr Vec3d operator*(Vec3d a, double s) noexcept {
    return {a.x * s, a.y * s, a.z * s};
}
[[nodiscard]] constexpr double dot(Vec3d a, Vec3d b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
[[nodiscard]] inline double length(Vec3d value) noexcept {
    return std::sqrt(dot(value, value));
}
[[nodiscard]] inline Vec3d normalized(Vec3d value) noexcept {
    const double magnitude = length(value);
    return magnitude > 0.0 ? value * (1.0 / magnitude) : Vec3d{0.0, 0.0, 1.0};
}
[[nodiscard]] constexpr Vec3d cross(Vec3d a, Vec3d b) noexcept {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

struct HighLow3 {
    std::array<float, 3> high{};
    std::array<float, 3> low{};
};

[[nodiscard]] inline HighLow3 splitHighLow(Vec3d value) noexcept {
    const std::array<float, 3> high{
        static_cast<float>(value.x), static_cast<float>(value.y),
        static_cast<float>(value.z)};
    return {high,
            {static_cast<float>(value.x - static_cast<double>(high[0])),
             static_cast<float>(value.y - static_cast<double>(high[1])),
             static_cast<float>(value.z - static_cast<double>(high[2]))}};
}

struct Orbit {
    double radius{};
    double angularSpeed{};
    double phase{};
    double inclination{};
};

struct Planet {
    std::uint32_t seed{};
    double radius{1.0};
    Orbit orbit{};
};

struct BodyFrame {
    Vec3d position{};
    Vec3d velocity{};
    Vec3d radial{1.0, 0.0, 0.0};
    Vec3d tangent{0.0, 1.0, 0.0};
    Vec3d normal{0.0, 0.0, 1.0};
};

[[nodiscard]] inline BodyFrame evaluateOrbit(const Orbit& orbit,
                                              double simulationTime,
                                              double speedScale) noexcept {
    const double angle = orbit.phase + simulationTime * orbit.angularSpeed * speedScale;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double ci = std::cos(orbit.inclination);
    const double si = std::sin(orbit.inclination);
    const Vec3d radial{c, s * ci, s * si};
    const Vec3d tangent{-s, c * ci, c * si};
    const double speed = orbit.radius * orbit.angularSpeed * speedScale;
    return {radial * orbit.radius, tangent * speed, radial, tangent,
            normalized(cross(radial, tangent))};
}

[[nodiscard]] inline std::array<Planet, kPlanetCount> defaultPlanets() noexcept {
    std::array<Planet, kPlanetCount> planets{};
    for (std::uint32_t index = 0; index < kPlanetCount; ++index) {
        const double orbitRadius = 7.0 + 2.75 * static_cast<double>(index);
        planets[index] = {
            0x9e3779b9U * (index + 1U) + 1337U,
            0.72 + 0.055 * static_cast<double>(index),
            {orbitRadius,
             0.16 / std::sqrt(orbitRadius),
             2.0 * std::numbers::pi * static_cast<double>(index) /
                 static_cast<double>(kPlanetCount),
             (static_cast<double>(index % 3U) - 1.0) * 0.055}};
    }
    return planets;
}

struct MouthFrame {
    Vec3d position{};
    Vec3d velocity{};
    Vec3d right{};
    Vec3d up{};
    Vec3d forward{};
};

[[nodiscard]] inline MouthFrame anchoredMouth(const BodyFrame& body,
                                               double planetRadius,
                                               bool outgoing) noexcept {
    const double sign = outgoing ? 1.0 : -1.0;
    const Vec3d outward = normalized(body.radial * 0.78 + body.normal * (0.32 * sign));
    const Vec3d right = normalized(cross(body.normal, outward));
    const Vec3d up = normalized(cross(outward, right));
    return {body.position + outward * (planetRadius * 1.24), body.velocity,
            right, up, outward};
}

struct TransportedState {
    Vec3d position{};
    Vec3d direction{};
    Vec3d velocity{};
    Vec3d right{};
    Vec3d up{};
};

[[nodiscard]] inline Vec3d frameToWorld(const MouthFrame& frame,
                                         Vec3d local) noexcept {
    return frame.right * local.x + frame.up * local.y + frame.forward * local.z;
}
[[nodiscard]] inline Vec3d worldToFrame(const MouthFrame& frame,
                                         Vec3d world) noexcept {
    return {dot(world, frame.right), dot(world, frame.up), dot(world, frame.forward)};
}

[[nodiscard]] inline TransportedState transportThroughHandle(
    const MouthFrame& source, const MouthFrame& destination,
    Vec3d position, Vec3d direction, Vec3d velocity,
    Vec3d cameraRight, Vec3d cameraUp) noexcept {
    Vec3d localPosition = worldToFrame(source, position - source.position);
    Vec3d localDirection = worldToFrame(source, direction);
    Vec3d localVelocity = worldToFrame(source, velocity - source.velocity);
    Vec3d localRight = worldToFrame(source, cameraRight);
    Vec3d localUp = worldToFrame(source, cameraUp);
    // The complete orthonormal triad is identified without a reflection.
    // An entering ray (negative local forward) therefore emerges toward the
    // destination planet while frame handedness and relative velocity remain
    // continuous in the two moving endpoint frames.
    return {destination.position + frameToWorld(destination, localPosition),
            normalized(frameToWorld(destination, localDirection)),
            destination.velocity + frameToWorld(destination, localVelocity),
            normalized(frameToWorld(destination, localRight)),
            normalized(frameToWorld(destination, localUp))};
}

struct CycleHandle {
    std::uint32_t source{};
    std::uint32_t destination{};
};

[[nodiscard]] constexpr std::array<CycleHandle, kPlanetCount>
directedCycle() noexcept {
    std::array<CycleHandle, kPlanetCount> handles{};
    for (std::uint32_t index = 0; index < kPlanetCount; ++index) {
        handles[index] = {index, (index + 1U) % kPlanetCount};
    }
    return handles;
}

struct PrimalTriangleId {
    std::uint64_t value{};
    [[nodiscard]] constexpr std::uint32_t level() const noexcept {
        return static_cast<std::uint32_t>((value >> 58U) & 0x3fU);
    }
    [[nodiscard]] constexpr std::uint32_t rootFace() const noexcept {
        return static_cast<std::uint32_t>((value >> 53U) & 0x1fU);
    }
    [[nodiscard]] constexpr PrimalTriangleId parent() const noexcept {
        const std::uint32_t currentLevel = level();
        if (currentLevel == 0U) return *this;
        const std::uint64_t pathMask = (std::uint64_t{1} << 53U) - 1U;
        const std::uint64_t path = (value & pathMask) >> 2U;
        return {std::uint64_t(currentLevel - 1U) << 58U |
                std::uint64_t(rootFace()) << 53U | path};
    }
    [[nodiscard]] constexpr PrimalTriangleId child(std::uint32_t childIndex) const noexcept {
        const std::uint64_t pathMask = (std::uint64_t{1} << 53U) - 1U;
        return {std::uint64_t(level() + 1U) << 58U |
                std::uint64_t(rootFace()) << 53U |
                (((value & pathMask) << 2U) | (childIndex & 3U))};
    }
    [[nodiscard]] static constexpr PrimalTriangleId root(std::uint32_t face) noexcept {
        return {std::uint64_t(face & 0x1fU) << 53U};
    }
    friend constexpr bool operator==(PrimalTriangleId, PrimalTriangleId) = default;
    friend constexpr auto operator<=>(PrimalTriangleId, PrimalTriangleId) = default;
};

[[nodiscard]] constexpr std::uint64_t triangleCount(std::uint32_t level) noexcept {
    return std::uint64_t{20} << (2U * level);
}
[[nodiscard]] constexpr std::uint64_t dualCellCount(std::uint32_t level) noexcept {
    return std::uint64_t{10} * (std::uint64_t{1} << (2U * level)) + 2U;
}
[[nodiscard]] constexpr std::uint64_t hexagonCount(std::uint32_t level) noexcept {
    return dualCellCount(level) - kPentagonsPerLevel;
}

struct Aggregate {
    float minimumHeight{};
    float maximumHeight{};
    std::array<std::uint32_t, 4> materialCounts{};
    bool editDirty{};
    float maximumProjectedError{};
};

[[nodiscard]] inline Aggregate conservativeAggregate(
    std::span<const Aggregate> children) noexcept {
    Aggregate parent{};
    parent.minimumHeight = std::numeric_limits<float>::infinity();
    parent.maximumHeight = -std::numeric_limits<float>::infinity();
    for (const Aggregate& child : children) {
        parent.minimumHeight = std::min(parent.minimumHeight, child.minimumHeight);
        parent.maximumHeight = std::max(parent.maximumHeight, child.maximumHeight);
        for (std::size_t index = 0; index < parent.materialCounts.size(); ++index) {
            parent.materialCounts[index] += child.materialCounts[index];
        }
        parent.editDirty = parent.editDirty || child.editDirty;
        parent.maximumProjectedError = std::max(
            parent.maximumProjectedError, child.maximumProjectedError);
    }
    if (children.empty()) {
        parent.minimumHeight = 0.0F;
        parent.maximumHeight = 0.0F;
    }
    return parent;
}

[[nodiscard]] inline float lodTransition(float projectedPixels) noexcept {
    return std::clamp((projectedPixels - 0.25F) / 0.75F, 0.0F, 1.0F);
}

[[nodiscard]] inline std::uint32_t selectLod(float projectedRootPixels,
                                             float geodesicJacobian,
                                             std::uint32_t maximumLevel) noexcept {
    const float footprint = std::max(projectedRootPixels, 0.0F) *
        std::sqrt(std::max(geodesicJacobian, 0.0F));
    std::uint32_t level = 0U;
    float pixels = footprint;
    while (level < maximumLevel && pixels > 1.0F) {
        ++level;
        pixels *= 0.5F;
    }
    return level;
}

inline void enforceTwoToOne(std::span<std::uint32_t> levels,
                            std::span<const std::array<std::uint32_t, 6>> neighbors) noexcept {
    bool changed = true;
    while (changed) {
        changed = false;
        for (std::size_t index = 0; index < levels.size(); ++index) {
            for (const std::uint32_t neighbor : neighbors[index]) {
                if (neighbor >= levels.size()) continue;
                if (levels[index] + 1U < levels[neighbor]) {
                    levels[index] = levels[neighbor] - 1U;
                    changed = true;
                }
            }
        }
    }
}

struct SparseEdit {
    float heightDelta{};
    std::uint32_t material{};
};

class SparseEditHierarchy {
public:
    void set(PrimalTriangleId leaf, SparseEdit edit) { edits_[leaf] = edit; }
    [[nodiscard]] const SparseEdit* find(PrimalTriangleId leaf) const noexcept {
        const auto found = edits_.find(leaf);
        return found == edits_.end() ? nullptr : &found->second;
    }
    [[nodiscard]] bool ancestorDirty(PrimalTriangleId node) const noexcept {
        for (const auto& [leaf, unused] : edits_) {
            (void)unused;
            PrimalTriangleId cursor = leaf;
            while (cursor.level() > node.level()) cursor = cursor.parent();
            if (cursor == node) return true;
        }
        return false;
    }
    void evictResidentPages() noexcept {} // Edits deliberately live outside residency.
private:
    std::map<PrimalTriangleId, SparseEdit> edits_;
};

// GPU aggregates are deliberately compact because they are independent per
// planet. Heights and error are conservatively quantized: minima round down,
// maxima/error round up. This keeps the complete f1..f256 hierarchy under the
// system memory guardrail while preserving a genuine one-level f256->f512
// boundary.
struct PackedAggregateGpu {
    std::uint32_t heights{};       // min UNORM16 | max UNORM16
    std::uint32_t materialError{}; // dominant/mask/error UNORM16/edit-dirty
};
static_assert(sizeof(PackedAggregateGpu) == 8U);

struct GpuAggregateState {
    float minimumHeight{};
    float maximumHeight{};
    std::uint32_t dominantMaterial{1U};
    std::uint32_t materialMask{};
    bool editDirty{};
    float maximumProjectedError{};
};

[[nodiscard]] inline std::uint32_t aggregateQuantizeFloor(float value) noexcept {
    return static_cast<std::uint32_t>(std::floor(
        std::clamp(value, 0.0F, 1.0F) * 65535.0F));
}

[[nodiscard]] inline std::uint32_t aggregateQuantizeCeil(float value) noexcept {
    return static_cast<std::uint32_t>(std::ceil(
        std::clamp(value, 0.0F, 1.0F) * 65535.0F));
}

[[nodiscard]] inline PackedAggregateGpu packAggregateGpu(
    const Aggregate& aggregate) noexcept {
    const std::uint32_t minimum = aggregateQuantizeFloor(
        aggregate.minimumHeight);
    const std::uint32_t maximum = aggregateQuantizeCeil(
        aggregate.maximumHeight);
    const auto dominant = static_cast<std::uint32_t>(std::distance(
        aggregate.materialCounts.begin(),
        std::max_element(aggregate.materialCounts.begin(),
                         aggregate.materialCounts.end()))) + 1U;
    std::uint32_t materialMask = 0U;
    for (std::uint32_t material = 0U; material < 4U; ++material) {
        if (aggregate.materialCounts[material] != 0U) {
            materialMask |= 1U << material;
        }
    }
    const std::uint32_t error = aggregateQuantizeCeil(
        aggregate.maximumProjectedError);
    return {
        minimum | (maximum << 16U),
        dominant | (materialMask << 8U) | (error << 12U) |
            (aggregate.editDirty ? 0x80000000U : 0U)};
}

[[nodiscard]] inline GpuAggregateState unpackAggregateGpu(
    PackedAggregateGpu packed) noexcept {
    constexpr float inverseUnorm16 = 1.0F / 65535.0F;
    return {
        static_cast<float>(packed.heights & 0xffffU) * inverseUnorm16,
        static_cast<float>(packed.heights >> 16U) * inverseUnorm16,
        std::max(packed.materialError & 0xffU, 1U),
        (packed.materialError >> 8U) & 0x0fU,
        (packed.materialError & 0x80000000U) != 0U,
        static_cast<float>((packed.materialError >> 12U) & 0xffffU) *
            inverseUnorm16};
}

// The system lab deliberately keeps procedural source state and edits separate
// from sparse residency. A cache miss is not silently accepted as empty: it is
// represented by a full conservative column until the authoritative page has
// been generated. This is the same rule used by collision and edit picking.
struct CompactColumnState {
    std::uint8_t filledLayers{32U};
    std::uint8_t material{1U};
    std::uint8_t flags{};
    std::uint8_t reserved{};
    std::uint32_t generation{};

    [[nodiscard]] constexpr bool unknown() const noexcept {
        return (flags & 1U) != 0U;
    }
    [[nodiscard]] constexpr bool edited() const noexcept {
        return (flags & 2U) != 0U;
    }
};
static_assert(sizeof(CompactColumnState) == 8U);

[[nodiscard]] constexpr std::uint32_t packCompactColumnGpu(
    CompactColumnState state) noexcept {
    return static_cast<std::uint32_t>(state.filledLayers) |
        (static_cast<std::uint32_t>(state.material) << 8U) |
        (static_cast<std::uint32_t>(state.flags) << 16U);
}

[[nodiscard]] constexpr CompactColumnState unpackCompactColumnGpu(
    std::uint32_t packed, std::uint32_t generation = 1U) noexcept {
    return {static_cast<std::uint8_t>(packed & 0xffU),
            static_cast<std::uint8_t>((packed >> 8U) & 0xffU),
            static_cast<std::uint8_t>((packed >> 16U) & 0xffU),
            0U, generation};
}

[[nodiscard]] constexpr float compactColumnCapScale(
    CompactColumnState state, float baseScale = 0.92F,
    float radialDepthScale = 0.12F) noexcept {
    return baseScale + radialDepthScale *
        static_cast<float>(state.filledLayers) / 32.0F;
}

struct AtmosphereCloudControls {
    float atmosphereDensity{0.22F};
    float atmosphereHeight{0.075F};
    float cloudCoverage{0.48F};
    float cloudDensity{0.65F};
    float cloudBase{0.045F};
    float cloudTop{0.085F};
};

[[nodiscard]] constexpr std::uint32_t systemAvalanche(
    std::uint32_t value) noexcept {
    value ^= value >> 16U;
    value *= 0x7feb352dU;
    value ^= value >> 15U;
    value *= 0x846ca68bU;
    value ^= value >> 16U;
    return value;
}

[[nodiscard]] constexpr CompactColumnState generatedColumn(
    std::uint32_t seed, std::uint32_t tileIndex,
    std::uint32_t generation = 1U) noexcept {
    const std::uint32_t heightBits = systemAvalanche(
        tileIndex ^ seed ^ generation * 0x9e3779b9U);
    const std::uint32_t materialBits = systemAvalanche(
        tileIndex * 0x85ebca6bU ^ seed * 0xc2b2ae35U);
    // f512 columns retain a visible but bounded relief band. The common
    // immutable radial frame supplies 32 layers; only this compact result is
    // planet-specific.
    return {static_cast<std::uint8_t>(20U + heightBits % 13U),
            static_cast<std::uint8_t>(1U + materialBits % 5U),
            0U, 0U, generation};
}

struct ResidentPage {
    std::uint32_t virtualPage{std::numeric_limits<std::uint32_t>::max()};
    std::uint32_t generation{};
    std::uint64_t lastUse{};
    bool pinned{};
    std::array<CompactColumnState, kSystemPageColumns> columns{};
};

struct PlanetMemoryTelemetry {
    std::uint64_t pageTableBytes{};
    std::uint64_t residentPoolBytes{};
    std::uint64_t editOverlayBytes{};
    std::uint32_t residentPages{};
    std::uint32_t pinnedPages{};
    std::uint32_t edits{};
    std::uint32_t generation{};
};

class PlanetTerrainAuthority {
public:
    explicit PlanetTerrainAuthority(
        std::uint32_t seed = 1U,
        std::uint32_t pageCapacity = kSystemResidentPagesPerPlanet)
        : seed_(seed == 0U ? 1U : seed), pages_(std::max(pageCapacity, 1U)) {}

    [[nodiscard]] std::uint32_t seed() const noexcept { return seed_; }
    [[nodiscard]] std::uint32_t generation() const noexcept { return generation_; }
    [[nodiscard]] const AtmosphereCloudControls& media() const noexcept {
        return media_;
    }
    void setMedia(AtmosphereCloudControls media) noexcept { media_ = media; }

    void regenerate(std::uint32_t seed) {
        seed_ = seed == 0U ? 1U : seed;
        ++generation_;
        if (generation_ == 0U) generation_ = 1U;
        pageToSlot_.clear();
        for (ResidentPage& page : pages_) page = {};
        // Generation invalidates generated pages, never authored edits.
    }

    [[nodiscard]] CompactColumnState query(std::uint32_t tileIndex) noexcept {
        if (const auto edited = edits_.find(tileIndex); edited != edits_.end()) {
            CompactColumnState state = edited->second;
            state.flags |= 2U;
            state.generation = generation_;
            return state;
        }
        const std::uint32_t virtualPage = tileIndex / kSystemPageColumns;
        const auto resident = pageToSlot_.find(virtualPage);
        if (resident == pageToSlot_.end()) {
            return {32U, 1U, 1U, 0U, generation_};
        }
        ResidentPage& page = pages_[resident->second];
        page.lastUse = ++clock_;
        CompactColumnState state =
            page.columns[tileIndex % kSystemPageColumns];
        if (state.generation != generation_) {
            return {32U, 1U, 1U, 0U, generation_};
        }
        return state;
    }

    void makeResident(std::uint32_t tileIndex, bool pin = false) {
        const std::uint32_t virtualPage = tileIndex / kSystemPageColumns;
        if (const auto found = pageToSlot_.find(virtualPage);
            found != pageToSlot_.end()) {
            ResidentPage& page = pages_[found->second];
            page.lastUse = ++clock_;
            page.pinned = page.pinned || pin;
            return;
        }
        std::size_t slot = pages_.size();
        for (std::size_t index = 0; index < pages_.size(); ++index) {
            if (pages_[index].virtualPage == std::numeric_limits<std::uint32_t>::max()) {
                slot = index;
                break;
            }
        }
        if (slot == pages_.size()) {
            std::uint64_t oldest = std::numeric_limits<std::uint64_t>::max();
            for (std::size_t index = 0; index < pages_.size(); ++index) {
                if (!pages_[index].pinned && pages_[index].lastUse < oldest) {
                    oldest = pages_[index].lastUse;
                    slot = index;
                }
            }
        }
        if (slot == pages_.size()) {
            throw std::runtime_error(
                "eight-planet residency guardrail: every physical page is pinned");
        }
        ResidentPage& page = pages_[slot];
        if (page.virtualPage != std::numeric_limits<std::uint32_t>::max()) {
            pageToSlot_.erase(page.virtualPage);
        }
        page.virtualPage = virtualPage;
        page.generation = generation_;
        page.lastUse = ++clock_;
        page.pinned = pin;
        const std::uint32_t firstTile = virtualPage * kSystemPageColumns;
        for (std::uint32_t column = 0U; column < kSystemPageColumns; ++column) {
            page.columns[column] = generatedColumn(
                seed_, firstTile + column, generation_);
        }
        pageToSlot_[virtualPage] = slot;
    }

    void edit(std::uint32_t tileIndex, CompactColumnState state) {
        state.filledLayers = std::clamp<std::uint8_t>(state.filledLayers, 1U, 32U);
        state.material = std::max<std::uint8_t>(state.material, 1U);
        state.flags = static_cast<std::uint8_t>(state.flags | 2U);
        state.generation = generation_;
        edits_[tileIndex] = state;
        makeResident(tileIndex, true);
        ResidentPage& page = pages_[pageToSlot_.at(tileIndex / kSystemPageColumns)];
        page.columns[tileIndex % kSystemPageColumns] = state;
    }

    void evictUnpinned() noexcept {
        pageToSlot_.clear();
        for (ResidentPage& page : pages_) {
            if (page.pinned) {
                pageToSlot_[page.virtualPage] =
                    static_cast<std::size_t>(&page - pages_.data());
            } else {
                page = {};
            }
        }
    }

    void pinFinestTwoRing(const GeodesicTopology& topology,
                          std::uint32_t centerTile) {
        if (centerTile >= topology.tiles.size()) return;
        std::set<std::uint32_t> frontier{centerTile};
        std::set<std::uint32_t> visited{centerTile};
        for (std::uint32_t ring = 0U; ring < 2U; ++ring) {
            std::set<std::uint32_t> next;
            for (const std::uint32_t tileIndex : frontier) {
                const GeodesicTileGpu& tile = topology.tiles[tileIndex];
                for (std::uint32_t slot = 0U; slot < tile.brickInfo[2]; ++slot) {
                    const std::uint32_t neighbor = slot < 4U
                        ? tile.neighborsLow[slot]
                        : tile.neighborsHigh[slot - 4U];
                    if (neighbor < topology.tiles.size() && visited.insert(neighbor).second) {
                        next.insert(neighbor);
                    }
                }
            }
            frontier = std::move(next);
        }
        for (const std::uint32_t tileIndex : visited) makeResident(tileIndex, true);
    }

    [[nodiscard]] PlanetMemoryTelemetry telemetry() const noexcept {
        PlanetMemoryTelemetry result{};
        result.pageTableBytes = pageToSlot_.size() *
            (sizeof(std::uint32_t) + sizeof(std::size_t));
        result.residentPoolBytes = pages_.size() * sizeof(ResidentPage);
        result.editOverlayBytes = edits_.size() *
            (sizeof(std::uint32_t) + sizeof(CompactColumnState));
        result.residentPages = static_cast<std::uint32_t>(pageToSlot_.size());
        result.pinnedPages = static_cast<std::uint32_t>(std::count_if(
            pages_.begin(), pages_.end(), [](const ResidentPage& page) {
                return page.pinned;
            }));
        result.edits = static_cast<std::uint32_t>(edits_.size());
        result.generation = generation_;
        return result;
    }

    [[nodiscard]] const std::vector<ResidentPage>& pages() const noexcept {
        return pages_;
    }
    [[nodiscard]] const std::unordered_map<std::uint32_t, CompactColumnState>&
    edits() const noexcept { return edits_; }

private:
    std::uint32_t seed_{1U};
    std::uint32_t generation_{1U};
    std::uint64_t clock_{};
    AtmosphereCloudControls media_{};
    std::vector<ResidentPage> pages_;
    std::unordered_map<std::uint32_t, std::size_t> pageToSlot_;
    std::unordered_map<std::uint32_t, CompactColumnState> edits_;
};

struct SystemMemoryTelemetry {
    std::uint64_t immutableTopologyBytes{};
    std::uint64_t sharedHierarchyBytes{};
    std::uint64_t authorityBytes{};
    std::array<PlanetMemoryTelemetry, kPlanetCount> planets{};

    [[nodiscard]] constexpr std::uint64_t totalBytes() const noexcept {
        return immutableTopologyBytes + sharedHierarchyBytes + authorityBytes;
    }
    [[nodiscard]] constexpr bool withinGuardrail() const noexcept {
        return totalBytes() <= kSystemAuthorityMemoryGuardrail;
    }
};

class EightPlanetTerrainAuthorities {
public:
    explicit EightPlanetTerrainAuthorities(
        std::shared_ptr<const GeodesicTopology> topology,
        const std::array<std::uint32_t, kPlanetCount>& seeds)
        : topology_(std::move(topology)) {
        if (!topology_ || topology_->frequency != 512U) {
            throw std::invalid_argument(
                "eight-planet authorities require one immutable f512 topology");
        }
        for (std::uint32_t index = 0U; index < kPlanetCount; ++index) {
            planets_[index] = std::make_unique<PlanetTerrainAuthority>(seeds[index]);
        }
    }

    [[nodiscard]] const GeodesicTopology& topology() const noexcept {
        return *topology_;
    }
    [[nodiscard]] PlanetTerrainAuthority& planet(std::uint32_t index) noexcept {
        return *planets_[index % kPlanetCount];
    }
    [[nodiscard]] const PlanetTerrainAuthority& planet(
        std::uint32_t index) const noexcept {
        return *planets_[index % kPlanetCount];
    }
    [[nodiscard]] const GeodesicTopology* topologyAddress(
        std::uint32_t) const noexcept { return topology_.get(); }

    [[nodiscard]] SystemMemoryTelemetry telemetry(
        std::uint64_t sharedHierarchyBytes = 0U) const noexcept {
        SystemMemoryTelemetry result{};
        result.immutableTopologyBytes =
            topology_->tiles.size() * sizeof(GeodesicTileGpu) +
            topology_->directionLookup.size() * sizeof(std::uint32_t);
        result.sharedHierarchyBytes = sharedHierarchyBytes;
        for (std::uint32_t index = 0U; index < kPlanetCount; ++index) {
            result.planets[index] = planets_[index]->telemetry();
            result.authorityBytes += result.planets[index].pageTableBytes +
                result.planets[index].residentPoolBytes +
                result.planets[index].editOverlayBytes;
        }
        return result;
    }

private:
    std::shared_ptr<const GeodesicTopology> topology_;
    std::array<std::unique_ptr<PlanetTerrainAuthority>, kPlanetCount> planets_{};
};

struct SharedLodNode {
    std::array<float, 3> center{};
    float surfaceWidth{};
    std::array<std::uint32_t, 6> neighbors{};
    std::uint32_t parent{std::numeric_limits<std::uint32_t>::max()};
    std::uint32_t level{};
    bool pentagon{};
};

struct SharedLodHierarchy {
    std::uint32_t activeLevels{kSystemCoarseLodLevels};
    std::array<std::uint32_t, kSystemCoarseLodLevels> levelOffsets{};
    std::array<std::uint32_t, kSystemCoarseLodLevels> levelCounts{};
    std::vector<SharedLodNode> nodes;
    std::vector<std::uint32_t> leafToFinestCoarse;

    [[nodiscard]] std::uint64_t memoryBytes() const noexcept {
        return nodes.size() * sizeof(SharedLodNode) +
            leafToFinestCoarse.size() * sizeof(std::uint32_t);
    }
};

[[nodiscard]] inline std::uint32_t locateGeodesicTile(
    const GeodesicTopology& topology, std::array<float, 3> direction) noexcept {
    const float magnitude = std::sqrt(direction[0] * direction[0] +
                                      direction[1] * direction[1] +
                                      direction[2] * direction[2]);
    if (!(magnitude > 0.0F)) direction = {0.0F, 0.0F, 1.0F};
    else for (float& component : direction) component /= magnitude;
    const float longitude = std::atan2(direction[2], direction[0]);
    const float latitude = std::asin(std::clamp(direction[1], -1.0F, 1.0F));
    const float u = longitude / (2.0F * std::numbers::pi_v<float>) + 0.5F;
    const float v = 0.5F - latitude / std::numbers::pi_v<float>;
    const std::uint32_t x = std::min(
        static_cast<std::uint32_t>(u * static_cast<float>(topology.lookupWidth)),
        topology.lookupWidth - 1U);
    const std::uint32_t y = std::min(
        static_cast<std::uint32_t>(v * static_cast<float>(topology.lookupHeight)),
        topology.lookupHeight - 1U);
    std::uint32_t tileIndex = topology.directionLookup[
        x + static_cast<std::size_t>(topology.lookupWidth) * y];
    for (std::uint32_t walk = 0U; walk < 32U; ++walk) {
        const GeodesicTileGpu& tile = topology.tiles[tileIndex];
        auto alignment = [&](std::uint32_t candidate) {
            const auto& center = topology.tiles[candidate].center;
            return direction[0] * center[0] + direction[1] * center[1] +
                   direction[2] * center[2];
        };
        float best = alignment(tileIndex);
        std::uint32_t next = tileIndex;
        for (std::uint32_t slot = 0U; slot < tile.brickInfo[2]; ++slot) {
            const std::uint32_t candidate = slot < 4U
                ? tile.neighborsLow[slot] : tile.neighborsHigh[slot - 4U];
            const float score = alignment(candidate);
            if (score > best) {
                best = score;
                next = candidate;
            }
        }
        if (next == tileIndex) break;
        tileIndex = next;
    }
    return tileIndex;
}

[[nodiscard]] inline SharedLodHierarchy buildSharedLodHierarchy(
    const GeodesicTopology& exactTopology,
    std::uint32_t requestedLevels = kSystemCoarseLodLevels) {
    SharedLodHierarchy result{};
    result.activeLevels = std::clamp(requestedLevels, 1U,
                                     kSystemCoarseLodLevels);
    std::array<GeodesicTopology, kSystemCoarseLodLevels> levels;
    for (std::uint32_t level = 0U; level < result.activeLevels; ++level) {
        levels[level] = GeodesicTopology::build(
            1U << level, 1U, 128U, 64U, 1U, false);
        result.levelOffsets[level] = static_cast<std::uint32_t>(result.nodes.size());
        result.levelCounts[level] = static_cast<std::uint32_t>(levels[level].tiles.size());
        for (const GeodesicTileGpu& tile : levels[level].tiles) {
            SharedLodNode node{};
            node.center = {tile.center[0], tile.center[1], tile.center[2]};
            node.surfaceWidth = tile.bitangent[3];
            node.neighbors.fill(std::numeric_limits<std::uint32_t>::max());
            for (std::uint32_t slot = 0U; slot < tile.brickInfo[2]; ++slot) {
                const std::uint32_t neighbor = slot < 4U
                    ? tile.neighborsLow[slot] : tile.neighborsHigh[slot - 4U];
                node.neighbors[slot] = result.levelOffsets[level] + neighbor;
            }
            node.level = level;
            node.pentagon = tile.brickInfo[3] != 0U;
            result.nodes.push_back(node);
        }
    }
    for (std::uint32_t level = 1U; level < result.activeLevels; ++level) {
        const std::uint32_t offset = result.levelOffsets[level];
        for (std::uint32_t local = 0U; local < result.levelCounts[level]; ++local) {
            const SharedLodNode& node = result.nodes[offset + local];
            result.nodes[offset + local].parent = result.levelOffsets[level - 1U] +
                locateGeodesicTile(levels[level - 1U], node.center);
        }
    }
    result.leafToFinestCoarse.resize(exactTopology.tiles.size());
    const std::uint32_t finest = result.activeLevels - 1U;
    for (std::uint32_t leaf = 0U; leaf < exactTopology.tiles.size(); ++leaf) {
        const auto& center = exactTopology.tiles[leaf].center;
        result.leafToFinestCoarse[leaf] = result.levelOffsets[finest] +
            locateGeodesicTile(levels[finest], {center[0], center[1], center[2]});
    }
    return result;
}

[[nodiscard]] inline std::array<std::vector<Aggregate>, kPlanetCount>
buildPlanetAggregates(const GeodesicTopology& exactTopology,
                      const SharedLodHierarchy& hierarchy,
                      const std::array<std::uint32_t, kPlanetCount>& seeds) {
    std::array<std::vector<Aggregate>, kPlanetCount> result;
    for (std::uint32_t planet = 0U; planet < kPlanetCount; ++planet) {
        result[planet].resize(hierarchy.nodes.size());
        for (Aggregate& aggregate : result[planet]) {
            aggregate.minimumHeight = std::numeric_limits<float>::infinity();
            aggregate.maximumHeight = -std::numeric_limits<float>::infinity();
        }
    }
    for (std::uint32_t leaf = 0U; leaf < exactTopology.tiles.size(); ++leaf) {
        const std::uint32_t node = hierarchy.leafToFinestCoarse[leaf];
        for (std::uint32_t planet = 0U; planet < kPlanetCount; ++planet) {
            const CompactColumnState state = generatedColumn(seeds[planet], leaf);
            Aggregate& aggregate = result[planet][node];
            const float height = static_cast<float>(state.filledLayers) / 32.0F;
            aggregate.minimumHeight = std::min(aggregate.minimumHeight, height);
            aggregate.maximumHeight = std::max(aggregate.maximumHeight, height);
            ++aggregate.materialCounts[(state.material - 1U) % 4U];
        }
    }
    for (std::uint32_t level = hierarchy.activeLevels - 1U;
         level > 0U; --level) {
        const std::uint32_t offset = hierarchy.levelOffsets[level];
        const std::uint32_t count = hierarchy.levelCounts[level];
        for (std::uint32_t local = 0U; local < count; ++local) {
            const std::uint32_t childIndex = offset + local;
            const std::uint32_t parentIndex = hierarchy.nodes[childIndex].parent;
            for (std::uint32_t planet = 0U; planet < kPlanetCount; ++planet) {
                const Aggregate& child = result[planet][childIndex];
                Aggregate& parent = result[planet][parentIndex];
                parent.minimumHeight = std::min(parent.minimumHeight,
                                                child.minimumHeight);
                parent.maximumHeight = std::max(parent.maximumHeight,
                                                child.maximumHeight);
                for (std::uint32_t material = 0U; material < 4U; ++material) {
                    parent.materialCounts[material] += child.materialCounts[material];
                }
                parent.editDirty = parent.editDirty || child.editDirty;
            }
        }
    }
    for (auto& planet : result) {
        for (Aggregate& aggregate : planet) {
            if (!std::isfinite(aggregate.minimumHeight)) {
                aggregate.minimumHeight = 0.0F;
                aggregate.maximumHeight = 1.0F;
            }
            aggregate.maximumProjectedError =
                aggregate.maximumHeight - aggregate.minimumHeight;
        }
    }
    return result;
}

struct StarLight {
    double radius{2.0};
    double radiance{8.0};
    std::array<double, 3> color{1.0, 0.90, 0.72};
    std::uint32_t sampleCount{12U};
    std::array<double, 3> ambientColor{0.09, 0.12, 0.18};
    double ambientStrength{0.22};
};

[[nodiscard]] inline std::array<double, 2> hammersleyDisk(
    std::uint32_t index, std::uint32_t count) noexcept {
    std::uint32_t bits = index;
    bits = (bits << 16U) | (bits >> 16U);
    bits = ((bits & 0x55555555U) << 1U) | ((bits & 0xaaaaaaaaU) >> 1U);
    bits = ((bits & 0x33333333U) << 2U) | ((bits & 0xccccccccU) >> 2U);
    bits = ((bits & 0x0f0f0f0fU) << 4U) | ((bits & 0xf0f0f0f0U) >> 4U);
    bits = ((bits & 0x00ff00ffU) << 8U) | ((bits & 0xff00ff00U) >> 8U);
    const double radial = std::sqrt((static_cast<double>(index) + 0.5) /
                                    static_cast<double>(std::max(count, 1U)));
    const double angle = 2.0 * std::numbers::pi *
        static_cast<double>(bits) / 4294967296.0;
    return {radial * std::cos(angle), radial * std::sin(angle)};
}

[[nodiscard]] inline double boundedAmbient(const StarLight& light) noexcept {
    const double maximumChannel = std::max({light.ambientColor[0],
                                            light.ambientColor[1],
                                            light.ambientColor[2]});
    return std::clamp(maximumChannel * light.ambientStrength, 0.0,
                      std::max(light.radiance, 0.0));
}

[[nodiscard]] inline double sampledDiskVisibility(
    double emitterRadius, double blockerRadius, std::array<double, 2> blockerOffset,
    std::uint32_t sampleCount = 256U) noexcept {
    std::uint32_t visible = 0U;
    const double safeEmitterRadius = std::max(emitterRadius, 0.0);
    for (std::uint32_t index = 0; index < std::max(sampleCount, 1U); ++index) {
        const auto disk = hammersleyDisk(index, std::max(sampleCount, 1U));
        const double x = disk[0] * safeEmitterRadius - blockerOffset[0];
        const double y = disk[1] * safeEmitterRadius - blockerOffset[1];
        visible += x * x + y * y > blockerRadius * blockerRadius ? 1U : 0U;
    }
    return static_cast<double>(visible) /
        static_cast<double>(std::max(sampleCount, 1U));
}

struct BoundSegment {
    double nearDistance{};
    double farDistance{};
    std::uint32_t kind{};
    std::uint32_t index{};
};

[[nodiscard]] inline std::vector<BoundSegment> sortFrontToBack(
    std::vector<BoundSegment> segments) {
    std::stable_sort(segments.begin(), segments.end(),
        [](const BoundSegment& a, const BoundSegment& b) {
            if (a.nearDistance != b.nearDistance) return a.nearDistance < b.nearDistance;
            if (a.kind != b.kind) return a.kind < b.kind;
            return a.index < b.index;
        });
    return segments;
}

} // namespace voxel::system_lab
