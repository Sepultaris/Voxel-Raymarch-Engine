#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <functional>
#include <limits>
#include <string_view>
#include <vector>

namespace voxel {

constexpr std::uint32_t kInvalidTile = 0xffffffffU;
constexpr std::uint32_t kInvalidPage = 0xffffffffU;
constexpr std::uint32_t kPageResident = 1U;
constexpr std::uint32_t kPageDirty = 2U;

struct alignas(16) GeodesicTileGpu {
    // center.w is tangent half-extent in planet-radius units.
    std::array<float, 4> center{};
    // tangent.w is the brick's radial depth in planet-radius units.
    std::array<float, 4> tangent{};
    // bitangent.w is the approximate local surface-cell width.
    std::array<float, 4> bitangent{};
    std::array<std::uint32_t, 4> neighborsLow{};
    std::array<std::uint32_t, 4> neighborsHigh{};
    // offset in uint cells, brick edge length, neighbor count, pentagon flag
    std::array<std::uint32_t, 4> brickInfo{};
};
static_assert(sizeof(GeodesicTileGpu) == 96);

// Scale-lab ray-query layout. Tangent/bitangent basis vectors are not consumed
// by any GPU planetary query; only their scalar radial depth and surface width
// are required. The fixed f1024/l32 lab can also derive the brick offset from
// the tile index and infer the 5/6-neighbor count from the final neighbor.
// Keeping the center, all neighbors, and both exact metric scalars in one
// 48-byte record halves random tile traffic without reducing topology data.
struct alignas(16) GeodesicRayTileGpu {
    std::array<float, 4> center{};
    std::array<std::uint32_t, 4> neighborsLow{};
    // neighbor 4, neighbor 5 (invalid for pentagons), radial-depth float bits,
    // local surface-width float bits.
    std::array<std::uint32_t, 4> query{};
};
static_assert(sizeof(GeodesicRayTileGpu) == 48);
inline constexpr std::uint32_t kGeodesicRayTileLayoutVersion = 1U;

[[nodiscard]] inline GeodesicRayTileGpu compactGeodesicRayTile(
    const GeodesicTileGpu& source) noexcept {
    return {
        source.center,
        source.neighborsLow,
        {source.neighborsHigh[0], source.neighborsHigh[1],
         std::bit_cast<std::uint32_t>(source.tangent[3]),
         std::bit_cast<std::uint32_t>(source.bitangent[3])}};
}

struct alignas(16) GeodesicBvhNodeGpu {
    std::array<float, 4> minimum{};
    std::array<float, 4> maximum{};
    // Internal: left, right, 0, 0. Leaf: tile index, 0, 1, 0.
    std::array<std::uint32_t, 4> data{};
};
static_assert(sizeof(GeodesicBvhNodeGpu) == 48);

// Stackless, preorder wide-BVH node.  The 64-bit mask deliberately matches
// the mask/popcount layout used by modern sparse voxel trees while the AABB
// keeps the hierarchy conservative for irregular geodesic cells.
struct alignas(16) GeodesicWideNodeGpu {
    std::array<float, 4> minimum{};
    std::array<float, 4> maximum{};
    // Internal: first child, child count, escape node, 0.
    // Leaf: first item, item count, escape node, 1.
    std::array<std::uint32_t, 4> data{};
    // Active child/item bits, low word then high word, followed by padding.
    std::array<std::uint32_t, 4> mask{};
};
static_assert(sizeof(GeodesicWideNodeGpu) == 64);

struct alignas(16) GeodesicColumnStateGpu {
    // occupancy mask, occupied layer count, top material in z[7:0]
    // (the f1024 lab caches quantized cap AO in z[15:8]), reserved
    std::array<std::uint32_t, 4> data{};
};
static_assert(sizeof(GeodesicColumnStateGpu) == 16);

struct alignas(16) GeodesicPageTableEntryGpu {
    // physical page, reuse generation, flags, GPU request-deduplication stamp
    std::array<std::uint32_t, 4> data{kInvalidPage, 0U, 0U, 0U};
};
static_assert(sizeof(GeodesicPageTableEntryGpu) == 16);

struct GeodesicTopology {
    using ProgressCallback = std::function<void(
        std::string_view, std::uint64_t, std::uint64_t)>;
    std::uint32_t frequency{};
    std::uint32_t brickResolution{};
    std::uint32_t pentagonCount{};
    std::uint32_t hexagonCount{};
    std::uint32_t triangleCount{};
    std::vector<GeodesicTileGpu> tiles;
    std::vector<std::uint32_t> directionLookup;
    std::vector<std::uint32_t> brickCells;
    std::vector<GeodesicBvhNodeGpu> bvhNodes;
    std::vector<GeodesicWideNodeGpu> wideNodes;
    std::vector<std::uint32_t> wideItems;
    std::vector<GeodesicColumnStateGpu> columnStates;
    std::vector<GeodesicPageTableEntryGpu> pageTable;
    std::uint32_t lookupWidth{};
    std::uint32_t lookupHeight{};
    float outerBoundingRadius{1.0F};
    // A radius strictly inside every column's immutable layer-zero prism.
    // Rays crossing this sphere cannot legitimately miss planetary terrain.
    float bedrockRadius{};
    bool referenceTraversalBuilt{};
    std::uint32_t nextPageGeneration{1U};

    // Total virtual cells addressable by all surface columns.
    [[nodiscard]] std::uint64_t brickCellCapacity() const noexcept;
    // Cells currently backed by physical page-pool storage.
    [[nodiscard]] std::uint64_t residentBrickCellCapacity() const noexcept;
    [[nodiscard]] std::uint32_t residentPageCount() const noexcept;
    [[nodiscard]] std::uint32_t remapResidentPage(
        std::uint32_t physicalPageIndex, std::uint32_t tileIndex);
    [[nodiscard]] std::uint32_t remapResidentPageFromOwner(
        std::uint32_t physicalPageIndex, std::uint32_t tileIndex,
        std::uint32_t currentOwnerTile);
    [[nodiscard]] bool remapResidentPageIfGeneration(
        std::uint32_t physicalPageIndex, std::uint32_t tileIndex,
        std::uint32_t expectedGeneration, std::uint32_t& evictedTile);
    void validate() const;

    [[nodiscard]] static GeodesicTopology build(
        std::uint32_t frequency,
        std::uint32_t brickResolution,
        std::uint32_t lookupWidth = 512,
        std::uint32_t lookupHeight = 256,
        std::uint32_t residentPageCapacity = std::numeric_limits<std::uint32_t>::max(),
        bool buildReferenceTraversal = true,
        const ProgressCallback& progress = {});
};

} // namespace voxel
