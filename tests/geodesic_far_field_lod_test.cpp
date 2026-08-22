#include "planet/geodesic_far_field_lod.hpp"
#include "planet/geodesic_topology.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "far-field LOD test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool inTwoRing(const voxel::GeodesicTopology& topology,
               std::uint32_t center, std::uint32_t candidate) {
    if (center == candidate) {
        return true;
    }
    const auto neighbor = [&](const voxel::GeodesicTileGpu& tile, std::uint32_t slot) {
        return slot < 4U ? tile.neighborsLow[slot] : tile.neighborsHigh[slot - 4U];
    };
    const auto& tile = topology.tiles[center];
    for (std::uint32_t firstSlot = 0U; firstSlot < tile.brickInfo[2]; ++firstSlot) {
        const std::uint32_t first = neighbor(tile, firstSlot);
        if (first == candidate) {
            return true;
        }
        const auto& firstTile = topology.tiles[first];
        for (std::uint32_t secondSlot = 0U;
             secondSlot < firstTile.brickInfo[2]; ++secondSlot) {
            if (neighbor(firstTile, secondSlot) == candidate) {
                return true;
            }
        }
    }
    return false;
}

} // namespace

int main() {
    require(voxel::farFieldNodeCount(1024U, 512U) == 698'368U,
            "hierarchy node count changed");
    require(voxel::farFieldLevelOffset(1024U, 512U, 1U) == 524'288U,
            "level-one offset is not deterministic");

    const float below = std::nextafter(0.70F, 0.0F);
    const float above = std::nextafter(0.70F, 1.0F);
    const auto belowLevel = voxel::farFieldLodForProjectedPixels(below);
    const auto aboveLevel = voxel::farFieldLodForProjectedPixels(above);
    require(belowLevel == 1U && aboveLevel == 0U && belowLevel - aboveLevel == 1U,
            "ULP LOD transition skips a level");
    require(voxel::halfOpenWrappedCoordinate(-std::numeric_limits<double>::epsilon(), 1024U)
                == 1023U,
            "negative seam does not map to the final half-open owner");
    require(voxel::halfOpenWrappedCoordinate(1.0, 1024U) == 0U,
            "positive seam does not wrap to the canonical owner");

    voxel::FarFieldHeightRange range{};
    for (const std::uint32_t height : {4U, 17U, 9U, 22U, 11U}) {
        range.include(height);
    }
    range.conservativelyPad(2U);
    require(range.minimum == 2U && range.maximum == 24U,
            "aggregate range is not conservative");

    auto topology = voxel::GeodesicTopology::build(16U, 32U, 64U, 32U, 128U, false);
    const std::uint32_t center = 0U;
    const std::uint32_t first = topology.tiles[center].neighborsLow[0];
    const std::uint32_t second = topology.tiles[first].neighborsLow[1];
    require(inTwoRing(topology, center, center) && inTwoRing(topology, center, first) &&
                inTwoRing(topology, center, second),
            "F6 full-resolution pin does not cover its two-ring");

    topology.columnStates[first].data[3] = 1U;
    const std::uint32_t oldGeneration = topology.pageTable[first].data[1];
    std::uint32_t evicted = voxel::kInvalidTile;
    (void)topology.remapResidentPageIfGeneration(
        0U, first, oldGeneration, evicted);
    require(topology.columnStates[first].data[3] == 1U,
            "sparse eviction discarded the edit-refinement override");
}
