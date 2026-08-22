#include "planet/geodesic_topology.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    try {
        for (const std::uint32_t frequency : {1U, 2U, 4U, 8U, 16U, 32U}) {
            const voxel::GeodesicTopology topology =
                voxel::GeodesicTopology::build(frequency, 8, 32, 16);
            topology.validate();
            if (topology.tiles.size() != 10U * frequency * frequency + 2U ||
                topology.pentagonCount != 12U ||
                topology.hexagonCount + topology.pentagonCount != topology.tiles.size() ||
                topology.brickCellCapacity() != topology.tiles.size() * 8U ||
                topology.bvhNodes.size() != topology.tiles.size() * 2U - 1U ||
                topology.columnStates.size() != topology.tiles.size()) {
                return 1;
            }
            for (const auto& tile : topology.tiles) {
                const float surfaceCellWidth = tile.center[3] * 2.0F;
                const float expectedDepth = std::min(
                    surfaceCellWidth * 0.5F * static_cast<float>(topology.brickResolution),
                    0.25F);
                if (std::abs(tile.tangent[3] - expectedDepth) > 2e-6F ||
                    std::abs(tile.bitangent[3] - surfaceCellWidth) > 2e-6F ||
                    tile.tangent[3] <= 0.0F || tile.tangent[3] > 0.25F) {
                    throw std::runtime_error("Geodesic radial voxel aspect ratio is invalid");
                }
                if (expectedDepth < 0.249F) {
                    const float layerHeight = tile.tangent[3] /
                                              static_cast<float>(topology.brickResolution);
                    if (std::abs(layerHeight / surfaceCellWidth - 0.5F) > 2e-5F) {
                        throw std::runtime_error(
                            "Geodesic radial voxel height is not half its surface width");
                    }
                }
            }
        }
        bool rejectedOversizedMask = false;
        try {
            (void)voxel::GeodesicTopology::build(1U, 33U, 8U, 4U);
        } catch (const std::invalid_argument&) {
            rejectedOversizedMask = true;
        }
        if (!rejectedOversizedMask) {
            return 1;
        }
        const voxel::GeodesicTopology productionTopology =
            voxel::GeodesicTopology::build(16U, 8U, 32U, 16U, 256U, false);
        productionTopology.validate();
        const auto& fullTile = productionTopology.tiles.front();
        const voxel::GeodesicRayTileGpu queryTile =
            voxel::compactGeodesicRayTile(fullTile);
        if (voxel::kGeodesicRayTileLayoutVersion != 1U ||
            queryTile.center != fullTile.center ||
            queryTile.neighborsLow != fullTile.neighborsLow ||
            queryTile.query[0] != fullTile.neighborsHigh[0] ||
            queryTile.query[1] != fullTile.neighborsHigh[1] ||
            std::bit_cast<float>(queryTile.query[2]) != fullTile.tangent[3] ||
            std::bit_cast<float>(queryTile.query[3]) != fullTile.bitangent[3]) {
            throw std::runtime_error("Compact exact ray-query record changed topology data");
        }
        if (productionTopology.referenceTraversalBuilt ||
            !productionTopology.bvhNodes.empty() ||
            !productionTopology.wideNodes.empty() ||
            !productionTopology.wideItems.empty() ||
            productionTopology.bedrockRadius <= 0.0F ||
            productionTopology.bedrockRadius >= 1.0F) {
            throw std::runtime_error("Production topology retained diagnostic acceleration data");
        }
        std::cout << "Geodesic topology invariants passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Topology gate failed: " << error.what() << '\n';
        return 1;
    }
}
