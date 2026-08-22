#include "planet/geodesic_material.hpp"
#include "render/geodesic_shading_lod.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main() {
    try {
        for (std::uint32_t filled = 1U; filled <= 32U; ++filled) {
            for (std::uint32_t surface = 1U; surface <= 5U; ++surface) {
                for (std::uint32_t layer = 0U; layer < 32U; ++layer) {
                    const std::uint32_t coarse = voxel::geodesicMaterialForLayer(
                        filled, surface, layer);
                    const std::uint32_t resident = layer >= filled
                                                       ? 0U
                                                       : (layer + 3U < filled ? 5U : surface);
                    require(coarse == resident,
                            "Resident and nonresident layer materials disagree");
                }
            }
        }

        constexpr float productionLayerHeight = 0.0015F;
        const float distantPixels = voxel::geodesicProjectedCellPixels(
            productionLayerHeight, 1.5F, 768.0F);
        const float closePixels = voxel::geodesicProjectedCellPixels(
            productionLayerHeight, 0.025F, 768.0F);
        const float distantSide = voxel::geodesicSideDetailWeight(
            0.0F, distantPixels, distantPixels, 1.0F);
        const float closeSide = voxel::geodesicSideDetailWeight(
            0.0F, closePixels, closePixels, 1.0F);
        const float distantCap = voxel::geodesicSideDetailWeight(
            1.0F, distantPixels, distantPixels, 1.0F);
        require(distantSide < 0.05F,
                "Subpixel distant side faces retain unstable full contrast");
        require(closeSide > 0.999F,
                "Resolved nearby cliffs are incorrectly filtered");
        require(distantCap > 0.999F,
                "Distance filtering incorrectly changes terrain caps");
        const float equalLayerMicroStrip = voxel::geodesicSideDetailWeight(
            0.0F, 0.08F, closePixels, 1.0F);
        require(equalLayerMicroStrip < 0.001F,
                "Microscopic equal-layer side strips retain stippled contrast");
        const float tallEdgeOnCliff = voxel::geodesicSideDetailWeight(
            0.0F, closePixels, 12.0F, 0.02F);
        require(tallEdgeOnCliff < 0.001F,
                "Tall but edge-on radial walls retain dotted full contrast");
        const float tallFaceOnCliff = voxel::geodesicSideDetailWeight(
            0.0F, closePixels, 12.0F, 1.0F);
        require(tallFaceOnCliff > 0.999F,
                "Resolved nearby face-on cliffs are incorrectly filtered");

        std::cout << "Residency-equivalent materials and geodesic side-face LOD passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Geodesic visual consistency gate failed: " << error.what() << '\n';
        return 1;
    }
}
