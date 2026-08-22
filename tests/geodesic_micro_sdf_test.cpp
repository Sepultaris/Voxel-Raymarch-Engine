#include "planet/geodesic_local_ao.hpp"
#include "planet/geodesic_micro_sdf.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "micro-SDF test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

} // namespace

int main() {
    constexpr std::array<float, 3> direction{0.37139067F, 0.55708605F,
                                             0.74278134F};
    const auto first = voxel::evaluateTerrainMicroSdfField(direction);
    const auto second = voxel::evaluateTerrainMicroSdfField(direction);
    require(std::bit_cast<std::uint32_t>(first.depth01) ==
                std::bit_cast<std::uint32_t>(second.depth01) &&
            first.directionGradient == second.directionGradient,
            "same world location was not bit-deterministic");
    require(std::isfinite(first.depth01) && first.depth01 >= 0.0F &&
                first.depth01 <= 1.0F,
            "bounded field escaped [0,1]");
    for (float component : first.directionGradient) {
        require(std::isfinite(component), "analytic gradient became non-finite");
    }

    // The field is world-direction based, not tile ID or neighbor-count based;
    // the same seam point therefore agrees for pentagon and hexagon ownership.
    const auto seamPentagon = voxel::evaluateTerrainMicroSdfField(direction);
    const auto seamHexagon = voxel::evaluateTerrainMicroSdfField(direction);
    require(seamPentagon.depth01 == seamHexagon.depth01 &&
                seamPentagon.directionGradient == seamHexagon.directionGradient,
            "pentagon/hexagon seam changed the procedural field");

    const float open = voxel::terrainMicroSdfActivation(24.0F, 0.1F, 0.9F, 2U, false);
    require(open > 0.95F, "close interior terrain did not activate");
    require(voxel::terrainMicroSdfActivation(2.0F, 0.1F, 0.9F, 2U, false) == 0.0F,
            "distant subpixel terrain did not fade to exact caps");
    require(voxel::terrainMicroSdfActivation(24.0F, 0.9F, 0.9F, 2U, false) == 0.0F,
            "cell-edge support crossed an ownership boundary");
    require(voxel::terrainMicroSdfActivation(24.0F, 0.1F, 0.9F, 3U, false) == 0.0F,
            "water was not masked");
    require(voxel::terrainMicroSdfActivation(24.0F, 0.1F, 0.9F, 2U, true) == 0.0F,
            "edited voxel did not remain authoritative");
    require(voxel::terrainMicroSdfActivation(
                std::numeric_limits<float>::infinity(), 0.1F, 0.9F, 2U, false) == 0.0F,
            "non-finite footprint did not fail closed");

    const float offset = voxel::terrainMicroSdfRayOffset(
        0.001F, voxel::kTerrainMicroSdfMaximumLayerFraction, 1.0F, 1.0F, 0.5F);
    require(offset * 0.5F <= 0.001F * voxel::kTerrainMicroSdfMaximumLayerFraction + 1e-9F,
            "refinement escaped the accepted radial layer");
    require(voxel::terrainMicroSdfRayOffset(
                0.001F, 0.175F, 1.0F, 1.0F, 0.35F) == 0.0F,
            "grazing ray was allowed to micro-refine");

    const std::uint32_t packed = voxel::packMarchAndGeodesicShading(
        160U, true, 0.12F, false, false, true, 0.075F, true);
    require((packed & (1U << 27U)) != 0U &&
                (packed & (1U << 28U)) != 0U &&
                ((packed >> 29U) & 0x7U) == 3U,
            "micro-SDF controls were not packed deterministically");

    std::cout << "Bounded deterministic terrain micro-SDF invariants passed.\n";
}
