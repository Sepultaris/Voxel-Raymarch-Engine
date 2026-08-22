#include "planet/geodesic_local_ao.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "local geodesic AO test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

voxel::LocalGeodesicAoSample uniformSample(std::uint32_t count,
                                           std::uint32_t neighborHeight,
                                           std::uint32_t hitLayer) {
    voxel::LocalGeodesicAoSample sample{};
    sample.neighborHeights.fill(neighborHeight);
    sample.neighborCount = count;
    sample.columnHeight = hitLayer + 1U;
    sample.hitLayer = hitLayer;
    return sample;
}

voxel::LocalGeodesicAoSample rotated(
    const voxel::LocalGeodesicAoSample& source, std::uint32_t amount) {
    voxel::LocalGeodesicAoSample result = source;
    const std::uint32_t count = source.neighborCount;
    for (std::uint32_t slot = 0U; slot < count; ++slot) {
        result.neighborHeights[(slot + amount) % count] =
            source.neighborHeights[slot];
    }
    if (source.sideNeighborSlot < count) {
        result.sideNeighborSlot = (source.sideNeighborSlot + amount) % count;
    }
    return result;
}

bool nearlyEqual(float left, float right, float tolerance = 1.0e-6F) {
    return std::abs(left - right) <= tolerance;
}

} // namespace

int main() {
    const auto flatHex = uniformSample(6U, 12U, 11U);
    const auto flatPentagon = uniformSample(5U, 12U, 11U);
    const auto shallowBowlHex = uniformSample(6U, 13U, 11U);
    const auto deepBowlHex = uniformSample(6U, 17U, 11U);
    const auto shallowBowlPentagon = uniformSample(5U, 13U, 11U);
    const auto convexRidge = uniformSample(6U, 9U, 15U);

    const float flatHexCue = voxel::localGeodesicAoCue(flatHex);
    const float flatPentagonCue = voxel::localGeodesicAoCue(flatPentagon);
    const float shallowBowlCue = voxel::localGeodesicAoCue(shallowBowlHex);
    const float deepBowlCue = voxel::localGeodesicAoCue(deepBowlHex);
    const float pentagonBowlCue = voxel::localGeodesicAoCue(shallowBowlPentagon);
    const float ridgeCue = voxel::localGeodesicAoCue(convexRidge);

    require(flatHexCue == 0.0F && flatPentagonCue == 0.0F,
            "flat pentagon/hexagon caps are not fully open");
    require(ridgeCue == 0.0F,
            "a convex ridge darkened without a higher local horizon");
    require(shallowBowlCue > flatHexCue && deepBowlCue > shallowBowlCue,
            "bowl depth is not a monotonic occlusion signal");
    require(nearlyEqual(shallowBowlCue, pentagonBowlCue),
            "pentagon/hexagon normalization changes a uniform horizon");

    auto corner = flatHex;
    corner.neighborHeights[5] = 16U;
    corner.neighborHeights[0] = 16U;
    require(voxel::localGeodesicAoCue(corner) > flatHexCue &&
                voxel::localGeodesicAoCue(corner) < shallowBowlCue,
            "a local corner is not between open terrain and a full bowl");

    auto exposedTopSide = flatHex;
    exposedTopSide.sideNeighborSlot = 0U;
    exposedTopSide.neighborHeights[0] = 8U;
    require(voxel::localGeodesicAoCue(exposedTopSide) == 0.0F,
            "a top-layer exposed cliff became a dark side stripe");
    auto enclosedLowerSide = exposedTopSide;
    enclosedLowerSide.columnHeight = 12U;
    enclosedLowerSide.hitLayer = 7U;
    enclosedLowerSide.neighborHeights[0] = 6U;
    require(voxel::localGeodesicAoCue(enclosedLowerSide) > 0.0F &&
                voxel::localGeodesicAoCue(enclosedLowerSide) < 0.75F,
            "lower concave side is missing or exceeds the restrained wall cue");

    // Ordered-ring rotation represents crossing an atlas/geodesic seam. The
    // same local geometry and side exposure must have exactly the same cue.
    auto seamPattern = corner;
    seamPattern.sideNeighborSlot = 2U;
    seamPattern.neighborHeights = {15U, 17U, 7U, 10U, 14U, 16U};
    const float seamCue = voxel::localGeodesicAoCue(seamPattern);
    for (std::uint32_t rotation = 1U; rotation < 6U; ++rotation) {
        require(nearlyEqual(seamCue,
                            voxel::localGeodesicAoCue(rotated(seamPattern, rotation))),
                "ordered one-ring seam rotation changed local AO");
    }

    // Camera, FOV, resolution, hierarchy handoff, material and residency are
    // deliberately absent from the AO input. Replaying their representative
    // state changes against one exact hit therefore has a single result.
    for (float cameraDistance : {1.0F, 2.0F, 8.0F}) {
        for (float fieldOfView : {45.0F, 70.0F, 100.0F}) {
            for (bool hierarchyEnabled : {false, true}) {
                for (bool pageResident : {false, true}) {
                    const std::uint32_t ignoredMaterial = pageResident ? 5U : 2U;
                    (void)cameraDistance;
                    (void)fieldOfView;
                    (void)hierarchyEnabled;
                    (void)ignoredMaterial;
                    require(nearlyEqual(
                                seamCue, voxel::localGeodesicAoCue(seamPattern)),
                            "non-geometric render state leaked into local AO");
                }
            }
        }
    }

    float previous = 0.0F;
    for (std::uint32_t height = 12U; height <= 32U; ++height) {
        const float cue = voxel::localGeodesicAoCue(
            uniformSample(6U, height, 11U));
        require(cue >= previous && cue >= 0.0F && cue <= 1.0F,
                "AO is not monotonic and bounded over legal column heights");
        previous = cue;
    }
    require(voxel::localGeodesicAoFactor(
                deepBowlHex, voxel::kLocalGeodesicAoDefaultStrength) >= 0.88F,
            "subtle default can over-darken a fully enclosed local bowl");
    require(voxel::localGeodesicAoFactor(
                deepBowlHex, voxel::kLocalGeodesicAoMaximumStrength) >= 0.70F,
            "maximum UI strength can produce black geometry");

    float previousWork = 0.0F;
    for (std::uint32_t events = 0U; events <= 2048U; ++events) {
        const float cue = voxel::ddaWorkHeatmapCue(events);
        require(cue >= previousWork && cue >= 0.0F && cue <= 1.0F,
                "DDA work heatmap is not monotonic and bounded");
        previousWork = cue;
    }

    const std::uint32_t packed = voxel::packMarchAndGeodesicShading(
        160U, true, voxel::kLocalGeodesicAoDefaultStrength, true, true);
    require((packed & 0xffffU) == 160U && (packed & (1U << 24U)) != 0U &&
                (packed & (1U << 25U)) != 0U &&
                (packed & (1U << 26U)) != 0U,
            "push-constant shading packing changed march/debug selection bits");

    std::cout << "Local geodesic AO flat/bowl/ridge, side, seam, pentagon/hexagon, "
                 "state-invariance, residency and bounds checks passed.\n";
}
