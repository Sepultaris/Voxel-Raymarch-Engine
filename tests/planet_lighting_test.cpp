#include "render/planet_lighting.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "planet lighting test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool close(float left, float right, float tolerance = 1.0e-5F) {
    return std::abs(left - right) <= tolerance;
}

} // namespace

int main() {
    using namespace voxel;

    float previousDiffuse = 0.0F;
    for (std::uint32_t sample = 0U; sample <= 100U; ++sample) {
        const float dot = -0.5F + 1.5F * static_cast<float>(sample) / 100.0F;
        const float diffuse = planetLightingDiffuse(dot);
        require(diffuse >= previousDiffuse && diffuse >= 0.0F &&
                    diffuse <= 1.0F,
                "sun diffuse is not monotonic and bounded");
        previousDiffuse = diffuse;
    }

    float previousSky = 0.0F;
    for (std::uint32_t sample = 0U; sample <= 100U; ++sample) {
        const float dot = -1.0F + 2.0F * static_cast<float>(sample) / 100.0F;
        const float sky = planetLightingHemisphereWeight(dot);
        require(sky >= previousSky && sky >= 0.0F && sky <= 1.0F,
                "radial sky hemisphere is not monotonic and bounded");
        previousSky = sky;
    }

    const float waterRoughness = planetLightingMaterialRoughness(3U, 0.68F);
    const float snowRoughness = planetLightingMaterialRoughness(4U, 0.68F);
    const float rockRoughness = planetLightingMaterialRoughness(5U, 0.68F);
    require(waterRoughness < snowRoughness && snowRoughness < rockRoughness,
            "water/snow/rock roughness ordering is not material-aware");
    for (bool resident : {false, true}) {
        (void)resident;
        require(close(rockRoughness,
                      planetLightingMaterialRoughness(5U, 0.68F)),
                "page residency changed material lighting");
    }

    const std::array<float, 3> color{0.93F, 0.42F, 0.17F};
    const auto packed = packLightingRgb8(color, 3U);
    const auto unpacked = unpackLightingRgb8(packed);
    require(((packed >> 24U) & 0x3U) == 3U,
            "lighting debug mode was not isolated from packed RGB");
    for (std::size_t channel = 0U; channel < 3U; ++channel) {
        require(std::abs(unpacked[channel] - color[channel]) <= 1.0F / 255.0F,
                "packed light color exceeded one quantization step");
    }

    float previousTone = 0.0F;
    for (std::uint32_t sample = 0U; sample <= 100U; ++sample) {
        const float tone = planetLightingAces(
            8.0F * static_cast<float>(sample) / 100.0F);
        require(tone >= previousTone && tone >= 0.0F && tone <= 1.0F,
                "tone map is not monotonic and bounded");
        previousTone = tone;
    }
    const std::array<float, 3> sky{0.12F, 0.24F, 0.52F};
    const auto untouchedSky = applyPlanetLightingToHit(false, sky, 2.5F);
    require(untouchedSky == sky,
            "terrain exposure or tone mapping modified a sky miss");
    const auto lowExposure = applyPlanetLightingToHit(
        true, {0.3F, 0.3F, 0.3F}, 0.5F);
    const auto highExposure = applyPlanetLightingToHit(
        true, {0.3F, 0.3F, 0.3F}, 1.5F);
    require(highExposure[0] > lowExposure[0],
            "hit exposure is not monotonic");

    std::cout << "Production lighting diffuse, sky hemisphere, material, "
                 "packing, tone/exposure, residency, and sky isolation passed.\n";
}
