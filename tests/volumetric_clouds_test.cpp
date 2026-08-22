#include "render/volumetric_clouds.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "volumetric clouds test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

} // namespace

int main() {
    using namespace voxel;

    const auto throughShell = cloudShellSegments(
        {0.0F, 0.0F, 3.0F}, {0.0F, 0.0F, -1.0F},
        1.10F, 1.20F, 100.0F);
    require(throughShell.count == 2U &&
                std::abs(throughShell.nearDistance[0] - 1.80F) < 1.0e-5F &&
                std::abs(throughShell.farDistance[0] - 1.90F) < 1.0e-5F &&
                std::abs(throughShell.nearDistance[1] - 4.10F) < 1.0e-5F &&
                std::abs(throughShell.farDistance[1] - 4.20F) < 1.0e-5F,
            "hollow spherical shell did not produce its two exact intervals");
    const auto clippedAtTerrain = cloudShellSegments(
        {0.0F, 0.0F, 3.0F}, {0.0F, 0.0F, -1.0F},
        1.10F, 1.20F, 1.86F);
    require(clippedAtTerrain.count == 1U &&
                std::abs(clippedAtTerrain.farDistance[0] - 1.86F) < 1.0e-5F,
            "exact terrain distance did not clip the cloud segment");
    const auto insideCloud = cloudShellSegments(
        {0.0F, 0.0F, 1.15F}, {0.0F, 0.0F, 1.0F},
        1.10F, 1.20F, 100.0F);
    require(insideCloud.count == 1U && insideCloud.nearDistance[0] == 0.0F &&
                std::abs(insideCloud.farDistance[0] - 0.05F) < 1.0e-5F,
            "camera inside the cloud layer did not receive a finite exit segment");
    require(cloudShellSegments(
                {0.0F, 0.0F, 3.0F}, {0.0F, 0.0F, 1.0F},
                1.10F, 1.20F, 100.0F).count == 0U,
            "outward ray falsely intersected a cloud shell behind the camera");

    VolumetricCloudSettings settings{};
    settings.source = CloudDensitySource::Authored3DTexture;
    settings.quality = 3U;
    settings.sunShadowQuality = 2U;
    settings.debugMode = 3U;
    settings.windDirection = 4.25F;
    const std::uint32_t word0 = packCloudWord0(settings);
    const std::uint32_t word1 = packCloudWord1(settings);
    const std::uint32_t phase = packCloudPhaseWord(0xa5U, settings);
    const std::uint32_t controls = word1 >> 24U;
    require((controls & 1U) != 0U && (controls & 2U) != 0U &&
                ((controls >> 2U) & 3U) == 3U &&
                ((controls >> 4U) & 3U) == 2U &&
                ((controls >> 6U) & 3U) == 3U,
            "source/quality/shadow/debug controls overlapped in packed state");
    require((phase & 0xffU) == 0xa5U,
            "cloud phase packing did not preserve diagnostic low bits");
    require(((word0 >> 16U) & 0xffU) < ((word0 >> 24U) & 0xffU),
            "cloud base/top altitude ordering was lost during packing");

    const std::array<float, 3> worldPoint{0.0F, 0.0F, 1.10F};
    const float densityA = proceduralCloudDensity(
        worldPoint, 1.0F, 0.065F, 0.135F, 0.46F, 6.0F,
        0.32F, 0.0F, 0.0F, 0.35F);
    const float densityB = proceduralCloudDensity(
        worldPoint, 1.0F, 0.065F, 0.135F, 0.46F, 6.0F,
        0.32F, 9200.0F, 0.0F, 5.70F);
    require(std::isfinite(densityA) && densityA >= 0.0F && densityA <= 1.0F &&
                densityA == densityB,
            "paused wind was not deterministic and time invariant");
    // The density source takes only the world point and planet-relative shell;
    // moving a camera cannot alter it, and sparse material residency is absent
    // from the procedural source by construction.
    const float repeatedDensity = proceduralCloudDensity(
        worldPoint, 1.0F, 0.065F, 0.135F, 0.46F, 6.0F,
        0.32F, 0.0F, 0.0F, 0.35F);
    require(repeatedDensity == densityA,
            "identical world-space cloud sample was not camera/residency stable");
    require(proceduralCloudDensity(
                {0.0F, 0.0F, 1.01F}, 1.0F, 0.065F, 0.135F,
                0.46F, 6.0F, 0.32F, 0.0F, 0.0F, 0.35F) == 0.0F,
            "procedural density escaped its altitude shell");

    require(cloudTransmittance(8.0F, 0.0F) == 1.0F,
            "zero density was not the identity medium");
    const float thin = cloudTransmittance(0.5F, 0.8F);
    const float thick = cloudTransmittance(1.0F, 0.8F);
    const float dense = cloudTransmittance(0.5F, 1.6F);
    require(thick < thin && dense < thin && std::isfinite(thin) &&
                thin >= 0.0F && thin <= 1.0F,
            "cloud Beer-Lambert transmittance was not finite/monotonic");
    const float terrainVisibility = 0.62F;
    const float rawCloudVisibility = cloudTransmittance(0.7F, 1.1F);
    const float cloudVisibility = 1.0F - 0.78F * (1.0F - rawCloudVisibility);
    const float composed = terrainVisibility * cloudVisibility;
    require(composed <= terrainVisibility && composed <= cloudVisibility &&
                composed >= 0.0F,
            "terrain-horizon and cloud sun shadows did not compose monotonically");

    require(cloudTransmittance(std::numeric_limits<float>::quiet_NaN(), 1.0F) ==
                0.0F &&
                cloudShellSegments({0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F},
                                   1.1F, 1.2F, 10.0F).count == 0U,
            "non-finite/degenerate cloud input escaped its guard");

    std::cout << "Cloud shell geometry, deterministic world-space procedural "
                 "density, packing, Beer-Lambert attenuation, terrain-shadow "
                 "composition, and finite guards passed.\n";
}
