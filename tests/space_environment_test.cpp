#include "render/space_environment.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "space environment test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

} // namespace

int main() {
    using namespace voxel;

    constexpr std::array<float, 3> worldRay{0.2F, 0.3F, 0.9327379F};
    constexpr std::array<float, 3> localRay{-0.4F, 0.1F, 0.9110434F};
    const auto worldLocked = spaceEnvironmentDirection(
        worldRay, localRay, false);
    const auto cameraAligned = spaceEnvironmentDirection(
        worldRay, localRay, true);
    require(worldLocked == worldRay && cameraAligned == localRay,
            "world/camera orientation mode selected the wrong ray");

    // Translation is deliberately not an input to the infinitely distant
    // star atlas. Replaying arbitrary camera positions yields one direction.
    for (const std::array<float, 3>& ignoredPosition : {
             std::array<float, 3>{0.0F, 0.0F, 2.0F},
             std::array<float, 3>{1000.0F, -30.0F, 8.0F},
             std::array<float, 3>{-4.0F, 2.0F, 0.1F}}) {
        (void)ignoredPosition;
        require(spaceEnvironmentDirection(worldRay, localRay, false) ==
                    worldLocked,
                "camera translation introduced star parallax");
    }

    SpaceEnvironmentSettings settings{};
    settings.density = 0.42F;
    settings.enabled = true;
    settings.cameraAligned = true;
    settings.debug = true;
    const std::uint32_t packed = packSpaceEnvironmentFlags(settings);
    require((packed & (1U << 24U)) != 0U &&
                (packed & (1U << 25U)) != 0U &&
                (packed & (1U << 26U)) != 0U &&
                std::abs(spaceEnvironmentDensity(packed) - 0.42F) <=
                    1.0F / 255.0F,
            "space environment controls were not packed independently");

    require(spaceEnvironmentHorizonVisibility(0.079F, 1.0F) == 0.0F &&
                spaceEnvironmentHorizonVisibility(0.151F, 1.0F) == 1.0F,
            "planet silhouette exclusion does not fully suppress nearby stars");
    float previousVisibility = 0.0F;
    for (std::uint32_t sample = 0U; sample <= 100U; ++sample) {
        const float clearance = 0.06F + 0.11F *
            static_cast<float>(sample) / 100.0F;
        const float visibility = spaceEnvironmentHorizonVisibility(
            clearance, 1.0F);
        require(std::isfinite(visibility) && visibility >= previousVisibility &&
                    visibility >= 0.0F && visibility <= 1.0F,
                "silhouette exclusion is not finite, monotonic and bounded");
        previousVisibility = visibility;
    }

    // The exclusion belongs only to the forward ray half.  These paired rays
    // exercise several observer distances, including rotated directions that
    // model post-wormhole mapped exits.  The antipodal sky must never inherit
    // a duplicate planet disk while the forward silhouette remains excluded.
    constexpr float planetRadius = 1.0F;
    constexpr float planetOuterRadius = 1.08F;
    for (const float distance : {1.25F, 2.0F, 4.0F, 12.0F}) {
        const std::array<float, 3> camera{0.0F, 0.0F, distance};
        require(spaceEnvironmentForwardStarVisibility(
                    camera, {0.0F, 0.0F, -1.0F}, planetOuterRadius,
                    planetRadius) == 0.0F,
                "forward planet silhouette did not exclude stars");
        require(spaceEnvironmentForwardStarVisibility(
                    camera, {0.0F, 0.0F, 1.0F}, planetOuterRadius,
                    planetRadius) == 1.0F,
                "antipodal sky inherited a duplicate planet mask");

        const float targetClearance = 0.11F * planetRadius;
        const float impact = planetOuterRadius + targetClearance;
        const float sine = impact / distance;
        if (sine < 1.0F) {
            const float cosine = std::sqrt(1.0F - sine * sine);
            const std::array<float, 3> grazing{sine, 0.0F, -cosine};
            const std::array<float, 3> antiGrazing{-sine, 0.0F, cosine};
            const float expected = spaceEnvironmentHorizonVisibility(
                targetClearance, planetRadius);
            require(std::abs(spaceEnvironmentForwardStarVisibility(
                        camera, grazing, planetOuterRadius, planetRadius) -
                    expected) < 2.0e-5F,
                    "forward grazing fade disagrees with horizon oracle");
            require(spaceEnvironmentForwardStarVisibility(
                        camera, antiGrazing, planetOuterRadius,
                        planetRadius) == 1.0F,
                    "away grazing ray was distance-masked");
        }
    }
    const std::array<float, 3> mappedCamera{4.0F, -3.0F, 2.0F};
    const float mappedLength = std::sqrt(29.0F);
    const std::array<float, 3> mappedToward{
        -4.0F / mappedLength, 3.0F / mappedLength,
        -2.0F / mappedLength};
    const std::array<float, 3> mappedAway{
        4.0F / mappedLength, -3.0F / mappedLength,
        2.0F / mappedLength};
    require(spaceEnvironmentForwardStarVisibility(
                mappedCamera, mappedToward, planetOuterRadius,
                planetRadius) == 0.0F &&
                spaceEnvironmentForwardStarVisibility(
                    mappedCamera, mappedAway, planetOuterRadius,
                    planetRadius) == 1.0F,
            "rotated/lensed exit did not preserve toward/away ownership");
    require(spaceEnvironmentForwardStarVisibility(
                {0.0F, 0.0F, std::nanf("")}, {0.0F, 0.0F, 1.0F},
                planetOuterRadius, planetRadius) == 0.0F,
            "nonfinite forward visibility input escaped its guard");

    float previousFootprint = 1.0F;
    for (std::uint32_t sample = 0U; sample <= 100U; ++sample) {
        const float distance = 0.003F * static_cast<float>(sample) / 100.0F;
        const float footprint = spaceEnvironmentStarFootprint(
            distance, 0.0005F, 0.0015F);
        require(std::isfinite(footprint) && footprint <= previousFootprint &&
                    footprint >= 0.0F && footprint <= 1.0F,
                "anti-aliased star footprint is unstable or unbounded");
        previousFootprint = footprint;
    }
    require(spaceEnvironmentStarFootprint(
                std::nanf(""), 0.001F, 0.001F) == 0.0F,
            "nonfinite star input escaped the finite guard");
    require(std::abs(spaceEnvironmentLensedAngularFootprint(
                0.002F, 0.002F, 0.002F) - 0.002F) < 1.0e-7F &&
                std::abs(spaceEnvironmentLensedAngularFootprint(
                    0.002F, 0.008F, 0.002F) - 0.008F) < 1.0e-7F &&
                std::abs(spaceEnvironmentLensedAngularFootprint(
                    0.002F, 1.0F, 0.002F) - 0.016F) < 1.0e-7F,
            "lensed star ray footprint was not identity/magnified/bounded");
    require(spaceEnvironmentLensedAngularFootprint(
                0.002F, std::nanf(""), 0.002F) == 0.0F,
            "nonfinite lensed star footprint escaped its guard");

    // Direction-native 3-D shell addressing has no longitude coordinate and
    // therefore no pole at which all azimuths collapse into one stretched
    // atlas row. A small ring around either pole must retain distinct cells.
    std::array<std::array<int, 3>, 8> northCells{};
    std::array<std::array<int, 3>, 8> southCells{};
    for (std::size_t index = 0; index < northCells.size(); ++index) {
        const float angle = 6.28318530718F * static_cast<float>(index) /
                            static_cast<float>(northCells.size());
        const std::array<float, 3> north{
            0.02F * std::cos(angle), 0.02F * std::sin(angle), 0.9998F};
        const std::array<float, 3> south{
            0.02F * std::cos(angle), 0.02F * std::sin(angle), -0.9998F};
        northCells[index] = spaceEnvironmentUnitShellCell(north, 256);
        southCells[index] = spaceEnvironmentUnitShellCell(south, 256);
    }
    std::size_t distinctNorth = 0U;
    std::size_t distinctSouth = 0U;
    for (std::size_t i = 0; i < northCells.size(); ++i) {
        bool firstNorth = true;
        bool firstSouth = true;
        for (std::size_t j = 0; j < i; ++j) {
            firstNorth = firstNorth && northCells[i] != northCells[j];
            firstSouth = firstSouth && southCells[i] != southCells[j];
        }
        distinctNorth += firstNorth ? 1U : 0U;
        distinctSouth += firstSouth ? 1U : 0U;
    }
    require(distinctNorth >= 6U && distinctSouth >= 6U,
            "direction-native star addressing pinched at a sphere pole");
    require(spaceEnvironmentUnitShellCell(
                {std::nanf(""), 0.0F, 1.0F}, 256) ==
                std::array<int, 3>{},
            "direction-native star cell accepted a nonfinite direction");

    std::cout << "Space environment translation/orientation, direction-native "
                 "polar-safe cells, forward-only horizon exclusion, lensed "
                 "toward/away ownership, finite and anti-aliased footprint "
                 "checks passed.\n";
}
