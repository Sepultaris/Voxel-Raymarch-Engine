#include "planet/fractal_planet_sdf.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

using Vector = std::array<float, 3>;

[[nodiscard]] float dot(const Vector& left, const Vector& right) {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

[[nodiscard]] Vector add(const Vector& left, const Vector& right) {
    return {left[0] + right[0], left[1] + right[1], left[2] + right[2]};
}

[[nodiscard]] Vector scale(const Vector& value, float amount) {
    return {value[0] * amount, value[1] * amount, value[2] * amount};
}

[[nodiscard]] Vector cross(const Vector& left, const Vector& right) {
    return {left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0]};
}

[[nodiscard]] Vector normalized(const Vector& value) {
    const float length = std::sqrt(dot(value, value));
    return scale(value, 1.0F / std::max(length, 1e-20F));
}

[[nodiscard]] bool sphereInterval(const Vector& origin, const Vector& ray,
                                  float radius, float& nearDistance,
                                  float& farDistance) {
    const float projection = dot(origin, ray);
    const float discriminant = projection * projection - dot(origin, origin) +
                               radius * radius;
    if (discriminant < 0.0F) {
        return false;
    }
    const float root = std::sqrt(discriminant);
    nearDistance = std::max(-projection - root, 0.0F);
    farDistance = -projection + root;
    return farDistance >= 0.0F;
}

[[nodiscard]] float videoPathContinuousLevel(float travel) {
    constexpr float projectedParentNumerator =
        voxel::kFractalPlanetParentAngularWidth * 900.0F * 0.5F;
    const float projectedPixels = projectedParentNumerator /
        std::max(travel, 2.5e-7F);
    return std::clamp(std::log2(std::max(
        projectedPixels / voxel::kFractalPlanetDefaultTargetPixels, 1.0F)),
        0.0F, float(voxel::kFractalPlanetMaximumLevels));
}

[[nodiscard]] float videoPathSignedField(const Vector& camera, const Vector& ray,
                                         float travel,
                                         std::uint32_t evaluationLevels =
                                             voxel::kFractalPlanetMaximumLevels,
                                         float* unresolved = nullptr) {
    const Vector position = add(camera, scale(ray, travel));
    const float radius = std::sqrt(dot(position, position));
    const Vector direction = scale(position, 1.0F / std::max(radius, 1e-20F));
    const auto field = voxel::evaluateFractalPlanetField(
        direction, 1337U, videoPathContinuousLevel(travel),
        voxel::kFractalPlanetDefaultMicroRelief, evaluationLevels);
    if (unresolved != nullptr) {
        *unresolved = field.unresolvedMicroBound;
    }
    return radius - field.surfaceRadiusScale;
}

// Dense front-to-back oracle used only by the screenshot-class regression.
// It deliberately does not share the production stepping policy.
[[nodiscard]] bool denseVideoPathOracle(const Vector& camera, const Vector& ray,
                                        float& hitDistance,
                                        std::uint32_t samples = 4096U) {
    float nearDistance = 0.0F;
    float farDistance = 0.0F;
    if (!sphereInterval(camera, ray, voxel::kFractalPlanetOuterRadiusScale,
                        nearDistance, farDistance)) {
        return false;
    }
    constexpr float tolerance = 2e-6F;
    float previousTravel = nearDistance;
    float previous = videoPathSignedField(camera, ray, previousTravel);
    if (previous <= tolerance) {
        hitDistance = previousTravel;
        return true;
    }
    for (std::uint32_t sample = 1U; sample <= samples; ++sample) {
        const float travel = nearDistance + (farDistance - nearDistance) *
            static_cast<float>(sample) / static_cast<float>(samples);
        const float value = videoPathSignedField(camera, ray, travel);
        if (value <= tolerance) {
            float low = previousTravel;
            float high = travel;
            for (std::uint32_t root = 0U; root < 20U; ++root) {
                const float candidate = 0.5F * (low + high);
                if (videoPathSignedField(camera, ray, candidate) > 0.0F) {
                    low = candidate;
                } else {
                    high = candidate;
                }
            }
            hitDistance = 0.5F * (low + high);
            return true;
        }
        previous = value;
        previousTravel = travel;
    }
    return false;
}

struct MarchDiagnostic {
    std::uint32_t steps{};
    float finalTravel{};
    float farDistance{};
    float minimumExact{std::numeric_limits<float>::infinity()};
    float minimumOutsideBound{std::numeric_limits<float>::infinity()};
};

[[nodiscard]] bool boundedVideoPathMarch(const Vector& camera, const Vector& ray,
                                         std::uint32_t maximumSteps,
                                         float& hitDistance,
                                         MarchDiagnostic* diagnostic = nullptr) {
    float nearDistance = 0.0F;
    float farDistance = 0.0F;
    if (!sphereInterval(camera, ray, voxel::kFractalPlanetOuterRadiusScale,
                        nearDistance, farDistance)) {
        return false;
    }
    if (diagnostic != nullptr) {
        diagnostic->farDistance = farDistance;
    }
    constexpr float tolerance = 2e-6F;
    float travel = nearDistance;
    float lastOutsideTravel = travel;
    bool lastOutsideValid = false;
    for (std::uint32_t step = 0U;
         step < maximumSteps && travel <= farDistance; ++step) {
        if (diagnostic != nullptr) {
            diagnostic->steps = step + 1U;
            diagnostic->finalTravel = travel;
        }
        const float level = videoPathContinuousLevel(travel);
        const std::uint32_t desired = static_cast<std::uint32_t>(
            std::clamp(std::ceil(level), 0.0F,
                       float(voxel::kFractalPlanetMaximumLevels)));
        std::uint32_t evaluatedLevels = std::min(desired, 2U);
        float unresolved = 0.0F;
        float sampledSigned = videoPathSignedField(
            camera, ray, travel, evaluatedLevels, &unresolved);
        float conservativeOutside = std::max(
            sampledSigned - unresolved, 0.0F);
        float refinementBand = std::max(
            tolerance * 2.0F, unresolved * 0.125F);
        while (evaluatedLevels < desired &&
               conservativeOutside <= refinementBand) {
            evaluatedLevels = std::min(evaluatedLevels + 2U, desired);
            sampledSigned = videoPathSignedField(
                camera, ray, travel, evaluatedLevels, &unresolved);
            conservativeOutside = std::max(
                sampledSigned - unresolved, 0.0F);
            refinementBand = std::max(
                tolerance * 2.0F, unresolved * 0.125F);
        }
        if (diagnostic != nullptr) {
            diagnostic->minimumOutsideBound = std::min(
                diagnostic->minimumOutsideBound, sampledSigned - unresolved);
        }
        const float outsideLipschitz = voxel::fractalPlanetLipschitzBound(
            evaluatedLevels);
        const float exactLipschitz = voxel::fractalPlanetLipschitzBound(desired);
        if (conservativeOutside > refinementBand) {
            lastOutsideTravel = travel;
            lastOutsideValid = true;
            travel += std::max(conservativeOutside / outsideLipschitz,
                               tolerance * 0.5F);
            continue;
        }
        const float exact = evaluatedLevels == desired
            ? sampledSigned
            : videoPathSignedField(camera, ray, travel, desired);
        if (diagnostic != nullptr) {
            diagnostic->minimumExact = std::min(diagnostic->minimumExact, exact);
        }
        if (std::abs(exact) <= tolerance) {
            hitDistance = travel;
            return true;
        }
        if (lastOutsideValid && exact < 0.0F) {
            float low = lastOutsideTravel;
            float high = travel;
            for (std::uint32_t root = 0U; root < 20U; ++root) {
                const float candidate = 0.5F * (low + high);
                if (videoPathSignedField(camera, ray, candidate) > 0.0F) {
                    low = candidate;
                } else {
                    high = candidate;
                }
            }
            hitDistance = 0.5F * (low + high);
            return true;
        }
        if (exact > 0.0F) {
            lastOutsideTravel = travel;
            lastOutsideValid = true;
        } else if (!lastOutsideValid) {
            hitDistance = travel;
            return true;
        }
        travel += std::max(std::abs(exact) / exactLipschitz,
                           tolerance * 0.5F);
    }
    return false;
}

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

} // namespace

int main() {
    using namespace voxel;
    const auto farLod = selectFractalPlanetLod(2.0F, 1.0F, 900U);
    const auto nearLod = selectFractalPlanetLod(0.012F, 1.0F, 900U);
    const auto closerLod = selectFractalPlanetLod(0.003F, 1.0F, 900U);
    require(farLod.leafLevel == 0U,
            "Far fractal planet leaves must merge to the parent level");
    require(nearLod.leafLevel >= 6U && closerLod.leafLevel >= nearLod.leafLevel,
            "Approach must monotonically reveal physically smaller SDF leaves");
    require(closerLod.leafWorldWidth < nearLod.leafWorldWidth,
            "Closer adaptive leaves did not become physically smaller");
    const auto leaf = fractalPlanetLeafAddress(
        {0.21F, 0.31F, 0.92F}, nearLod.leafLevel);
    const auto repeatedLeaf = fractalPlanetLeafAddress(
        {0.21F, 0.31F, 0.92F}, nearLod.leafLevel);
    const auto parentLeaf = fractalPlanetLeafAddress(
        {0.21F, 0.31F, 0.92F}, nearLod.leafLevel - 1U);
    require(leaf == repeatedLeaf && leaf.face == parentLeaf.face &&
                (leaf.x >> 1U) == parentLeaf.x &&
                (leaf.y >> 1U) == parentLeaf.y,
            "Procedural virtual leaf addressing is not deterministic and nested");
    require(fractalPlanetLipschitzBound(10U) >
                fractalPlanetLipschitzBound(2U) &&
                fractalPlanetLipschitzBound(10U) > 9.0F,
            "Fractal planet derivative bound does not cover active spectral bands");

    const std::array<float, 3> direction{0.26726124F, 0.53452248F, 0.80178373F};
    const auto level2 = evaluateFractalPlanetField(direction, 1337U, 2.0F);
    const auto level8 = evaluateFractalPlanetField(direction, 1337U, 8.0F);
    const auto repeated = evaluateFractalPlanetField(direction, 1337U, 8.0F);
    require(level2.finite && level8.finite && repeated.finite,
            "Fractal planet field produced a nonfinite sample");
    require(std::bit_cast<std::uint32_t>(level8.surfaceRadiusScale) ==
                std::bit_cast<std::uint32_t>(repeated.surfaceRadiusScale),
            "Fractal planet field is not deterministic");
    require(std::abs(level8.terrainRadiusScale - level2.terrainRadiusScale) > 1e-7F,
            "Higher adaptive levels failed to add new coherent terrain detail");
    const auto belowTransition = evaluateFractalPlanetField(
        direction, 1337U, 5.0F - 1e-4F);
    const auto aboveTransition = evaluateFractalPlanetField(
        direction, 1337U, 5.0F + 1e-4F);
    require(std::abs(belowTransition.terrainRadiusScale -
                     aboveTransition.terrainRadiusScale) < 2e-5F,
            "Smooth band transition popped at an integer leaf boundary");
    require(level8.surfaceRadiusScale >= kFractalPlanetSeaRadiusScale &&
                level8.surfaceRadiusScale <= kFractalPlanetOuterRadiusScale,
            "Fractal planet surface escaped its conservative radial shell");

    const auto coarse = evaluateFractalPlanetField(direction, 1337U, 8.0F,
                                                    0.012F, 2U);
    require(coarse.finite && coarse.unresolvedMicroBound > 0.0F &&
                std::abs(level8.terrainRadiusScale - coarse.terrainRadiusScale) <=
                    coarse.unresolvedMicroBound + 2e-6F,
            "Coarse SDF interval did not conservatively contain refined terrain");

    const std::array<float, 3> seamA{0.70710677F, 0.70710677F, 0.0F};
    const std::array<float, 3> seamB{0.70710671F, 0.70710683F, 0.0F};
    const auto seamFieldA = evaluateFractalPlanetField(seamA, 42U, 9.0F);
    const auto seamFieldB = evaluateFractalPlanetField(seamB, 42U, 9.0F);
    require(std::abs(seamFieldA.surfaceRadiusScale -
                     seamFieldB.surfaceRadiusScale) < 1e-4F,
            "World-space fractal planet field is discontinuous across an address seam");

    const auto invalid = evaluateFractalPlanetField(
        direction, 1U, std::numeric_limits<float>::quiet_NaN());
    require(!invalid.finite,
            "Nonfinite fractal planet input did not fail closed");

    // Exhaustive closed-surface reference: every sampled camera ray crossing
    // the analytic sea sphere must have a positive outer-shell value and a
    // front-to-back zero bracket before or at the sea entry.
    const std::array<float, 3> camera{0.0F, 0.0F, 3.0F};
    std::uint32_t closedShellRays = 0U;
    for (int iy = -20; iy <= 20; ++iy) {
        for (int ix = -20; ix <= 20; ++ix) {
            std::array<float, 3> ray{
                static_cast<float>(ix) / 42.0F,
                static_cast<float>(iy) / 42.0F, -1.0F};
            const float rayLength = std::sqrt(ray[0] * ray[0] +
                                              ray[1] * ray[1] +
                                              ray[2] * ray[2]);
            for (float& value : ray) {
                value /= rayLength;
            }
            const auto sphereNear = [&](float radius, float& nearDistance) {
                const float projection = camera[0] * ray[0] +
                    camera[1] * ray[1] + camera[2] * ray[2];
                const float cameraLengthSquared = 9.0F;
                const float discriminant = projection * projection -
                    cameraLengthSquared + radius * radius;
                if (discriminant < 0.0F) {
                    return false;
                }
                nearDistance = -projection - std::sqrt(discriminant);
                return nearDistance >= 0.0F;
            };
            float outerNear = 0.0F;
            float seaNear = 0.0F;
            if (!sphereNear(kFractalPlanetSeaRadiusScale, seaNear)) {
                continue;
            }
            require(sphereNear(kFractalPlanetOuterRadiusScale, outerNear),
                    "Closed sea ray missed the conservative outer shell");
            ++closedShellRays;
            float previousSigned = 0.0F;
            bool previousValid = false;
            bool bracketed = false;
            for (std::uint32_t sample = 0U; sample <= 64U; ++sample) {
                const float t = outerNear + (seaNear - outerNear) *
                    static_cast<float>(sample) / 64.0F;
                std::array<float, 3> position{
                    camera[0] + ray[0] * t,
                    camera[1] + ray[1] * t,
                    camera[2] + ray[2] * t};
                const float radius = std::sqrt(position[0] * position[0] +
                                               position[1] * position[1] +
                                               position[2] * position[2]);
                std::array<float, 3> sampleDirection{
                    position[0] / radius, position[1] / radius,
                    position[2] / radius};
                const auto field = evaluateFractalPlanetField(
                    sampleDirection, 1337U, 8.0F);
                require(field.finite,
                        "Closed-shell reference encountered a nonfinite field");
                const float signedValue = radius - field.surfaceRadiusScale;
                if (sample == 0U) {
                    require(signedValue > 0.0F,
                            "Conservative outer shell began inside the field");
                }
                if (previousValid && previousSigned > 0.0F &&
                    signedValue <= 2e-6F) {
                    bracketed = true;
                    break;
                }
                previousSigned = signedValue;
                previousValid = true;
            }
            require(bracketed,
                    "Ray crossing the closed sea shell had no SDF zero bracket");
        }
    }
    require(closedShellRays > 500U,
            "Closed-shell regression sampled too few camera rays");

    // Recreate the August 16 surface-player recording: the camera is just
    // above the procedural surface, pitched slightly down, and scans the
    // displaced silhouette above the analytic sea sphere. The previous
    // sea-only recovery gate could not see these terrain-only intersections.
    const Vector videoRadial = normalized(
        {0.0F, std::sin(0.16F), std::cos(0.16F)});
    const Vector videoRight = normalized(cross({0.0F, 1.0F, 0.0F}, videoRadial));
    const Vector videoTangent = normalized(cross(videoRadial, videoRight));
    const float videoPitch = -0.18F;
    const Vector videoForward = normalized(add(
        scale(videoTangent, std::cos(videoPitch)),
        scale(videoRadial, std::sin(videoPitch))));
    const Vector videoScreenRight = normalized(cross(videoForward, videoRadial));
    const Vector videoScreenUp = normalized(cross(videoScreenRight, videoForward));
    const auto cameraField = evaluateFractalPlanetField(
        videoRadial, 1337U, 10.0F);
    const Vector videoCamera = scale(
        videoRadial, cameraField.surfaceRadiusScale + 0.00539F);
    std::uint32_t videoOracleHits = 0U;
    std::uint32_t videoDisplacedOnlyHits = 0U;
    std::uint32_t videoFastMisses = 0U;
    std::uint32_t videoUniformRecoveryMisses = 0U;
    std::uint32_t videoExtendedMisses = 0U;
    std::uint32_t videoDistanceMismatches = 0U;
    float videoMaximumDistanceError = 0.0F;
    Vector firstMissRay{};
    float firstMissOracleDistance{};
    bool haveFirstMiss = false;
    for (std::uint32_t row = 0U; row < 24U; ++row) {
        const float screenY = 0.04F + 0.56F *
            static_cast<float>(row) / 23.0F;
        for (std::uint32_t column = 0U; column < 72U; ++column) {
            const float screenX = -1.2F + 2.8F *
                static_cast<float>(column) / 71.0F;
            const Vector ray = normalized(add(videoForward, add(
                scale(videoScreenRight, screenX),
                scale(videoScreenUp, screenY))));
            float oracleDistance = 0.0F;
            if (!denseVideoPathOracle(videoCamera, ray, oracleDistance)) {
                continue;
            }
            ++videoOracleHits;
            float seaNear = 0.0F;
            float seaFar = 0.0F;
            const bool crossesSea = sphereInterval(
                videoCamera, ray, kFractalPlanetSeaRadiusScale,
                seaNear, seaFar);
            if (!crossesSea) {
                ++videoDisplacedOnlyHits;
            }
            float boundedDistance = 0.0F;
            if (!boundedVideoPathMarch(videoCamera, ray, 256U,
                                       boundedDistance)) {
                ++videoFastMisses;
                float recoveredDistance = 0.0F;
                if (!denseVideoPathOracle(videoCamera, ray,
                                          recoveredDistance,
                                          crossesSea ? 64U : 192U)) {
                    ++videoUniformRecoveryMisses;
                } else {
                    const float distanceError = std::abs(
                        recoveredDistance - oracleDistance);
                    videoMaximumDistanceError = std::max(
                        videoMaximumDistanceError, distanceError);
                    if (distanceError > 1e-4F) {
                        ++videoDistanceMismatches;
                    }
                }
                if (!haveFirstMiss) {
                    firstMissRay = ray;
                    firstMissOracleDistance = oracleDistance;
                    haveFirstMiss = true;
                }
            } else {
                const float distanceError = std::abs(
                    boundedDistance - oracleDistance);
                videoMaximumDistanceError = std::max(
                    videoMaximumDistanceError, distanceError);
                if (distanceError > 1e-4F) {
                    ++videoDistanceMismatches;
                }
            }
            if (!boundedVideoPathMarch(videoCamera, ray, 1024U,
                                       boundedDistance)) {
                ++videoExtendedMisses;
            }
        }
    }
    std::cout << "Video-path oracle hits=" << videoOracleHits
              << " displaced-only=" << videoDisplacedOnlyHits
              << " fast-256-misses=" << videoFastMisses
              << " production-recovery-misses="
              << videoUniformRecoveryMisses
              << " corrected-1024-misses=" << videoExtendedMisses
              << " distance-mismatches=" << videoDistanceMismatches
              << " max-distance-error=" << videoMaximumDistanceError << '\n';
    if (haveFirstMiss) {
        MarchDiagnostic diagnostic{};
        float ignoredDistance = 0.0F;
        const bool diagnosticHit = boundedVideoPathMarch(
            videoCamera, firstMissRay, 256U, ignoredDistance, &diagnostic);
        std::cout << "First video-path miss: hit=" << diagnosticHit
                  << " steps=" << diagnostic.steps << " travel/far="
                  << diagnostic.finalTravel << '/' << diagnostic.farDistance
                  << " oracle=" << firstMissOracleDistance
                  << " min-exact=" << diagnostic.minimumExact
                  << " min-outside-bound=" << diagnostic.minimumOutsideBound
                  << "\n";
    }
    require(videoOracleHits > 100U && videoDisplacedOnlyHits > 100U,
            "Video-path oracle did not cover the displaced terrain silhouette");
    require(videoUniformRecoveryMisses == 0U,
            "Full-interval recovery scan missed an oracle terrain hit");
    require(videoExtendedMisses == 0U && videoDistanceMismatches == 0U,
            "Corrected sign-aware traversal diverged from the dense video-path oracle");

    std::cout << "Fractal planet SDF adaptive voxelization invariants passed.\n";
    return 0;
}
