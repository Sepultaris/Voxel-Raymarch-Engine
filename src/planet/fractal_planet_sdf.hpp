#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace voxel {

inline constexpr std::uint32_t kFractalPlanetMaximumLevels = 10U;
inline constexpr std::uint32_t kFractalPlanetRootIterations = 8U;
inline constexpr std::uint32_t kFractalPlanetMaximumMarchSteps = 256U;
inline constexpr float kFractalPlanetParentAngularWidth = 0.003F;
inline constexpr float kFractalPlanetDefaultTargetPixels = 1.25F;
inline constexpr float kFractalPlanetDefaultMicroRelief = 0.012F;
inline constexpr float kFractalPlanetBaseRadiusScale = 1.012F;
inline constexpr float kFractalPlanetSeaRadiusScale = 1.006F;
inline constexpr float kFractalPlanetOuterRadiusScale = 1.048F;
inline constexpr float kFractalPlanetInnerRadiusScale = 0.982F;

struct FractalPlanetLod {
    float continuousLevel{};
    std::uint32_t leafLevel{};
    float projectedParentPixels{};
    float leafWorldWidth{};
};

struct FractalPlanetField {
    float terrainRadiusScale{};
    float surfaceRadiusScale{};
    float unresolvedMicroBound{};
    std::array<float, 3> directionGradient{};
    bool water{};
    bool finite{};
};

struct FractalPlanetLeafAddress {
    std::uint32_t face{};
    std::uint32_t level{};
    std::uint32_t x{};
    std::uint32_t y{};

    [[nodiscard]] bool operator==(const FractalPlanetLeafAddress&) const noexcept = default;
};

[[nodiscard]] inline float fractalSmoothstep(float edge0, float edge1,
                                             float value) noexcept {
    const float t = std::clamp((value - edge0) / (edge1 - edge0), 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

[[nodiscard]] inline std::uint32_t fractalPlanetHash(std::uint32_t value) noexcept {
    value ^= value >> 16U;
    value *= 0x7feb352dU;
    value ^= value >> 15U;
    value *= 0x846ca68bU;
    return value ^ (value >> 16U);
}

[[nodiscard]] inline float fractalPlanetPhase(std::uint32_t seed,
                                              std::uint32_t lane) noexcept {
    constexpr float twoPiOver24Bits =
        6.2831853071795864769F / 16777216.0F;
    const std::uint32_t bits = fractalPlanetHash(
        seed ^ (0x9e3779b9U * (lane + 1U)));
    return static_cast<float>(bits & 0x00ffffffU) * twoPiOver24Bits;
}

[[nodiscard]] inline FractalPlanetLod selectFractalPlanetLod(
    float cameraDistanceToSurface, float planetRadius, std::uint32_t resolutionY,
    float targetPixels = kFractalPlanetDefaultTargetPixels,
    std::uint32_t maximumLevels = kFractalPlanetMaximumLevels) noexcept {
    FractalPlanetLod result{};
    if (!(cameraDistanceToSurface > 0.0F) || !(planetRadius > 0.0F) ||
        resolutionY == 0U || !std::isfinite(cameraDistanceToSurface) ||
        !std::isfinite(planetRadius)) {
        return result;
    }
    maximumLevels = std::min(maximumLevels, kFractalPlanetMaximumLevels);
    targetPixels = std::max(targetPixels, 0.125F);
    const float parentWidth = kFractalPlanetParentAngularWidth * planetRadius;
    result.projectedParentPixels = parentWidth *
        (static_cast<float>(resolutionY) * 0.5F) / cameraDistanceToSurface;
    result.continuousLevel = std::clamp(
        std::log2(std::max(result.projectedParentPixels / targetPixels, 1.0F)),
        0.0F, static_cast<float>(maximumLevels));
    result.leafLevel = static_cast<std::uint32_t>(std::ceil(result.continuousLevel));
    result.leafWorldWidth = parentWidth /
        static_cast<float>(std::uint32_t{1U} << result.leafLevel);
    return result;
}

[[nodiscard]] inline float fractalPlanetLipschitzBound(
    std::uint32_t activeLevels) noexcept {
    activeLevels = std::min(activeLevels, kFractalPlanetMaximumLevels);
    // The three macro bands contribute <0.23. Each micro band has
    // amplitude*frequency=0.288 and three unit basis derivatives, hence 0.864.
    return 1.25F + 0.864F * static_cast<float>(activeLevels);
}

[[nodiscard]] inline FractalPlanetLeafAddress fractalPlanetLeafAddress(
    const std::array<float, 3>& direction, std::uint32_t level) noexcept {
    level = std::min(level, kFractalPlanetMaximumLevels);
    const float ax = std::abs(direction[0]);
    const float ay = std::abs(direction[1]);
    const float az = std::abs(direction[2]);
    float u = 0.0F;
    float v = 0.0F;
    std::uint32_t face = 0U;
    if (ax >= ay && ax >= az) {
        const float inverse = 1.0F / std::max(ax, 1e-20F);
        const bool positive = direction[0] >= 0.0F;
        face = positive ? 0U : 1U;
        u = (positive ? -direction[2] : direction[2]) * inverse;
        v = direction[1] * inverse;
    } else if (ay >= az) {
        const float inverse = 1.0F / std::max(ay, 1e-20F);
        const bool positive = direction[1] >= 0.0F;
        face = positive ? 2U : 3U;
        u = direction[0] * inverse;
        v = (positive ? -direction[2] : direction[2]) * inverse;
    } else {
        const float inverse = 1.0F / std::max(az, 1e-20F);
        const bool positive = direction[2] >= 0.0F;
        face = positive ? 4U : 5U;
        u = (positive ? direction[0] : -direction[0]) * inverse;
        v = direction[1] * inverse;
    }
    const std::uint32_t resolution = 1U << level;
    const auto coordinate = [resolution](float value) noexcept {
        const float normalized = std::clamp(value * 0.5F + 0.5F, 0.0F, 1.0F);
        return std::min(static_cast<std::uint32_t>(
                            normalized * static_cast<float>(resolution)),
                        resolution - 1U);
    };
    return {face, level, coordinate(u), coordinate(v)};
}

[[nodiscard]] inline FractalPlanetField evaluateFractalPlanetField(
    const std::array<float, 3>& direction, std::uint32_t seed,
    float continuousLevel, float microRelief = kFractalPlanetDefaultMicroRelief,
    std::uint32_t evaluationLevels = kFractalPlanetMaximumLevels) noexcept {
    FractalPlanetField result{};
    result.terrainRadiusScale = kFractalPlanetBaseRadiusScale;
    result.surfaceRadiusScale = kFractalPlanetSeaRadiusScale;
    if (!std::isfinite(continuousLevel) || !std::isfinite(microRelief) ||
        !std::isfinite(direction[0]) || !std::isfinite(direction[1]) ||
        !std::isfinite(direction[2])) {
        return result;
    }
    continuousLevel = std::clamp(
        continuousLevel, 0.0F, static_cast<float>(kFractalPlanetMaximumLevels));
    evaluationLevels = std::min(evaluationLevels, kFractalPlanetMaximumLevels);
    microRelief = std::clamp(microRelief, 0.0F, 0.018F);
    constexpr std::array<std::array<float, 3>, 3> basis{{
        {0.5773502692F, 0.5773502692F, 0.5773502692F},
        {0.7071067812F, -0.7071067812F, 0.0F},
        {0.4082482905F, 0.4082482905F, -0.8164965809F}}};
    const auto addBand = [&](float frequency, float amplitude,
                             std::uint32_t laneBase) {
        std::array<float, 3> angle{};
        std::array<float, 3> sine{};
        std::array<float, 3> cosine{};
        for (std::size_t lane = 0U; lane < 3U; ++lane) {
            const float projection = direction[0] * basis[lane][0] +
                direction[1] * basis[lane][1] +
                direction[2] * basis[lane][2];
            angle[lane] = frequency * projection +
                fractalPlanetPhase(seed, laneBase + static_cast<std::uint32_t>(lane));
            sine[lane] = std::sin(angle[lane]);
            cosine[lane] = std::cos(angle[lane]);
        }
        const float signal = sine[0] * sine[1] * sine[2];
        result.terrainRadiusScale += amplitude * signal;
        for (std::size_t axis = 0U; axis < 3U; ++axis) {
            result.directionGradient[axis] += amplitude * frequency *
                (cosine[0] * sine[1] * sine[2] * basis[0][axis] +
                 sine[0] * cosine[1] * sine[2] * basis[1][axis] +
                 sine[0] * sine[1] * cosine[2] * basis[2][axis]);
        }
    };

    // Planet-scale bands are always present. Adaptive bands below them are
    // the real progressively voxelized SDF detail.
    addBand(1.7F, 0.010F, 40U);
    addBand(4.3F, 0.006F, 50U);
    addBand(11.0F, 0.003F, 60U);

    float amplitude = microRelief * 0.5F;
    float frequency = 48.0F;
    for (std::uint32_t octave = 0U;
         octave < kFractalPlanetMaximumLevels; ++octave) {
        const float weight = fractalSmoothstep(
            0.0F, 1.0F, continuousLevel - static_cast<float>(octave));
        if (octave < evaluationLevels && weight > 0.0F) {
            addBand(frequency, amplitude * weight, octave * 3U);
        } else if (weight > 0.0F) {
            result.unresolvedMicroBound += amplitude * weight;
        }
        amplitude *= 0.5F;
        frequency *= 2.0F;
    }
    result.water = result.terrainRadiusScale < kFractalPlanetSeaRadiusScale;
    result.surfaceRadiusScale = std::max(
        result.terrainRadiusScale, kFractalPlanetSeaRadiusScale);
    if (result.water) {
        result.directionGradient = {0.0F, 0.0F, 0.0F};
    }
    result.finite = std::isfinite(result.terrainRadiusScale) &&
        std::isfinite(result.surfaceRadiusScale) &&
        std::isfinite(result.unresolvedMicroBound) &&
        std::isfinite(result.directionGradient[0]) &&
        std::isfinite(result.directionGradient[1]) &&
        std::isfinite(result.directionGradient[2]);
    return result;
}

} // namespace voxel
