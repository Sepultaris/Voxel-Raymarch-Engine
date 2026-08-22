#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace voxel {

inline constexpr std::uint32_t kAdaptivePlanetSdfMaximumLevels = 9U;
inline constexpr float kAdaptivePlanetSdfDefaultTargetPixels = 1.25F;
inline constexpr float kAdaptivePlanetSdfDefaultLayerFraction = 0.15F;
inline constexpr float kAdaptivePlanetSdfMaximumLayerFraction = 0.20F;
inline constexpr std::uint32_t kAdaptivePlanetSdfRootIterations = 7U;

struct AdaptivePlanetSdfLod {
    float continuousLevel{};
    std::uint32_t completeLevels{};
    float transition{};
    float virtualCellPixels{};
};

struct AdaptivePlanetSdfField {
    float depthLayerFraction{};
    std::array<float, 3> directionGradient{};
    float evaluatedAmplitude{};
    float residualAmplitudeBound{};
};

struct AdaptivePlanetSdfCellAddress {
    std::uint32_t face{};
    std::uint32_t level{};
    std::uint32_t x{};
    std::uint32_t y{};

    [[nodiscard]] bool operator==(const AdaptivePlanetSdfCellAddress&) const noexcept = default;
};

struct AdaptivePlanetSdfRoot {
    bool hit{};
    float rayOffset{};
    std::array<float, 3> normal{};
    std::uint32_t iterations{};
};

[[nodiscard]] inline float adaptiveSdfSmoothstep(float value) noexcept {
    const float t = std::clamp(value, 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

[[nodiscard]] inline AdaptivePlanetSdfLod adaptivePlanetSdfLod(
    float projectedParentPixels, float targetPixels,
    std::uint32_t maximumLevels = kAdaptivePlanetSdfMaximumLevels,
    bool forceFinest = false) noexcept {
    AdaptivePlanetSdfLod result{};
    maximumLevels = std::min(maximumLevels, kAdaptivePlanetSdfMaximumLevels);
    if (!std::isfinite(projectedParentPixels) ||
        !std::isfinite(targetPixels) || targetPixels <= 0.0F) {
        return result;
    }
    const float rawLevel = forceFinest
        ? static_cast<float>(maximumLevels)
        : std::log2(std::max(projectedParentPixels / targetPixels, 1.0F));
    result.continuousLevel = std::clamp(
        rawLevel, 0.0F, static_cast<float>(maximumLevels));
    result.completeLevels = std::min(
        static_cast<std::uint32_t>(std::floor(result.continuousLevel)),
        maximumLevels);
    result.transition = adaptiveSdfSmoothstep(
        result.continuousLevel - static_cast<float>(result.completeLevels));
    result.virtualCellPixels = projectedParentPixels /
        std::exp2(static_cast<float>(result.completeLevels));
    return result;
}

[[nodiscard]] inline float balanceAdaptivePlanetSdfLevel(
    float requestedLevel, float coarsestNeighborRequestedLevel,
    std::uint32_t maximumLevels = kAdaptivePlanetSdfMaximumLevels) noexcept {
    if (!std::isfinite(requestedLevel) ||
        !std::isfinite(coarsestNeighborRequestedLevel)) {
        return 0.0F;
    }
    const float maximum = static_cast<float>(
        std::min(maximumLevels, kAdaptivePlanetSdfMaximumLevels));
    return std::clamp(
        std::min(requestedLevel, coarsestNeighborRequestedLevel + 1.0F),
        0.0F, maximum);
}

[[nodiscard]] inline std::uint32_t adaptiveSdfHash(std::uint32_t value) noexcept {
    value ^= value >> 16U;
    value *= 0x7feb352dU;
    value ^= value >> 15U;
    value *= 0x846ca68bU;
    return value ^ (value >> 16U);
}

[[nodiscard]] inline float adaptiveSdfPhase(std::uint32_t seed,
                                            std::uint32_t lane) noexcept {
    const std::uint32_t bits = adaptiveSdfHash(seed ^ (0x9e3779b9U * (lane + 1U)));
    return static_cast<float>(bits & 0x00ffffffU) *
        (6.2831853071795864769F / 16777216.0F);
}

[[nodiscard]] inline AdaptivePlanetSdfField evaluateAdaptivePlanetSdf(
    const std::array<float, 3>& direction, std::uint32_t seed,
    float continuousLevel,
    float layerFraction = kAdaptivePlanetSdfDefaultLayerFraction) noexcept {
    constexpr std::array<float, 3> basis0{
        0.5773502692F, 0.5773502692F, 0.5773502692F};
    constexpr std::array<float, 3> basis1{
        0.7071067812F, -0.7071067812F, 0.0F};
    constexpr std::array<float, 3> basis2{
        0.4082482905F, 0.4082482905F, -0.8164965809F};
    const auto dot = [](const std::array<float, 3>& left,
                        const std::array<float, 3>& right) noexcept {
        return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
    };

    AdaptivePlanetSdfField result{};
    if (!std::isfinite(continuousLevel) || !std::isfinite(layerFraction)) {
        result.residualAmplitudeBound = kAdaptivePlanetSdfMaximumLayerFraction;
        return result;
    }
    continuousLevel = std::clamp(
        continuousLevel, 0.0F,
        static_cast<float>(kAdaptivePlanetSdfMaximumLevels));
    layerFraction = std::clamp(
        layerFraction, 0.0F, kAdaptivePlanetSdfMaximumLayerFraction);
    float frequency = 384.0F;
    float amplitude = layerFraction * 0.5F;
    for (std::uint32_t octave = 0U;
         octave < kAdaptivePlanetSdfMaximumLevels; ++octave) {
        const float weight = adaptiveSdfSmoothstep(
            continuousLevel - static_cast<float>(octave));
        if (!(weight > 0.0F)) {
            amplitude *= 0.5F;
            frequency *= 2.0F;
            continue;
        }
        const float a = frequency * dot(direction, basis0) +
                        adaptiveSdfPhase(seed, octave * 3U + 0U);
        const float b = frequency * dot(direction, basis1) +
                        adaptiveSdfPhase(seed, octave * 3U + 1U);
        const float c = frequency * dot(direction, basis2) +
                        adaptiveSdfPhase(seed, octave * 3U + 2U);
        const std::array<float, 3> sine{std::sin(a), std::sin(b), std::sin(c)};
        const std::array<float, 3> cosine{std::cos(a), std::cos(b), std::cos(c)};
        const float product = sine[0] * sine[1] * sine[2];
        const float weightedAmplitude = amplitude * weight;
        result.depthLayerFraction += weightedAmplitude * (0.5F + 0.5F * product);
        result.evaluatedAmplitude += weightedAmplitude;
        for (std::size_t axis = 0U; axis < 3U; ++axis) {
            result.directionGradient[axis] += weightedAmplitude * 0.5F * frequency *
                (cosine[0] * sine[1] * sine[2] * basis0[axis] +
                 sine[0] * cosine[1] * sine[2] * basis1[axis] +
                 sine[0] * sine[1] * cosine[2] * basis2[axis]);
        }
        amplitude *= 0.5F;
        frequency *= 2.0F;
    }
    result.residualAmplitudeBound = std::max(
        layerFraction - result.evaluatedAmplitude, 0.0F);
    return result;
}

[[nodiscard]] inline AdaptivePlanetSdfCellAddress adaptivePlanetSdfCellAddress(
    const std::array<float, 3>& direction, std::uint32_t level) noexcept {
    level = std::min(level, kAdaptivePlanetSdfMaximumLevels);
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
        return std::min(
            static_cast<std::uint32_t>(normalized * static_cast<float>(resolution)),
            resolution - 1U);
    };
    return {face, level, coordinate(u), coordinate(v)};
}

[[nodiscard]] inline float adaptivePlanetSdfSupport(float continuousLevel,
                                                    float centerRatio,
                                                    float incidence) noexcept {
    if (!std::isfinite(continuousLevel) || !std::isfinite(centerRatio) ||
        !std::isfinite(incidence) || incidence <= 0.35F) {
        return 0.0F;
    }
    const float level = adaptiveSdfSmoothstep(continuousLevel);
    const float interior = 1.0F - adaptiveSdfSmoothstep(
        (centerRatio - 0.62F) / (0.78F - 0.62F));
    const float facing = adaptiveSdfSmoothstep(
        (incidence - 0.35F) / (0.65F - 0.35F));
    return std::clamp(level * interior * facing, 0.0F, 1.0F);
}

[[nodiscard]] inline AdaptivePlanetSdfRoot solveAdaptivePlanetSdfRoot(
    const std::array<float, 3>& basePosition,
    const std::array<float, 3>& capNormal,
    const std::array<float, 3>& rayDirection,
    float layerHeight, float activation, float continuousLevel,
    std::uint32_t seed,
    float layerFraction = kAdaptivePlanetSdfDefaultLayerFraction) noexcept {
    const auto dot = [](const std::array<float, 3>& left,
                        const std::array<float, 3>& right) noexcept {
        return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
    };
    AdaptivePlanetSdfRoot result{};
    result.normal = capNormal;
    const float incidence = -dot(capNormal, rayDirection);
    if (!(incidence > 0.35F) || !(layerHeight > 0.0F) ||
        !(activation > 0.0F) || !std::isfinite(incidence) ||
        !std::isfinite(layerHeight) || !std::isfinite(activation)) {
        return result;
    }
    layerFraction = std::clamp(
        layerFraction, 0.0F, kAdaptivePlanetSdfMaximumLayerFraction);
    float low = 0.0F;
    float high = layerHeight * layerFraction * activation / incidence;
    AdaptivePlanetSdfField field{};
    std::array<float, 3> sample{};
    for (std::uint32_t iteration = 0U;
         iteration < kAdaptivePlanetSdfRootIterations; ++iteration) {
        const float candidate = 0.5F * (low + high);
        for (std::size_t axis = 0U; axis < 3U; ++axis) {
            sample[axis] = basePosition[axis] + rayDirection[axis] * candidate;
        }
        const float radiusSquared = dot(sample, sample);
        if (!(radiusSquared > 0.0F) || !std::isfinite(radiusSquared)) {
            return {};
        }
        const float inverseRadius = 1.0F / std::sqrt(radiusSquared);
        std::array<float, 3> direction{
            sample[0] * inverseRadius, sample[1] * inverseRadius,
            sample[2] * inverseRadius};
        field = evaluateAdaptivePlanetSdf(
            direction, seed, continuousLevel, layerFraction);
        std::array<float, 3> delta{
            sample[0] - basePosition[0], sample[1] - basePosition[1],
            sample[2] - basePosition[2]};
        const float implicit = dot(capNormal, delta) +
            layerHeight * activation * field.depthLayerFraction;
        if (!std::isfinite(implicit)) {
            return {};
        }
        if (implicit > 0.0F) {
            low = candidate;
        } else {
            high = candidate;
        }
    }
    result.rayOffset = 0.5F * (low + high);
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        sample[axis] = basePosition[axis] +
                       rayDirection[axis] * result.rayOffset;
    }
    const float radiusSquared = dot(sample, sample);
    if (!(radiusSquared > 0.0F) || !std::isfinite(radiusSquared)) {
        return {};
    }
    const float inverseRadius = 1.0F / std::sqrt(radiusSquared);
    std::array<float, 3> direction{
        sample[0] * inverseRadius, sample[1] * inverseRadius,
        sample[2] * inverseRadius};
    field = evaluateAdaptivePlanetSdf(
        direction, seed, continuousLevel, layerFraction);
    const float gradientProjection = dot(field.directionGradient, direction);
    std::array<float, 3> candidateNormal{};
    float normalLengthSquared = 0.0F;
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        const float tangentGradient = field.directionGradient[axis] -
                                      direction[axis] * gradientProjection;
        candidateNormal[axis] = capNormal[axis] + tangentGradient *
            (layerHeight * activation * inverseRadius);
        normalLengthSquared += candidateNormal[axis] * candidateNormal[axis];
    }
    if (!(normalLengthSquared > 0.0F) || !std::isfinite(normalLengthSquared)) {
        return {};
    }
    const float inverseNormalLength = 1.0F / std::sqrt(normalLengthSquared);
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        result.normal[axis] = candidateNormal[axis] * inverseNormalLength;
    }
    result.hit = true;
    result.iterations = kAdaptivePlanetSdfRootIterations;
    return result;
}

[[nodiscard]] inline std::uint32_t packVisualizationAndAdaptiveSdf(
    std::uint32_t visualizationMode, bool enabled, std::uint32_t maximumLevels,
    float targetPixels, std::uint32_t debugMode, float layerFraction,
    bool comparisonView = false) noexcept {
    const std::uint32_t targetQuantized = static_cast<std::uint32_t>(std::lround(
        std::clamp(targetPixels, 0.125F, 31.875F) * 8.0F));
    const std::uint32_t strengthQuantized = static_cast<std::uint32_t>(std::lround(
        std::clamp(layerFraction, 0.0F, kAdaptivePlanetSdfMaximumLayerFraction) *
        (255.0F / kAdaptivePlanetSdfMaximumLayerFraction)));
    return (visualizationMode & 0xffU) |
           (enabled ? 1U << 8U : 0U) |
           ((std::min(maximumLevels, 15U) & 0xfU) << 9U) |
           ((targetQuantized & 0xffU) << 13U) |
           ((debugMode & 0x3U) << 21U) |
           ((strengthQuantized & 0xffU) << 23U) |
           (comparisonView ? 1U << 31U : 0U);
}

} // namespace voxel
