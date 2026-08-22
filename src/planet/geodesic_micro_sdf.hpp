#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>

namespace voxel {

inline constexpr std::uint32_t kTerrainMicroSdfOctaves = 3U;
inline constexpr std::uint32_t kTerrainMicroSdfRefinementIterations = 2U;
inline constexpr float kTerrainMicroSdfMaximumLayerFraction = 0.175F;

struct TerrainMicroSdfField {
    float depth01{};
    std::array<float, 3> directionGradient{};
};

[[nodiscard]] inline TerrainMicroSdfField evaluateTerrainMicroSdfField(
    const std::array<float, 3>& direction) noexcept {
    constexpr std::array<float, 3> basis0{0.5773502692F, 0.5773502692F,
                                          0.5773502692F};
    constexpr std::array<float, 3> basis1{0.7071067812F, -0.7071067812F, 0.0F};
    constexpr std::array<float, 3> basis2{0.4082482905F, 0.4082482905F,
                                          -0.8164965809F};
    const auto dot = [](const std::array<float, 3>& left,
                        const std::array<float, 3>& right) noexcept {
        return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
    };
    float value = 0.0F;
    std::array<float, 3> gradient{};
    float amplitude = 0.5F;
    float frequency = 512.0F;
    std::array<float, 3> phase{0.37F, 1.11F, 2.03F};
    for (std::uint32_t octave = 0U; octave < kTerrainMicroSdfOctaves; ++octave) {
        const float a = frequency * dot(direction, basis0) + phase[0];
        const float b = frequency * dot(direction, basis1) + phase[1];
        const float c = frequency * dot(direction, basis2) + phase[2];
        const std::array<float, 3> sine{std::sin(a), std::sin(b), std::sin(c)};
        const std::array<float, 3> cosine{std::cos(a), std::cos(b), std::cos(c)};
        value += amplitude * sine[0] * sine[1] * sine[2];
        for (std::size_t axis = 0U; axis < 3U; ++axis) {
            gradient[axis] += amplitude * frequency *
                (cosine[0] * sine[1] * sine[2] * basis0[axis] +
                 sine[0] * cosine[1] * sine[2] * basis1[axis] +
                 sine[0] * sine[1] * cosine[2] * basis2[axis]);
        }
        amplitude *= 0.5F;
        frequency *= 2.0F;
        phase[0] += 1.73F;
        phase[1] += 2.41F;
        phase[2] += 0.89F;
    }
    constexpr float amplitudeSum = 0.875F;
    TerrainMicroSdfField result{};
    result.depth01 = 0.5F + 0.5F *
        std::clamp(value / amplitudeSum, -1.0F, 1.0F);
    for (std::size_t axis = 0U; axis < 3U; ++axis) {
        result.directionGradient[axis] = 0.5F * gradient[axis] / amplitudeSum;
    }
    return result;
}

[[nodiscard]] inline float terrainMicroSdfActivation(
    float projectedCellPixels, float centerRatio, float incidence,
    std::uint32_t material, bool edited) noexcept {
    if (material == 3U || edited || !std::isfinite(projectedCellPixels) ||
        !std::isfinite(centerRatio) || !std::isfinite(incidence) ||
        incidence <= 0.35F) {
        return 0.0F;
    }
    const auto smoothstep = [](float low, float high, float value) noexcept {
        const float t = std::clamp((value - low) / (high - low), 0.0F, 1.0F);
        return t * t * (3.0F - 2.0F * t);
    };
    const float footprint = smoothstep(5.0F, 14.0F, projectedCellPixels);
    const float interior = 1.0F - smoothstep(0.45F, 0.78F, centerRatio);
    const float facing = smoothstep(0.35F, 0.65F, incidence);
    return std::clamp(footprint * interior * facing, 0.0F, 1.0F);
}

[[nodiscard]] inline float terrainMicroSdfRayOffset(
    float layerHeight, float strength, float activation,
    float depth01, float incidence) noexcept {
    if (!std::isfinite(layerHeight) || !std::isfinite(strength) ||
        !std::isfinite(activation) || !std::isfinite(depth01) ||
        !std::isfinite(incidence) || layerHeight <= 0.0F ||
        incidence <= 0.35F) {
        return 0.0F;
    }
    const float maximumDepth = layerHeight *
        std::clamp(strength, 0.0F, kTerrainMicroSdfMaximumLayerFraction) *
        std::clamp(activation, 0.0F, 1.0F);
    return std::clamp(maximumDepth * std::clamp(depth01, 0.0F, 1.0F) /
                          incidence,
                      0.0F, maximumDepth / incidence);
}

} // namespace voxel
