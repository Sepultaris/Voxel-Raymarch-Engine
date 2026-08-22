#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace voxel {

inline constexpr float kLocalGeodesicAoMaximumStrength = 0.30F;
inline constexpr float kLocalGeodesicAoDefaultStrength = 0.12F;
inline constexpr std::uint32_t kInvalidAoNeighborSlot = 0xffffffffU;

struct LocalGeodesicAoSample {
    std::array<std::uint32_t, 6> neighborHeights{};
    std::uint32_t neighborCount{6U};
    std::uint32_t columnHeight{};
    std::uint32_t hitLayer{};
    std::uint32_t sideNeighborSlot{kInvalidAoNeighborSlot};
    float horizontalRunLayers{2.0F};
};

[[nodiscard]] inline float localGeodesicHorizonSample(
    std::uint32_t neighborHeight, std::uint32_t surfaceLevel,
    float horizontalRunLayers) noexcept {
    const float rise = static_cast<float>(neighborHeight > surfaceLevel
                                              ? neighborHeight - surfaceLevel
                                              : 0U);
    const float run = std::max(horizontalRunLayers, 0.25F);
    return rise > 0.0F ? rise / std::sqrt(rise * rise + run * run) : 0.0F;
}

[[nodiscard]] inline float localGeodesicAoCue(
    const LocalGeodesicAoSample& input) noexcept {
    const std::uint32_t count = std::clamp(input.neighborCount, 5U, 6U);
    const bool sideHit = input.sideNeighborSlot < count;
    const std::uint32_t surfaceLevel = std::min(input.hitLayer + 1U,
                                                input.columnHeight);
    std::array<float, 6> horizon{};
    std::array<float, 6> weights{};
    float horizonSum = 0.0F;
    float weightSum = 0.0F;

    for (std::uint32_t slot = 0U; slot < count; ++slot) {
        float weight = 1.0F;
        if (sideHit) {
            const std::uint32_t forward = (slot + count - input.sideNeighborSlot) % count;
            const std::uint32_t ringDistance = std::min(forward, count - forward);
            weight = ringDistance == 0U ? 0.0F
                   : ringDistance == 1U ? 1.0F
                   : ringDistance == 2U ? 0.35F
                                        : 0.15F;
        }
        horizon[slot] = localGeodesicHorizonSample(
            input.neighborHeights[slot], surfaceLevel,
            input.horizontalRunLayers);
        weights[slot] = weight;
        horizonSum += horizon[slot] * weight;
        weightSum += weight;
    }

    const float horizonMean = weightSum > 0.0F ? horizonSum / weightSum : 0.0F;
    float wedgeSum = 0.0F;
    float wedgeWeightSum = 0.0F;
    for (std::uint32_t slot = 0U; slot < count; ++slot) {
        const std::uint32_t next = (slot + 1U) % count;
        const float weight = std::min(weights[slot], weights[next]);
        wedgeSum += std::sqrt(horizon[slot] * horizon[next]) * weight;
        wedgeWeightSum += weight;
    }
    const float wedgeMean = wedgeWeightSum > 0.0F
                                ? wedgeSum / wedgeWeightSum
                                : 0.0F;
    float raw = 0.62F * horizonMean + 0.38F * wedgeMean;
    if (sideHit) {
        // Side walls receive only a restrained corner cue. This preserves real
        // concave steps without turning long exposed cliffs into dark stripes.
        raw = std::min(raw * 0.65F, 0.75F);
    }
    const float normalized = std::clamp((raw - 0.03F) / (0.85F - 0.03F),
                                        0.0F, 1.0F);
    return normalized * normalized * (3.0F - 2.0F * normalized);
}

[[nodiscard]] inline float localGeodesicAoFactor(
    const LocalGeodesicAoSample& input, float strength) noexcept {
    return 1.0F - std::clamp(strength, 0.0F,
                             kLocalGeodesicAoMaximumStrength) *
                      localGeodesicAoCue(input);
}

[[nodiscard]] inline float ddaWorkHeatmapCue(std::uint32_t leafEvents) noexcept {
    const float normalized = std::clamp(static_cast<float>(leafEvents) / 256.0F,
                                        0.0F, 1.0F);
    const float t = std::clamp((normalized - 0.08F) / (0.65F - 0.08F),
                               0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

[[nodiscard]] inline std::uint32_t packMarchAndGeodesicShading(
    std::uint32_t maxMarchSteps, bool aoEnabled, float aoStrength,
    bool aoDebugVisualization, bool ddaWorkHeatmap,
    bool microSdfEnabled = false, float microSdfStrength = 0.0F,
    bool microSdfDebug = false) noexcept {
    const auto quantized = static_cast<std::uint32_t>(std::lround(
        std::clamp(aoStrength, 0.0F, kLocalGeodesicAoMaximumStrength) *
        (255.0F / kLocalGeodesicAoMaximumStrength)));
    const auto microQuantized = static_cast<std::uint32_t>(std::lround(
        std::clamp(microSdfStrength, 0.0F, 0.175F) * (7.0F / 0.175F)));
    return (maxMarchSteps & 0xffffU) | ((quantized & 0xffU) << 16U) |
           (aoEnabled ? 1U << 24U : 0U) |
           (aoDebugVisualization ? 1U << 25U : 0U) |
           (ddaWorkHeatmap ? 1U << 26U : 0U) |
           (microSdfEnabled ? 1U << 27U : 0U) |
           (microSdfDebug ? 1U << 28U : 0U) |
           ((microQuantized & 0x7U) << 29U);
}

} // namespace voxel
