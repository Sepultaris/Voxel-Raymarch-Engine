#pragma once

#include "planet/fractal_planet_sdf.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace voxel {

inline constexpr float kFractalVoxelSplitPixels = 1.60F;
inline constexpr float kFractalVoxelMergePixels = 0.72F;

struct FractalVoxelCacheState {
    std::uint32_t level{};
    std::uint64_t refinements{};
    std::uint64_t merges{};
    float projectedPixels{};
    float transition{};
};

[[nodiscard]] inline FractalVoxelCacheState updateFractalVoxelCache(
    FractalVoxelCacheState state, float cameraDistanceToSurface,
    float planetRadius, std::uint32_t resolutionY, float targetPixels,
    std::uint32_t maximumLevels) noexcept {
    maximumLevels = std::min(maximumLevels, kFractalPlanetMaximumLevels);
    if (!(cameraDistanceToSurface > 0.0F) || !(planetRadius > 0.0F) ||
        resolutionY == 0U || !std::isfinite(cameraDistanceToSurface) ||
        !std::isfinite(planetRadius)) {
        return state;
    }
    const float projectedParent = planetRadius *
        kFractalPlanetParentAngularWidth *
        (static_cast<float>(resolutionY) * 0.5F) /
        cameraDistanceToSurface;
    const auto leafPixels = [&]() noexcept {
        return projectedParent /
            static_cast<float>(std::uint32_t{1U} << state.level);
    };
    while (state.level < maximumLevels &&
           leafPixels() > kFractalVoxelSplitPixels) {
        ++state.level;
        ++state.refinements;
    }
    while (state.level > 0U &&
           leafPixels() < kFractalVoxelMergePixels) {
        --state.level;
        ++state.merges;
    }
    state.projectedPixels = leafPixels();
    if (state.level < maximumLevels) {
        const float denominator = std::max(
            kFractalVoxelSplitPixels - targetPixels, 0.05F);
        const float t = std::clamp(
            (state.projectedPixels - targetPixels) / denominator,
            0.0F, 1.0F);
        state.transition = t * t * (3.0F - 2.0F * t);
    } else {
        state.transition = 0.0F;
    }
    return state;
}

[[nodiscard]] inline float fractalVoxelSourceHeight(
    const std::array<float, 3>& direction, std::uint32_t seed,
    std::uint32_t parentLevel, float transition,
    float microRelief = kFractalPlanetDefaultMicroRelief) noexcept {
    const float continuousLevel = std::clamp(
        static_cast<float>(parentLevel) + transition, 0.0F,
        static_cast<float>(kFractalPlanetMaximumLevels));
    const auto field = evaluateFractalPlanetField(
        direction, seed, continuousLevel, microRelief,
        static_cast<std::uint32_t>(std::ceil(continuousLevel)));
    return field.surfaceRadiusScale;
}

[[nodiscard]] inline float fractalVoxelBilinear(
    float h00, float h10, float h01, float h11,
    float x, float y) noexcept {
    x = std::clamp(x, 0.0F, 1.0F);
    y = std::clamp(y, 0.0F, 1.0F);
    return std::lerp(std::lerp(h00, h10, x),
                     std::lerp(h01, h11, x), y);
}

[[nodiscard]] inline float fractalVoxelDiscreteCellHeight(
    float h00, float h10, float h01, float h11) noexcept {
    return 0.25F * (h00 + h10 + h01 + h11);
}

[[nodiscard]] inline float fractalVoxelMaterializedHeight(
    float h00, float h10, float h01, float h11,
    float localX, float localY, float cacheInterior,
    bool discreteSurface) noexcept {
    const float smooth = fractalVoxelBilinear(
        h00, h10, h01, h11, localX, localY);
    if (!discreteSurface) {
        return smooth;
    }
    return std::lerp(
        smooth, fractalVoxelDiscreteCellHeight(h00, h10, h01, h11),
        std::clamp(cacheInterior, 0.0F, 1.0F));
}

[[nodiscard]] inline float fractalVoxelBandCavity(
    float finestBandShape, float activationWeight) noexcept {
    if (!std::isfinite(finestBandShape) ||
        !std::isfinite(activationWeight)) {
        return 0.0F;
    }
    const float scaled = std::max(-finestBandShape, 0.0F) *
                         std::clamp(activationWeight, 0.0F, 1.0F);
    const float t = std::clamp(
        (scaled - 0.08F) / (0.72F - 0.08F), 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

[[nodiscard]] inline float fractalVoxelLocalCavityAo(
    float cavity, float strength) noexcept {
    return std::clamp(
        1.0F - std::clamp(strength, 0.0F, 0.45F) *
            std::clamp(cavity, 0.0F, 1.0F),
        0.55F, 1.0F);
}

[[nodiscard]] inline std::uint32_t packFractalVoxelCacheSample(
    float height, float cavity) noexcept {
    constexpr float heightMinimum = 0.96F;
    constexpr float heightRange = 0.10F;
    const auto quantizedHeight = static_cast<std::uint32_t>(std::lround(
        std::clamp((height - heightMinimum) / heightRange, 0.0F, 1.0F) *
        16777215.0F));
    const auto quantizedCavity = static_cast<std::uint32_t>(std::lround(
        std::clamp(cavity, 0.0F, 1.0F) * 255.0F));
    return (quantizedCavity << 24U) | (quantizedHeight & 0x00ffffffU);
}

[[nodiscard]] inline float unpackFractalVoxelCacheHeight(
    std::uint32_t packed) noexcept {
    return 0.96F + static_cast<float>(packed & 0x00ffffffU) *
        (0.10F / 16777215.0F);
}

[[nodiscard]] inline float unpackFractalVoxelCacheCavity(
    std::uint32_t packed) noexcept {
    return static_cast<float>(packed >> 24U) * (1.0F / 255.0F);
}

// Zero is reserved for cache memory that has not been generated yet. The
// procedural field used by the lab is comfortably above the minimum encoded
// height, so no valid generated sample aliases this sentinel.
[[nodiscard]] inline bool fractalVoxelCacheSampleValid(
    std::uint32_t packed) noexcept {
    return packed != 0U;
}

[[nodiscard]] inline bool fractalVoxelCacheCoordinateValid(
    float x, float y, std::uint32_t resolution) noexcept {
    if (resolution < 4U || !std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }
    const float maximum = static_cast<float>(resolution - 2U);
    return x >= 1.0F && y >= 1.0F && x <= maximum && y <= maximum;
}

[[nodiscard]] inline float fractalVoxelCacheInteriorBlend(
    float x, float y, std::uint32_t resolution) noexcept {
    if (!fractalVoxelCacheCoordinateValid(x, y, resolution)) {
        return 0.0F;
    }
    const float maximum = static_cast<float>(resolution - 2U);
    const float border = std::min(
        std::min(x - 1.0F, y - 1.0F),
        std::min(maximum - x, maximum - y));
    const float t = std::clamp((border - 2.0F) / 16.0F, 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

[[nodiscard]] inline float fractalVoxelCellEdgeVisibility(
    float projectedPixels) noexcept {
    if (!std::isfinite(projectedPixels) || projectedPixels <= 1.45F) {
        return 0.0F;
    }
    const float t = std::clamp(
        (projectedPixels - 1.45F) / (2.50F - 1.45F), 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

[[nodiscard]] inline std::array<float, 3> normalizedFractalVoxelDirection(
    const std::array<float, 3>& center,
    const std::array<float, 3>& right,
    const std::array<float, 3>& forward,
    float tangentX, float tangentY, float planetRadius) noexcept {
    std::array<float, 3> result{
        center[0] + (right[0] * tangentX + forward[0] * tangentY) / planetRadius,
        center[1] + (right[1] * tangentX + forward[1] * tangentY) / planetRadius,
        center[2] + (right[2] * tangentX + forward[2] * tangentY) / planetRadius};
    const float lengthSquared = result[0] * result[0] +
        result[1] * result[1] + result[2] * result[2];
    if (!(lengthSquared > 0.0F) || !std::isfinite(lengthSquared)) {
        return center;
    }
    const float inverseLength = 1.0F / std::sqrt(lengthSquared);
    result[0] *= inverseLength;
    result[1] *= inverseLength;
    result[2] *= inverseLength;
    return result;
}

} // namespace voxel
