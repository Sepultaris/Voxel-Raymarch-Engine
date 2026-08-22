#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

namespace voxel {

inline constexpr std::uint32_t kDistanceBoundLevels = 11U;

[[nodiscard]] constexpr std::uint32_t distanceBoundLevelOffset(
    std::uint32_t baseWidth, std::uint32_t baseHeight,
    std::uint32_t level) noexcept {
    std::uint32_t offset = 0U;
    for (std::uint32_t current = 0U; current < level; ++current) {
        offset += baseWidth * baseHeight;
        baseWidth = std::max(baseWidth >> 1U, 1U);
        baseHeight = std::max(baseHeight >> 1U, 1U);
    }
    return offset;
}

[[nodiscard]] constexpr std::uint32_t distanceBoundNodeCount(
    std::uint32_t baseWidth, std::uint32_t baseHeight) noexcept {
    return distanceBoundLevelOffset(baseWidth, baseHeight,
                                    kDistanceBoundLevels);
}

[[nodiscard]] inline double conservativeAngularBoundaryLowerBound(
    double longitudeFraction, double latitudeFraction,
    double latitudeCosine, std::uint32_t width,
    std::uint32_t height) noexcept {
    if (!std::isfinite(longitudeFraction) ||
        !std::isfinite(latitudeFraction) ||
        !std::isfinite(latitudeCosine) || width == 0U || height == 0U) {
        return 0.0;
    }
    const double longitudeMargin =
        std::min(longitudeFraction, 1.0 - longitudeFraction) *
        (2.0 * std::numbers::pi / static_cast<double>(width)) *
        std::abs(latitudeCosine) * 0.99;
    const double latitudeMargin =
        std::min(latitudeFraction, 1.0 - latitudeFraction) *
        (std::numbers::pi / static_cast<double>(height));
    return std::max(std::min(longitudeMargin, latitudeMargin), 0.0);
}

[[nodiscard]] inline double conservativeDistanceStep(
    double radialGap, double angularMargin,
    double minimumRadius) noexcept {
    if (!std::isfinite(radialGap) || !std::isfinite(angularMargin) ||
        !std::isfinite(minimumRadius) || radialGap <= 0.0 ||
        angularMargin <= 0.0 || minimumRadius <= 0.0) {
        return 0.0;
    }
    const double bound = std::min(radialGap, angularMargin * minimumRadius);
    return std::nextafter(bound, 0.0);
}

} // namespace voxel
