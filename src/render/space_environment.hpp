#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace voxel {

struct SpaceEnvironmentSettings {
    bool enabled{true};
    bool cameraAligned{};
    float density{0.42F};
    float brightness{1.0F};
    float size{1.0F};
    bool debug{};
};

[[nodiscard]] inline std::array<int, 3> spaceEnvironmentUnitShellCell(
    std::array<float, 3> direction, int resolution) noexcept {
    const float lengthSquared = direction[0] * direction[0] +
        direction[1] * direction[1] + direction[2] * direction[2];
    if (!(lengthSquared > 0.0F) || !std::isfinite(lengthSquared) ||
        resolution <= 0) {
        return {};
    }
    const float inverseLength = 1.0F / std::sqrt(lengthSquared);
    return {
        static_cast<int>(std::floor(direction[0] * inverseLength *
                                    static_cast<float>(resolution))),
        static_cast<int>(std::floor(direction[1] * inverseLength *
                                    static_cast<float>(resolution))),
        static_cast<int>(std::floor(direction[2] * inverseLength *
                                    static_cast<float>(resolution)))};
}

[[nodiscard]] inline std::array<float, 3> spaceEnvironmentDirection(
    const std::array<float, 3>& worldRay,
    const std::array<float, 3>& cameraLocalRay,
    bool cameraAligned) noexcept {
    return cameraAligned ? cameraLocalRay : worldRay;
}

[[nodiscard]] inline std::uint32_t packSpaceEnvironmentFlags(
    const SpaceEnvironmentSettings& settings) noexcept {
    const auto density = static_cast<std::uint32_t>(std::lround(
        std::clamp(settings.density, 0.0F, 1.0F) * 255.0F));
    return (density << 16U) |
           (settings.enabled ? 1U << 24U : 0U) |
           (settings.cameraAligned ? 1U << 25U : 0U) |
           (settings.debug ? 1U << 26U : 0U);
}

[[nodiscard]] inline float spaceEnvironmentDensity(
    std::uint32_t packed) noexcept {
    return static_cast<float>((packed >> 16U) & 0xffU) / 255.0F;
}

[[nodiscard]] inline float spaceEnvironmentHorizonVisibility(
    float angularClearance, float planetRadius) noexcept {
    if (!(planetRadius > 0.0F) || !std::isfinite(angularClearance)) {
        return 0.0F;
    }
    const float t = std::clamp(
        (angularClearance - planetRadius * 0.08F) /
            (planetRadius * (0.15F - 0.08F)),
        0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

// The planet silhouette is a forward-ray exclusion, not an infinite-line
// exclusion.  Using only the perpendicular distance to the infinite ray line
// suppresses an identical, camera-distance-dependent disk in the antipodal
// sky.  Normalize here so the CPU oracle remains valid for controller and
// lensed-exit directions that carry harmless length drift.
[[nodiscard]] inline float spaceEnvironmentForwardStarVisibility(
    const std::array<float, 3>& cameraPosition,
    const std::array<float, 3>& rayDirection, float planetOuterRadius,
    float planetRadius) noexcept {
    const float directionLengthSquared =
        rayDirection[0] * rayDirection[0] +
        rayDirection[1] * rayDirection[1] +
        rayDirection[2] * rayDirection[2];
    const bool finiteInput =
        std::isfinite(cameraPosition[0]) &&
        std::isfinite(cameraPosition[1]) &&
        std::isfinite(cameraPosition[2]) &&
        std::isfinite(directionLengthSquared) &&
        std::isfinite(planetOuterRadius) &&
        std::isfinite(planetRadius);
    if (!finiteInput || !(directionLengthSquared > 0.0F) ||
        !(planetOuterRadius > 0.0F) || !(planetRadius > 0.0F)) {
        return 0.0F;
    }

    const float inverseDirectionLength =
        1.0F / std::sqrt(directionLengthSquared);
    const std::array<float, 3> direction{
        rayDirection[0] * inverseDirectionLength,
        rayDirection[1] * inverseDirectionLength,
        rayDirection[2] * inverseDirectionLength};
    const float closestDistance = -(
        cameraPosition[0] * direction[0] +
        cameraPosition[1] * direction[1] +
        cameraPosition[2] * direction[2]);
    if (!(closestDistance > 0.0F)) {
        return 1.0F;
    }

    const std::array<float, 3> closestPoint{
        cameraPosition[0] + direction[0] * closestDistance,
        cameraPosition[1] + direction[1] * closestDistance,
        cameraPosition[2] + direction[2] * closestDistance};
    const float closestRadius = std::sqrt(
        closestPoint[0] * closestPoint[0] +
        closestPoint[1] * closestPoint[1] +
        closestPoint[2] * closestPoint[2]);
    return spaceEnvironmentHorizonVisibility(
        closestRadius - planetOuterRadius, planetRadius);
}

[[nodiscard]] inline float spaceEnvironmentStarFootprint(
    float angularDistance, float radius, float pixelAngularWidth) noexcept {
    if (!std::isfinite(angularDistance) || !std::isfinite(radius) ||
        !std::isfinite(pixelAngularWidth) || pixelAngularWidth <= 0.0F) {
        return 0.0F;
    }
    const float edge = radius + pixelAngularWidth;
    const float t = std::clamp(
        (angularDistance - radius) / std::max(edge - radius, 1.0e-9F),
        0.0F, 1.0F);
    const float smooth = t * t * (3.0F - 2.0F * t);
    return 1.0F - smooth;
}

[[nodiscard]] inline float spaceEnvironmentLensedAngularFootprint(
    float basePixelAngularWidth, float mappedAngularDelta,
    float inputAngularDelta) noexcept {
    if (!(basePixelAngularWidth > 0.0F) ||
        !(inputAngularDelta > 0.0F) ||
        !std::isfinite(basePixelAngularWidth) ||
        !std::isfinite(mappedAngularDelta) ||
        !std::isfinite(inputAngularDelta)) {
        return 0.0F;
    }
    const float scale = std::clamp(
        std::abs(mappedAngularDelta) / inputAngularDelta, 0.5F, 8.0F);
    return basePixelAngularWidth * std::max(scale, 1.0F);
}

} // namespace voxel
