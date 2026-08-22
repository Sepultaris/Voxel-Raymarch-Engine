#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace voxel {

enum class CloudDensitySource : std::uint32_t {
    Procedural3D = 0U,
    Authored3DTexture = 1U,
};

struct VolumetricCloudSettings {
    bool enabled{true};
    CloudDensitySource source{CloudDensitySource::Procedural3D};
    float coverage{0.46F};
    float density{1.15F};
    float baseAltitude{0.065F};
    float topAltitude{0.135F};
    float shapeScale{6.0F};
    float detailStrength{0.32F};
    float windSpeed{};
    float windDirection{0.35F};
    std::uint32_t quality{1U};
    std::uint32_t sunShadowQuality{1U};
    float sunShadowStrength{0.78F};
    std::uint32_t debugMode{};
};

struct CloudShellSegments {
    std::array<float, 2> nearDistance{};
    std::array<float, 2> farDistance{};
    std::uint32_t count{};
};

// Returns the positive ray intervals inside the outer cloud sphere but
// outside its hollow inner sphere. A grazing/exterior ray can produce one
// interval; a ray through both sides of the shell produces two. The renderer
// uses the same construction before clipping the intervals to an exact terrain
// hit, so this helper also serves as the CPU regression oracle.
[[nodiscard]] inline CloudShellSegments cloudShellSegments(
    const std::array<float, 3>& origin,
    const std::array<float, 3>& direction,
    float innerRadius, float outerRadius,
    float maximumDistance) noexcept {
    CloudShellSegments result{};
    if (!(innerRadius >= 0.0F) || !(outerRadius > innerRadius) ||
        !(maximumDistance > 0.0F) || !std::isfinite(maximumDistance)) {
        return result;
    }
    const float directionLengthSquared =
        direction[0] * direction[0] + direction[1] * direction[1] +
        direction[2] * direction[2];
    if (!(directionLengthSquared > 0.0F) ||
        !std::isfinite(directionLengthSquared)) {
        return result;
    }
    const auto roots = [&](float radius, float& nearDistance,
                           float& farDistance) noexcept {
        const float b = origin[0] * direction[0] +
                        origin[1] * direction[1] +
                        origin[2] * direction[2];
        const float c = origin[0] * origin[0] + origin[1] * origin[1] +
                        origin[2] * origin[2] - radius * radius;
        const float discriminant = b * b - directionLengthSquared * c;
        if (!(discriminant >= 0.0F) || !std::isfinite(discriminant)) {
            return false;
        }
        const float root = std::sqrt(std::max(discriminant, 0.0F));
        nearDistance = (-b - root) / directionLengthSquared;
        farDistance = (-b + root) / directionLengthSquared;
        return std::isfinite(nearDistance) && std::isfinite(farDistance);
    };

    float outerNear{};
    float outerFar{};
    if (!roots(outerRadius, outerNear, outerFar)) {
        return result;
    }
    outerNear = std::max(outerNear, 0.0F);
    outerFar = std::min(outerFar, maximumDistance);
    if (!(outerFar > outerNear)) {
        return result;
    }

    float innerNear{};
    float innerFar{};
    if (!roots(innerRadius, innerNear, innerFar) ||
        innerFar <= outerNear || innerNear >= outerFar) {
        result.nearDistance[0] = outerNear;
        result.farDistance[0] = outerFar;
        result.count = 1U;
        return result;
    }
    const float firstFar = std::min(outerFar, innerNear);
    if (firstFar > outerNear) {
        result.nearDistance[result.count] = outerNear;
        result.farDistance[result.count] = firstFar;
        ++result.count;
    }
    const float secondNear = std::max(outerNear, innerFar);
    if (outerFar > secondNear) {
        result.nearDistance[result.count] = secondNear;
        result.farDistance[result.count] = outerFar;
        ++result.count;
    }
    return result;
}

[[nodiscard]] inline std::uint32_t quantizeCloud(float value,
                                                 float maximum) noexcept {
    if (!(maximum > 0.0F) || !std::isfinite(value)) {
        return 0U;
    }
    return static_cast<std::uint32_t>(std::lround(
        std::clamp(value / maximum, 0.0F, 1.0F) * 255.0F));
}

[[nodiscard]] inline std::uint32_t packCloudWord0(
    const VolumetricCloudSettings& settings) noexcept {
    return quantizeCloud(settings.coverage, 1.0F) |
           (quantizeCloud(settings.density, 3.0F) << 8U) |
           (quantizeCloud(settings.baseAltitude, 0.25F) << 16U) |
           (quantizeCloud(settings.topAltitude, 0.30F) << 24U);
}

[[nodiscard]] inline std::uint32_t packCloudWord1(
    const VolumetricCloudSettings& settings) noexcept {
    const auto shape = quantizeCloud(
        std::max(settings.shapeScale - 1.0F, 0.0F), 15.0F);
    const auto detail = quantizeCloud(settings.detailStrength, 1.0F);
    const auto wind = quantizeCloud(settings.windSpeed, 0.10F);
    const std::uint32_t controls =
        (settings.enabled ? 1U : 0U) |
        (settings.source == CloudDensitySource::Authored3DTexture ? 1U << 1U : 0U) |
        ((std::min(settings.quality, 3U) & 0x3U) << 2U) |
        ((std::min(settings.sunShadowQuality, 3U) & 0x3U) << 4U) |
        ((std::min(settings.debugMode, 3U) & 0x3U) << 6U);
    return shape | (detail << 8U) | (wind << 16U) | (controls << 24U);
}

[[nodiscard]] inline std::uint32_t packCloudPhaseWord(
    std::uint32_t preservedLowByte,
    const VolumetricCloudSettings& settings) noexcept {
    constexpr float twoPi = 6.28318530717958647692F;
    float direction = std::fmod(settings.windDirection, twoPi);
    if (direction < 0.0F) {
        direction += twoPi;
    }
    return (preservedLowByte & 0xffU) |
           (quantizeCloud(direction, twoPi) << 8U) |
           (quantizeCloud(settings.sunShadowStrength, 1.0F) << 16U);
}

[[nodiscard]] inline float cloudTransmittance(float opticalDepth,
                                              float density) noexcept {
    if (!std::isfinite(opticalDepth) || !std::isfinite(density)) {
        return 0.0F;
    }
    return std::clamp(std::exp(-std::max(opticalDepth, 0.0F) *
                               std::max(density, 0.0F)),
                      0.0F, 1.0F);
}

[[nodiscard]] inline float proceduralCloudDensity(
    const std::array<float, 3>& worldPosition, float planetRadius,
    float baseAltitude, float topAltitude, float coverage, float shapeScale,
    float detailStrength, float time, float windSpeed,
    float windDirection) noexcept {
    if (!(planetRadius > 0.0F) || !(topAltitude > baseAltitude)) {
        return 0.0F;
    }
    const float radius = std::sqrt(
        worldPosition[0] * worldPosition[0] +
        worldPosition[1] * worldPosition[1] +
        worldPosition[2] * worldPosition[2]);
    const float altitude = radius / planetRadius - 1.0F;
    if (altitude < baseAltitude || altitude > topAltitude) {
        return 0.0F;
    }
    const float height = std::clamp(
        (altitude - baseAltitude) / (topAltitude - baseAltitude), 0.0F, 1.0F);
    const float vertical = std::clamp(height / 0.16F, 0.0F, 1.0F) *
        std::clamp((1.0F - height) / 0.28F, 0.0F, 1.0F);
    const float windX = std::cos(windDirection) * windSpeed * time;
    const float windZ = std::sin(windDirection) * windSpeed * time;
    const float x = worldPosition[0] / planetRadius + windX;
    const float y = worldPosition[1] / planetRadius;
    const float z = worldPosition[2] / planetRadius + windZ;
    const float scale = std::max(shapeScale, 1.0F);
    const float shape = 0.5F + 0.25F * std::sin(scale * (1.17F * x + 0.73F * y - 0.91F * z)) +
        0.16F * std::sin(scale * (0.43F * x - 1.31F * y + 0.67F * z) + 1.7F) +
        0.09F * std::sin(scale * 2.13F * (x + y + z) - 0.8F);
    const float detail = 0.5F + 0.5F *
        std::sin(scale * 4.7F * (0.81F * x - 0.57F * y + 1.11F * z));
    const float threshold = 1.0F - std::clamp(coverage, 0.0F, 1.0F);
    return std::clamp((shape - threshold) * 2.4F +
                      (detail - 0.5F) * detailStrength, 0.0F, 1.0F) * vertical;
}

} // namespace voxel
