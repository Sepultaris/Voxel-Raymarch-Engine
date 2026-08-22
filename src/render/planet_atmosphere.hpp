#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace voxel {

// A deliberately compact production control set. The GPU evaluates a real
// spherical participating-medium segment; these values only select the shell
// and its optical density. Scattering colors follow the production sky/sun so
// lighting and atmosphere cannot drift into unrelated palettes.
struct PlanetAtmosphereSettings {
    bool enabled{true};
    bool physicalSurface{true};
    bool artisticNearFade{};
    float startHeight{0.0F};
    float endHeight{0.18F};
    float density{1.0F};
    float rayleighScattering{1.0F};
    float mieScattering{0.58F};
    float absorption{0.18F};
    float scaleHeight{0.26F};
    float mieAnisotropy{0.68F};
    // User-authored SpaceBattleSimulator depth-aware integration, adapted to
    // this engine's single exact terrain endpoint. The legacy one-sample path
    // remains available for deterministic A/B and very low-end hardware.
    bool depthIntegrated{true};
    std::uint32_t integrationQuality{2U};
    float nightScattering{0.035F};
    bool terrainShadows{true};
    std::uint32_t terrainShadowQuality{2U};
    float terrainShadowDistance{0.055F};
    float terrainShadowStrength{0.72F};
    std::uint32_t debugMode{};
};

struct AtmosphereRayInterval {
    bool intersects{};
    float nearDistance{};
    float farDistance{};
};

struct OrderedMediumSample {
    std::array<float, 3> inScattering{};
    std::array<float, 3> transmittance{1.0F, 1.0F, 1.0F};
};

[[nodiscard]] inline std::array<float, 3> composeOrderedMediumSegment(
    const std::array<float, 3>& radianceBehind,
    const OrderedMediumSample& foreground) noexcept {
    std::array<float, 3> result{};
    for (std::size_t channel = 0; channel < result.size(); ++channel) {
        const float transmittance = std::clamp(
            foreground.transmittance[channel], 0.0F, 1.0F);
        result[channel] = foreground.inScattering[channel] +
                          radianceBehind[channel] * transmittance;
    }
    return result;
}

[[nodiscard]] inline std::array<float, 3> composeOrderedWormholeMedia(
    const std::array<float, 3>& destinationRadiance,
    const OrderedMediumSample& destination,
    const OrderedMediumSample& source) noexcept {
    return composeOrderedMediumSegment(
        composeOrderedMediumSegment(destinationRadiance, destination), source);
}

constexpr float kAtmosphereMaximumStartHeight = 0.04F;
constexpr float kAtmosphereMaximumEndHeight = 0.30F;
constexpr float kAtmosphereMaximumDensity = 3.0F;
constexpr float kAtmosphereMaximumCoefficient = 3.0F;
constexpr float kAtmosphereMinimumScaleHeight = 0.08F;
constexpr float kAtmosphereMaximumScaleHeight = 0.65F;
constexpr float kAtmosphereMinimumMieAnisotropy = -0.20F;
constexpr float kAtmosphereMaximumMieAnisotropy = 0.90F;
constexpr float kAtmosphereMinimumTerrainShadowDistance = 0.01F;
constexpr float kAtmosphereMaximumTerrainShadowDistance = 0.12F;
constexpr float kAtmosphereMaximumNightScattering = 0.25F;

struct AtmosphereQuadratureSample {
    float fraction{};
    float weight{};
};

[[nodiscard]] inline std::uint32_t atmosphereViewSampleCount(
    std::uint32_t quality) noexcept {
    return quality == 0U ? 1U : quality == 1U ? 2U : quality == 2U ? 3U : 5U;
}

[[nodiscard]] inline std::uint32_t atmosphereSunSampleCount(
    std::uint32_t quality) noexcept {
    return quality >= 3U ? 4U : quality == 0U ? 0U : 2U;
}

// Fixed Gauss-Legendre nodes mapped to [0,1]. They are stable under camera
// motion, integrate smooth density far better than an equal midpoint budget,
// and need no temporal jitter/history buffer.
[[nodiscard]] inline AtmosphereQuadratureSample atmosphereQuadratureSample(
    std::uint32_t sampleCount, std::uint32_t index) noexcept {
    if (sampleCount == 2U) {
        constexpr float offset = 0.2886751345948129F;
        return {index == 0U ? 0.5F - offset : 0.5F + offset, 0.5F};
    }
    if (sampleCount == 3U) {
        constexpr std::array<float, 3> fractions{
            0.1127016653792583F, 0.5F, 0.8872983346207417F};
        constexpr std::array<float, 3> weights{
            0.2777777777777778F, 0.4444444444444444F,
            0.2777777777777778F};
        const auto safeIndex = std::min(index, 2U);
        return {fractions[safeIndex], weights[safeIndex]};
    }
    if (sampleCount >= 5U) {
        constexpr std::array<float, 5> fractions{
            0.0469100770306680F, 0.2307653449471585F, 0.5F,
            0.7692346550528415F, 0.9530899229693320F};
        constexpr std::array<float, 5> weights{
            0.1184634425280945F, 0.2393143352496832F,
            0.2844444444444444F, 0.2393143352496832F,
            0.1184634425280945F};
        const auto safeIndex = std::min(index, 4U);
        return {fractions[safeIndex], weights[safeIndex]};
    }
    return {0.5F, 1.0F};
}

[[nodiscard]] inline AtmosphereRayInterval atmosphereSphereInterval(
    const std::array<float, 3>& origin,
    const std::array<float, 3>& direction,
    float radius) noexcept {
    if (!(radius > 0.0F) || !std::isfinite(radius)) {
        return {};
    }
    const float originDotDirection = origin[0] * direction[0] +
        origin[1] * direction[1] + origin[2] * direction[2];
    const float originSquared = origin[0] * origin[0] +
        origin[1] * origin[1] + origin[2] * origin[2];
    const float discriminant = originDotDirection * originDotDirection -
        (originSquared - radius * radius);
    if (!(discriminant >= 0.0F) || !std::isfinite(discriminant)) {
        return {};
    }
    const float root = std::sqrt(std::max(discriminant, 0.0F));
    const float nearDistance = -originDotDirection - root;
    const float farDistance = -originDotDirection + root;
    if (!(farDistance >= 0.0F) || !std::isfinite(nearDistance) ||
        !std::isfinite(farDistance)) {
        return {};
    }
    return {true, std::max(nearDistance, 0.0F), farDistance};
}

[[nodiscard]] inline float atmosphereDensityProfile(float radius,
                                                     float innerRadius,
                                                     float outerRadius,
                                                     float scaleHeight = 0.25F) noexcept {
    const float thickness = outerRadius - innerRadius;
    if (!(thickness > 0.0F) || !std::isfinite(radius) ||
        !std::isfinite(innerRadius) || !std::isfinite(outerRadius)) {
        return 0.0F;
    }
    const float height = std::clamp((radius - innerRadius) / thickness,
                                    0.0F, 1.0F);
    // A scale height of roughly one quarter of the stylized shell preserves a
    // dense soft limb while still transitioning cleanly to black space.
    return std::exp(-height / std::clamp(scaleHeight, 0.01F, 1.0F)) *
           (1.0F - height);
}

[[nodiscard]] inline float atmosphereIntegratedDensity(
    const std::array<float, 3>& origin,
    const std::array<float, 3>& direction,
    float segmentNear, float segmentFar,
    float innerRadius, float outerRadius, float scaleHeight,
    std::uint32_t sampleCount) noexcept {
    const float length = segmentFar - segmentNear;
    if (!(length > 0.0F) || sampleCount == 0U || sampleCount > 5U) {
        return 0.0F;
    }
    float integral = 0.0F;
    for (std::uint32_t index = 0U; index < sampleCount; ++index) {
        const auto sample = atmosphereQuadratureSample(sampleCount, index);
        const float distance = segmentNear + length * sample.fraction;
        const float x = origin[0] + direction[0] * distance;
        const float y = origin[1] + direction[1] * distance;
        const float z = origin[2] + direction[2] * distance;
        integral += atmosphereDensityProfile(
            std::sqrt(x * x + y * y + z * z), innerRadius, outerRadius,
            scaleHeight) * sample.weight;
    }
    return integral * length;
}

[[nodiscard]] inline std::array<float, 3>
atmosphereDepthIntegratedRayleighSpectrum(float strength) noexcept {
    const float value = std::max(strength, 0.0F);
    return {0.18F * value, 0.43F * value, value};
}

[[nodiscard]] inline std::array<float, 3>
atmosphereDepthIntegratedMieSpectrum(float strength) noexcept {
    const float value = std::max(strength, 0.0F);
    return {0.95F * value, 0.91F * value, 0.84F * value};
}

[[nodiscard]] inline std::array<float, 3>
atmosphereDepthIntegratedAbsorptionSpectrum(float strength) noexcept {
    const float value = std::max(strength, 0.0F);
    return {0.08F * value, 0.16F * value, 0.34F * value};
}

[[nodiscard]] inline float atmosphereTransmittance(float opticalDepth,
                                                   float density) noexcept {
    if (!std::isfinite(opticalDepth) || !std::isfinite(density)) {
        return 0.0F;
    }
    return std::clamp(std::exp(-std::max(opticalDepth, 0.0F) *
                               std::max(density, 0.0F)),
                      0.0F, 1.0F);
}

[[nodiscard]] inline std::uint32_t quantizeAtmosphere(float value,
                                                       float maximum,
                                                       std::uint32_t levels) noexcept {
    if (!(maximum > 0.0F) || levels == 0U || !std::isfinite(value)) {
        return 0U;
    }
    return static_cast<std::uint32_t>(std::lround(
        std::clamp(value / maximum, 0.0F, 1.0F) *
        static_cast<float>(levels)));
}

[[nodiscard]] inline std::uint32_t quantizeAtmosphereRange(
    float value, float minimum, float maximum) noexcept {
    if (!(maximum > minimum) || !std::isfinite(value)) {
        return 0U;
    }
    return quantizeAtmosphere(value - minimum, maximum - minimum, 255U);
}

[[nodiscard]] inline std::uint32_t packAtmosphereTerrainSeedHighBits(
    const PlanetAtmosphereSettings& settings,
    std::uint32_t lightingDebugMode) noexcept {
    const std::uint32_t start = quantizeAtmosphere(
        settings.startHeight, kAtmosphereMaximumStartHeight, 7U);
    return (lightingDebugMode & 0x3U) |
           ((std::min(settings.debugMode, 7U) & 0x7U) << 2U) |
           (start << 5U);
}

[[nodiscard]] inline std::uint32_t packAtmosphereSimulationFlags(
    const PlanetAtmosphereSettings& settings) noexcept {
    return (settings.enabled ? 1U << 27U : 0U) |
           (settings.physicalSurface ? 1U << 28U : 0U) |
           (settings.artisticNearFade ? 1U << 29U : 0U);
}

[[nodiscard]] inline std::uint32_t packAtmosphereIntegrationPhaseWord(
    std::uint32_t preservedLow24,
    const PlanetAtmosphereSettings& settings) noexcept {
    const std::uint32_t night = quantizeAtmosphere(
        settings.nightScattering, kAtmosphereMaximumNightScattering, 31U);
    const std::uint32_t controls =
        (settings.depthIntegrated ? 1U : 0U) |
        ((std::min(settings.integrationQuality, 3U) & 0x3U) << 1U) |
        ((night & 0x1fU) << 3U);
    return (preservedLow24 & 0x00ffffffU) | (controls << 24U);
}

[[nodiscard]] inline float unpackAtmosphereStartHeight(
    std::uint32_t terrainSeed) noexcept {
    return static_cast<float>((terrainSeed >> 29U) & 0x7U) *
        (kAtmosphereMaximumStartHeight / 7.0F);
}

[[nodiscard]] inline std::uint32_t packAtmosphereRenderWord0(
    float starBrightness, float starSize,
    const PlanetAtmosphereSettings& settings) noexcept {
    const auto brightness = quantizeAtmosphere(starBrightness, 2.5F, 255U);
    const auto size = quantizeAtmosphere(
        std::max(starSize - 0.5F, 0.0F), 2.0F, 255U);
    const auto rayleigh = quantizeAtmosphere(
        settings.rayleighScattering, kAtmosphereMaximumCoefficient, 255U);
    const auto mie = quantizeAtmosphere(
        settings.mieScattering, kAtmosphereMaximumCoefficient, 255U);
    return brightness | (size << 8U) | (rayleigh << 16U) | (mie << 24U);
}

[[nodiscard]] inline std::uint32_t packAtmosphereRenderWord1(
    const PlanetAtmosphereSettings& settings) noexcept {
    const auto absorption = quantizeAtmosphere(
        settings.absorption, kAtmosphereMaximumCoefficient, 255U);
    const auto scaleHeight = quantizeAtmosphereRange(
        settings.scaleHeight, kAtmosphereMinimumScaleHeight,
        kAtmosphereMaximumScaleHeight);
    const auto anisotropy = quantizeAtmosphereRange(
        settings.mieAnisotropy, kAtmosphereMinimumMieAnisotropy,
        kAtmosphereMaximumMieAnisotropy);
    const auto shadowDistance = quantizeAtmosphere(
        std::max(settings.terrainShadowDistance -
                 kAtmosphereMinimumTerrainShadowDistance, 0.0F),
        kAtmosphereMaximumTerrainShadowDistance -
            kAtmosphereMinimumTerrainShadowDistance, 7U);
    const auto shadowStrength = quantizeAtmosphere(
        settings.terrainShadowStrength, 1.0F, 3U);
    const std::uint32_t shadowControls =
        (settings.terrainShadows ? 1U : 0U) |
        ((std::min(settings.terrainShadowQuality, 3U) & 0x3U) << 1U) |
        ((shadowDistance & 0x7U) << 3U) |
        ((shadowStrength & 0x3U) << 6U);
    return absorption | (scaleHeight << 8U) | (anisotropy << 16U) |
           (shadowControls << 24U);
}

[[nodiscard]] inline float atmosphereHenyeyGreenstein(float cosineTheta,
                                                       float anisotropy) noexcept {
    const float cosine = std::clamp(cosineTheta, -1.0F, 1.0F);
    const float g = std::clamp(anisotropy, -0.95F, 0.95F);
    const float denominator = std::max(1.0F + g * g - 2.0F * g * cosine,
                                       1.0e-4F);
    return (1.0F - g * g) /
           (4.0F * 3.14159265358979323846F *
            std::pow(denominator, 1.5F));
}

[[nodiscard]] inline std::array<float, 3> atmosphereAbsorptionCoefficients(
    float strength) noexcept {
    const float clamped = std::max(strength, 0.0F);
    // Ozone-inspired broad Chappuis-band absorption: green/yellow is removed
    // more strongly than red or blue.
    return {0.18F * clamped, 0.62F * clamped, 0.10F * clamped};
}

[[nodiscard]] inline bool atmospherePlanetOccluded(
    const std::array<float, 3>& position,
    const std::array<float, 3>& sunDirection,
    float planetRadius) noexcept {
    if (!(planetRadius > 0.0F)) {
        return false;
    }
    const float projection = position[0] * sunDirection[0] +
        position[1] * sunDirection[1] + position[2] * sunDirection[2];
    const float positionSquared = position[0] * position[0] +
        position[1] * position[1] + position[2] * position[2];
    const float discriminant = projection * projection - positionSquared +
                               planetRadius * planetRadius;
    return projection < 0.0F && discriminant >= 0.0F;
}

[[nodiscard]] inline bool atmosphereTerrainHorizonBlocked(
    float originRadius, float blockerRadius, float angularSeparation,
    float sunRadial, float sunTangent) noexcept {
    if (!std::isfinite(originRadius) || !std::isfinite(blockerRadius) ||
        !std::isfinite(angularSeparation) || !std::isfinite(sunRadial) ||
        !std::isfinite(sunTangent) || originRadius <= 0.0F ||
        angularSeparation <= 0.0F || sunTangent <= 0.0F) {
        return false;
    }
    const float surfaceRun = originRadius * angularSeparation;
    const float rayRadius = originRadius +
        surfaceRun * (sunRadial / sunTangent) +
        0.5F * surfaceRun * surfaceRun / originRadius;
    return blockerRadius > rayRadius;
}

[[nodiscard]] inline float unpackAtmosphereEndHeight(
    std::uint32_t brushMaterial) noexcept {
    return static_cast<float>((brushMaterial >> 24U) & 0xffU) *
        (kAtmosphereMaximumEndHeight / 255.0F);
}

[[nodiscard]] inline float unpackAtmosphereDensity(
    std::uint32_t editMode) noexcept {
    return static_cast<float>((editMode >> 24U) & 0xffU) *
        (kAtmosphereMaximumDensity / 255.0F);
}

} // namespace voxel
