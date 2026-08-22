#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace voxel {

struct PlanetLightingSettings {
    float sunAzimuth{-0.65F};
    float sunElevation{0.78F};
    std::array<float, 3> sunColor{1.0F, 0.94F, 0.84F};
    float sunIntensity{1.15F};
    std::array<float, 3> skyZenithColor{0.22F, 0.42F, 0.68F};
    std::array<float, 3> skyHorizonColor{0.48F, 0.55F, 0.58F};
    float skyIntensity{0.32F};
    float roughness{0.68F};
    float specularStrength{0.18F};
    float rimStrength{0.055F};
    float exposure{1.05F};
    float contactShadowStrength{0.18F};
    std::uint32_t debugMode{};
};

[[nodiscard]] inline std::uint32_t packLightingRgb8(
    const std::array<float, 3>& color, std::uint32_t highBits = 0U) noexcept {
    const auto channel = [](float value) noexcept {
        return static_cast<std::uint32_t>(std::lround(
            std::clamp(value, 0.0F, 1.0F) * 255.0F));
    };
    return channel(color[0]) | (channel(color[1]) << 8U) |
           (channel(color[2]) << 16U) | (highBits << 24U);
}

[[nodiscard]] inline std::array<float, 3> unpackLightingRgb8(
    std::uint32_t packed) noexcept {
    constexpr float inverse = 1.0F / 255.0F;
    return {static_cast<float>(packed & 0xffU) * inverse,
            static_cast<float>((packed >> 8U) & 0xffU) * inverse,
            static_cast<float>((packed >> 16U) & 0xffU) * inverse};
}

[[nodiscard]] inline float planetLightingDiffuse(float normalDotLight) noexcept {
    return std::clamp(normalDotLight, 0.0F, 1.0F);
}

[[nodiscard]] inline float planetLightingHemisphereWeight(
    float normalDotRadial) noexcept {
    const float t = std::clamp(normalDotRadial * 0.5F + 0.5F, 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

[[nodiscard]] inline float planetLightingMaterialRoughness(
    std::uint32_t material, float globalRoughness) noexcept {
    const float base = std::clamp(globalRoughness, 0.12F, 1.0F);
    if (material == 3U) { // water
        return std::clamp(base * 0.24F, 0.08F, 0.28F);
    }
    if (material == 4U) { // snow
        return std::clamp(base * 0.76F, 0.35F, 0.82F);
    }
    if (material == 5U) { // rock
        return std::clamp(base * 1.15F, 0.55F, 1.0F);
    }
    return std::clamp(base, 0.35F, 1.0F);
}

[[nodiscard]] inline float planetLightingAces(float hdr) noexcept {
    hdr = std::max(hdr, 0.0F);
    return std::clamp((hdr * (2.51F * hdr + 0.03F)) /
                          (hdr * (2.43F * hdr + 0.59F) + 0.14F),
                      0.0F, 1.0F);
}

[[nodiscard]] inline std::array<float, 3> applyPlanetLightingToHit(
    bool hit, const std::array<float, 3>& input, float exposure) noexcept {
    if (!hit) {
        return input;
    }
    return {planetLightingAces(input[0] * exposure),
            planetLightingAces(input[1] * exposure),
            planetLightingAces(input[2] * exposure)};
}

} // namespace voxel
