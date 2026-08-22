#include "render/render_settings.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <set>
#include <stdexcept>

namespace {

struct Vec2 {
    double x{};
    double y{};
};

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

struct TerrainSample {
    double surfaceHeight{};
    std::uint32_t filledLayers{};
    std::uint32_t occupancyMask{};
    std::uint32_t material{};
};

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

[[nodiscard]] Vec3 normalized(Vec3 value) {
    const double length = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    return {value.x / length, value.y / length, value.z / length};
}

[[nodiscard]] std::uint32_t avalanche(std::uint32_t value) {
    value ^= value >> 16U;
    value *= 0x7feb352dU;
    value ^= value >> 15U;
    value *= 0x846ca68bU;
    value ^= value >> 16U;
    return value;
}

[[nodiscard]] double latticeValue(std::int32_t x, std::int32_t y, std::uint32_t seed) {
    std::uint32_t bits = static_cast<std::uint32_t>(x) * 0x9e3779b9U;
    bits ^= static_cast<std::uint32_t>(y) * 0x85ebca6bU;
    bits ^= seed * 0xc2b2ae35U;
    return static_cast<double>(avalanche(bits) & 0x00ffffffU) / 16777215.0;
}

[[nodiscard]] double valueNoise(Vec2 point, std::uint32_t seed) {
    const auto x = static_cast<std::int32_t>(std::floor(point.x));
    const auto y = static_cast<std::int32_t>(std::floor(point.y));
    const double fractionX = point.x - std::floor(point.x);
    const double fractionY = point.y - std::floor(point.y);
    const double blendX = fractionX * fractionX * (3.0 - 2.0 * fractionX);
    const double blendY = fractionY * fractionY * (3.0 - 2.0 * fractionY);
    const double bottom = std::lerp(latticeValue(x, y, seed),
                                    latticeValue(x + 1, y, seed), blendX);
    const double top = std::lerp(latticeValue(x, y + 1, seed),
                                 latticeValue(x + 1, y + 1, seed), blendX);
    return std::lerp(bottom, top, blendY);
}

[[nodiscard]] double orthographicNoise(Vec3 direction, double scale, std::uint32_t seed) {
    double weightX = std::pow(std::abs(direction.x), 4.0);
    double weightY = std::pow(std::abs(direction.y), 4.0);
    double weightZ = std::pow(std::abs(direction.z), 4.0);
    const double weightSum = weightX + weightY + weightZ;
    weightX /= weightSum;
    weightY /= weightSum;
    weightZ /= weightSum;
    const double projectedX = valueNoise(
        {direction.y * scale + 19.1, direction.z * scale + 7.7}, seed ^ 0xa511e9b3U);
    const double projectedY = valueNoise(
        {direction.x * scale - 5.4, direction.z * scale + 13.8}, seed ^ 0x63d83595U);
    const double projectedZ = valueNoise(
        {direction.x * scale + 11.3, direction.y * scale - 17.2}, seed ^ 0xb5297a4dU);
    return projectedX * weightX + projectedY * weightY + projectedZ * weightZ;
}

[[nodiscard]] double fractalNoise(Vec3 direction, double scale, std::uint32_t seed,
                                  double roughness) {
    double result = 0.0;
    double amplitude = 1.0;
    double amplitudeSum = 0.0;
    for (std::uint32_t octave = 0; octave < 4U; ++octave) {
        result += orthographicNoise(direction, scale,
                                    seed + octave * 0x9e3779b9U) * amplitude;
        amplitudeSum += amplitude;
        scale *= 2.03;
        amplitude *= roughness;
    }
    return result / amplitudeSum;
}

[[nodiscard]] double ridgedNoise(Vec3 direction, double scale, std::uint32_t seed,
                                 double roughness) {
    double result = 0.0;
    double amplitude = 1.0;
    double amplitudeSum = 0.0;
    for (std::uint32_t octave = 0; octave < 3U; ++octave) {
        const double value = orthographicNoise(
            direction, scale, seed + octave * 0x68e31da4U);
        const double ridge = 1.0 - std::abs(value * 2.0 - 1.0);
        result += ridge * ridge * amplitude;
        amplitudeSum += amplitude;
        scale *= 2.11;
        amplitude *= roughness;
    }
    return result / amplitudeSum;
}

[[nodiscard]] double smoothstep(double lower, double upper, double value) {
    const double t = std::clamp((value - lower) / (upper - lower), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

[[nodiscard]] TerrainSample sampleTerrain(
    Vec3 direction, const voxel::TerrainGenerationSettings& settings,
    std::uint32_t layerCount = 32U) {
    direction = normalized(direction);
    const double roughness = std::clamp(static_cast<double>(settings.roughness), 0.15, 0.85);
    const double continents = fractalNoise(
        direction, std::max(static_cast<double>(settings.continentScale), 0.05),
        settings.seed, roughness);
    const double mountains = ridgedNoise(
        direction, std::max(static_cast<double>(settings.mountainScale), 0.1),
        settings.seed ^ 0xd1b54a35U, roughness);
    constexpr double seaHeight = 0.36;
    const double landSignal = continents -
        std::clamp(static_cast<double>(settings.oceanLevel), 0.12, 0.88);
    const double landMask = smoothstep(-0.065, 0.075, landSignal);
    const double inlandMask = smoothstep(0.04, 0.24, landSignal);
    const double mountainPeak = smoothstep(0.34, 0.86, mountains);
    const double basin = (1.0 - mountains) * (1.0 - mountains) * inlandMask;
    double groundHeight = 0.12 + continents * 0.40;
    groundHeight += landSignal * settings.continentStrength * 0.55;
    groundHeight += mountainPeak * mountainPeak * landMask * settings.mountainStrength;
    groundHeight -= basin * settings.continentStrength * 0.22;
    groundHeight += std::pow(std::abs(direction.y), 7.0) * landMask * settings.polarStrength;
    const double surfaceHeight = std::clamp(std::max(groundHeight, seaHeight), 0.08, 0.98);
    const auto filledLayers = std::clamp(
        static_cast<std::uint32_t>(std::round(surfaceHeight * layerCount)), 1U, layerCount);
    const std::uint32_t occupancyMask = filledLayers == 32U
                                            ? 0xffffffffU
                                            : (1U << filledLayers) - 1U;
    std::uint32_t material = 1U;
    if (groundHeight < seaHeight) {
        material = 3U;
    } else if (groundHeight < seaHeight + 0.055) {
        material = 2U;
    } else if ((std::abs(direction.y) > 0.76 && groundHeight > seaHeight + 0.05) ||
               groundHeight > 0.80) {
        material = 4U;
    } else if (mountainPeak > 0.64 && groundHeight > 0.58) {
        material = 5U;
    }
    return {surfaceHeight, filledLayers, occupancyMask, material};
}

void verifyContinuousBoundary(Vec3 left, Vec3 right,
                              const voxel::TerrainGenerationSettings& settings,
                              const char* message) {
    const TerrainSample a = sampleTerrain(left, settings);
    const TerrainSample b = sampleTerrain(right, settings);
    require(std::abs(a.surfaceHeight - b.surfaceHeight) < 0.001, message);
}

} // namespace

int main() {
    try {
        const voxel::TerrainGenerationSettings settings{};
        voxel::TerrainGenerationSettings alternate = settings;
        alternate.seed += 1U;
        std::set<std::uint32_t> materials;
        std::uint32_t changedBySeed = 0U;
        std::uint32_t highReliefSamples = 0U;
        std::uint32_t lowReliefSamples = 0U;
        double minimumHeight = 1.0;
        double maximumHeight = 0.0;
        constexpr std::uint32_t sampleCount = 8192U;
        constexpr double goldenAngle = 2.39996322972865332;
        for (std::uint32_t index = 0; index < sampleCount; ++index) {
            const double y = 1.0 - 2.0 * (static_cast<double>(index) + 0.5) / sampleCount;
            const double radius = std::sqrt(std::max(1.0 - y * y, 0.0));
            const double angle = goldenAngle * index;
            const Vec3 direction{radius * std::cos(angle), y, radius * std::sin(angle)};
            const TerrainSample first = sampleTerrain(direction, settings);
            const TerrainSample repeated = sampleTerrain(direction, settings);
            const TerrainSample changed = sampleTerrain(direction, alternate);
            require(first.filledLayers == repeated.filledLayers &&
                        first.occupancyMask == repeated.occupancyMask &&
                        first.material == repeated.material &&
                        std::bit_cast<std::uint64_t>(first.surfaceHeight) ==
                            std::bit_cast<std::uint64_t>(repeated.surfaceHeight),
                    "Terrain generation is not deterministic");
            require(first.filledLayers >= 1U && first.filledLayers <= 32U,
                    "Generated height is outside the radial brick");
            const std::uint32_t expectedMask = first.filledLayers == 32U
                                                   ? 0xffffffffU
                                                   : (1U << first.filledLayers) - 1U;
            require(first.occupancyMask == expectedMask,
                    "Generated occupancy is not bottom-contiguous");
            require(first.material >= 1U && first.material <= 5U,
                    "Generated material is invalid");
            materials.insert(first.material);
            minimumHeight = std::min(minimumHeight, first.surfaceHeight);
            maximumHeight = std::max(maximumHeight, first.surfaceHeight);
            highReliefSamples += first.surfaceHeight > 0.80 ? 1U : 0U;
            lowReliefSamples += first.surfaceHeight < 0.46 ? 1U : 0U;
            changedBySeed += first.filledLayers != changed.filledLayers ||
                                     first.material != changed.material
                                 ? 1U
                                 : 0U;
        }
        require(changedBySeed > sampleCount / 3U,
                "Changing the seed does not materially change the planet");
        require(materials.contains(1U) && materials.contains(2U) && materials.contains(3U) &&
                    materials.size() >= 4U,
                "Default terrain does not exercise the material bands");
        require(maximumHeight - minimumHeight > 0.48,
                "Default terrain does not span enough normalized vertical relief");
        require(highReliefSamples > sampleCount / 100U,
                "Default terrain does not generate enough high mountain terrain");
        require(lowReliefSamples > sampleCount / 8U,
                "Default terrain does not generate enough deep valley or basin terrain");

        constexpr double epsilon = 1e-5;
        verifyContinuousBoundary({-1.0, 0.25, epsilon}, {-1.0, 0.25, -epsilon}, settings,
                                 "Anti-meridian introduced a height seam");
        verifyContinuousBoundary({epsilon, 1.0, epsilon}, {-epsilon, 1.0, -epsilon}, settings,
                                 "Polar projection introduced a height seam");
        verifyContinuousBoundary({1.0 + epsilon, 1.0 - epsilon, 0.3},
                                 {1.0 - epsilon, 1.0 + epsilon, 0.3}, settings,
                                 "Tri-planar axis transition introduced a height seam");

        std::cout << "Procedural orthographic terrain invariants passed with "
                  << materials.size() << " material bands.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Procedural terrain gate failed: " << error.what() << '\n';
        return 1;
    }
}
