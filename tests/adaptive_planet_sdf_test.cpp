#include "planet/adaptive_planet_sdf.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main() {
    using namespace voxel;
    const std::array<float, 3> direction{0.42426407F, 0.56568545F, 0.70710677F};

    const AdaptivePlanetSdfLod distant = adaptivePlanetSdfLod(1.0F, 1.25F, 9U);
    const AdaptivePlanetSdfLod split = adaptivePlanetSdfLod(10.0F, 1.25F, 9U);
    const AdaptivePlanetSdfLod close = adaptivePlanetSdfLod(1280.0F, 1.25F, 9U);
    require(distant.continuousLevel == 0.0F,
            "Subpixel parents must merge to the exact macro cap");
    require(split.completeLevels == 3U && split.virtualCellPixels <= 1.25F,
            "Pixel policy did not split until virtual cells reached the target footprint");
    require(close.completeLevels == 9U,
            "Close terrain did not reach the bounded finest subdivision");
    const AdaptivePlanetSdfLod receded = adaptivePlanetSdfLod(10.0F, 1.25F, 9U);
    require(std::bit_cast<std::uint32_t>(receded.continuousLevel) ==
                std::bit_cast<std::uint32_t>(split.continuousLevel),
            "Refine/merge policy is not reversible");
    const float balancedFine = balanceAdaptivePlanetSdfLevel(8.0F, 4.0F, 9U);
    const float balancedCoarse = balanceAdaptivePlanetSdfLevel(4.0F, 8.0F, 9U);
    require(balancedFine == 5.0F && balancedCoarse == 4.0F &&
                std::abs(balancedFine - balancedCoarse) <= 1.0F,
            "Two-to-one topology-ring balancing failed");

    const AdaptivePlanetSdfField first = evaluateAdaptivePlanetSdf(
        direction, 1337U, split.continuousLevel, 0.15F);
    const AdaptivePlanetSdfField repeat = evaluateAdaptivePlanetSdf(
        direction, 1337U, split.continuousLevel, 0.15F);
    require(std::bit_cast<std::uint32_t>(first.depthLayerFraction) ==
                std::bit_cast<std::uint32_t>(repeat.depthLayerFraction),
            "Adaptive planet field is not deterministic");
    require(first.depthLayerFraction >= 0.0F &&
                first.depthLayerFraction <= first.evaluatedAmplitude + 1e-6F &&
                first.evaluatedAmplitude + first.residualAmplitudeBound <= 0.150001F,
            "Adaptive planet field escaped its conservative amplitude bound");
    for (float gradient : first.directionGradient) {
        require(std::isfinite(gradient), "Adaptive planet gradient became non-finite");
    }

    float previousAmplitude = 0.0F;
    float previousResidual = 1.0F;
    for (std::uint32_t level = 0U; level <= 9U; ++level) {
        const AdaptivePlanetSdfField field = evaluateAdaptivePlanetSdf(
            direction, 1337U, static_cast<float>(level), 0.15F);
        require(field.evaluatedAmplitude + 1e-7F >= previousAmplitude,
                "Subdivision reduced evaluated conservative amplitude");
        require(field.residualAmplitudeBound <= previousResidual + 1e-7F,
                "Subdivision increased the unresolved tail bound");
        previousAmplitude = field.evaluatedAmplitude;
        previousResidual = field.residualAmplitudeBound;
    }

    const AdaptivePlanetSdfField invalid = evaluateAdaptivePlanetSdf(
        direction, 1337U, std::numeric_limits<float>::quiet_NaN(), 0.15F);
    require(invalid.depthLayerFraction == 0.0F &&
                invalid.residualAmplitudeBound >= 0.15F,
            "Non-finite adaptive input did not fail closed");

    const auto key = adaptivePlanetSdfCellAddress(direction, 7U);
    const auto repeatedKey = adaptivePlanetSdfCellAddress(direction, 7U);
    require(key == repeatedKey && key.face < 6U && key.x < 128U && key.y < 128U,
            "Half-open virtual cell addressing is invalid or unstable");
    const auto tieKey = adaptivePlanetSdfCellAddress({1.0F, 1.0F, 1.0F}, 4U);
    require(tieKey.face == 0U,
            "Cube-face ties must use deterministic X/Y/Z half-open priority");

    const float fullSupport = adaptivePlanetSdfSupport(5.0F, 0.1F, 0.9F);
    require(fullSupport > 0.99F &&
                adaptivePlanetSdfSupport(5.0F, 0.78F, 0.9F) == 0.0F &&
                adaptivePlanetSdfSupport(5.0F, 0.1F, 0.35F) == 0.0F,
            "Adaptive SDF topology/silhouette support guard is invalid");
    const AdaptivePlanetSdfRoot root = solveAdaptivePlanetSdfRoot(
        {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 1.0F},
        {0.0F, 0.0F, -1.0F}, 0.0015F, fullSupport, 7.0F,
        1337U, 0.15F);
    const AdaptivePlanetSdfRoot repeatedRoot = solveAdaptivePlanetSdfRoot(
        {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 1.0F},
        {0.0F, 0.0F, -1.0F}, 0.0015F, fullSupport, 7.0F,
        1337U, 0.15F);
    require(root.hit && root.iterations == kAdaptivePlanetSdfRootIterations &&
                root.rayOffset >= 0.0F && root.rayOffset <= 0.0015F * 0.15F &&
                std::bit_cast<std::uint32_t>(root.rayOffset) ==
                    std::bit_cast<std::uint32_t>(repeatedRoot.rayOffset),
            "Bounded local adaptive SDF root is invalid or nondeterministic");
    const AdaptivePlanetSdfRoot grazingRoot = solveAdaptivePlanetSdfRoot(
        {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 1.0F},
        {1.0F, 0.0F, -0.1F}, 0.0015F, 1.0F, 7.0F, 1337U, 0.15F);
    require(!grazingRoot.hit,
            "Grazing adaptive root did not preserve the exact macro silhouette");

    const std::array<float, 3> seamA{0.70710677F, 0.70710677F, 0.0F};
    const std::array<float, 3> seamB{0.70710671F, 0.70710683F, 0.0F};
    const auto seamFieldA = evaluateAdaptivePlanetSdf(seamA, 1337U, 6.0F, 0.15F);
    const auto seamFieldB = evaluateAdaptivePlanetSdf(seamB, 1337U, 6.0F, 0.15F);
    require(std::abs(seamFieldA.depthLayerFraction -
                     seamFieldB.depthLayerFraction) < 0.002F,
            "World-space adaptive field is discontinuous across a virtual face seam");

    const std::uint32_t packed = packVisualizationAndAdaptiveSdf(
        1U, true, 9U, 1.25F, 3U, 0.15F, true);
    require((packed & 0xffU) == 1U && (packed & (1U << 8U)) != 0U &&
                ((packed >> 9U) & 0xfU) == 9U &&
                ((packed >> 13U) & 0xffU) == 10U &&
                ((packed >> 21U) & 0x3U) == 3U &&
                (packed & (1U << 31U)) != 0U,
            "Adaptive SDF push-constant packing is inconsistent");

    std::cout << "Adaptive fractal planet SDF subdivision invariants passed.\n";
    return 0;
}
