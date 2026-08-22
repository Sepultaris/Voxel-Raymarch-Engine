#include "planet/fractal_voxel_sdf.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "Fractal voxel SDF regression failed: " << message << '\n';
        std::exit(1);
    }
}

struct Grid {
    std::uint32_t resolution{};
    float cellSize{};
    std::vector<float> values;
};

Grid buildGrid(std::uint32_t resolution, float cellSize,
               std::uint32_t parentLevel, float transition) {
    Grid grid{resolution, cellSize,
              std::vector<float>(resolution * resolution)};
    constexpr std::array<float, 3> center{0.0F, 0.0F, 1.0F};
    constexpr std::array<float, 3> right{1.0F, 0.0F, 0.0F};
    constexpr std::array<float, 3> forward{0.0F, 1.0F, 0.0F};
    const float half = static_cast<float>(resolution - 1U) * 0.5F;
    for (std::uint32_t y = 0U; y < resolution; ++y) {
        for (std::uint32_t x = 0U; x < resolution; ++x) {
            const auto direction = voxel::normalizedFractalVoxelDirection(
                center, right, forward,
                (static_cast<float>(x) - half) * cellSize,
                (static_cast<float>(y) - half) * cellSize, 1.0F);
            grid.values[x + resolution * y] = voxel::fractalVoxelSourceHeight(
                direction, 1337U, parentLevel, transition);
        }
    }
    return grid;
}

float sampleGrid(const Grid& grid, float x, float y) {
    x = std::clamp(x, 0.0F,
                   static_cast<float>(grid.resolution - 1U) - 1e-4F);
    y = std::clamp(y, 0.0F,
                   static_cast<float>(grid.resolution - 1U) - 1e-4F);
    const auto ix = static_cast<std::uint32_t>(std::floor(x));
    const auto iy = static_cast<std::uint32_t>(std::floor(y));
    const float localX = x - static_cast<float>(ix);
    const float localY = y - static_cast<float>(iy);
    const auto at = [&](std::uint32_t px, std::uint32_t py) {
        return grid.values[px + grid.resolution * py];
    };
    return voxel::fractalVoxelBilinear(
        at(ix, iy), at(ix + 1U, iy), at(ix, iy + 1U),
        at(ix + 1U, iy + 1U), localX, localY);
}

float sampleDiscreteGrid(const Grid& grid, float x, float y) {
    x = std::clamp(x, 0.0F,
                   static_cast<float>(grid.resolution - 1U) - 1e-4F);
    y = std::clamp(y, 0.0F,
                   static_cast<float>(grid.resolution - 1U) - 1e-4F);
    const auto ix = static_cast<std::uint32_t>(std::floor(x));
    const auto iy = static_cast<std::uint32_t>(std::floor(y));
    const auto at = [&](std::uint32_t px, std::uint32_t py) {
        return grid.values[px + grid.resolution * py];
    };
    return voxel::fractalVoxelDiscreteCellHeight(
        at(ix, iy), at(ix + 1U, iy), at(ix, iy + 1U),
        at(ix + 1U, iy + 1U));
}

} // namespace

int main() {
    using namespace voxel;

    // Full zoom sweep: outside the configured maximum, the retained leaf is
    // always bounded by the split/merge band and changes by one 2:1 level.
    FractalVoxelCacheState state{};
    std::uint32_t previousLevel = 0U;
    for (std::uint32_t sample = 0U; sample <= 160U; ++sample) {
        const float exponent = 1.0F - 4.5F *
            static_cast<float>(sample) / 160.0F;
        const float distance = std::pow(10.0F, exponent);
        state = updateFractalVoxelCache(
            state, distance, 1.0F, 900U, 1.25F, 10U);
        require(state.level >= previousLevel &&
                    state.level - previousLevel <= 1U,
                "approach skipped a 2:1 refinement level");
        require(state.level == 10U ||
                    state.projectedPixels <= kFractalVoxelSplitPixels + 1e-4F,
                "approach leaf exceeded the split threshold");
        require(state.level == 0U ||
                    state.projectedPixels >= kFractalVoxelMergePixels - 1e-4F,
                "approach leaf fell below the merge threshold");
        previousLevel = state.level;
    }
    for (std::uint32_t sample = 0U; sample <= 160U; ++sample) {
        const float exponent = -3.5F + 4.5F *
            static_cast<float>(sample) / 160.0F;
        const float distance = std::pow(10.0F, exponent);
        previousLevel = state.level;
        state = updateFractalVoxelCache(
            state, distance, 1.0F, 900U, 1.25F, 10U);
        require(previousLevel >= state.level &&
                    previousLevel - state.level <= 1U,
                "retreat skipped a 2:1 merge level");
    }
    require(state.refinements > 0U && state.merges > 0U,
            "zoom sweep did not exercise split and merge transitions");

    // Jitter around the split point must remain in the child because the
    // merge threshold is deliberately below its 0.8 px post-split size.
    FractalVoxelCacheState jitter{};
    const float splitDistance = kFractalPlanetParentAngularWidth * 450.0F /
        kFractalVoxelSplitPixels;
    jitter = updateFractalVoxelCache(
        jitter, splitDistance * 0.999F, 1.0F, 900U, 1.25F, 10U);
    const auto stableLevel = jitter.level;
    const auto initialRefines = jitter.refinements;
    for (std::uint32_t frame = 0U; frame < 240U; ++frame) {
        const float scale = (frame & 1U) == 0U ? 0.998F : 1.002F;
        jitter = updateFractalVoxelCache(
            jitter, splitDistance * scale, 1.0F, 900U, 1.25F, 10U);
        require(jitter.level == stableLevel,
                "camera jitter thrashed the virtual brick level");
    }
    require(jitter.refinements == initialRefines && jitter.merges == 0U,
            "camera jitter generated cache split/merge traffic");

    // Readability cues are field-local post-hit terms. Flat and convex
    // terrain remain open, progressively deeper bowls darken monotonically,
    // and the result is bounded independently of camera/path work.
    const float flatCavity = fractalVoxelBandCavity(0.0F, 1.0F);
    const float ridgeCavity = fractalVoxelBandCavity(0.8F, 1.0F);
    const float shallowBowlCavity = fractalVoxelBandCavity(-0.32F, 1.0F);
    const float deepBowlCavity = fractalVoxelBandCavity(-0.92F, 1.0F);
    const float flatAo = fractalVoxelLocalCavityAo(flatCavity, 0.22F);
    const float ridgeAo = fractalVoxelLocalCavityAo(ridgeCavity, 0.22F);
    const float shallowBowlAo = fractalVoxelLocalCavityAo(
        shallowBowlCavity, 0.22F);
    const float deepBowlAo = fractalVoxelLocalCavityAo(
        deepBowlCavity, 0.22F);
    require(std::abs(flatAo - 1.0F) < 1e-6F &&
                std::abs(ridgeAo - 1.0F) < 1e-6F,
            "flat/convex field AO was not fully open");
    require(deepBowlAo < shallowBowlAo && shallowBowlAo < flatAo,
            "local field AO was not monotonic with concavity");
    require(fractalVoxelBandCavity(-0.92F, 0.25F) < deepBowlCavity,
            "local field AO did not fade with octave activation");
    require(deepBowlAo >= 0.55F && deepBowlAo <= 1.0F,
            "local field AO escaped its output bounds");
    require(fractalVoxelCellEdgeVisibility(1.25F) == 0.0F &&
                fractalVoxelCellEdgeVisibility(3.0F) > 0.99F,
            "cell edge overlay did not suppress subpixel speckle");
    const std::uint32_t packedSample = packFractalVoxelCacheSample(
        1.012345F, 0.61F);
    require(std::abs(unpackFractalVoxelCacheHeight(packedSample) - 1.012345F) <
                1e-7F,
            "packed cache height lost root precision");
    require(std::abs(unpackFractalVoxelCacheCavity(packedSample) - 0.61F) <
                1.0F / 255.0F,
            "packed cache cavity exceeded one quantization step");
    require(fractalVoxelCacheSampleValid(packedSample) &&
                !fractalVoxelCacheSampleValid(0U),
            "uninitialized cache sentinel was accepted as terrain");

    // The materialized brick is a bounded acceleration/detail region, never a
    // square surface. Rays outside it and samples that touch uninitialized
    // memory must select the direct SDF oracle. Valid detail is blended in
    // continuously over an interior guard band so the footprint cannot alter
    // either the first hit or the visible palette at its rectangle boundary.
    constexpr std::uint32_t cacheResolution = 664U;
    require(!fractalVoxelCacheCoordinateValid(
                std::nextafter(1.0F, 0.0F), 332.0F, cacheResolution) &&
                !fractalVoxelCacheCoordinateValid(
                    332.0F, std::nextafter(662.0F, 663.0F),
                    cacheResolution),
            "outside cache-boundary coordinate was accepted");
    require(fractalVoxelCacheCoordinateValid(
                1.0F, 1.0F, cacheResolution) &&
                fractalVoxelCacheCoordinateValid(
                    332.0F, 332.0F, cacheResolution),
            "valid cache coordinate was rejected");
    require(fractalVoxelCacheInteriorBlend(
                1.0F, 332.0F, cacheResolution) == 0.0F &&
                fractalVoxelCacheInteriorBlend(
                    332.0F, 332.0F, cacheResolution) == 1.0F,
            "cache boundary did not fall back to the direct SDF");
    float previousBoundaryBlend = 0.0F;
    for (std::uint32_t offset = 0U; offset <= 24U; ++offset) {
        const float blend = fractalVoxelCacheInteriorBlend(
            1.0F + static_cast<float>(offset), 332.0F, cacheResolution);
        require(blend >= previousBoundaryBlend && blend >= 0.0F &&
                    blend <= 1.0F,
                "cache/detail boundary blend was discontinuous or unbounded");
        previousBoundaryBlend = blend;
    }
    require(previousBoundaryBlend == 1.0F,
            "cache guard band never reached full discrete ownership");

    // Discrete mode changes the selected geometry, not merely its shading. A
    // half-open leaf owns one constant radial cap independent of the ray's
    // sub-cell position; an adjacent leaf may have a different cap, making the
    // intervening radial face real. Only the bounded-cache guard reconstructs
    // the smooth oracle to prevent a rectangular footprint seam.
    constexpr float discreteH00 = 0.998F;
    constexpr float discreteH10 = 1.014F;
    constexpr float discreteH01 = 1.006F;
    constexpr float discreteH11 = 1.022F;
    const float expectedDiscrete = fractalVoxelDiscreteCellHeight(
        discreteH00, discreteH10, discreteH01, discreteH11);
    require(std::abs(fractalVoxelMaterializedHeight(
                discreteH00, discreteH10, discreteH01, discreteH11,
                0.07F, 0.91F, 1.0F, true) - expectedDiscrete) < 1e-7F &&
                std::abs(fractalVoxelMaterializedHeight(
                    discreteH00, discreteH10, discreteH01, discreteH11,
                    0.93F, 0.04F, 1.0F, true) - expectedDiscrete) < 1e-7F,
            "discrete leaf cap still varied smoothly within the cell");
    require(std::abs(fractalVoxelMaterializedHeight(
                discreteH00, discreteH10, discreteH01, discreteH11,
                0.37F, 0.62F, 0.0F, true) -
                fractalVoxelBilinear(
                    discreteH00, discreteH10, discreteH01, discreteH11,
                    0.37F, 0.62F)) < 1e-7F,
            "cache boundary did not reconstruct the smooth direct fallback");
    require(std::abs(expectedDiscrete -
                fractalVoxelDiscreteCellHeight(
                    discreteH10, 1.031F, discreteH11, 1.027F)) > 1e-4F,
            "neighboring discrete cells did not create a real step face");

    constexpr std::uint32_t level = 5U;
    constexpr float transition = 0.63F;
    constexpr std::uint32_t resolution = 65U;
    const float parentSize = kFractalPlanetParentAngularWidth /
        static_cast<float>(1U << level);
    const Grid parent = buildGrid(resolution, parentSize, level, transition);
    const Grid child = buildGrid(resolution, parentSize * 0.5F,
                                 level, transition);
    float maximumHeightError = 0.0F;
    float maximumDiscreteRefineDelta = 0.0F;
    std::uint32_t oracleHits = 0U;
    for (std::uint32_t y = 8U; y < resolution - 8U; ++y) {
        for (std::uint32_t x = 8U; x < resolution - 8U; ++x) {
            const float fx = static_cast<float>(x) + 0.371F;
            const float fy = static_cast<float>(y) + 0.619F;
            const float half = static_cast<float>(resolution - 1U) * 0.5F;
            const float tangentX = (fx - half) * parentSize * 0.5F;
            const float tangentY = (fy - half) * parentSize * 0.5F;
            constexpr std::array<float, 3> center{0.0F, 0.0F, 1.0F};
            constexpr std::array<float, 3> right{1.0F, 0.0F, 0.0F};
            constexpr std::array<float, 3> forward{0.0F, 1.0F, 0.0F};
            const auto direction = normalizedFractalVoxelDirection(
                center, right, forward, tangentX, tangentY, 1.0F);
            const float oracle = fractalVoxelSourceHeight(
                direction, 1337U, level, transition);
            const float parentX = tangentX / parentSize + half;
            const float parentY = tangentY / parentSize + half;
            const float childX = tangentX / (parentSize * 0.5F) + half;
            const float childY = tangentY / (parentSize * 0.5F) + half;
            const float voxelHeight = std::lerp(
                sampleGrid(parent, parentX, parentY),
                sampleGrid(child, childX, childY), transition);
            const float parentDiscrete = sampleDiscreteGrid(
                parent, parentX, parentY);
            const float voxelDiscrete = std::lerp(
                parentDiscrete,
                sampleDiscreteGrid(child, childX, childY), transition);
            require(std::isfinite(voxelHeight) && voxelHeight > 0.98F,
                    "materialized voxel cache produced a hole/nonfinite height");
            require(std::isfinite(voxelDiscrete) && voxelDiscrete > 0.98F,
                    "discrete voxel surface produced a hole/nonfinite cap");
            ++oracleHits;
            maximumHeightError = std::max(
                maximumHeightError, std::abs(voxelHeight - oracle));
            maximumDiscreteRefineDelta = std::max(
                maximumDiscreteRefineDelta,
                std::abs(voxelDiscrete - parentDiscrete));
        }
    }
    require(oracleHits > 2000U,
            "cache/oracle differential sampled too few rays");
    require(maximumHeightError < parentSize * 1.5F,
            "voxelized SDF depth diverged by more than one conservative leaf");
    // The fixed cache-test patch can legitimately be ocean-flat. Sweep the
    // whole deterministic source to require that promotion to the next leaf
    // level exposes a new band somewhere, while every quantized cap remains a
    // valid closed radial column.
    for (std::uint32_t latitude = 1U; latitude < 16U; ++latitude) {
        const float phi = -1.57079632679F + 3.14159265359F *
            static_cast<float>(latitude) / 16.0F;
        for (std::uint32_t longitude = 0U; longitude < 32U; ++longitude) {
            const float theta = 6.28318530718F *
                static_cast<float>(longitude) / 32.0F;
            const std::array<float, 3> direction{
                std::cos(phi) * std::cos(theta), std::sin(phi),
                std::cos(phi) * std::sin(theta)};
            const float coarse = fractalVoxelSourceHeight(
                direction, 1337U, level, 0.0F);
            const float refined = fractalVoxelSourceHeight(
                direction, 1337U, level + 1U, 0.0F);
            require(std::isfinite(coarse) && std::isfinite(refined) &&
                        coarse > 0.98F && refined > 0.98F,
                    "source refinement produced an open/nonfinite column");
            maximumDiscreteRefineDelta = std::max(
                maximumDiscreteRefineDelta, std::abs(refined - coarse));
        }
    }
    require(maximumDiscreteRefineDelta > 1e-6F,
            "refining discrete leaves revealed no new fractal geometry");

    // Half-open bilinear ownership must be continuous on either side of a
    // shared cell edge, including ULP-scale perturbations.
    const float seamX = 31.0F;
    const float seamY = 30.417F;
    const float seamLeft = sampleGrid(parent,
        std::nextafter(seamX, 0.0F), seamY);
    const float seamRight = sampleGrid(parent,
        std::nextafter(seamX, 64.0F), seamY);
    require(std::abs(seamLeft - seamRight) < 2e-6F,
            "half-open neighboring leaf cells are discontinuous");

    // At a completed split the old child and new parent are the same samples;
    // the 2:1 transition therefore has no pop.
    const Grid completedChild = buildGrid(
        resolution, parentSize * 0.5F, level, 1.0F);
    const Grid promotedParent = buildGrid(
        resolution, parentSize * 0.5F, level + 1U, 0.0F);
    for (std::size_t index = 0U; index < completedChild.values.size(); index += 97U) {
        require(std::abs(completedChild.values[index] - promotedParent.values[index]) < 1e-7F,
                "completed split did not equal the promoted parent cache");
    }

    std::cout << "Fractal voxel SDF cache regression passed: refines="
              << state.refinements << " merges=" << state.merges
              << " oracle-rays=" << oracleHits
              << " max-height-error=" << maximumHeightError << '\n';
    return 0;
}
