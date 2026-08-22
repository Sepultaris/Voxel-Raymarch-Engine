#pragma once

#include <algorithm>
#include <cmath>

namespace voxel {

[[nodiscard]] inline float geodesicProjectedCellPixels(
    float cellWidth, float hitDistance, float viewportHeight) noexcept {
    return cellWidth * (viewportHeight * 0.5F) /
           std::max(hitDistance, 1.0e-6F);
}

[[nodiscard]] inline float geodesicSmoothstep(
    float lower, float upper, float value) noexcept {
    const float t = std::clamp((value - lower) / (upper - lower), 0.0F, 1.0F);
    return t * t * (3.0F - 2.0F * t);
}

// A radial wall needs usable coverage in both screen-space dimensions. A tall
// wall viewed edge-on can span many pixels vertically while remaining much
// less than one pixel wide; point-sampling its full side lighting produces the
// characteristic dotted radial chains. Fade only the wall's shading
// representation when either projected dimension is unresolved. Intersection
// distance and nearby, face-on cliff geometry remain exact.
[[nodiscard]] inline float geodesicSideDetailWeight(
    float normalRadialAlignment, float projectedExposedSideHeightPixels,
    float projectedSideWidthPixels, float sideViewIncidence) noexcept {
    const float sideMask = 1.0F - geodesicSmoothstep(
        0.20F, 0.70F, std::abs(normalRadialAlignment));
    const float projectedMinimumDimension = std::min(
        projectedExposedSideHeightPixels, projectedSideWidthPixels);
    const float resolvedDetail = geodesicSmoothstep(
        1.0F, 6.0F, projectedMinimumDimension);
    // At close range, multiplying a large projected cell width by a tiny
    // incidence can still exceed the pixel threshold even though the wall is
    // visually edge-on. Require a stable angular footprint as well.
    const float angularDetail = geodesicSmoothstep(
        0.08F, 0.25F, std::abs(sideViewIncidence));
    return 1.0F - sideMask * (1.0F - resolvedDetail * angularDetail);
}

} // namespace voxel
