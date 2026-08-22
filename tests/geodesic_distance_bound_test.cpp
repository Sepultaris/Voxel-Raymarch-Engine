#include "planet/geodesic_distance_bound.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numbers>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "distance-bound test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

} // namespace

int main() {
    require(voxel::distanceBoundNodeCount(1024U, 512U) == 699'051U,
            "hierarchy node count changed");
    require(voxel::distanceBoundLevelOffset(1024U, 512U, 1U) == 524'288U,
            "level-one offset is not deterministic");

    for (std::uint32_t sample = 1U; sample < 1000U; ++sample) {
        const double longitudeFraction = static_cast<double>(sample % 997U) / 997.0;
        const double latitudeFraction = static_cast<double>((sample * 37U) % 991U) / 991.0;
        const double latitude = -1.4 + 2.8 * static_cast<double>(sample) / 1000.0;
        const double cosine = std::cos(latitude);
        const double lower = voxel::conservativeAngularBoundaryLowerBound(
            longitudeFraction, latitudeFraction, cosine, 32U, 16U);
        const double longitudeDelta = std::min(longitudeFraction,
                                                1.0 - longitudeFraction) *
                                      (2.0 * std::numbers::pi / 32.0);
        const double exactLongitude = std::asin(std::clamp(
            std::abs(std::sin(longitudeDelta) * cosine), 0.0, 1.0));
        const double exactLatitude = std::min(latitudeFraction,
                                              1.0 - latitudeFraction) *
                                     (std::numbers::pi / 16.0);
        require(lower >= 0.0 && lower <= exactLongitude + 1e-14 &&
                    lower <= exactLatitude + 1e-14,
                "angular boundary estimate is not a lower bound");

        const double radialGap = 0.001 + 0.02 * longitudeFraction;
        const double step = voxel::conservativeDistanceStep(
            radialGap, lower, 1.5);
        require(step >= 0.0 && step <= radialGap && step <= lower * 1.5,
                "distance step exceeded a proof bound");
        // A synthetic exact surface at radialGap cannot be crossed by the
        // hybrid handoff. This is the core first-hit differential invariant.
        require(step <= radialGap, "hybrid entry advanced past exact first hit");
    }

    require(voxel::conservativeDistanceStep(
                std::numeric_limits<double>::infinity(), 0.1, 1.0) == 0.0,
            "non-finite bound did not force exact refinement");
    require(voxel::conservativeDistanceStep(0.1, 0.0, 1.0) == 0.0,
            "boundary ray did not force exact refinement");
}
