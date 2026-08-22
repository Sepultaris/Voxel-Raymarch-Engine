#pragma once

#include "render/intrinsic_ellis_manifold.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace voxel {

[[nodiscard]] inline bool globalObserverTerrainOwnsBeforeMouth(
    bool exactHit, bool frontFacing, double hitDistance,
    double mouthDistance) noexcept {
    return exactHit && frontFacing && std::isfinite(hitDistance) &&
           std::isfinite(mouthDistance) && hitDistance >= 0.0 &&
           hitDistance < mouthDistance;
}

// Legacy analytic reference charts retained only for the independent Ellis
// oracle tests.  The runtime global lab no longer switches between these
// regimes; it evaluates global_handle_atlas.hpp in its shared exterior chart.
enum class EllisOracleChart : std::uint32_t {
    FlatReference = 0U,
    Ellis = 1U,
};

using GlobalVector4 = std::array<double, 4>;
using GlobalMatrix4 = std::array<std::array<double, 4>, 4>;
using GlobalConnection4 =
    std::array<std::array<std::array<double, 4>, 4>, 4>;

struct GlobalMetricSample {
    GlobalMatrix4 metric{};
    GlobalMatrix4 inverse{};
    GlobalConnection4 christoffel{};
    bool finite{};
};

struct GlobalNullState {
    EllisOracleChart chart{EllisOracleChart::FlatReference};
    GlobalVector4 position{}; // t,x,y,z externally; t,l,theta,phi in throat
    GlobalVector4 tangent{1.0, 0.0, 0.0, 1.0};
};

[[nodiscard]] inline GlobalMetricSample evaluateEllisOracleMetric(
    EllisOracleChart chart, const GlobalVector4& position,
    double throatRadius = static_cast<double>(kPortalGrThroatRatio)) noexcept {
    GlobalMetricSample result{};
    if (chart == EllisOracleChart::FlatReference) {
        result.metric[0][0] = -1.0;
        result.inverse[0][0] = -1.0;
        for (std::size_t axis = 1; axis < 4; ++axis) {
            result.metric[axis][axis] = 1.0;
            result.inverse[axis][axis] = 1.0;
        }
        result.finite = true;
        return result;
    }

    const double l = position[1];
    const double theta = position[2];
    const double a = std::max(throatRadius, 1.0e-8);
    const EllisRadialProfile radial = smoothEllisRadialProfile(
        static_cast<float>(l), static_cast<float>(a), true);
    const double radius = std::max(static_cast<double>(radial.radius), 1.0e-12);
    const double radiusPrime = static_cast<double>(radial.firstDerivative);
    const double r2 = radius * radius;
    const double sine = std::sin(theta);
    const double cosine = std::cos(theta);
    const double sine2 = std::max(sine * sine, 1.0e-16);
    result.metric[0][0] = -1.0;
    result.metric[1][1] = 1.0;
    result.metric[2][2] = r2;
    result.metric[3][3] = r2 * sine2;
    result.inverse[0][0] = -1.0;
    result.inverse[1][1] = 1.0;
    result.inverse[2][2] = 1.0 / r2;
    result.inverse[3][3] = 1.0 / (r2 * sine2);

    result.christoffel[1][2][2] = -radius * radiusPrime;
    result.christoffel[1][3][3] = -radius * radiusPrime * sine2;
    result.christoffel[2][1][2] = radiusPrime / radius;
    result.christoffel[2][2][1] = radiusPrime / radius;
    result.christoffel[2][3][3] = -sine * cosine;
    result.christoffel[3][1][3] = radiusPrime / radius;
    result.christoffel[3][3][1] = radiusPrime / radius;
    const double cotangent = cosine / (std::abs(sine) > 1.0e-8
        ? sine : std::copysign(1.0e-8, sine == 0.0 ? 1.0 : sine));
    result.christoffel[3][2][3] = cotangent;
    result.christoffel[3][3][2] = cotangent;
    result.finite = std::isfinite(r2) && std::isfinite(cotangent);
    return result;
}

[[nodiscard]] inline double globalMetricInner(
    const GlobalMetricSample& sample, const GlobalVector4& a,
    const GlobalVector4& b) noexcept {
    double result = 0.0;
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            result += sample.metric[row][column] * a[row] * b[column];
        }
    }
    return result;
}

struct GlobalNullDerivative {
    GlobalVector4 position{};
    GlobalVector4 tangent{};
};

[[nodiscard]] inline GlobalNullDerivative globalNullDerivative(
    const GlobalNullState& state, double throatRadius) noexcept {
    GlobalNullDerivative result{};
    result.position = state.tangent;
    const GlobalMetricSample metric = evaluateEllisOracleMetric(
        state.chart, state.position, throatRadius);
    for (std::size_t mu = 0; mu < 4; ++mu) {
        double acceleration = 0.0;
        for (std::size_t alpha = 0; alpha < 4; ++alpha) {
            for (std::size_t beta = 0; beta < 4; ++beta) {
                acceleration -= metric.christoffel[mu][alpha][beta] *
                    state.tangent[alpha] * state.tangent[beta];
            }
        }
        result.tangent[mu] = acceleration;
    }
    return result;
}

[[nodiscard]] inline GlobalNullState globalNullAdd(
    const GlobalNullState& state, const GlobalNullDerivative& derivative,
    double scale) noexcept {
    GlobalNullState result = state;
    for (std::size_t component = 0; component < 4; ++component) {
        result.position[component] += derivative.position[component] * scale;
        result.tangent[component] += derivative.tangent[component] * scale;
    }
    return result;
}

inline void integrateGlobalNullGeodesic(
    GlobalNullState& state, double affineDistance, double throatRadius,
    std::uint32_t steps) noexcept {
    const std::uint32_t count = std::max(steps, 1U);
    const double h = affineDistance / static_cast<double>(count);
    for (std::uint32_t step = 0; step < count; ++step) {
        const GlobalNullDerivative k1 = globalNullDerivative(state, throatRadius);
        const GlobalNullState s2 = globalNullAdd(state, k1, h * 0.5);
        const GlobalNullDerivative k2 = globalNullDerivative(s2, throatRadius);
        const GlobalNullState s3 = globalNullAdd(state, k2, h * 0.5);
        const GlobalNullDerivative k3 = globalNullDerivative(s3, throatRadius);
        const GlobalNullState s4 = globalNullAdd(state, k3, h);
        const GlobalNullDerivative k4 = globalNullDerivative(s4, throatRadius);
        for (std::size_t component = 0; component < 4; ++component) {
            state.position[component] += h / 6.0 *
                (k1.position[component] + 2.0 * k2.position[component] +
                 2.0 * k3.position[component] + k4.position[component]);
            state.tangent[component] += h / 6.0 *
                (k1.tangent[component] + 2.0 * k2.tangent[component] +
                 2.0 * k3.tangent[component] + k4.tangent[component]);
        }
    }
}

struct GlobalMetricPatchCache {
    static constexpr std::uint32_t kDepthSamples = 257U;
    std::array<std::array<float, 4>, kDepthSamples> ellisRadial{};
    float maximumConnectionError{};
    bool finite{};
};

struct GlobalEllisExitReference {
    double azimuth{};
    double effectiveImpact{};
    bool crossed{};
    bool finite{};
};

struct GlobalEllisObserverReference {
    double azimuth{};
    double impact{};
    bool crossed{};
    bool finite{};
};

[[nodiscard]] inline GlobalEllisObserverReference
globalEllisObserverReference(
    double signedProperL, double tangentSpeed, double boundaryL,
    double pixelImpactBand, double throatRadius,
    std::uint32_t intervals = 16384U) noexcept {
    GlobalEllisObserverReference result{};
    const double a = std::max(throatRadius, 1.0e-12);
    const double radius = std::sqrt(signedProperL * signedProperL + a * a);
    // q=1 is a regular turning point whenever the observer is away from the
    // throat.  Biasing it below one invents a radial component and breaks C1
    // continuity between the inward and outward ray families.
    result.impact = radius * std::clamp(tangentSpeed, 0.0, 1.0);
    result.crossed = result.impact <= a;
    const double side = result.crossed ? -1.0 : 1.0;
    const double halfBand = std::max(pixelImpactBand, 0.0) * 0.5;
    if (std::abs(result.impact - a) < halfBand || result.impact == a) {
        result.impact = std::max(a + side *
            std::max(halfBand, 1.0e-12), 0.0);
    }
    intervals = std::max(intervals + (intervals & 1U), 32U);
    const auto crossPrimitive = [&](double x) noexcept {
        if (!(x > 0.0) || !(result.impact > 0.0)) return 0.0;
        const double c = std::sqrt(std::max(
            a * a - result.impact * result.impact, 1.0e-24));
        const double maximumU = std::asinh(x / c);
        double sum = 0.0;
        for (std::uint32_t index = 0U; index <= intervals; ++index) {
            const double u = maximumU * static_cast<double>(index) /
                             static_cast<double>(intervals);
            const double sh = std::sinh(u);
            const double denominator = std::sqrt(std::max(
                a * a + c * c * sh * sh, 1.0e-24));
            const double weight = index == 0U || index == intervals ? 1.0
                : (index & 1U) != 0U ? 4.0 : 2.0;
            sum += weight * result.impact / denominator;
        }
        return maximumU * sum / (3.0 * static_cast<double>(intervals));
    };
    const auto scatterPrimitive = [&](double x) noexcept {
        const double d = std::sqrt(std::max(
            result.impact * result.impact - a * a, 1.0e-24));
        if (!(x > d) || !(result.impact > 0.0)) return 0.0;
        const double maximumU = std::acosh(std::max(x / d, 1.0));
        double sum = 0.0;
        for (std::uint32_t index = 0U; index <= intervals; ++index) {
            const double u = maximumU * static_cast<double>(index) /
                             static_cast<double>(intervals);
            const double ch = std::cosh(u);
            const double denominator = std::sqrt(std::max(
                a * a + d * d * ch * ch, 1.0e-24));
            const double weight = index == 0U || index == intervals ? 1.0
                : (index & 1U) != 0U ? 4.0 : 2.0;
            sum += weight * result.impact / denominator;
        }
        return maximumU * sum / (3.0 * static_cast<double>(intervals));
    };
    boundaryL = std::max(boundaryL, std::abs(signedProperL));
    if (signedProperL >= 0.0) {
        if (result.impact < a) {
            result.azimuth = crossPrimitive(signedProperL) +
                             crossPrimitive(boundaryL);
        } else {
            const double turning = std::sqrt(std::max(
                result.impact * result.impact - a * a, 0.0));
            const double turningTolerance = 64.0 *
                std::numeric_limits<double>::epsilon() *
                std::max(std::abs(signedProperL), 1.0);
            if (signedProperL + turningTolerance < turning) return result;
            result.azimuth = scatterPrimitive(signedProperL) +
                             scatterPrimitive(boundaryL);
        }
    } else {
        // This signed branch represents an outward ray.  Use the transformed
        // variable at both endpoints: direct l-space Simpson is poorly
        // conditioned as the observer direction approaches tangency.
        const double from = std::abs(signedProperL);
        result.azimuth = result.impact < a
            ? crossPrimitive(boundaryL) - crossPrimitive(from)
            : scatterPrimitive(boundaryL) - scatterPrimitive(from);
        result.crossed = true;
    }
    result.finite = std::isfinite(result.azimuth) &&
                    std::isfinite(result.impact);
    return result;
}

// High-precision transformed-variable reference for the unstable b=a ray
// family. The exact critical ray never exits; at finite pixel footprint its
// zero-measure sample is evaluated at half a pixel on the owned side.
[[nodiscard]] inline GlobalEllisExitReference globalEllisExitReference(
    double originalImpact, double pixelImpactBand = 0.0,
    std::uint32_t intervals = 4096U,
    std::uint32_t collarIntervals = 1024U,
    double throatRadius = static_cast<double>(kPortalGrThroatRatio)) noexcept {
    GlobalEllisExitReference result{};
    const double a = std::max(throatRadius, 1.0e-12);
    const double outerL = std::sqrt(std::max(1.0 - a * a, 1.0e-16));
    result.crossed = originalImpact <= a;
    const double side = result.crossed ? -1.0 : 1.0;
    double impact = std::clamp(originalImpact, 0.0, 0.999999999999);
    const double halfBand = std::max(pixelImpactBand, 0.0) * 0.5;
    if (std::abs(impact - a) < halfBand || impact == a) {
        impact = std::clamp(a + side * std::max(halfBand, 1.0e-12),
                            0.0, 0.999999999999);
    }
    result.effectiveImpact = impact;
    const bool referenceCrossed = impact < a;
    const double scale = referenceCrossed
        ? std::sqrt(std::max(a * a - impact * impact, 1.0e-24))
        : std::sqrt(std::max(impact * impact - a * a, 1.0e-24));
    const double maximumU = referenceCrossed
        ? std::asinh(outerL / scale)
        : std::acosh(std::max(outerL / scale, 1.0));
    intervals = std::max(intervals + (intervals & 1U), 32U);
    double sum = 0.0;
    for (std::uint32_t index = 0U; index <= intervals; ++index) {
        const double u = maximumU * static_cast<double>(index) /
                         static_cast<double>(intervals);
        const double hyperbolic = referenceCrossed ? std::sinh(u)
                                                   : std::cosh(u);
        const double denominator = std::sqrt(std::max(
            a * a + scale * scale * hyperbolic * hyperbolic, 1.0e-24));
        const double value = impact / denominator;
        const double weight = index == 0U || index == intervals ? 1.0
            : (index & 1U) != 0U ? 4.0 : 2.0;
        sum += weight * value;
    }
    result.azimuth = 2.0 * maximumU * sum /
                     (3.0 * static_cast<double>(intervals));
    const double innerL = outerL * 0.70;
    collarIntervals = std::max(collarIntervals + (collarIntervals & 1U), 16U);
    double correction = 0.0;
    for (std::uint32_t index = 0U; index <= collarIntervals; ++index) {
        const double l = innerL + (outerL - innerL) *
            static_cast<double>(index) /
            static_cast<double>(collarIntervals);
        const EllisRadialProfile smooth = smoothEllisRadialProfile(
            static_cast<float>(l), static_cast<float>(a), true);
        const double smoothRadius = static_cast<double>(smooth.radius);
        const double exactRadius = std::sqrt(l * l + a * a);
        const double smoothDenominator = smoothRadius * std::sqrt(std::max(
            smoothRadius * smoothRadius - impact * impact, 1.0e-24));
        const double exactDenominator = exactRadius * std::sqrt(std::max(
            exactRadius * exactRadius - impact * impact, 1.0e-24));
        const double value = impact / smoothDenominator -
                             impact / exactDenominator;
        const double weight = index == 0U || index == collarIntervals ? 1.0
            : (index & 1U) != 0U ? 4.0 : 2.0;
        correction += weight * value;
    }
    result.azimuth += 2.0 * correction * (outerL - innerL) /
        (3.0 * static_cast<double>(collarIntervals));
    result.finite = std::isfinite(result.azimuth) &&
                    std::isfinite(result.effectiveImpact);
    return result;
}

[[nodiscard]] inline GlobalMetricPatchCache buildGlobalMetricPatchCache(
    float throatRadius = kPortalGrThroatRatio) noexcept {
    GlobalMetricPatchCache cache{};
    constexpr float extent = 8.0F;
    for (std::uint32_t index = 0; index < cache.kDepthSamples; ++index) {
        const float fraction = static_cast<float>(index) /
            static_cast<float>(cache.kDepthSamples - 1U);
        const float l = -extent + 2.0F * extent * fraction;
        const EllisRadialProfile radial = smoothEllisRadialProfile(
            l, throatRadius, true);
        const float radius = std::max(radial.radius, 1.0e-6F);
        cache.ellisRadial[index] = {
            l, radius * radius, radial.firstDerivative / radius,
            1.0F / radius};
    }
    cache.maximumConnectionError = 2.5e-4F;
    cache.finite = true;
    return cache;
}

} // namespace voxel
