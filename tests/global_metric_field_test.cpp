#include "app/intrinsic_ellis_camera.hpp"
#include "render/global_metric_field.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {
void require(bool value, const std::string& message) {
    if (!value) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

struct OdeExitReference {
    double azimuth{};
    bool crossed{};
    bool finite{};
};

struct ObserverMappedRay {
    std::array<double, 3> origin{};
    std::array<double, 3> direction{};
    bool finite{};
};

double vectorAngle(const std::array<double, 3>& a,
                   const std::array<double, 3>& b) {
    const double aLength = std::sqrt(
        a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const double bLength = std::sqrt(
        b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    const double cosine = std::clamp(
        (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) /
            std::max(aLength * bLength, 1.0e-30), -1.0, 1.0);
    return std::acos(cosine);
}

// Independent reference: integrate the metric connection itself.  It does
// not call either transformed-variable primitive used by the CPU/GPU fast
// path, so an algebraic error in that closed form cannot validate itself.
OdeExitReference integrateObserverOde(double startL, double radialSign,
                                      double impact, double boundaryL,
                                      double throatRadius) {
    using namespace voxel;
    OdeExitReference result{};
    const double radius = std::sqrt(startL * startL +
                                    throatRadius * throatRadius);
    const double q = std::clamp(impact / radius, 0.0, 0.999999999999);
    GlobalNullState state{};
    state.chart = EllisOracleChart::Ellis;
    state.position = {0.0, startL, 1.57079632679489661923, 0.0};
    state.tangent = {1.0,
                     std::copysign(std::sqrt(std::max(1.0 - q * q, 0.0)),
                                   radialSign),
                     0.0, impact / (radius * radius)};
    const bool movingTowardThroat = startL * radialSign < 0.0;
    const bool crosses = movingTowardThroat && impact < throatRadius;
    const double target = std::copysign(
        boundaryL, crosses ? -startL : startL);
    constexpr double affineStep = 2.5e-5;
    constexpr std::uint32_t maximumSteps = 2'000'000U;
    for (std::uint32_t step = 0U; step < maximumSteps; ++step) {
        const GlobalNullState previous = state;
        integrateGlobalNullGeodesic(state, affineStep, throatRadius, 1U);
        if (!std::isfinite(state.position[1]) ||
            !std::isfinite(state.position[3]) ||
            !std::isfinite(state.tangent[1]) ||
            !std::isfinite(state.tangent[3])) {
            return result;
        }
        const bool reached = target > 0.0
            ? previous.position[1] < target && state.position[1] >= target
            : previous.position[1] > target && state.position[1] <= target;
        if (reached) {
            const double denominator = state.position[1] -
                                       previous.position[1];
            const double fraction = std::abs(denominator) > 1.0e-15
                ? std::clamp((target - previous.position[1]) / denominator,
                             0.0, 1.0)
                : 1.0;
            result.azimuth = std::abs(std::lerp(
                previous.position[3], state.position[3], fraction));
            result.crossed = crosses;
            result.finite = std::isfinite(result.azimuth);
            return result;
        }
    }
    return result;
}

ObserverMappedRay mapRecordedObserverRay(
    double cameraL, double radialComponent, double boundaryL,
    double throatRadius, bool useIndependentOde) {
    using namespace voxel;
    ObserverMappedRay result{};
    const double q = std::sqrt(std::max(
        1.0 - radialComponent * radialComponent, 0.0));
    // This exactly mirrors the GPU's half-open ownership: zero belongs to the
    // non-positive side, but q itself is never biased away from one.
    const bool positiveRadialOwner = radialComponent > 0.0;
    const double sampleL = positiveRadialOwner ? -cameraL : cameraL;
    const auto fast = globalEllisObserverReference(
        sampleL, q, boundaryL, 0.0, throatRadius, 32768U);
    if (!fast.finite) return result;
    double azimuth = fast.azimuth;
    if (useIndependentOde) {
        const auto ode = integrateObserverOde(
            sampleL, -1.0, fast.impact, boundaryL, throatRadius);
        if (!ode.finite) return result;
        azimuth = ode.azimuth;
    }
    const double boundaryRadius = std::sqrt(
        boundaryL * boundaryL + throatRadius * throatRadius);
    const double qExit = std::clamp(
        fast.impact / boundaryRadius, 0.0, 1.0);
    const double radial = std::sqrt(std::max(1.0 - qExit * qExit, 0.0));
    const std::array<double, 3> exitN{
        std::cos(azimuth), std::sin(azimuth), 0.0};
    const std::array<double, 3> exitTangent{
        -std::sin(azimuth), std::cos(azimuth), 0.0};
    result.origin = {boundaryRadius * exitN[0],
                     boundaryRadius * exitN[1], 0.0};
    result.direction = {
        radial * exitN[0] + qExit * exitTangent[0],
        radial * exitN[1] + qExit * exitTangent[1], 0.0};
    result.finite = std::isfinite(result.origin[0]) &&
                    std::isfinite(result.origin[1]) &&
                    std::isfinite(result.direction[0]) &&
                    std::isfinite(result.direction[1]);
    return result;
}
}

int main() {
    using namespace voxel;
    require(globalObserverTerrainOwnsBeforeMouth(
                true, true, 0.8, 1.2) &&
                !globalObserverTerrainOwnsBeforeMouth(
                    true, true, 1.2, 1.2) &&
                !globalObserverTerrainOwnsBeforeMouth(
                    true, false, 0.8, 1.2) &&
                !globalObserverTerrainOwnsBeforeMouth(
                    false, true, 0.8, 1.2),
            "observer-side terrain ownership was not exact/front-facing/half-open");
    const GlobalVector4 externalPosition{0.0, 1.0, -2.0, 0.5};
    const GlobalMetricSample external = evaluateEllisOracleMetric(
        EllisOracleChart::FlatReference, externalPosition);
    require(external.finite && external.metric[0][0] == -1.0 &&
                external.metric[3][3] == 1.0,
            "external asymptotic metric is not Minkowski");

    GlobalNullState flat{};
    flat.position = externalPosition;
    flat.tangent = {1.0, 0.6, 0.0, 0.8};
    const double flatNullBefore = globalMetricInner(
        external, flat.tangent, flat.tangent);
    integrateGlobalNullGeodesic(flat, 10.0, kPortalGrThroatRatio, 8U);
    const double flatNullAfter = globalMetricInner(
        evaluateEllisOracleMetric(flat.chart, flat.position), flat.tangent,
        flat.tangent);
    require(std::abs(flatNullBefore) < 1.0e-12 &&
                std::abs(flatNullAfter) < 1.0e-12 &&
                std::abs(flat.position[1] - 7.0) < 1.0e-10,
            "global external null ray is not an exact straight geodesic");

    GlobalNullState radial{};
    radial.chart = EllisOracleChart::Ellis;
    radial.position = {0.0, 0.62, 1.1, 0.7};
    radial.tangent = {1.0, -1.0, 0.0, 0.0};
    const GlobalNullState radialStart = radial;
    integrateGlobalNullGeodesic(radial, 0.92, kPortalGrThroatRatio, 512U);
    require(std::abs(radial.position[1] + 0.30) < 2.0e-8 &&
                std::abs(radial.position[2] - radialStart.position[2]) < 1.0e-12,
            "global Ellis radial null ray did not cross l=0 continuously");
    integrateGlobalNullGeodesic(radial, -0.92, kPortalGrThroatRatio, 512U);
    require(std::abs(radial.position[1] - radialStart.position[1]) < 2.0e-8,
            "global metric integration failed time reversal");

    const GlobalMetricSample throat = evaluateEllisOracleMetric(
        EllisOracleChart::Ellis, {0.0, 0.0, 1.2, 0.4});
    require(throat.finite &&
                std::abs(throat.metric[2][2] -
                    kPortalGrThroatRatio * kPortalGrThroatRatio) < 1.0e-7,
            "Ellis throat metric does not have the analytic areal radius");

    for (const PortalVector pole : {
             PortalVector{0.0F, 1.0F, 0.0F},
             PortalVector{0.0F, -1.0F, 0.0F},
             portalNormalize({1.0e-8F, 1.0F, -2.0e-8F})}) {
        const auto basis = ellisAngularBasis(pole);
        require(portalFinite(basis[0]) && portalFinite(basis[1]) &&
                    std::abs(portalDot(basis[0], basis[1])) < 1.0e-5F,
                "global chart atlas is singular at an angular pole");
    }

    const GlobalMetricPatchCache cache = buildGlobalMetricPatchCache();
    require(cache.finite && cache.maximumConnectionError < 3.0e-4F &&
                cache.ellisRadial.front()[0] < 0.0F &&
                cache.ellisRadial.back()[0] > 0.0F,
            "global metric/connection patch cache is invalid");

    constexpr float mouthRadius = 0.34F;
    constexpr float throatRadius = mouthRadius * kPortalGrThroatRatio;
    const float outerL = std::sqrt(
        mouthRadius * mouthRadius - throatRadius * throatRadius);
    float previousCurvature = std::numeric_limits<float>::infinity();
    for (const float l : {0.0F, outerL * 0.5F, outerL, outerL * 2.0F,
                          outerL * 4.0F, outerL * 8.0F}) {
        const auto radial = smoothEllisRadialProfile(l, throatRadius, true);
        const float curvature = throatRadius * throatRadius /
            std::max(radial.radius * radial.radius * radial.radius *
                     radial.radius, 1.0e-12F);
        require(radial.radius >= throatRadius &&
                    radial.firstDerivative >= 0.0F &&
                    radial.firstDerivative < 1.0F &&
                    curvature <= previousCurvature + 1.0e-5F,
                "all-space handle curvature is not finite/monotone");
        previousCurvature = curvature;
    }
    const auto attachment = smoothEllisRadialProfile(
        outerL, throatRadius, true);
    require(std::abs(attachment.radius - mouthRadius) < 2.0e-6F,
            "physical Ellis metric does not match the configured mouth radius");

    GlobalNullState formerBoundaryRay{};
    formerBoundaryRay.chart = EllisOracleChart::Ellis;
    formerBoundaryRay.position = {0.0, static_cast<double>(outerL + 0.08F),
                          1.5707963267948966, 0.0};
    formerBoundaryRay.tangent = {1.0, -0.88, 0.0,
                         std::sqrt(1.0 - 0.88 * 0.88) /
                             smoothEllisRadialProfile(
                                 outerL + 0.08F, throatRadius, true).radius};
    const GlobalNullState formerBoundaryStart = formerBoundaryRay;
    integrateGlobalNullGeodesic(
        formerBoundaryRay, 0.42, throatRadius, 4096U);
    integrateGlobalNullGeodesic(
        formerBoundaryRay, -0.42, throatRadius, 4096U);
    require(std::abs(formerBoundaryRay.position[1] -
                         formerBoundaryStart.position[1]) <
                    2.0e-8 &&
                std::abs(formerBoundaryRay.position[3] -
                         formerBoundaryStart.position[3]) <
                    2.0e-8,
            "ray lost C1/time-reversal continuity through the former border");

    constexpr double critical = static_cast<double>(kPortalGrThroatRatio);
    constexpr double pixelBand = 2.0 / 900.0;
    for (const double impact : {
             critical - 0.08, critical - 0.02, critical - 0.004,
             critical - 1.0e-6, critical, critical + 1.0e-6,
             critical + 0.004, critical + 0.02, critical + 0.08}) {
        const GlobalEllisExitReference reference =
            globalEllisExitReference(impact, pixelBand);
        require(reference.finite && reference.azimuth >= 0.0 &&
                    reference.azimuth < 32.0 &&
                    reference.crossed == (impact <= critical),
                "near-critical smooth-handle ray remained unresolved");
    }
    for (const double impact : {
             critical - 0.08, critical - 0.02,
             critical + 0.02, critical + 0.08}) {
        const GlobalEllisExitReference gpuBudget =
            globalEllisExitReference(impact, pixelBand, 96U, 48U);
        const GlobalEllisExitReference oracle =
            globalEllisExitReference(impact, pixelBand, 16384U, 4096U);
        require(gpuBudget.finite && oracle.finite &&
                    gpuBudget.crossed == oracle.crossed &&
                    std::abs(gpuBudget.azimuth - oracle.azimuth) < 7.5e-4,
                "bounded GPU critical resolver exceeded the smooth-handle oracle");
    }

    const double physicalBoundary = 3.0 * mouthRadius;
    const double formerSeam = outerL;
    const double seamStep = 1.0e-4;
    const double viewQ = 0.20;
    const auto observerAngle = [&](double l, double q) {
        return globalEllisObserverReference(
            l, q, physicalBoundary,
            std::sqrt(l * l + throatRadius * throatRadius) *
                (2.0 / 900.0),
            throatRadius, 8192U).azimuth;
    };
    const double seamAngle = observerAngle(formerSeam, viewQ);
    const double leftSlope = (seamAngle -
        observerAngle(formerSeam - seamStep, viewQ)) / seamStep;
    const double rightSlope = (observerAngle(
        formerSeam + seamStep, viewQ) - seamAngle) / seamStep;
    require(std::isfinite(leftSlope) && std::isfinite(rightSlope) &&
                std::abs(leftSlope - rightSlope) < 0.02,
            "lens mapping retains a first-derivative seam at the old boundary");
    const auto jacobian = [&](double l) {
        constexpr double dq = 1.0e-4;
        return (observerAngle(l, viewQ + dq) -
                observerAngle(l, viewQ - dq)) / (2.0 * dq);
    };
    const double leftJacobian = jacobian(formerSeam - seamStep);
    const double rightJacobian = jacobian(formerSeam + seamStep);
    require(std::isfinite(leftJacobian) && std::isfinite(rightJacobian) &&
                std::abs(leftJacobian - rightJacobian) < 0.02,
            "screen-space lens Jacobian changes at the former sphere");

    const double closeObserverL = outerL * 0.38;
    const double closeRadius = std::sqrt(closeObserverL * closeObserverL +
                                         throatRadius * throatRadius);
    const double criticalQ = throatRadius / closeRadius;
    const double closeBand = closeRadius * (2.0 / 900.0);
    for (const double q : {criticalQ - 1.0e-6, criticalQ,
                           criticalQ + 1.0e-6}) {
        const auto closeReference = globalEllisObserverReference(
            closeObserverL, q, physicalBoundary, closeBand,
            throatRadius, 16384U);
        require(closeReference.finite && closeReference.azimuth >= 0.0 &&
                    closeReference.azimuth < 32.0,
                "close inside-observer critical family has no finite exit");
    }
    for (const double q : {0.08, 0.20, 0.42, 0.72}) {
        const auto gpuBudget = globalEllisObserverReference(
            closeObserverL, q, physicalBoundary, closeBand,
            throatRadius, 24U);
        const auto oracle = globalEllisObserverReference(
            closeObserverL, q, physicalBoundary, closeBand,
            throatRadius, 16384U);
        require(gpuBudget.finite && oracle.finite &&
                    gpuBudget.crossed == oracle.crossed &&
                    std::abs(gpuBudget.azimuth - oracle.azimuth) < 9.0e-4,
                "regular all-space GPU integral exceeded observer oracle bound");
    }

    // Compare the transformed-variable fast path to a genuinely independent
    // connection-ODE integration for arbitrary starts and content boundaries.
    // Negative signed starts exercise the outward branch used by the GPU's
    // radial-sign transform; positive starts cover crossing and same-end
    // scattering rays.
    for (const double boundary : {mouthRadius * 1.55,
                                  mouthRadius * 3.0}) {
        for (const double signedL : {
                 static_cast<double>(closeObserverL),
                 static_cast<double>(outerL) * 1.35,
                 -static_cast<double>(closeObserverL),
                 -static_cast<double>(outerL) * 1.35}) {
            const double radius = std::sqrt(
                signedL * signedL + throatRadius * throatRadius);
            const double qCritical = throatRadius / radius;
            for (const double q : {
                     0.12, std::max(qCritical - 0.018, 0.01),
                     std::min(qCritical + 0.018, 0.96), 0.82}) {
                const auto fast = globalEllisObserverReference(
                    signedL, q, boundary, 0.0, throatRadius, 32768U);
                const auto ode = integrateObserverOde(
                    signedL, -1.0, fast.impact, boundary, throatRadius);
                require(fast.finite && ode.finite &&
                            std::abs(fast.azimuth - ode.azimuth) < 2.5e-4,
                        "closed-form observer map diverged from independent ODE");
                if (signedL > 0.0) {
                    require(fast.crossed == ode.crossed,
                            "observer map chose the wrong Ellis end");
                }
            }
        }
    }

    // Tangency is where the old 2*observer primitive created the visible
    // sheet: inward rays approached zero deflection while outward neighbors
    // retained the boundary tail.  Require the mapped angle and its practical
    // screen derivative to agree across radial-sign zero.
    const auto tangentAngle = [&](double radialComponent) {
        const double q = std::sqrt(std::max(
            1.0 - radialComponent * radialComponent, 0.0));
        const double transformedL = radialComponent > 0.0
            ? -static_cast<double>(closeObserverL)
            : static_cast<double>(closeObserverL);
        return globalEllisObserverReference(
            transformedL, q, physicalBoundary, 0.0,
            throatRadius, 32768U).azimuth;
    };
    constexpr double tangentStep = 2.0e-4;
    const double tangentCenter = tangentAngle(0.0);
    const double tangentLeft = tangentAngle(-tangentStep);
    const double tangentRight = tangentAngle(tangentStep);
    const double tangentLeftSlope =
        (tangentCenter - tangentLeft) / tangentStep;
    const double tangentRightSlope =
        (tangentRight - tangentCenter) / tangentStep;
    require(std::abs(tangentLeft - tangentRight) < 7.5e-4,
            "observer exit direction is not C0 across tangency: left=" +
                std::to_string(tangentLeft) + " center=" +
                std::to_string(tangentCenter) + " right=" +
                std::to_string(tangentRight));
    require(std::isfinite(tangentLeftSlope) &&
                std::isfinite(tangentRightSlope) &&
                std::abs(tangentLeftSlope - tangentRightSlope) < 0.08,
            "observer exit direction is not practically C1 across tangency: left=" +
                std::to_string(tangentLeftSlope) + " right=" +
                std::to_string(tangentRightSlope));

    // Replay the reported lateral path through the actual wrapper policy.
    // The legacy q<=0.9998 call-site clamp produced a fixed ~0.04 rad hole,
    // large enough to swallow the final 12-20 pixel terrain image at 900p.
    const double negativeUlp = std::nextafter(0.0, -1.0);
    const double positiveUlp = std::nextafter(0.0, 1.0);
    for (const double recordedL : {
             0.446271, 1.800275, 2.002498,
             2.250839, 2.502278, 2.594145}) {
        const double observerBoundary = recordedL + throatRadius * 0.05;
        const double minimumBoundary = static_cast<double>(outerL);
        const double blendWidth = throatRadius * 0.25;
        const double difference = observerBoundary - minimumBoundary;
        const double boundary = 0.5 * (
            observerBoundary + minimumBoundary +
            std::sqrt(difference * difference + blendWidth * blendWidth));
        const ObserverMappedRay left = mapRecordedObserverRay(
            recordedL, negativeUlp, boundary, throatRadius, false);
        const ObserverMappedRay center = mapRecordedObserverRay(
            recordedL, 0.0, boundary, throatRadius, false);
        const ObserverMappedRay right = mapRecordedObserverRay(
            recordedL, positiveUlp, boundary, throatRadius, false);
        require(left.finite && center.finite && right.finite &&
                    vectorAngle(left.direction, center.direction) < 1.0e-6 &&
                    vectorAngle(center.direction, right.direction) < 1.0e-6 &&
                    vectorAngle(left.origin, right.origin) < 1.0e-6,
                "full observer wrapper is not C0 at +/-ULP tangency for l=" +
                    std::to_string(recordedL) + " direction left/center=" +
                    std::to_string(vectorAngle(left.direction,
                                               center.direction)) +
                    " center/right=" +
                    std::to_string(vectorAngle(center.direction,
                                               right.direction)) +
                    " origin=" +
                    std::to_string(vectorAngle(left.origin, right.origin)));

        const ObserverMappedRay odeLeft = mapRecordedObserverRay(
            recordedL, negativeUlp, boundary, throatRadius, true);
        const ObserverMappedRay odeCenter = mapRecordedObserverRay(
            recordedL, 0.0, boundary, throatRadius, true);
        const ObserverMappedRay odeRight = mapRecordedObserverRay(
            recordedL, positiveUlp, boundary, throatRadius, true);
        require(odeLeft.finite && odeCenter.finite && odeRight.finite &&
                    vectorAngle(left.direction, odeLeft.direction) < 2.0e-3 &&
                    vectorAngle(center.direction, odeCenter.direction) < 2.0e-3 &&
                    vectorAngle(right.direction, odeRight.direction) < 2.0e-3,
                "recorded-path tangency map diverged from independent ODE for l=" +
                    std::to_string(recordedL));

        constexpr double derivativeStep = 2.0e-4;
        const ObserverMappedRay practicalLeft = mapRecordedObserverRay(
            recordedL, -derivativeStep, boundary, throatRadius, false);
        const ObserverMappedRay practicalRight = mapRecordedObserverRay(
            recordedL, derivativeStep, boundary, throatRadius, false);
        const double leftRate = vectorAngle(
            practicalLeft.direction, center.direction) / derivativeStep;
        const double rightRate = vectorAngle(
            center.direction, practicalRight.direction) / derivativeStep;
        require(std::isfinite(leftRate) && std::isfinite(rightRate) &&
                    std::abs(leftRate - rightRate) < 0.08,
                "full observer wrapper is not practically C1 for l=" +
                    std::to_string(recordedL));
    }
    {
        constexpr double recordedL = 2.594145;
        constexpr double legacyQ = 0.9998;
        const double observerBoundary = recordedL + throatRadius * 0.05;
        const double difference = observerBoundary - outerL;
        const double boundary = 0.5 * (observerBoundary + outerL +
            std::sqrt(difference * difference +
                      throatRadius * throatRadius * 0.0625));
        const double inward = globalEllisObserverReference(
            recordedL, legacyQ, boundary, 0.0,
            throatRadius, 32768U).azimuth;
        const double outward = globalEllisObserverReference(
            -recordedL, legacyQ, boundary, 0.0,
            throatRadius, 32768U).azimuth;
        require(inward - outward > 0.03,
                "tangency regression no longer discriminates the legacy clamp");
    }

    // Explicitly mirror the same physical ray on both Ellis ends and both
    // radial signs.  The ODE is the authority for chart/sign symmetry.
    for (const double startSign : {-1.0, 1.0}) {
        for (const double radialSign : {-1.0, 1.0}) {
            const double startL = startSign * closeObserverL;
            const double radius = std::sqrt(
                startL * startL + throatRadius * throatRadius);
            const double impact = radius * 0.41;
            const auto ode = integrateObserverOde(
                startL, radialSign, impact, physicalBoundary,
                throatRadius);
            require(ode.finite && ode.azimuth >= 0.0,
                    "independent ODE failed a chart/radial-sign ray");
        }
    }

    IntrinsicEllisSettings settings{};
    settings.globalMetricField = true;
    settings.contentSphereRadius = mouthRadius;
    settings.throatRadius = throatRadius;
    settings.contentExitProperDepth = outerL;
    settings.freeFlySpeed = 0.4F;
    require(std::abs(settings.globalHandleLengthDiameters - 2.00F) < 1.0e-7F,
            "global handle default is not the accepted 2.00-diameter path");

    // Mouse look must rotate the authoritative intrinsic tetrad while the
    // observer is in the handle chart.  Before this regression existed,
    // IntrinsicEllisCamera::rotate changed only the exterior forward/up
    // mirror; the next move/render sync copied the untouched handleFrame back
    // over it, so F7 look appeared disabled anywhere inside either mouth.
    const auto makeHandleObserver = [&settings](
        std::uint32_t source, float signedDepth,
        PortalVector exteriorNormal) {
        const auto centers = globalHandleCenters(settings);
        exteriorNormal = portalNormalize(exteriorNormal);
        GlobalHandleRayState ray{};
        ray.origin = centers[source] + exteriorNormal *
            globalHandleMouthRadius(settings);
        ray.direction = exteriorNormal * -1.0F;
        ray.footprintU = ray.direction;
        const PortalVector upReference = std::abs(ray.direction.y) < 0.92F
            ? PortalVector{0.0F, 1.0F, 0.0F}
            : PortalVector{1.0F, 0.0F, 0.0F};
        ray.footprintV = portalOrthonormalUp(ray.direction, upReference);
        globalHandleEnterInterior(ray, source, settings);
        ray.handleState.properDepth = signedDepth;
        GlobalHandleObserverState result{};
        result.position = ray.origin;
        result.forward = portalNormalize(ellisTangentToEmbedded(
            ray.handleFrame.forward, ray.handleState.angularPosition));
        result.up = portalOrthonormalUp(result.forward,
            ellisTangentToEmbedded(ray.handleFrame.up,
                                   ray.handleState.angularPosition));
        result.lastMouth = source;
        result.chart = 1U;
        result.handleState = ray.handleState;
        result.handleFrame = ray.handleFrame;
        result.finite = true;
        return result;
    };
    const auto packGlobalCamera = [&settings](
        const GlobalHandleObserverState& observer) {
        IntrinsicEllisSettings packed = settings;
        packed.globalPosition = observer.position;
        packed.globalForward = observer.forward;
        packed.globalUp = observer.up;
        packed.globalVelocity = observer.velocity;
        packed.globalLastMouth = observer.lastMouth;
        packed.globalAffineDistance = observer.affineDistance;
        packed.globalHandleChart = observer.chart;
        packed.globalHandleU = observer.handleState.properDepth;
        packed.globalHandleN = observer.handleState.angularPosition;
        packed.globalHandleForwardLocal = ellisTangentToEmbedded(
            observer.handleFrame.forward,
            observer.handleState.angularPosition);
        packed.globalHandleUpLocal = ellisTangentToEmbedded(
            observer.handleFrame.up,
            observer.handleState.angularPosition);
        return intrinsicEllisGpuParameters(packed, {}, {});
    };

    // The least-aligned angular basis intentionally changes reference axes.
    // It is safe as a temporary coordinate chart, but it must never be the
    // persistent camera packet: the old packet produced a whole-screen flip
    // at this exact x/y ownership boundary even though the intrinsic tangent
    // was continuous. The embedded representation has no such branch.
    const PortalVector basisN0 = portalNormalize(
        {0.20001F, 0.19999F, 0.95917F});
    const PortalVector basisN1 = portalNormalize(
        {0.19999F, 0.20001F, 0.95917F});
    const PortalVector embeddedReference = portalNormalize(
        {0.37F, -0.71F, 0.59F});
    const EllisTangent basisT0 = ellisNormalize(
        ellisTangentFromEmbedded(embeddedReference, basisN0), basisN0);
    const EllisTangent basisT1 = ellisNormalize(
        ellisTangentFromEmbedded(embeddedReference, basisN1), basisN1);
    const PortalVector embedded0 = portalNormalize(
        ellisTangentToEmbedded(basisT0, basisN0));
    const PortalVector embedded1 = portalNormalize(
        ellisTangentToEmbedded(basisT1, basisN1));
    const PortalVector legacyLocal0 = portalNormalize(
        ellisTangentToLocal(basisT0, basisN0));
    const PortalVector legacyLocal1 = portalNormalize(
        ellisTangentToLocal(basisT1, basisN1));
    require(portalDot(embedded0, embedded1) > 0.999999F,
            "basis-free handle camera packet is not C0 across axis owner");
    require(portalDot(legacyLocal0, legacyLocal1) < 0.90F,
            "basis-switch regression no longer exercises a discontinuity");
    const EllisTangent embeddedRoundTrip = ellisNormalize(
        ellisTangentFromEmbedded(embedded0, basisN0), basisN0);
    require(ellisDot(embeddedRoundTrip, basisT0) > 0.999999F,
            "embedded handle tangent packet failed round trip");
    const auto reconstructPackedScreenRay = [](PortalVector n,
                                                PortalVector packedForward,
                                                PortalVector packedUp,
                                                float screenX,
                                                float screenY) {
        packedForward = portalNormalize(packedForward);
        packedUp = portalOrthonormalUp(packedForward, packedUp);
        const PortalVector packedRight = portalNormalize(portalCross(
            packedForward, packedUp));
        const PortalVector embeddedDirection = portalNormalize(
            packedForward + packedRight * screenX + packedUp * screenY);
        return ellisNormalize(ellisTangentFromEmbedded(
            embeddedDirection, n), n);
    };
    for (const auto& sample : std::array<std::pair<PortalVector,
                                                   EllisTangent>, 2>{
             std::pair{basisN0, basisT0}, std::pair{basisN1, basisT1}}) {
        const PortalVector packedForward = ellisTangentToEmbedded(
            sample.second, sample.first);
        const PortalVector upSeed = portalOrthonormalUp(
            packedForward, {0.12F, 0.91F, 0.38F});
        const EllisTangent packedRay = reconstructPackedScreenRay(
            sample.first, packedForward, upSeed, 0.17F, -0.09F);
        const EllisTangent directForward = ellisTangentFromEmbedded(
            packedForward, sample.first);
        const EllisTangent directRight = ellisTangentFromEmbedded(
            portalNormalize(portalCross(packedForward, upSeed)),
            sample.first);
        const EllisTangent directUp = ellisTangentFromEmbedded(
            upSeed, sample.first);
        const EllisTangent directRay = ellisNormalize(
            directForward + directRight * 0.17F + directUp * -0.09F,
            sample.first);
        require(ellisDot(packedRay, directRay) > 0.999999F,
                "GPU-style embedded screen ray disagrees with handle tetrad");
    }
    IntrinsicEllisCamera lookCamera;
    lookCamera.reset(settings);
    const float handleHalfLength = globalHandleHalfLength(settings);
    for (const auto& sample : std::array<std::pair<float, PortalVector>, 5>{
             std::pair{handleHalfLength * 0.72F,
                       PortalVector{0.31F, 0.42F, 0.85F}},
             std::pair{0.0F, PortalVector{0.31F, 0.42F, 0.85F}},
             std::pair{-handleHalfLength * 0.72F,
                       PortalVector{0.31F, 0.42F, 0.85F}},
             std::pair{handleHalfLength * 0.35F,
                       PortalVector{1.0e-5F, 1.0F, -2.0e-5F}},
             std::pair{-handleHalfLength * 0.35F,
                       PortalVector{-1.0e-5F, -1.0F, 2.0e-5F}}}) {
        const std::uint32_t source = sample.first >= 0.0F ? 0U : 1U;
        GlobalHandleObserverState observer = makeHandleObserver(
            source, sample.first, sample.second);
        lookCamera.setGlobalState(observer);
        const GlobalHandleObserverState before = lookCamera.globalState();
        const PortalVector beforeForward = before.forward;
        const PortalVector beforeUp = before.up;
        lookCamera.rotate(0.17F, -0.09F, 0.035F);
        const GlobalHandleObserverState& after = lookCamera.globalState();
        require(after.chart == 1U &&
                    std::abs(after.handleState.properDepth -
                             before.handleState.properDepth) < 1.0e-8F &&
                    portalLength(after.handleState.angularPosition -
                                 before.handleState.angularPosition) < 1.0e-8F &&
                    portalLength(after.position - before.position) < 1.0e-8F,
                "handle mouse look changed the observer position");
        require(portalDot(beforeForward, after.forward) < 0.995F &&
                    portalDot(beforeUp, after.up) < 0.9999F,
                "handle mouse yaw/pitch/roll did not rotate the local tetrad");
        require(ellisFrameHandedness(
                    after.handleFrame,
                    after.handleState.angularPosition) > 0.999F &&
                    lookCamera.handedness() > 0.999F,
                "handle mouse look lost tetrad orthonormality/handedness");

        const GlobalHandleObserverState rotated = after;
        lookCamera.rotate(0.0F, 0.0F, 0.0F);
        require(portalDot(rotated.forward,
                          lookCamera.globalState().forward) > 0.999999F &&
                    portalDot(rotated.up,
                              lookCamera.globalState().up) > 0.999999F,
                "zero mouse input changed the transported handle frame");

        const IntrinsicEllisGpuParameters packet = packGlobalCamera(
            lookCamera.globalState());
        const PortalVector packetForward{packet.angular[0],
                                         packet.angular[1],
                                         packet.angular[2]};
        const PortalVector packetUp{packet.forward[0], packet.forward[1],
                                    packet.forward[2]};
        require(packet.control[1] == 1.0F &&
                    portalDot(packetForward,
                              lookCamera.globalState().forward) > 0.999999F &&
                    portalDot(packetUp,
                              lookCamera.globalState().up) > 0.999999F,
                "CPU handle look and same-frame GPU center ray packet disagree");

        lookCamera.move(1.0F, 0.0F, 0.0F, 0.0125F, false, settings);
        const GlobalHandleObserverState& moved = lookCamera.globalState();
        const float movedForwardAlignment = portalDot(
            portalNormalize(moved.velocity), moved.forward);
        if (!moved.finite || lookCamera.handedness() <= 0.999F ||
            movedForwardAlignment <= 0.999F) {
            std::cerr << "look-move depth/chart/alignment/hand="
                      << sample.first << '/' << moved.chart << '/'
                      << movedForwardAlignment << '/'
                      << lookCamera.handedness() << '\n';
        }
        require(moved.finite && lookCamera.handedness() > 0.999F &&
                    movedForwardAlignment > 0.999F,
                "movement did not follow the looked handle-local forward axis");
    }

    for (std::uint32_t source = 0U; source < 2U; ++source) {
        const float startDepth = (source == 0U ? 1.0F : -1.0F) *
            handleHalfLength * 0.78F;
        lookCamera.setGlobalState(makeHandleObserver(
            source, startDepth,
            source == 0U ? PortalVector{0.28F, 0.35F, 0.89F}
                         : PortalVector{-0.41F, 0.91F, 0.06F}));
        const std::uint32_t crossingsBefore =
            lookCamera.globalState().crossings;
        bool crossedCenter = false;
        for (std::uint32_t step = 0U; step < 320U; ++step) {
            const float depthBefore =
                lookCamera.globalState().handleState.properDepth;
            lookCamera.rotate(source == 0U ? 0.00035F : -0.00035F,
                              0.00012F);
            lookCamera.move(1.0F, 0.0F, 0.0F, 0.01F, false, settings);
            const GlobalHandleObserverState& current =
                lookCamera.globalState();
            crossedCenter = crossedCenter ||
                (current.chart == 1U && depthBefore *
                    current.handleState.properDepth <= 0.0F);
            require(current.finite && lookCamera.handedness() > 0.998F,
                    "continuous look was overwritten/flipped during crossing");
            if (current.crossings != crossingsBefore) break;
        }
        require(crossedCenter &&
                    lookCamera.globalState().crossings ==
                        crossingsBefore + 1U,
                source == 0U
                    ? "continuous look did not survive A-to-B traversal"
                    : "continuous look did not survive B-to-A traversal");
    }

    // Manual-style short-tunnel stress at low, normal, and high frame rates.
    // Look is applied to the transported tetrad on every frame; movement
    // must cross once without tunneling, mirroring, or a stale GPU chart.
    for (const float deltaSeconds : {1.0F / 20.0F, 1.0F / 60.0F,
                                     1.0F / 240.0F}) {
        for (std::uint32_t source = 0U; source < 2U; ++source) {
            const float sign = source == 0U ? 1.0F : -1.0F;
            lookCamera.setGlobalState(makeHandleObserver(
                source, sign * handleHalfLength * 0.95F,
                source == 0U ? PortalVector{0.21F, 0.44F, 0.87F}
                             : PortalVector{-0.38F, 0.82F, 0.42F}));
            const std::uint32_t startCrossings =
                lookCamera.globalState().crossings;
            float elapsed = 0.0F;
            for (std::uint32_t frame = 0U; frame < 512U; ++frame) {
                const float frameScale = deltaSeconds * 60.0F;
                lookCamera.rotate(
                    (source == 0U ? 1.0F : -1.0F) * 0.00018F * frameScale,
                    0.00007F * frameScale, 0.00005F * frameScale);
                const IntrinsicEllisGpuParameters beforeMovePacket =
                    packGlobalCamera(lookCamera.globalState());
                require(beforeMovePacket.control[1] == 1.0F,
                        "short crossing packed a stale exterior chart");
                lookCamera.move(1.0F, 0.0F, 0.0F, deltaSeconds, false,
                                settings);
                elapsed += deltaSeconds;
                require(lookCamera.globalState().finite &&
                            lookCamera.handedness() > 0.999F,
                        "short crossing flipped/lost the camera tetrad");
                if (lookCamera.globalState().crossings != startCrossings) {
                    break;
                }
            }
            constexpr float maximumAcceptedTraversalSeconds = 1.50F;
            if (!(lookCamera.globalState().crossings == startCrossings + 1U &&
                  lookCamera.globalState().chart == 0U &&
                  elapsed < maximumAcceptedTraversalSeconds)) {
                std::cerr << "manual traversal dt/source elapsed/chart/cross="
                          << deltaSeconds << '/' << source << '/' << elapsed
                          << '/' << lookCamera.globalState().chart << '/'
                          << lookCamera.globalState().crossings << '\n';
            }
            require(lookCamera.globalState().crossings == startCrossings + 1U &&
                        lookCamera.globalState().chart == 0U &&
                        elapsed < maximumAcceptedTraversalSeconds,
                    "short-tunnel manual traversal did not complete once");
            const IntrinsicEllisGpuParameters afterPacket =
                packGlobalCamera(lookCamera.globalState());
            require(afterPacket.control[1] == 0.0F,
                    "short crossing GPU packet lagged one chart frame");
        }
    }

    // Same-exterior overlap pullback.  The exterior conformal chart and the
    // handle radial chart must carry the same areal-radius jet at the overlap;
    // a merely C1 match is visible as a circular curvature boundary.
    const auto handleCenters = globalHandleCenters(settings);
    const PortalVector overlapN = portalNormalize(handleCenters[0]);
    const float overlapRadius = globalHandleMouthRadius(settings);
    const float overlapU = globalHandleHalfLength(settings);
    const EllisRadialProfile overlapProfile = globalHandleRadialProfile(
        overlapU, settings);
    const float expectedAreal = globalHandleBoundaryConformal(settings) *
        overlapRadius;
    const float expectedFirst = overlapU / expectedAreal;
    const float expectedSecond = settings.throatRadius * settings.throatRadius /
        (expectedAreal * expectedAreal * expectedAreal);
    const float expectedPhiFirst = (expectedFirst - 1.0F) / overlapRadius;
    const float expectedPhiSecond =
        (globalHandleBoundaryConformal(settings) * expectedSecond -
         expectedPhiFirst) / overlapRadius;
    require(std::abs(overlapProfile.radius - expectedAreal) < 2.0e-6F &&
                std::abs(overlapProfile.firstDerivative - expectedFirst) <
                    2.0e-5F &&
                std::abs(overlapProfile.secondDerivative - expectedSecond) <
                    2.0e-4F,
            "handle/exterior overlap areal-radius jet is not C2");
    const PortalVector overlapPoint = handleCenters[0] +
        overlapN * overlapRadius;
    const GlobalHandleMetricSample overlapMetric =
        evaluateGlobalHandleMetric(overlapPoint, settings);
    require(overlapMetric.finite && overlapMetric.conformal > 0.0 &&
                portalLength(overlapMetric.logConformalGradient -
                             overlapN * expectedPhiFirst) < 2.0e-4F &&
                std::abs(overlapMetric.metric[0][0] *
                    overlapMetric.inverse[0][0] - 1.0) < 1.0e-10,
            "same-exterior overlap metric/inverse/Gamma is invalid");
    constexpr float overlapH = 5.0e-4F;
    const double phiMinus = globalHandleLogConformal(
        overlapPoint - overlapN * overlapH, settings).value;
    const double phiCenter = globalHandleLogConformal(
        overlapPoint, settings).value;
    const double phiPlus = globalHandleLogConformal(
        overlapPoint + overlapN * overlapH, settings).value;
    const double overlapFirst = (phiPlus - phiMinus) /
        (2.0 * overlapH);
    const double overlapSecond = (phiPlus - 2.0 * phiCenter + phiMinus) /
        (overlapH * overlapH);
    if (!(std::abs(overlapFirst - expectedPhiFirst) < 2.0e-3 &&
          std::abs(overlapSecond - expectedPhiSecond) < 5.0e-1)) {
        std::cerr << "overlap jets numeric/expected first=" << overlapFirst
                  << '/' << expectedPhiFirst << " second=" << overlapSecond
                  << '/' << expectedPhiSecond << '\n';
    }
    require(std::abs(overlapFirst - expectedPhiFirst) < 2.0e-3 &&
                std::abs(overlapSecond - expectedPhiSecond) < 5.0e-1,
            "same-exterior exterior metric retains a C1/C2 collar seam");

    IntrinsicEllisCamera camera;
    EllisState externalCamera{};
    externalCamera.properDepth = settings.contentExitProperDepth + 0.01F;
    externalCamera.angularPosition = {0.0F, 0.0F, 1.0F};
    const auto externalBasis = ellisAngularBasis(
        externalCamera.angularPosition);
    EllisFrame externalFrame{};
    externalFrame.forward = {0.0F, externalBasis[0]};
    externalFrame.right = {-1.0F, {}};
    externalFrame.up = {0.0F, externalBasis[1]};
    externalCamera.velocity = externalFrame.forward;
    camera.setState(externalCamera, externalFrame);
    const IntrinsicEllisContentPose poseBefore = intrinsicEllisContentPose(
        settings, camera.state(), camera.frame(), 1.0F);
    camera.move(1.0F, 0.0F, 0.0F, 0.1F, false, settings);
    const IntrinsicEllisContentPose poseAfter = intrinsicEllisContentPose(
        settings, camera.state(), camera.frame(), 1.0F);
    const float angularMotion = std::acos(std::clamp(portalDot(
        portalNormalize(externalCamera.angularPosition),
        portalNormalize(camera.state().angularPosition)), -1.0F, 1.0F));
    const float localTangentialDistance = 0.5F *
        (poseBefore.radialDistance + poseAfter.radialDistance) * angularMotion;
    require(std::abs(ellisLength(camera.velocity()) - 0.4F) < 2.0e-5F &&
                std::abs(localTangentialDistance - 0.04F) < 8.0e-4F &&
                portalDot(poseBefore.forward, poseAfter.forward) > 0.999F,
            "global free-fly lost tetrad proper speed across former overlap");

    EllisState insideCamera = externalCamera;
    insideCamera.properDepth = settings.contentExitProperDepth - 0.01F;
    camera.setState(insideCamera, externalFrame);
    const IntrinsicEllisContentPose insideBefore = intrinsicEllisContentPose(
        settings, camera.state(), camera.frame(), 1.0F);
    const PortalVector insideN = camera.state().angularPosition;
    camera.move(1.0F, 0.0F, 0.0F, 0.1F, false, settings);
    const IntrinsicEllisContentPose insideAfter = intrinsicEllisContentPose(
        settings, camera.state(), camera.frame(), 1.0F);
    const float insideAngle = std::acos(std::clamp(portalDot(
        portalNormalize(insideN), portalNormalize(camera.state().angularPosition)),
        -1.0F, 1.0F));
    const float insideDistance = 0.5F *
        (insideBefore.radialDistance + insideAfter.radialDistance) * insideAngle;
    require(std::abs(insideDistance - localTangentialDistance) < 9.0e-4F &&
                std::abs(ellisLength(camera.velocity()) - 0.4F) < 2.0e-5F,
            "proper tangential speed changes across the former region boundary");

    EllisFrame radialFrame{};
    radialFrame.forward = {-1.0F, {}};
    radialFrame.right = {0.0F, externalBasis[0]};
    radialFrame.up = {0.0F, externalBasis[1]};
    for (const float startL : {settings.contentExitProperDepth + 0.01F,
                               settings.contentExitProperDepth - 0.01F}) {
        EllisState radialState{};
        radialState.properDepth = startL;
        radialState.angularPosition = externalCamera.angularPosition;
        radialState.velocity = radialFrame.forward;
        camera.setState(radialState, radialFrame);
        camera.move(1.0F, 0.0F, 0.0F, 0.1F, false, settings);
        require(std::abs((startL - camera.state().properDepth) - 0.04F) <
                        2.0e-5F &&
                    std::abs(ellisLength(camera.velocity()) - 0.4F) < 2.0e-5F,
                "proper radial speed changes across the former region boundary");
    }

    const PortalVector arbitraryContent = portalNormalize(
        {0.31F, -0.27F, 0.91F});
    const PortalVector bIntrinsicN = portalNormalize(
        {0.24F, 0.87F, -0.43F});
    const EllisTangent bTangent = intrinsicEllisTangentFromContent(
        settings, false, bIntrinsicN, arbitraryContent);
    EllisState bState{};
    bState.properDepth = -settings.contentExitProperDepth;
    bState.angularPosition = bIntrinsicN;
    const PortalVector roundTrip = portalNormalize(
        intrinsicEllisTangentToContent(settings, bState, bTangent));
    require(portalDot(roundTrip, arbitraryContent) > 0.99999F,
            "negative-end chart overlap changed a content tangent");

    // The accepted interactive packet carries the observer's native signed-l
    // event/tetrad. It has no Euclidean mouth owner lane.
    EllisState nativePacketState{};
    nativePacketState.properDepth = 0.013F;
    nativePacketState.angularPosition = portalNormalize({0.2F, 0.3F, 0.9F});
    const auto nativeBasis = ellisAngularBasis(
        nativePacketState.angularPosition);
    EllisFrame nativePacketFrame{};
    nativePacketFrame.forward = {-0.8F, nativeBasis[0] * 0.6F};
    nativePacketFrame.right = {0.6F, nativeBasis[0] * 0.8F};
    nativePacketFrame.up = {0.0F, nativeBasis[1]};
    settings.globalNativeEllisPath = true;
    const IntrinsicEllisGpuParameters nativePacket =
        intrinsicEllisGpuParameters(
            settings, nativePacketState, nativePacketFrame);
    require(nativePacket.control[0] < -0.5F &&
                std::abs(nativePacket.camera[0] -
                         nativePacketState.properDepth) < 1.0e-7F &&
                portalLength(PortalVector{nativePacket.angular[0],
                                          nativePacket.angular[1],
                                          nativePacket.angular[2]} -
                             nativePacketState.angularPosition) < 1.0e-6F,
            "native Ellis GPU packet reverted to a mouth-owner pose");

    // Rejected shared-exterior handle regression remains as a historical
    // differential, not the interactive renderer.
    settings.globalNativeEllisPath = false;
    settings.globalHandleTailScale = 0.85F;
    settings.globalHandleMetricStrength = 0.72F;

    // HANDLE PROPER LENGTH is an intrinsic metric distance, independent of
    // the exterior mouth radius.  The presets must change that distance
    // monotonically while retaining a positive finite C2 radial profile.
    const float referenceHandleDiameters =
        globalHandleReferenceLengthDiameters(settings);
    float previousHandleLength = 0.0F;
    for (const float preset :
         {0.32F, 0.75F, 2.00F, referenceHandleDiameters}) {
        IntrinsicEllisSettings lengthSettings = settings;
        lengthSettings.globalHandleLengthDiameters = preset;
        const float half = globalHandleHalfLength(lengthSettings);
        const float total = 2.0F * half;
        require(std::abs(total /
                    (2.0F * lengthSettings.throatRadius) - preset) < 2.0e-5F &&
                    total > previousHandleLength,
                "handle proper-length presets are not intrinsic/monotone");
        previousHandleLength = total;
        float previousRadius = 0.0F;
        for (std::uint32_t sample = 0U; sample <= 2048U; ++sample) {
            const float u = half * static_cast<float>(sample) / 2048.0F;
            const EllisRadialProfile profile = globalHandleRadialProfile(
                u, lengthSettings);
            require(std::isfinite(profile.radius) &&
                        std::isfinite(profile.firstDerivative) &&
                        std::isfinite(profile.secondDerivative) &&
                        profile.radius >= lengthSettings.throatRadius - 1.0e-6F &&
                        profile.radius + 1.0e-6F >= previousRadius,
                    "short-handle radial metric became nonfinite/nonmonotone");
            previousRadius = profile.radius;
        }
        const EllisRadialProfile center = globalHandleRadialProfile(
            0.0F, lengthSettings);
        const EllisRadialProfile positive = globalHandleRadialProfile(
            half, lengthSettings);
        const EllisRadialProfile negative = globalHandleRadialProfile(
            -half, lengthSettings);
        const float expectedBoundary = globalHandleBoundaryConformal(
            lengthSettings) * globalHandleMouthRadius(lengthSettings);
        const float expectedEndD = half / expectedBoundary;
        const float expectedEndDD = lengthSettings.throatRadius *
            lengthSettings.throatRadius /
            (expectedBoundary * expectedBoundary * expectedBoundary);
        require(std::abs(center.radius - lengthSettings.throatRadius) < 1.0e-6F &&
                    std::abs(center.firstDerivative) < 1.0e-6F &&
                    std::abs(positive.radius - expectedBoundary) < 2.0e-5F &&
                    std::abs(negative.radius - expectedBoundary) < 2.0e-5F &&
                    std::abs(positive.firstDerivative - expectedEndD) < 2.0e-4F &&
                    std::abs(negative.firstDerivative + expectedEndD) < 2.0e-4F &&
                    std::abs(positive.secondDerivative - expectedEndDD) < 3.0e-3F &&
                    std::abs(negative.secondDerivative - expectedEndDD) < 3.0e-3F,
                "short-handle collar lost its C2 boundary jet");

        GlobalHandleRayState radialLength{};
        radialLength.origin = globalHandleCenters(lengthSettings)[0] +
            PortalVector{globalHandleMouthRadius(lengthSettings), 0.0F, 0.0F};
        radialLength.direction = {-1.0F, 0.0F, 0.0F};
        radialLength.footprintU = radialLength.direction;
        radialLength.footprintV = {0.0F, 1.0F, 0.0F};
        globalHandleEnterInterior(radialLength, 0U, lengthSettings);
        const float eventMargin = std::max(total * 1.0e-4F, 2.0e-6F);
        const float beforeEvent = total - eventMargin;
        globalHandleIntegrateRay(
            radialLength, beforeEvent, lengthSettings, 4096U);
        require(radialLength.finite && radialLength.chart == 1U &&
                    radialLength.crossings == 0U,
                "short handle tunneled through a collar event");
        globalHandleIntegrateRay(
            radialLength, eventMargin + 2.0e-6F,
            lengthSettings, 4096U);
        require(radialLength.finite && radialLength.chart == 0U &&
                    radialLength.crossings == 1U &&
                    radialLength.lastMouth == 1U,
                "short handle did not event-split at configured proper length");
    }
    // All accepted-path invariants below exercise the Planet-safe physical
    // scale and 2.00-diameter default.
    // The 0.32 value is retained only in the explicit surface-like
    // differential below; it must never silently become the validated path.
    globalHandleApplyLensFootprintPreset(
        settings, GlobalHandleLensFootprintPreset::PlanetSafe);
    settings.globalHandleLengthDiameters = 2.00F;

    struct LensFootprintMetrics {
        double significantFraction{};
        double criticalFraction{};
        double primaryReplacementFraction{};
        double predictedCriticalArea{};
        std::uint64_t samples{};
    };
    const auto measureLensFootprint = [](const IntrinsicEllisSettings& sample,
                                         std::uint32_t source,
                                         float standoff) {
        LensFootprintMetrics metrics{};
        const auto centers = globalHandleCenters(sample);
        const PortalVector outward = portalNormalize(centers[source]);
        const PortalVector observer = centers[source] + outward * standoff;
        const PortalVector forward = outward * -1.0F;
        const PortalVector up = portalOrthonormalUp(
            forward, {0.0F, 1.0F, 0.0F});
        const PortalVector right = portalNormalize(portalCross(forward, up));
        std::uint64_t significant = 0U;
        std::uint64_t critical = 0U;
        std::uint64_t primary = 0U;
        std::uint64_t replaced = 0U;
        constexpr std::uint32_t width = 41U;
        constexpr std::uint32_t height = 23U;
        for (std::uint32_t y = 0U; y < height; ++y) {
            const float sy = (2.0F * (static_cast<float>(y) + 0.5F) /
                static_cast<float>(height) - 1.0F);
            for (std::uint32_t x = 0U; x < width; ++x) {
                const float sx = (2.0F * (static_cast<float>(x) + 0.5F) /
                    static_cast<float>(width) - 1.0F) * (16.0F / 9.0F);
                const PortalVector direction = portalNormalize(
                    forward + right * sx + up * sy);
                GlobalHandleRayState ray{};
                ray.origin = observer;
                ray.direction = direction;
                ray.footprintU = right;
                ray.footprintV = up;
                globalHandleIntegrateRay(ray, 4.0F, sample, 4096U);
                require(ray.finite,
                        "lens-footprint sweep produced a nonfinite ray");
                const float directionDelta = std::acos(std::clamp(
                    portalDot(direction, ray.direction), -1.0F, 1.0F));
                // "Significant" means a plainly visible >=5-degree mapping
                // change over the full affine query, not the deliberately
                // noncompact metric's sub-degree asymptotic tail.
                const bool changed = directionDelta > 0.0872664626F ||
                    ray.crossings != 0U;
                significant += changed ? 1U : 0U;
                critical += ray.crossings != 0U ? 1U : 0U;

                const float projection = -portalDot(observer, direction);
                const float discriminant = projection * projection -
                    portalDot(observer, observer) + 1.12F * 1.12F;
                const bool directPlanet = projection > 0.0F &&
                    discriminant >= 0.0F;
                primary += directPlanet ? 1U : 0U;
                replaced += directPlanet && ray.crossings != 0U ? 1U : 0U;
            }
        }
        metrics.samples = width * height;
        metrics.significantFraction = static_cast<double>(significant) /
            static_cast<double>(metrics.samples);
        metrics.criticalFraction = static_cast<double>(critical) /
            static_cast<double>(metrics.samples);
        metrics.primaryReplacementFraction = primary != 0U
            ? static_cast<double>(replaced) / static_cast<double>(primary)
            : 0.0;
        const float angularDiameter = globalHandleCriticalAngularDiameter(
            sample, observer, source);
        const double screenRadius = std::tan(angularDiameter * 0.5F);
        metrics.predictedCriticalArea = 3.141592653589793 *
            screenRadius * screenRadius / (4.0 * (16.0 / 9.0));
        return metrics;
    };

    struct PlanetSubjectMetrics {
        double primaryRetention{};
        double totalSilhouetteRetention{};
        double apparentAreaRatio{};
        double baselineReplacement{};
        double criticalToPlanetDiameter{};
        std::uint64_t baselinePixels{};
        std::uint64_t totalPlanetPixels{};
    };
    const auto segmentHitsSphere = [](PortalVector start, PortalVector end,
                                      float radius) noexcept {
        const PortalVector delta = end - start;
        const double aa = portalDot(delta, delta);
        const double bb = 2.0 * portalDot(start, delta);
        const double cc = portalDot(start, start) -
            static_cast<double>(radius) * radius;
        const double discriminant = bb * bb - 4.0 * aa * cc;
        if (!(aa > 1.0e-16) || discriminant < 0.0) return false;
        const double root = std::sqrt(discriminant);
        const double t0 = (-bb - root) / (2.0 * aa);
        const double t1 = (-bb + root) / (2.0 * aa);
        return (t0 >= 0.0 && t0 <= 1.0) ||
               (t1 >= 0.0 && t1 <= 1.0);
    };
    const auto rayHitsSphere = [](PortalVector origin, PortalVector direction,
                                  float radius) noexcept {
        const double projection = -portalDot(origin, direction);
        const double discriminant = projection * projection -
            portalDot(origin, origin) +
            static_cast<double>(radius) * radius;
        return projection > 0.0 && discriminant >= 0.0;
    };
    const auto measurePlanetSubject = [&](const IntrinsicEllisSettings& sample,
                                          std::uint32_t source,
                                          std::uint32_t pose) {
        constexpr float planetSilhouetteRadius = 1.12F;
        constexpr std::uint32_t width = 49U;
        constexpr std::uint32_t height = 27U;
        const auto centers = globalHandleCenters(sample);
        const PortalVector outward = portalNormalize(centers[source]);
        const PortalVector tangent = portalNormalize(portalCross(
            std::abs(outward.y) < 0.92F ? PortalVector{0.0F, 1.0F, 0.0F}
                                       : PortalVector{1.0F, 0.0F, 0.0F},
            outward));
        const PortalVector secondTangent = portalNormalize(portalCross(
            outward, tangent));
        const float resetDistance = globalHandleMouthRadius(sample) +
            std::max(globalHandleMouthRadius(sample) * 1.35F, 0.46F);
        PortalVector observer = centers[source] + outward * resetDistance;
        if (pose == 1U) observer = observer + tangent * 0.30F;
        if (pose == 2U) {
            observer = observer + tangent * 0.20F +
                secondTangent * 0.16F;
        }
        const PortalVector forward = pose == 0U
            ? outward * -1.0F : portalNormalize(observer * -1.0F);
        const PortalVector up = portalOrthonormalUp(
            forward, secondTangent);
        const PortalVector right = portalNormalize(portalCross(forward, up));
        std::uint64_t baseline = 0U;
        std::uint64_t primaryRetained = 0U;
        std::uint64_t totalRetained = 0U;
        std::uint64_t totalPlanet = 0U;
        for (std::uint32_t y = 0U; y < height; ++y) {
            const float sy = 2.0F * (static_cast<float>(y) + 0.5F) /
                static_cast<float>(height) - 1.0F;
            for (std::uint32_t x = 0U; x < width; ++x) {
                const float sx = (2.0F * (static_cast<float>(x) + 0.5F) /
                    static_cast<float>(width) - 1.0F) * (16.0F / 9.0F);
                const PortalVector direction = portalNormalize(
                    forward + right * sx + up * sy);
                const bool baselineHit = rayHitsSphere(
                    observer, direction, planetSilhouetteRadius);
                baseline += baselineHit ? 1U : 0U;

                GlobalHandleRayState ray{};
                ray.origin = observer;
                ray.direction = direction;
                ray.footprintU = right;
                ray.footprintV = up;
                bool planetHit = false;
                bool primaryHit = false;
                for (std::uint32_t segment = 0U;
                     segment < 240U && !planetHit; ++segment) {
                    const PortalVector previousOrigin = ray.origin;
                    const std::uint32_t previousChart = ray.chart;
                    const std::uint32_t previousCrossings = ray.crossings;
                    globalHandleIntegrateRay(ray, 0.020F, sample, 256U);
                    require(ray.finite,
                            "subject-relative lens sweep became nonfinite");
                    if (previousChart == 0U && ray.chart == 0U &&
                        previousCrossings == ray.crossings &&
                        segmentHitsSphere(previousOrigin, ray.origin,
                                          planetSilhouetteRadius)) {
                        planetHit = true;
                        primaryHit = ray.crossings == 0U;
                    }
                }
                totalPlanet += planetHit ? 1U : 0U;
                primaryRetained += baselineHit && primaryHit ? 1U : 0U;
                totalRetained += baselineHit && planetHit ? 1U : 0U;
            }
        }
        PlanetSubjectMetrics metrics{};
        metrics.baselinePixels = baseline;
        metrics.totalPlanetPixels = totalPlanet;
        metrics.primaryRetention = baseline != 0U
            ? static_cast<double>(primaryRetained) / baseline : 0.0;
        metrics.totalSilhouetteRetention = baseline != 0U
            ? static_cast<double>(totalRetained) / baseline : 0.0;
        metrics.apparentAreaRatio = baseline != 0U
            ? static_cast<double>(totalPlanet) / baseline : 0.0;
        metrics.baselineReplacement = 1.0 - metrics.primaryRetention;
        const float criticalDiameter = globalHandleCriticalAngularDiameter(
            sample, observer, source);
        const float observerDistance = portalLength(observer);
        const float planetDiameter = 2.0F * std::asin(std::clamp(
            planetSilhouetteRadius /
                std::max(observerDistance, planetSilhouetteRadius),
            0.0F, 1.0F));
        metrics.criticalToPlanetDiameter = criticalDiameter /
            std::max(planetDiameter, 1.0e-5F);
        return metrics;
    };

    IntrinsicEllisSettings planetSafeFootprint = settings;
    globalHandleApplyLensFootprintPreset(
        planetSafeFootprint, GlobalHandleLensFootprintPreset::PlanetSafe);
    require(std::abs(planetSafeFootprint.contentSphereRadius - 0.025F) <
                    1.0e-7F &&
                std::abs(planetSafeFootprint.throatRadius - 0.00875F) <
                    1.0e-7F &&
                std::abs(planetSafeFootprint.globalHandleTailScale - 0.009F) <
                    1.0e-7F &&
                std::abs(planetSafeFootprint.globalHandleLengthDiameters -
                         2.00F) < 1.0e-7F,
            "accepted planet-safe checkpoint parameters drifted");

    struct MouthExitClasses {
        std::uint64_t traversed{};
        std::uint64_t scattered{};
        std::uint64_t missed{};
    };
    const auto measureMouthExitClasses = [](
        const IntrinsicEllisSettings& sample, std::uint32_t source) {
        MouthExitClasses classes{};
        const auto centers = globalHandleCenters(sample);
        const PortalVector outward = portalNormalize(centers[source]);
        const PortalVector up = portalOrthonormalUp(
            outward * -1.0F, {0.0F, 1.0F, 0.0F});
        const PortalVector right = portalNormalize(portalCross(
            outward * -1.0F, up));
        const float mouthRadius = globalHandleMouthRadius(sample);
        const PortalVector observer = centers[source] + outward * 0.14F;
        for (std::int32_t y = -20; y <= 20; ++y) {
            for (std::int32_t x = -20; x <= 20; ++x) {
                const float nx = static_cast<float>(x) / 20.0F;
                const float ny = static_cast<float>(y) / 20.0F;
                if (nx * nx + ny * ny >= 0.96F * 0.96F) continue;
                const PortalVector target = centers[source] +
                    right * (nx * mouthRadius) + up * (ny * mouthRadius);
                GlobalHandleRayState ray{};
                ray.origin = observer;
                ray.direction = portalNormalize(target - observer);
                ray.footprintU = right;
                ray.footprintV = up;
                const PortalVector end = ray.origin + ray.direction * 0.30F;
                float fraction = 0.0F;
                if (!globalHandleSegmentSphereEntry(
                        ray.origin, end, centers[source], mouthRadius,
                        fraction)) {
                    ++classes.missed;
                    continue;
                }
                ray.origin = ray.origin + (end - ray.origin) * fraction;
                globalHandleEnterInterior(ray, source, sample);
                const EllisRadialProfile entryProfile =
                    globalHandleRadialProfile(
                        ray.handleState.properDepth, sample);
                const float impact = entryProfile.radius *
                    portalLength(ray.handleState.velocity.angular);
                if (impact < sample.throatRadius) {
                    ++classes.traversed;
                } else {
                    ++classes.scattered;
                }
            }
        }
        return classes;
    };
    IntrinsicEllisSettings surfaceLikeFootprint = planetSafeFootprint;
    surfaceLikeFootprint.globalHandleLengthDiameters = 0.32F;
    const MouthExitClasses acceptedExitClasses = measureMouthExitClasses(
        planetSafeFootprint, 0U);
    const MouthExitClasses surfaceExitClasses = measureMouthExitClasses(
        surfaceLikeFootprint, 0U);
    std::cout << "mouth entry accepted-2.00 traverse/scatter/missed="
              << acceptedExitClasses.traversed << '/'
              << acceptedExitClasses.scattered << '/'
              << acceptedExitClasses.missed
              << " surface-like-0.32=" << surfaceExitClasses.traversed << '/'
              << surfaceExitClasses.scattered << '/'
              << surfaceExitClasses.missed << '\n';
    require(acceptedExitClasses.missed == 0U &&
                surfaceExitClasses.missed == 0U &&
                acceptedExitClasses.scattered >
                    acceptedExitClasses.traversed &&
                acceptedExitClasses.scattered >
                    surfaceExitClasses.scattered * 2U,
            "2.00-diameter handle did not restore the exterior scattering annulus");
    // Recreate the exact rejected Compact configuration. Its small critical
    // ring hid a broad, noncompact conformal field that redirected most of the
    // primary planet; total-screen crossing counts failed to measure that.
    IntrinsicEllisSettings rejectedCompactFootprint = settings;
    rejectedCompactFootprint.contentSphereRadius = 0.18F;
    rejectedCompactFootprint.throatRadius = 0.18F * kPortalGrThroatRatio;
    rejectedCompactFootprint.globalHandleTailScale = 0.225F;
    rejectedCompactFootprint.contentExitProperDepth = std::sqrt(
        rejectedCompactFootprint.contentSphereRadius *
            rejectedCompactFootprint.contentSphereRadius -
        rejectedCompactFootprint.throatRadius *
            rejectedCompactFootprint.throatRadius);
    IntrinsicEllisSettings dramaticFootprint = settings;
    globalHandleApplyLensFootprintPreset(
        dramaticFootprint, GlobalHandleLensFootprintPreset::Dramatic);
    double planetSafeSignificant = 0.0;
    double dramaticSignificant = 0.0;
    double planetSafeCritical = 0.0;
    double dramaticCritical = 0.0;
    double planetSafeReplacement = 0.0;
    double dramaticReplacement = 0.0;
    constexpr std::array<float, 3> footprintDistances{0.45F, 0.80F, 1.20F};
    for (std::uint32_t source = 0U; source < 2U; ++source) {
        for (const float distance : footprintDistances) {
            const LensFootprintMetrics planetSafe = measureLensFootprint(
                planetSafeFootprint, source, distance);
            const LensFootprintMetrics dramatic = measureLensFootprint(
                dramaticFootprint, source, distance);
            planetSafeSignificant += planetSafe.significantFraction;
            dramaticSignificant += dramatic.significantFraction;
            planetSafeCritical += planetSafe.criticalFraction;
            dramaticCritical += dramatic.criticalFraction;
            planetSafeReplacement += planetSafe.primaryReplacementFraction;
            dramaticReplacement += dramatic.primaryReplacementFraction;
            require(planetSafe.predictedCriticalArea <=
                        dramatic.predictedCriticalArea * 0.36 + 1.0e-8,
                    "planet-safe physical throat did not shrink critical area");
        }
    }
    constexpr double footprintSweeps = 6.0;
    planetSafeSignificant /= footprintSweeps;
    dramaticSignificant /= footprintSweeps;
    planetSafeCritical /= footprintSweeps;
    dramaticCritical /= footprintSweeps;
    planetSafeReplacement /= footprintSweeps;
    dramaticReplacement /= footprintSweeps;
    std::cout << "lens footprint planet-safe/dramatic significant="
              << planetSafeSignificant << '/' << dramaticSignificant
              << " critical=" << planetSafeCritical << '/' << dramaticCritical
              << " planet-replaced=" << planetSafeReplacement << '/'
              << dramaticReplacement << '\n';
    require(planetSafeSignificant < 0.10 &&
                planetSafeCritical < dramaticCritical * 0.25 &&
                planetSafeReplacement < 0.25,
            "planet-safe lens preset still dominates the planet/view sweep");

    for (std::uint32_t source = 0U; source < 2U; ++source) {
        for (std::uint32_t pose = 0U; pose < 3U; ++pose) {
            const PlanetSubjectMetrics safe = measurePlanetSubject(
                planetSafeFootprint, source, pose);
            const PlanetSubjectMetrics rejected = measurePlanetSubject(
                rejectedCompactFootprint, source, pose);
            std::cout << "planet subject source/pose=" << source << '/' << pose
                      << " safe primary/total/area/replaced/critical="
                      << safe.primaryRetention << '/'
                      << safe.totalSilhouetteRetention << '/'
                      << safe.apparentAreaRatio << '/'
                      << safe.baselineReplacement << '/'
                      << safe.criticalToPlanetDiameter
                      << " rejected-primary=" << rejected.primaryRetention
                      << '\n';
            require(safe.baselinePixels > 64U &&
                        safe.primaryRetention >= 0.80 &&
                        safe.totalSilhouetteRetention >= 0.80 &&
                        safe.baselineReplacement <= 0.20 &&
                        safe.criticalToPlanetDiameter <= 0.25,
                    "planet-safe preset does not retain the primary planet");
        }
    }

    // The accepted profile has exactly one areal-radius minimum and one
    // symmetric Ellis curvature pulse. A collar-local shoulder or sign change
    // here is the mathematical signature of the rejected "two wormholes in
    // one hole" appearance.
    {
        const float half = globalHandleHalfLength(settings);
        float previousRadius = std::numeric_limits<float>::infinity();
        float previousTangentialCurvature = 0.0F;
        std::uint32_t minima = 0U;
        for (std::uint32_t sample = 0U; sample <= 2048U; ++sample) {
            const float u = -half + 2.0F * half *
                static_cast<float>(sample) / 2048.0F;
            const EllisRadialProfile profile = globalHandleRadialProfile(
                u, settings);
            const float tangentialCurvature =
                (1.0F - profile.firstDerivative *
                    profile.firstDerivative) /
                (profile.radius * profile.radius);
            if (sample > 0U && profile.radius > previousRadius && u > 0.0F &&
                minima == 0U) ++minima;
            if (u > 0.0F && sample > 1024U) {
                require(tangentialCurvature <=
                            previousTangentialCurvature + 2.0e-4F,
                        "short handle has a secondary collar lens peak");
            }
            previousRadius = profile.radius;
            previousTangentialCurvature = tangentialCurvature;
            require(profile.secondDerivative > 0.0F,
                    "short handle radius developed a secondary shoulder");
        }
        require(minima == 1U,
                "short handle does not have exactly one optical waist");
    }

    // Inside the declared exact core, the same-exterior solver and native
    // Ellis reference must agree not only on R but on off-axis geodesic
    // bending and tetrad parallel transport (the camera/optical Jacobian
    // primitives). The engineered collar is deliberately excluded.
    {
        const PortalVector n = portalNormalize({0.31F, -0.27F, 0.91F});
        const auto basis = ellisAngularBasis(n);
        EllisState handleState{-0.008F, n,
            ellisNormalize({0.72F, basis[0] * 0.52F}, n)};
        EllisFrame handleFrame{};
        handleFrame.forward = handleState.velocity;
        handleFrame.right = ellisNormalize(
            {-0.52F, basis[0] * 0.72F}, n);
        handleFrame.up = {0.0F, basis[1]};
        EllisState nativeState = handleState;
        EllisFrame nativeFrame = handleFrame;
        constexpr float comparisonDistance = 0.018F;
        constexpr std::uint32_t comparisonSteps = 180U;
        for (std::uint32_t step = 0U; step < comparisonSteps; ++step) {
            globalHandleIntegrateInteriorStep(
                handleState, handleFrame,
                comparisonDistance / static_cast<float>(comparisonSteps),
                settings);
        }
        ellisIntegrateGeodesic(
            nativeState, nativeFrame, comparisonDistance,
            settings.throatRadius, 256U, true);
        const float positionDelta = std::abs(
            handleState.properDepth - nativeState.properDepth) +
            portalLength(handleState.angularPosition -
                         nativeState.angularPosition);
        const float directionDelta = std::abs(
            handleState.velocity.radial - nativeState.velocity.radial) +
            portalLength(handleState.velocity.angular -
                         nativeState.velocity.angular);
        const PortalVector handleForward = ellisTangentToEmbedded(
            handleFrame.forward, handleState.angularPosition);
        const PortalVector nativeForward = ellisTangentToEmbedded(
            nativeFrame.forward, nativeState.angularPosition);
        const PortalVector handleUp = ellisTangentToEmbedded(
            handleFrame.up, handleState.angularPosition);
        const PortalVector nativeUp = ellisTangentToEmbedded(
            nativeFrame.up, nativeState.angularPosition);
        if (!(positionDelta < 4.0e-5F && directionDelta < 5.0e-5F &&
              portalLength(handleForward - nativeForward) < 1.0e-3F &&
              portalLength(handleUp - nativeUp) < 8.0e-5F)) {
            std::cerr << "native differential position/direction/forward/up="
                      << positionDelta << '/' << directionDelta << '/'
                      << portalLength(handleForward - nativeForward) << '/'
                      << portalLength(handleUp - nativeUp) << '\n';
        }
        require(positionDelta < 4.0e-5F && directionDelta < 5.0e-5F &&
                    portalLength(handleForward - nativeForward) < 1.0e-3F &&
                    portalLength(handleUp - nativeUp) < 8.0e-5F,
                "exact handle core diverges from native Ellis ray/camera transport");
    }

    // Both collar maps must preserve 3-D orientation.  End B previously
    // reversed only the radial axis (det=-1), which mirrored the entire
    // screen after traversal even though the intrinsic tetrad stayed finite.
    const PortalVector orientationN = portalNormalize(
        {0.27F, -0.43F, 0.86F});
    const auto orientationBasis = ellisAngularBasis(orientationN);
    const float intrinsicOrientation = portalDot(
        portalCross(orientationN, orientationBasis[0]),
        orientationBasis[1]);
    for (std::uint32_t endpoint = 0U; endpoint < 2U; ++endpoint) {
        const PortalVector radial = globalHandleTangentToExterior(
            {1.0F, {}}, orientationN, endpoint, settings);
        const PortalVector tangent0 = globalHandleTangentToExterior(
            {0.0F, orientationBasis[0]}, orientationN, endpoint, settings);
        const PortalVector tangent1 = globalHandleTangentToExterior(
            {0.0F, orientationBasis[1]}, orientationN, endpoint, settings);
        const float mappedOrientation = portalDot(
            portalCross(radial, tangent0), tangent1);
        require(intrinsicOrientation * mappedOrientation > 0.999F,
                "handle collar transition mirrors the camera frame");
    }
    settings.globalPosition = {0.42F, -0.18F, 1.77F};
    settings.globalForward = portalNormalize({-0.22F, 0.08F, -0.97F});
    settings.globalUp = portalOrthonormalUp(
        settings.globalForward, {0.0F, 1.0F, 0.0F});
    settings.globalLastMouth = 1U;
    settings.globalAffineDistance = 2.75F;
    const IntrinsicEllisGpuParameters packedCrossing =
        intrinsicEllisGpuParameters(settings, {}, {});
    require(packedCrossing.control[0] < -1.5F &&
                packedCrossing.control[1] == 0.0F &&
                std::abs(packedCrossing.control[2] -
                         settings.throatRadius) < 1.0e-6F &&
                packedCrossing.endpointA[3] == 1.0F &&
                std::abs(packedCrossing.angular[3] -
                         settings.globalSpatialAaDistortionThreshold) <
                    1.0e-6F &&
                std::abs(packedCrossing.endBRotation[3] -
                         settings.globalSpatialAaMagnificationThreshold) <
                    1.0e-6F &&
                portalLength(PortalVector{
                    packedCrossing.camera[0], packedCrossing.camera[1],
                    packedCrossing.camera[2]} - settings.globalPosition) <
                    1.0e-7F,
            "same-exterior GPU packet lost the exterior chart pose");
    const std::array<std::array<float, 2>, 4> aaOffsets{{
        {-0.375F, -0.125F}, {0.125F, -0.375F},
        {0.375F, 0.125F}, {-0.125F, 0.375F}}};
    float aaMeanX = 0.0F;
    float aaMeanY = 0.0F;
    float aaMomentX = 0.0F;
    float aaMomentY = 0.0F;
    for (const auto& offset : aaOffsets) {
        aaMeanX += offset[0];
        aaMeanY += offset[1];
        aaMomentX += offset[0] * offset[0];
        aaMomentY += offset[1] * offset[1];
    }
    require(std::abs(aaMeanX) < 1.0e-7F &&
                std::abs(aaMeanY) < 1.0e-7F &&
                std::abs(aaMomentX - aaMomentY) < 1.0e-7F,
            "rotated four-sample AA pattern is biased or anisotropic");
    const PortalVector testNormal = portalNormalize({0.31F, 0.42F, 0.85F});
    const PortalVector mappedNormal = portalNormalize(
        globalHandleRotateAcross(testNormal * -1.0F, 0U, settings));
    const float mouth = globalHandleMouthRadius(settings);
    const auto metricA = evaluateGlobalHandleMetric(
        handleCenters[0] + testNormal * mouth, settings);
    const auto metricB = evaluateGlobalHandleMetric(
        handleCenters[1] + mappedNormal * mouth, settings);
    require(metricA.finite && metricB.finite &&
                std::abs(metricA.conformal - metricB.conformal) < 2.0e-6 &&
                std::abs(metricA.metric[0][0] - metricB.metric[0][0]) < 5.0e-6,
            "shared-handle pullback metric is not C0 at the two throats");
    for (const float epsilon : {2.0e-3F, 1.0e-3F, 5.0e-4F}) {
        const auto outsideA = evaluateGlobalHandleMetric(
            handleCenters[0] + testNormal * (mouth + epsilon), settings);
        const auto outsideB = evaluateGlobalHandleMetric(
            handleCenters[1] + mappedNormal * (mouth + epsilon), settings);
        require(outsideA.finite && outsideB.finite &&
                    std::abs(outsideA.conformal - outsideB.conformal) <
                        8.0e-4,
                "shared-handle metric/connection overlap lost A/B symmetry");
    }

    const auto exerciseMouth = [&](std::uint32_t source) {
        GlobalHandleObserverState body = globalHandleResetObserver(settings, source);
        GlobalHandleRayState optical{};
        optical.origin = body.position;
        optical.direction = body.forward;
        optical.footprintU = body.forward;
        optical.footprintV = body.up;
        constexpr float kAtlasTraversalDistance = 2.75F;
        globalHandleIntegrateRay(
            optical, kAtlasTraversalDistance, settings, 2048U);
        const GlobalHandleObserverState beforeBody = body;
        body.properSpeed = 1.0F;
        globalHandleAdvanceObserver(
            body, body.forward, kAtlasTraversalDistance, settings);
        require(optical.finite && body.finite && optical.crossings == 1U &&
                    body.crossings == 1U,
                source == 0U ? "Home->A center did not traverse"
                             : "alternate reset->B center did not traverse");
        require(portalLength(optical.origin - body.position) < 2.0e-4F &&
                    portalDot(optical.direction,
                              portalNormalize(body.velocity)) > 0.9999F,
                "CPU body and optical ray disagree on a mouth event");
        require(portalDot(portalCross(portalNormalize(portalCross(
                    body.forward, body.up)), body.forward), body.up) > 0.999F,
                "shared-handle traversal changed frame handedness");

        GlobalHandleRayState reverse{};
        reverse.origin = optical.origin;
        reverse.direction = optical.direction * -1.0F;
        reverse.footprintU = optical.footprintU;
        reverse.footprintV = optical.footprintV;
        globalHandleIntegrateRay(reverse, optical.affineDistance,
                                 settings, 2048U);
        std::cout << "same-exterior reverse source=" << source
                  << " finite=" << reverse.finite
                  << " crossings=" << reverse.crossings
                  << " delta="
                  << portalLength(reverse.origin - beforeBody.position)
                  << " chart=" << reverse.chart << '\n';
        require(reverse.finite && reverse.crossings == 1U &&
                    portalLength(reverse.origin - beforeBody.position) < 3.5e-2F,
                "shared-handle A/B path failed practical time reversal");
    };
    exerciseMouth(0U);
    exerciseMouth(1U);

    // Diagnostic replay of the user-recorded forward crossing.  Keep this
    // bundle deterministic so a discontinuity in the camera event or its
    // corresponding optical start can be promoted to an acceptance gate.
    for (std::uint32_t source = 0U; source < 2U; ++source) {
        GlobalHandleObserverState replay = globalHandleResetObserver(
            settings, source);
        replay.properSpeed = settings.freeFlySpeed;
        std::array<GlobalHandleRayState, 63> previousBundle{};
        bool havePreviousBundle = false;
        float maximumCenterDelta = 0.0F;
        float maximumBundleDelta = 0.0F;
        float maximumCenterOriginDelta = 0.0F;
        float maximumJacobianDelta = 0.0F;
        float previousJacobian = 0.0F;
        std::uint32_t largeRayFamilyChanges = 0U;
        std::size_t largestRayFamilyIndex = 0U;
        std::uint32_t crossingFrame = 0xffffffffU;
        bool enteredHandle = false;
        std::uint32_t chartTransitions = 0U;
        // Four deterministic subframes per displayed 60 Hz frame distinguish
        // an atlas seam from the genuine rim separatrix without weakening the
        // physical ray-family test.
        for (std::uint32_t frameIndex = 0U; frameIndex < 720U; ++frameIndex) {
            const std::uint32_t crossingsBefore = replay.crossings;
            const std::uint32_t chartBefore = replay.chart;
            const float affineBefore = replay.affineDistance;
            globalHandleAdvanceObserver(
                replay, replay.forward,
                settings.freeFlySpeed * settings.sprintMultiplier / 240.0F,
                settings);
            require(replay.finite &&
                        replay.affineDistance > affineBefore &&
                        portalDot(portalCross(portalNormalize(portalCross(
                            replay.forward, replay.up)), replay.forward),
                            replay.up) > 0.999F,
                    "video-style camera lost affine/frame continuity");
            const bool chartTransition = replay.chart != chartBefore;
            if (chartTransition) {
                ++chartTransitions;
                enteredHandle = enteredHandle || replay.chart == 1U;
            }
            if (replay.crossings != crossingsBefore) {
                crossingFrame = frameIndex;
                const IntrinsicEllisGpuParameters crossingPacket =
                    intrinsicEllisGpuParameters(
                        [&] {
                            IntrinsicEllisSettings packed = settings;
                            packed.globalPosition = replay.position;
                            packed.globalForward = replay.forward;
                            packed.globalUp = replay.up;
                            packed.globalLastMouth = replay.lastMouth;
                            packed.globalAffineDistance = replay.affineDistance;
                            packed.globalHandleChart = replay.chart;
                            packed.globalHandleU = replay.handleState.properDepth;
                            packed.globalHandleN = replay.handleState.angularPosition;
                            packed.globalHandleForwardLocal = ellisTangentToEmbedded(
                                replay.handleFrame.forward,
                                replay.handleState.angularPosition);
                            packed.globalHandleUpLocal = ellisTangentToEmbedded(
                                replay.handleFrame.up,
                                replay.handleState.angularPosition);
                            return packed;
                        }(), {}, {});
                require(crossingPacket.control[0] < -1.5F &&
                            crossingPacket.control[1] == 0.0F,
                        "first post-handle GPU packet lost exterior atlas state");
            }
            const PortalVector right = portalNormalize(portalCross(
                replay.forward, replay.up), {1.0F, 0.0F, 0.0F});
            std::array<GlobalHandleRayState, 63> bundle{};
            for (std::uint32_t y = 0U; y < 7U; ++y) {
                for (std::uint32_t x = 0U; x < 9U; ++x) {
                    const std::size_t index = y * 9U + x;
                    const float screenX =
                        (static_cast<float>(x) - 4.0F) * 0.035F;
                    const float screenY =
                        (static_cast<float>(y) - 3.0F) * 0.035F;
                    GlobalHandleRayState ray{};
                    ray.origin = replay.position;
                    ray.direction = portalNormalize(
                        replay.forward + right * screenX +
                        replay.up * screenY);
                    ray.footprintU = replay.forward;
                    ray.footprintV = replay.up;
                    ray.lastMouth = replay.lastMouth;
                    ray.chart = replay.chart;
                    ray.handleState = replay.handleState;
                    ray.handleFrame = replay.handleFrame;
                    if (replay.chart == 1U) {
                        const float forwardAmount = portalDot(
                            ray.direction, replay.forward);
                        const float rightAmount = portalDot(
                            ray.direction, right);
                        const float upAmount = portalDot(
                            ray.direction, replay.up);
                        ray.handleState.velocity = ellisNormalize(
                            replay.handleFrame.forward * forwardAmount +
                                replay.handleFrame.right * rightAmount +
                                replay.handleFrame.up * upAmount,
                            replay.handleState.angularPosition,
                            replay.handleState.velocity);
                    }
                    globalHandleIntegrateRay(ray, 3.5F, settings, 4096U);
                    if (!ray.finite) {
                        std::cerr << "video bundle nonfinite source/frame/pixel/chart/u="
                                  << source << '/' << frameIndex << '/' << x
                                  << ',' << y << '/' << replay.chart << '/'
                                  << replay.handleState.properDepth
                                  << " ray-chart/u/steps/affine=" << ray.chart
                                  << '/' << ray.handleState.properDepth << '/'
                                  << ray.steps << '/' << ray.affineDistance
                                  << '\n';
                    }
                    require(ray.finite,
                            "video-style crossing bundle became nonfinite");
                    bundle[index] = ray;
                    if (havePreviousBundle && chartTransition) {
                        const float cosine = std::clamp(portalDot(
                            previousBundle[index].direction,
                            ray.direction), -1.0F, 1.0F);
                        const float delta = std::acos(cosine);
                        maximumBundleDelta = std::max(
                            maximumBundleDelta, delta);
                        if (delta > 0.80F) {
                            ++largeRayFamilyChanges;
                        }
                        if (delta >= maximumBundleDelta) {
                            largestRayFamilyIndex = index;
                        }
                        if (index == 31U) {
                            maximumCenterDelta = std::max(
                                maximumCenterDelta, delta);
                        }
                    }
                }
            }
            const float jacobian = std::acos(std::clamp(portalDot(
                bundle[30U].direction, bundle[32U].direction),
                -1.0F, 1.0F)) / 0.07F;
            if (havePreviousBundle && chartTransition) {
                maximumCenterOriginDelta = std::max(
                    maximumCenterOriginDelta,
                    portalLength(bundle[31U].origin -
                                 previousBundle[31U].origin));
                maximumJacobianDelta = std::max(
                    maximumJacobianDelta,
                    std::abs(jacobian - previousJacobian));
            }
            previousBundle = bundle;
            previousJacobian = jacobian;
            havePreviousBundle = true;
        }
        std::cout << "crossing replay source=" << source
                  << " frame=" << crossingFrame
                  << " center/bundle=" << maximumCenterDelta << '/'
                  << maximumBundleDelta
                  << " origin/jacobian=" << maximumCenterOriginDelta << '/'
                  << maximumJacobianDelta
                  << " localized-family=" << largeRayFamilyChanges << '/'
                  << largestRayFamilyIndex << '\n';
        require(crossingFrame != 0xffffffffU && enteredHandle &&
                    chartTransitions >= 2U &&
                    maximumCenterDelta < 0.012F &&
                    // One off-axis sample may cross the genuine handle-rim
                    // separatrix during a full 60 Hz movement step.  Center
                    // and Jacobian continuity remain the chart seam gates;
                    // the image gate coverage-samples that physical family.
                    // A very short, strongly flared throat has a genuine
                    // narrow separatrix: individual off-axis samples may
                    // change ray family.  It is not a camera flip when the
                    // center, camera tetrad, and neighboring majority remain
                    // continuous. The restored 2.00-diameter scattering
                    // annulus can change the finite-difference Jacobian by up
                    // to two radians per screen unit without changing any
                    // sampled ray family. Keep any actual family event
                    // localized to at most one 4-sample pixel footprint.
                    largeRayFamilyChanges <= 4U &&
                    maximumCenterOriginDelta < 0.02F &&
                    maximumJacobianDelta < 2.00F,
                "video-style off-axis perspective jumped at a chart crossing");
    }

    // Dense exterior/handle overlap limit. The two starts approach the same
    // physical overlap point from their respective charts; after identical
    // affine work their mapped origins, directions and transported footprint
    // must converge. No A/B exterior scene is exchanged at this event.
    for (std::uint32_t source = 0U; source < 2U; ++source) {
        const PortalVector sourceN = portalNormalize(handleCenters[source]);
        const PortalVector reference = std::abs(sourceN.y) < 0.92F
            ? PortalVector{0.0F, 1.0F, 0.0F}
            : PortalVector{1.0F, 0.0F, 0.0F};
        const PortalVector beforeForward = sourceN * -1.0F;
        const PortalVector beforeUp = portalOrthonormalUp(
            beforeForward, reference);
        const PortalVector beforeRight = portalNormalize(portalCross(
            beforeForward, beforeUp));
        for (const float epsilon : {
                 mouth * 2.0e-3F, mouth * 2.0e-4F,
                 std::nextafter(mouth,
                     std::numeric_limits<float>::infinity()) - mouth}) {
            float maximumOriginDelta = 0.0F;
            float maximumDirectionDelta = 0.0F;
            float maximumFootprintDelta = 0.0F;
            std::uint32_t maximumX = 0U;
            std::uint32_t maximumY = 0U;
            std::uint32_t maximumBeforeCrossings = 0U;
            std::uint32_t maximumAfterCrossings = 0U;
            for (std::uint32_t y = 0U; y < 7U; ++y) {
                for (std::uint32_t x = 0U; x < 9U; ++x) {
                    const float sx = (static_cast<float>(x) - 4.0F) * 0.035F;
                    const float sy = (static_cast<float>(y) - 3.0F) * 0.035F;
                    GlobalHandleRayState before{};
                    before.origin = handleCenters[source] + sourceN *
                        (mouth + epsilon);
                    before.direction = portalNormalize(
                        beforeForward + beforeRight * sx + beforeUp * sy);
                    before.footprintU = beforeForward;
                    before.footprintV = beforeUp;
                    before.lastMouth = 2U;

                    GlobalHandleRayState after{};
                    after.origin = handleCenters[source] + sourceN * mouth;
                    after.direction = before.direction;
                    after.footprintU = beforeForward;
                    after.footprintV = beforeUp;
                    globalHandleEnterInterior(after, source, settings);
                    const float properInset =
                        globalHandleBoundaryConformal(settings) * epsilon;
                    after.handleState.properDepth = source == 0U
                        ? globalHandleHalfLength(settings) - properInset
                        : -globalHandleHalfLength(settings) + properInset;

                    globalHandleIntegrateRay(before, 3.5F, settings, 4096U);
                    globalHandleIntegrateRay(after, 3.5F, settings, 4096U);
                    require(before.finite && after.finite,
                            "dense crossing subframe ray became nonfinite");
                    const float originDelta = portalLength(
                        before.origin - after.origin);
                    if (originDelta > maximumOriginDelta) {
                        maximumOriginDelta = originDelta;
                        maximumX = x;
                        maximumY = y;
                        maximumBeforeCrossings = before.crossings;
                        maximumAfterCrossings = after.crossings;
                    }
                    maximumDirectionDelta = std::max(maximumDirectionDelta,
                        std::acos(std::clamp(portalDot(
                            before.direction, after.direction), -1.0F, 1.0F)));
                    maximumFootprintDelta = std::max(maximumFootprintDelta,
                        std::acos(std::clamp(portalDot(
                            portalNormalize(before.footprintV),
                            portalNormalize(after.footprintV)), -1.0F, 1.0F)));
                }
            }
            const bool ulpLimit = epsilon < mouth * 1.0e-5F;
            if (ulpLimit &&
                !(maximumOriginDelta < 0.006F &&
                  maximumDirectionDelta < 0.008F &&
                  maximumFootprintDelta < 0.008F)) {
                std::cerr << "dense crossing source/epsilon=" << source << '/'
                          << epsilon << " origin/direction/footprint="
                          << maximumOriginDelta << '/' << maximumDirectionDelta
                          << '/' << maximumFootprintDelta << " pixel="
                          << maximumX << ',' << maximumY << " crossings="
                          << maximumBeforeCrossings << '/'
                          << maximumAfterCrossings << '\n';
            }
            if (ulpLimit) {
                require(maximumOriginDelta < 0.006F &&
                            maximumDirectionDelta < 0.008F &&
                            maximumFootprintDelta < 0.008F,
                        "dense +/-ULP overlap changed mapped perspective");
            }
        }
    }
    for (const std::array<float, 3> rotation : {
             std::array<float, 3>{0.37F, -0.21F, 0.14F},
             std::array<float, 3>{-1.12F, 0.43F, -0.31F}}) {
        IntrinsicEllisSettings rotated = settings;
        rotated.endBYaw = rotation[0];
        rotated.endBPitch = rotation[1];
        rotated.endBRoll = rotation[2];
        for (std::uint32_t source = 0U; source < 2U; ++source) {
            GlobalHandleObserverState body = globalHandleResetObserver(
                rotated, source);
            body.properSpeed = 1.0F;
            globalHandleAdvanceObserver(body, body.forward, 2.75F, rotated);
            const PortalVector right = portalNormalize(portalCross(
                body.forward, body.up));
            require(body.finite && body.crossings == 1U &&
                        portalDot(portalCross(right, body.forward),
                                  body.up) > 0.999F,
                    "End-B rotation sweep mirrored or blocked a mouth");
        }
    }

    // Exact tangency belongs to the exterior half-open owner; a one-ULP
    // inward perturbation enters and the outward perturbation misses.  No
    // finite angular tolerance is allowed to steer the physical trajectory.
    // Endpoint A has exact x=0 for the default azimuth, so construct the ULP
    // tangency test on that component rather than burying it in an unrelated
    // center+offset rounding error.
    const PortalVector tangentStart = handleCenters[0] +
        PortalVector{mouth, -mouth * 2.0F, 0.0F};
    const PortalVector tangentEnd = tangentStart +
        PortalVector{0.0F, mouth * 4.0F, 0.0F};
    float tangentFraction = 0.0F;
    require(!globalHandleSegmentSphereEntry(
                tangentStart, tangentEnd, handleCenters[0], mouth,
                tangentFraction),
            "exact tangent was incorrectly owned by a mouth");
    float inwardX = mouth;
    float outwardX = mouth;
    for (std::uint32_t ulp = 1U; ulp <= 4U; ++ulp) {
        inwardX = std::nextafter(inwardX, 0.0F);
        outwardX = std::nextafter(
            outwardX, std::numeric_limits<float>::infinity());
        require(globalHandleSegmentSphereEntry(
                    handleCenters[0] +
                        PortalVector{inwardX, -mouth * 2.0F, 0.0F},
                    handleCenters[0] +
                        PortalVector{inwardX, mouth * 2.0F, 0.0F},
                    handleCenters[0], mouth, tangentFraction) &&
                !globalHandleSegmentSphereEntry(
                    handleCenters[0] +
                        PortalVector{outwardX, -mouth * 2.0F, 0.0F},
                    handleCenters[0] +
                        PortalVector{outwardX, mouth * 2.0F, 0.0F},
                    handleCenters[0], mouth, tangentFraction),
                "ULP half-open mouth ownership is biased or non-deterministic");
    }

    // One shared-exterior observer can see and enter both localized mouths;
    // they are not one-mouth-per-asymptotic-end scene attachments.  This pose
    // sits above the planet so the two exact center rays are unobstructed and
    // fall inside one wide-screen view.
    const PortalVector sharedObserver{0.0F, 3.2F, 0.0F};
    std::array<PortalVector, 2> sharedMouthDirections{};
    for (std::uint32_t mouthIndex = 0U; mouthIndex < 2U; ++mouthIndex) {
        sharedMouthDirections[mouthIndex] = portalNormalize(
            handleCenters[mouthIndex] - sharedObserver);
        GlobalHandleRayState centerRay{};
        centerRay.origin = sharedObserver;
        centerRay.direction = sharedMouthDirections[mouthIndex];
        centerRay.footprintU = centerRay.direction;
        centerRay.footprintV = portalOrthonormalUp(
            centerRay.direction, {0.0F, 0.0F, 1.0F});
        globalHandleIntegrateRay(centerRay, 5.0F, settings, 4096U);
        if (!(centerRay.finite && centerRay.crossings == 1U)) {
            std::cerr << "shared center mouth=" << mouthIndex
                      << " finite/cross/chart/steps=" << centerRay.finite
                      << '/' << centerRay.crossings << '/' << centerRay.chart
                      << '/' << centerRay.steps << " origin="
                      << centerRay.origin.x << ',' << centerRay.origin.y << ','
                      << centerRay.origin.z << '\n';
        }
        require(centerRay.finite && centerRay.crossings == 1U,
                "shared exterior cannot optically enter both mouth centers");
    }
    require(std::acos(std::clamp(portalDot(
                sharedMouthDirections[0], sharedMouthDirections[1]),
                -1.0F, 1.0F)) < 2.0F,
            "representative shared observer cannot frame both mouths");

    // The production GPU policy uses the embedded RK4/midpoint step while
    // the small-step midpoint integrator remains an independent correctness
    // reference. Compare the complete event-split path, not merely the local
    // ODE step, across both mouths and the near-critical image family.
    for (std::uint32_t source = 0U; source < 2U; ++source) {
        const GlobalHandleObserverState observer =
            globalHandleResetObserver(settings, source);
        const PortalVector right = portalNormalize(
            portalCross(observer.forward, observer.up));
        float maximumOriginError = 0.0F;
        float maximumDirectionError = 0.0F;
        std::uint32_t maximumOriginSample = 0U;
        std::uint32_t maximumDirectionSample = 0U;
        std::uint32_t localizedCriticalDifferentials = 0U;
        for (std::uint32_t sample = 0U; sample <= 256U; ++sample) {
            const float screenX = std::lerp(
                -0.70F, 0.70F, static_cast<float>(sample) / 256.0F);
            const PortalVector initialDirection = portalNormalize(
                observer.forward + right * screenX);
            GlobalHandleRayState optimized{};
            optimized.origin = observer.position;
            optimized.direction = initialDirection;
            optimized.footprintU = observer.forward;
            optimized.footprintV = observer.up;
            GlobalHandleRayState reference = optimized;
            globalHandleIntegrateRay(
                optimized, 3.5F, settings, 4096U, true);
            globalHandleIntegrateRay(
                reference, 3.5F, settings, 8192U, true, true);
            require(optimized.finite && reference.finite,
                    "optimized/reference global path became nonfinite");
            const PortalVector startRelative = observer.position -
                handleCenters[source];
            const float startProjected = portalDot(
                startRelative, initialDirection);
            const float startClosest = std::sqrt(std::max(
                portalDot(startRelative, startRelative) -
                    startProjected * startProjected, 0.0F));
            const bool certifiedCriticalNeighborhood =
                std::abs(startClosest - mouth) < 0.012F;
            if (optimized.crossings != reference.crossings ||
                optimized.lastMouth != reference.lastMouth) {
                std::cerr << "ownership differential source/sample/screen="
                          << source << '/' << sample << '/' << screenX
                          << " optimized=" << optimized.crossings << '/'
                          << optimized.lastMouth << " reference="
                          << reference.crossings << '/'
                          << reference.lastMouth << " closest="
                          << startClosest << '\n';
            }
            require((optimized.crossings == reference.crossings &&
                         optimized.lastMouth == reference.lastMouth) ||
                        certifiedCriticalNeighborhood,
                    "optimized solver changed throat event ownership");
            if (optimized.crossings != reference.crossings ||
                optimized.lastMouth != reference.lastMouth) continue;
            const float originError = portalLength(
                optimized.origin - reference.origin);
            const float directionError = std::acos(std::clamp(portalDot(
                optimized.direction, reference.direction), -1.0F, 1.0F));
            if (originError >= maximumOriginError) {
                maximumOriginError = originError;
                maximumOriginSample = sample;
            }
            if (directionError >= maximumDirectionError) {
                maximumDirectionError = directionError;
                maximumDirectionSample = sample;
            }
            if (originError >= 0.012F || directionError >= 0.012F) {
                ++localizedCriticalDifferentials;
            }
        }
        if (!(maximumOriginError < 0.012F &&
              maximumDirectionError < 0.012F)) {
            std::cerr << "optimized differential source=" << source
                      << " origin/direction=" << maximumOriginError << '/'
                      << maximumDirectionError << " samples="
                      << maximumOriginSample << '/'
                      << maximumDirectionSample << " localized="
                      << localizedCriticalDifferentials << '\n';
        }
        require(maximumOriginError < 0.050F &&
                    maximumDirectionError < 0.020F &&
                    localizedCriticalDifferentials <= 4U,
                "optimized global solver diverges from small-step reference");
    }

    // The 1.30R planet-centred shell is a content acceleration/query bound,
    // never a geodesic endpoint. Recreate the screenshot pose and straddle
    // its projected tangent by ULP and subpixel offsets. Every ray must retain
    // the same full affine policy and practical C1 endpoint/frame, agreeing
    // with the independent small-step oracle on both mouth views.
    constexpr float contentQueryRadius = 1.30005F;
    for (std::uint32_t source = 0U; source < 2U; ++source) {
        const PortalVector outward = portalNormalize(handleCenters[source]);
        const PortalVector referenceUp = std::abs(outward.y) < 0.92F
            ? PortalVector{0.0F, 1.0F, 0.0F}
            : PortalVector{1.0F, 0.0F, 0.0F};
        const PortalVector right = portalNormalize(
            portalCross(referenceUp, outward));
        const PortalVector observer = handleCenters[source] + outward *
            (settings.contentSphereRadius * 2.40F);
        const PortalVector forward = portalNormalize(observer * -1.0F);
        const float distance = portalLength(observer);
        require(distance > contentQueryRadius,
                "content-shell tangent observer began inside query bound");
        const float tangentScreen = contentQueryRadius / std::sqrt(
            distance * distance - contentQueryRadius * contentQueryRadius);
        const std::array<float, 5> shellSamples{
            tangentScreen - 2.0F / 900.0F,
            std::nextafter(tangentScreen, -std::numeric_limits<float>::infinity()),
            tangentScreen,
            std::nextafter(tangentScreen, std::numeric_limits<float>::infinity()),
            tangentScreen + 2.0F / 900.0F};
        GlobalHandleRayState previousShell{};
        bool havePreviousShell = false;
        for (float screenX : shellSamples) {
            GlobalHandleRayState optimized{};
            optimized.origin = observer;
            optimized.direction = portalNormalize(forward + right * screenX);
            optimized.footprintU = right;
            optimized.footprintV = referenceUp;
            GlobalHandleRayState oracle = optimized;
            globalHandleIntegrateRay(
                optimized, 3.5F, settings, 2048U, true);
            globalHandleIntegrateRay(
                oracle, 3.5F, settings, 8192U, true, true);
            if (!optimized.finite || !oracle.finite ||
                std::abs(optimized.affineDistance - 3.5F) >= 1.0e-4F ||
                std::abs(oracle.affineDistance - 3.5F) >= 1.0e-4F) {
                std::cerr << "content shell state source/screen=" << source
                          << '/' << screenX << " finite="
                          << optimized.finite << '/' << oracle.finite
                          << " affine=" << optimized.affineDistance << '/'
                          << oracle.affineDistance << " chart="
                          << optimized.chart << '/' << oracle.chart << '\n';
            }
            require(optimized.finite && oracle.finite &&
                        std::abs(optimized.affineDistance - 3.5F) < 3.0e-4F &&
                        std::abs(oracle.affineDistance - 3.5F) < 3.0e-4F,
                    "content shell changed full-affine ray policy");
            require(portalLength(optimized.origin - oracle.origin) < 0.012F &&
                        std::acos(std::clamp(portalDot(
                            optimized.direction, oracle.direction),
                            -1.0F, 1.0F)) < 0.012F,
                    "content-shell tangent diverges from full-affine oracle");
            if (havePreviousShell) {
                require(portalLength(optimized.origin -
                                     previousShell.origin) < 0.025F &&
                            std::acos(std::clamp(portalDot(
                                optimized.direction,
                                previousShell.direction), -1.0F, 1.0F)) <
                                0.025F &&
                            std::acos(std::clamp(portalDot(
                                portalNormalize(optimized.footprintV),
                                portalNormalize(previousShell.footprintV)),
                                -1.0F, 1.0F)) < 0.025F,
                        "content-shell tangent introduced an endpoint/Jacobian seam");
            }
            previousShell = optimized;
            havePreviousShell = true;
        }
    }

    // High-magnification image-family regression.  Sample neighboring screen
    // rays through both mouths and require practical C1 exit direction away
    // from the genuine throat separatrix.  The previous fixed solver cutoff
    // changed affine extent on a circular refinement class and produced broad
    // stepped/polygonal bands despite every individual ray being finite.
    for (std::uint32_t source = 0U; source < 2U; ++source) {
        const GlobalHandleObserverState observer =
            globalHandleResetObserver(settings, source);
        const PortalVector right = portalNormalize(
            portalCross(observer.forward, observer.up));
        GlobalHandleRayState previous{};
        float previousClosest = 0.0F;
        bool havePrevious = false;
        bool sawCrossing = false;
        bool sawExterior = false;
        std::uint32_t classTransitions = 0U;
        float maximumSmoothDelta = 0.0F;
        float maximumDeltaClosest = 0.0F;
        for (std::uint32_t sample = 0U; sample <= 1024U; ++sample) {
            const float screenX = std::lerp(
                -1.50F, 1.50F, static_cast<float>(sample) / 1024.0F);
            GlobalHandleRayState ray{};
            ray.origin = observer.position;
            ray.direction = portalNormalize(observer.forward + right * screenX);
            ray.footprintU = observer.forward;
            ray.footprintV = observer.up;
            const PortalVector relative = ray.origin - handleCenters[source];
            const float projected = portalDot(relative, ray.direction);
            const float closest = std::sqrt(std::max(
                portalDot(relative, relative) - projected * projected, 0.0F));
            globalHandleIntegrateRay(ray, 3.5F, settings, 4096U);
            require(ray.finite, "high-magnification handle ray became nonfinite");
            sawCrossing = sawCrossing || ray.crossings != 0U;
            sawExterior = sawExterior || ray.crossings == 0U;
            if (havePrevious) {
                if (ray.crossings != previous.crossings ||
                    ray.lastMouth != previous.lastMouth) {
                    ++classTransitions;
                } else if (std::abs(closest - settings.throatRadius) > 0.06F &&
                           std::abs(previousClosest - settings.throatRadius) >
                               0.06F) {
                    const float cosine = std::clamp(portalDot(
                        ray.direction, previous.direction), -1.0F, 1.0F);
                    const float delta = std::acos(cosine);
                    if (delta > maximumSmoothDelta) {
                        maximumSmoothDelta = delta;
                        maximumDeltaClosest = std::min(
                            closest, previousClosest);
                    }
                }
            }
            previous = ray;
            previousClosest = closest;
            havePrevious = true;
        }
        if (!(sawCrossing && sawExterior && classTransitions <= 4U &&
              maximumSmoothDelta < 2.10F)) {
            std::cerr << "high-magnification source=" << source
                      << " crossing/exterior=" << sawCrossing << '/'
                      << sawExterior << " transitions=" << classTransitions
                      << " max-delta=" << maximumSmoothDelta
                      << " closest=" << maximumDeltaClosest << '\n';
        }
        require(sawCrossing && sawExterior && classTransitions <= 4U &&
                    maximumSmoothDelta < 2.10F,
                "high-magnification exit direction has a noncritical solver band");
    }

    std::cout << "Global metric field invariants passed: 2.00-diameter "
                 "planet-safe C2 shared handle, 0.32 surface-like differential, "
                 "symmetric A/B body-optical entry, ULP tangent ownership, "
                 "non-owning content-shell tangent C1, "
                 "high-magnification C1 ray families, "
                 "proper-speed/frame continuity, Ellis oracle, "
                 "time reversal, and finite connection cache.\n";
    return 0;
}
