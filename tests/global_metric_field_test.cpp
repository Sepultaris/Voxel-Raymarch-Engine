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
    settings.globalHandleTailScale = 1.75F;
    settings.globalHandleMetricStrength = 0.72F;
    settings.globalPosition = {0.42F, -0.18F, 1.77F};
    settings.globalForward = portalNormalize({-0.22F, 0.08F, -0.97F});
    settings.globalUp = portalOrthonormalUp(
        settings.globalForward, {0.0F, 1.0F, 0.0F});
    settings.globalLastMouth = 1U;
    settings.globalAffineDistance = 2.75F;
    const IntrinsicEllisGpuParameters packedCrossing =
        intrinsicEllisGpuParameters(settings, {}, {});
    require(static_cast<std::uint32_t>(packedCrossing.control[0]) == 1U &&
                std::abs(packedCrossing.control[1] - 2.75F) < 1.0e-6F &&
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
            "GPU crossing packet lost the CPU chart owner/affine pose");
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
    const auto handleCenters = globalHandleCenters(settings);
    const PortalVector testNormal = portalNormalize({0.31F, 0.42F, 0.85F});
    const PortalVector mappedNormal = portalNormalize(
        globalHandleRotateAcross(testNormal * -1.0F, 0U, settings));
    const float mouth = settings.throatRadius;
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
        globalHandleIntegrateRay(optical, 1.35F, settings, 512U);
        const GlobalHandleObserverState beforeBody = body;
        body.properSpeed = 1.0F;
        globalHandleAdvanceObserver(body, body.forward, 1.35F, settings);
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
                                 settings, 512U);
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
        std::uint32_t crossingFrame = 0xffffffffU;
        bool retainedDestinationOwner = false;
        for (std::uint32_t frameIndex = 0U; frameIndex < 240U; ++frameIndex) {
            const std::uint32_t crossingsBefore = replay.crossings;
            const float affineBefore = replay.affineDistance;
            globalHandleAdvanceObserver(
                replay, replay.forward,
                settings.freeFlySpeed / 60.0F, settings);
            require(replay.finite &&
                        replay.affineDistance > affineBefore &&
                        portalDot(portalCross(portalNormalize(portalCross(
                            replay.forward, replay.up)), replay.forward),
                            replay.up) > 0.999F,
                    "video-style camera lost affine/frame continuity");
            if (replay.crossings != crossingsBefore) {
                crossingFrame = frameIndex;
                require(replay.lastMouth == 1U - source,
                        "camera crossing discarded destination half-open owner");
                const IntrinsicEllisGpuParameters crossingPacket =
                    intrinsicEllisGpuParameters(
                        [&] {
                            IntrinsicEllisSettings packed = settings;
                            packed.globalPosition = replay.position;
                            packed.globalForward = replay.forward;
                            packed.globalUp = replay.up;
                            packed.globalLastMouth = replay.lastMouth;
                            packed.globalAffineDistance = replay.affineDistance;
                            return packed;
                        }(), {}, {});
                require(static_cast<std::uint32_t>(
                            crossingPacket.control[0]) == 1U - source,
                        "first post-crossing GPU packet lost destination owner");
            }
            if (replay.lastMouth == 1U - source) {
                const float destinationDistance = portalLength(
                    replay.position - handleCenters[1U - source]);
                require(destinationDistance <= mouth * 1.101F,
                        "destination owner survived outside overlap release");
                retainedDestinationOwner = true;
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
                    globalHandleIntegrateRay(ray, 3.5F, settings, 512U);
                    require(ray.finite,
                            "video-style crossing bundle became nonfinite");
                    bundle[index] = ray;
                    if (havePreviousBundle) {
                        const float cosine = std::clamp(portalDot(
                            previousBundle[index].direction,
                            ray.direction), -1.0F, 1.0F);
                        const float delta = std::acos(cosine);
                        maximumBundleDelta = std::max(
                            maximumBundleDelta, delta);
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
            if (havePreviousBundle) {
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
        require(crossingFrame != 0xffffffffU &&
                    retainedDestinationOwner &&
                    maximumCenterDelta < 0.0015F &&
                    maximumBundleDelta < 0.008F &&
                    maximumCenterOriginDelta < 0.02F &&
                    maximumJacobianDelta < 0.10F,
                "video-style off-axis perspective jumped at a chart crossing");
        std::cout << "crossing replay source=" << source
                  << " frame=" << crossingFrame
                  << " center/bundle=" << maximumCenterDelta << '/'
                  << maximumBundleDelta
                  << " origin/jacobian=" << maximumCenterOriginDelta << '/'
                  << maximumJacobianDelta << '\n';
    }

    // Dense subframe limit at l=0/the identified mouth chart event. Compare
    // the same 9x7 camera ray bundle immediately before and after the A/B
    // coordinate remap. The physical camera state is one point in the handle;
    // only its chart coordinates jump. Mapped origin/direction, transported
    // footprint, and the finite-difference Jacobian must converge from both
    // sides instead of switching the complete rendered scene family.
    for (std::uint32_t source = 0U; source < 2U; ++source) {
        const std::uint32_t destination = 1U - source;
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
                    require(globalHandleApplyThroatCrossing(
                                after.origin, after.direction, nullptr,
                                source, settings),
                            "dense crossing could not map destination chart");
                    const PortalVector destinationN = portalNormalize(
                        after.origin - handleCenters[destination]);
                    after.origin = after.origin + destinationN * epsilon;
                    after.footprintU = portalNormalize(globalHandleRotateAcross(
                        beforeForward, source, settings));
                    // These are screen-ray differential directions, not the
                    // observer's orthonormal camera-up vector.  The throat
                    // coordinate map rotates the complete differential
                    // rigidly; reprojecting each off-axis ray's differential
                    // against that ray would manufacture a field-angle jump.
                    after.footprintV = portalNormalize(globalHandleRotateAcross(
                        beforeUp, source, settings));
                    after.lastMouth = destination;

                    globalHandleIntegrateRay(before, 3.5F, settings, 512U);
                    globalHandleIntegrateRay(after, 3.5F, settings, 512U);
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
            if (!(maximumOriginDelta < std::max(0.002F, epsilon * 6.0F) &&
                  maximumDirectionDelta < 0.002F &&
                  maximumFootprintDelta < 0.002F)) {
                std::cerr << "dense crossing source/epsilon=" << source << '/'
                          << epsilon << " origin/direction/footprint="
                          << maximumOriginDelta << '/' << maximumDirectionDelta
                          << '/' << maximumFootprintDelta << " pixel="
                          << maximumX << ',' << maximumY << " crossings="
                          << maximumBeforeCrossings << '/'
                          << maximumAfterCrossings << '\n';
            }
            require(maximumOriginDelta < std::max(0.002F, epsilon * 6.0F) &&
                        maximumDirectionDelta < 0.002F &&
                        maximumFootprintDelta < 0.002F,
                    "dense +/-epsilon crossing changed mapped perspective");
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
            globalHandleAdvanceObserver(body, body.forward, 1.35F, rotated);
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
    const float inwardX = std::nextafter(
        mouth, 0.0F);
    const float outwardX = std::nextafter(
        mouth, std::numeric_limits<float>::infinity());
    require(globalHandleSegmentSphereEntry(
                handleCenters[0] + PortalVector{inwardX, -mouth * 2.0F, 0.0F},
                handleCenters[0] + PortalVector{inwardX, mouth * 2.0F, 0.0F},
                handleCenters[0], mouth, tangentFraction) &&
            !globalHandleSegmentSphereEntry(
                handleCenters[0] + PortalVector{outwardX, -mouth * 2.0F, 0.0F},
                handleCenters[0] + PortalVector{outwardX, mouth * 2.0F, 0.0F},
                handleCenters[0], mouth, tangentFraction),
            "ULP half-open mouth ownership is biased or non-deterministic");

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
                optimized, 3.5F, settings, 512U, true);
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
            maximumOriginError = std::max(maximumOriginError,
                portalLength(optimized.origin - reference.origin));
            maximumDirectionError = std::max(maximumDirectionError,
                std::acos(std::clamp(portalDot(
                    optimized.direction, reference.direction),
                    -1.0F, 1.0F)));
        }
        if (!(maximumOriginError < 0.012F &&
              maximumDirectionError < 0.012F)) {
            std::cerr << "optimized differential source=" << source
                      << " origin/direction=" << maximumOriginError << '/'
                      << maximumDirectionError << '\n';
        }
        require(maximumOriginError < 0.012F &&
                    maximumDirectionError < 0.012F,
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
                optimized, 3.5F, settings, 512U, true);
            globalHandleIntegrateRay(
                oracle, 3.5F, settings, 8192U, true, true);
            require(optimized.finite && oracle.finite &&
                        std::abs(optimized.affineDistance - 3.5F) < 1.0e-4F &&
                        std::abs(oracle.affineDistance - 3.5F) < 1.0e-4F,
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
                -0.75F, 0.75F, static_cast<float>(sample) / 1024.0F);
            GlobalHandleRayState ray{};
            ray.origin = observer.position;
            ray.direction = portalNormalize(observer.forward + right * screenX);
            ray.footprintU = observer.forward;
            ray.footprintV = observer.up;
            const PortalVector relative = ray.origin - handleCenters[source];
            const float projected = portalDot(relative, ray.direction);
            const float closest = std::sqrt(std::max(
                portalDot(relative, relative) - projected * projected, 0.0F));
            globalHandleIntegrateRay(ray, 3.5F, settings, 512U);
            require(ray.finite, "high-magnification handle ray became nonfinite");
            sawCrossing = sawCrossing || ray.crossings != 0U;
            sawExterior = sawExterior || ray.crossings == 0U;
            if (havePrevious) {
                if (ray.crossings != previous.crossings ||
                    ray.lastMouth != previous.lastMouth) {
                    ++classTransitions;
                } else if (std::abs(closest - mouth) > 0.06F &&
                           std::abs(previousClosest - mouth) > 0.06F) {
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
        if (!(sawCrossing && sawExterior && classTransitions == 2U &&
              maximumSmoothDelta < 0.08F)) {
            std::cerr << "high-magnification source=" << source
                      << " crossing/exterior=" << sawCrossing << '/'
                      << sawExterior << " transitions=" << classTransitions
                      << " max-delta=" << maximumSmoothDelta
                      << " closest=" << maximumDeltaClosest << '\n';
        }
        require(sawCrossing && sawExterior && classTransitions == 2U &&
                    maximumSmoothDelta < 0.08F,
                "high-magnification exit direction has a noncritical solver band");
    }

    std::cout << "Global metric field invariants passed: engineered C2 shared "
                 "handle, symmetric A/B body-optical entry, exact tangent "
                 "ownership, non-owning content-shell tangent C1, "
                 "high-magnification C1 ray families, "
                 "proper-speed/frame continuity, Ellis oracle, "
                 "time reversal, and finite connection cache.\n";
    return 0;
}
