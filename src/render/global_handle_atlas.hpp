#pragma once

#include "render/intrinsic_ellis_manifold.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace voxel {

// Engineered, static same-universe handle used only by the global-metric lab.
// It is deliberately not described as a vacuum Einstein solution.  The two
// exterior mouth charts share a conformally-flat optical metric whose
// non-compact tails are blended by a C2 partition.  At a throat the opposite
// chart is only a coordinate continuation; metric evaluation and transport do
// not change policy there.
struct GlobalHandleMetricSample {
    std::array<std::array<double, 3>, 3> metric{};
    std::array<std::array<double, 3>, 3> inverse{};
    std::array<std::array<std::array<double, 3>, 3>, 3> christoffel{};
    PortalVector logConformalGradient{};
    double conformal{1.0};
    double curvatureCue{};
    bool finite{true};
};

struct GlobalHandleObserverState {
    PortalVector position{};
    PortalVector forward{0.0F, 0.0F, -1.0F};
    PortalVector up{0.0F, 1.0F, 0.0F};
    PortalVector velocity{};
    std::uint32_t lastMouth{2U};
    std::uint32_t crossings{};
    float affineDistance{};
    float properSpeed{};
    std::uint32_t chart{}; // 0 shared exterior, 1 handle atlas
    EllisState handleState{};
    EllisFrame handleFrame{};
    bool finite{true};
};

struct GlobalHandleRayState {
    PortalVector origin{};
    PortalVector direction{0.0F, 0.0F, -1.0F};
    PortalVector footprintU{};
    PortalVector footprintV{};
    std::uint32_t lastMouth{2U};
    std::uint32_t crossings{};
    std::uint32_t steps{};
    float affineDistance{};
    std::uint32_t chart{};
    EllisState handleState{};
    EllisFrame handleFrame{};
    bool finite{true};
};

enum class GlobalHandleLensFootprintPreset : std::uint32_t {
    PlanetSafe = 0U,
    Compact = 1U,
    Moderate = 2U,
    Dramatic = 3U,
    Custom = 4U
};

inline void globalHandleApplyLensFootprintPreset(
    IntrinsicEllisSettings& settings,
    GlobalHandleLensFootprintPreset preset) noexcept {
    switch (preset) {
    case GlobalHandleLensFootprintPreset::PlanetSafe:
        // A close-view-safe physical mouth with a tail that is broad in mouth
        // units but remains localized in world units. This preserves a real
        // exterior lens annulus instead of exposing the crossing family as a
        // hard aperture, without restoring the rejected frame-wide tail.
        settings.contentSphereRadius = 0.025F;
        settings.globalHandleTailScale = 0.009F;
        break;
    case GlobalHandleLensFootprintPreset::Compact:
        settings.contentSphereRadius = 0.12F;
        settings.globalHandleTailScale = 0.020F;
        break;
    case GlobalHandleLensFootprintPreset::Moderate:
        settings.contentSphereRadius = 0.18F;
        settings.globalHandleTailScale = 0.025F;
        break;
    case GlobalHandleLensFootprintPreset::Dramatic:
        settings.contentSphereRadius = 0.34F;
        settings.globalHandleTailScale = 0.85F;
        break;
    default:
        return;
    }
    settings.throatRadius = settings.contentSphereRadius *
        kPortalGrThroatRatio;
    settings.contentExitProperDepth = std::sqrt(std::max(
        settings.contentSphereRadius * settings.contentSphereRadius -
            settings.throatRadius * settings.throatRadius,
        1.0e-8F));
}

[[nodiscard]] inline std::array<PortalVector, 2> globalHandleCenters(
    const IntrinsicEllisSettings& settings) noexcept {
    return {
        portalEndpoint(settings.endpointAAzimuth, settings.endpointAElevation,
                       settings.endpointADistance),
        portalEndpoint(settings.endpointBAzimuth, settings.endpointBElevation,
                       settings.endpointBDistance)};
}

[[nodiscard]] inline float globalHandleCriticalAngularDiameter(
    const IntrinsicEllisSettings& settings, PortalVector observer,
    std::uint32_t mouthIndex) noexcept {
    const auto centers = globalHandleCenters(settings);
    const float distance = portalLength(observer -
        centers[std::min(mouthIndex, 1U)]);
    const float ratio = std::clamp(settings.throatRadius /
        std::max(distance, settings.throatRadius), 0.0F, 1.0F);
    return 2.0F * std::asin(ratio);
}

[[nodiscard]] inline double globalHandleFourth(double value) noexcept {
    const double square = value * value;
    return square * square;
}

[[nodiscard]] inline float globalHandleMouthRadius(
    const IntrinsicEllisSettings& settings) noexcept {
    return std::max(settings.contentSphereRadius, 1.0e-4F);
}

[[nodiscard]] inline float globalHandleBoundaryConformal(
    const IntrinsicEllisSettings& settings) noexcept {
    const float throat = std::max(settings.throatRadius, 1.0e-4F);
    const float halfLength = throat * std::clamp(
        settings.globalHandleLengthDiameters, 0.20F, 8.0F);
    const float endRadius = std::sqrt(
        halfLength * halfLength + throat * throat);
    return endRadius / globalHandleMouthRadius(settings);
}

[[nodiscard]] inline float globalHandleHalfLength(
    const IntrinsicEllisSettings& settings) noexcept {
    const float throat = std::max(settings.throatRadius, 1.0e-4F);
    return throat * std::clamp(settings.globalHandleLengthDiameters,
                               0.20F, 8.0F);
}

[[nodiscard]] inline float globalHandleCriticalImpactBand(
    float entryRadius, float throatRadius) noexcept {
    // Match the GPU's representable-scale half-open owner for the one
    // measure-zero critical geodesic. This is not an angular/pixel tolerance:
    // every finite near-critical ray outside this float-sized band is
    // integrated normally.
    return std::max(throatRadius * 2.0e-5F,
                    entryRadius * 16.0F *
                        std::numeric_limits<float>::epsilon());
}

[[nodiscard]] inline float globalHandleReferenceLengthDiameters(
    const IntrinsicEllisSettings& settings) noexcept {
    // Historical compact-collar geometry used exp(strength)*mouth as its
    // attachment radius. Recover that length only for the comparison preset;
    // the accepted short handle no longer compresses this scale internally.
    const float boundaryAreal = std::exp(std::clamp(
        settings.globalHandleMetricStrength, 0.0F, 4.0F)) *
        globalHandleMouthRadius(settings);
    const float throat = std::min(std::max(settings.throatRadius, 1.0e-4F),
                                  boundaryAreal * 0.95F);
    const float referenceHalf = std::sqrt(std::max(
        boundaryAreal * boundaryAreal - throat * throat, 1.0e-8F));
    return referenceHalf / throat;
}

// The complete intrinsic handle is exact Ellis. Scale matching to the shared
// exterior is carried by a broad conformal tail whose boundary value and first
// two derivatives are the pullback of this profile. This guarantees one and
// only one optical waist at u=0 instead of adding a collar lens at each end.
[[nodiscard]] inline EllisRadialProfile globalHandleRadialProfile(
    float signedU, const IntrinsicEllisSettings& settings) noexcept {
    const float x = std::abs(signedU);
    const float throat = std::max(settings.throatRadius, 1.0e-4F);
    const float exactR = std::sqrt(x * x + throat * throat);
    const float exactD = x / std::max(exactR, 1.0e-8F);
    const float exactDD = throat * throat /
        std::max(exactR * exactR * exactR, 1.0e-10F);
    return {exactR, std::copysign(exactD, signedU), exactDD};
}

// Project a transported camera tetrad back onto SO(3) without selecting an
// angular reference axis and without a sign-ambiguous Gram-Schmidt repair.
// Forward and up determine the visible camera orientation; right is derived
// with the engine's right=cross(forward,up) convention. The only fallback is
// the already-transported right vector, so no world-axis branch can cause a
// sudden roll flip at a pole or chart owner change.
[[nodiscard]] inline bool globalHandleOrthonormalizeFrame(
    EllisFrame& frame, PortalVector angularPosition,
    EllisTangent forwardFallback) noexcept {
    const PortalVector n = portalNormalize(angularPosition);
    PortalVector forward = ellisTangentToEmbedded(frame.forward, n);
    if (!(portalLength(forward) > 1.0e-6F) || !portalFinite(forward)) {
        forward = ellisTangentToEmbedded(forwardFallback, n);
    }
    if (!(portalLength(forward) > 1.0e-6F) || !portalFinite(forward)) {
        return false;
    }
    forward = portalNormalize(forward);
    PortalVector up = ellisTangentToEmbedded(frame.up, n);
    up = up - forward * portalDot(up, forward);
    if (!(portalLength(up) > 1.0e-6F) || !portalFinite(up)) {
        PortalVector transportedRight = ellisTangentToEmbedded(frame.right, n);
        transportedRight = transportedRight -
            forward * portalDot(transportedRight, forward);
        if (!(portalLength(transportedRight) > 1.0e-6F) ||
            !portalFinite(transportedRight)) {
            return false;
        }
        up = portalCross(portalNormalize(transportedRight), forward);
    }
    up = portalNormalize(up);
    const PortalVector right = portalNormalize(portalCross(forward, up));
    up = portalNormalize(portalCross(right, forward));
    frame.forward = ellisTangentFromEmbedded(forward, n);
    frame.right = ellisTangentFromEmbedded(right, n);
    frame.up = ellisTangentFromEmbedded(up, n);
    return ellisFrameHandedness(frame, n) > 0.999F;
}

[[nodiscard]] inline EllisDerivative globalHandleInteriorDerivative(
    const EllisState& state, const IntrinsicEllisSettings& settings) noexcept {
    const EllisRadialProfile profile = globalHandleRadialProfile(
        state.properDepth, settings);
    const float radius = std::max(profile.radius, 1.0e-6F);
    const float expansion = profile.firstDerivative / radius;
    const float angularSpeed2 = portalDot(state.velocity.angular,
                                          state.velocity.angular);
    return {state.velocity.radial,
            state.velocity.angular * (1.0F / radius),
            {expansion * angularSpeed2,
             state.velocity.angular * (-expansion * state.velocity.radial) -
                 state.angularPosition * (angularSpeed2 / radius)}};
}

[[nodiscard]] inline EllisTangent globalHandleInteriorParallelDerivative(
    const EllisState& path, const EllisTangent& transported,
    const IntrinsicEllisSettings& settings) noexcept {
    const EllisRadialProfile profile = globalHandleRadialProfile(
        path.properDepth, settings);
    const float radius = std::max(profile.radius, 1.0e-6F);
    const float expansion = profile.firstDerivative / radius;
    const float coupling = portalDot(path.velocity.angular,
                                     transported.angular);
    return {expansion * coupling,
            // In the physical orthonormal angular representation the
            // angular basis is parallel along the radial coordinate:
            // nabla_(e_l) e_a == 0.  The previous extra
            // -R'/R * ldot * transported.angular term treated the basis as
            // coordinate-scaled a second time.  It changed frame length and
            // made a transported camera forward peel away from an identical
            // geodesic tangent, producing the manual whole-view roll/flip.
            path.velocity.angular * (-expansion * transported.radial) -
                path.angularPosition * (coupling / radius)};
}

inline void globalHandleIntegrateInteriorStep(
    EllisState& state, EllisFrame& frame, float h,
    const IntrinsicEllisSettings& settings) noexcept {
    const EllisState initial = state;
    const EllisFrame initialFrame = frame;
    const EllisDerivative k1 = globalHandleInteriorDerivative(initial, settings);
    const EllisState s2 = ellisAddDerivative(initial, k1, h * 0.5F);
    const EllisDerivative k2 = globalHandleInteriorDerivative(s2, settings);
    const EllisState s3 = ellisAddDerivative(initial, k2, h * 0.5F);
    const EllisDerivative k3 = globalHandleInteriorDerivative(s3, settings);
    const EllisState s4 = ellisAddDerivative(initial, k3, h);
    const EllisDerivative k4 = globalHandleInteriorDerivative(s4, settings);
    state.properDepth += h / 6.0F *
        (k1.properDepth + 2.0F * k2.properDepth +
         2.0F * k3.properDepth + k4.properDepth);
    state.angularPosition = portalNormalize(
        state.angularPosition + (k1.angularPosition +
            k2.angularPosition * 2.0F + k3.angularPosition * 2.0F +
            k4.angularPosition) * (h / 6.0F), initial.angularPosition);
    state.velocity = ellisNormalize(
        state.velocity + (k1.velocity + k2.velocity * 2.0F +
            k3.velocity * 2.0F + k4.velocity) * (h / 6.0F),
        state.angularPosition, initial.velocity);
    const auto transport = [&](EllisTangent value) noexcept {
        const EllisTangent p1 = globalHandleInteriorParallelDerivative(
            initial, value, settings);
        const EllisTangent p2 = globalHandleInteriorParallelDerivative(
            s2, value + p1 * (h * 0.5F), settings);
        const EllisTangent p3 = globalHandleInteriorParallelDerivative(
            s3, value + p2 * (h * 0.5F), settings);
        const EllisTangent p4 = globalHandleInteriorParallelDerivative(
            s4, value + p3 * h, settings);
        return ellisProjectTangent(
            value + (p1 + p2 * 2.0F + p3 * 2.0F + p4) * (h / 6.0F),
            state.angularPosition);
    };
    frame.forward = transport(initialFrame.forward);
    frame.right = transport(initialFrame.right);
    frame.up = transport(initialFrame.up);

    // Parallel transport preserves an orthonormal tetrad analytically. Remove
    // finite-RK drift with a basis-free SO(3) projection; never negate one
    // lateral axis as a post-hoc handedness repair.
    if (!globalHandleOrthonormalizeFrame(
            frame, state.angularPosition, initialFrame.forward)) {
        frame = initialFrame;
        frame.forward = ellisProjectTangent(
            frame.forward, state.angularPosition);
        frame.right = ellisProjectTangent(
            frame.right, state.angularPosition);
        frame.up = ellisProjectTangent(frame.up, state.angularPosition);
        (void)globalHandleOrthonormalizeFrame(
            frame, state.angularPosition, state.velocity);
    }
}

struct GlobalHandleScalarField {
    double value{};
    PortalVector gradient{};
    bool finite{true};
};

[[nodiscard]] inline GlobalHandleScalarField globalHandleLogConformal(
    PortalVector position, const IntrinsicEllisSettings& settings) noexcept {
    const auto centers = globalHandleCenters(settings);
    // This radius is an overlap coordinate surface, not a visibility owner.
    // Its physical areal radius is exp(phi)*mouth and matches the handle
    // collar pullback through second order.
    const double mouth = std::max(
        static_cast<double>(globalHandleMouthRadius(settings)), 1.0e-5);
    const double throat = std::max(
        static_cast<double>(settings.throatRadius), 1.0e-5);
    const double tail = std::max(
        static_cast<double>(settings.globalHandleTailScale), mouth * 0.04);
    const double halfLength = static_cast<double>(
        globalHandleHalfLength(settings));
    const double endRadius = std::sqrt(
        halfLength * halfLength + throat * throat);
    const double endDerivative = halfLength / endRadius;
    const double endSecond = throat * throat /
        std::max(endRadius * endRadius * endRadius, 1.0e-24);
    const double boundaryOmega = endRadius / mouth;
    const double phi0 = std::log(boundaryOmega);
    const double phi1 = (endDerivative - 1.0) / mouth;
    const double phi2 = (boundaryOmega * endSecond - phi1) / mouth;
    // A positive rational tail carries the exact Ellis boundary value and
    // first two proper-radial jets, while its s^4 term controls only far-field
    // decay (zero boundary jet through second order).
    const bool zeroBoundaryValue = std::abs(phi0) < 1.0e-4;
    const double safePhi0 = zeroBoundaryValue
        ? std::copysign(1.0e-4, phi0 == 0.0 ? 1.0 : phi0) : phi0;
    const double coefficientB = -phi1 / safePhi0;
    const double coefficientC = coefficientB * coefficientB -
        phi2 / (2.0 * safePhi0);
    const double inverseTail4 = 1.0 / globalHandleFourth(tail);

    std::array<double, 2> s{};
    std::array<double, 2> s4{};
    std::array<double, 2> local{};
    std::array<double, 2> localDerivative{};
    std::array<PortalVector, 2> radial{};
    for (std::size_t index = 0; index < 2; ++index) {
        const PortalVector relative = position - centers[index];
        const double radius = std::max(
            static_cast<double>(portalLength(relative)), 1.0e-9);
        radial[index] = relative * static_cast<float>(1.0 / radius);
        // The external chart owns r>=mouth.  Values a few ULP inside can occur
        // only while a throat event is being split; the even continuation is
        // used so g and Gamma converge from both charts.
        s[index] = radius - mouth;
        s4[index] = globalHandleFourth(s[index]);
        const double localDenominator = 1.0 +
            coefficientB * s[index] +
            coefficientC * s[index] * s[index] +
            s4[index] * inverseTail4;
        const double localDenominatorDerivative = coefficientB +
            2.0 * coefficientC * s[index] +
            4.0 * s[index] * s[index] * s[index] * inverseTail4;
        if (zeroBoundaryValue) {
            const double numerator = phi0 + phi1 * s[index] +
                0.5 * phi2 * s[index] * s[index];
            const double numeratorDerivative = phi1 + phi2 * s[index];
            const double farDenominator = 1.0 + s4[index] * inverseTail4;
            const double farDerivative = 4.0 * s[index] * s[index] *
                s[index] * inverseTail4;
            local[index] = numerator / farDenominator;
            localDerivative[index] = (numeratorDerivative * farDenominator -
                numerator * farDerivative) /
                (farDenominator * farDenominator);
        } else {
            local[index] = phi0 / localDenominator;
            localDerivative[index] = -phi0 * localDenominatorDerivative /
                (localDenominator * localDenominator);
        }

    }

    const double tail4 = globalHandleFourth(tail);
    const double denominator = s4[0] * s4[1] +
        tail4 * (s4[0] + s4[1]);
    GlobalHandleScalarField result{};
    if (!(denominator > 1.0e-30) || !std::isfinite(denominator)) {
        result.finite = false;
        return result;
    }
    const double w0 = tail4 * s4[1] / denominator;
    const double w1 = tail4 * s4[0] / denominator;
    result.value = w0 * local[0] + w1 * local[1];

    // Analytic derivative of the C2 partition.  s^4 makes the opposite chart
    // and flat-exterior contribution vanish with three derivatives at either
    // throat, so the pullback metric is C2 and Gamma is continuous.
    const double dDenominatorDs0 = 4.0 * s[0] * s[0] * s[0] *
        (s4[1] + tail4);
    const double dDenominatorDs1 = 4.0 * s[1] * s[1] * s[1] *
        (s4[0] + tail4);
    const double dw0Ds0 = -tail4 * s4[1] * dDenominatorDs0 /
        (denominator * denominator);
    const double dw0Ds1 = tail4 * 4.0 * s[1] * s[1] * s[1] /
        denominator - tail4 * s4[1] * dDenominatorDs1 /
        (denominator * denominator);
    const double dw1Ds0 = tail4 * 4.0 * s[0] * s[0] * s[0] /
        denominator - tail4 * s4[0] * dDenominatorDs0 /
        (denominator * denominator);
    const double dw1Ds1 = -tail4 * s4[0] * dDenominatorDs1 /
        (denominator * denominator);
    const double dValueDs0 = dw0Ds0 * local[0] +
        w0 * localDerivative[0] + dw1Ds0 * local[1];
    const double dValueDs1 = dw0Ds1 * local[0] +
        dw1Ds1 * local[1] + w1 * localDerivative[1];
    result.gradient = radial[0] * static_cast<float>(dValueDs0) +
                      radial[1] * static_cast<float>(dValueDs1);
    result.finite = std::isfinite(result.value) &&
        portalFinite(result.gradient);
    return result;
}

[[nodiscard]] inline GlobalHandleMetricSample evaluateGlobalHandleMetric(
    PortalVector position, const IntrinsicEllisSettings& settings) noexcept {
    GlobalHandleMetricSample result{};
    const GlobalHandleScalarField field = globalHandleLogConformal(
        position, settings);
    result.conformal = std::exp(field.value);
    const double conformal2 = result.conformal * result.conformal;
    result.logConformalGradient = field.gradient;
    result.curvatureCue = static_cast<double>(portalLength(field.gradient));
    for (std::size_t axis = 0; axis < 3; ++axis) {
        result.metric[axis][axis] = conformal2;
        result.inverse[axis][axis] = 1.0 / conformal2;
    }
    const double gradient[3]{field.gradient.x, field.gradient.y,
                             field.gradient.z};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            for (std::size_t k = 0; k < 3; ++k) {
                result.christoffel[i][j][k] =
                    (i == j ? gradient[k] : 0.0) +
                    (i == k ? gradient[j] : 0.0) -
                    (j == k ? gradient[i] : 0.0);
            }
        }
    }
    result.finite = field.finite && std::isfinite(result.conformal) &&
        result.conformal > 0.0;
    return result;
}

[[nodiscard]] inline bool globalHandleSegmentSphereEntry(
    PortalVector start, PortalVector end, PortalVector center, float radius,
    float& fraction) noexcept {
    const PortalVector delta = end - start;
    const PortalVector relative = start - center;
    const double aa = static_cast<double>(delta.x) * delta.x +
        static_cast<double>(delta.y) * delta.y +
        static_cast<double>(delta.z) * delta.z;
    const double bb = 2.0 * (static_cast<double>(relative.x) * delta.x +
        static_cast<double>(relative.y) * delta.y +
        static_cast<double>(relative.z) * delta.z);
    const double rr = static_cast<double>(radius) * radius;
    const double cc = static_cast<double>(relative.x) * relative.x +
        static_cast<double>(relative.y) * relative.y +
        static_cast<double>(relative.z) * relative.z - rr;
    // A chart-boundary point reconstructed in float can land one ULP on the
    // interior side.  It still owns the zero-distance inward event.  Taking
    // only the quadratic's near root would otherwise reject that event as a
    // negative fraction (observed asymmetrically at mouth B).  This is a
    // roundoff-sized half-open ownership rule, not a directional tolerance.
    const double boundaryRoundoff = 32.0 *
        std::numeric_limits<float>::epsilon() *
        std::max(rr, static_cast<double>(relative.x) * relative.x +
                     static_cast<double>(relative.y) * relative.y +
                     static_cast<double>(relative.z) * relative.z);
    const double inward = static_cast<double>(relative.x) * delta.x +
        static_cast<double>(relative.y) * delta.y +
        static_cast<double>(relative.z) * delta.z;
    if (cc <= boundaryRoundoff && cc >= -boundaryRoundoff && inward < 0.0) {
        fraction = 0.0F;
        return true;
    }
    const double discriminant = bb * bb - 4.0 * aa * cc;
    const double discriminantRoundoff = 16.0 *
        std::numeric_limits<double>::epsilon() *
        (bb * bb + std::abs(4.0 * aa * cc));
    if (!(aa > 1.0e-18) ||
        !(discriminant > discriminantRoundoff)) return false;
    const double root = std::sqrt(std::max(discriminant, 0.0));
    const double candidate = (-bb - root) / (2.0 * aa);
    if (!(candidate >= 0.0 && candidate <= 1.0)) return false;
    const PortalVector hit = start + delta * static_cast<float>(candidate);
    const PortalVector normal = portalNormalize(hit - center);
    if (portalDot(delta, normal) >= 0.0F) return false;
    fraction = static_cast<float>(candidate);
    return true;
}

[[nodiscard]] inline PortalVector globalHandleRotateAcross(
    PortalVector value, std::uint32_t source,
    const IntrinsicEllisSettings& settings) noexcept {
    return source == 0U ? intrinsicEllisRotateEndB(value, settings)
                        : intrinsicEllisRotateEndBInverse(value, settings);
}

[[nodiscard]] inline EllisTangent globalHandleTangentFromExterior(
    PortalVector value, PortalVector exteriorN, std::uint32_t mouthIndex,
    const IntrinsicEllisSettings& settings) noexcept {
    PortalVector n = portalNormalize(exteriorN);
    PortalVector localValue = value;
    if (mouthIndex == 1U) {
        // End B is an antipodal sphere attachment. Reversing only the radial
        // axis has determinant -1 and mirrors the complete camera image.
        // Reversing the two angular differentials as well makes this a proper
        // SO(3) transition through the shared handle.
        n = portalNormalize(
            intrinsicEllisRotateEndBInverse(n, settings) * -1.0F);
        localValue = intrinsicEllisRotateEndBInverse(localValue, settings);
    }
    const float exteriorRadial = portalDot(localValue, n);
    PortalVector angular = localValue - n * exteriorRadial;
    if (mouthIndex == 1U) angular = angular * -1.0F;
    return ellisProjectTangent({exteriorRadial, angular}, n);
}

[[nodiscard]] inline PortalVector globalHandleTangentToExterior(
    const EllisTangent& value, PortalVector intrinsicN,
    std::uint32_t mouthIndex,
    const IntrinsicEllisSettings& settings) noexcept {
    const PortalVector n = portalNormalize(intrinsicN);
    PortalVector result = mouthIndex == 0U
        ? n * value.radial + value.angular
        : n * value.radial - value.angular;
    if (mouthIndex == 1U) {
        result = intrinsicEllisRotateEndB(result, settings);
    }
    return portalNormalize(result);
}

inline void globalHandleEnterInterior(
    GlobalHandleRayState& state, std::uint32_t source,
    const IntrinsicEllisSettings& settings) noexcept {
    const auto centers = globalHandleCenters(settings);
    const PortalVector exteriorN = portalNormalize(
        state.origin - centers[source]);
    const PortalVector intrinsicN = source == 0U ? exteriorN
        : portalNormalize(intrinsicEllisRotateEndBInverse(
              exteriorN, settings) * -1.0F);
    state.handleState.properDepth = source == 0U
        ? globalHandleHalfLength(settings) : -globalHandleHalfLength(settings);
    state.handleState.angularPosition = intrinsicN;
    state.handleState.velocity = globalHandleTangentFromExterior(
        state.direction, exteriorN, source, settings);
    const EllisRadialProfile entryProfile = globalHandleRadialProfile(
        state.handleState.properDepth, settings);
    const float tangentMagnitude = portalLength(
        state.handleState.velocity.angular);
    const float impact = entryProfile.radius * tangentMagnitude;
    const float impactBand = globalHandleCriticalImpactBand(
        entryProfile.radius, settings.throatRadius);
    if (std::abs(impact - settings.throatRadius) < impactBand * 0.5F) {
        const float ownedSide = impact <= settings.throatRadius ? -1.0F : 1.0F;
        const float ownedImpact = std::max(
            settings.throatRadius + ownedSide * impactBand * 0.5F, 0.0F);
        const float ownedTangent = std::clamp(
            ownedImpact / std::max(entryProfile.radius, 1.0e-8F),
            0.0F, 0.999999F);
        if (tangentMagnitude > 1.0e-8F) {
            state.handleState.velocity.angular =
                state.handleState.velocity.angular *
                (ownedTangent / tangentMagnitude);
        }
        state.handleState.velocity.radial = std::copysign(
            std::sqrt(std::max(1.0F - ownedTangent * ownedTangent, 0.0F)),
            state.handleState.velocity.radial);
    }
    state.handleFrame.forward = globalHandleTangentFromExterior(
        state.footprintU, exteriorN, source, settings);
    state.handleFrame.up = globalHandleTangentFromExterior(
        state.footprintV, exteriorN, source, settings);
    const PortalVector exteriorRight = portalNormalize(portalCross(
        state.footprintU, state.footprintV));
    state.handleFrame.right = globalHandleTangentFromExterior(
        exteriorRight, exteriorN, source, settings);
    if (!globalHandleOrthonormalizeFrame(
            state.handleFrame, intrinsicN, state.handleState.velocity)) {
        state.finite = false;
        return;
    }
    state.chart = 1U;
    state.lastMouth = source;
}

inline void globalHandleExitInterior(
    GlobalHandleRayState& state, std::uint32_t destination,
    const IntrinsicEllisSettings& settings) noexcept {
    const auto centers = globalHandleCenters(settings);
    PortalVector exteriorN = state.handleState.angularPosition;
    if (destination == 1U) {
        exteriorN = portalNormalize(intrinsicEllisRotateEndB(
            exteriorN, settings) * -1.0F);
    }
    const float mouth = globalHandleMouthRadius(settings);
    const float epsilon = std::max(
        std::nextafter(mouth, std::numeric_limits<float>::infinity()) - mouth,
        2.0e-7F);
    state.origin = centers[destination] + exteriorN * (mouth + epsilon);
    state.direction = globalHandleTangentToExterior(
        state.handleState.velocity, state.handleState.angularPosition,
        destination, settings);
    state.footprintU = globalHandleTangentToExterior(
        state.handleFrame.forward, state.handleState.angularPosition,
        destination, settings);
    state.footprintV = globalHandleTangentToExterior(
        state.handleFrame.up, state.handleState.angularPosition,
        destination, settings);
    state.chart = 0U;
    state.lastMouth = destination;
    ++state.crossings;
}

inline bool globalHandleApplyThroatCrossing(
    PortalVector& position, PortalVector& direction, PortalVector* frameUp,
    std::uint32_t source, const IntrinsicEllisSettings& settings) noexcept {
    const auto centers = globalHandleCenters(settings);
    const std::uint32_t destination = 1U - source;
    const float mouth = std::max(settings.throatRadius, 1.0e-5F);
    const PortalVector sourceN = portalNormalize(position - centers[source]);
    const PortalVector destinationN = portalNormalize(
        globalHandleRotateAcross(sourceN * -1.0F, source, settings));
    const float epsilon = std::max(
        std::nextafter(mouth, std::numeric_limits<float>::infinity()) - mouth,
        2.0e-7F);
    position = centers[destination] + destinationN * (mouth + epsilon);
    direction = portalNormalize(globalHandleRotateAcross(
        direction, source, settings), destinationN);
    if (frameUp != nullptr) {
        *frameUp = portalNormalize(globalHandleRotateAcross(
            *frameUp, source, settings));
        *frameUp = portalOrthonormalUp(direction, *frameUp);
    }
    return portalFinite(position) && portalFinite(direction) &&
        (frameUp == nullptr || portalFinite(*frameUp));
}

[[nodiscard]] inline bool globalHandleDerivative(
    PortalVector position, PortalVector direction,
    const IntrinsicEllisSettings& settings,
    PortalVector& positionDerivative, PortalVector& directionDerivative,
    float& curvature) noexcept {
    const GlobalHandleMetricSample metric = evaluateGlobalHandleMetric(
        position, settings);
    if (!metric.finite) return false;
    const float inverseConformal = 1.0F /
        std::max(static_cast<float>(metric.conformal), 1.0e-5F);
    const PortalVector gradient = metric.logConformalGradient;
    curvature = portalLength(gradient);
    positionDerivative = direction * inverseConformal;
    directionDerivative = (gradient - direction *
        portalDot(direction, gradient)) * inverseConformal;
    return portalFinite(positionDerivative) &&
        portalFinite(directionDerivative);
}

[[nodiscard]] inline bool globalHandleOptimizedStep(
    PortalVector position, PortalVector direction, float requestedStep,
    const IntrinsicEllisSettings& settings,
    PortalVector& nextPosition, PortalVector& nextDirection,
    float& acceptedStep, float& curvature) noexcept {
    PortalVector dx1{};
    PortalVector dd1{};
    float c1{};
    if (!globalHandleDerivative(position, direction, settings,
                                dx1, dd1, c1)) return false;
    float step = requestedStep;
    const float minimumStep = std::clamp(
        settings.globalHandleTailScale * 0.05F, 0.00025F, 0.008F);
    for (std::uint32_t refinement = 0U; refinement < 8U; ++refinement) {
        const PortalVector p2 = position + dx1 * (step * 0.5F);
        const PortalVector d2 = portalNormalize(
            direction + dd1 * (step * 0.5F), direction);
        PortalVector dx2{};
        PortalVector dd2{};
        float c2{};
        if (!globalHandleDerivative(p2, d2, settings,
                                    dx2, dd2, c2)) return false;
        const PortalVector p3 = position + dx2 * (step * 0.5F);
        const PortalVector d3 = portalNormalize(
            direction + dd2 * (step * 0.5F), direction);
        PortalVector dx3{};
        PortalVector dd3{};
        float c3{};
        if (!globalHandleDerivative(p3, d3, settings,
                                    dx3, dd3, c3)) return false;
        const PortalVector p4 = position + dx3 * step;
        const PortalVector d4 = portalNormalize(
            direction + dd3 * step, direction);
        PortalVector dx4{};
        PortalVector dd4{};
        float c4{};
        if (!globalHandleDerivative(p4, d4, settings,
                                    dx4, dd4, c4)) return false;
        const PortalVector rkPosition = position +
            (dx1 + dx2 * 2.0F + dx3 * 2.0F + dx4) * (step / 6.0F);
        const PortalVector rkDirection = portalNormalize(
            direction +
                (dd1 + dd2 * 2.0F + dd3 * 2.0F + dd4) * (step / 6.0F),
            direction);
        const PortalVector midpointPosition = position + dx2 * step;
        const PortalVector midpointDirection = portalNormalize(
            direction + dd2 * step, direction);
        const float error = std::max(
            portalLength(rkPosition - midpointPosition),
            portalLength(rkDirection - midpointDirection));
        nextPosition = rkPosition;
        nextDirection = rkDirection;
        acceptedStep = step;
        curvature = std::max(std::max(c1, c2), std::max(c3, c4));
        if (error <= 2.0e-4F || step <= minimumStep) return true;
        step *= 0.5F;
    }
    return true;
}

inline void globalHandleIntegrateRay(
    GlobalHandleRayState& state, float properDistance,
    const IntrinsicEllisSettings& settings,
    std::uint32_t maximumSteps = 192U,
    bool optimized = true, bool highPrecision = false) noexcept {
    const auto centers = globalHandleCenters(settings);
    const float mouth = globalHandleMouthRadius(settings);
    const float halfLength = globalHandleHalfLength(settings);
    float remaining = std::max(properDistance, 0.0F);
    state.direction = portalNormalize(state.direction);
    if (portalLength(state.footprintU) <= 1.0e-6F) {
        state.footprintU = state.direction;
    }
    if (portalLength(state.footprintV) <= 1.0e-6F) {
        const PortalVector reference = std::abs(state.direction.y) < 0.92F
            ? PortalVector{0.0F, 1.0F, 0.0F}
            : PortalVector{1.0F, 0.0F, 0.0F};
        state.footprintV = portalOrthonormalUp(state.direction, reference);
    }
    const auto transportRotation = [](PortalVector value,
                                      PortalVector from,
                                      PortalVector to) noexcept {
        from = portalNormalize(from);
        to = portalNormalize(to, from);
        const PortalVector axis = portalCross(from, to);
        const float sine = portalLength(axis);
        const float cosine = std::clamp(portalDot(from, to), -1.0F, 1.0F);
        if (!(sine > 1.0e-7F)) return value;
        const PortalVector unitAxis = axis * (1.0F / sine);
        return value * cosine + portalCross(unitAxis, value) * sine +
            unitAxis * (portalDot(unitAxis, value) * (1.0F - cosine));
    };
    for (std::uint32_t step = 0U;
         step < maximumSteps && remaining > 1.0e-6F; ++step) {
        if (state.chart == 1U) {
            const float collarStep = halfLength *
                (highPrecision ? 0.002F : 0.010F);
            float interiorStep = std::min(
                remaining, std::max(std::min(
                    settings.throatRadius *
                        (highPrecision ? 0.006F : 0.018F),
                    collarStep), 2.5e-5F));
            const float radial = state.handleState.velocity.radial;
            std::uint32_t destination = 2U;
            if (radial > 1.0e-7F) {
                const float eventDistance =
                    (halfLength - state.handleState.properDepth) / radial;
                if (eventDistance >= 0.0F && eventDistance <= interiorStep) {
                    interiorStep = eventDistance;
                    destination = 0U;
                }
            } else if (radial < -1.0e-7F) {
                const float eventDistance =
                    (-halfLength - state.handleState.properDepth) / radial;
                if (eventDistance >= 0.0F && eventDistance <= interiorStep) {
                    interiorStep = eventDistance;
                    destination = 1U;
                }
            }
            if (!(interiorStep > 1.0e-8F)) {
                if (destination < 2U) {
                    state.handleState.properDepth = destination == 0U
                        ? halfLength : -halfLength;
                    globalHandleExitInterior(state, destination, settings);
                    continue;
                }
                state.finite = false;
                return;
            }
            globalHandleIntegrateInteriorStep(
                state.handleState, state.handleFrame, interiorStep, settings);
            remaining -= interiorStep;
            state.affineDistance += interiorStep;
            ++state.steps;
            // A near-critical ray can reverse radial sign inside one RK step.
            // In that case the pre-step linear event estimate checks the old
            // end and can miss the opposite collar. Clamp the first resolved
            // post-step crossing instead of allowing u to run outside the
            // finite handle chart on every later iteration.
            if (destination >= 2U) {
                if (state.handleState.properDepth >= halfLength) {
                    destination = 0U;
                } else if (state.handleState.properDepth <= -halfLength) {
                    destination = 1U;
                }
            }
            if (destination < 2U) {
                state.handleState.properDepth = destination == 0U
                    ? halfLength : -halfLength;
                globalHandleExitInterior(state, destination, settings);
            }
            if (!portalFinite(state.handleState.angularPosition) ||
                !ellisFinite(state.handleState.velocity)) {
                state.finite = false;
                return;
            }
            continue;
        }
        const GlobalHandleMetricSample metric = evaluateGlobalHandleMetric(
            state.origin, settings);
        if (!metric.finite) { state.finite = false; return; }
        const float startCurvature = portalLength(
            metric.logConformalGradient);
        const float optimizedMinimumStep = std::clamp(
            settings.globalHandleTailScale * 0.05F, 0.00025F, 0.008F);
        float requestedStep = std::min(remaining,
            highPrecision
                ? std::clamp(0.004F /
                                 (1.0F + startCurvature * 0.20F),
                             0.0005F, 0.004F)
            : optimized
                ? std::clamp(0.32F / (1.0F + startCurvature * 0.45F),
                             optimizedMinimumStep, 0.32F)
                : std::clamp(0.035F / (1.0F + startCurvature * 1.5F),
                             0.0015F, 0.035F));
        for (std::uint32_t endpoint = 0U; endpoint < 2U; ++endpoint) {
            if (state.lastMouth == endpoint &&
                portalLength(state.origin - centers[endpoint]) <
                    mouth * 1.10F) continue;
            const float surfaceDistance = std::abs(
                portalLength(state.origin - centers[endpoint]) - mouth);
            requestedStep = std::min(requestedStep,
                std::max(surfaceDistance * 0.45F,
                         highPrecision ? 0.00025F : 0.0015F));
        }
        float properStep = requestedStep;
        float acceptedCurvature = startCurvature;
        PortalVector candidate{};
        PortalVector candidateDirection{};
        if (optimized || highPrecision) {
            if (!globalHandleOptimizedStep(
                    state.origin, state.direction, requestedStep, settings,
                    candidate, candidateDirection, properStep,
                    acceptedCurvature)) {
                state.finite = false;
                return;
            }
        } else {
            const float euclideanStep = properStep /
                std::max(static_cast<float>(metric.conformal), 1.0e-5F);
            const PortalVector gradient = metric.logConformalGradient;
            const PortalVector bend = gradient - state.direction *
                portalDot(state.direction, gradient);
            const PortalVector midpointDirection = portalNormalize(
                state.direction + bend * (0.5F * euclideanStep),
                state.direction);
            candidate = state.origin + midpointDirection * euclideanStep;
            candidateDirection = portalNormalize(
                state.direction + bend * euclideanStep, state.direction);
        }

        float bestFraction = 2.0F;
        std::uint32_t source = 2U;
        for (std::uint32_t mouthIndex = 0U; mouthIndex < 2U; ++mouthIndex) {
            if (state.lastMouth == mouthIndex &&
                portalLength(state.origin - centers[mouthIndex]) <
                    mouth * 1.001F) continue;
            float fraction = 0.0F;
            if (globalHandleSegmentSphereEntry(
                    state.origin, candidate, centers[mouthIndex], mouth,
                    fraction) && fraction < bestFraction) {
                bestFraction = fraction;
                source = mouthIndex;
            }
        }
        if (source < 2U) {
            state.origin = state.origin +
                (candidate - state.origin) * bestFraction;
            const PortalVector entryDirection = portalNormalize(
                state.direction +
                    (candidateDirection - state.direction) * bestFraction,
                state.direction);
            state.footprintU = transportRotation(
                state.footprintU, state.direction, entryDirection);
            state.footprintV = transportRotation(
                state.footprintV, state.direction, entryDirection);
            state.direction = entryDirection;
            globalHandleEnterInterior(state, source, settings);
            const float consumedProperStep = properStep * bestFraction;
            remaining -= consumedProperStep;
            state.affineDistance += consumedProperStep;
            ++state.steps;
            // The overlap event changes coordinates only. Re-evaluate the
            // residual proper distance in the handle chart; no exterior
            // position, scene owner, or radiance family is swapped here.
            continue;
        } else {
            state.origin = candidate;
            const PortalVector nextDirection = candidateDirection;
            state.footprintU = transportRotation(
                state.footprintU, state.direction, nextDirection);
            state.footprintV = transportRotation(
                state.footprintV, state.direction, nextDirection);
            state.direction = nextDirection;
            // Keep the destination chart's half-open owner through the
            // unconsumed crossing step and the first post-crossing frame.
            // Clearing it unconditionally here made the CPU packet ownerless
            // immediately after a body crossing, while pre-crossing optical
            // rays still carried the destination owner. Near-throat screen
            // rays could then re-enter the sphere and swap the whole visible
            // content family in one frame. Release only after leaving the
            // small overlap neighborhood where the coordinate event can be
            // encountered again through floating-point roundoff.
            if (state.lastMouth < 2U &&
                portalLength(state.origin - centers[state.lastMouth]) >
                    mouth * 1.10F) {
                state.lastMouth = 2U;
            }
        }
        remaining -= properStep;
        state.affineDistance += properStep;
        ++state.steps;
    }
    state.finite = state.finite && remaining <= 1.0e-4F &&
        portalFinite(state.origin) && portalFinite(state.direction);
}

inline void globalHandleAdvanceObserver(
    GlobalHandleObserverState& state, PortalVector localDirection,
    float properDistance, const IntrinsicEllisSettings& settings) noexcept {
    PortalVector direction = portalNormalize(localDirection, state.forward);
    GlobalHandleRayState ray{};
    ray.origin = state.position;
    ray.direction = direction;
    ray.footprintU = state.forward;
    ray.footprintV = state.up;
    ray.lastMouth = state.lastMouth;
    ray.chart = state.chart;
    ray.handleState = state.handleState;
    ray.handleFrame = state.handleFrame;
    if (state.chart == 1U) {
        const PortalVector right = portalNormalize(portalCross(
            state.forward, state.up), {1.0F, 0.0F, 0.0F});
        const float forwardAmount = portalDot(direction, state.forward);
        const float rightAmount = portalDot(direction, right);
        const float upAmount = portalDot(direction, state.up);
        ray.handleState.velocity = ellisNormalize(
            state.handleFrame.forward * forwardAmount +
                state.handleFrame.right * rightAmount +
                state.handleFrame.up * upAmount,
            state.handleState.angularPosition,
            state.handleState.velocity);
    }
    const std::uint32_t cameraBudget = std::clamp(
        static_cast<std::uint32_t>(std::ceil(
            properDistance /
            std::max(settings.throatRadius * 0.012F, 2.0e-4F))) + 64U,
        256U, 4096U);
    globalHandleIntegrateRay(ray, properDistance, settings, cameraBudget);
    if (!ray.finite) { state.finite = false; return; }
    state.position = ray.origin;
    state.forward = portalNormalize(ray.footprintU, state.forward);
    state.up = portalOrthonormalUp(state.forward, ray.footprintV);
    state.velocity = ray.direction * state.properSpeed;
    state.lastMouth = ray.lastMouth;
    state.chart = ray.chart;
    state.handleState = ray.handleState;
    state.handleFrame = ray.handleFrame;
    if (state.chart == 1U) {
        state.forward = portalNormalize(ellisTangentToEmbedded(
            state.handleFrame.forward,
            state.handleState.angularPosition));
        state.up = portalOrthonormalUp(state.forward,
            ellisTangentToEmbedded(state.handleFrame.up,
                                   state.handleState.angularPosition));
        state.velocity = portalNormalize(ellisTangentToEmbedded(
            state.handleState.velocity,
            state.handleState.angularPosition)) * state.properSpeed;
    }
    state.crossings += ray.crossings;
    state.affineDistance += ray.affineDistance;
    state.finite = portalFinite(state.position) && portalFinite(state.forward) &&
        portalFinite(state.up) && portalFinite(state.velocity) &&
        portalDot(portalCross(portalNormalize(portalCross(state.forward,
            state.up)), state.forward), state.up) > 0.99F;
}

[[nodiscard]] inline GlobalHandleObserverState globalHandleResetObserver(
    const IntrinsicEllisSettings& settings,
    std::uint32_t mouthIndex = 0U) noexcept {
    const auto centers = globalHandleCenters(settings);
    const std::uint32_t source = std::min(mouthIndex, 1U);
    const PortalVector outward = portalNormalize(
        centers[source], source == 0U ? PortalVector{0.0F, 0.0F, 1.0F}
                                     : PortalVector{0.0F, 0.0F, -1.0F});
    const float distance = settings.contentSphereRadius +
        // Keep preset comparisons at a useful world-space viewing distance.
        // Scaling the reset standoff with the mouth made every preset occupy
        // the same angle and concealed the compact preset's physical benefit.
        std::max(settings.contentSphereRadius * 1.35F, 0.46F);
    GlobalHandleObserverState result{};
    result.position = centers[source] + outward * distance;
    result.forward = outward * -1.0F;
    const PortalVector reference = std::abs(result.forward.y) < 0.92F
        ? PortalVector{0.0F, 1.0F, 0.0F}
        : PortalVector{1.0F, 0.0F, 0.0F};
    result.up = portalOrthonormalUp(result.forward, reference);
    result.lastMouth = 2U;
    return result;
}

} // namespace voxel
