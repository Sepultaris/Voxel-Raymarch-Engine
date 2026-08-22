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
    bool finite{true};
};

[[nodiscard]] inline std::array<PortalVector, 2> globalHandleCenters(
    const IntrinsicEllisSettings& settings) noexcept {
    return {
        portalEndpoint(settings.endpointAAzimuth, settings.endpointAElevation,
                       settings.endpointADistance),
        portalEndpoint(settings.endpointBAzimuth, settings.endpointBElevation,
                       settings.endpointBDistance)};
}

[[nodiscard]] inline double globalHandleFourth(double value) noexcept {
    const double square = value * value;
    return square * square;
}

struct GlobalHandleScalarField {
    double value{};
    PortalVector gradient{};
    bool finite{true};
};

[[nodiscard]] inline GlobalHandleScalarField globalHandleLogConformal(
    PortalVector position, const IntrinsicEllisSettings& settings) noexcept {
    const auto centers = globalHandleCenters(settings);
    // The topology event is the physical Ellis throat.  contentSphereRadius
    // is only a camera/content standoff and must never define a projected
    // ray-ownership aperture.
    const double mouth = std::max(
        static_cast<double>(settings.throatRadius), 1.0e-5);
    const double throat = std::max(
        static_cast<double>(settings.throatRadius), 1.0e-5);
    const double tail = std::max(
        static_cast<double>(settings.globalHandleTailScale), mouth * 2.0);
    const double strength = std::clamp(
        static_cast<double>(settings.globalHandleMetricStrength), 0.0, 4.0);

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
        s[index] = std::abs(radius - mouth);
        s4[index] = globalHandleFourth(s[index]);
        const double denominator = s[index] * s[index] + throat * throat;
        local[index] = strength * throat * throat / denominator;
        localDerivative[index] = -2.0 * strength * throat * throat * s[index] /
            std::max(denominator * denominator, 1.0e-24);
        if (radius < mouth) {
            localDerivative[index] = -localDerivative[index];
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
    for (std::uint32_t refinement = 0U; refinement < 4U; ++refinement) {
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
        if (error <= 2.0e-4F || step <= 0.008F) return true;
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
    const float mouth = std::max(settings.throatRadius, 1.0e-5F);
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
        const GlobalHandleMetricSample metric = evaluateGlobalHandleMetric(
            state.origin, settings);
        if (!metric.finite) { state.finite = false; return; }
        const float startCurvature = portalLength(
            metric.logConformalGradient);
        float requestedStep = std::min(remaining,
            highPrecision
                ? std::clamp(0.004F /
                                 (1.0F + startCurvature * 0.20F),
                             0.0005F, 0.004F)
            : optimized
                ? std::clamp(0.32F / (1.0F + startCurvature * 0.45F),
                             0.008F, 0.32F)
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
            if (!globalHandleApplyThroatCrossing(
                    state.origin, state.direction, nullptr, source, settings)) {
                state.finite = false;
                return;
            }
            state.footprintU = globalHandleRotateAcross(
                state.footprintU, source, settings);
            state.footprintV = globalHandleRotateAcross(
                state.footprintV, source, settings);
            state.lastMouth = 1U - source;
            ++state.crossings;
            const float consumedProperStep = properStep * bestFraction;
            remaining -= consumedProperStep;
            state.affineDistance += consumedProperStep;
            ++state.steps;
            // The throat event divides, rather than discards, this solver
            // step. Re-evaluate the unconsumed distance in the destination
            // chart so camera pose, frame transport and optical affine depth
            // remain continuous at the coordinate remap.
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
    globalHandleIntegrateRay(ray, properDistance, settings, 256U);
    if (!ray.finite) { state.finite = false; return; }
    state.position = ray.origin;
    state.forward = portalNormalize(ray.footprintU, state.forward);
    state.up = portalOrthonormalUp(state.forward, ray.footprintV);
    state.velocity = ray.direction * state.properSpeed;
    state.lastMouth = ray.lastMouth;
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
        std::max(settings.contentSphereRadius * 1.35F, 0.12F);
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
