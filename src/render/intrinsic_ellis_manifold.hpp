#pragma once

#include "render/portal_gr_precompute.hpp"
#include "render/portal_lab.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace voxel {

// Coordinate-free representation of the Ellis spatial metric
//
//     ds^2 = dl^2 + (l^2 + a^2) dOmega^2.
//
// The angular position is a unit vector rather than theta/phi. Tangent vectors
// are stored as a radial component plus a physical (orthonormal) tangent vector
// in R^3. This is regular at the angular poles and at the throat l == 0.
struct EllisTangent {
    float radial{};
    PortalVector angular{};
};

struct EllisState {
    float properDepth{};
    PortalVector angularPosition{0.0F, 0.0F, 1.0F};
    EllisTangent velocity{-1.0F, {}};
};

struct EllisFrame {
    EllisTangent right{};
    EllisTangent up{};
    EllisTangent forward{-1.0F, {}};
};

struct EllisRadialProfile {
    float radius{};
    float firstDerivative{};
    float secondDerivative{};
};

// Physically scaled all-space Ellis radial profile.  Curvature decays
// analytically as |l| grows; there is no compact collar or influence cutoff.
// `smoothGlobalHandle` remains in the signature to keep the isolated legacy
// lab call sites source-compatible while the global lab transitions.
[[nodiscard]] inline EllisRadialProfile smoothEllisRadialProfile(
    float signedProperDepth, float throatRadius,
    bool /*smoothGlobalHandle*/ = true) noexcept {
    const float a = std::max(throatRadius, 1.0e-5F);
    const float radius = std::sqrt(
        signedProperDepth * signedProperDepth + a * a);
    return {radius, signedProperDepth / radius,
            a * a / std::max(radius * radius * radius, 1.0e-12F)};
}

struct IntrinsicEllisSettings {
    bool enabled{true};
    // Used only by the isolated global-metric lab. The older intrinsic lab
    // keeps its original all-Ellis observer integration for comparison.
    bool globalMetricField{};
    // The accepted global-lab path keeps the observer and every screen ray in
    // native signed-l Ellis coordinates.  The older shared-exterior spherical
    // mouth remap remains available only for regression/comparison because its
    // tangent aperture is a real binary policy boundary, not a smooth metric
    // chart transition.
    bool globalNativeEllisPath{true};
    float throatRadius{kPortalGrThroatRatio};
    float contentExitProperDepth{
        std::sqrt(1.0F - kPortalGrThroatRatio * kPortalGrThroatRatio)};
    float contentSphereRadius{0.34F};
    float endpointAAzimuth{};
    float endpointAElevation{0.18F};
    float endpointADistance{1.58F};
    float endpointBAzimuth{2.15F};
    float endpointBElevation{0.28F};
    float endpointBDistance{1.58F};
    float endBYaw{};
    float endBPitch{};
    float endBRoll{};
    float freeFlySpeed{0.34F};
    float sprintMultiplier{3.0F};
    float mouseSensitivity{0.0035F};
    // Isolated global-handle lab parameters/state.  These are ignored by the
    // intrinsic comparison lab and f512 production.
    float globalHandleTailScale{1.75F};
    float globalHandleMetricStrength{0.72F};
    // Global-lab-only deterministic geodesic-Jacobian spatial AA.  The
    // center ray is the classifier; only rapid-distortion/silhouette pixels
    // evaluate the rotated four-sample coverage pattern.
    bool globalSpatialAaEnabled{true};
    float globalSpatialAaDistortionThreshold{6.00F};
    float globalSpatialAaMagnificationThreshold{12.0F};
    PortalVector globalPosition{};
    PortalVector globalForward{0.0F, 0.0F, -1.0F};
    PortalVector globalUp{0.0F, 1.0F, 0.0F};
    PortalVector globalVelocity{};
    std::uint32_t globalLastMouth{2U};
    std::uint32_t globalCrossings{};
    float globalAffineDistance{};
    std::uint32_t integrationQuality{1U};
    std::uint32_t debugMode{};
    EllisState cameraState{};
    EllisFrame cameraFrame{};
};

struct IntrinsicEllisContentPose {
    PortalVector position{};
    PortalVector forward{0.0F, 0.0F, -1.0F};
    PortalVector up{0.0F, 1.0F, 0.0F};
    float radialDistance{};
    bool positiveEnd{true};
    bool finite{};
};

[[nodiscard]] inline PortalVector operator-(PortalVector value) noexcept {
    return {-value.x, -value.y, -value.z};
}

[[nodiscard]] inline EllisTangent operator+(EllisTangent a,
                                             EllisTangent b) noexcept {
    return {a.radial + b.radial, a.angular + b.angular};
}

[[nodiscard]] inline EllisTangent operator-(EllisTangent a,
                                             EllisTangent b) noexcept {
    return {a.radial - b.radial, a.angular - b.angular};
}

[[nodiscard]] inline EllisTangent operator*(EllisTangent value,
                                             float scale) noexcept {
    return {value.radial * scale, value.angular * scale};
}

[[nodiscard]] inline float ellisDot(const EllisTangent& a,
                                    const EllisTangent& b) noexcept {
    return a.radial * b.radial + portalDot(a.angular, b.angular);
}

[[nodiscard]] inline float ellisLength(const EllisTangent& value) noexcept {
    return std::sqrt(std::max(ellisDot(value, value), 0.0F));
}

[[nodiscard]] inline bool ellisFinite(const EllisTangent& value) noexcept {
    return std::isfinite(value.radial) && portalFinite(value.angular);
}

[[nodiscard]] inline EllisTangent ellisProjectTangent(
    EllisTangent value, PortalVector angularPosition) noexcept {
    value.angular = value.angular - angularPosition *
        portalDot(value.angular, angularPosition);
    return value;
}

[[nodiscard]] inline EllisTangent ellisNormalize(
    EllisTangent value, PortalVector angularPosition,
    EllisTangent fallback = {-1.0F, {}}) noexcept {
    value = ellisProjectTangent(value, angularPosition);
    const float length = ellisLength(value);
    if (!(length > 1.0e-7F) || !std::isfinite(length)) {
        return ellisProjectTangent(fallback, angularPosition);
    }
    return value * (1.0F / length);
}

[[nodiscard]] inline std::array<PortalVector, 2> ellisAngularBasis(
    PortalVector angularPosition) noexcept {
    const PortalVector n = portalNormalize(angularPosition);
    // Pick the least aligned Cartesian axis. This remains well-conditioned at
    // both conventional spherical-coordinate poles.
    const std::array<PortalVector, 3> axes{{{1.0F, 0.0F, 0.0F},
                                            {0.0F, 1.0F, 0.0F},
                                            {0.0F, 0.0F, 1.0F}}};
    std::size_t reference = 0U;
    float alignment = std::abs(portalDot(n, axes[0]));
    for (std::size_t index = 1U; index < axes.size(); ++index) {
        const float candidate = std::abs(portalDot(n, axes[index]));
        if (candidate < alignment) {
            alignment = candidate;
            reference = index;
        }
    }
    const PortalVector x = portalNormalize(portalCross(axes[reference], n),
                                            {1.0F, 0.0F, 0.0F});
    return {x, portalNormalize(portalCross(n, x), {0.0F, 1.0F, 0.0F})};
}

[[nodiscard]] inline PortalVector intrinsicEllisRotateEndB(
    PortalVector value, const IntrinsicEllisSettings& settings) noexcept {
    const float cy = std::cos(settings.endBYaw);
    const float sy = std::sin(settings.endBYaw);
    const float cp = std::cos(settings.endBPitch);
    const float sp = std::sin(settings.endBPitch);
    const float cr = std::cos(settings.endBRoll);
    const float sr = std::sin(settings.endBRoll);
    const PortalVector yawed{cy * value.x + sy * value.z, value.y,
                             -sy * value.x + cy * value.z};
    const PortalVector pitched{yawed.x,
                               cp * yawed.y - sp * yawed.z,
                               sp * yawed.y + cp * yawed.z};
    return {cr * pitched.x - sr * pitched.y,
            sr * pitched.x + cr * pitched.y, pitched.z};
}

[[nodiscard]] inline PortalVector intrinsicEllisRotateEndBInverse(
    PortalVector value, const IntrinsicEllisSettings& settings) noexcept {
    const float cy = std::cos(settings.endBYaw);
    const float sy = std::sin(settings.endBYaw);
    const float cp = std::cos(settings.endBPitch);
    const float sp = std::sin(settings.endBPitch);
    const float cr = std::cos(settings.endBRoll);
    const float sr = std::sin(settings.endBRoll);
    const PortalVector unrolled{cr * value.x + sr * value.y,
                                -sr * value.x + cr * value.y, value.z};
    const PortalVector unpitched{unrolled.x,
                                 cp * unrolled.y + sp * unrolled.z,
                                 -sp * unrolled.y + cp * unrolled.z};
    return {cy * unpitched.x - sy * unpitched.z, unpitched.y,
            sy * unpitched.x + cy * unpitched.z};
}

[[nodiscard]] inline PortalVector intrinsicEllisContentCenter(
    const IntrinsicEllisSettings& settings, bool positiveEnd,
    float planetRadius = 1.0F) noexcept {
    return portalEndpoint(
        positiveEnd ? settings.endpointAAzimuth : settings.endpointBAzimuth,
        positiveEnd ? settings.endpointAElevation : settings.endpointBElevation,
        positiveEnd ? settings.endpointADistance : settings.endpointBDistance) *
        planetRadius;
}

[[nodiscard]] inline PortalVector ellisTangentToLocal(
    const EllisTangent& tangent, PortalVector angularPosition) noexcept {
    const auto basis = ellisAngularBasis(angularPosition);
    return {tangent.radial, portalDot(tangent.angular, basis[0]),
            portalDot(tangent.angular, basis[1])};
}

[[nodiscard]] inline PortalVector intrinsicEllisTangentToContent(
    const IntrinsicEllisSettings& settings, const EllisState& state,
    const EllisTangent& tangent) noexcept {
    const bool positive = state.properDepth >= 0.0F;
    const PortalVector n = portalNormalize(state.angularPosition);
    const auto basis = ellisAngularBasis(n);
    const PortalVector local = ellisTangentToLocal(tangent, n);
    PortalVector mapped = n * (positive ? local.x : -local.x) +
        basis[0] * local.y + basis[1] * local.z;
    return positive ? mapped : intrinsicEllisRotateEndB(mapped, settings);
}

[[nodiscard]] inline EllisTangent intrinsicEllisTangentFromContent(
    const IntrinsicEllisSettings& settings, bool positiveEnd,
    PortalVector intrinsicN, PortalVector contentVector) noexcept {
    const PortalVector localContent = positiveEnd
        ? contentVector
        : intrinsicEllisRotateEndBInverse(contentVector, settings);
    const PortalVector n = portalNormalize(intrinsicN);
    const float contentRadial = portalDot(localContent, n);
    return ellisProjectTangent(
        {positiveEnd ? contentRadial : -contentRadial,
         localContent - n * contentRadial}, n);
}

[[nodiscard]] inline IntrinsicEllisContentPose intrinsicEllisContentPose(
    const IntrinsicEllisSettings& settings, const EllisState& camera,
    const EllisFrame& frame, float planetRadius) noexcept {
    const bool positive = camera.properDepth >= 0.0F;
    const PortalVector n = portalNormalize(camera.angularPosition);
    const PortalVector contentN = positive
        ? n : intrinsicEllisRotateEndB(n, settings);
    const PortalVector center = intrinsicEllisContentCenter(
        settings, positive, planetRadius);
    IntrinsicEllisContentPose result{};
    if (settings.globalMetricField) {
        result.radialDistance = smoothEllisRadialProfile(
            camera.properDepth, settings.throatRadius, true).radius *
            planetRadius;
    } else {
        const float continuation = std::max(
            std::abs(camera.properDepth) - settings.contentExitProperDepth,
            0.0F);
        result.radialDistance =
            (settings.contentSphereRadius + continuation) * planetRadius;
    }
    result.position = center + contentN * result.radialDistance;
    result.forward = portalNormalize(intrinsicEllisTangentToContent(
        settings, camera, frame.forward));
    result.up = portalNormalize(intrinsicEllisTangentToContent(
        settings, camera, frame.up));
    result.positiveEnd = positive;
    result.finite = portalFinite(result.position) &&
        portalFinite(result.forward) && portalFinite(result.up) &&
        std::isfinite(result.radialDistance);
    return result;
}

struct IntrinsicEllisMouthEntry {
    bool entered{};
    bool positiveEnd{};
    float segmentFraction{};
    PortalVector intrinsicAngular{0.0F, 0.0F, 1.0F};
};

[[nodiscard]] inline IntrinsicEllisMouthEntry intrinsicEllisOtherMouthEntry(
    const IntrinsicEllisSettings& settings, const EllisState& before,
    const EllisFrame& beforeFrame, const EllisState& after,
    const EllisFrame& afterFrame) noexcept {
    IntrinsicEllisMouthEntry result{};
    const float exitL = settings.contentExitProperDepth;
    if (!(std::abs(before.properDepth) > exitL) ||
        !(std::abs(after.properDepth) > exitL) ||
        ((before.properDepth >= 0.0F) != (after.properDepth >= 0.0F))) {
        return result;
    }
    const bool currentPositive = before.properDepth >= 0.0F;
    const bool targetPositive = !currentPositive;
    const IntrinsicEllisContentPose start = intrinsicEllisContentPose(
        settings, before, beforeFrame, 1.0F);
    const IntrinsicEllisContentPose finish = intrinsicEllisContentPose(
        settings, after, afterFrame, 1.0F);
    if (!start.finite || !finish.finite) {
        return result;
    }
    const PortalVector center = intrinsicEllisContentCenter(
        settings, targetPositive, 1.0F);
    const float radius = std::max(settings.contentSphereRadius, 1.0e-5F);
    const PortalVector segment = finish.position - start.position;
    const PortalVector relative = start.position - center;
    const float aa = portalDot(segment, segment);
    const float bb = 2.0F * portalDot(relative, segment);
    const float cc = portalDot(relative, relative) - radius * radius;
    const float discriminant = bb * bb - 4.0F * aa * cc;
    if (!(aa > 1.0e-12F) || discriminant < 0.0F) {
        return result;
    }
    const float root = std::sqrt(std::max(discriminant, 0.0F));
    const float fraction = (-bb - root) / (2.0F * aa);
    if (!(fraction >= 0.0F && fraction <= 1.0F)) {
        return result;
    }
    const PortalVector contentN = portalNormalize(
        start.position + segment * fraction - center);
    result.entered = true;
    result.positiveEnd = targetPositive;
    result.segmentFraction = fraction;
    result.intrinsicAngular = targetPositive
        ? contentN : portalNormalize(
            intrinsicEllisRotateEndBInverse(contentN, settings));
    return result;
}

[[nodiscard]] inline EllisTangent ellisTangentFromLocal(
    PortalVector local, PortalVector angularPosition) noexcept {
    const auto basis = ellisAngularBasis(angularPosition);
    return ellisProjectTangent(
        {local.x, basis[0] * local.y + basis[1] * local.z},
        portalNormalize(angularPosition));
}

struct EllisDerivative {
    float properDepth{};
    PortalVector angularPosition{};
    EllisTangent velocity{};
};

[[nodiscard]] inline EllisDerivative ellisGeodesicDerivative(
    const EllisState& state, float throatRadius,
    bool smoothGlobalHandle = false) noexcept {
    const EllisRadialProfile profile = smoothEllisRadialProfile(
        state.properDepth, throatRadius, smoothGlobalHandle);
    const float radius = std::max(profile.radius, 1.0e-6F);
    const float expansion = profile.firstDerivative / radius;
    const float angularSpeed2 = portalDot(state.velocity.angular,
                                          state.velocity.angular);
    return {
        state.velocity.radial,
        state.velocity.angular * (1.0F / radius),
        {expansion * angularSpeed2,
         state.velocity.angular *
             (-expansion * state.velocity.radial) -
             state.angularPosition * (angularSpeed2 / radius)}};
}

[[nodiscard]] inline EllisTangent ellisParallelDerivative(
    const EllisState& path, const EllisTangent& transported,
    float throatRadius, bool smoothGlobalHandle = false) noexcept {
    const EllisRadialProfile profile = smoothEllisRadialProfile(
        path.properDepth, throatRadius, smoothGlobalHandle);
    const float radius = std::max(profile.radius, 1.0e-6F);
    const float expansion = profile.firstDerivative / radius;
    const float coupling = portalDot(path.velocity.angular,
                                     transported.angular);
    return {
        expansion * coupling,
        transported.angular *
            (-expansion * path.velocity.radial) -
        path.velocity.angular *
            (expansion * transported.radial) -
        path.angularPosition * (coupling / radius)};
}

[[nodiscard]] inline EllisState ellisAddDerivative(
    const EllisState& state, const EllisDerivative& derivative,
    float scale) noexcept {
    EllisState result{
        state.properDepth + derivative.properDepth * scale,
        state.angularPosition + derivative.angularPosition * scale,
        state.velocity + derivative.velocity * scale};
    result.angularPosition = portalNormalize(result.angularPosition,
                                              state.angularPosition);
    result.velocity = ellisNormalize(result.velocity, result.angularPosition,
                                     state.velocity);
    return result;
}

inline void ellisIntegrateGeodesic(EllisState& state, EllisFrame& frame,
                                   float distance, float throatRadius,
                                   std::uint32_t maximumSubsteps = 64U,
                                   bool smoothGlobalHandle = false) noexcept {
    if (!(std::abs(distance) > 0.0F) || !std::isfinite(distance)) {
        return;
    }
    const float safeThroat = std::max(throatRadius, 1.0e-4F);
    const float maximumStep = safeThroat * 0.025F;
    const std::uint32_t substeps = std::clamp(
        static_cast<std::uint32_t>(std::ceil(std::abs(distance) /
                                             maximumStep)), 1U,
        std::max(maximumSubsteps, 1U));
    const float h = distance / static_cast<float>(substeps);
    for (std::uint32_t step = 0U; step < substeps; ++step) {
        const EllisState initial = state;
        const EllisFrame initialFrame = frame;
        const EllisDerivative k1 = ellisGeodesicDerivative(
            initial, safeThroat, smoothGlobalHandle);
        const EllisState s2 = ellisAddDerivative(initial, k1, h * 0.5F);
        const EllisDerivative k2 = ellisGeodesicDerivative(
            s2, safeThroat, smoothGlobalHandle);
        const EllisState s3 = ellisAddDerivative(initial, k2, h * 0.5F);
        const EllisDerivative k3 = ellisGeodesicDerivative(
            s3, safeThroat, smoothGlobalHandle);
        const EllisState s4 = ellisAddDerivative(initial, k3, h);
        const EllisDerivative k4 = ellisGeodesicDerivative(
            s4, safeThroat, smoothGlobalHandle);
        state.properDepth += h / 6.0F *
            (k1.properDepth + 2.0F * k2.properDepth +
             2.0F * k3.properDepth + k4.properDepth);
        state.angularPosition = portalNormalize(
            state.angularPosition + (k1.angularPosition +
                k2.angularPosition * 2.0F + k3.angularPosition * 2.0F +
                k4.angularPosition) * (h / 6.0F),
            initial.angularPosition);
        state.velocity = ellisNormalize(
            state.velocity + (k1.velocity + k2.velocity * 2.0F +
                k3.velocity * 2.0F + k4.velocity) * (h / 6.0F),
            state.angularPosition, initial.velocity);

        const auto transport = [&](EllisTangent value) noexcept {
            const EllisTangent p1 = ellisParallelDerivative(
                initial, value, safeThroat, smoothGlobalHandle);
            const EllisTangent p2 = ellisParallelDerivative(
                s2, value + p1 * (h * 0.5F), safeThroat,
                smoothGlobalHandle);
            const EllisTangent p3 = ellisParallelDerivative(
                s3, value + p2 * (h * 0.5F), safeThroat,
                smoothGlobalHandle);
            const EllisTangent p4 = ellisParallelDerivative(
                s4, value + p3 * h, safeThroat, smoothGlobalHandle);
            return ellisProjectTangent(
                value + (p1 + p2 * 2.0F + p3 * 2.0F + p4) * (h / 6.0F),
                state.angularPosition);
        };
        frame.right = transport(initialFrame.right);
        frame.up = transport(initialFrame.up);
        frame.forward = transport(initialFrame.forward);

        // A light Gram-Schmidt correction only removes numerical drift. The
        // transported frame is never rebuilt from a Euclidean world basis.
        frame.forward = ellisNormalize(frame.forward, state.angularPosition,
                                       state.velocity);
        frame.right = ellisNormalize(
            frame.right - frame.forward * ellisDot(frame.right, frame.forward),
            state.angularPosition, initialFrame.right);
        frame.up = ellisNormalize(
            frame.up - frame.forward * ellisDot(frame.up, frame.forward) -
            frame.right * ellisDot(frame.up, frame.right),
            state.angularPosition, initialFrame.up);
    }
}

[[nodiscard]] inline float ellisFrameHandedness(
    const EllisFrame& frame, PortalVector angularPosition) noexcept {
    const PortalVector right = ellisTangentToLocal(frame.right,
                                                   angularPosition);
    const PortalVector up = ellisTangentToLocal(frame.up, angularPosition);
    const PortalVector forward = ellisTangentToLocal(frame.forward,
                                                     angularPosition);
    return portalDot(portalCross(right, up), -forward);
}

struct alignas(16) IntrinsicEllisGpuParameters {
    std::array<float, 4> camera{};       // l, throat a, exit |l|, enabled
    std::array<float, 4> angular{};      // n.xyz, content sphere radius
    std::array<float, 4> forward{};      // radial, local tangent x/y, quality
    std::array<float, 4> up{};           // radial, local tangent x/y, debug
    std::array<float, 4> endpointA{};    // normalized engine world, unused w
    std::array<float, 4> endpointB{};    // normalized engine world, unused w
    std::array<float, 4> endBRotation{}; // yaw, pitch, roll, frame handedness
    std::array<float, 4> control{};      // speed, table status, reserved
};
static_assert(sizeof(IntrinsicEllisGpuParameters) == 128U);

struct alignas(16) IntrinsicEllisGpuBuffer {
    IntrinsicEllisGpuParameters parameters{};
    PortalGrPrecomputedTable gr{};
    std::array<std::uint32_t, 4> telemetry{};
};
static_assert(sizeof(IntrinsicEllisGpuBuffer) == sizeof(PortalLabGpuBuffer));

[[nodiscard]] inline IntrinsicEllisGpuParameters intrinsicEllisGpuParameters(
    const IntrinsicEllisSettings& settings, const EllisState& camera,
    const EllisFrame& frame) noexcept {
    const PortalVector localForward = ellisTangentToLocal(
        frame.forward, camera.angularPosition);
    const PortalVector localUp = ellisTangentToLocal(
        frame.up, camera.angularPosition);
    const PortalVector endpointA = portalEndpoint(
        settings.endpointAAzimuth, settings.endpointAElevation,
        settings.endpointADistance);
    const PortalVector endpointB = portalEndpoint(
        settings.endpointBAzimuth, settings.endpointBElevation,
        settings.endpointBDistance);
    IntrinsicEllisGpuParameters result{};
    if (settings.globalMetricField && !settings.globalNativeEllisPath) {
        const PortalVector globalForward = portalNormalize(
            settings.globalForward, {0.0F, 0.0F, -1.0F});
        const PortalVector globalUp = portalOrthonormalUp(
            globalForward, settings.globalUp);
        result.camera = {settings.globalPosition.x, settings.globalPosition.y,
                         settings.globalPosition.z, settings.enabled ? 1.0F : 0.0F};
        // In the global lab the content standoff is already carried in
        // endpointA.w.  Reuse this otherwise-dead lane for the AA distortion
        // threshold without growing the fixed 128-byte parameter block.
        result.angular = {globalForward.x, globalForward.y, globalForward.z,
                          settings.globalSpatialAaDistortionThreshold};
        result.forward = {globalUp.x, globalUp.y, globalUp.z,
                          static_cast<float>(settings.integrationQuality)};
        result.up = {settings.globalVelocity.x, settings.globalVelocity.y,
                     settings.globalVelocity.z,
                     static_cast<float>(settings.debugMode)};
        result.endpointA = {endpointA.x, endpointA.y, endpointA.z,
                            settings.globalSpatialAaEnabled ? 1.0F : 0.0F};
        result.endpointB = {endpointB.x, endpointB.y, endpointB.z,
                            settings.throatRadius};
        result.endBRotation = {settings.endBYaw, settings.endBPitch,
                               settings.endBRoll,
                               settings.globalSpatialAaMagnificationThreshold};
        // The first two lanes carry the observer's half-open chart owner and
        // accumulated affine distance.  They make the first GPU ray after a
        // body crossing begin from exactly the same event-split state rather
        // than silently reverting to an ownerless/zero-affine wrapper.
        result.control = {static_cast<float>(settings.globalLastMouth),
                          settings.globalAffineDistance,
                          settings.globalHandleTailScale,
                          settings.globalHandleMetricStrength};
        return result;
    }
    result.camera = {camera.properDepth, settings.throatRadius,
                     settings.contentExitProperDepth,
                     settings.enabled ? 1.0F : 0.0F};
    result.angular = {camera.angularPosition.x, camera.angularPosition.y,
                      camera.angularPosition.z,
                      settings.globalMetricField
                          ? settings.globalSpatialAaDistortionThreshold
                          : settings.contentSphereRadius};
    result.forward = {localForward.x, localForward.y, localForward.z,
                      static_cast<float>(settings.integrationQuality)};
    result.up = {localUp.x, localUp.y, localUp.z,
                 static_cast<float>(settings.debugMode)};
    result.endpointA = {endpointA.x, endpointA.y, endpointA.z,
                        settings.globalMetricField &&
                            settings.globalSpatialAaEnabled ? 1.0F : 0.0F};
    result.endpointB = {endpointB.x, endpointB.y, endpointB.z,
                        settings.globalMetricField
                            ? settings.throatRadius : 0.0F};
    result.endBRotation = {settings.endBYaw, settings.endBPitch,
                           settings.endBRoll,
                           settings.globalMetricField
                               ? settings.globalSpatialAaMagnificationThreshold
                               : ellisFrameHandedness(
                                     frame, camera.angularPosition)};
    result.control = {settings.globalMetricField ? -1.0F
                                                  : settings.freeFlySpeed,
                      1.0F, settings.globalMetricField ? 1.0F : 0.0F, 0.0F};
    return result;
}

} // namespace voxel
