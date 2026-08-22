#pragma once

#include "render/portal_gr_precompute.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace voxel {

struct PortalVector {
    float x{};
    float y{};
    float z{};
};

[[nodiscard]] inline PortalVector operator+(PortalVector a, PortalVector b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
[[nodiscard]] inline PortalVector operator-(PortalVector a, PortalVector b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
[[nodiscard]] inline PortalVector operator*(PortalVector value, float scale) noexcept {
    return {value.x * scale, value.y * scale, value.z * scale};
}
[[nodiscard]] inline float portalDot(PortalVector a, PortalVector b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
[[nodiscard]] inline PortalVector portalCross(PortalVector a, PortalVector b) noexcept {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}
[[nodiscard]] inline float portalLength(PortalVector value) noexcept {
    return std::sqrt(std::max(portalDot(value, value), 0.0F));
}
[[nodiscard]] inline bool portalFinite(PortalVector value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}
[[nodiscard]] inline PortalVector portalNormalize(
    PortalVector value, PortalVector fallback = {0.0F, 0.0F, 1.0F}) noexcept {
    const float length = portalLength(value);
    return length > 1.0e-7F && std::isfinite(length)
        ? value * (1.0F / length) : fallback;
}

struct PortalLabSettings {
    bool enabled{true};
    bool cameraTraversalEnabled{true};
    float endpointAAzimuth{};
    float endpointAElevation{0.16F};
    float endpointADistance{1.58F};
    float endpointBAzimuth{2.15F};
    float endpointBElevation{0.28F};
    float endpointBDistance{1.58F};
    float influenceRadius{0.30F};
    float throatRadius{0.105F};
    float lensStrength{1.15F};
    float massBias{0.08F};
    float linkYaw{};
    float linkPitch{};
    float linkRoll{};
    std::uint32_t quality{1U};
    std::uint32_t maximumTraversals{2U};
    std::uint32_t debugMode{};
    PortalVector freeFlyPosition{0.0F, 0.0F, 3.0F};
    PortalVector freeFlyForward{0.0F, 0.0F, -1.0F};
    PortalVector freeFlyUp{0.0F, 1.0F, 0.0F};
    // Transient observer state shared with the ray path. Once the physical
    // camera has crossed the throat, rays are continued through the remaining
    // destination half of the same chart instead of abruptly becoming plain
    // Euclidean rays at the throat.
    bool observerManifoldActive{};
    bool observerDestinationSide{};
};

struct alignas(16) PortalLabGpuParameters {
    std::array<float, 4> endpointA{};
    std::array<float, 4> endpointB{};
    std::array<float, 4> optical{};
    std::array<float, 4> link{};
    std::array<float, 4> control{};
    std::array<float, 4> freeFlyPosition{};
    std::array<float, 4> freeFlyForward{};
    std::array<float, 4> freeFlyUp{};
};
static_assert(sizeof(PortalLabGpuParameters) == 128U);

struct alignas(16) PortalLabGpuBuffer {
    PortalLabGpuParameters parameters{};
    PortalGrPrecomputedTable gr{};
    std::array<std::uint32_t, 4> telemetry{};
};
static_assert(sizeof(PortalLabGpuBuffer) ==
              sizeof(PortalLabGpuParameters) + sizeof(PortalGrPrecomputedTable) + 16U);

[[nodiscard]] inline PortalVector portalEndpoint(float azimuth, float elevation,
                                                  float distance) noexcept {
    const float cosineElevation = std::cos(elevation);
    return {std::sin(azimuth) * cosineElevation * distance,
            std::sin(elevation) * distance,
            std::cos(azimuth) * cosineElevation * distance};
}

[[nodiscard]] inline PortalLabGpuParameters portalGpuParameters(
    const PortalLabSettings& settings) noexcept {
    const PortalVector a = portalEndpoint(settings.endpointAAzimuth,
                                          settings.endpointAElevation,
                                          settings.endpointADistance);
    const PortalVector b = portalEndpoint(settings.endpointBAzimuth,
                                          settings.endpointBElevation,
                                          settings.endpointBDistance);
    PortalLabGpuParameters result{};
    result.endpointA = {a.x, a.y, a.z, settings.influenceRadius};
    result.endpointB = {b.x, b.y, b.z, settings.throatRadius};
    result.optical = {settings.lensStrength, settings.massBias,
                      static_cast<float>(settings.quality),
                      static_cast<float>(settings.maximumTraversals)};
    result.link = {settings.linkYaw, settings.linkPitch, settings.linkRoll,
                   static_cast<float>(settings.debugMode)};
    result.control = {settings.enabled ? 1.0F : 0.0F,
                      settings.cameraTraversalEnabled ? 1.0F : 0.0F,
                      settings.observerManifoldActive ? 1.0F : 0.0F,
                      settings.observerDestinationSide ? 1.0F : 0.0F};
    result.freeFlyPosition = {settings.freeFlyPosition.x,
                              settings.freeFlyPosition.y,
                              settings.freeFlyPosition.z, 1.0F};
    const PortalVector freeFlyForward = portalNormalize(settings.freeFlyForward);
    const PortalVector freeFlyUp = portalNormalize(
        settings.freeFlyUp - freeFlyForward *
            portalDot(settings.freeFlyUp, freeFlyForward),
        {0.0F, 1.0F, 0.0F});
    result.freeFlyForward = {freeFlyForward.x, freeFlyForward.y,
                             freeFlyForward.z, 0.0F};
    result.freeFlyUp = {freeFlyUp.x, freeFlyUp.y, freeFlyUp.z, 0.0F};
    return result;
}

struct PortalFrame {
    PortalVector x{};
    PortalVector y{};
    PortalVector z{};
};

struct PortalQuaternion {
    float x{};
    float y{};
    float z{};
    float w{1.0F};
};

[[nodiscard]] inline PortalQuaternion portalQuaternionNormalize(
    PortalQuaternion value) noexcept {
    const float lengthSquared = value.x * value.x + value.y * value.y +
                                value.z * value.z + value.w * value.w;
    if (!(lengthSquared > 1.0e-12F) || !std::isfinite(lengthSquared)) {
        return {};
    }
    const float inverse = 1.0F / std::sqrt(lengthSquared);
    return {value.x * inverse, value.y * inverse,
            value.z * inverse, value.w * inverse};
}

[[nodiscard]] inline PortalQuaternion operator*(PortalQuaternion a,
                                                  PortalQuaternion b) noexcept {
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

[[nodiscard]] inline PortalQuaternion portalQuaternionConjugate(
    PortalQuaternion value) noexcept {
    return {-value.x, -value.y, -value.z, value.w};
}

[[nodiscard]] inline PortalVector portalQuaternionRotate(
    PortalQuaternion rotation, PortalVector value) noexcept {
    rotation = portalQuaternionNormalize(rotation);
    const PortalVector axis{rotation.x, rotation.y, rotation.z};
    const PortalVector twiceCross = portalCross(axis, value) * 2.0F;
    return value + twiceCross * rotation.w + portalCross(axis, twiceCross);
}

[[nodiscard]] inline float portalQuintic(float value) noexcept {
    const float t = std::clamp(value, 0.0F, 1.0F);
    return t * t * t * (t * (t * 6.0F - 15.0F) + 10.0F);
}

[[nodiscard]] inline PortalQuaternion portalQuaternionSlerpIdentity(
    PortalQuaternion target, float progress) noexcept {
    target = portalQuaternionNormalize(target);
    if (target.w < 0.0F) {
        target = {-target.x, -target.y, -target.z, -target.w};
    }
    const float t = portalQuintic(progress);
    const float cosine = std::clamp(target.w, -1.0F, 1.0F);
    if (cosine > 0.9995F) {
        return portalQuaternionNormalize({target.x * t, target.y * t,
                                          target.z * t,
                                          1.0F + (target.w - 1.0F) * t});
    }
    const float angle = std::acos(cosine);
    const float sine = std::sin(angle);
    if (!(sine > 1.0e-6F)) {
        return {};
    }
    const float targetWeight = std::sin(t * angle) / sine;
    const float identityWeight = std::sin((1.0F - t) * angle) / sine;
    return portalQuaternionNormalize({target.x * targetWeight,
                                      target.y * targetWeight,
                                      target.z * targetWeight,
                                      identityWeight + target.w * targetWeight});
}

[[nodiscard]] inline PortalFrame portalFrame(PortalVector center) noexcept {
    const PortalVector z = portalNormalize(center);
    const PortalVector reference = std::abs(z.y) < 0.92F
        ? PortalVector{0.0F, 1.0F, 0.0F}
        : PortalVector{1.0F, 0.0F, 0.0F};
    const PortalVector x = portalNormalize(portalCross(reference, z),
                                           {1.0F, 0.0F, 0.0F});
    return {x, portalNormalize(portalCross(z, x), {0.0F, 1.0F, 0.0F}), z};
}

[[nodiscard]] inline PortalVector portalToLocal(const PortalFrame& frame,
                                                 PortalVector value) noexcept {
    return {portalDot(value, frame.x), portalDot(value, frame.y),
            portalDot(value, frame.z)};
}
[[nodiscard]] inline PortalVector portalFromLocal(const PortalFrame& frame,
                                                   PortalVector value) noexcept {
    return frame.x * value.x + frame.y * value.y + frame.z * value.z;
}

[[nodiscard]] inline PortalVector portalRotateLink(PortalVector value,
                                                    const PortalLabSettings& settings) noexcept {
    const float cy = std::cos(settings.linkYaw);
    const float sy = std::sin(settings.linkYaw);
    const float cp = std::cos(settings.linkPitch);
    const float sp = std::sin(settings.linkPitch);
    const float cr = std::cos(settings.linkRoll);
    const float sr = std::sin(settings.linkRoll);
    const PortalVector yawed{cy * value.x + sy * value.z, value.y,
                             -sy * value.x + cy * value.z};
    const PortalVector pitched{yawed.x, cp * yawed.y - sp * yawed.z,
                               sp * yawed.y + cp * yawed.z};
    return {cr * pitched.x - sr * pitched.y,
            sr * pitched.x + cr * pitched.y, pitched.z};
}

[[nodiscard]] inline PortalVector portalRotateLinkInverse(
    PortalVector value, const PortalLabSettings& settings) noexcept {
    const float cy = std::cos(settings.linkYaw);
    const float sy = std::sin(settings.linkYaw);
    const float cp = std::cos(settings.linkPitch);
    const float sp = std::sin(settings.linkPitch);
    const float cr = std::cos(settings.linkRoll);
    const float sr = std::sin(settings.linkRoll);
    const PortalVector unrolled{cr * value.x + sr * value.y,
                                -sr * value.x + cr * value.y, value.z};
    const PortalVector unpitched{unrolled.x,
                                 cp * unrolled.y + sp * unrolled.z,
                                 -sp * unrolled.y + cp * unrolled.z};
    return {cy * unpitched.x - sy * unpitched.z, unpitched.y,
            sy * unpitched.x + cy * unpitched.z};
}

[[nodiscard]] inline PortalVector portalMapVectorAcross(
    PortalVector value, std::uint32_t sourceEndpoint,
    const PortalLabSettings& settings) noexcept {
    const PortalVector centers[2]{
        portalEndpoint(settings.endpointAAzimuth, settings.endpointAElevation,
                       settings.endpointADistance),
        portalEndpoint(settings.endpointBAzimuth, settings.endpointBElevation,
                       settings.endpointBDistance)};
    const std::uint32_t source = std::min(sourceEndpoint, 1U);
    const std::uint32_t destination = 1U - source;
    const PortalFrame sourceFrame = portalFrame(centers[source]);
    const PortalFrame destinationFrame = portalFrame(centers[destination]);
    PortalVector local = portalToLocal(sourceFrame, value);
    if (source == 0U) {
        local = portalRotateLink(local, settings);
    } else {
        local = portalRotateLinkInverse(local, settings);
    }
    return portalFromLocal(destinationFrame, local);
}

[[nodiscard]] inline PortalQuaternion portalQuaternionFromMappedBasis(
    PortalVector mappedX, PortalVector mappedY, PortalVector mappedZ) noexcept {
    mappedX = portalNormalize(mappedX, {1.0F, 0.0F, 0.0F});
    mappedY = portalNormalize(mappedY - mappedX * portalDot(mappedX, mappedY),
                              {0.0F, 1.0F, 0.0F});
    mappedZ = portalNormalize(portalCross(mappedX, mappedY), mappedZ);
    const float m00 = mappedX.x;
    const float m01 = mappedY.x;
    const float m02 = mappedZ.x;
    const float m10 = mappedX.y;
    const float m11 = mappedY.y;
    const float m12 = mappedZ.y;
    const float m20 = mappedX.z;
    const float m21 = mappedY.z;
    const float m22 = mappedZ.z;
    PortalQuaternion result{};
    const float trace = m00 + m11 + m22;
    if (trace > 0.0F) {
        const float scale = std::sqrt(trace + 1.0F) * 2.0F;
        result = {(m21 - m12) / scale, (m02 - m20) / scale,
                  (m10 - m01) / scale, 0.25F * scale};
    } else if (m00 > m11 && m00 > m22) {
        const float scale = std::sqrt(1.0F + m00 - m11 - m22) * 2.0F;
        result = {0.25F * scale, (m01 + m10) / scale,
                  (m02 + m20) / scale, (m21 - m12) / scale};
    } else if (m11 > m22) {
        const float scale = std::sqrt(1.0F + m11 - m00 - m22) * 2.0F;
        result = {(m01 + m10) / scale, 0.25F * scale,
                  (m12 + m21) / scale, (m02 - m20) / scale};
    } else {
        const float scale = std::sqrt(1.0F + m22 - m00 - m11) * 2.0F;
        result = {(m02 + m20) / scale, (m12 + m21) / scale,
                  0.25F * scale, (m10 - m01) / scale};
    }
    return portalQuaternionNormalize(result);
}

[[nodiscard]] inline PortalQuaternion portalLinkQuaternion(
    std::uint32_t sourceEndpoint, const PortalLabSettings& settings) noexcept {
    return portalQuaternionFromMappedBasis(
        portalMapVectorAcross({1.0F, 0.0F, 0.0F}, sourceEndpoint, settings),
        portalMapVectorAcross({0.0F, 1.0F, 0.0F}, sourceEndpoint, settings),
        portalMapVectorAcross({0.0F, 0.0F, 1.0F}, sourceEndpoint, settings));
}

[[nodiscard]] inline float portalTransportProgress(
    float radius, const PortalLabSettings& settings, float bodyRadius = 0.0F) noexcept {
    const float effectiveThroat = settings.throatRadius - std::max(bodyRadius, 0.0F);
    return std::clamp((settings.influenceRadius - radius) /
                          std::max(settings.influenceRadius - effectiveThroat,
                                   1.0e-5F),
                      0.0F, 1.0F);
}

[[nodiscard]] inline float portalGrTransportProgress(
    float radius, const PortalLabSettings& settings,
    const PortalGrPrecomputedTable& table,
    float bodyRadius = 0.0F) noexcept {
    const float outer = std::max(settings.influenceRadius, 1.0e-5F);
    const float throat = std::max(
        outer * table.header[0] - std::max(bodyRadius, 0.0F), 1.0e-5F);
    const float clampedRadius = std::clamp(radius, throat, outer);
    const float outerL = std::sqrt(std::max(outer * outer - throat * throat,
                                             1.0e-10F));
    const float l = std::sqrt(std::max(
        clampedRadius * clampedRadius - throat * throat, 0.0F));
    // The compact quintic collar matches the finite Ellis chart to ordinary
    // Euclidean scene coordinates with zero first derivative at the cutoff.
    return portalQuintic((outerL - l) / std::max(outerL, 1.0e-6F));
}

[[nodiscard]] inline PortalQuaternion portalTransportRotation(
    std::uint32_t sourceEndpoint, float progress,
    const PortalLabSettings& settings) noexcept {
    return portalQuaternionSlerpIdentity(
        portalLinkQuaternion(sourceEndpoint, settings), progress);
}

[[nodiscard]] inline bool portalRaySphere(PortalVector origin, PortalVector direction,
                                          PortalVector center, float radius,
                                          float& nearDistance,
                                          float& farDistance) noexcept {
    const PortalVector relative = origin - center;
    const float halfB = portalDot(relative, direction);
    const float c = portalDot(relative, relative) - radius * radius;
    const float discriminant = halfB * halfB - c;
    if (!(discriminant >= 0.0F) || !std::isfinite(discriminant)) {
        return false;
    }
    const float root = std::sqrt(discriminant);
    nearDistance = -halfB - root;
    farDistance = -halfB + root;
    return farDistance >= 0.0F && std::isfinite(nearDistance) &&
           std::isfinite(farDistance);
}

[[nodiscard]] inline PortalVector portalCurvature(
    PortalVector position, PortalVector direction, PortalVector center,
    const PortalLabSettings& settings) noexcept {
    const PortalVector relative = position - center;
    const float radius = portalLength(relative);
    if (!(radius > settings.throatRadius) || radius >= settings.influenceRadius) {
        return {};
    }
    const PortalVector inward = portalNormalize(relative * -1.0F);
    const PortalVector perpendicular = inward - direction * portalDot(inward, direction);
    const float normalizedBoundary = std::clamp(
        (settings.influenceRadius - radius) /
            std::max(settings.influenceRadius - settings.throatRadius, 1.0e-5F),
        0.0F, 1.0F);
    const float window = normalizedBoundary * normalizedBoundary *
                         (3.0F - 2.0F * normalizedBoundary);
    const float q2 = settings.throatRadius * settings.throatRadius;
    const float denominator = std::pow(radius * radius + q2, 1.5F);
    const float ellis = settings.lensStrength * q2 /
                        std::max(denominator, 1.0e-7F);
    const float mass = settings.massBias * settings.throatRadius /
                       std::max(radius * radius + q2, 1.0e-6F);
    return perpendicular * ((ellis + mass) * window * window);
}

struct PortalTraceResult {
    PortalVector origin{};
    PortalVector direction{0.0F, 0.0F, -1.0F};
    std::uint32_t integrationSteps{};
    std::uint32_t throatTraversals{};
    bool affected{};
    bool finite{true};
};

struct PortalIntegrationState {
    PortalVector position{};
    PortalVector direction{0.0F, 0.0F, -1.0F};
};

[[nodiscard]] inline PortalIntegrationState portalRk4Step(
    PortalIntegrationState state, PortalVector center,
    const PortalLabSettings& settings, float stepLength) noexcept {
    const PortalVector k1Position = state.direction;
    const PortalVector k1Direction = portalCurvature(
        state.position, state.direction, center, settings);
    const PortalVector direction2 = portalNormalize(
        state.direction + k1Direction * (0.5F * stepLength), state.direction);
    const PortalVector position2 = state.position +
                                   k1Position * (0.5F * stepLength);
    const PortalVector k2Position = direction2;
    const PortalVector k2Direction = portalCurvature(
        position2, direction2, center, settings);
    const PortalVector direction3 = portalNormalize(
        state.direction + k2Direction * (0.5F * stepLength), state.direction);
    const PortalVector position3 = state.position +
                                   k2Position * (0.5F * stepLength);
    const PortalVector k3Position = direction3;
    const PortalVector k3Direction = portalCurvature(
        position3, direction3, center, settings);
    const PortalVector direction4 = portalNormalize(
        state.direction + k3Direction * stepLength, state.direction);
    const PortalVector position4 = state.position + k3Position * stepLength;
    const PortalVector k4Position = direction4;
    const PortalVector k4Direction = portalCurvature(
        position4, direction4, center, settings);
    const float sixthStep = stepLength / 6.0F;
    const PortalVector integratedPosition = state.position +
        (k1Position + k2Position * 2.0F + k3Position * 2.0F + k4Position) *
            sixthStep;
    const PortalVector integratedDirection = portalNormalize(
        state.direction +
            (k1Direction + k2Direction * 2.0F + k3Direction * 2.0F +
             k4Direction) * sixthStep,
        state.direction);
    return {integratedPosition, integratedDirection};
}

[[nodiscard]] inline bool portalSegmentSphereFraction(
    PortalVector start, PortalVector end, PortalVector center, float radius,
    bool exiting, float& fraction) noexcept {
    const PortalVector segment = end - start;
    const PortalVector relative = start - center;
    const float a = portalDot(segment, segment);
    const float b = 2.0F * portalDot(relative, segment);
    const float c = portalDot(relative, relative) - radius * radius;
    const float discriminant = b * b - 4.0F * a * c;
    if (!(a > 1.0e-12F) || !(discriminant >= 0.0F) ||
        !std::isfinite(discriminant)) {
        return false;
    }
    const float root = std::sqrt(discriminant);
    const float inverse = 0.5F / a;
    const float first = (-b - root) * inverse;
    const float second = (-b + root) * inverse;
    const float selected = exiting ? second : first;
    if (!(selected >= 0.0F && selected <= 1.0F) || !std::isfinite(selected)) {
        return false;
    }
    fraction = selected;
    return true;
}

[[nodiscard]] inline PortalTraceResult tracePortalRay(
    PortalVector origin, PortalVector direction,
    const PortalLabSettings& settings) noexcept {
    PortalTraceResult result{origin, portalNormalize(direction)};
    if (!settings.enabled || !(settings.influenceRadius > settings.throatRadius) ||
        settings.maximumTraversals == 0U) {
        return result;
    }
    const PortalVector centers[2]{
        portalEndpoint(settings.endpointAAzimuth, settings.endpointAElevation,
                       settings.endpointADistance),
        portalEndpoint(settings.endpointBAzimuth, settings.endpointBElevation,
                       settings.endpointBDistance)};
    const std::uint32_t stepsPerInfluence = settings.quality == 0U ? 32U
        : settings.quality == 1U ? 64U : 96U;
    // N steps cover a full diameter, not one radius. The prior radius/N
    // increment plus N-step limit stopped non-throat rays inside the volume
    // and produced visible concentric termination shells.
    const float stepLength = (2.0F * settings.influenceRadius) /
                             static_cast<float>(stepsPerInfluence);
    const float epsilon = std::max(settings.throatRadius * 0.002F, 1.0e-5F);

    for (std::uint32_t region = 0U; region < 4U; ++region) {
        float bestNear = std::numeric_limits<float>::infinity();
        std::uint32_t endpoint = 2U;
        for (std::uint32_t candidate = 0U; candidate < 2U; ++candidate) {
            float nearDistance = 0.0F;
            float farDistance = 0.0F;
            if (portalRaySphere(result.origin, result.direction, centers[candidate],
                                settings.influenceRadius, nearDistance, farDistance)) {
                nearDistance = std::max(nearDistance, 0.0F);
                if (nearDistance < bestNear && farDistance > epsilon) {
                    bestNear = nearDistance;
                    endpoint = candidate;
                }
            }
        }
        if (endpoint >= 2U || !std::isfinite(bestNear)) {
            break;
        }
        result.origin = result.origin + result.direction * (bestNear + epsilon);
        result.affected = true;
        bool transferred = false;
        bool exited = false;
        for (std::uint32_t step = 0U; step < stepsPerInfluence + 8U; ++step) {
            PortalIntegrationState integrated = portalRk4Step(
                {result.origin, result.direction}, centers[endpoint],
                settings, stepLength);
            const float currentProgress = portalTransportProgress(
                portalLength(result.origin - centers[endpoint]), settings);
            const float nextProgress = portalTransportProgress(
                portalLength(integrated.position - centers[endpoint]), settings);
            const PortalQuaternion currentRotation = portalTransportRotation(
                endpoint, currentProgress, settings);
            const PortalQuaternion nextRotation = portalTransportRotation(
                endpoint, nextProgress, settings);
            const PortalQuaternion incrementalRotation = nextRotation *
                portalQuaternionConjugate(currentRotation);
            integrated.position = centers[endpoint] + portalQuaternionRotate(
                incrementalRotation, integrated.position - centers[endpoint]);
            integrated.direction = portalNormalize(portalQuaternionRotate(
                incrementalRotation, integrated.direction), integrated.direction);
            const PortalVector nextDirection = integrated.direction;
            const PortalVector nextPosition = integrated.position;
            ++result.integrationSteps;
            if (!portalFinite(nextPosition) || !portalFinite(nextDirection)) {
                result.finite = false;
                return result;
            }
            float throatFraction = 0.0F;
            if (portalSegmentSphereFraction(
                    result.origin, nextPosition, centers[endpoint],
                    settings.throatRadius, false, throatFraction)) {
                if (result.throatTraversals >= settings.maximumTraversals) {
                    result.origin = nextPosition;
                    result.direction = nextDirection;
                    return result;
                }
                const std::uint32_t destination = 1U - endpoint;
                const PortalVector crossingDirection = portalNormalize(
                    result.direction * (1.0F - throatFraction) +
                        nextDirection * throatFraction,
                    nextDirection);
                const PortalVector crossingPosition = result.origin *
                    (1.0F - throatFraction) + nextPosition * throatFraction;
                const PortalVector outgoingRadial = portalNormalize(
                    crossingPosition - centers[endpoint], crossingDirection);
                result.direction = crossingDirection;
                result.origin = centers[destination] - outgoingRadial *
                    (settings.throatRadius + epsilon);
                float destinationNear = 0.0F;
                float destinationFar = 0.0F;
                if (!portalRaySphere(result.origin, result.direction,
                                     centers[destination],
                                     settings.influenceRadius,
                                     destinationNear, destinationFar)) {
                    result.finite = false;
                    return result;
                }
                result.origin = result.origin + result.direction *
                    (destinationFar + epsilon);
                ++result.throatTraversals;
                transferred = true;
                break;
            }
            float exitFraction = 0.0F;
            if (portalSegmentSphereFraction(
                    result.origin, nextPosition, centers[endpoint],
                    settings.influenceRadius, true, exitFraction)) {
                result.origin = result.origin * (1.0F - exitFraction) +
                                nextPosition * exitFraction;
                result.direction = portalNormalize(
                    result.direction * (1.0F - exitFraction) +
                        nextDirection * exitFraction,
                    nextDirection);
                exited = true;
                break;
            }
            result.origin = nextPosition;
            result.direction = nextDirection;
        }
        if (!transferred) {
            if (!exited) {
                result.finite = false;
                return result;
            }
            result.origin = result.origin + result.direction * epsilon;
        }
    }
    return result;
}

[[nodiscard]] inline PortalVector portalRotateAxis(
    PortalVector value, PortalVector axis, float angle) noexcept {
    axis = portalNormalize(axis);
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    return value * cosine + portalCross(axis, value) * sine +
           axis * (portalDot(axis, value) * (1.0F - cosine));
}

[[nodiscard]] inline PortalTraceResult tracePortalRayGr(
    PortalVector origin, PortalVector direction,
    const PortalLabSettings& settings,
    const PortalGrPrecomputedTable& table) noexcept {
    PortalTraceResult result{origin, portalNormalize(direction)};
    if (!settings.enabled || table.status[3] < 0.5F) {
        return result;
    }
    const PortalVector centers[2]{
        portalEndpoint(settings.endpointAAzimuth, settings.endpointAElevation,
                       settings.endpointADistance),
        portalEndpoint(settings.endpointBAzimuth, settings.endpointBElevation,
                       settings.endpointBDistance)};
    float bestNear = std::numeric_limits<float>::infinity();
    std::uint32_t endpoint = 2U;
    for (std::uint32_t candidate = 0U; candidate < 2U; ++candidate) {
        float nearDistance = 0.0F;
        float farDistance = 0.0F;
        if (portalRaySphere(result.origin, result.direction, centers[candidate],
                            settings.influenceRadius,
                            nearDistance, farDistance)) {
            nearDistance = std::max(nearDistance, 0.0F);
            if (nearDistance < bestNear) {
                bestNear = nearDistance;
                endpoint = candidate;
            }
        }
    }
    if (endpoint >= 2U || !std::isfinite(bestNear)) {
        return result;
    }
    const float epsilon = std::max(settings.influenceRadius * 2.0e-4F, 1.0e-5F);
    const PortalVector entry = result.origin + result.direction *
        (bestNear + epsilon);
    const PortalVector entryRadial = portalNormalize(entry - centers[endpoint]);
    const float radialVelocity = portalDot(result.direction, entryRadial);
    if (!(radialVelocity < -1.0e-5F)) {
        return result;
    }
    PortalVector tangent = result.direction - entryRadial * radialVelocity;
    const float impact = std::clamp(portalLength(tangent), 0.0F, table.header[2]);
    tangent = portalNormalize(tangent,
        portalNormalize(portalCross(
            std::abs(entryRadial.y) < 0.9F
                ? PortalVector{0.0F, 1.0F, 0.0F}
                : PortalVector{1.0F, 0.0F, 0.0F}, entryRadial)));
    const PortalVector planeAxis = portalNormalize(
        portalCross(entryRadial, tangent));
    PortalGrGpuSample exit = samplePortalGrExitTable(table, impact);
    bool crossed = exit.value[3] >= 0.0F;
    if (std::abs(exit.value[3]) > 5.0e-4F) {
        const PortalGrGeodesicResult refined = integratePortalEllisGeodesic(
            impact, settings.quality == 0U ? 4096U
                    : settings.quality == 1U ? 8192U : 16384U);
        if (refined.finite) {
            exit.value[0] = static_cast<float>(refined.azimuth);
            exit.value[1] = static_cast<float>(refined.affineLength);
            exit.value[2] = static_cast<float>(refined.frameAngle);
            crossed = refined.crossed;
            result.integrationSteps = settings.quality == 0U ? 4096U
                : settings.quality == 1U ? 8192U : 16384U;
        }
    }
    const PortalVector exitRadial = portalNormalize(
        portalRotateAxis(entryRadial, planeAxis, exit.value[0]));
    const PortalVector exitTangent = portalNormalize(
        portalRotateAxis(tangent, planeAxis, exit.value[0]));
    const float radialSpeed = std::sqrt(std::max(1.0F - impact * impact, 0.0F));
    const PortalVector exitDirection = portalNormalize(
        exitRadial * radialSpeed + exitTangent * impact);
    if (crossed) {
        const std::uint32_t destination = 1U - endpoint;
        const PortalVector mappedRadial = portalNormalize(
            portalMapVectorAcross(exitRadial, endpoint, settings));
        const PortalVector commonCrossedDirection = portalNormalize(
            exitRadial * -radialSpeed + exitTangent * impact);
        const PortalVector mappedDirection = portalNormalize(
            portalMapVectorAcross(commonCrossedDirection, endpoint, settings));
        result.origin = centers[destination] - mappedRadial *
            (settings.influenceRadius + epsilon);
        result.direction = mappedDirection;
        result.throatTraversals = 1U;
    } else {
        result.origin = centers[endpoint] + exitRadial *
            (settings.influenceRadius + epsilon);
        result.direction = exitDirection;
    }
    result.affected = true;
    result.finite = portalFinite(result.origin) && portalFinite(result.direction);
    return result;
}

struct PortalManifoldBodyState;

[[nodiscard]] inline PortalTraceResult tracePortalObserverRayGr(
    PortalVector origin, PortalVector direction,
    const PortalLabSettings& settings,
    const PortalGrPrecomputedTable& table,
    const PortalManifoldBodyState& observer) noexcept;

struct PortalBodyTransfer {
    PortalVector position{};
    PortalVector forward{0.0F, 0.0F, -1.0F};
    PortalVector velocity{};
    std::uint32_t sourceEndpoint{2U};
    bool transferred{};
};

struct PortalManifoldBodyState {
    PortalVector position{};
    PortalVector forward{0.0F, 0.0F, -1.0F};
    PortalVector up{0.0F, 1.0F, 0.0F};
    PortalVector velocity{};
    std::uint32_t sourceEndpoint{2U};
    float properProgress{};
    bool active{};
    bool destinationSide{};
    bool entered{};
    bool crossed{};
    bool exited{};
    bool finite{true};
};

[[nodiscard]] inline PortalVector portalOrthonormalUp(
    PortalVector forward, PortalVector up) noexcept {
    forward = portalNormalize(forward);
    PortalVector projected = up - forward * portalDot(up, forward);
    if (portalLength(projected) <= 1.0e-6F) {
        const PortalVector reference = std::abs(forward.y) < 0.92F
            ? PortalVector{0.0F, 1.0F, 0.0F}
            : PortalVector{1.0F, 0.0F, 0.0F};
        projected = reference - forward * portalDot(reference, forward);
    }
    return portalNormalize(projected, {0.0F, 1.0F, 0.0F});
}

// Advances a physical body through the same smooth SO(3) connection used by
// light rays. Position is represented in two overlapping endpoint charts;
// `properProgress` is the continuous manifold coordinate. Changing the
// active Euclidean chart at the throat is therefore a coordinate change, not
// a physical event or an orientation remap.
[[nodiscard]] inline PortalManifoldBodyState portalAdvanceBodyManifold(
    PortalVector previousPosition, PortalVector currentPosition,
    PortalVector forward, PortalVector up, PortalVector velocity,
    const PortalLabSettings& settings,
    const PortalManifoldBodyState& previousState = {},
    float bodyRadius = 0.0F,
    const PortalGrPrecomputedTable* grTable = nullptr) noexcept {
    PortalManifoldBodyState result = previousState;
    result.position = currentPosition;
    result.forward = portalNormalize(forward);
    result.up = portalOrthonormalUp(result.forward, up);
    result.velocity = velocity;
    result.entered = false;
    result.crossed = false;
    result.exited = false;
    result.finite = true;
    if (!settings.enabled || !settings.cameraTraversalEnabled ||
        !(settings.influenceRadius > settings.throatRadius)) {
        result.active = false;
        result.destinationSide = false;
        result.sourceEndpoint = 2U;
        result.properProgress = 0.0F;
        return result;
    }
    const PortalVector centers[2]{
        portalEndpoint(settings.endpointAAzimuth, settings.endpointAElevation,
                       settings.endpointADistance),
        portalEndpoint(settings.endpointBAzimuth, settings.endpointBElevation,
                       settings.endpointBDistance)};
    const float triggerRadius = settings.throatRadius - std::max(bodyRadius, 0.0F);
    if (!(triggerRadius > 1.0e-4F)) {
        result.active = false;
        return result;
    }

    if (!previousState.active) {
        float bestEntry = 2.0F;
        std::uint32_t source = 2U;
        for (std::uint32_t endpoint = 0U; endpoint < 2U; ++endpoint) {
            const float currentRadius = portalLength(currentPosition - centers[endpoint]);
            float fraction = 0.0F;
            if (currentRadius < settings.influenceRadius &&
                portalSegmentSphereFraction(previousPosition, currentPosition,
                                            centers[endpoint],
                                            settings.influenceRadius, false,
                                            fraction) && fraction < bestEntry) {
                bestEntry = fraction;
                source = endpoint;
            } else if (currentRadius < settings.influenceRadius && source >= 2U) {
                source = endpoint;
                bestEntry = 0.0F;
            }
        }
        if (source >= 2U) {
            result.active = false;
            result.sourceEndpoint = 2U;
            result.properProgress = 0.0F;
            return result;
        }
        result.active = true;
        result.destinationSide = false;
        result.sourceEndpoint = source;
        result.properProgress = 0.0F;
        result.entered = true;
    }

    const std::uint32_t source = std::min(result.sourceEndpoint, 1U);
    const std::uint32_t destination = 1U - source;
    if (previousState.destinationSide) {
        const float destinationRadius = portalLength(currentPosition - centers[destination]);
        result.destinationSide = true;
        result.properProgress = 1.0F;
        if (destinationRadius >= settings.influenceRadius) {
            result.active = false;
            result.destinationSide = false;
            result.sourceEndpoint = 2U;
            result.properProgress = 0.0F;
            result.exited = true;
        }
        return result;
    }

    const float currentRadius = portalLength(currentPosition - centers[source]);
    const float nextProgress = grTable != nullptr
        ? portalGrTransportProgress(currentRadius, settings, *grTable, bodyRadius)
        : portalTransportProgress(currentRadius, settings, bodyRadius);
    result.properProgress = nextProgress;

    float throatFraction = 0.0F;
    if (portalSegmentSphereFraction(previousPosition, currentPosition,
                                    centers[source], triggerRadius, false,
                                    throatFraction) || currentRadius < triggerRadius) {
        const PortalQuaternion fullRotation = portalTransportRotation(
            source, 1.0F, settings);
        PortalVector relative = portalQuaternionRotate(
            fullRotation, result.position - centers[source]);
        result.forward = portalNormalize(
            portalQuaternionRotate(fullRotation, result.forward), result.forward);
        result.up = portalOrthonormalUp(result.forward,
            portalQuaternionRotate(fullRotation, result.up));
        result.velocity = portalQuaternionRotate(fullRotation, result.velocity);
        const float overshoot = std::max(triggerRadius - currentRadius, 0.0F);
        const float hysteresis = std::max(settings.throatRadius * 0.02F, 1.0e-4F);
        relative = portalNormalize(relative, result.forward) *
                   (triggerRadius + overshoot + hysteresis);
        result.position = centers[destination] - relative;
        result.properProgress = 1.0F;
        result.destinationSide = true;
        result.crossed = true;
    }
    result.finite = portalFinite(result.position) &&
                    portalFinite(result.forward) && portalFinite(result.up) &&
                    portalFinite(result.velocity) &&
                    std::isfinite(result.properProgress);
    if (!result.finite) {
        result.active = false;
    }
    return result;
}

[[nodiscard]] inline PortalTraceResult tracePortalObserverRayGr(
    PortalVector origin, PortalVector direction,
    const PortalLabSettings& settings,
    const PortalGrPrecomputedTable& table,
    const PortalManifoldBodyState& observer) noexcept {
    if (!observer.active) {
        return tracePortalRayGr(origin, direction, settings, table);
    }
    PortalTraceResult result{origin, portalNormalize(direction)};
    const PortalVector centers[2]{
        portalEndpoint(settings.endpointAAzimuth, settings.endpointAElevation,
                       settings.endpointADistance),
        portalEndpoint(settings.endpointBAzimuth, settings.endpointBElevation,
                       settings.endpointBDistance)};
    const float epsilon = std::max(settings.influenceRadius * 2.0e-4F, 1.0e-5F);
    std::uint32_t containingEndpoint = 2U;
    float bestRadius = std::numeric_limits<float>::infinity();
    for (std::uint32_t endpoint = 0U; endpoint < 2U; ++endpoint) {
        const float radius = portalLength(origin - centers[endpoint]);
        if (radius <= settings.influenceRadius + epsilon && radius < bestRadius) {
            bestRadius = radius;
            containingEndpoint = endpoint;
        }
    }
    if (containingEndpoint >= 2U) {
        return result;
    }
    const std::uint32_t sourceEndpoint = observer.destinationSide
        ? 1U - containingEndpoint : containingEndpoint;
    const std::uint32_t destinationEndpoint = 1U - sourceEndpoint;
    const PortalVector endpointRadial = portalNormalize(
        origin - centers[containingEndpoint]);
    const PortalVector commonRadial = observer.destinationSide
        ? portalNormalize(portalMapVectorAcross(
              endpointRadial * -1.0F, containingEndpoint, settings))
        : endpointRadial;
    const PortalVector commonDirection = observer.destinationSide
        ? portalNormalize(portalMapVectorAcross(
              result.direction, containingEndpoint, settings))
        : result.direction;
    const float normalizedRadius = std::clamp(
        portalLength(origin - centers[containingEndpoint]) /
            std::max(settings.influenceRadius, 1.0e-6F),
        kPortalGrThroatRatio, 1.0F);
    const float properL = std::sqrt(std::max(
        normalizedRadius * normalizedRadius -
        kPortalGrThroatRatio * kPortalGrThroatRatio, 0.0F));
    const float signedProperL = observer.destinationSide ? -properL : properL;
    const float radialVelocity = portalDot(commonDirection, commonRadial);
    if (!(radialVelocity < -1.0e-5F)) {
        float nearDistance = 0.0F;
        float farDistance = 0.0F;
        if (portalRaySphere(origin, result.direction, centers[containingEndpoint],
                            settings.influenceRadius,
                            nearDistance, farDistance)) {
            result.origin = result.origin + result.direction *
                (farDistance + epsilon);
        }
        result.affected = true;
        return result;
    }
    PortalVector commonTangentVector = commonDirection -
        commonRadial * radialVelocity;
    const float tangentSpeed = std::clamp(
        portalLength(commonTangentVector), 0.0F, 0.985F);
    const PortalVector commonTangent = portalNormalize(
        commonTangentVector,
        portalNormalize(portalCross(
            std::abs(commonRadial.y) < 0.9F
                ? PortalVector{0.0F, 1.0F, 0.0F}
                : PortalVector{1.0F, 0.0F, 0.0F}, commonRadial)));
    const PortalVector planeAxis = portalNormalize(
        portalCross(commonRadial, commonTangent));
    const PortalGrGpuSample exit = samplePortalGrObserverTable(
        table, signedProperL, tangentSpeed);
    if (exit.value[3] > 0.5F) {
        result.finite = false;
        return result;
    }
    const PortalVector exitRadial = portalNormalize(portalRotateAxis(
        commonRadial, planeAxis, exit.value[0]));
    const PortalVector exitTangent = portalNormalize(portalRotateAxis(
        commonTangent, planeAxis, exit.value[0]));
    const float exitTangentSpeed = std::clamp(exit.value[1], 0.0F, 0.999999F);
    const float exitRadialSpeed = std::sqrt(std::max(
        1.0F - exitTangentSpeed * exitTangentSpeed, 0.0F));
    if (exit.value[2] >= 0.0F) {
        const PortalVector mappedRadial = portalNormalize(
            portalMapVectorAcross(exitRadial, sourceEndpoint, settings));
        const PortalVector commonExitDirection = portalNormalize(
            exitRadial * -exitRadialSpeed +
            exitTangent * exitTangentSpeed);
        result.origin = centers[destinationEndpoint] - mappedRadial *
            (settings.influenceRadius + epsilon);
        result.direction = portalNormalize(portalMapVectorAcross(
            commonExitDirection, sourceEndpoint, settings));
        result.throatTraversals = 1U;
    } else {
        result.origin = centers[sourceEndpoint] + exitRadial *
            (settings.influenceRadius + epsilon);
        result.direction = portalNormalize(
            exitRadial * exitRadialSpeed +
            exitTangent * exitTangentSpeed);
    }
    result.affected = true;
    result.finite = portalFinite(result.origin) && portalFinite(result.direction);
    return result;
}

[[nodiscard]] inline PortalBodyTransfer portalTransferBodySegment(
    PortalVector previousPosition, PortalVector currentPosition,
    PortalVector forward, PortalVector velocity,
    const PortalLabSettings& settings, float bodyRadius = 0.0F) noexcept {
    PortalBodyTransfer result{currentPosition, portalNormalize(forward), velocity};
    if (!settings.enabled) {
        return result;
    }
    const PortalVector centers[2]{
        portalEndpoint(settings.endpointAAzimuth, settings.endpointAElevation,
                       settings.endpointADistance),
        portalEndpoint(settings.endpointBAzimuth, settings.endpointBElevation,
                       settings.endpointBDistance)};
    const float triggerRadius = settings.throatRadius - std::max(bodyRadius, 0.0F);
    if (!(triggerRadius > 1.0e-4F)) {
        return result;
    }
    float bestFraction = 2.0F;
    std::uint32_t source = 2U;
    for (std::uint32_t endpoint = 0U; endpoint < 2U; ++endpoint) {
        if (portalLength(previousPosition - centers[endpoint]) <= triggerRadius) {
            continue;
        }
        float fraction = 0.0F;
        if (portalSegmentSphereFraction(previousPosition, currentPosition,
                                        centers[endpoint], triggerRadius,
                                        false, fraction) &&
            fraction < bestFraction) {
            bestFraction = fraction;
            source = endpoint;
        }
    }
    if (source >= 2U) {
        return result;
    }
    const std::uint32_t destination = 1U - source;
    const PortalFrame sourceFrame = portalFrame(centers[source]);
    const PortalFrame destinationFrame = portalFrame(centers[destination]);
    const auto mapVector = [&](PortalVector value) {
        return portalFromLocal(destinationFrame,
            portalRotateLink(portalToLocal(sourceFrame, value), settings));
    };
    const PortalVector displacement = currentPosition - previousPosition;
    const PortalVector mappedDisplacement = mapVector(displacement);
    const PortalVector mappedDirection = portalNormalize(
        mappedDisplacement, mapVector(portalNormalize(displacement, result.forward)));
    const float remainingFraction = 1.0F - bestFraction;
    const float hysteresis = std::max(settings.throatRadius * 0.04F, 2.0e-4F);
    result.position = centers[destination] + mappedDirection *
        (triggerRadius + hysteresis) + mappedDisplacement * remainingFraction;
    result.forward = portalNormalize(mapVector(result.forward), result.forward);
    result.velocity = mapVector(velocity);
    result.sourceEndpoint = source;
    result.transferred = portalFinite(result.position) &&
                         portalFinite(result.forward) &&
                         portalFinite(result.velocity);
    return result;
}

} // namespace voxel
