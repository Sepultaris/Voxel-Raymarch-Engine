#include "app/surface_camera_controller.hpp"

#include <algorithm>
#include <cmath>

namespace voxel {
namespace {

using Vector = SurfaceCameraController::Vector;

[[nodiscard]] float dot(const Vector& left, const Vector& right) noexcept {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

[[nodiscard]] Vector cross(const Vector& left, const Vector& right) noexcept {
    return {
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0]};
}

[[nodiscard]] Vector scaled(const Vector& value, float scale) noexcept {
    return {value[0] * scale, value[1] * scale, value[2] * scale};
}

[[nodiscard]] Vector added(const Vector& left, const Vector& right) noexcept {
    return {left[0] + right[0], left[1] + right[1], left[2] + right[2]};
}

[[nodiscard]] Vector normalized(const Vector& value, const Vector& fallback) noexcept {
    const float lengthSquared = dot(value, value);
    if (!(lengthSquared > 1e-12F) || !std::isfinite(lengthSquared)) {
        return fallback;
    }
    return scaled(value, 1.0F / std::sqrt(lengthSquared));
}

[[nodiscard]] Vector rotated(const Vector& value, const Vector& unitAxis,
                             float radians) noexcept {
    const float cosine = std::cos(radians);
    const float sine = std::sin(radians);
    return added(added(scaled(value, cosine), scaled(cross(unitAxis, value), sine)),
                 scaled(unitAxis, dot(unitAxis, value) * (1.0F - cosine)));
}

void stableTangentBasis(const Vector& radial, Vector& forward, Vector& right) noexcept {
    const Vector reference = std::abs(radial[1]) < 0.95F
                                 ? Vector{0.0F, 1.0F, 0.0F}
                                 : Vector{0.0F, 0.0F, 1.0F};
    right = normalized(cross(reference, radial), {1.0F, 0.0F, 0.0F});
    forward = normalized(cross(radial, right), {0.0F, 1.0F, 0.0F});
}

} // namespace

void SurfaceCameraController::resetFromOrbit(float yaw, float pitch) noexcept {
    const float cosinePitch = std::cos(pitch);
    radial_ = normalized({std::sin(yaw) * cosinePitch, std::sin(pitch),
                          std::cos(yaw) * cosinePitch},
                         {0.0F, 0.0F, 1.0F});
    Vector ignoredRight{};
    stableTangentBasis(radial_, tangentForward_, ignoredRight);
    lookPitch_ = 0.0F;
}

void SurfaceCameraController::rotateHeading(float radians) noexcept {
    tangentForward_ = normalized(rotated(tangentForward_, radial_, radians),
                                 tangentForward_);
    tangentForward_ = normalized(
        added(tangentForward_, scaled(radial_, -dot(tangentForward_, radial_))),
        tangentForward_);
}

void SurfaceCameraController::rotateLook(float radians) noexcept {
    constexpr float maximumPitch = 1.45F;
    lookPitch_ = std::clamp(lookPitch_ + radians, -maximumPitch, maximumPitch);
}

void SurfaceCameraController::move(float forwardInput, float rightInput,
                                   float deltaSeconds, float cameraRadius,
                                   bool sprint) noexcept {
    const float inputLength = std::sqrt(
        forwardInput * forwardInput + rightInput * rightInput);
    if (!(inputLength > 1e-6F) || !(deltaSeconds > 0.0F) || !(cameraRadius > 1e-6F)) {
        return;
    }
    const Vector right = normalized(cross(tangentForward_, radial_),
                                    {1.0F, 0.0F, 0.0F});
    const Vector movementTangent = normalized(
        added(scaled(tangentForward_, forwardInput / inputLength),
              scaled(right, rightInput / inputLength)),
        tangentForward_);
    const float speed = movementSpeed_ * (sprint ? 1.75F : 1.0F);
    const float angularDistance = std::min(
        speed * deltaSeconds * std::min(inputLength, 1.0F) / cameraRadius, 0.25F);
    const Vector rotationAxis = normalized(cross(radial_, movementTangent), right);
    radial_ = normalized(rotated(radial_, rotationAxis, angularDistance), radial_);
    tangentForward_ = normalized(
        rotated(tangentForward_, rotationAxis, angularDistance), tangentForward_);
    tangentForward_ = normalized(
        added(tangentForward_, scaled(radial_, -dot(tangentForward_, radial_))),
        movementTangent);
}

void SurfaceCameraController::adjustClearance(float delta) noexcept {
    setClearance(clearance_ + delta);
}

void SurfaceCameraController::configureVoxelGeometry(float surfaceCellWidth,
                                                     float radialLayerHeight,
                                                     float planetRadius) noexcept {
    const float width = std::max(surfaceCellWidth, planetRadius * 1e-6F);
    const float layerHeight = std::max(radialLayerHeight, width * 0.5F);
    capsule_.height = layerHeight * 4.0F;
    capsule_.radius = std::min(width * 0.45F, capsule_.height * 0.45F);
    capsule_.eyeHeight = capsule_.height - capsule_.radius * 0.45F;
    capsule_.standingSkin = std::max(layerHeight * 0.03F, planetRadius * 1e-7F);
    capsule_.stepHeight = layerHeight * 1.05F;
    capsule_.gravity = layerHeight * 120.0F;
    capsule_.jumpSpeed = std::sqrt(
        2.0F * capsule_.gravity * layerHeight * 2.0F);
    movementSpeed_ = width * 6.0F;
    footRadius_ = std::max(footRadius_, planetRadius);
}

void SurfaceCameraController::resetToSurface(float planetRadius,
                                             const TerrainQuery& terrain) {
    const SurfaceTerrainContact contact = terrain(radial_, capsule_.radius, 0.0F);
    footRadius_ = std::max(contact.floorRadius + capsule_.standingSkin,
                          planetRadius * 0.5F);
    verticalVelocity_ = 0.0F;
    grounded_ = true;
    terrainContactExact_ = contact.exact;
}

void SurfaceCameraController::simulatePlayer(float forwardInput, float rightInput,
                                             float deltaSeconds, float planetRadius,
                                             bool sprint, bool jump,
                                             const TerrainQuery& terrain) {
    if (!collisionEnabled_) {
        move(forwardInput, rightInput, deltaSeconds,
             planetRadius + clearance_, sprint);
        footRadius_ = planetRadius + clearance_ - capsule_.eyeHeight;
        grounded_ = false;
        terrainContactExact_ = false;
        return;
    }

    const float clampedDelta = std::clamp(deltaSeconds, 0.0F, 0.1F);
    const float inputLength = std::min(
        std::sqrt(forwardInput * forwardInput + rightInput * rightInput), 1.0F);
    const float speed = movementSpeed_ * (sprint ? 1.75F : 1.0F);
    const float movementDistance = speed * clampedDelta * inputLength;
    const std::uint32_t movementSteps = std::max(
        1U, static_cast<std::uint32_t>(std::ceil(
            movementDistance / std::max(capsule_.radius * 0.45F, 1e-7F))));
    const float stepDelta = movementSteps > 0U
        ? clampedDelta / static_cast<float>(movementSteps) : 0.0F;
    for (std::uint32_t step = 0U; step < movementSteps && inputLength > 1e-6F; ++step) {
        const Vector previousRadial = radial_;
        const Vector previousForward = tangentForward_;
        move(forwardInput, rightInput, stepDelta,
             std::max(footRadius_, planetRadius * 0.5F), sprint);
        const float stepSweep = speed * stepDelta * inputLength;
        const SurfaceTerrainContact destination = terrain(
            radial_, capsule_.radius, stepSweep);
        terrainContactExact_ = destination.exact;
        const float permittedRise = grounded_ ? capsule_.stepHeight
                                              : capsule_.standingSkin;
        if (destination.floorRadius + capsule_.standingSkin - footRadius_ >
            permittedRise) {
            radial_ = previousRadial;
            tangentForward_ = previousForward;
            break;
        }
        if (grounded_ && destination.floorRadius + capsule_.standingSkin > footRadius_) {
            footRadius_ = destination.floorRadius + capsule_.standingSkin;
        }
    }

    if (jump && grounded_) {
        verticalVelocity_ = capsule_.jumpSpeed;
        grounded_ = false;
    }
    verticalVelocity_ -= capsule_.gravity * clampedDelta;
    footRadius_ += verticalVelocity_ * clampedDelta;

    const SurfaceTerrainContact ground = terrain(radial_, capsule_.radius, 0.0F);
    terrainContactExact_ = ground.exact;
    const float standingRadius = ground.floorRadius + capsule_.standingSkin;
    if (footRadius_ <= standingRadius) {
        footRadius_ = standingRadius;
        verticalVelocity_ = 0.0F;
        grounded_ = true;
    } else {
        grounded_ = false;
    }
}

void SurfaceCameraController::teleportToCameraPose(
    const Vector& cameraPosition, const Vector& viewForward,
    const Vector& worldVelocity, float planetRadius,
    const TerrainQuery& terrain) {
    const float radiusSquared = dot(cameraPosition, cameraPosition);
    if (!(radiusSquared > 1e-12F) || !std::isfinite(radiusSquared)) {
        return;
    }
    const float cameraRadiusValue = std::sqrt(radiusSquared);
    radial_ = normalized(cameraPosition, radial_);
    const Vector normalizedForward = normalized(viewForward, tangentForward_);
    lookPitch_ = std::asin(std::clamp(dot(normalizedForward, radial_),
                                     -1.0F, 1.0F));
    const Vector projectedForward = added(
        normalizedForward, scaled(radial_, -dot(normalizedForward, radial_)));
    Vector fallbackForward{};
    Vector ignoredRight{};
    stableTangentBasis(radial_, fallbackForward, ignoredRight);
    tangentForward_ = normalized(projectedForward, fallbackForward);
    footRadius_ = std::max(cameraRadiusValue - capsule_.eyeHeight,
                           planetRadius * 0.5F);
    clearance_ = std::max(cameraRadiusValue - planetRadius, 0.0F);
    verticalVelocity_ = dot(worldVelocity, radial_);
    grounded_ = false;
    terrainContactExact_ = false;
    if (collisionEnabled_) {
        const SurfaceTerrainContact contact = terrain(radial_, capsule_.radius, 0.0F);
        const float standingRadius = contact.floorRadius + capsule_.standingSkin;
        terrainContactExact_ = contact.exact;
        if (footRadius_ <= standingRadius) {
            footRadius_ = standingRadius;
            verticalVelocity_ = 0.0F;
            grounded_ = true;
        }
    }
}

void SurfaceCameraController::setClearance(float value) noexcept {
    clearance_ = std::clamp(value, 0.0F, 20.0F);
}

void SurfaceCameraController::setMovementSpeed(float value) noexcept {
    movementSpeed_ = std::clamp(value, 0.01F, 10.0F);
}

void SurfaceCameraController::setMouseSensitivity(float value) noexcept {
    mouseSensitivity_ = std::clamp(value, 0.0005F, 0.03F);
}

void SurfaceCameraController::setCollisionEnabled(bool enabled) noexcept {
    collisionEnabled_ = enabled;
    verticalVelocity_ = 0.0F;
    grounded_ = false;
}

} // namespace voxel
