#pragma once

#include "render/portal_lab.hpp"

#include <algorithm>
#include <cmath>

namespace voxel {

class PortalFreeFlyCamera final {
public:
    void resetForPortal(const PortalLabSettings& settings,
                        float planetRadius) noexcept {
        const float radius = std::max(planetRadius, 1.0e-6F);
        const PortalVector center = portalEndpoint(
            settings.endpointAAzimuth, settings.endpointAElevation,
            settings.endpointADistance) * radius;
        const PortalFrame frame = portalFrame(center);
        position_ = center + frame.z *
            ((settings.influenceRadius + 0.18F) * radius);
        forward_ = portalNormalize(center - position_, frame.z * -1.0F);
        up_ = portalOrthonormalUp(forward_, frame.y);
        velocity_ = {};
    }

    void setPose(PortalVector position, PortalVector forward, PortalVector up,
                 PortalVector velocity = {}) noexcept {
        if (!portalFinite(position) || !portalFinite(forward) ||
            !portalFinite(up) || !portalFinite(velocity)) {
            return;
        }
        position_ = position;
        forward_ = portalNormalize(forward, forward_);
        up_ = portalOrthonormalUp(forward_, up);
        velocity_ = velocity;
    }

    void rotate(float yawRadians, float pitchRadians,
                float rollRadians = 0.0F) noexcept {
        const PortalVector right = portalNormalize(
            portalCross(forward_, up_), {1.0F, 0.0F, 0.0F});
        const auto axisRotation = [](PortalVector value, PortalVector axis,
                                     float radians) noexcept {
            axis = portalNormalize(axis);
            const float cosine = std::cos(radians);
            const float sine = std::sin(radians);
            return value * cosine + portalCross(axis, value) * sine +
                   axis * (portalDot(axis, value) * (1.0F - cosine));
        };
        forward_ = portalNormalize(axisRotation(forward_, up_, yawRadians),
                                   forward_);
        PortalVector rotatedUp = axisRotation(up_, up_, yawRadians);
        const PortalVector yawedRight = portalNormalize(
            portalCross(forward_, rotatedUp), right);
        forward_ = portalNormalize(
            axisRotation(forward_, yawedRight, pitchRadians), forward_);
        rotatedUp = portalOrthonormalUp(forward_,
            axisRotation(rotatedUp, yawedRight, pitchRadians));
        up_ = portalOrthonormalUp(forward_,
            axisRotation(rotatedUp, forward_, rollRadians));
    }

    void move(float forwardInput, float rightInput, float verticalInput,
              float deltaSeconds, bool sprint) noexcept {
        const float clampedDelta = std::clamp(deltaSeconds, 0.0F, 0.1F);
        const PortalVector right = portalNormalize(
            portalCross(forward_, up_), {1.0F, 0.0F, 0.0F});
        PortalVector input = forward_ * forwardInput + right * rightInput +
                             up_ * verticalInput;
        const float magnitude = portalLength(input);
        if (!(magnitude > 1.0e-6F) || !(clampedDelta > 0.0F)) {
            velocity_ = {};
            return;
        }
        input = input * (1.0F / std::max(magnitude, 1.0F));
        const float actualSpeed = speed_ * (sprint ? sprintMultiplier_ : 1.0F);
        velocity_ = input * actualSpeed;
        position_ = position_ + velocity_ * clampedDelta;
    }

    [[nodiscard]] const PortalVector& position() const noexcept { return position_; }
    [[nodiscard]] const PortalVector& forward() const noexcept { return forward_; }
    [[nodiscard]] const PortalVector& up() const noexcept { return up_; }
    [[nodiscard]] const PortalVector& velocity() const noexcept { return velocity_; }
    [[nodiscard]] float speed() const noexcept { return speed_; }
    [[nodiscard]] float sprintMultiplier() const noexcept { return sprintMultiplier_; }
    [[nodiscard]] float mouseSensitivity() const noexcept { return mouseSensitivity_; }

    void setSpeed(float value) noexcept {
        speed_ = std::clamp(value, 0.01F, 10.0F);
    }
    void setSprintMultiplier(float value) noexcept {
        sprintMultiplier_ = std::clamp(value, 1.0F, 8.0F);
    }
    void setMouseSensitivity(float value) noexcept {
        mouseSensitivity_ = std::clamp(value, 0.0005F, 0.03F);
    }

private:
    PortalVector position_{0.0F, 0.0F, 3.0F};
    PortalVector forward_{0.0F, 0.0F, -1.0F};
    PortalVector up_{0.0F, 1.0F, 0.0F};
    PortalVector velocity_{};
    float speed_{0.30F};
    float sprintMultiplier_{3.0F};
    float mouseSensitivity_{0.0035F};
};

} // namespace voxel
