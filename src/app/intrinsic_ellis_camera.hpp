#pragma once

#include "render/global_handle_atlas.hpp"
#include "render/intrinsic_ellis_manifold.hpp"

#include <algorithm>
#include <cmath>

namespace voxel {

class IntrinsicEllisCamera final {
public:
    enum class MotionStatus {
        IntrinsicCore,
        PositiveAsymptoticContent,
        NegativeAsymptoticContent,
        RejectedNonFinite
    };

    IntrinsicEllisCamera() noexcept { reset({}); }

    void reset(const IntrinsicEllisSettings& settings) noexcept {
        globalMode_ = settings.globalMetricField &&
            !settings.globalNativeEllisPath;
        if (globalMode_) {
            globalState_ = globalHandleResetObserver(settings, 0U);
        }
        state_.properDepth = std::clamp(
            settings.contentExitProperDepth * 0.72F,
            settings.throatRadius * 0.25F,
            settings.contentExitProperDepth * 0.95F);
        state_.angularPosition = {0.0F, 0.0F, 1.0F};
        state_.velocity = {-1.0F, {}};
        const auto basis = ellisAngularBasis(state_.angularPosition);
        frame_.forward = {-1.0F, {}};
        frame_.right = {0.0F, basis[0]};
        frame_.up = {0.0F, basis[1]};
        velocity_ = {};
        lastMoveRejected_ = false;
        lastEnteredMouth_ = -1;
    }

    void resetGlobal(const IntrinsicEllisSettings& settings,
                     std::uint32_t mouthIndex) noexcept {
        reset(settings);
        if (settings.globalMetricField && !settings.globalNativeEllisPath) {
            globalState_ = globalHandleResetObserver(settings, mouthIndex);
        } else if (settings.globalMetricField && mouthIndex == 1U) {
            // Symmetric native reset on the negative end.  Decreasing l is
            // outward there, so the inward-facing test camera points +l.
            state_.properDepth = -state_.properDepth;
            state_.velocity = {1.0F, {}};
            frame_.forward = {1.0F, {}};
            const auto basis = ellisAngularBasis(state_.angularPosition);
            frame_.right = {0.0F, basis[0]};
            frame_.up = {0.0F, basis[1]};
            orthonormalizeFrame();
        }
    }

    void setState(const EllisState& state, const EllisFrame& frame) noexcept {
        if (!std::isfinite(state.properDepth) ||
            !portalFinite(state.angularPosition) ||
            !ellisFinite(state.velocity)) {
            return;
        }
        state_ = state;
        state_.angularPosition = portalNormalize(state_.angularPosition);
        state_.velocity = ellisNormalize(state_.velocity,
                                         state_.angularPosition);
        frame_ = frame;
        lastMoveRejected_ = false;
        lastEnteredMouth_ = -1;
        orthonormalizeFrame();
    }

    void rotate(float yawRadians, float pitchRadians,
                float rollRadians = 0.0F) noexcept {
        if (globalMode_) {
            const auto rotateAxis = [](PortalVector value, PortalVector axis,
                                       float angle) noexcept {
                axis = portalNormalize(axis);
                const float cosine = std::cos(angle);
                const float sine = std::sin(angle);
                return value * cosine + portalCross(axis, value) * sine +
                    axis * (portalDot(axis, value) * (1.0F - cosine));
            };
            PortalVector forward = globalState_.forward;
            PortalVector up = globalState_.up;
            PortalVector right = portalNormalize(portalCross(forward, up),
                                                  {1.0F, 0.0F, 0.0F});
            forward = portalNormalize(rotateAxis(forward, up, yawRadians),
                                      forward);
            right = portalNormalize(portalCross(forward, up), right);
            forward = portalNormalize(rotateAxis(forward, right, pitchRadians),
                                      forward);
            up = portalNormalize(rotateAxis(up, right, pitchRadians), up);
            up = portalNormalize(rotateAxis(up, forward, rollRadians), up);
            globalState_.forward = forward;
            globalState_.up = portalOrthonormalUp(forward, up);
            return;
        }
        PortalVector forward = ellisTangentToLocal(
            frame_.forward, state_.angularPosition);
        PortalVector up = ellisTangentToLocal(
            frame_.up, state_.angularPosition);
        PortalVector right = portalNormalize(portalCross(forward, up),
                                              {0.0F, 1.0F, 0.0F});
        const auto rotateAxis = [](PortalVector value, PortalVector axis,
                                   float angle) noexcept {
            axis = portalNormalize(axis);
            const float cosine = std::cos(angle);
            const float sine = std::sin(angle);
            return value * cosine + portalCross(axis, value) * sine +
                axis * (portalDot(axis, value) * (1.0F - cosine));
        };
        forward = portalNormalize(rotateAxis(forward, up, yawRadians),
                                  forward);
        up = portalNormalize(rotateAxis(up, up, yawRadians), up);
        right = portalNormalize(portalCross(forward, up), right);
        forward = portalNormalize(rotateAxis(forward, right, pitchRadians),
                                  forward);
        up = portalNormalize(rotateAxis(up, right, pitchRadians), up);
        up = portalNormalize(rotateAxis(up, forward, rollRadians), up);
        right = portalNormalize(portalCross(forward, up), right);
        up = portalNormalize(portalCross(right, forward), up);
        frame_.forward = ellisTangentFromLocal(forward,
                                               state_.angularPosition);
        frame_.right = ellisTangentFromLocal(right,
                                             state_.angularPosition);
        frame_.up = ellisTangentFromLocal(up, state_.angularPosition);
        orthonormalizeFrame();
    }

    void move(float forwardInput, float rightInput, float verticalInput,
              float deltaSeconds, bool sprint,
              const IntrinsicEllisSettings& settings) noexcept {
        if (globalMode_) {
            const float delta = std::clamp(deltaSeconds, 0.0F, 0.1F);
            const PortalVector right = portalNormalize(portalCross(
                globalState_.forward, globalState_.up),
                {1.0F, 0.0F, 0.0F});
            PortalVector input = globalState_.forward * forwardInput +
                right * rightInput + globalState_.up * verticalInput;
            const float magnitude = portalLength(input);
            if (!(magnitude > 1.0e-6F) || !(delta > 0.0F)) {
                globalState_.velocity = {};
                return;
            }
            input = input * (1.0F / std::max(magnitude, 1.0F));
            const float speed = settings.freeFlySpeed *
                (sprint ? settings.sprintMultiplier : 1.0F);
            globalState_.properSpeed = speed;
            globalHandleAdvanceObserver(globalState_, input, speed * delta,
                                        settings);
            lastMoveRejected_ = !globalState_.finite;
            lastEnteredMouth_ = globalState_.crossings != globalCrossingsSeen_
                ? static_cast<int>(globalState_.lastMouth == 0U ? 1U : 0U)
                : -1;
            globalCrossingsSeen_ = globalState_.crossings;
            return;
        }
        const float delta = std::clamp(deltaSeconds, 0.0F, 0.1F);
        EllisTangent input = frame_.forward * forwardInput +
            frame_.right * rightInput + frame_.up * verticalInput;
        const float magnitude = ellisLength(input);
        if (!(magnitude > 1.0e-6F) || !(delta > 0.0F)) {
            velocity_ = {};
            return;
        }
        input = ellisNormalize(input * (1.0F / std::max(magnitude, 1.0F)),
                               state_.angularPosition, frame_.forward);
        const float speed = settings.freeFlySpeed *
            (sprint ? settings.sprintMultiplier : 1.0F);
        velocity_ = input * speed;
        state_.velocity = input;
        const EllisState previousState = state_;
        const EllisFrame previousFrame = frame_;
        lastEnteredMouth_ = -1;
        // One local-orthonormal tetrad path everywhere. `speed * delta` is
        // proper distance, and RK substeps naturally split the l=0 chart
        // crossing instead of changing controller units at a region flag.
        ellisIntegrateGeodesic(state_, frame_, speed * delta,
                               settings.throatRadius, 128U,
                               settings.globalMetricField);
        // The camera state is never converted to an endpoint Euclidean pose.
        // It may pass l=0 and continue without limit into either asymptotic
        // content chart. The renderer evaluates the same all-space metric for
        // every ray rather than re-entering through an influence sphere.
        if (!std::isfinite(state_.properDepth) ||
            !portalFinite(state_.angularPosition) ||
            !ellisFinite(state_.velocity) ||
            !ellisFinite(frame_.forward) || !ellisFinite(frame_.right) ||
            !ellisFinite(frame_.up)) {
            state_ = previousState;
            frame_ = previousFrame;
            velocity_ = {};
            lastMoveRejected_ = true;
            return;
        }
        const IntrinsicEllisMouthEntry mouthEntry = settings.globalMetricField
            ? IntrinsicEllisMouthEntry{}
            : intrinsicEllisOtherMouthEntry(settings, previousState,
                                            previousFrame, state_, frame_);
        if (mouthEntry.entered) {
            const PortalVector worldForward =
                intrinsicEllisTangentToContent(settings, state_,
                                               frame_.forward);
            const PortalVector worldRight =
                intrinsicEllisTangentToContent(settings, state_,
                                               frame_.right);
            const PortalVector worldUp =
                intrinsicEllisTangentToContent(settings, state_, frame_.up);
            const PortalVector worldVelocity =
                intrinsicEllisTangentToContent(settings, state_,
                                               state_.velocity);
            const float inset = std::max(settings.throatRadius * 1.0e-4F,
                                         1.0e-6F);
            state_.properDepth = mouthEntry.positiveEnd
                ? settings.contentExitProperDepth - inset
                : -settings.contentExitProperDepth + inset;
            state_.angularPosition = mouthEntry.intrinsicAngular;
            state_.velocity = ellisNormalize(
                intrinsicEllisTangentFromContent(
                    settings, mouthEntry.positiveEnd,
                    state_.angularPosition, worldVelocity),
                state_.angularPosition);
            frame_.forward = intrinsicEllisTangentFromContent(
                settings, mouthEntry.positiveEnd,
                state_.angularPosition, worldForward);
            frame_.right = intrinsicEllisTangentFromContent(
                settings, mouthEntry.positiveEnd,
                state_.angularPosition, worldRight);
            frame_.up = intrinsicEllisTangentFromContent(
                settings, mouthEntry.positiveEnd,
                state_.angularPosition, worldUp);
            lastEnteredMouth_ = mouthEntry.positiveEnd ? 0 : 1;
        }
        lastMoveRejected_ = false;
        orthonormalizeFrame();
    }

    [[nodiscard]] const EllisState& state() const noexcept { return state_; }
    [[nodiscard]] const EllisFrame& frame() const noexcept { return frame_; }
    [[nodiscard]] const EllisTangent& velocity() const noexcept {
        return velocity_;
    }
    [[nodiscard]] float handedness() const noexcept {
        if (globalMode_) {
            const PortalVector right = portalNormalize(portalCross(
                globalState_.forward, globalState_.up));
            return portalDot(portalCross(right, globalState_.forward),
                             globalState_.up);
        }
        return ellisFrameHandedness(frame_, state_.angularPosition);
    }
    [[nodiscard]] const GlobalHandleObserverState& globalState() const noexcept {
        return globalState_;
    }
    void setGlobalState(const GlobalHandleObserverState& state) noexcept {
        if (!globalMode_ || !state.finite || !portalFinite(state.position) ||
            !portalFinite(state.forward) || !portalFinite(state.up)) return;
        globalState_ = state;
        globalState_.forward = portalNormalize(globalState_.forward);
        globalState_.up = portalOrthonormalUp(globalState_.forward,
                                              globalState_.up);
        globalCrossingsSeen_ = globalState_.crossings;
    }
    [[nodiscard]] MotionStatus motionStatus(
        const IntrinsicEllisSettings& settings) const noexcept {
        if (lastMoveRejected_) {
            return MotionStatus::RejectedNonFinite;
        }
        if (state_.properDepth > settings.contentExitProperDepth) {
            return MotionStatus::PositiveAsymptoticContent;
        }
        if (state_.properDepth < -settings.contentExitProperDepth) {
            return MotionStatus::NegativeAsymptoticContent;
        }
        return MotionStatus::IntrinsicCore;
    }
    [[nodiscard]] const char* motionStatusText(
        const IntrinsicEllisSettings& settings) const noexcept {
        switch (motionStatus(settings)) {
        case MotionStatus::PositiveAsymptoticContent:
            return "+end all-space Ellis asymptote (unclamped)";
        case MotionStatus::NegativeAsymptoticContent:
            return "-end all-space Ellis asymptote (unclamped)";
        case MotionStatus::RejectedNonFinite:
            return "movement rejected: non-finite safety guard";
        default:
            return "intrinsic Ellis core (unclamped)";
        }
    }
    [[nodiscard]] int lastEnteredMouth() const noexcept {
        return lastEnteredMouth_;
    }
    [[nodiscard]] const char* lastEnteredMouthText() const noexcept {
        return lastEnteredMouth_ == 0 ? "A (+l)" :
            lastEnteredMouth_ == 1 ? "B (-l)" : "none this step";
    }

private:
    void orthonormalizeFrame() noexcept {
        frame_.forward = ellisNormalize(frame_.forward,
                                        state_.angularPosition,
                                        {-1.0F, {}});
        frame_.right = ellisNormalize(
            frame_.right - frame_.forward *
                ellisDot(frame_.right, frame_.forward),
            state_.angularPosition,
            ellisTangentFromLocal({0.0F, 1.0F, 0.0F},
                                   state_.angularPosition));
        frame_.up = ellisNormalize(
            frame_.up - frame_.forward *
                ellisDot(frame_.up, frame_.forward) -
                frame_.right * ellisDot(frame_.up, frame_.right),
            state_.angularPosition,
            ellisTangentFromLocal({0.0F, 0.0F, 1.0F},
                                   state_.angularPosition));
        // The negative-end attachment reverses its outward radial coordinate.
        // Reconstruct the unused lateral sign from forward/up so crossing a
        // shared-chart mouth cannot mirror the observer tetrad.
        if (ellisFrameHandedness(frame_, state_.angularPosition) < 0.0F) {
            frame_.right = frame_.right * -1.0F;
        }
    }

    EllisState state_{};
    EllisFrame frame_{};
    EllisTangent velocity_{};
    GlobalHandleObserverState globalState_{};
    std::uint32_t globalCrossingsSeen_{};
    bool globalMode_{};
    bool lastMoveRejected_{};
    int lastEnteredMouth_{-1};
};

} // namespace voxel
