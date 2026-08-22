#pragma once

#include "app/surface_player_collision.hpp"

#include <array>
#include <functional>

namespace voxel {

class SurfaceCameraController final {
public:
    using Vector = std::array<float, 3>;
    using TerrainQuery = std::function<SurfaceTerrainContact(
        const Vector&, float capsuleRadius, float sweepDistance)>;

    struct CapsuleGeometry {
        float radius{};
        float height{};
        float eyeHeight{};
        float standingSkin{};
        float stepHeight{};
        float gravity{};
        float jumpSpeed{};
    };

    void resetFromOrbit(float yaw, float pitch) noexcept;
    void rotateHeading(float radians) noexcept;
    void rotateLook(float radians) noexcept;
    void move(float forwardInput, float rightInput, float deltaSeconds,
              float cameraRadius, bool sprint) noexcept;
    void adjustClearance(float delta) noexcept;
    void configureVoxelGeometry(float surfaceCellWidth, float radialLayerHeight,
                                float planetRadius) noexcept;
    void resetToSurface(float planetRadius, const TerrainQuery& terrain);
    void simulatePlayer(float forwardInput, float rightInput, float deltaSeconds,
                        float planetRadius, bool sprint, bool jump,
                        const TerrainQuery& terrain);
    void teleportToCameraPose(const Vector& cameraPosition,
                              const Vector& viewForward,
                              const Vector& worldVelocity,
                              float planetRadius,
                              const TerrainQuery& terrain);

    [[nodiscard]] const Vector& radial() const noexcept { return radial_; }
    [[nodiscard]] const Vector& tangentForward() const noexcept { return tangentForward_; }
    [[nodiscard]] float lookPitch() const noexcept { return lookPitch_; }
    [[nodiscard]] float clearance() const noexcept { return clearance_; }
    [[nodiscard]] float movementSpeed() const noexcept { return movementSpeed_; }
    [[nodiscard]] float mouseSensitivity() const noexcept { return mouseSensitivity_; }
    [[nodiscard]] const CapsuleGeometry& capsule() const noexcept { return capsule_; }
    [[nodiscard]] float footRadius() const noexcept { return footRadius_; }
    [[nodiscard]] float cameraRadius() const noexcept {
        return footRadius_ + capsule_.eyeHeight;
    }
    [[nodiscard]] float verticalVelocity() const noexcept { return verticalVelocity_; }
    [[nodiscard]] bool grounded() const noexcept { return grounded_; }
    [[nodiscard]] bool collisionEnabled() const noexcept { return collisionEnabled_; }
    [[nodiscard]] bool terrainContactExact() const noexcept { return terrainContactExact_; }

    void setClearance(float value) noexcept;
    void setMovementSpeed(float value) noexcept;
    void setMouseSensitivity(float value) noexcept;
    void setCollisionEnabled(bool enabled) noexcept;

private:
    Vector radial_{0.0F, 0.0F, 1.0F};
    Vector tangentForward_{0.0F, 1.0F, 0.0F};
    float lookPitch_{};
    float clearance_{0.025F};
    float movementSpeed_{0.35F};
    float mouseSensitivity_{0.006F};
    CapsuleGeometry capsule_{0.00135F, 0.006F, 0.00539F, 0.00005F,
                             0.001575F, 0.18F, 0.0465F};
    float footRadius_{1.0F};
    float verticalVelocity_{};
    bool grounded_{};
    bool collisionEnabled_{true};
    bool terrainContactExact_{};
};

} // namespace voxel
