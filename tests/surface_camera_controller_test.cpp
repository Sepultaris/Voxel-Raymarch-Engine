#include "app/surface_camera_controller.hpp"
#include "planet/fractal_planet_sdf.hpp"
#include "render/render_settings.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

using Vector = voxel::SurfaceCameraController::Vector;

[[nodiscard]] float dot(const Vector& left, const Vector& right) {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

[[nodiscard]] float length(const Vector& value) {
    return std::sqrt(dot(value, value));
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void verifyFrame(const voxel::SurfaceCameraController& controller) {
    const Vector& radial = controller.radial();
    const Vector& forward = controller.tangentForward();
    require(std::isfinite(length(radial)) && std::isfinite(length(forward)),
            "Surface camera frame became non-finite");
    require(std::abs(length(radial) - 1.0F) < 2e-5F,
            "Surface camera radial direction lost unit length");
    require(std::abs(length(forward) - 1.0F) < 2e-5F,
            "Surface camera forward direction lost unit length");
    require(std::abs(dot(radial, forward)) < 2e-5F,
            "Surface camera forward direction left the tangent plane");
}

} // namespace

int main() {
    try {
        voxel::SurfaceCameraController controller;
        controller.configureVoxelGeometry(0.01F, 0.005F, 1.0F);
        controller.setMovementSpeed(0.05F);

        // Begin almost at the north pole, cross it repeatedly, and mix heading
        // changes with lateral great-circle motion. No longitude coordinate is
        // stored, so neither the pole nor anti-meridian is a special case.
        controller.resetFromOrbit(3.13F, 1.44F);
        for (int step = 0; step < 20'000; ++step) {
            const float forward = step % 7 < 4 ? 1.0F : -0.35F;
            const float right = step % 11 < 5 ? 0.65F : -0.45F;
            controller.rotateHeading((step % 3 == 0 ? 1.0F : -0.4F) * 0.0015F);
            controller.move(forward, right, 1.0F / 120.0F, 1.02F, step % 97 == 0);
            verifyFrame(controller);
        }

        controller.rotateLook(100.0F);
        require(controller.lookPitch() <= 1.45F,
                "First-person look exceeded its upper pole-safe limit");
        controller.rotateLook(-200.0F);
        require(controller.lookPitch() >= -1.45F,
                "First-person look exceeded its lower pole-safe limit");

        const float clearance = voxel::minimumSurfaceClearance(1.0F);
        require(clearance >= 0.01F,
                "Surface player clearance is below the radial safety margin");
        require(1.001F + clearance > 1.001F,
                "Surface player can enter the conservative outer geometry");

        const auto flatTerrain = [](const Vector&, float, float) {
            return voxel::SurfaceTerrainContact{1.0F, 0U, true};
        };
        controller.resetFromOrbit(0.0F, 0.0F);
        controller.resetToSurface(1.0F, flatTerrain);
        for (int tick = 0; tick < 240; ++tick) {
            controller.simulatePlayer(0.0F, 0.0F, 1.0F / 120.0F,
                                      1.0F, false, false, flatTerrain);
        }
        require(controller.grounded(), "Capsule did not settle on a flat voxel cap");
        require(controller.footRadius() >= 1.0F,
                "Capsule foot penetrated its resting terrain cap");
        require(controller.cameraRadius() > controller.footRadius() &&
                controller.cameraRadius() <
                    controller.footRadius() + controller.capsule().height,
                "First-person eye is not inside the standing capsule");
        require(std::abs(controller.capsule().height - 0.02F) < 1e-6F,
                "Capsule is not exactly four radial voxel layers tall");

        // A conventional jump must leave the cap, reach a positive apex, and
        // settle back onto the same exact profile without tunneling.
        const float standingFoot = controller.footRadius();
        controller.simulatePlayer(0.0F, 0.0F, 1.0F / 120.0F,
                                  1.0F, false, true, flatTerrain);
        float maximumFoot = controller.footRadius();
        for (int tick = 0; tick < 360; ++tick) {
            controller.simulatePlayer(0.0F, 0.0F, 1.0F / 120.0F,
                                      1.0F, false, false, flatTerrain);
            maximumFoot = std::max(maximumFoot, controller.footRadius());
        }
        require(maximumFoot > standingFoot + 0.004F,
                "Jump did not clear a meaningful voxel-layer height");
        require(controller.grounded() &&
                std::abs(controller.footRadius() - standingFoot) < 1e-5F,
                "Jump did not land deterministically on the voxel cap");

        // A linked-portal camera pose is revalidated against the same exact
        // terrain query before the next simulation step. A mapped capsule may
        // never remain below its destination cap.
        controller.teleportToCameraPose(
            {0.0F, 0.0F, 0.90F}, {0.0F, 1.0F, 0.0F},
            {0.0F, 0.0F, -0.1F}, 1.0F, flatTerrain);
        require(controller.grounded() && controller.terrainContactExact() &&
                    controller.footRadius() >= 1.0F,
                "Portal destination collision did not recover terrain penetration");
        verifyFrame(controller);
        controller.teleportToCameraPose(
            {0.0F, 0.0F, 1.20F}, {0.0F, 0.4F, -0.9F},
            {0.0F, 0.0F, -0.1F}, 1.0F, flatTerrain);
        require(!controller.grounded() && controller.verticalVelocity() < 0.0F &&
                    controller.terrainContactExact(),
                "Portal destination did not preserve airborne radial velocity");
        verifyFrame(controller);

        // One-layer steps are walkable, while a four-layer cliff blocks the
        // swept capsule before its center crosses the boundary.
        controller.resetFromOrbit(0.0F, 0.0F);
        const auto oneLayerStep = [](const Vector& radial, float, float) {
            return voxel::SurfaceTerrainContact{
                radial[1] > 0.002F ? 1.005F : 1.0F, 0U, true};
        };
        controller.resetToSurface(1.0F, oneLayerStep);
        for (int tick = 0; tick < 90; ++tick) {
            controller.simulatePlayer(1.0F, 0.0F, 1.0F / 120.0F,
                                      1.0F, false, false, oneLayerStep);
        }
        require(controller.radial()[1] > 0.002F &&
                controller.footRadius() >= 1.005F,
                "Capsule failed to climb a legal one-layer voxel step");

        controller.resetFromOrbit(0.0F, 0.0F);
        const auto cliff = [](const Vector& radial, float, float) {
            return voxel::SurfaceTerrainContact{
                radial[1] > 0.002F ? 1.02F : 1.0F, 0U, true};
        };
        controller.resetToSurface(1.0F, cliff);
        for (int tick = 0; tick < 120; ++tick) {
            controller.simulatePlayer(1.0F, 0.0F, 1.0F / 120.0F,
                                      1.0F, false, false, cliff);
        }
        require(controller.radial()[1] <= 0.0025F,
                "Swept capsule crossed an over-height voxel cliff");

        // Radial gravity follows a descending profile instead of snapping the
        // player to a smooth sphere.
        controller.resetFromOrbit(0.0F, -0.02F);
        const auto downhill = [](const Vector& radial, float, float) {
            return voxel::SurfaceTerrainContact{
                radial[1] < 0.0F ? 1.004F : 1.0F, 0U, true};
        };
        controller.resetToSurface(1.0F, downhill);
        const float highGround = controller.footRadius();
        for (int tick = 0; tick < 240; ++tick) {
            controller.simulatePlayer(1.0F, 0.0F, 1.0F / 120.0F,
                                      1.0F, false, false, downhill);
        }
        require(controller.radial()[1] > 0.0F &&
                controller.footRadius() < highGround - 0.003F &&
                controller.grounded(),
                "Radial gravity did not settle the capsule downhill");

        // The topology query uses compact occupied-layer counts and becomes
        // conservatively maximum-height until GPU feedback marks them fresh.
        voxel::GeodesicTopology topology = voxel::GeodesicTopology::build(
            8U, 32U, 64U, 32U, 64U, false);
        for (auto& state : topology.columnStates) {
            state.data = {0x000003ffU, 10U, 2U, 0U};
        }
        std::vector<std::uint8_t> fresh(topology.tiles.size(), 1U);
        const auto exactContact = voxel::queryGeodesicTerrain(
            topology, fresh, {0.0F, 0.0F, 1.0F}, 1.0F, 0.01F);
        const auto adaptiveContact = voxel::queryGeodesicTerrain(
            topology, fresh, {0.0F, 0.0F, 1.0F}, 1.0F, 0.01F,
            0.0F, true, 1337U, 9U, 0.15F);
        const auto repeatedAdaptiveContact = voxel::queryGeodesicTerrain(
            topology, fresh, {0.0F, 0.0F, 1.0F}, 1.0F, 0.01F,
            0.0F, true, 1337U, 9U, 0.15F);
        require(adaptiveContact.exact &&
                    adaptiveContact.floorRadius <= exactContact.floorRadius &&
                    adaptiveContact.floorRadius == repeatedAdaptiveContact.floorRadius,
                "Finest local adaptive-SDF collision is not exact and deterministic");
        const auto fractalContact = voxel::queryGeodesicTerrain(
            topology, fresh, {0.0F, 0.0F, 1.0F}, 1.0F, 0.01F,
            0.0F, false, 1337U, 9U, 0.15F,
            true, 1337U, 10U, 0.012F);
        const auto repeatedFractalContact = voxel::queryGeodesicTerrain(
            topology, fresh, {0.0F, 0.0F, 1.0F}, 1.0F, 0.01F,
            0.0F, false, 1337U, 9U, 0.15F,
            true, 1337U, 10U, 0.012F);
        require(fractalContact.exact &&
                    fractalContact.floorRadius >=
                        voxel::kFractalPlanetSeaRadiusScale &&
                    fractalContact.floorRadius <=
                        voxel::kFractalPlanetOuterRadiusScale &&
                    fractalContact.floorRadius ==
                        repeatedFractalContact.floorRadius,
                "Finest local true-fractal collision is not bounded and deterministic");
        for (auto& state : topology.columnStates) {
            state.data[3] = 1U;
        }
        const auto editedContact = voxel::queryGeodesicTerrain(
            topology, fresh, {0.0F, 0.0F, 1.0F}, 1.0F, 0.01F,
            0.0F, true, 1337U, 9U, 0.15F);
        require(editedContact.floorRadius == exactContact.floorRadius,
                "Edited columns did not override adaptive procedural collision");
        const auto editedFractalContact = voxel::queryGeodesicTerrain(
            topology, fresh, {0.0F, 0.0F, 1.0F}, 1.0F, 0.01F,
            0.0F, false, 1337U, 9U, 0.15F,
            true, 1337U, 10U, 0.012F);
        require(std::abs(editedFractalContact.floorRadius -
                         exactContact.floorRadius) < 1e-6F,
                "Edited macro columns did not override the true fractal field");
        for (auto& state : topology.columnStates) {
            state.data[3] = 0U;
        }
        std::fill(fresh.begin(), fresh.end(), 0U);
        const auto conservativeContact = voxel::queryGeodesicTerrain(
            topology, fresh, {0.0F, 0.0F, 1.0F}, 1.0F, 0.01F);
        require(exactContact.exact && !conservativeContact.exact &&
                conservativeContact.floorRadius > exactContact.floorRadius,
                "GPU profile freshness does not fail conservatively");

        std::cout << "Spherical capsule-player collision invariants passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Surface camera controller gate failed: " << error.what() << '\n';
        return 1;
    }
}
