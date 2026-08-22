#include "render/render_settings.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

} // namespace

int main() {
    try {
        constexpr float planetRadius = 1.0F;
        constexpr float conservativeOuterScale = 1.001F;
        const float minimum = voxel::minimumCameraDistance(
            planetRadius, conservativeOuterScale);
        require(minimum > planetRadius * conservativeOuterScale,
                "Close zoom can place the camera inside the conservative planet bound");
        require(minimum < 1.02F,
                "Close zoom retains too much clearance from the planetary surface");

        const float largePlanetMinimum = voxel::minimumCameraDistance(
            2.0F, conservativeOuterScale);
        require(largePlanetMinimum > 2.0F * conservativeOuterScale,
                "Zoom floor does not scale with planet radius");
        require(std::abs(largePlanetMinimum - 2.0F * minimum) < 1e-5F,
                "Relative surface clearance changes with planet radius");
        require(voxel::cameraZoomStep(1.01F) < voxel::cameraZoomStep(3.0F),
                "Mouse-wheel zoom does not become finer near the surface");

        std::cout << "Close-surface camera zoom invariants passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Camera zoom gate failed: " << error.what() << '\n';
        return 1;
    }
}
