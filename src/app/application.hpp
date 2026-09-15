#pragma once

#include "audio/audio_engine.hpp"
#include "app/intrinsic_ellis_camera.hpp"
#include "app/portal_free_fly_camera.hpp"
#include "app/surface_camera_controller.hpp"
#include "physics/physics_engine.hpp"
#include "render/render_settings.hpp"

#include <memory>
#include <cstdint>
#include <filesystem>

struct SDL_Window;

namespace voxel {

class VulkanRenderer;

class Application final {
public:
    explicit Application(bool forceTopologyRegeneration = false,
                         bool headlessCapture = false);
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    int run(std::uint32_t maximumRenderedFrames = 0U, bool requirePageStreaming = false,
            bool rotateCameraForTest = false,
            std::uint32_t traversalMode = 0xffffffffU,
            bool exerciseSurfaceCamera = false,
            bool artifactRegression = false);
    void setTerrainMicroSdfEnabled(bool enabled) noexcept {
        renderSettings_.terrainMicroSdfEnabled = enabled;
    }
    void setGlobalSpatialAaEnabled(bool enabled) noexcept {
        renderSettings_.intrinsicEllis.globalSpatialAaEnabled = enabled;
        syncIntrinsicEllisRenderSettings();
    }
    void setGlobalNoHandleReference(bool enabled) noexcept {
        renderSettings_.intrinsicEllis.debugMode = enabled ? 12U : 0U;
        syncIntrinsicEllisRenderSettings();
    }
    void beginAdaptiveSdfInspection() {
        setCameraMode(CameraControllerMode::SurfaceTraversal, false);
        surfaceCamera_.rotateLook(-0.18F);
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
        // Start with the readable materialized-leaf shading. The explicit
        // cache/grid diagnostics remain available in the lab UI.
        renderSettings_.fractalPlanetSdfDebugMode = 0U;
#endif
        syncSurfaceRenderSettings();
    }
    void beginIntrinsicEllisTraversalRegression() noexcept;
    void beginGlobalMetricVisualRegression(bool mediaEnabled) noexcept;
    void setGlobalLensCapturePreset(std::uint32_t preset) noexcept;
    void setGlobalCaptureTailScale(float tailScale) noexcept;
    void setGlobalCaptureCameraPreset(std::uint32_t preset) noexcept;
    void setGlobalCaptureDebugMode(std::uint32_t mode) noexcept;
    void setEightPlanetCapturePreset(std::uint32_t preset) noexcept {
        renderSettings_.visualizationMode = 101U + (preset < 7U ? preset : 6U);
    }
    [[nodiscard]] bool captureFrame(const std::filesystem::path& outputPath,
                                    std::string& error);
    [[nodiscard]] bool headlessWindowStayedHidden() const noexcept;

private:
    void buildDevelopmentUi(float deltaSeconds);
    void queueVoxelEdit(int mouseX, int mouseY, std::uint32_t mode);
    void setCameraMode(CameraControllerMode mode, bool captureSurfaceMouse);
    void setSurfaceMouseCapture(bool enabled) noexcept;
    void updateSurfaceController(float deltaSeconds);
    void updatePortalFreeFlyController(float deltaSeconds);
    void updateIntrinsicEllisController(float deltaSeconds);
    void updatePortalCameraTraversal(float deltaSeconds);
    void syncSurfaceRenderSettings() noexcept;
    void syncPortalFreeFlyRenderSettings() noexcept;
    void syncIntrinsicEllisRenderSettings() noexcept;

    SDL_Window* window_{};
    bool headlessCapture_{};
    std::unique_ptr<VulkanRenderer> renderer_;
    AudioEngine audio_;
    PhysicsEngine physics_;
    RenderSettings renderSettings_{};
    bool running_{true};
    bool orbitingCamera_{};
    bool surfaceMouseCaptured_{};
    bool surfaceJumpHeld_{};
    SurfaceCameraController surfaceCamera_{};
    PortalFreeFlyCamera portalFreeFlyCamera_{};
    IntrinsicEllisCamera intrinsicEllisCamera_{};
    PortalVector previousPortalCameraPosition_{};
    PortalVector previousPortalEndpointA_{};
    PortalVector previousPortalEndpointB_{};
    PortalManifoldBodyState portalCameraManifold_{};
    std::uint64_t portalCameraTransferCount_{};
    std::uint32_t portalLastSourceEndpoint_{2U};
    bool portalCameraPositionInitialized_{};
    bool intrinsicTraversalRegression_{};
    bool intrinsicTraversalCrossed_{};
    std::uint32_t intrinsicTraversalNativeCrossingCount_{};
    bool intrinsicTraversalFinite_{true};
    std::uint64_t intrinsicTraversalNonfiniteRays_{};
    float intrinsicTraversalMaximumFrameDelta_{};
#if VOXEL_GLOBAL_METRIC_LAB
    std::uint32_t intrinsicTraversalCoverageFrames_{};
    std::uint32_t intrinsicTraversalZeroTerrainFrames_{};
    std::uint32_t intrinsicTraversalCurrentZeroTerrainRun_{};
    std::uint32_t intrinsicTraversalMaximumZeroTerrainRun_{};
    std::uint32_t intrinsicTraversalInvalidZeroTerrainFrames_{};
    std::uint64_t intrinsicTraversalAffineBudgetExhaustions_{};
    std::uint32_t intrinsicTraversalOwnerMismatchFrames_{};
    float intrinsicTraversalMaximumExitOriginDelta_{};
    float intrinsicTraversalMaximumExitDirectionDelta_{};
    bool intrinsicTraversalPreviousCenterValid_{};
    float intrinsicTraversalPreviousCenterR_{};
    float intrinsicTraversalPreviousCenterG_{};
    float intrinsicTraversalPreviousCenterB_{};
    float intrinsicTraversalPreviousMappedOriginX_{};
    float intrinsicTraversalPreviousMappedOriginY_{};
    float intrinsicTraversalPreviousMappedOriginZ_{};
    float intrinsicTraversalPreviousMappedDirectionX_{};
    float intrinsicTraversalPreviousMappedDirectionY_{};
    float intrinsicTraversalPreviousMappedDirectionZ_{};
    float intrinsicTraversalPreviousStarFootprint_{};
    bool intrinsicTraversalPreviousCenterHit_{};
    std::uint32_t intrinsicTraversalPreviousCenterHitClass_{};
    float intrinsicTraversalMaximumCenterHdrDelta_{};
    float intrinsicTraversalMaximumCenterMappedOriginDelta_{};
    float intrinsicTraversalMaximumCenterMappedDirectionDelta_{};
    float intrinsicTraversalMaximumCenterFootprintDelta_{};
    std::uint32_t intrinsicTraversalInvalidCenterSamples_{};
#endif
    bool globalMetricVisualRegression_{};
    bool globalMetricVisualMediaEnabled_{};
    std::uint32_t globalMetricCoverageFrames_{};
    std::uint32_t globalMetricZeroTerrainCoverageFrames_{};
    std::uint32_t globalMetricZeroAtmosphereCoverageFrames_{};
    std::uint32_t globalMetricZeroCloudCoverageFrames_{};
    double elapsedSeconds_{};
    double cellularAccumulator_{};
};

} // namespace voxel
