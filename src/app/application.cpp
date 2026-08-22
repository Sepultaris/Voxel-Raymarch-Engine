#include "app/application.hpp"

#include "render/vulkan_renderer.hpp"
#include "render/global_metric_field.hpp"

#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace voxel {
namespace {

constexpr std::uint32_t kMinMarchSteps = 16;
constexpr std::uint32_t kMaxMarchSteps = 512;

[[nodiscard]] std::runtime_error sdlError(const char* operation) {
    return std::runtime_error(std::string(operation) + ": " + SDL_GetError());
}

} // namespace

Application::Application(bool forceTopologyRegeneration) {
    SDL_SetMainReady();
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS | SDL_INIT_GAMECONTROLLER) != 0) {
        throw sdlError("SDL initialization failed");
    }

    window_ = SDL_CreateWindow(
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
        "Voxel Planet Engine - ADAPTIVE VOXELIZED FRACTAL SDF Lab",
#elif VOXEL_GLOBAL_METRIC_LAB
        "Voxel Planet Engine - GLOBAL STATIC SPACETIME Lab - NATIVE ELLIS ATLAS",
#elif VOXEL_INTRINSIC_PORTAL_LAB
        "Voxel Planet Engine - INTRINSIC ELLIS MANIFOLD Lab",
#elif VOXEL_PORTAL_LAB
        "Voxel Planet Engine - SPHERICAL WORMHOLE PORTAL Lab",
#elif VOXEL_FRACTAL_PLANET_SDF_LAB
        "Voxel Planet Engine - TRUE FRACTAL PLANET SDF Lab",
#elif VOXEL_ADAPTIVE_SDF_LAB
        "Voxel Planet Engine - Adaptive Fractal SDF Lab",
#elif VOXEL_SCALE_LAB
        "Voxel Planet Engine - f1024 Scale Lab",
#else
        "Voxel Planet Engine",
#endif
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1600,
        900,
        SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (window_ == nullptr) {
        SDL_Quit();
        throw sdlError("Window creation failed");
    }

#if VOXEL_FRACTAL_PLANET_SDF_LAB
    // The f512 topology is generated synchronously during renderer startup.
    // Show and service the lab window before that work begins so Windows does
    // not replace the application with a short-lived unresponsive ghost.
    SDL_ShowWindow(window_);
    SDL_RaiseWindow(window_);
    SDL_PumpEvents();
#endif
#if VOXEL_INTRINSIC_PORTAL_LAB
        // Renderer/topology startup is synchronous. Windows can queue a stale
        // close/keyboard event while focus changes during those few seconds;
        // without flushing it, an Explorer-style launch destroys the SDL
        // window on the first event pump while Vulkan is still winding down.
        SDL_FlushEvent(SDL_QUIT);
        SDL_FlushEvent(SDL_WINDOWEVENT);
        SDL_FlushEvents(SDL_KEYDOWN, SDL_KEYUP);
        SDL_FlushEvents(SDL_MOUSEBUTTONDOWN, SDL_MOUSEBUTTONUP);
        SDL_SetWindowTitle(window_,
#if VOXEL_GLOBAL_METRIC_LAB
                           "GLOBAL STATIC SPACETIME Lab - Loading native Ellis atlas and topology...");
#else
                           "Voxel Planet Engine - INTRINSIC ELLIS MANIFOLD Lab");
#endif
        SDL_ShowWindow(window_);
        SDL_RaiseWindow(window_);
        SDL_PumpEvents();
#endif

    try {
#if VOXEL_GLOBAL_METRIC_LAB
        const GlobalMetricPatchCache globalMetricCache =
            buildGlobalMetricPatchCache();
        if (!globalMetricCache.finite) {
            throw std::runtime_error(
                "Global metric/Christoffel patch precompute was non-finite");
        }
#endif
        renderer_ = std::make_unique<VulkanRenderer>(window_, forceTopologyRegeneration);
        const RendererStats& stats = renderer_->stats();
        surfaceCamera_.configureVoxelGeometry(
            stats.averageSurfaceCellWidth * renderSettings_.planetRadius,
            stats.averageRadialLayerHeight * renderSettings_.planetRadius,
            renderSettings_.planetRadius);
#if VOXEL_INTRINSIC_PORTAL_LAB
#if VOXEL_GLOBAL_METRIC_LAB
        renderSettings_.intrinsicEllis.globalMetricField = true;
        renderSettings_.intrinsicEllis.throatRadius =
            renderSettings_.intrinsicEllis.contentSphereRadius *
            kPortalGrThroatRatio;
        renderSettings_.intrinsicEllis.contentExitProperDepth = std::sqrt(
            renderSettings_.intrinsicEllis.contentSphereRadius *
                renderSettings_.intrinsicEllis.contentSphereRadius -
            renderSettings_.intrinsicEllis.throatRadius *
                renderSettings_.intrinsicEllis.throatRadius);
#endif
        intrinsicEllisCamera_.reset(renderSettings_.intrinsicEllis);
        syncIntrinsicEllisRenderSettings();
        renderSettings_.cameraMode = CameraControllerMode::PortalFreeFly;
        SDL_FlushEvent(SDL_QUIT);
        SDL_FlushEvent(SDL_WINDOWEVENT);
        SDL_FlushEvents(SDL_KEYDOWN, SDL_KEYUP);
        SDL_FlushEvents(SDL_MOUSEBUTTONDOWN, SDL_MOUSEBUTTONUP);
        SDL_SetWindowTitle(window_,
#if VOXEL_GLOBAL_METRIC_LAB
                           "Voxel Planet Engine - GLOBAL STATIC SPACETIME Lab - NATIVE ELLIS ATLAS");
#else
                           "Voxel Planet Engine - INTRINSIC ELLIS MANIFOLD Lab");
#endif
        SDL_ShowWindow(window_);
        SDL_RaiseWindow(window_);
        SDL_PumpEvents();
#endif
#if VOXEL_FRACTAL_PLANET_SDF_LAB
        SDL_SetWindowTitle(window_,
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
                           "Voxel Planet Engine - ADAPTIVE VOXELIZED FRACTAL SDF Lab");
#else
                           "Voxel Planet Engine - TRUE FRACTAL PLANET SDF Lab");
#endif
        SDL_ShowWindow(window_);
        SDL_RaiseWindow(window_);
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
        // Topology construction is synchronous and can take long enough for
        // unrelated keys from the previously focused application to queue on
        // the newly raised SDL window. Do not let a stale Escape close the lab
        // on its first event pump after the renderer becomes ready.
        SDL_FlushEvents(SDL_KEYDOWN, SDL_KEYUP);
        SDL_FlushEvents(SDL_MOUSEBUTTONDOWN, SDL_MOUSEBUTTONUP);
#endif
#endif
    } catch (...) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
        SDL_Quit();
        throw;
    }
}

Application::~Application() {
    setSurfaceMouseCapture(false);
    renderer_.reset();
    if (window_ != nullptr) {
        SDL_DestroyWindow(window_);
    }
    SDL_Quit();
}

void Application::beginIntrinsicEllisTraversalRegression() noexcept {
#if VOXEL_INTRINSIC_PORTAL_LAB
    intrinsicTraversalRegression_ = true;
    intrinsicTraversalCrossed_ = false;
    intrinsicTraversalNativeCrossingCount_ = 0U;
    intrinsicTraversalFinite_ = true;
    intrinsicTraversalNonfiniteRays_ = 0U;
    intrinsicTraversalMaximumFrameDelta_ = 0.0F;
#if VOXEL_GLOBAL_METRIC_LAB
    intrinsicTraversalCoverageFrames_ = 0U;
    intrinsicTraversalZeroTerrainFrames_ = 0U;
    intrinsicTraversalCurrentZeroTerrainRun_ = 0U;
    intrinsicTraversalMaximumZeroTerrainRun_ = 0U;
    intrinsicTraversalInvalidZeroTerrainFrames_ = 0U;
    intrinsicTraversalAffineBudgetExhaustions_ = 0U;
#endif
    intrinsicEllisCamera_.reset(renderSettings_.intrinsicEllis);
    EllisState state = intrinsicEllisCamera_.state();
#if VOXEL_GLOBAL_METRIC_LAB
    // Begin inside the former mouth region so the full-screen near-critical
    // family is exercised before, through, and after l=0. The old replay only
    // checked the final external frame and missed close-observer black rings.
    state.properDepth =
        renderSettings_.intrinsicEllis.contentExitProperDepth * 0.38F;
    intrinsicEllisCamera_.setState(state, intrinsicEllisCamera_.frame());
    intrinsicEllisCamera_.rotate(0.035F, 0.012F);
    renderSettings_.intrinsicEllis.debugMode = 0U;
#else
    state.properDepth = 0.31F;
    intrinsicEllisCamera_.setState(state, intrinsicEllisCamera_.frame());
    intrinsicEllisCamera_.rotate(0.72F, 0.13F);
#endif
    renderSettings_.cameraMode = CameraControllerMode::PortalFreeFly;
    syncIntrinsicEllisRenderSettings();
#endif
}

void Application::beginGlobalMetricVisualRegression(
    bool mediaEnabled) noexcept {
#if VOXEL_GLOBAL_METRIC_LAB
    // Preserve the rejected spherical-mouth implementation only as a
    // historical differential for its shell/ordering tests. Interactive and
    // l=0 crossing acceptance use the native signed-l atlas.
    renderSettings_.intrinsicEllis.globalNativeEllisPath = false;
    // Recreate the reported lateral A/D trajectory as an explicit observer
    // path. The view tetrad is centered on the radial-tangency family, so
    // adjacent screen rays exercise both deterministic half-open owners.
    intrinsicTraversalRegression_ = false;
    globalMetricVisualRegression_ = true;
    globalMetricVisualMediaEnabled_ = mediaEnabled;
    globalMetricCoverageFrames_ = 0U;
    globalMetricZeroTerrainCoverageFrames_ = 0U;
    globalMetricZeroAtmosphereCoverageFrames_ = 0U;
    globalMetricZeroCloudCoverageFrames_ = 0U;
    renderSettings_.atmosphere.enabled = mediaEnabled;
    renderSettings_.clouds.enabled = mediaEnabled;
    intrinsicEllisCamera_.resetGlobal(renderSettings_.intrinsicEllis, 0U);
    renderSettings_.cameraMode = CameraControllerMode::PortalFreeFly;
    syncIntrinsicEllisRenderSettings();
#else
    (void)mediaEnabled;
#endif
}

int Application::run(std::uint32_t maximumRenderedFrames, bool requirePageStreaming,
                     bool rotateCameraForTest, std::uint32_t traversalMode,
                     bool exerciseSurfaceCamera, bool artifactRegression) {
    if (traversalMode <= 8U) {
        renderSettings_.geodesicTraversalMode = traversalMode;
    }
    if (artifactRegression) {
        surfaceCamera_.setCollisionEnabled(false);
    }
    using Clock = std::chrono::steady_clock;
    auto previousTime = Clock::now();
    const auto runStartTime = previousTime;
    std::uint32_t renderedFrames = 0U;
    std::uint32_t firstExactCollisionFrame = 0xffffffffU;

    while (running_) {
#if VOXEL_FRACTAL_PLANET_SDF_LAB || VOXEL_INTRINSIC_PORTAL_LAB
        if ((SDL_GetWindowFlags(window_) & SDL_WINDOW_SHOWN) == 0U) {
            SDL_ShowWindow(window_);
        }
#endif
        SDL_Event event{};
        while (SDL_PollEvent(&event) != 0) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
#if VOXEL_INTRINSIC_PORTAL_LAB
                std::cerr << "Intrinsic Ellis lab received SDL_QUIT.\n";
#endif
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
                std::cerr << "Adaptive voxelized-SDF lab received SDL_QUIT.\n";
#endif
                running_ = false;
            }
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window_)) {
#if VOXEL_INTRINSIC_PORTAL_LAB
                std::cerr << "Intrinsic Ellis lab window was closed.\n";
#endif
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
                std::cerr << "Adaptive voxelized-SDF lab window was closed.\n";
#endif
                running_ = false;
            }
            if (event.type == SDL_WINDOWEVENT &&
                (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED ||
                 event.window.event == SDL_WINDOWEVENT_MINIMIZED ||
                 event.window.event == SDL_WINDOWEVENT_RESTORED)) {
                renderer_->requestResize();
            }
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                event.key.keysym.sym == SDLK_ESCAPE) {
                if (surfaceMouseCaptured_) {
                    setSurfaceMouseCapture(false);
                } else {
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
                    std::cerr << "Adaptive voxelized-SDF lab received Escape.\n";
#endif
                    running_ = false;
                }
            }
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                event.key.keysym.sym == SDLK_F6) {
                const CameraControllerMode nextMode =
                    renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal
                        ? CameraControllerMode::Orbit
                        : CameraControllerMode::SurfaceTraversal;
                setCameraMode(nextMode, nextMode == CameraControllerMode::SurfaceTraversal);
            }
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                event.key.keysym.sym == SDLK_F7) {
#if VOXEL_PORTAL_LAB
                const CameraControllerMode nextMode =
                    renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly
                        ? CameraControllerMode::Orbit
                        : CameraControllerMode::PortalFreeFly;
                setCameraMode(nextMode, nextMode == CameraControllerMode::PortalFreeFly);
#else
                renderSettings_.geodesicTraversalMode =
                    renderSettings_.geodesicTraversalMode == 2U
                        ? 3U
                        : renderSettings_.geodesicTraversalMode == 3U ? 4U : 2U;
#endif
            }
#if VOXEL_PORTAL_LAB
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                event.key.keysym.sym == SDLK_HOME &&
                renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly) {
#if VOXEL_INTRINSIC_PORTAL_LAB
                intrinsicEllisCamera_.reset(renderSettings_.intrinsicEllis);
                syncIntrinsicEllisRenderSettings();
#else
                portalFreeFlyCamera_.resetForPortal(
                    renderSettings_.portal, renderSettings_.planetRadius);
                portalCameraManifold_ = {};
                portalCameraPositionInitialized_ = false;
                syncPortalFreeFlyRenderSettings();
#endif
            }
#if VOXEL_GLOBAL_METRIC_LAB
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                event.key.keysym.sym == SDLK_END &&
                renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly) {
                intrinsicEllisCamera_.resetGlobal(
                    renderSettings_.intrinsicEllis, 1U);
                syncIntrinsicEllisRenderSettings();
            }
#endif
#endif
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                event.key.keysym.sym == SDLK_F8) {
                renderSettings_.geodesicTraversalMode =
                    renderSettings_.geodesicTraversalMode == 6U ? 2U : 6U;
            }
            if (event.type == SDL_KEYDOWN && event.key.repeat == 0 &&
                event.key.keysym.sym == SDLK_F9) {
                renderSettings_.geodesicTraversalMode =
                    renderSettings_.geodesicTraversalMode == 7U ? 2U : 7U;
            }
            if (event.type == SDL_MOUSEBUTTONDOWN && !ImGui::GetIO().WantCaptureMouse) {
                if (event.button.button == SDL_BUTTON_MIDDLE) {
                    if (renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal ||
                        renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly) {
                        setSurfaceMouseCapture(true);
                    } else {
                        orbitingCamera_ = true;
                    }
                } else if (event.button.button == SDL_BUTTON_LEFT) {
                    queueVoxelEdit(event.button.x, event.button.y, 1U);
                } else if (event.button.button == SDL_BUTTON_RIGHT) {
                    queueVoxelEdit(event.button.x, event.button.y, 2U);
                }
            }
            if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_MIDDLE &&
                renderSettings_.cameraMode == CameraControllerMode::Orbit) {
                orbitingCamera_ = false;
            }
            if (event.type == SDL_MOUSEMOTION && surfaceMouseCaptured_ &&
                renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal) {
                surfaceCamera_.rotateHeading(
                    -static_cast<float>(event.motion.xrel) * surfaceCamera_.mouseSensitivity());
                surfaceCamera_.rotateLook(
                    -static_cast<float>(event.motion.yrel) * surfaceCamera_.mouseSensitivity());
#if VOXEL_PORTAL_LAB
            } else if (event.type == SDL_MOUSEMOTION && surfaceMouseCaptured_ &&
                       renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly) {
#if VOXEL_INTRINSIC_PORTAL_LAB
                intrinsicEllisCamera_.rotate(
                    -static_cast<float>(event.motion.xrel) *
                        renderSettings_.intrinsicEllis.mouseSensitivity,
                    -static_cast<float>(event.motion.yrel) *
                        renderSettings_.intrinsicEllis.mouseSensitivity);
#else
                portalFreeFlyCamera_.rotate(
                    -static_cast<float>(event.motion.xrel) *
                        portalFreeFlyCamera_.mouseSensitivity(),
                    -static_cast<float>(event.motion.yrel) *
                        portalFreeFlyCamera_.mouseSensitivity());
#endif
#endif
            } else if (event.type == SDL_MOUSEMOTION && orbitingCamera_) {
                renderSettings_.cameraYaw -= static_cast<float>(event.motion.xrel) * 0.006F;
                renderSettings_.cameraPitch = std::clamp(
                    renderSettings_.cameraPitch + static_cast<float>(event.motion.yrel) * 0.006F,
                    -1.45F,
                    1.45F);
            }
            if (event.type == SDL_MOUSEWHEEL && !ImGui::GetIO().WantCaptureMouse) {
#if VOXEL_PORTAL_LAB
                if (renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly) {
#if VOXEL_INTRINSIC_PORTAL_LAB
                    renderSettings_.intrinsicEllis.freeFlySpeed = std::clamp(
                        renderSettings_.intrinsicEllis.freeFlySpeed *
                            std::pow(1.12F, static_cast<float>(event.wheel.y)),
                        0.01F, 3.0F);
#else
                    portalFreeFlyCamera_.setSpeed(
                        portalFreeFlyCamera_.speed() *
                        std::pow(1.12F, static_cast<float>(event.wheel.y)));
#endif
                } else
#endif
                if (renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal) {
                    if (surfaceCamera_.collisionEnabled()) {
                        continue;
                    }
                    const float minimumClearance =
                        minimumSurfaceClearance(renderSettings_.planetRadius);
                    const float step = std::max(
                        surfaceCamera_.clearance() * 0.16F,
                        renderSettings_.planetRadius * 0.005F);
                    surfaceCamera_.setClearance(std::max(
                        minimumClearance,
                        surfaceCamera_.clearance() -
                            static_cast<float>(event.wheel.y) * step));
                } else {
                    const float minimumDistance = minimumCameraDistance(
                        renderSettings_.planetRadius, renderer_->stats().planetOuterScale);
                    renderSettings_.cameraDistance = std::clamp(
                        renderSettings_.cameraDistance - static_cast<float>(event.wheel.y) *
                            cameraZoomStep(renderSettings_.cameraDistance),
                        minimumDistance,
                        std::max(8.0F, minimumDistance * 4.0F));
                }
            }
        }

        if (!running_) {
            break;
        }

        const auto currentTime = Clock::now();
        const float deltaSeconds = std::chrono::duration<float>(currentTime - previousTime).count();
        previousTime = currentTime;
        if (renderSettings_.animate) {
            elapsedSeconds_ += static_cast<double>(std::min(deltaSeconds, 0.1F)) *
                               static_cast<double>(renderSettings_.timeScale);
        }
        physics_.update(deltaSeconds);
        updateSurfaceController(deltaSeconds);
        updateIntrinsicEllisController(deltaSeconds);
        updatePortalFreeFlyController(deltaSeconds);
        if (artifactRegression && renderedFrames == 0U) {
            setCameraMode(CameraControllerMode::SurfaceTraversal, false);
            surfaceCamera_.rotateLook(-0.31F);
            surfaceCamera_.rotateHeading(-0.42F);
        }
        if (artifactRegression && renderedFrames == 360U) {
            // Distant bright-ridgeline case: low clearance and a nearly level
            // view make minified terrain silhouettes dominate the upper frame.
            surfaceCamera_.resetFromOrbit(1.15F, 0.08F);
            surfaceCamera_.setClearance(0.014F);
            surfaceCamera_.rotateHeading(1.10F);
            surfaceCamera_.rotateLook(-0.07F);
        }
        if (artifactRegression &&
            renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal) {
            // Deterministic, shallow-horizon player motion based on the user recording:
            // a slow heading sweep plus a small diagonal walk over the surface.
            const std::uint32_t sweepFrame = renderedFrames % 240U;
            surfaceCamera_.rotateHeading(sweepFrame < 120U ? 0.0025F : -0.0025F);
            const float cameraRadius =
                renderSettings_.planetRadius * renderer_->stats().planetOuterScale +
                surfaceCamera_.clearance();
            surfaceCamera_.move(0.32F, 0.11F, 1.0F / 60.0F, cameraRadius, false);
        }
        if (exerciseSurfaceCamera && renderedFrames == 1U) {
            setCameraMode(CameraControllerMode::SurfaceTraversal, false);
            surfaceCamera_.rotateLook(-0.35F);
        }
        if (exerciseSurfaceCamera &&
            renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal) {
            const auto terrain = [this](const SurfaceCameraController::Vector& radial,
                                        float radius, float sweep) {
                return renderer_->querySurfaceTerrain(
                    radial, renderSettings_.planetRadius, radius, sweep);
            };
            surfaceCamera_.simulatePlayer(
                1.0F, 0.35F, 1.0F / 60.0F, renderSettings_.planetRadius,
                false, false, terrain);
            if (surfaceCamera_.terrainContactExact() &&
                firstExactCollisionFrame == 0xffffffffU) {
                // The deterministic surface benchmark activates F6 at frame 1.
                // Report request-to-exact latency, not the absolute run frame.
                firstExactCollisionFrame = renderedFrames > 0U
                    ? renderedFrames - 1U : 0U;
            }
        }
        if (rotateCameraForTest) {
            renderSettings_.cameraYaw = static_cast<float>(renderedFrames) * 0.025F;
        }
#if VOXEL_INTRINSIC_PORTAL_LAB
        if (intrinsicTraversalRegression_) {
#if VOXEL_GLOBAL_METRIC_LAB
            if (renderSettings_.intrinsicEllis.globalNativeEllisPath) {
                const EllisState previousState = intrinsicEllisCamera_.state();
                const EllisFrame previousFrame = intrinsicEllisCamera_.frame();
                intrinsicEllisCamera_.rotate(0.00035F, -0.00011F);
                intrinsicEllisCamera_.move(
                    renderedFrames < 40U ? 1.0F : -1.0F,
                    0.055F, 0.018F, 1.0F / 60.0F, false,
                    renderSettings_.intrinsicEllis);
                const EllisState& currentState = intrinsicEllisCamera_.state();
                const EllisFrame& currentFrame = intrinsicEllisCamera_.frame();
                const PortalVector previousForward = ellisTangentToLocal(
                    previousFrame.forward, previousState.angularPosition);
                const PortalVector currentForward = ellisTangentToLocal(
                    currentFrame.forward, currentState.angularPosition);
                const float cosine = std::clamp(portalDot(
                    portalNormalize(previousForward),
                    portalNormalize(currentForward)), -1.0F, 1.0F);
                const float frameAngle = std::acos(cosine);
                if (frameAngle > intrinsicTraversalMaximumFrameDelta_) {
                    std::cout << "Native Ellis max frame transport frame="
                              << renderedFrames << " angle=" << frameAngle
                              << '\n';
                }
                intrinsicTraversalMaximumFrameDelta_ = std::max(
                    intrinsicTraversalMaximumFrameDelta_, frameAngle);
                const bool crossed =
                    (previousState.properDepth >= 0.0F &&
                     currentState.properDepth < 0.0F) ||
                    (previousState.properDepth <= 0.0F &&
                     currentState.properDepth > 0.0F);
                intrinsicTraversalCrossed_ = intrinsicTraversalCrossed_ || crossed;
                if (crossed) {
                    ++intrinsicTraversalNativeCrossingCount_;
                    intrinsicTraversalMaximumExitDirectionDelta_ = std::max(
                        intrinsicTraversalMaximumExitDirectionDelta_,
                        frameAngle);
                    std::cout << "Native Ellis l=0 crossing event frame="
                              << renderedFrames << " l="
                              << previousState.properDepth << "->"
                              << currentState.properDepth << " angle="
                              << frameAngle << '\n';
                }
                intrinsicTraversalFinite_ = intrinsicTraversalFinite_ &&
                    std::isfinite(currentState.properDepth) &&
                    portalFinite(currentState.angularPosition) &&
                    intrinsicEllisCamera_.handedness() > 0.99F;
            } else {
            const GlobalHandleObserverState previous =
                intrinsicEllisCamera_.globalState();
            intrinsicEllisCamera_.rotate(0.00035F, -0.00011F);
            intrinsicEllisCamera_.move(
                1.0F, 0.055F, 0.018F, 1.0F / 60.0F, false,
                renderSettings_.intrinsicEllis);
            const GlobalHandleObserverState current =
                intrinsicEllisCamera_.globalState();
            const float cosine = std::clamp(portalDot(
                portalNormalize(previous.forward),
                portalNormalize(current.forward)), -1.0F, 1.0F);
            intrinsicTraversalMaximumFrameDelta_ = std::max(
                intrinsicTraversalMaximumFrameDelta_, std::acos(cosine));
            intrinsicTraversalCrossed_ = intrinsicTraversalCrossed_ ||
                current.crossings > previous.crossings;
            if (current.crossings > previous.crossings) {
                GlobalHandleRayState beforeRay{};
                beforeRay.origin = previous.position;
                beforeRay.direction = previous.forward;
                beforeRay.footprintU = previous.forward;
                beforeRay.footprintV = previous.up;
                beforeRay.lastMouth = previous.lastMouth;
                GlobalHandleRayState afterRay{};
                afterRay.origin = current.position;
                afterRay.direction = current.forward;
                afterRay.footprintU = current.forward;
                afterRay.footprintV = current.up;
                afterRay.lastMouth = current.lastMouth;
                globalHandleIntegrateRay(beforeRay, 3.5F,
                                         renderSettings_.intrinsicEllis, 512U);
                globalHandleIntegrateRay(afterRay, 3.5F,
                                         renderSettings_.intrinsicEllis, 512U);
                const float exitAngle = std::acos(std::clamp(portalDot(
                    beforeRay.direction, afterRay.direction), -1.0F, 1.0F));
                const float exitOriginDelta = portalLength(
                    beforeRay.origin - afterRay.origin);
                intrinsicTraversalMaximumExitOriginDelta_ = std::max(
                    intrinsicTraversalMaximumExitOriginDelta_,
                    exitOriginDelta);
                intrinsicTraversalMaximumExitDirectionDelta_ = std::max(
                    intrinsicTraversalMaximumExitDirectionDelta_,
                    exitAngle);
                const auto centers = globalHandleCenters(
                    renderSettings_.intrinsicEllis);
                const std::uint32_t source =
                    portalLength(previous.position - centers[0]) <
                    portalLength(previous.position - centers[1]) ? 0U : 1U;
                if (current.lastMouth != 1U - source) {
                    ++intrinsicTraversalOwnerMismatchFrames_;
                }
                std::cout << "Intrinsic crossing event frame=" << renderedFrames
                          << " owner=" << previous.lastMouth << "->"
                          << current.lastMouth << " camera=("
                          << previous.position.x << ',' << previous.position.y
                          << ',' << previous.position.z << ")->("
                          << current.position.x << ',' << current.position.y
                          << ',' << current.position.z << ") exit-origin-delta="
                          << exitOriginDelta
                          << " exit-angle=" << exitAngle << '\n';
            }
            intrinsicTraversalFinite_ = intrinsicTraversalFinite_ &&
                current.finite && portalFinite(current.position) &&
                intrinsicEllisCamera_.handedness() > 0.99F;
            }
#else
            const EllisTangent previousForward =
                intrinsicEllisCamera_.frame().forward;
            const float previousDepth =
                intrinsicEllisCamera_.state().properDepth;
            intrinsicEllisCamera_.rotate(0.00035F, -0.00011F);
            intrinsicEllisCamera_.move(
                1.0F, 0.055F, 0.018F, 1.0F / 60.0F, false,
                renderSettings_.intrinsicEllis);
            const EllisTangent currentForward =
                intrinsicEllisCamera_.frame().forward;
            const PortalVector previousLocal = ellisTangentToLocal(
                previousForward,
                intrinsicEllisCamera_.state().angularPosition);
            const PortalVector currentLocal = ellisTangentToLocal(
                currentForward,
                intrinsicEllisCamera_.state().angularPosition);
            const float cosine = std::clamp(
                portalDot(portalNormalize(previousLocal),
                          portalNormalize(currentLocal)), -1.0F, 1.0F);
            intrinsicTraversalMaximumFrameDelta_ = std::max(
                intrinsicTraversalMaximumFrameDelta_, std::acos(cosine));
            intrinsicTraversalCrossed_ = intrinsicTraversalCrossed_ ||
                (previousDepth >= 0.0F &&
                 intrinsicEllisCamera_.state().properDepth < 0.0F);
            intrinsicTraversalFinite_ = intrinsicTraversalFinite_ &&
                std::isfinite(intrinsicEllisCamera_.state().properDepth) &&
                portalFinite(intrinsicEllisCamera_.state().angularPosition) &&
                intrinsicEllisCamera_.handedness() > 0.99F;
#endif
            syncIntrinsicEllisRenderSettings();
        }
#endif
#if VOXEL_GLOBAL_METRIC_LAB
        if (globalMetricVisualRegression_) {
            const std::uint32_t source = renderedFrames < 90U ? 0U : 1U;
            const std::uint32_t localFrame = renderedFrames < 90U
                ? renderedFrames : renderedFrames - 90U;
            const float replay = std::clamp(
                static_cast<float>(localFrame) / 89.0F, 0.0F, 1.0F);
            const float smoothReplay = replay * replay * (3.0F - 2.0F * replay);
            const auto centers = globalHandleCenters(
                renderSettings_.intrinsicEllis);
            const PortalVector outward = portalNormalize(centers[source]);
            const PortalVector reference = std::abs(outward.y) < 0.92F
                ? PortalVector{0.0F, 1.0F, 0.0F}
                : PortalVector{1.0F, 0.0F, 0.0F};
            const PortalVector lateral = portalNormalize(
                portalCross(reference, outward));
            GlobalHandleObserverState state = intrinsicEllisCamera_.globalState();
            const float standoff =
                renderSettings_.intrinsicEllis.contentSphereRadius;
            const float throat = renderSettings_.intrinsicEllis.throatRadius;
            // Each A/B half now approaches the mouth as it sweeps laterally.
            // This exercises the distance-varying horizon projection which
            // previously exposed the antipodal infinite-line star mask.
            const float radialStandoff = standoff *
                std::lerp(2.80F, 2.05F, smoothReplay);
            state.position = centers[source] + outward * radialStandoff +
                lateral * std::lerp(-throat * 1.35F, throat * 1.35F,
                                    smoothReplay);
            state.forward = portalNormalize(centers[source] - state.position);
            state.up = portalOrthonormalUp(state.forward, reference);
            state.velocity = lateral;
            state.finite = true;
            intrinsicEllisCamera_.setGlobalState(state);
            syncIntrinsicEllisRenderSettings();
        }
#endif

        constexpr double kCellularStep = 1.0 / 60.0;
        constexpr std::uint32_t kMaximumCellularCatchupSteps = 4;
        renderSettings_.simulationSteps = 0;
        if (renderSettings_.simulateMaterials) {
            cellularAccumulator_ += static_cast<double>(std::min(deltaSeconds, 0.1F));
            while (cellularAccumulator_ >= kCellularStep &&
                   renderSettings_.simulationSteps < kMaximumCellularCatchupSteps) {
                cellularAccumulator_ -= kCellularStep;
                ++renderSettings_.simulationSteps;
            }
            if (renderSettings_.simulationSteps == kMaximumCellularCatchupSteps) {
                cellularAccumulator_ = std::min(cellularAccumulator_, kCellularStep);
            }
        } else {
            cellularAccumulator_ = 0.0;
        }

        if (!renderer_->beginUiFrame()) {
            SDL_Delay(16);
            continue;
        }
        buildDevelopmentUi(deltaSeconds);
        updatePortalCameraTraversal(deltaSeconds);
        syncSurfaceRenderSettings();
        renderer_->render(static_cast<float>(elapsedSeconds_), renderSettings_);
#if VOXEL_INTRINSIC_PORTAL_LAB
        if (intrinsicTraversalRegression_) {
            intrinsicTraversalNonfiniteRays_ +=
                renderer_->stats().portalGrNonfiniteRays;
#if VOXEL_GLOBAL_METRIC_LAB
            const RendererStats& crossingStats = renderer_->stats();
            const bool centerFinite = crossingStats.globalFrameCenterSampleValid &&
                std::isfinite(crossingStats.globalFrameCenterLinearR) &&
                std::isfinite(crossingStats.globalFrameCenterLinearG) &&
                std::isfinite(crossingStats.globalFrameCenterLinearB) &&
                std::isfinite(crossingStats.globalFrameCenterMappedOriginX) &&
                std::isfinite(crossingStats.globalFrameCenterMappedOriginY) &&
                std::isfinite(crossingStats.globalFrameCenterMappedOriginZ) &&
                std::isfinite(crossingStats.globalFrameCenterMappedDirectionX) &&
                std::isfinite(crossingStats.globalFrameCenterMappedDirectionY) &&
                std::isfinite(crossingStats.globalFrameCenterMappedDirectionZ) &&
                std::isfinite(crossingStats.globalFrameCenterStarFootprint);
            if (!centerFinite) {
                ++intrinsicTraversalInvalidCenterSamples_;
            } else {
                const PortalVector centerOrigin{
                    crossingStats.globalFrameCenterMappedOriginX,
                    crossingStats.globalFrameCenterMappedOriginY,
                    crossingStats.globalFrameCenterMappedOriginZ};
                const PortalVector centerDirection = portalNormalize({
                    crossingStats.globalFrameCenterMappedDirectionX,
                    crossingStats.globalFrameCenterMappedDirectionY,
                    crossingStats.globalFrameCenterMappedDirectionZ});
                if (intrinsicTraversalPreviousCenterValid_) {
                    const float dr = crossingStats.globalFrameCenterLinearR -
                        intrinsicTraversalPreviousCenterR_;
                    const float dg = crossingStats.globalFrameCenterLinearG -
                        intrinsicTraversalPreviousCenterG_;
                    const float db = crossingStats.globalFrameCenterLinearB -
                        intrinsicTraversalPreviousCenterB_;
                    const bool sameFamily =
                        crossingStats.globalFrameCenterHit ==
                            intrinsicTraversalPreviousCenterHit_ &&
                        crossingStats.globalFrameCenterHitClass ==
                            intrinsicTraversalPreviousCenterHitClass_;
                    if (sameFamily) {
                        intrinsicTraversalMaximumCenterHdrDelta_ = std::max(
                            intrinsicTraversalMaximumCenterHdrDelta_,
                            std::sqrt(dr * dr + dg * dg + db * db));
                    }
                    const PortalVector previousOrigin{
                        intrinsicTraversalPreviousMappedOriginX_,
                        intrinsicTraversalPreviousMappedOriginY_,
                        intrinsicTraversalPreviousMappedOriginZ_};
                    const PortalVector previousDirection = portalNormalize({
                        intrinsicTraversalPreviousMappedDirectionX_,
                        intrinsicTraversalPreviousMappedDirectionY_,
                        intrinsicTraversalPreviousMappedDirectionZ_});
                    intrinsicTraversalMaximumCenterMappedOriginDelta_ = std::max(
                        intrinsicTraversalMaximumCenterMappedOriginDelta_,
                        portalLength(centerOrigin - previousOrigin));
                    intrinsicTraversalMaximumCenterMappedDirectionDelta_ = std::max(
                        intrinsicTraversalMaximumCenterMappedDirectionDelta_,
                        std::acos(std::clamp(portalDot(
                            centerDirection, previousDirection), -1.0F, 1.0F)));
                    intrinsicTraversalMaximumCenterFootprintDelta_ = std::max(
                        intrinsicTraversalMaximumCenterFootprintDelta_,
                        std::abs(crossingStats.globalFrameCenterStarFootprint -
                            intrinsicTraversalPreviousStarFootprint_));
                }
                intrinsicTraversalPreviousCenterValid_ = true;
                intrinsicTraversalPreviousCenterR_ =
                    crossingStats.globalFrameCenterLinearR;
                intrinsicTraversalPreviousCenterG_ =
                    crossingStats.globalFrameCenterLinearG;
                intrinsicTraversalPreviousCenterB_ =
                    crossingStats.globalFrameCenterLinearB;
                intrinsicTraversalPreviousMappedOriginX_ = centerOrigin.x;
                intrinsicTraversalPreviousMappedOriginY_ = centerOrigin.y;
                intrinsicTraversalPreviousMappedOriginZ_ = centerOrigin.z;
                intrinsicTraversalPreviousMappedDirectionX_ = centerDirection.x;
                intrinsicTraversalPreviousMappedDirectionY_ = centerDirection.y;
                intrinsicTraversalPreviousMappedDirectionZ_ = centerDirection.z;
                intrinsicTraversalPreviousStarFootprint_ =
                    crossingStats.globalFrameCenterStarFootprint;
                intrinsicTraversalPreviousCenterHit_ =
                    crossingStats.globalFrameCenterHit;
                intrinsicTraversalPreviousCenterHitClass_ =
                    crossingStats.globalFrameCenterHitClass;
            }
            if (crossingStats.globalFrameRaySamples != 0U) {
                ++intrinsicTraversalCoverageFrames_;
                if (crossingStats.globalFrameTerrainIntervals == 0U) {
                    std::cout << "Intrinsic crossing zero terrain interval at frame "
                              << renderedFrames << " position="
                              << intrinsicEllisCamera_.globalState().position.x << ','
                              << intrinsicEllisCamera_.globalState().position.y << ','
                              << intrinsicEllisCamera_.globalState().position.z
                              << " shell/mouthA/mouthB="
                              << crossingStats.globalFrameContentShellCrossingRays
                              << '/' << crossingStats.globalFrameMouthARays
                              << '/' << crossingStats.globalFrameMouthBRays << '\n';
                }
                intrinsicTraversalZeroTerrainFrames_ +=
                    crossingStats.globalFrameTerrainIntervals == 0U ? 1U : 0U;
                if (crossingStats.globalFrameTerrainIntervals == 0U) {
                    ++intrinsicTraversalCurrentZeroTerrainRun_;
                    intrinsicTraversalMaximumZeroTerrainRun_ = std::max(
                        intrinsicTraversalMaximumZeroTerrainRun_,
                        intrinsicTraversalCurrentZeroTerrainRun_);
                    const bool liveOpticalFamily =
                        crossingStats.globalFrameMouthARays != 0U ||
                        crossingStats.globalFrameMouthBRays != 0U;
                    const bool requiresLegacyShell =
                        !renderSettings_.intrinsicEllis.globalNativeEllisPath;
                    if (!liveOpticalFamily ||
                        (requiresLegacyShell &&
                         crossingStats.globalFrameContentShellCrossingRays == 0U)) {
                        ++intrinsicTraversalInvalidZeroTerrainFrames_;
                    }
                } else {
                    intrinsicTraversalCurrentZeroTerrainRun_ = 0U;
                }
                intrinsicTraversalAffineBudgetExhaustions_ +=
                    crossingStats.globalFrameAffineBudgetExhaustions;
            }
#endif
        }
#endif
#if VOXEL_GLOBAL_METRIC_LAB
        if (globalMetricVisualRegression_ && renderedFrames > 3U) {
            const RendererStats& frameStats = renderer_->stats();
            if (frameStats.globalFrameRaySamples != 0U) {
                ++globalMetricCoverageFrames_;
                if (frameStats.globalFrameTerrainIntervals == 0U) {
                    std::cout << "Global replay zero terrain interval at frame "
                              << renderedFrames << " l="
                              << intrinsicEllisCamera_.state().properDepth << '\n';
                }
                globalMetricZeroTerrainCoverageFrames_ +=
                    frameStats.globalFrameTerrainIntervals == 0U ? 1U : 0U;
                if (globalMetricVisualMediaEnabled_) {
                    globalMetricZeroAtmosphereCoverageFrames_ +=
                        frameStats.globalFrameAtmosphereIntervals == 0U ? 1U : 0U;
                    globalMetricZeroCloudCoverageFrames_ +=
                        frameStats.globalFrameCloudIntervals == 0U ? 1U : 0U;
                }
            }
        }
#endif
        ++renderedFrames;
        renderSettings_.editMode = 0;
        renderSettings_.resetVoxelVolume = false;
        renderSettings_.regenerateTerrain = false;
        if (maximumRenderedFrames > 0U && renderedFrames >= maximumRenderedFrames) {
            running_ = false;
        }
    }

    if (maximumRenderedFrames > 0U) {
        const auto& finalStats = renderer_->stats();
        const double runSeconds = std::chrono::duration<double>(Clock::now() - runStartTime).count();
        std::cout << "Bounded run: " << renderedFrames << " frames in " << runSeconds
                  << " s (" << (runSeconds > 0.0 ? renderedFrames / runSeconds : 0.0)
                  << " FPS)\n";
        std::cout << "GPU planet compute: " << finalStats.gpuRenderMilliseconds << " ms\n";
        std::cout << "GPU streaming / total compute: "
                  << finalStats.gpuStreamingMilliseconds << " / "
                  << finalStats.gpuTotalComputeMilliseconds << " ms\n";
        constexpr const char* kTraversalNames[] = {
            "binary-bvh", "wide-mask", "geodesic-dda"};
        std::cout << "Geodesic traversal: "
                  << kTraversalNames[std::min(finalStats.geodesicTraversalMode, 2U)] << '\n';
        std::cout << "Page streaming: " << finalStats.totalPageRequests << " requests, "
                  << finalStats.totalPagesStreamed << " uploads, "
                  << finalStats.totalPageRequestOverflow << " overflow, "
                  << finalStats.totalStalePageRequests << " stale rejected\n";
        std::cout << "Final streaming frame: " << finalStats.pageRequests << " requests, "
                  << finalStats.pagesStreamed << " uploads\n";
        std::cout << "Streaming request samples: " << finalStats.streamRequestSamples
                  << " / " << finalStats.surfaceTileCount << " surface columns\n";
        std::cout << "Average voxel radial H/W: " << finalStats.radialHeightWidthRatio
                  << " (height " << finalStats.averageRadialLayerHeight
                  << ", width " << finalStats.averageSurfaceCellWidth << ")\n";
#if VOXEL_PORTAL_LAB
        std::cout << "Portal GR precompute: "
                  << (finalStats.portalGrCacheHit ? "warm-cache" : "cold-build")
                  << " " << finalStats.portalGrPrecomputeSeconds * 1000.0
                  << " ms, table " << finalStats.portalGrTableBytes
                  << " bytes, max certified error "
                  << finalStats.portalGrMaximumError
                  << ", previous-frame lookups/refinements "
                  << finalStats.portalGrTableLookups << '/'
                  << finalStats.portalGrLocalRefinements
#if VOXEL_INTRINSIC_PORTAL_LAB
                  << ", nonfinite=" << finalStats.portalGrNonfiniteRays
                  << ", negative-end=" << finalStats.portalGrNegativeEndRays
#endif
                  << '\n';
#endif
#if VOXEL_GLOBAL_METRIC_LAB
        const double aaPercent = finalStats.globalSpatialAaPixelSamples != 0U
            ? 100.0 * static_cast<double>(finalStats.globalSpatialAaPixels) /
                  static_cast<double>(finalStats.globalSpatialAaPixelSamples)
            : 0.0;
        const double aaCandidatePercent =
            finalStats.globalSpatialAaPixelSamples != 0U
            ? 100.0 * static_cast<double>(
                  finalStats.globalSpatialAaCandidatePixels) /
                  static_cast<double>(finalStats.globalSpatialAaPixelSamples)
            : 0.0;
        std::cout << "Global spatial AA: accepted/candidate="
                  << aaPercent << "%/" << aaCandidatePercent
                  << "%, subrays=" << finalStats.globalSpatialAaSubrays
                  << ", family transitions="
                  << finalStats.globalSpatialAaFamilyTransitions
                  << ", rejected candidates="
                  << finalStats.globalSpatialAaRejectedPixels
                  << ", center path avg steps="
                  << (finalStats.globalMetricPathSamples != 0U
                      ? static_cast<double>(finalStats.globalMetricPathSteps) /
                            static_cast<double>(finalStats.globalMetricPathSamples)
                      : 0.0)
                  << ", AA path avg steps="
                  << (finalStats.globalSpatialAaPathSamples != 0U
                      ? static_cast<double>(
                            finalStats.globalSpatialAaPathSteps) /
                            static_cast<double>(
                                finalStats.globalSpatialAaPathSamples)
                      : 0.0) << '\n';
#endif
#if VOXEL_INTRINSIC_PORTAL_LAB
        if (intrinsicTraversalRegression_) {
            std::cout << "Intrinsic traversal: crossed="
                      << (intrinsicTraversalCrossed_ ? "yes" : "no")
                      << ", finite="
                      << (intrinsicTraversalFinite_ ? "yes" : "no")
                      << ", native-crossings="
                      << intrinsicTraversalNativeCrossingCount_
                      << ", maximum adjacent-frame angle="
                      << intrinsicTraversalMaximumFrameDelta_ << " rad"
                      << ", all-frame unresolved="
                      << intrinsicTraversalNonfiniteRays_
#if VOXEL_GLOBAL_METRIC_LAB
                      << ", coverage/zero-terrain="
                      << intrinsicTraversalCoverageFrames_ << '/'
                      << intrinsicTraversalZeroTerrainFrames_
                      << " (max valid-sky run="
                      << intrinsicTraversalMaximumZeroTerrainRun_
                      << ", invalid="
                      << intrinsicTraversalInvalidZeroTerrainFrames_ << ')'
                      << ", affine-budget-exhaustions="
                      << intrinsicTraversalAffineBudgetExhaustions_
                      << ", owner-mismatches="
                      << intrinsicTraversalOwnerMismatchFrames_
                      << ", crossing exit origin/direction="
                      << intrinsicTraversalMaximumExitOriginDelta_ << '/'
                      << intrinsicTraversalMaximumExitDirectionDelta_
                      << ", center HDR/origin/direction/footprint="
                      << intrinsicTraversalMaximumCenterHdrDelta_ << '/'
                      << intrinsicTraversalMaximumCenterMappedOriginDelta_ << '/'
                      << intrinsicTraversalMaximumCenterMappedDirectionDelta_ << '/'
                      << intrinsicTraversalMaximumCenterFootprintDelta_
                      << " (invalid="
                      << intrinsicTraversalInvalidCenterSamples_ << ')'
#endif
                      << ", closed/recoverable misses="
                      << finalStats.artifactClosedShellMisses << '/'
                      << finalStats.artifactRecoverableHorizonMisses
                      << '\n';
            const float acceptedFrameTransport =
#if VOXEL_GLOBAL_METRIC_LAB
                renderSettings_.intrinsicEllis.globalNativeEllisPath
                    // This is the complete curved flight, including the
                    // deliberate reversal for the B->A replay.  The actual
                    // l=0 event has the tighter 0.005-radian gate below.
                    ? 0.15F : 0.02F;
#else
                0.02F;
#endif
            if (!intrinsicTraversalCrossed_ || !intrinsicTraversalFinite_ ||
#if VOXEL_GLOBAL_METRIC_LAB
                (renderSettings_.intrinsicEllis.globalNativeEllisPath &&
                 intrinsicTraversalNativeCrossingCount_ < 2U) ||
#endif
                intrinsicTraversalMaximumFrameDelta_ > acceptedFrameTransport ||
                intrinsicTraversalNonfiniteRays_ != 0U ||
#if VOXEL_GLOBAL_METRIC_LAB
                intrinsicTraversalCoverageFrames_ == 0U ||
                intrinsicTraversalMaximumZeroTerrainRun_ > 3U ||
                intrinsicTraversalInvalidZeroTerrainFrames_ != 0U ||
                intrinsicTraversalAffineBudgetExhaustions_ != 0U ||
                intrinsicTraversalOwnerMismatchFrames_ != 0U ||
                intrinsicTraversalMaximumExitOriginDelta_ > 0.02F ||
                intrinsicTraversalMaximumExitDirectionDelta_ > 0.005F ||
                intrinsicTraversalInvalidCenterSamples_ > 1U ||
#endif
                finalStats.artifactClosedShellMisses != 0U ||
                finalStats.artifactRecoverableHorizonMisses != 0U) {
                std::cerr << "Intrinsic Ellis traversal regression failed.\n";
                return 14;
            }
        }
#endif
#if VOXEL_GLOBAL_METRIC_LAB
        if (globalMetricVisualRegression_) {
            const char* media = globalMetricVisualMediaEnabled_
                ? "atmosphere+clouds" : "no-media";
            std::cout << "Global visual regression (" << media
                      << "): samples="
                      << (finalStats.globalRadialPositiveRays +
                          finalStats.globalRadialNonPositiveRays)
                      << ", mapped-finite="
                      << finalStats.globalMappedFiniteRays
                      << ", radial +/-="
                      << finalStats.globalRadialPositiveRays << '/'
                      << finalStats.globalRadialNonPositiveRays
                      << ", near/exact tangent="
                      << finalStats.globalNearTangentRays << '/'
                      << finalStats.globalExactTangentRays
                      << ", terrain intervals/candidates="
                      << finalStats.globalTerrainIntervalRays << '/'
                      << finalStats.globalDdaCandidateRays
                      << ", final sky=" << finalStats.globalFinalSkyRays
                      << ", atmosphere/cloud intervals="
                      << finalStats.globalAtmosphereIntervalRays << '/'
                      << finalStats.globalCloudIntervalRays
                      << ", mapped-origin repairs="
                      << finalStats.globalMappedOriginRepairs
                      << ", front-facing hits="
                      << finalStats.globalFrontFacingHits
                      << ", back/interior rejects="
                      << finalStats.globalBackFacingHitRejects
                      << ", recovered="
                      << finalStats.globalBackFacingHitRecoveries
                      << ", observer broad/exact/selected/behind="
                      << finalStats.globalObserverBroadCandidates << '/'
                      << finalStats.globalObserverExactHits << '/'
                      << finalStats.globalObserverSelectedHits << '/'
                      << finalStats.globalObserverHitsBehindMouth
                      << ", curved/mouthA/mouthB/exterior/media="
                      << finalStats.globalCurvedSelectedRays << '/'
                      << finalStats.globalMouthARays << '/'
                      << finalStats.globalMouthBRays << '/'
                      << finalStats.globalExteriorDirectRays << '/'
                      << finalStats.globalObserverMediaSegments
                      << ", broad-shell curved/A/B="
                      << finalStats.globalBroadShellCurvedRays << '/'
                      << finalStats.globalBroadShellMouthARays << '/'
                      << finalStats.globalBroadShellMouthBRays
                      << ", content-shell query rays="
                      << finalStats.globalContentShellCrossingRays
                      << ", affine-budget-exhaustions="
                      << finalStats.globalAffineBudgetExhaustions
                      << ", coverage frames/zero terrain/zero atmosphere/zero cloud="
                      << globalMetricCoverageFrames_ << '/'
                      << globalMetricZeroTerrainCoverageFrames_ << '/'
                      << globalMetricZeroAtmosphereCoverageFrames_ << '/'
                      << globalMetricZeroCloudCoverageFrames_ << '\n';
            const std::uint64_t raySamples =
                finalStats.globalRadialPositiveRays +
                finalStats.globalRadialNonPositiveRays;
            const bool mediaCoverageValid = !globalMetricVisualMediaEnabled_ ||
                (globalMetricZeroAtmosphereCoverageFrames_ == 0U &&
                 globalMetricZeroCloudCoverageFrames_ == 0U);
            const bool orderedMediaValid = globalMetricVisualMediaEnabled_
                ? finalStats.globalObserverMediaSegments > 0U
                : finalStats.globalObserverMediaSegments == 0U;
            const bool spatialAaValid =
                !renderSettings_.intrinsicEllis.globalSpatialAaEnabled ||
                (finalStats.globalSpatialAaPixelSamples > 0U &&
                 finalStats.globalSpatialAaCandidatePixels > 0U &&
                 finalStats.globalSpatialAaPixels > 0U &&
                 finalStats.globalSpatialAaSubrays ==
                     finalStats.globalSpatialAaPixels * 4U &&
                 finalStats.globalSpatialAaFamilyTransitions > 0U &&
                 finalStats.globalSpatialAaPathSamples > 0U);
            if (raySamples == 0U ||
                finalStats.globalMappedFiniteRays != raySamples ||
                finalStats.globalFrontFacingHits == 0U ||
                finalStats.globalCurvedSelectedRays == 0U ||
                finalStats.globalMouthARays == 0U ||
                finalStats.globalMouthBRays == 0U ||
                finalStats.globalObserverBroadCandidates == 0U ||
                finalStats.globalBroadShellCurvedRays == 0U ||
                finalStats.globalContentShellCrossingRays == 0U ||
                finalStats.globalBackFacingHitRejects !=
                    finalStats.globalBackFacingHitRecoveries ||
                finalStats.globalAffineBudgetExhaustions != 0U ||
                globalMetricCoverageFrames_ == 0U ||
                globalMetricZeroTerrainCoverageFrames_ != 0U ||
                !mediaCoverageValid ||
                !orderedMediaValid ||
                !spatialAaValid ||
                finalStats.portalGrNonfiniteRays != 0U) {
                std::cerr << "Global metric visual ordering regression failed.\n";
                return 15;
            }
        }
#endif
#if VOXEL_SCALE_LAB
        std::cout << "Scale-lab telemetry: DDA p50/p95/p99/max="
                  << finalStats.ddaEventsP50 << '/' << finalStats.ddaEventsP95 << '/'
                  << finalStats.ddaEventsP99 << '/' << finalStats.ddaEventsMaximum
                  << " atlas=" << finalStats.atlasRefinementsP50 << '/'
                  << finalStats.atlasRefinementsP95 << '/'
                  << finalStats.atlasRefinementsP99 << '/'
                  << finalStats.atlasRefinementsMaximum
                  << " samples=" << finalStats.telemetryRaySamples << '\n'
                  << "Scale-lab conservative bound: skipped="
                  << finalStats.conservativeBoundSkips << " nonfinite="
                  << finalStats.conservativeBoundNonfinite << '\n'
                  << "Scale-lab two-stage sampled traffic (conservative upper bounds): nodes="
                  << finalStats.macroNodeTests << " candidates="
                  << finalStats.macroCandidates << " fallback="
                  << finalStats.macroFallbacks << " compact-tile-loads="
                  << finalStats.compactTileLoads << '\n'
                  << "Scale-lab hybrid/exact differential: samples="
                  << finalStats.differentialSamples << " hit-mismatch="
                  << finalStats.differentialHitMismatches
                  << " distance-mismatch="
                  << finalStats.differentialDistanceMismatches
                  << " max-distance-ULP="
                  << finalStats.differentialMaximumDistanceUlps << '\n'
                  << "Scale-lab invalid DDA statuses: invalid-neighbor="
                  << finalStats.artifactInvalidNeighbors << " closed-shell="
                  << finalStats.artifactClosedShellMisses << " recoverable-horizon="
                  << finalStats.artifactRecoverableHorizonMisses << '\n'
                  << "Scale-lab startup/CPU peak: " << finalStats.startupSeconds << " s / "
                  << static_cast<double>(finalStats.cpuPeakBytes) /
                         static_cast<double>(1024ULL * 1024ULL * 1024ULL)
                  << " GiB\n"
                  << "Scale-lab topology cache: "
                  << (finalStats.topologyCacheHit ? "warm hit" : "cold generation")
                  << ", " << finalStats.topologyCacheSeconds << " s, "
                  << static_cast<double>(finalStats.topologyCacheBytes) /
                         static_cast<double>(1024ULL * 1024ULL)
                  << " MiB\n";
        if (exerciseSurfaceCamera) {
            std::cout << "Scale-lab collision profile exact frame: "
                      << firstExactCollisionFrame << '\n';
        }
#endif
#if VOXEL_ADAPTIVE_SDF_LAB
        std::cout << "Adaptive-SDF telemetry: active="
                  << finalStats.adaptiveSdfActiveSamples
                  << " nonfinite=" << finalStats.adaptiveSdfNonfiniteRejects
                  << " boundary-reject=" << finalStats.adaptiveSdfBoundaryRejects
                  << " root-iterations=" << finalStats.adaptiveSdfRootIterations
                  << " level-p50/p95/max=" << finalStats.adaptiveSdfLevelP50 << '/'
                  << finalStats.adaptiveSdfLevelP95 << '/'
                  << finalStats.adaptiveSdfLevelMaximum << '\n';
        if (exerciseSurfaceCamera) {
            std::cout << "Adaptive-SDF collision profile exact frame: "
                      << firstExactCollisionFrame << '\n';
        }
#endif
#if VOXEL_FRACTAL_PLANET_SDF_LAB
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
        std::cout << "Adaptive voxelized-fractal SDF telemetry: hits="
#else
        std::cout << "True fractal-planet SDF telemetry: hits="
#endif
                  << finalStats.fractalPlanetHits
                  << " conservative-interval-skips="
                  << finalStats.fractalPlanetIntervalSkips
                  << " nonfinite=" << finalStats.fractalPlanetNonfiniteRejects
                  << " root-iterations=" << finalStats.fractalPlanetRootIterations
                  << " recovered-hits=" << finalStats.fractalPlanetRecoveredHits
                  << " closed-shell-misses="
                  << finalStats.fractalPlanetClosedShellMisses
                  << " leaf-level-p50/p95/max="
                  << finalStats.fractalPlanetLevelP50 << '/'
                  << finalStats.fractalPlanetLevelP95 << '/'
                  << finalStats.fractalPlanetLevelMaximum << '\n';
        if (exerciseSurfaceCamera) {
            std::cout << "True fractal-planet collision exact frame: "
                      << firstExactCollisionFrame << '\n';
        }
#endif
        if (artifactRegression) {
#if VOXEL_FRACTAL_PLANET_SDF_LAB
            if (finalStats.fractalPlanetHits == 0U ||
                finalStats.fractalPlanetIntervalSkips == 0U ||
                finalStats.fractalPlanetNonfiniteRejects != 0U ||
                finalStats.fractalPlanetRootIterations == 0U ||
                finalStats.fractalPlanetClosedShellMisses != 0U) {
                std::cerr << "True fractal-planet regression failed: the procedural "
                             "surface was inactive, nonfinite, or not conservatively refined.\n";
                return 6;
            }
#else
            std::cout << "Artifact diagnostics: "
                      << finalStats.artifactDiagnosticRecords << " records, "
                      << finalStats.artifactDiagnosticOverflow << " overflow\n"
                      << "  cap=" << finalStats.artifactCapHits
                      << " non-exposed-side=" << finalStats.artifactNonExposedSides
                      << " filtered-side=" << finalStats.artifactFilteredSides
                      << " partial-side=" << finalStats.artifactPartialSides
                      << " resolved-side=" << finalStats.artifactResolvedSides
                      << " invalid-neighbor=" << finalStats.artifactInvalidNeighbors
                      << " closed-shell-miss=" << finalStats.artifactClosedShellMisses
                      << " recoverable-horizon-miss="
                      << finalStats.artifactRecoverableHorizonMisses << '\n'
                      << "  all ordinary miss terminations: no-owner="
                      << finalStats.artifactAllNoOwnerMisses << " far="
                      << finalStats.artifactAllFarMisses << '\n'
                      << "  near-horizon procedural star pixels="
                      << finalStats.artifactNearHorizonStarPixels << '\n'
                      << "  maximum isolated partial/resolved side pixels="
                      << finalStats.artifactMaximumIsolatedSidePixels << '\n'
                      << "  maximum pixels belonging to dotted side chains="
                      << finalStats.artifactMaximumDottedChainPixels << '\n'
#if VOXEL_ADAPTIVE_SDF_LAB
                      << "  adaptive planet SDF: active-samples="
                      << finalStats.adaptiveSdfActiveSamples << " nonfinite="
                      << finalStats.adaptiveSdfNonfiniteRejects << " boundary-reject="
                      << finalStats.adaptiveSdfBoundaryRejects << " root-iterations="
                      << finalStats.adaptiveSdfRootIterations << '\n';
#else
                      << "  terrain micro-SDF: active-samples="
                      << finalStats.microSdfActiveSamples << " nonfinite="
                      << finalStats.microSdfNonfiniteRejects << " boundary-reject="
                      << finalStats.microSdfBoundaryRejects << " overstep="
                      << finalStats.microSdfOversteps << '\n';
#endif
            if (finalStats.artifactFirstChainTile != 0xffffffffU) {
                std::cout << "  first dotted-chain side: pixel=("
                          << finalStats.artifactFirstChainPixelX << ','
                          << finalStats.artifactFirstChainPixelY << ") class="
                          << finalStats.artifactFirstChainClass << " tile="
                          << finalStats.artifactFirstChainTile << " distance="
                          << finalStats.artifactFirstChainDistance << " projected=("
                          << finalStats.artifactFirstChainProjectedWidth << ','
                          << finalStats.artifactFirstChainProjectedHeight
                          << ") local-interior="
                          << finalStats.artifactFirstChainLocalInterior << '\n';
            }
            if (finalStats.artifactFirstTile != 0xffffffffU) {
                std::cout << "  first suspect: pixel=(" << finalStats.artifactFirstPixelX
                          << ',' << finalStats.artifactFirstPixelY << ") class="
                          << finalStats.artifactFirstClass << " tile="
                          << finalStats.artifactFirstTile << " layer="
                          << finalStats.artifactFirstLayer << " neighbor="
                          << finalStats.artifactFirstNeighbor << " heights=("
                          << finalStats.artifactFirstColumnHeight << ','
                          << finalStats.artifactFirstNeighborHeight << ") material="
                          << finalStats.artifactFirstMaterial << " termination="
                          << finalStats.artifactFirstTermination << " distance="
                          << finalStats.artifactFirstDistance << " detail="
                          << finalStats.artifactFirstGeometryDetail << " incidence="
                          << finalStats.artifactFirstSideIncidence << " projected=("
                          << finalStats.artifactFirstProjectedWidth << ','
                          << finalStats.artifactFirstProjectedHeight << ") side-mask="
                          << finalStats.artifactFirstSideMask << " local-interior="
                          << finalStats.artifactFirstLocalInteriorPixels << " ray=("
                          << finalStats.artifactFirstRayX << ','
                          << finalStats.artifactFirstRayY << ','
                          << finalStats.artifactFirstRayZ << ")\n";
            }
            std::cout << "  isolated minor-footprint histogram [<1,<2,<3,<4,<6,>=6]: ";
            for (const std::uint64_t count :
                 finalStats.artifactIsolatedMinorPixelHistogram) {
                std::cout << count << ' ';
            }
            std::cout << '\n';
            std::cout << "  recoverable closest-approach bins [<.94,<.97,<1.0,>=1.0]: ";
            for (const std::uint64_t count :
                 finalStats.artifactRecoverableClosestApproachBins) {
                std::cout << count << ' ';
            }
            std::cout << '\n';
            std::cout << "  all no-owner closest-approach bins [<.94,<.97,<1.0,>=1.0]: ";
            for (const std::uint64_t count :
                 finalStats.artifactNoOwnerClosestApproachBins) {
                std::cout << count << ' ';
            }
            std::cout << '\n';
            if (finalStats.artifactFirstHorizonTile != 0xffffffffU) {
                std::cout << "  first recoverable horizon miss: pixel=("
                          << finalStats.artifactFirstHorizonPixelX << ','
                          << finalStats.artifactFirstHorizonPixelY << ") termination="
                          << finalStats.artifactFirstHorizonTermination << " tile="
                          << finalStats.artifactFirstHorizonTile << " hit-distance="
                          << finalStats.artifactFirstHorizonDistance
                          << " closest-approach="
                          << finalStats.artifactFirstHorizonClosestApproach
                          << " terminal-cap=(denom "
                          << finalStats.artifactFirstHorizonCapDenominator
                          << ", distance "
                          << finalStats.artifactFirstHorizonCapDistance
                          << ") travel/far=("
                          << finalStats.artifactFirstHorizonTravel << ','
                          << finalStats.artifactFirstHorizonFar << ")\n";
            }
            std::cout << "  termination counts [unset,sphere-miss,hit,no-owner,far,steps]: ";
            for (const std::uint64_t count : finalStats.artifactTerminationCounts) {
                std::cout << count << ' ';
            }
            std::cout << '\n';
            if (finalStats.artifactFirstMissTermination != 0U) {
                std::cout << "  first closed-shell miss: pixel=("
                          << finalStats.artifactFirstMissPixelX << ','
                          << finalStats.artifactFirstMissPixelY << ") termination="
                          << finalStats.artifactFirstMissTermination << " tile="
                          << finalStats.artifactFirstMissTile << " column-height="
                          << finalStats.artifactFirstMissColumnHeight << " travel/far=("
                          << finalStats.artifactFirstMissTravel << ','
                          << finalStats.artifactFirstMissFar << ") cap=(denom "
                          << finalStats.artifactFirstMissCapDenominator << ", distance "
                          << finalStats.artifactFirstMissCapDistance << ", cone-margin "
                          << finalStats.artifactFirstMissConeMargin << ") ray=("
                          << finalStats.artifactFirstMissRayX << ','
                          << finalStats.artifactFirstMissRayY << ','
                          << finalStats.artifactFirstMissRayZ << ")\n";
            }
            if (finalStats.rayProbeValid) {
                std::cout << "  center ray GPU record: "
                          << (finalStats.rayProbeHit ? "hit" : "miss")
                          << " class=" << finalStats.rayProbeClass
                          << " termination=" << finalStats.rayProbeTermination
                          << " tile=" << finalStats.rayProbeTile
                          << " layer=" << finalStats.rayProbeLayer
                          << " material=" << finalStats.rayProbeMaterial
                          << " travel=" << finalStats.rayProbeTravel
                          << " interval=[" << finalStats.rayProbeIntervalNear << ','
                          << finalStats.rayProbeIntervalFar << "] candidate="
                          << finalStats.rayProbeCandidateDistance
                          << " closest=" << finalStats.rayProbeClosestApproach
                          << " normal=(" << finalStats.rayProbeNormalX << ','
                          << finalStats.rayProbeNormalY << ','
                          << finalStats.rayProbeNormalZ << ")\n";
            }
            constexpr std::uint32_t kMaximumAllowedDottedChainPixels = 4U;
#if VOXEL_ADAPTIVE_SDF_LAB
            const bool detailDiagnosticsFailed =
                finalStats.adaptiveSdfActiveSamples == 0U ||
                finalStats.adaptiveSdfNonfiniteRejects != 0U ||
                finalStats.adaptiveSdfBoundaryRejects != 0U ||
                finalStats.adaptiveSdfRootIterations == 0U;
#else
            const bool detailDiagnosticsFailed =
                finalStats.microSdfActiveSamples == 0U ||
                finalStats.microSdfNonfiniteRejects != 0U ||
                finalStats.microSdfBoundaryRejects != 0U ||
                finalStats.microSdfOversteps != 0U;
#endif
            if (finalStats.artifactDiagnosticRecords == 0U ||
                finalStats.artifactDiagnosticOverflow != 0U ||
                finalStats.artifactInvalidNeighbors != 0U ||
                finalStats.artifactClosedShellMisses != 0U ||
                finalStats.artifactRecoverableHorizonMisses != 0U ||
                finalStats.artifactNearHorizonStarPixels != 0U ||
                detailDiagnosticsFailed ||
                finalStats.artifactMaximumDottedChainPixels >
                    kMaximumAllowedDottedChainPixels) {
                std::cerr << "Artifact regression failed: close/distant coverage, "
                             "diagnostic integrity, or unstable radial sides remain.\n";
                return 6;
            }
#endif
        }
        if (requirePageStreaming &&
            finalStats.residentTileBricks < finalStats.surfaceTileCount &&
            finalStats.totalPagesStreamed == 0U) {
            std::cerr << "Streaming gate failed: no nonresident GPU page was uploaded.\n";
            return 2;
        }
        if (requirePageStreaming &&
            (finalStats.totalPageRequestOverflow != 0U ||
             finalStats.totalStalePageRequests != 0U)) {
            std::cerr << "Streaming gate failed: feedback overflowed or became stale.\n";
            return 3;
        }
        if (requirePageStreaming && finalStats.surfaceTileCount > 16'384U &&
            (finalStats.streamRequestSamples == 0U ||
             finalStats.streamRequestSamples >= finalStats.surfaceTileCount)) {
            std::cerr << "Streaming gate failed: request work is not bounded below topology size.\n";
            return 4;
        }
#if VOXEL_SCALE_LAB
        const float gpuBudget = artifactRegression
            ? std::numeric_limits<float>::infinity()
            : exerciseSurfaceCamera ? 12.0F
            : rotateCameraForTest ? 5.0F : 4.0F;
        const bool collisionLate = exerciseSurfaceCamera &&
            (firstExactCollisionFrame == 0xffffffffU || firstExactCollisionFrame > 2U);
        if (finalStats.gpuTotalComputeMilliseconds > gpuBudget ||
            finalStats.ddaEventsP99 >= 512U || finalStats.ddaEventsMaximum >= 2048U ||
            finalStats.artifactInvalidNeighbors != 0U ||
            finalStats.artifactClosedShellMisses != 0U ||
            finalStats.artifactRecoverableHorizonMisses != 0U ||
            finalStats.conservativeBoundNonfinite != 0U ||
            finalStats.differentialHitMismatches != 0U ||
            finalStats.differentialDistanceMismatches != 0U ||
            finalStats.totalPageRequestOverflow != 0U ||
            finalStats.totalStalePageRequests != 0U || collisionLate) {
            std::cerr << "Scale-lab acceptance failed: performance, traversal, streaming, "
                         "or collision freshness exceeded its authorized budget.\n";
            return 8;
        }
#endif
#if VOXEL_ADAPTIVE_SDF_LAB
        const float adaptiveGpuBudget = artifactRegression
            ? std::numeric_limits<float>::infinity()
            : exerciseSurfaceCamera ? 12.0F
            : rotateCameraForTest ? 5.0F : 4.0F;
        const bool adaptiveCollisionLate = exerciseSurfaceCamera &&
            (firstExactCollisionFrame == 0xffffffffU || firstExactCollisionFrame > 2U);
        if (finalStats.gpuTotalComputeMilliseconds > adaptiveGpuBudget ||
            finalStats.totalPageRequestOverflow != 0U ||
            finalStats.totalStalePageRequests != 0U || adaptiveCollisionLate) {
            std::cerr << "Adaptive-SDF lab acceptance failed: performance, streaming, "
                         "or finest collision freshness exceeded its budget.\n";
            return 9;
        }
#endif
#if VOXEL_FRACTAL_PLANET_SDF_LAB
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
        const float fractalGpuBudget = artifactRegression
            ? std::numeric_limits<float>::infinity()
            : exerciseSurfaceCamera ? 15.0F : 5.0F;
#else
        const float fractalGpuBudget = artifactRegression
            ? std::numeric_limits<float>::infinity()
            : exerciseSurfaceCamera ? 12.0F
            : rotateCameraForTest ? 5.0F : 4.0F;
#endif
        const bool fractalCollisionLate = exerciseSurfaceCamera &&
            (firstExactCollisionFrame == 0xffffffffU || firstExactCollisionFrame > 2U);
        if (finalStats.gpuTotalComputeMilliseconds > fractalGpuBudget ||
            finalStats.totalPageRequestOverflow != 0U ||
            finalStats.totalStalePageRequests != 0U || fractalCollisionLate) {
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
            std::cerr << "Adaptive voxelized-fractal SDF lab acceptance failed: performance, "
#else
            std::cerr << "True fractal-planet SDF lab acceptance failed: performance, "
#endif
                         "streaming, or finest collision freshness exceeded its budget.\n";
            return 10;
        }
#endif
    }
    return 0;
}

void Application::setCameraMode(CameraControllerMode mode, bool captureSurfaceMouse) {
    if (mode == CameraControllerMode::SurfaceTraversal &&
        renderSettings_.cameraMode != CameraControllerMode::SurfaceTraversal) {
        surfaceCamera_.resetFromOrbit(
            renderSettings_.cameraYaw + static_cast<float>(elapsedSeconds_),
            renderSettings_.cameraPitch);
        surfaceCamera_.setClearance(std::max(
            surfaceCamera_.clearance(), minimumSurfaceClearance(renderSettings_.planetRadius)));
        if (surfaceCamera_.collisionEnabled()) {
            const auto terrain = [this](const SurfaceCameraController::Vector& radial,
                                        float radius, float sweep) {
                return renderer_->querySurfaceTerrain(
                    radial, renderSettings_.planetRadius, radius, sweep);
            };
            surfaceCamera_.resetToSurface(renderSettings_.planetRadius, terrain);
        }
    }
#if VOXEL_PORTAL_LAB
    if (mode == CameraControllerMode::PortalFreeFly &&
        renderSettings_.cameraMode != CameraControllerMode::PortalFreeFly) {
#if VOXEL_INTRINSIC_PORTAL_LAB
        intrinsicEllisCamera_.reset(renderSettings_.intrinsicEllis);
        syncIntrinsicEllisRenderSettings();
#else
        portalFreeFlyCamera_.resetForPortal(
            renderSettings_.portal, renderSettings_.planetRadius);
        portalCameraManifold_ = {};
        portalCameraPositionInitialized_ = false;
        syncPortalFreeFlyRenderSettings();
#endif
    }
#endif
    renderSettings_.cameraMode = mode;
    orbitingCamera_ = false;
    setSurfaceMouseCapture(
        (mode == CameraControllerMode::SurfaceTraversal ||
         mode == CameraControllerMode::PortalFreeFly) && captureSurfaceMouse);
    syncSurfaceRenderSettings();
}

void Application::setSurfaceMouseCapture(bool enabled) noexcept {
    const bool shouldEnable = enabled && window_ != nullptr &&
        (renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal ||
         renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly);
    if (SDL_SetRelativeMouseMode(shouldEnable ? SDL_TRUE : SDL_FALSE) == 0) {
        surfaceMouseCaptured_ = shouldEnable;
    } else if (!shouldEnable) {
        surfaceMouseCaptured_ = false;
    }
}

void Application::updatePortalFreeFlyController(float deltaSeconds) {
#if VOXEL_PORTAL_LAB && !VOXEL_INTRINSIC_PORTAL_LAB
    if (renderSettings_.cameraMode != CameraControllerMode::PortalFreeFly) {
        return;
    }
    const std::uint8_t* keyboard = SDL_GetKeyboardState(nullptr);
    const float forward = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_W] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_S] != 0 ? 1.0F : 0.0F) : 0.0F;
    const float right = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_D] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_A] != 0 ? 1.0F : 0.0F) : 0.0F;
    const float vertical = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_SPACE] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_LCTRL] != 0 ||
           keyboard[SDL_SCANCODE_RCTRL] != 0 ? 1.0F : 0.0F) : 0.0F;
    const float roll = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_E] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_Q] != 0 ? 1.0F : 0.0F) : 0.0F;
    const bool sprint = surfaceMouseCaptured_ &&
        (keyboard[SDL_SCANCODE_LSHIFT] != 0 ||
         keyboard[SDL_SCANCODE_RSHIFT] != 0);
    if (roll != 0.0F) {
        portalFreeFlyCamera_.rotate(
            0.0F, 0.0F, roll * std::min(deltaSeconds, 0.1F) * 1.35F);
    }
    portalFreeFlyCamera_.move(
        forward, right, vertical, deltaSeconds, sprint);
    syncPortalFreeFlyRenderSettings();
#else
    static_cast<void>(deltaSeconds);
#endif
}

void Application::updateIntrinsicEllisController(float deltaSeconds) {
#if VOXEL_INTRINSIC_PORTAL_LAB
    if (renderSettings_.cameraMode != CameraControllerMode::PortalFreeFly) {
        return;
    }
    const std::uint8_t* keyboard = SDL_GetKeyboardState(nullptr);
    const float forward = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_W] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_S] != 0 ? 1.0F : 0.0F) : 0.0F;
    const float right = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_D] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_A] != 0 ? 1.0F : 0.0F) : 0.0F;
    const float vertical = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_SPACE] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_LCTRL] != 0 ||
           keyboard[SDL_SCANCODE_RCTRL] != 0 ? 1.0F : 0.0F) : 0.0F;
    const float roll = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_E] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_Q] != 0 ? 1.0F : 0.0F) : 0.0F;
    const bool sprint = surfaceMouseCaptured_ &&
        (keyboard[SDL_SCANCODE_LSHIFT] != 0 ||
         keyboard[SDL_SCANCODE_RSHIFT] != 0);
    if (roll != 0.0F) {
        intrinsicEllisCamera_.rotate(
            0.0F, 0.0F, roll * std::min(deltaSeconds, 0.1F) * 1.35F);
    }
    intrinsicEllisCamera_.move(
        forward, right, vertical, deltaSeconds, sprint,
        renderSettings_.intrinsicEllis);
    syncIntrinsicEllisRenderSettings();
#else
    static_cast<void>(deltaSeconds);
#endif
}

void Application::updateSurfaceController(float deltaSeconds) {
    if (renderSettings_.cameraMode != CameraControllerMode::SurfaceTraversal) {
        return;
    }
    const float minimumClearance = minimumSurfaceClearance(renderSettings_.planetRadius);
    surfaceCamera_.setClearance(std::max(surfaceCamera_.clearance(), minimumClearance));
    const std::uint8_t* keyboard = SDL_GetKeyboardState(nullptr);
    const float forward = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_W] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_S] != 0 ? 1.0F : 0.0F) : 0.0F;
    const float right = surfaceMouseCaptured_
        ? (keyboard[SDL_SCANCODE_D] != 0 ? 1.0F : 0.0F) -
          (keyboard[SDL_SCANCODE_A] != 0 ? 1.0F : 0.0F) : 0.0F;
    const bool sprint = surfaceMouseCaptured_ &&
        (keyboard[SDL_SCANCODE_LSHIFT] != 0 ||
         keyboard[SDL_SCANCODE_RSHIFT] != 0);
    const bool jumpDown = surfaceMouseCaptured_ &&
        keyboard[SDL_SCANCODE_SPACE] != 0;
    const bool jumpPressed = jumpDown && !surfaceJumpHeld_;
    surfaceJumpHeld_ = jumpDown;
    const auto terrain = [this](const SurfaceCameraController::Vector& radial,
                                float radius, float sweep) {
        return renderer_->querySurfaceTerrain(
            radial, renderSettings_.planetRadius, radius, sweep);
    };
    surfaceCamera_.simulatePlayer(
        forward, right, std::min(deltaSeconds, 0.1F), renderSettings_.planetRadius,
        sprint, jumpPressed, terrain);

    const float vertical = (keyboard[SDL_SCANCODE_E] != 0 ? 1.0F : 0.0F) -
                           (keyboard[SDL_SCANCODE_Q] != 0 ? 1.0F : 0.0F);
    if (!surfaceCamera_.collisionEnabled() && surfaceMouseCaptured_ && vertical != 0.0F) {
        const float verticalSpeed = std::max(renderSettings_.planetRadius * 0.25F, 0.05F);
        surfaceCamera_.setClearance(std::max(
            minimumClearance,
            surfaceCamera_.clearance() + vertical * verticalSpeed *
                std::min(deltaSeconds, 0.1F)));
    }
}

void Application::updatePortalCameraTraversal(float deltaSeconds) {
#if VOXEL_PORTAL_LAB && !VOXEL_INTRINSIC_PORTAL_LAB
    auto& portal = renderSettings_.portal;
    const float planetRadius = std::max(renderSettings_.planetRadius, 1.0e-6F);
    const PortalVector endpointA = portalEndpoint(
        portal.endpointAAzimuth, portal.endpointAElevation,
        portal.endpointADistance);
    const PortalVector endpointB = portalEndpoint(
        portal.endpointBAzimuth, portal.endpointBElevation,
        portal.endpointBDistance);

    PortalVector cameraPosition{};
    PortalVector viewForward{};
    PortalVector viewUp{};
    if (renderSettings_.cameraMode == CameraControllerMode::Orbit) {
        const float yaw = renderSettings_.cameraYaw +
                          static_cast<float>(elapsedSeconds_);
        const float cosinePitch = std::cos(renderSettings_.cameraPitch);
        const PortalVector radial{
            std::sin(yaw) * cosinePitch,
            std::sin(renderSettings_.cameraPitch),
            std::cos(yaw) * cosinePitch};
        cameraPosition = radial * renderSettings_.cameraDistance;
        viewForward = radial * -1.0F;
        viewUp = portalNormalize(radial);
    } else if (renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal) {
        const auto& radial = surfaceCamera_.radial();
        const auto& tangent = surfaceCamera_.tangentForward();
        const float cosineLook = std::cos(surfaceCamera_.lookPitch());
        const float sineLook = std::sin(surfaceCamera_.lookPitch());
        cameraPosition = PortalVector{radial[0], radial[1], radial[2]} *
                         surfaceCamera_.cameraRadius();
        viewForward = portalNormalize(
            PortalVector{tangent[0], tangent[1], tangent[2]} * cosineLook +
                PortalVector{radial[0], radial[1], radial[2]} * sineLook);
        viewUp = portalNormalize(PortalVector{radial[0], radial[1], radial[2]});
    } else {
        cameraPosition = portalFreeFlyCamera_.position();
        viewForward = portalFreeFlyCamera_.forward();
        viewUp = portalFreeFlyCamera_.up();
    }
    const PortalVector normalizedPosition = cameraPosition * (1.0F / planetRadius);
    const bool configurationChanged = portalCameraPositionInitialized_ &&
        (portalLength(endpointA - previousPortalEndpointA_) > 1.0e-5F ||
         portalLength(endpointB - previousPortalEndpointB_) > 1.0e-5F);
    previousPortalEndpointA_ = endpointA;
    previousPortalEndpointB_ = endpointB;
    if (!portalCameraPositionInitialized_ || configurationChanged ||
        !portal.enabled || !portal.cameraTraversalEnabled) {
        previousPortalCameraPosition_ = normalizedPosition;
        portalCameraPositionInitialized_ = true;
        portalCameraManifold_ = {};
        return;
    }

    const float inverseDelta = deltaSeconds > 1.0e-5F
        ? 1.0F / deltaSeconds : 0.0F;
    const PortalVector normalizedVelocity =
        (normalizedPosition - previousPortalCameraPosition_) * inverseDelta;
    const float bodyRadius =
        renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal &&
                surfaceCamera_.collisionEnabled()
            ? surfaceCamera_.capsule().radius / planetRadius
            : 0.0F;
    const PortalManifoldBodyState transport = portalAdvanceBodyManifold(
        previousPortalCameraPosition_, normalizedPosition,
        viewForward, viewUp, normalizedVelocity, portal,
        portalCameraManifold_, bodyRadius, &renderer_->portalGrTable());
    if (!transport.finite) {
        portalCameraManifold_ = {};
        previousPortalCameraPosition_ = normalizedPosition;
        return;
    }
    if (transport.entered &&
        renderSettings_.cameraMode == CameraControllerMode::Orbit) {
        // Orbit has no independent six-DOF tangent frame. Preserve its current
        // pose in the dedicated manifold-aware free-fly controller at the C1
        // mouth edge, before transport begins.
        portalFreeFlyCamera_.setPose(
            cameraPosition, viewForward, viewUp,
            normalizedVelocity * planetRadius);
        renderSettings_.cameraMode = CameraControllerMode::PortalFreeFly;
        setSurfaceMouseCapture(false);
    }
    if (transport.active || transport.crossed) {
        if (renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly) {
            portalFreeFlyCamera_.setPose(
                transport.position * planetRadius,
                transport.forward, transport.up,
                transport.velocity * planetRadius);
            syncPortalFreeFlyRenderSettings();
            portal.observerManifoldActive = transport.active;
            portal.observerDestinationSide = transport.destinationSide;
        } else {
        const SurfaceCameraController::Vector mappedPosition{
            transport.position.x * planetRadius,
            transport.position.y * planetRadius,
            transport.position.z * planetRadius};
        const SurfaceCameraController::Vector mappedForward{
            transport.forward.x, transport.forward.y, transport.forward.z};
        const SurfaceCameraController::Vector mappedVelocity{
            transport.velocity.x * planetRadius,
            transport.velocity.y * planetRadius,
            transport.velocity.z * planetRadius};
        const auto terrain = [this](const SurfaceCameraController::Vector& radial,
                                    float radius, float sweep) {
            return renderer_->querySurfaceTerrain(
                radial, renderSettings_.planetRadius, radius, sweep);
        };
        surfaceCamera_.teleportToCameraPose(
            mappedPosition, mappedForward, mappedVelocity,
            planetRadius, terrain);
        renderSettings_.cameraMode = CameraControllerMode::SurfaceTraversal;
        syncSurfaceRenderSettings();
        }
    }
    if (transport.crossed) {
        ++portalCameraTransferCount_;
        portalLastSourceEndpoint_ = transport.sourceEndpoint;
    }
    portalCameraManifold_ = transport;
    previousPortalCameraPosition_ = transport.position;
#else
    static_cast<void>(deltaSeconds);
#endif
}

void Application::syncSurfaceRenderSettings() noexcept {
    const auto& radial = surfaceCamera_.radial();
    const auto& forward = surfaceCamera_.tangentForward();
    renderSettings_.surfaceRadialX = radial[0];
    renderSettings_.surfaceRadialY = radial[1];
    renderSettings_.surfaceRadialZ = radial[2];
    renderSettings_.surfaceForwardX = forward[0];
    renderSettings_.surfaceForwardY = forward[1];
    renderSettings_.surfaceForwardZ = forward[2];
    renderSettings_.surfaceClearance = std::max(
        surfaceCamera_.clearance(), minimumSurfaceClearance(renderSettings_.planetRadius));
#if VOXEL_PORTAL_LAB
    renderSettings_.surfaceCameraRadius = surfaceCamera_.cameraRadius();
#else
    renderSettings_.surfaceCameraRadius = surfaceCamera_.collisionEnabled()
        ? surfaceCamera_.cameraRadius() : 0.0F;
#endif
    renderSettings_.surfaceLookPitch = surfaceCamera_.lookPitch();
}

void Application::syncPortalFreeFlyRenderSettings() noexcept {
#if VOXEL_PORTAL_LAB
    const float inverseRadius = 1.0F /
        std::max(renderSettings_.planetRadius, 1.0e-6F);
    renderSettings_.portal.freeFlyPosition =
        portalFreeFlyCamera_.position() * inverseRadius;
    renderSettings_.portal.freeFlyForward = portalFreeFlyCamera_.forward();
    renderSettings_.portal.freeFlyUp = portalFreeFlyCamera_.up();
    renderSettings_.portal.observerManifoldActive = false;
    renderSettings_.portal.observerDestinationSide = false;
#endif
}

void Application::syncIntrinsicEllisRenderSettings() noexcept {
#if VOXEL_INTRINSIC_PORTAL_LAB
    auto& settings = renderSettings_.intrinsicEllis;
    settings.cameraState = intrinsicEllisCamera_.state();
    settings.cameraFrame = intrinsicEllisCamera_.frame();
#if VOXEL_GLOBAL_METRIC_LAB
    const GlobalHandleObserverState& global = intrinsicEllisCamera_.globalState();
    settings.globalPosition = global.position;
    settings.globalForward = global.forward;
    settings.globalUp = global.up;
    settings.globalVelocity = global.velocity;
    settings.globalLastMouth = global.lastMouth;
    settings.globalCrossings = global.crossings;
    settings.globalAffineDistance = global.affineDistance;
#endif
#endif
}

void Application::queueVoxelEdit(int mouseX, int mouseY, std::uint32_t mode) {
    int windowWidth = 0;
    int windowHeight = 0;
    SDL_GetWindowSize(window_, &windowWidth, &windowHeight);
    if (windowWidth <= 0 || windowHeight <= 0) {
        return;
    }
    if (surfaceMouseCaptured_ &&
        (renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal ||
         renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly)) {
        mouseX = windowWidth / 2;
        mouseY = windowHeight / 2;
    }
    renderSettings_.brushNdcX =
        (static_cast<float>(mouseX) + 0.5F) / static_cast<float>(windowWidth) * 2.0F - 1.0F;
    renderSettings_.brushNdcY =
        (static_cast<float>(mouseY) + 0.5F) / static_cast<float>(windowHeight) * 2.0F - 1.0F;
    renderSettings_.editMode = mode;
}

void Application::buildDevelopmentUi(float deltaSeconds) {
    const auto& stats = renderer_->stats();
#if VOXEL_FRACTAL_PLANET_SDF_LAB || VOXEL_ADAPTIVE_SDF_LAB
    ImGui::SetNextWindowPos(ImVec2(16.0F, 68.0F), ImGuiCond_FirstUseEver);
#else
    ImGui::SetNextWindowPos(ImVec2(16.0F, 16.0F), ImGuiCond_FirstUseEver);
#endif
    ImGui::SetNextWindowSize(ImVec2(385.0F, 0.0F), ImGuiCond_FirstUseEver);
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
    ImGui::Begin("ADAPTIVE VOXELIZED FRACTAL SDF Controls");
    ImGui::TextColored(ImVec4(0.30F, 0.90F, 1.0F, 1.0F),
                       "VOXELIZED FRACTAL SDF - ISOLATED EXPERIMENT");
    ImGui::Text("Virtual octree leaves: L0..L%u",
                renderSettings_.fractalPlanetSdfMaximumLevels);
    ImGui::Text("Target %.2f px | split 1.60 | merge 0.72",
                renderSettings_.fractalPlanetSdfTargetPixels);
    ImGui::Text("Resident camera ring: L%u | %.2f px | morph %.2f",
                stats.fractalVoxelCacheLevel,
                stats.fractalVoxelProjectedPixels,
                stats.fractalVoxelTransition);
    ImGui::Text("Cache transitions: %llu refine | %llu merge",
                static_cast<unsigned long long>(stats.fractalVoxelRefineCount),
                static_cast<unsigned long long>(stats.fractalVoxelMergeCount));
    ImGui::TextDisabled(renderSettings_.fractalVoxelDiscreteSurface
        ? "Hits use half-open, constant-height leaf cells with real cap/side boundaries."
        : "Hits use the smooth bilinear SDF reconstruction for A/B comparison.");
    ImGui::SeparatorText("ADAPTIVE VIRTUAL BRICK CACHE");
    ImGui::Checkbox("Enable voxelized fractal planet",
                    &renderSettings_.fractalPlanetSdfEnabled);
    int fractalLevels = static_cast<int>(
        renderSettings_.fractalPlanetSdfMaximumLevels);
    if (ImGui::SliderInt("Maximum virtual leaf level", &fractalLevels, 1, 10)) {
        renderSettings_.fractalPlanetSdfMaximumLevels =
            static_cast<std::uint32_t>(fractalLevels);
    }
    ImGui::SliderFloat("Target leaf footprint",
                       &renderSettings_.fractalPlanetSdfTargetPixels,
                       1.0F, 2.0F, "%.2f px");
    ImGui::SliderFloat("Fine-octave relief",
                       &renderSettings_.fractalPlanetSdfMicroRelief,
                       0.002F, 0.018F, "%.4f R");
    int surfaceGeometry = renderSettings_.fractalVoxelDiscreteSurface ? 0 : 1;
    constexpr const char* surfaceGeometryModes[] = {
        "Voxel surface (visible cells)", "Smooth SDF reconstruction"};
    if (ImGui::Combo("Surface geometry", &surfaceGeometry,
                     surfaceGeometryModes,
                     static_cast<int>(std::size(surfaceGeometryModes)))) {
        renderSettings_.fractalVoxelDiscreteSurface = surfaceGeometry == 0;
    }
    int fractalDebug = static_cast<int>(
        renderSettings_.fractalPlanetSdfDebugMode);
    constexpr const char* fractalDebugModes[] = {
        "Voxelized SDF shaded surface", "Leaf screen footprint",
        "Virtual brick / transition state", "Voxel cell grid"};
    if (ImGui::Combo("Voxelized SDF output", &fractalDebug,
                     fractalDebugModes,
                     static_cast<int>(std::size(fractalDebugModes)))) {
        renderSettings_.fractalPlanetSdfDebugMode =
            static_cast<std::uint32_t>(fractalDebug);
    }
    ImGui::SeparatorText("LAB SHADING (POST-HIT ONLY)");
    ImGui::SliderAngle("Sun azimuth",
                       &renderSettings_.fractalVoxelSunAzimuth,
                       -180.0F, 180.0F);
    ImGui::SliderAngle("Sun elevation",
                       &renderSettings_.fractalVoxelSunElevation,
                       5.0F, 85.0F);
    ImGui::SliderFloat("Sun strength",
                       &renderSettings_.fractalVoxelSunStrength,
                       0.0F, 2.0F, "%.2f");
    ImGui::SliderFloat("Ambient light",
                       &renderSettings_.fractalVoxelAmbientStrength,
                       0.05F, 0.65F, "%.2f");
    ImGui::SliderFloat("Local field cavity AO",
                       &renderSettings_.fractalVoxelCavityStrength,
                       0.0F, 0.45F, "%.2f");
    ImGui::SliderFloat("Interpolated normal detail",
                       &renderSettings_.fractalVoxelNormalSharpness,
                       0.0F, 1.0F, "%.2f");
    ImGui::Checkbox("Voxel cell edge overlay",
                    &renderSettings_.fractalVoxelCellEdgesEnabled);
    if (renderSettings_.fractalVoxelCellEdgesEnabled) {
        ImGui::SliderFloat("Cell edge strength",
                           &renderSettings_.fractalVoxelCellEdgeStrength,
                           0.0F, 0.40F, "%.2f");
    }
    ImGui::Checkbox("Cavity AO debug",
                    &renderSettings_.fractalVoxelCavityDebug);
    ImGui::TextDisabled(
        "Discrete mode changes actual leaf hit depth; the edge overlay remains optional.");
    ImGui::TextDisabled(
        "Approach splits cells and adds source octaves; retreat merges equivalent parents.");
    ImGui::TextDisabled(
        "2:1 parent/child morph uses shared half-open cell corners; no rendered proxy.");
    ImGui::Separator();
#elif VOXEL_FRACTAL_PLANET_SDF_LAB
    ImGui::Begin("TRUE FRACTAL PLANET SDF Controls");
    ImGui::TextColored(ImVec4(0.35F, 1.0F, 0.55F, 1.0F),
                       "TRUE MULTISCALE PLANET SDF - ISOLATED LAB");
    ImGui::Text("Procedural virtual leaves: dynamic L0..L%u",
                renderSettings_.fractalPlanetSdfMaximumLevels);
    ImGui::Text("Target leaf footprint: %.2f px | exact root: 8 steps",
                renderSettings_.fractalPlanetSdfTargetPixels);
    ImGui::TextDisabled(
        "The ray hit comes from the fractal field itself; fixed geodesic caps do not select it.");
    ImGui::SeparatorText("REAL-TIME FRACTAL VOXELIZATION");
    ImGui::Checkbox("Enable procedural fractal planet",
                    &renderSettings_.fractalPlanetSdfEnabled);
    int fractalLevels = static_cast<int>(
        renderSettings_.fractalPlanetSdfMaximumLevels);
    if (ImGui::SliderInt("Maximum virtual leaf level", &fractalLevels, 1, 10)) {
        renderSettings_.fractalPlanetSdfMaximumLevels =
            static_cast<std::uint32_t>(fractalLevels);
    }
    ImGui::SliderFloat("Target leaf footprint",
                       &renderSettings_.fractalPlanetSdfTargetPixels,
                       0.75F, 2.5F, "%.2f px");
    ImGui::SliderFloat("Fine-octave relief",
                       &renderSettings_.fractalPlanetSdfMicroRelief,
                       0.002F, 0.018F, "%.4f R");
    int fractalDebug = static_cast<int>(
        renderSettings_.fractalPlanetSdfDebugMode);
    constexpr const char* fractalDebugModes[] = {
        "True shaded SDF surface", "Active virtual leaf level",
        "Conservative interval work", "Fractal terrain height"};
    if (ImGui::Combo("Fractal planet output", &fractalDebug,
                     fractalDebugModes,
                     static_cast<int>(std::size(fractalDebugModes)))) {
        renderSettings_.fractalPlanetSdfDebugMode =
            static_cast<std::uint32_t>(fractalDebug);
    }
    ImGui::TextDisabled(
        "Approach reveals new coherent octaves and smaller leaves; retreat reverses the morph.");
    ImGui::TextDisabled(
        "No coarse square proxy. Water and terrain are one continuous radial SDF shell.");
    ImGui::Separator();
#elif VOXEL_ADAPTIVE_SDF_LAB
    ImGui::Begin("ADAPTIVE FRACTAL SDF LAB Controls");
    ImGui::TextColored(ImVec4(1.0F, 0.72F, 0.12F, 1.0F),
                       "ADAPTIVE SDF LAB ACTIVE - NOT PRODUCTION");
    ImGui::Text("Virtual refinement: dynamic L0..L%u",
                renderSettings_.adaptivePlanetSdfMaximumLevels);
    ImGui::Text("Target footprint: %.2f px | root solve: 7 steps/hit",
                renderSettings_.adaptivePlanetSdfTargetPixels);
    ImGui::TextDisabled(
        "Frequency 512 and 32 layers below are exact parent voxels, not the virtual detail limit.");
    ImGui::SeparatorText("LIVE ADAPTIVE FRACTAL SURFACE");
    ImGui::Checkbox("Enable adaptive SDF surface",
                    &renderSettings_.adaptivePlanetSdfEnabled);
    int adaptiveLevels = static_cast<int>(
        renderSettings_.adaptivePlanetSdfMaximumLevels);
    if (ImGui::SliderInt("Maximum subdivision levels", &adaptiveLevels, 1, 9)) {
        renderSettings_.adaptivePlanetSdfMaximumLevels =
            static_cast<std::uint32_t>(adaptiveLevels);
    }
    ImGui::SliderFloat("Target virtual-cell footprint",
                       &renderSettings_.adaptivePlanetSdfTargetPixels,
                       0.75F, 2.5F, "%.2f px");
    ImGui::SliderFloat("Maximum inward detail",
                       &renderSettings_.adaptivePlanetSdfLayerFraction,
                       0.02F, 0.20F, "%.3f layer");
    int adaptiveDebug = static_cast<int>(
        renderSettings_.adaptivePlanetSdfDebugMode);
    constexpr const char* adaptiveDebugModes[] = {
        "Shaded fractal relief", "Bounded displacement field",
        "Active subdivision level", "Parent-cell support mask"};
    if (ImGui::Combo("Adaptive SDF output", &adaptiveDebug,
                     adaptiveDebugModes,
                     static_cast<int>(std::size(adaptiveDebugModes)))) {
        renderSettings_.adaptivePlanetSdfDebugMode =
            static_cast<std::uint32_t>(adaptiveDebug);
        if (adaptiveDebug != 0) {
            renderSettings_.adaptivePlanetSdfComparisonView = false;
            renderSettings_.localGeodesicAoDebug = false;
        }
    }
    if (ImGui::Checkbox("A/B split: exact parent | adaptive SDF",
                        &renderSettings_.adaptivePlanetSdfComparisonView) &&
        renderSettings_.adaptivePlanetSdfComparisonView) {
        renderSettings_.adaptivePlanetSdfDebugMode = 0U;
    }
    if (ImGui::Button("SHOW FRACTAL FIELD")) {
        renderSettings_.adaptivePlanetSdfDebugMode = 1U;
        renderSettings_.adaptivePlanetSdfComparisonView = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("SHOW A/B RELIEF")) {
        renderSettings_.adaptivePlanetSdfDebugMode = 0U;
        renderSettings_.adaptivePlanetSdfComparisonView = true;
        renderSettings_.adaptivePlanetSdfEnabled = true;
    }
    ImGui::TextDisabled(
        "Debug black = water, edits, side walls, silhouette guards, or inactive subpixel detail.");
    ImGui::TextDisabled(
        "Exact DDA owns every parent hit; adaptive roots never render a coarse square surface.");
    ImGui::Separator();
#else
    ImGui::Begin("Planet Engine Development");
#endif

    ImGui::TextUnformatted("GPU FOUNDATION");
    ImGui::Separator();
    ImGui::Text("Device: %s", stats.deviceName);
    ImGui::Text("Render extent: %u x %u", stats.width, stats.height);
    ImGui::Text("Swapchain images: %u", stats.swapchainImages);
    ImGui::Text("Resident voxel volume: %u^3 (%u cells)", stats.voxelResolution, stats.voxelCount);
    ImGui::Text("Occupancy macrocells: %u", stats.macrocellCount);
    ImGui::Text("Simulation tick: %u", stats.simulationTick);
    const float frameMs = deltaSeconds * 1000.0F;
    ImGui::Text("Frame: %.2f ms (%.1f FPS)", frameMs, deltaSeconds > 0.0F ? 1.0F / deltaSeconds : 0.0F);
    if (stats.gpuTimestampsSupported) {
        ImGui::Text("GPU planet compute: %.3f ms", stats.gpuRenderMilliseconds);
    } else {
        ImGui::TextDisabled("GPU planet compute timing unavailable");
    }
    ImGui::Text("Audio: %s", audio_.available() ? "OpenAL ready" : "disabled");

    ImGui::Spacing();
    ImGui::TextUnformatted("GEODESIC PLANET TOPOLOGY");
    ImGui::Separator();
#if VOXEL_FRACTAL_PLANET_SDF_LAB
    ImGui::Text("Geodesic edit-overlay frequency: %u", stats.geodesicFrequency);
#elif VOXEL_ADAPTIVE_SDF_LAB
    ImGui::Text("Exact parent geodesic frequency: %u", stats.geodesicFrequency);
#else
    ImGui::Text("Subdivision frequency: %u", stats.geodesicFrequency);
#endif
    ImGui::Text("Surface tiles: %u", stats.surfaceTileCount);
    ImGui::Text("Hexagons: %u", stats.hexagonCount);
    ImGui::Text("Pentagons: %u (required: 12)", stats.pentagonCount);
    ImGui::Text("Resident detailed pages: %u / %u", stats.residentTileBricks,
                stats.surfaceTileCount);
#if VOXEL_FRACTAL_PLANET_SDF_LAB
    ImGui::Text("Legacy edit-overlay radial layers: %u", stats.tileBrickResolution);
    ImGui::Text("Smallest virtual leaf width: parent / %u",
                1U << renderSettings_.fractalPlanetSdfMaximumLevels);
#elif VOXEL_ADAPTIVE_SDF_LAB
    ImGui::Text("Exact macro radial layers: %u", stats.tileBrickResolution);
    ImGui::Text("Virtual SDF linear refinement per parent: up to %ux",
                1U << renderSettings_.adaptivePlanetSdfMaximumLevels);
#else
    ImGui::Text("Radial cells per column: %u", stats.tileBrickResolution);
#endif
    ImGui::Text("Voxel radial H / surface W: %.3f", stats.radialHeightWidthRatio);
    ImGui::TextDisabled("Average width %.6f | radial height %.6f",
                        stats.averageSurfaceCellWidth,
                        stats.averageRadialLayerHeight);
    ImGui::Text("Virtual geodesic capacity: %llu cells",
                static_cast<unsigned long long>(stats.tileBrickCellCapacity));
    ImGui::Text("Physical detailed cells: %llu",
                static_cast<unsigned long long>(stats.residentBrickCellCapacity));
    ImGui::Text("Page requests: %u (%u streamed)", stats.pageRequests,
                stats.pagesStreamed);
    ImGui::Text("Request samples: %u / %u columns", stats.streamRequestSamples,
                stats.surfaceTileCount);
    ImGui::Text("Request overflow: %u | stale rejected: %u",
                stats.pageRequestOverflow, stats.stalePageRequests);
    ImGui::Text("Total streamed pages: %llu",
                static_cast<unsigned long long>(stats.totalPagesStreamed));
    ImGui::Text("Reference BVH nodes: %u", stats.geodesicBvhNodeCount);
    ImGui::Text("Wide-mask nodes: %u", stats.geodesicWideNodeCount);
    ImGui::TextDisabled("Constant-time column occupancy metadata: active");
    ImGui::TextDisabled("Nonresident pages use coarse height/material state.");
    constexpr const char* kVisualizations[] = {
        "Legacy material volume", "Watertight geodesic voxels"};
    int visualization = static_cast<int>(renderSettings_.visualizationMode);
    if (ImGui::Combo("Planet view", &visualization, kVisualizations,
                     static_cast<int>(std::size(kVisualizations)))) {
        renderSettings_.visualizationMode = static_cast<std::uint32_t>(visualization);
    }
    ImGui::TextDisabled("Shared side and cap planes meet without gaps.");

#if VOXEL_INTRINSIC_PORTAL_LAB
    ImGui::Spacing();
    ImGui::TextUnformatted(
#if VOXEL_GLOBAL_METRIC_LAB
        "GLOBAL STATIC SPACETIME METRIC FIELD");
#else
        "INTRINSIC ELLIS MANIFOLD");
#endif
    ImGui::Separator();
    auto& intrinsic = renderSettings_.intrinsicEllis;
    ImGui::Checkbox("Intrinsic renderer enabled", &intrinsic.enabled);
    intrinsic.throatRadius =
#if VOXEL_GLOBAL_METRIC_LAB
        intrinsic.contentSphereRadius * kPortalGrThroatRatio;
    intrinsic.contentExitProperDepth = std::sqrt(std::max(
        intrinsic.contentSphereRadius * intrinsic.contentSphereRadius -
            intrinsic.throatRadius * intrinsic.throatRadius, 1.0e-8F));
#else
        kPortalGrThroatRatio;
    intrinsic.contentExitProperDepth = std::sqrt(
        1.0F - kPortalGrThroatRatio * kPortalGrThroatRatio);
#endif
    ImGui::Text("Physical topology throat radius a: %.3f",
                intrinsic.throatRadius);
#if VOXEL_GLOBAL_METRIC_LAB
    if (intrinsic.globalNativeEllisPath) {
        ImGui::Text("Native signed proper depth l: %.6f | throat l=0",
                    intrinsicEllisCamera_.state().properDepth);
        ImGui::TextDisabled(
            "No Euclidean mouth sphere or lastMouth visibility owner is active.");
    } else {
        ImGui::TextDisabled(
            "LEGACY comparison: spherical mouth remap is active and rejected.");
    }
#else
    ImGui::Text("Native camera l: %.6f | throat l=0",
                intrinsicEllisCamera_.state().properDepth);
#endif
    ImGui::Text("Free-fly status: %s",
                intrinsicEllisCamera_.motionStatusText(intrinsic));
    ImGui::TextUnformatted(
        "Topology: two-ended all-space handle with linked content embeddings");
#if VOXEL_GLOBAL_METRIC_LAB
    ImGui::TextUnformatted(intrinsic.globalNativeEllisPath
        ? "Metric API: exact static Ellis dl^2+(l^2+a^2)dOmega^2"
        : "Metric API: LEGACY engineered conformal handle + sphere remap");
    ImGui::TextUnformatted(intrinsic.globalNativeEllisPath
        ? "Runtime rays/camera: one native signed-l atlas through l=0"
        : "Runtime rays: rejected shared-exterior comparison path");
    ImGui::TextDisabled(intrinsic.globalNativeEllisPath
        ? "A/B are asymptotic content attachments, not two Euclidean aperture spheres."
        : "This mode is retained only for automated historical regressions.");
    constexpr const char* metricIntegratorQuality[] = {
        "Low (embedded midpoint)", "Medium (embedded midpoint)",
        "High (embedded midpoint)"};
    int metricQuality = static_cast<int>(intrinsic.integrationQuality);
    if (ImGui::Combo("Global geodesic quality", &metricQuality,
                     metricIntegratorQuality,
                     static_cast<int>(std::size(metricIntegratorQuality)))) {
        intrinsic.integrationQuality = static_cast<std::uint32_t>(metricQuality);
    }
    ImGui::Checkbox("Geodesic-Jacobian spatial AA",
                    &intrinsic.globalSpatialAaEnabled);
    ImGui::SliderFloat("AA distortion threshold",
                       &intrinsic.globalSpatialAaDistortionThreshold,
                       1.05F, 8.0F, "%.2f");
    ImGui::SliderFloat("AA magnification threshold",
                       &intrinsic.globalSpatialAaMagnificationThreshold,
                       1.10F, 16.0F, "%.2f");
    const double aaPercent = stats.globalSpatialAaPixelSamples != 0U
        ? 100.0 * static_cast<double>(stats.globalSpatialAaPixels) /
              static_cast<double>(stats.globalSpatialAaPixelSamples)
        : 0.0;
    ImGui::Text("Adaptive AA pixels: %.2f%% | subrays: %llu",
                aaPercent,
                static_cast<unsigned long long>(stats.globalSpatialAaSubrays));
    ImGui::TextDisabled(
        "1x normally; deterministic rotated 4x only at classified distortion/coverage.");
#endif
    if (intrinsic.globalNativeEllisPath) {
        ImGui::TextUnformatted(
            "Content end A: +l | content end B: -l | l=0 is regular");
    } else {
        ImGui::Text("LEGACY Mouth A/B | last entry: %s",
                    intrinsicEllisCamera_.lastEnteredMouthText());
    }
    ImGui::Text("Angular chart n: %.4f %.4f %.4f",
                intrinsicEllisCamera_.state().angularPosition.x,
                intrinsicEllisCamera_.state().angularPosition.y,
                intrinsicEllisCamera_.state().angularPosition.z);
    ImGui::Text("Frame handedness: %.6f",
                intrinsicEllisCamera_.handedness());
    ImGui::SliderFloat(
#if VOXEL_GLOBAL_METRIC_LAB
                       "Handle scale / camera reset standoff",
#else
                       "Content chart sphere radius",
#endif
                       &intrinsic.contentSphereRadius,
                       0.12F, 0.65F, "%.3f R");
    ImGui::SliderAngle("End B orientation yaw", &intrinsic.endBYaw,
                       -180.0F, 180.0F);
    ImGui::SliderAngle("End B orientation pitch", &intrinsic.endBPitch,
                       -90.0F, 90.0F);
    ImGui::SliderAngle("End B orientation roll", &intrinsic.endBRoll,
                       -180.0F, 180.0F);
    constexpr const char* intrinsicDebugModes[] = {
        "Shaded content", "Asymptotic end", "Proper-depth bands",
        "Geodesic table work",
        "Finite/handedness validation (red = failure)",
        "Ray path class (blue direct, green RK4, amber critical, magenta failure)",
        "Metric curvature (dark = flat limit)"
#if VOXEL_GLOBAL_METRIC_LAB
        , "Spatial AA sample count", "Jacobian determinant / parity",
        "Jacobian magnification", "AA family rejection / coverage",
        "Content query shell (cyan = crossed; never owns path)"
#endif
    };
    int intrinsicDebug = static_cast<int>(intrinsic.debugMode);
    if (ImGui::Combo("Intrinsic output", &intrinsicDebug,
                     intrinsicDebugModes,
                     static_cast<int>(std::size(intrinsicDebugModes)))) {
        intrinsic.debugMode = static_cast<std::uint32_t>(intrinsicDebug);
    }
    ImGui::Text("Metric cache: %s | %.3f ms | %.1f KiB | error %.7g",
                stats.portalGrCacheHit ? "warm" : "cold",
                stats.portalGrPrecomputeSeconds * 1000.0,
                static_cast<double>(stats.portalGrTableBytes) / 1024.0,
                stats.portalGrMaximumError);
    ImGui::Text("Last frame rays: %llu | refined: %llu | nonfinite: %llu",
                static_cast<unsigned long long>(stats.portalGrTableLookups),
                static_cast<unsigned long long>(stats.portalGrLocalRefinements),
                static_cast<unsigned long long>(stats.portalGrNonfiniteRays));
    ImGui::Text("Content exits: +end %llu | -end %llu",
                static_cast<unsigned long long>(
                    stats.portalGrTableLookups >= stats.portalGrNegativeEndRays
                        ? stats.portalGrTableLookups -
                              stats.portalGrNegativeEndRays
                        : 0U),
                static_cast<unsigned long long>(
                    stats.portalGrNegativeEndRays));
    ImGui::TextDisabled(
        "ds2=dl2+(l2+a2)dOmega2; camera and rays share this native manifold.");
    ImGui::TextDisabled(
        "No influence sphere, compact collar, screen warp, or frame reset.");
    ImGui::TextDisabled(
        "Curvature decays analytically; exact DDA/media attach at a common smooth content-query surface.");
#elif VOXEL_PORTAL_LAB
    ImGui::Spacing();
    ImGui::TextUnformatted("SPHERICAL WORMHOLE PORTAL LAB");
    ImGui::Separator();
    auto& portal = renderSettings_.portal;
    if (ImGui::Button("Reset linked portal pair")) {
        portal = PortalLabSettings{};
        if (renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly) {
            portalFreeFlyCamera_.resetForPortal(portal, renderSettings_.planetRadius);
            syncPortalFreeFlyRenderSettings();
        }
        portalCameraManifold_ = {};
        portalCameraPositionInitialized_ = false;
    }
    ImGui::SameLine();
    if (ImGui::Button("Near-surface traversal preset")) {
        portal = PortalLabSettings{};
        portal.endpointAAzimuth = 0.12F;
        portal.endpointADistance = stats.planetOuterScale + 0.07F;
        portal.endpointBDistance = stats.planetOuterScale + 0.07F;
        portal.influenceRadius = 0.18F;
        portal.throatRadius = 0.075F;
        if (renderSettings_.cameraMode == CameraControllerMode::PortalFreeFly) {
            portalFreeFlyCamera_.resetForPortal(portal, renderSettings_.planetRadius);
            syncPortalFreeFlyRenderSettings();
        }
        portalCameraManifold_ = {};
        portalCameraPositionInitialized_ = false;
    }
    ImGui::SameLine();
    ImGui::Checkbox("Curved-ray portal", &portal.enabled);
    ImGui::Checkbox("Shared curved-space camera/player traversal",
                    &portal.cameraTraversalEnabled);
    ImGui::SliderAngle("Portal A azimuth", &portal.endpointAAzimuth,
                       -180.0F, 180.0F);
    ImGui::SliderAngle("Portal A elevation", &portal.endpointAElevation,
                       -75.0F, 75.0F);
    ImGui::SliderAngle("Portal B azimuth", &portal.endpointBAzimuth,
                       -180.0F, 180.0F);
    ImGui::SliderAngle("Portal B elevation", &portal.endpointBElevation,
                       -75.0F, 75.0F);
    ImGui::SliderFloat("Portal A radial distance", &portal.endpointADistance,
                       1.0F, 3.0F, "%.3f R");
    ImGui::SliderFloat("Portal B radial distance", &portal.endpointBDistance,
                       1.0F, 3.0F, "%.3f R");
    ImGui::SliderFloat("Influence radius", &portal.influenceRadius,
                       0.06F, 0.55F, "%.3f R");
    portal.throatRadius = portal.influenceRadius * kPortalGrThroatRatio;
    ImGui::Text("Fixed Ellis throat: %.3f R (a/Rmouth = %.2f)",
                portal.throatRadius, kPortalGrThroatRatio);
    ImGui::SliderAngle("Link yaw", &portal.linkYaw, -180.0F, 180.0F);
    ImGui::SliderAngle("Link pitch", &portal.linkPitch, -90.0F, 90.0F);
    ImGui::SliderAngle("Link roll", &portal.linkRoll, -180.0F, 180.0F);
    constexpr const char* portalQualities[] = {
        "Low local GR refinement", "Medium local GR refinement",
        "High local GR refinement"};
    int portalQuality = static_cast<int>(portal.quality);
    if (ImGui::Combo("Portal integration quality", &portalQuality,
                     portalQualities,
                     static_cast<int>(std::size(portalQualities)))) {
        portal.quality = static_cast<std::uint32_t>(portalQuality);
    }
    constexpr const char* portalDebugModes[] = {
        "Shaded destination", "Influence mask", "Throat transfer",
        "Optical-step heatmap", "Finite-state validation"};
    int portalDebug = static_cast<int>(portal.debugMode);
    if (ImGui::Combo("Portal output", &portalDebug, portalDebugModes,
                     static_cast<int>(std::size(portalDebugModes)))) {
        portal.debugMode = static_cast<std::uint32_t>(portalDebug);
    }
    constexpr std::uint32_t kMinimumPortalTraversals = 1U;
    constexpr std::uint32_t kMaximumPortalTraversals = 4U;
    ImGui::SliderScalar("Maximum throat traversals", ImGuiDataType_U32,
                        &portal.maximumTraversals,
                        &kMinimumPortalTraversals, &kMaximumPortalTraversals);
    ImGui::Text("GR precompute: %s | %s | %.3f ms | %.1f KiB",
                stats.portalGrPrecomputeReady ? "ready" : "not ready",
                stats.portalGrCacheHit ? "warm cache" : "cold build",
                stats.portalGrPrecomputeSeconds * 1000.0,
                static_cast<double>(stats.portalGrTableBytes) / 1024.0);
    ImGui::Text("Certified table error: %.7g | version %u",
                stats.portalGrMaximumError, kPortalGrTableVersion);
    ImGui::Text("Previous frame GR queries: %llu | exact refinements: %llu",
                static_cast<unsigned long long>(stats.portalGrTableLookups),
                static_cast<unsigned long long>(stats.portalGrLocalRefinements));
    ImGui::TextDisabled(
        "Static ultrastatic Ellis metric: ds2=-dt2+dl2+(l2+a2)dOmega2; not evolving Einstein equations.");
    ImGui::TextDisabled(
        "Cyan throat rays continue through exact geodesic DDA, atmosphere, clouds, and stars in the linked chart.");
    const float mediumOuterRadius = stats.planetOuterScale +
        std::max(renderSettings_.atmosphere.endHeight,
                 renderSettings_.clouds.topAltitude);
    if (std::min(portal.endpointADistance, portal.endpointBDistance) -
            portal.influenceRadius < mediumOuterRadius) {
        ImGui::TextColored(ImVec4(0.95F, 0.72F, 0.25F, 1.0F),
            "Near-surface mode: portal overlaps participating media; exact terrain still wins in front.");
    } else {
        ImGui::TextDisabled(
            "Portal volumes are outside participating media; opaque terrain in front always wins.");
    }
    ImGui::Text("Completed chart traversals: %llu",
                static_cast<unsigned long long>(portalCameraTransferCount_));
    ImGui::Text("Manifold state: %s | proper depth %.3f",
                portalCameraManifold_.active
                    ? portalCameraManifold_.destinationSide
                        ? "destination chart" : "source chart"
                    : "ordinary space",
                portalCameraManifold_.properProgress);
    ImGui::Text("Observer chart: %s | forward %.3f %.3f %.3f",
                portalCameraManifold_.active
                    ? portalCameraManifold_.destinationSide
                        ? "destination GR half" : "source GR half"
                    : "ordinary space",
                portalCameraManifold_.forward.x,
                portalCameraManifold_.forward.y,
                portalCameraManifold_.forward.z);
    ImGui::Text("Observer up: %.3f %.3f %.3f",
                portalCameraManifold_.up.x,
                portalCameraManifold_.up.y,
                portalCameraManifold_.up.z);
    if (portalLastSourceEndpoint_ < 2U) {
        ImGui::SameLine();
        ImGui::TextDisabled("last: %c -> %c",
                            portalLastSourceEndpoint_ == 0U ? 'A' : 'B',
                            portalLastSourceEndpoint_ == 0U ? 'B' : 'A');
    }
    ImGui::TextDisabled(
        "The observer basis is transported once and destination rays continue to the same GR exit before terrain shading.");
#endif

#if !VOXEL_SCALE_LAB && !VOXEL_ADAPTIVE_SDF_LAB && !VOXEL_FRACTAL_PLANET_SDF_LAB
    ImGui::Spacing();
    ImGui::TextUnformatted("LIGHTING");
    ImGui::Separator();
    auto& lighting = renderSettings_.lighting;
    if (ImGui::Button("Noon preset")) {
        lighting = PlanetLightingSettings{};
    }
    ImGui::SameLine();
    if (ImGui::Button("Sunset preset")) {
        lighting.sunAzimuth = -1.18F;
        lighting.sunElevation = 0.16F;
        lighting.sunColor = {1.0F, 0.34F, 0.10F};
        lighting.sunIntensity = 1.35F;
        lighting.skyZenithColor = {0.07F, 0.12F, 0.28F};
        lighting.skyHorizonColor = {0.72F, 0.25F, 0.13F};
        lighting.skyIntensity = 0.38F;
        lighting.contactShadowStrength = 0.28F;
        lighting.exposure = 1.10F;
    }
    ImGui::SliderAngle("Sun azimuth", &lighting.sunAzimuth,
                       -180.0F, 180.0F);
    ImGui::SliderAngle("Sun elevation", &lighting.sunElevation,
                       1.0F, 89.0F);
    ImGui::ColorEdit3("Sun color", lighting.sunColor.data());
    ImGui::SliderFloat("Sun intensity", &lighting.sunIntensity,
                       0.0F, 3.0F, "%.2f");
    ImGui::ColorEdit3("Sky zenith", lighting.skyZenithColor.data());
    ImGui::ColorEdit3("Sky horizon", lighting.skyHorizonColor.data());
    ImGui::SliderFloat("Sky ambient", &lighting.skyIntensity,
                       0.0F, 1.25F, "%.2f");
    ImGui::SliderFloat("Surface roughness", &lighting.roughness,
                       0.12F, 1.0F, "%.2f");
    ImGui::SliderFloat("Specular strength", &lighting.specularStrength,
                       0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Gentle rim", &lighting.rimStrength,
                       0.0F, 0.25F, "%.3f");
    ImGui::SliderFloat("Exposure", &lighting.exposure,
                       0.25F, 2.5F, "%.2f");
    ImGui::SliderFloat("Local contact horizon",
                       &lighting.contactShadowStrength,
                       0.0F, 0.65F, "%.2f");
    ImGui::Checkbox("Local voxel AO", &renderSettings_.localGeodesicAoEnabled);
    ImGui::SliderFloat("Local voxel AO strength",
                       &renderSettings_.localGeodesicAoStrength,
                       0.0F, 0.30F, "%.3f");
    if (ImGui::Checkbox("Local voxel AO debug",
                        &renderSettings_.localGeodesicAoDebug) &&
        renderSettings_.localGeodesicAoDebug) {
        renderSettings_.ddaWorkHeatmapDebug = false;
    }
    constexpr const char* lightingDebugModes[] = {
        "Shaded", "Sun diffuse", "Sky + local occlusion", "World normals"};
    int lightingDebug = static_cast<int>(lighting.debugMode);
    if (ImGui::Combo("Lighting output", &lightingDebug,
                     lightingDebugModes,
                     static_cast<int>(std::size(lightingDebugModes)))) {
        lighting.debugMode = static_cast<std::uint32_t>(lightingDebug);
    }
    ImGui::TextDisabled(
        "World-space sun; radial sky hemisphere; camera-independent one-ring AO/horizon.");
#endif

#if !VOXEL_SCALE_LAB && !VOXEL_ADAPTIVE_SDF_LAB && !VOXEL_FRACTAL_PLANET_SDF_LAB
    ImGui::Spacing();
    ImGui::TextUnformatted("PLANET ATMOSPHERE");
    ImGui::Separator();
    auto& atmosphere = renderSettings_.atmosphere;
    if (ImGui::Button("Warm small-planet preset")) {
        atmosphere = PlanetAtmosphereSettings{};
        lighting.sunColor = {1.0F, 0.82F, 0.55F};
        lighting.skyZenithColor = {0.10F, 0.47F, 0.66F};
        lighting.skyHorizonColor = {0.32F, 0.68F, 0.63F};
        lighting.skyIntensity = 0.30F;
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset atmosphere")) {
        atmosphere = PlanetAtmosphereSettings{};
    }
    ImGui::Checkbox("Volumetric spherical atmosphere", &atmosphere.enabled);
    ImGui::Checkbox("Physical surface atmosphere", &atmosphere.physicalSurface);
    ImGui::Checkbox("Artistic near-surface fade (optional)",
                    &atmosphere.artisticNearFade);
    if (ImGui::SliderFloat("Atmosphere start height",
                           &atmosphere.startHeight, 0.0F,
                           kAtmosphereMaximumStartHeight, "%.3f R")) {
        atmosphere.endHeight = std::max(
            atmosphere.endHeight, atmosphere.startHeight + 0.005F);
    }
    if (ImGui::SliderFloat("Atmosphere end height", &atmosphere.endHeight,
                           0.03F, kAtmosphereMaximumEndHeight, "%.3f R")) {
        atmosphere.startHeight = std::min(
            atmosphere.startHeight, atmosphere.endHeight - 0.005F);
    }
    ImGui::SliderFloat("Atmosphere density", &atmosphere.density,
                       0.0F, kAtmosphereMaximumDensity, "%.2f");
    ImGui::SliderFloat("Rayleigh scattering",
                       &atmosphere.rayleighScattering, 0.0F,
                       kAtmosphereMaximumCoefficient, "%.2f");
    ImGui::SliderFloat("Mie scattering", &atmosphere.mieScattering,
                       0.0F, kAtmosphereMaximumCoefficient, "%.2f");
    ImGui::SliderFloat("Spectral absorption", &atmosphere.absorption,
                       0.0F, kAtmosphereMaximumCoefficient, "%.2f");
    ImGui::SliderFloat("Density scale height", &atmosphere.scaleHeight,
                       kAtmosphereMinimumScaleHeight,
                       kAtmosphereMaximumScaleHeight, "%.2f shell");
    ImGui::SliderFloat("Mie forward anisotropy",
                       &atmosphere.mieAnisotropy,
                       kAtmosphereMinimumMieAnisotropy,
                       kAtmosphereMaximumMieAnisotropy, "%.2f");
    ImGui::Checkbox("SpaceBattle depth-integrated optics",
                    &atmosphere.depthIntegrated);
    constexpr const char* atmosphereIntegrationQualities[] = {
        "Legacy analytic (1 view)", "Low (2 view / 2 sun)",
        "Medium (3 view / 2 sun)", "High (5 view / 4 sun)"};
    int atmosphereIntegrationQuality = static_cast<int>(
        atmosphere.integrationQuality);
    if (ImGui::Combo("Atmosphere integration quality",
                     &atmosphereIntegrationQuality,
                     atmosphereIntegrationQualities,
                     static_cast<int>(std::size(
                         atmosphereIntegrationQualities)))) {
        atmosphere.integrationQuality = static_cast<std::uint32_t>(
            atmosphereIntegrationQuality);
    }
    ImGui::SliderFloat("Derived night scattering",
                       &atmosphere.nightScattering, 0.0F,
                       kAtmosphereMaximumNightScattering, "%.3f");
    ImGui::Checkbox("Terrain atmosphere shadows", &atmosphere.terrainShadows);
    constexpr const char* terrainShadowQualities[] = {
        "Off", "Low (4 columns)", "Medium (8 columns)",
        "High (16 columns)"};
    int terrainShadowQuality = static_cast<int>(
        atmosphere.terrainShadowQuality);
    if (ImGui::Combo("Terrain shadow quality", &terrainShadowQuality,
                     terrainShadowQualities,
                     static_cast<int>(std::size(terrainShadowQualities)))) {
        atmosphere.terrainShadowQuality = static_cast<std::uint32_t>(
            terrainShadowQuality);
    }
    ImGui::SliderFloat("Terrain shadow distance",
                       &atmosphere.terrainShadowDistance,
                       kAtmosphereMinimumTerrainShadowDistance,
                       kAtmosphereMaximumTerrainShadowDistance, "%.3f R");
    ImGui::SliderFloat("Terrain shadow strength",
                       &atmosphere.terrainShadowStrength,
                       0.0F, 1.0F, "%.2f");
    constexpr const char* atmosphereDebugModes[] = {
        "Composited", "RGB transmittance", "Total in-scattering",
        "Rayleigh only", "Mie only", "Absorption", "Optical depth",
        "Terrain sun visibility"};
    int atmosphereDebug = static_cast<int>(atmosphere.debugMode);
    if (ImGui::Combo("Atmosphere output", &atmosphereDebug,
                     atmosphereDebugModes,
                     static_cast<int>(std::size(atmosphereDebugModes)))) {
        atmosphere.debugMode = static_cast<std::uint32_t>(atmosphereDebug);
    }
    ImGui::TextDisabled(
        atmosphere.physicalSurface
            ? "Physical mode integrates from the shell entry all the way to the exact terrain hit."
            : "Legacy raised-base mode is active; enable Physical surface atmosphere for ground attenuation.");
    ImGui::TextDisabled(
        atmosphere.depthIntegrated
            ? "SpaceBattle-derived fixed quadrature: ordered spectral extinction, limb/terminator shaping, and derived night scatter."
            : "Legacy one-point atmosphere integration is active for comparison.");
    ImGui::TextDisabled(
        "Bounded geodesic column horizon shadows are world-stable and use residency-independent terrain heights.");
    ImGui::TextDisabled(
        "Stars remain behind the shell; terrain hit depth and voxel detail are unchanged.");
#endif

#if !VOXEL_SCALE_LAB && !VOXEL_ADAPTIVE_SDF_LAB && !VOXEL_FRACTAL_PLANET_SDF_LAB
    ImGui::Spacing();
    ImGui::TextUnformatted("VOLUMETRIC CLOUDS");
    ImGui::Separator();
    auto& clouds = renderSettings_.clouds;
    ImGui::Checkbox("Volumetric cloud shell", &clouds.enabled);
    constexpr const char* cloudSources[] = {
        "Procedural 3D density", "Authored 3D texture (future / procedural fallback)"};
    int cloudSource = static_cast<int>(clouds.source);
    if (ImGui::Combo("Cloud density source", &cloudSource, cloudSources,
                     static_cast<int>(std::size(cloudSources)))) {
        clouds.source = static_cast<CloudDensitySource>(cloudSource);
    }
    if (clouds.source == CloudDensitySource::Authored3DTexture) {
        ImGui::TextDisabled(
            "No authored volume is bound; the source interface safely falls back to procedural 3D density.");
    }
    ImGui::SliderFloat("Cloud coverage", &clouds.coverage, 0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Cloud density", &clouds.density, 0.0F, 3.0F, "%.2f");
    if (ImGui::SliderFloat("Cloud base altitude", &clouds.baseAltitude,
                           0.01F, 0.25F, "%.3f R")) {
        clouds.topAltitude = std::max(clouds.topAltitude,
                                      clouds.baseAltitude + 0.01F);
    }
    if (ImGui::SliderFloat("Cloud top altitude", &clouds.topAltitude,
                           0.02F, 0.30F, "%.3f R")) {
        clouds.baseAltitude = std::min(clouds.baseAltitude,
                                       clouds.topAltitude - 0.01F);
    }
    ImGui::SliderFloat("Cloud shape scale", &clouds.shapeScale,
                       1.0F, 16.0F, "%.1f");
    ImGui::SliderFloat("Cloud detail strength", &clouds.detailStrength,
                       0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Cloud wind speed", &clouds.windSpeed,
                       0.0F, 0.10F, "%.3f R/s");
    ImGui::SliderAngle("Cloud wind direction", &clouds.windDirection,
                       -180.0F, 180.0F);
    constexpr const char* cloudQualities[] = {
        "Off", "Low (3 view samples)", "Medium (6 view samples)",
        "High (10 view samples)"};
    int cloudQuality = static_cast<int>(clouds.quality);
    if (ImGui::Combo("Cloud quality", &cloudQuality, cloudQualities,
                     static_cast<int>(std::size(cloudQualities)))) {
        clouds.quality = static_cast<std::uint32_t>(cloudQuality);
    }
    constexpr const char* cloudShadowQualities[] = {
        "Off", "Low (2 sun samples)", "Medium (4 sun samples)",
        "High (6 sun samples)"};
    int cloudShadowQuality = static_cast<int>(clouds.sunShadowQuality);
    if (ImGui::Combo("Cloud sun-shadow quality", &cloudShadowQuality,
                     cloudShadowQualities,
                     static_cast<int>(std::size(cloudShadowQualities)))) {
        clouds.sunShadowQuality = static_cast<std::uint32_t>(cloudShadowQuality);
    }
    ImGui::SliderFloat("Cloud terrain-shadow strength",
                       &clouds.sunShadowStrength, 0.0F, 1.0F, "%.2f");
    constexpr const char* cloudDebugModes[] = {
        "Composited", "Density integral", "Cloud transmittance",
        "Cloud sun shadow"};
    int cloudDebug = static_cast<int>(clouds.debugMode);
    if (ImGui::Combo("Cloud output", &cloudDebug, cloudDebugModes,
                     static_cast<int>(std::size(cloudDebugModes)))) {
        clouds.debugMode = static_cast<std::uint32_t>(cloudDebug);
    }
    ImGui::TextDisabled(
        clouds.windSpeed > 0.0F && !renderSettings_.animate
            ? "Wind is configured but paused; enable Animate below to advance world-space cloud time."
            : "Deterministic world-space 3D density; camera translation does not move the cloud field.");
#endif

#if !VOXEL_SCALE_LAB && !VOXEL_ADAPTIVE_SDF_LAB && !VOXEL_FRACTAL_PLANET_SDF_LAB
    ImGui::Spacing();
    ImGui::TextUnformatted("SPACE ENVIRONMENT");
    ImGui::Separator();
    auto& space = renderSettings_.spaceEnvironment;
    ImGui::Checkbox("Black procedural starfield", &space.enabled);
    ImGui::SliderFloat("Star density", &space.density,
                       0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Star brightness", &space.brightness,
                       0.0F, 2.5F, "%.2f");
    ImGui::SliderFloat("Star size", &space.size,
                       0.5F, 2.5F, "%.2f px");
    ImGui::Checkbox("Camera-aligned stars", &space.cameraAligned);
    ImGui::Checkbox("Starfield debug", &space.debug);
    ImGui::TextDisabled(space.cameraAligned
        ? "Camera-centered and view-aligned: the pattern stays fixed to the screen."
        : "Camera-centered and world-oriented: no translation parallax; turning reveals sky.");
    ImGui::TextDisabled(
        "Static stars; forward planet-horizon exclusion prevents false speckles without masking the antipodal sky.");
#endif

    ImGui::Spacing();
    ImGui::TextUnformatted("PROCEDURAL PLANET TERRAIN");
    ImGui::Separator();
    ImGui::InputScalar("Seed", ImGuiDataType_U32, &renderSettings_.terrain.seed);
    ImGui::SliderFloat("Continent scale", &renderSettings_.terrain.continentScale,
                       0.25F, 5.0F, "%.2f");
    ImGui::SliderFloat("Continent / valley relief", &renderSettings_.terrain.continentStrength,
                       0.0F, 1.75F, "%.2f");
    ImGui::SliderFloat("Mountain scale", &renderSettings_.terrain.mountainScale,
                       2.0F, 24.0F, "%.1f");
    ImGui::SliderFloat("Mountain relief", &renderSettings_.terrain.mountainStrength,
                       0.0F, 1.0F, "%.2f");
    ImGui::SliderFloat("Roughness", &renderSettings_.terrain.roughness,
                       0.15F, 0.85F, "%.2f");
    ImGui::SliderFloat("Ocean coverage", &renderSettings_.terrain.oceanLevel,
                       0.20F, 0.78F, "%.2f");
    ImGui::SliderFloat("Polar uplift", &renderSettings_.terrain.polarStrength,
                       0.0F, 0.40F, "%.2f");
    if (ImGui::Button("Generate terrain")) {
        renderSettings_.regenerateTerrain = true;
    }
    ImGui::SameLine();
    ImGui::Text("Generation %u", stats.terrainGeneration);
    ImGui::TextDisabled("Three blended orthographic projections; no UV seam or poles.");
    ImGui::TextDisabled("Height, occupancy, and resident voxel materials are generated on GPU.");

    const auto& physicsStats = physics_.stats();
    ImGui::Spacing();
    ImGui::TextUnformatted("RIGID-BODY PHYSICS");
    ImGui::Separator();
    ImGui::Text("Backend: %s", physicsStats.backend);
    ImGui::Text("Bodies: %u (%u active)", physicsStats.bodyCount, physicsStats.activeBodyCount);
    ImGui::Text("Fixed-step tick: %llu", static_cast<unsigned long long>(physicsStats.simulationTick));

    ImGui::Spacing();
    ImGui::TextUnformatted("HYBRID GPU TRAVERSAL");
    ImGui::Separator();
    if (stats.geodesicBvhNodeCount != 0U) {
        constexpr const char* kTraversalModes[] = {
            "Reference binary BVH", "Wide 64-bit masks", "Geodesic neighbor DDA"};
        int traversalMode = static_cast<int>(renderSettings_.geodesicTraversalMode);
        if (ImGui::Combo("Surface traversal", &traversalMode, kTraversalModes,
                         static_cast<int>(std::size(kTraversalModes)))) {
            renderSettings_.geodesicTraversalMode = static_cast<std::uint32_t>(traversalMode);
        }
    } else {
        if (renderSettings_.geodesicTraversalMode < 2U ||
            renderSettings_.geodesicTraversalMode >
#if VOXEL_SCALE_LAB
                8U
#else
                7U
#endif
            ) {
            renderSettings_.geodesicTraversalMode = 2U;
        }
#if VOXEL_SCALE_LAB
        constexpr const char* kProductionTraversalNames[] = {
            "exact DDA + conservative empty-space bound", "hit-face classification",
            "flat hit coverage", "artifact regression capture",
            "per-pixel ray status", "failure-only overlay",
            "exact distant-horizon capture"};
#else
        constexpr const char* kProductionTraversalNames[] = {
            "production geodesic DDA", "hit-face classification", "flat hit coverage",
            "artifact regression capture", "per-pixel ray status",
            "failure-only overlay"};
#endif
        ImGui::TextDisabled("Surface traversal: %s",
            kProductionTraversalNames[renderSettings_.geodesicTraversalMode - 2U]);
    }
#if VOXEL_SCALE_LAB || VOXEL_ADAPTIVE_SDF_LAB || VOXEL_FRACTAL_PLANET_SDF_LAB
    ImGui::Checkbox("Local voxel AO", &renderSettings_.localGeodesicAoEnabled);
    ImGui::SliderFloat("Local voxel AO strength",
                       &renderSettings_.localGeodesicAoStrength,
                       0.0F, 0.30F, "%.3f");
    if (ImGui::Checkbox("Local voxel AO debug",
                        &renderSettings_.localGeodesicAoDebug) &&
        renderSettings_.localGeodesicAoDebug) {
        renderSettings_.ddaWorkHeatmapDebug = false;
    }
    ImGui::TextDisabled(
        "Camera-invariant one-ring height horizon; dark means locally occluded.");
#endif
#if !VOXEL_SCALE_LAB && !VOXEL_ADAPTIVE_SDF_LAB
    ImGui::SeparatorText("Terrain micro detail");
    ImGui::Checkbox("Bounded fractal micro-SDF",
                    &renderSettings_.terrainMicroSdfEnabled);
    ImGui::SliderFloat("Micro-SDF depth",
                       &renderSettings_.terrainMicroSdfStrength,
                       0.0F, 0.175F, "%.3f");
    if (ImGui::Checkbox("Micro-SDF debug",
                        &renderSettings_.terrainMicroSdfDebug) &&
        renderSettings_.terrainMicroSdfDebug) {
        renderSettings_.localGeodesicAoDebug = false;
    }
    ImGui::TextDisabled(
        "Cap-only inward relief; edits, water, cell edges, and silhouettes stay exact.");
#endif
#if VOXEL_SCALE_LAB
    if (ImGui::Checkbox("DDA work heatmap",
                        &renderSettings_.ddaWorkHeatmapDebug) &&
        renderSettings_.ddaWorkHeatmapDebug) {
        renderSettings_.localGeodesicAoDebug = false;
    }
    ImGui::TextDisabled(
        "Diagnostic only: white means more exact leaf traversal events.");
#endif
    bool rayStatusOverlay = renderSettings_.geodesicTraversalMode == 6U;
    if (ImGui::Checkbox("Per-pixel ray status (F8)", &rayStatusOverlay)) {
        renderSettings_.geodesicTraversalMode = rayStatusOverlay ? 6U : 2U;
    }
    bool failureOnlyOverlay = renderSettings_.geodesicTraversalMode == 7U;
    if (ImGui::Checkbox("Failure-only overlay (F9)", &failureOnlyOverlay)) {
        renderSettings_.geodesicTraversalMode = failureOnlyOverlay ? 7U : 2U;
        rayStatusOverlay = false;
    }
    if (failureOnlyOverlay) {
        ImGui::TextDisabled(
            "Normal shading is preserved; only confirmed lost hits are recolored.");
        ImGui::TextColored(ImVec4(1.0F, 0.0F, 1.0F, 1.0F),
                           "Magenta/red/yellow pixels are traversal failures.");
        ImGui::TextDisabled(
            "Rolling 1/32 audit: every pixel is checked over 32 frames.");
        ImGui::TextDisabled("Detailed readback is disabled in this lightweight mode.");
    }
    if (rayStatusOverlay) {
        ImGui::TextColored(ImVec4(0.1F, 0.85F, 0.28F, 1.0F), "Green: cap hit");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.0F, 0.85F, 0.9F, 1.0F), "Cyan/white: side hit");
        ImGui::TextColored(ImVec4(0.08F, 0.18F, 0.42F, 1.0F), "Blue: validated sky/empty shell");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0F, 0.0F, 1.0F, 1.0F), "Magenta: no owner");
        ImGui::TextColored(ImVec4(1.0F, 0.0F, 0.0F, 1.0F), "Red: far-bound exit");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0F, 0.92F, 0.0F, 1.0F), "Yellow: step limit");
        if (stats.rayProbeValid) {
            ImGui::SeparatorText("Center ray GPU record");
            ImGui::Text("%s | class %u | termination %u",
                        stats.rayProbeHit ? "HIT" : "MISS",
                        stats.rayProbeClass, stats.rayProbeTermination);
            ImGui::Text("Tile %u | layer %u | material %u",
                        stats.rayProbeTile, stats.rayProbeLayer,
                        stats.rayProbeMaterial);
            ImGui::Text("Travel %.7f | candidate %.7f",
                        stats.rayProbeTravel, stats.rayProbeCandidateDistance);
            ImGui::Text("Interval [%.7f, %.7f] | closest %.7f R",
                        stats.rayProbeIntervalNear, stats.rayProbeIntervalFar,
                        stats.rayProbeClosestApproach);
            ImGui::Text("Normal (%.5f, %.5f, %.5f)",
                        stats.rayProbeNormalX, stats.rayProbeNormalY,
                        stats.rayProbeNormalZ);
        } else {
            ImGui::TextDisabled("Center ray record pending...");
        }
    }
#if VOXEL_PORTAL_LAB
    constexpr const char* kCameraModes[] = {
        "Orbital inspection", "Surface player / collision",
#if VOXEL_INTRINSIC_PORTAL_LAB
        "Intrinsic manifold free-fly (6 DOF)"};
#else
        "Portal free-fly (6 DOF)"};
#endif
#else
    constexpr const char* kCameraModes[] = {"Orbital inspection", "Surface player"};
#endif
    int cameraMode = static_cast<int>(renderSettings_.cameraMode);
    if (ImGui::Combo("Camera controller", &cameraMode, kCameraModes,
                     static_cast<int>(std::size(kCameraModes)))) {
        const CameraControllerMode selected = static_cast<CameraControllerMode>(cameraMode);
        setCameraMode(selected, selected == CameraControllerMode::SurfaceTraversal);
    }
#if VOXEL_PORTAL_LAB
#if VOXEL_INTRINSIC_PORTAL_LAB
    ImGui::TextDisabled(
#if VOXEL_GLOBAL_METRIC_LAB
        "F6: orbit/surface | F7: shared-handle free-fly | Home/End: reset A/B"
#else
        "F6: orbit/surface | F7: intrinsic free-fly | Home: reset at +l"
#endif
    );
#else
    ImGui::TextDisabled("F6: orbit/surface | F7: portal free-fly | Home: reset free-fly");
#endif
#else
    ImGui::TextDisabled("F6 toggles orbital/surface controller.");
#endif
    if (ImGui::SliderFloat("Planet radius", &renderSettings_.planetRadius,
                           0.25F, 2.0F, "%.2f")) {
        surfaceCamera_.configureVoxelGeometry(
            stats.averageSurfaceCellWidth * renderSettings_.planetRadius,
            stats.averageRadialLayerHeight * renderSettings_.planetRadius,
            renderSettings_.planetRadius);
    }
    if (renderSettings_.cameraMode == CameraControllerMode::Orbit) {
        ImGui::Checkbox("Auto-orbit camera", &renderSettings_.animate);
        ImGui::SliderFloat("Time scale", &renderSettings_.timeScale, 0.0F, 2.0F, "%.2f");
        const float minimumDistance = minimumCameraDistance(
            renderSettings_.planetRadius, stats.planetOuterScale);
        renderSettings_.cameraDistance = std::max(renderSettings_.cameraDistance, minimumDistance);
        const float maximumDistance = std::max(8.0F, minimumDistance * 4.0F);
        ImGui::SliderFloat("Camera distance", &renderSettings_.cameraDistance,
                           minimumDistance, maximumDistance, "%.3f");
        ImGui::TextDisabled("Surface clearance: %.3f",
                            renderSettings_.cameraDistance -
                                renderSettings_.planetRadius * stats.planetOuterScale);
        ImGui::SliderAngle("Camera yaw", &renderSettings_.cameraYaw, -180.0F, 180.0F);
        ImGui::SliderAngle("Camera pitch", &renderSettings_.cameraPitch, -83.0F, 83.0F);
    } else if (renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal) {
        bool debugNoclip = !surfaceCamera_.collisionEnabled();
        if (ImGui::Checkbox("Debug noclip / free altitude", &debugNoclip)) {
            surfaceCamera_.setCollisionEnabled(!debugNoclip);
            if (!debugNoclip) {
                const auto terrain = [this](const SurfaceCameraController::Vector& radial,
                                            float radius, float sweep) {
                    return renderer_->querySurfaceTerrain(
                        radial, renderSettings_.planetRadius, radius, sweep);
                };
                surfaceCamera_.resetToSurface(renderSettings_.planetRadius, terrain);
            }
        }
        if (debugNoclip) {
            float clearance = std::max(
                surfaceCamera_.clearance(), minimumSurfaceClearance(renderSettings_.planetRadius));
            const float maximumClearance = std::max(renderSettings_.planetRadius * 2.0F, 0.25F);
            if (ImGui::SliderFloat("Noclip altitude", &clearance,
                                   minimumSurfaceClearance(renderSettings_.planetRadius),
                                   maximumClearance, "%.4f")) {
                surfaceCamera_.setClearance(clearance);
            }
        } else {
            const auto& capsule = surfaceCamera_.capsule();
            ImGui::Text("Capsule: radius %.6f | height %.6f",
                        capsule.radius, capsule.height);
            ImGui::Text("Eye %.6f | step %.6f", capsule.eyeHeight,
                        capsule.stepHeight);
            ImGui::Text("Grounded: %s | terrain profile: %s",
                        surfaceCamera_.grounded() ? "yes" : "no",
                        surfaceCamera_.terrainContactExact() ? "GPU exact" : "conservative pending");
        }
        float movementSpeed = surfaceCamera_.movementSpeed();
        const float minimumWalk = std::max(stats.averageSurfaceCellWidth *
            renderSettings_.planetRadius, 0.0001F);
        if (ImGui::SliderFloat("Walk speed", &movementSpeed,
                               minimumWalk, minimumWalk * 20.0F, "%.5f")) {
            surfaceCamera_.setMovementSpeed(movementSpeed);
        }
        float mouseSensitivity = surfaceCamera_.mouseSensitivity();
        if (ImGui::SliderFloat("Look sensitivity", &mouseSensitivity,
                               0.001F, 0.015F, "%.4f")) {
            surfaceCamera_.setMouseSensitivity(mouseSensitivity);
        }
        if (!surfaceMouseCaptured_ && ImGui::Button("Enter first-person control")) {
            setSurfaceMouseCapture(true);
        }
        ImGui::Text("Mouse: %s", surfaceMouseCaptured_ ? "captured" : "UI released");
        ImGui::TextDisabled(debugNoclip
            ? "WASD fly | Shift sprint | Q/E or wheel altitude"
            : "WASD walk | Shift sprint | Space jump");
        ImGui::TextDisabled("Mouse look is first-person; Escape releases the mouse.");
        ImGui::TextDisabled("Radial gravity/up and tangent heading are continuously transported.");
#if VOXEL_PORTAL_LAB
    } else {
#if VOXEL_INTRINSIC_PORTAL_LAB
        auto& intrinsicController = renderSettings_.intrinsicEllis;
        if (ImGui::Button(
#if VOXEL_GLOBAL_METRIC_LAB
                "Reset +l end (Home)"
#else
                "Reset intrinsic observer (Home)"
#endif
                )) {
            intrinsicEllisCamera_.resetGlobal(intrinsicController, 0U);
            syncIntrinsicEllisRenderSettings();
        }
#if VOXEL_GLOBAL_METRIC_LAB
        ImGui::SameLine();
        if (ImGui::Button("Reset -l end (End)")) {
            intrinsicEllisCamera_.resetGlobal(intrinsicController, 1U);
            syncIntrinsicEllisRenderSettings();
        }
#endif
        ImGui::SliderFloat("Free-fly speed", &intrinsicController.freeFlySpeed,
                           0.01F, 1.5F, "%.3f intrinsic/s");
        ImGui::SliderFloat("Free-fly sprint multiplier",
                           &intrinsicController.sprintMultiplier,
                           1.0F, 8.0F, "%.1fx");
        ImGui::SliderFloat("Free-fly look sensitivity",
                           &intrinsicController.mouseSensitivity,
                           0.0005F, 0.015F, "%.4f");
        if (!surfaceMouseCaptured_ &&
            ImGui::Button("Capture intrinsic free-fly mouse")) {
            setSurfaceMouseCapture(true);
        }
#if VOXEL_GLOBAL_METRIC_LAB
        if (intrinsicController.globalNativeEllisPath) {
            ImGui::Text("Native observer l: %.7f | end: %s",
                        intrinsicEllisCamera_.state().properDepth,
                        intrinsicEllisCamera_.state().properDepth >= 0.0F
                            ? "+" : "-");
            ImGui::TextUnformatted(
                "Visibility owner: geodesic end/impact only (no mouth sphere)");
        } else {
            const GlobalHandleObserverState& globalObserver =
                intrinsicEllisCamera_.globalState();
            ImGui::Text("LEGACY shared exterior position: %.4f, %.4f, %.4f",
                        globalObserver.position.x, globalObserver.position.y,
                        globalObserver.position.z);
            ImGui::Text("Legacy crossings: %u", globalObserver.crossings);
            ImGui::SliderFloat("Metric tail scale",
                               &intrinsicController.globalHandleTailScale,
                               intrinsicController.contentSphereRadius * 2.0F,
                               6.0F, "%.3f R");
            ImGui::SliderFloat("Metric strength",
                               &intrinsicController.globalHandleMetricStrength,
                               0.0F, 2.0F, "%.3f");
        }
#else
        ImGui::Text("Proper depth l: %.6f (%s end)",
                    intrinsicEllisCamera_.state().properDepth,
                    intrinsicEllisCamera_.state().properDepth >= 0.0F
                        ? "+" : "-");
        ImGui::Text("Movement limit/status: %s",
                    intrinsicEllisCamera_.motionStatusText(
                        intrinsicController));
#endif
        ImGui::Text("Visible linked mouths: A (+l) and B (-l) | last: %s",
                    intrinsicEllisCamera_.lastEnteredMouthText());
        ImGui::Text("Frame determinant: %.6f",
                    intrinsicEllisCamera_.handedness());
        ImGui::Text("Mouse: %s | terrain collision/gravity: deferred in lab",
                    surfaceMouseCaptured_ ? "captured" : "UI released");
        ImGui::TextDisabled("WASD + Space/Ctrl: intrinsic geodesic motion | Shift sprint");
        ImGui::TextDisabled(
#if VOXEL_GLOBAL_METRIC_LAB
            "Mouse look | Q/E roll | Home/End A/B reset | Escape releases mouse"
#else
            "Mouse look | Q/E roll | Home reset | Escape releases mouse"
#endif
        );
        ImGui::TextDisabled(
#if VOXEL_GLOBAL_METRIC_LAB
            "Camera and rays share one exterior-handle atlas; throat events only remap charts."
#else
            "The camera state never becomes a Euclidean endpoint pose."
#endif
        );
#else
        if (ImGui::Button("Reset aimed at portal A (Home)")) {
            portalFreeFlyCamera_.resetForPortal(
                renderSettings_.portal, renderSettings_.planetRadius);
            portalCameraManifold_ = {};
            portalCameraPositionInitialized_ = false;
            syncPortalFreeFlyRenderSettings();
        }
        float flySpeed = portalFreeFlyCamera_.speed();
        if (ImGui::SliderFloat("Free-fly speed", &flySpeed,
                               0.01F, 2.0F, "%.3f R/s")) {
            portalFreeFlyCamera_.setSpeed(flySpeed);
        }
        float sprintMultiplier = portalFreeFlyCamera_.sprintMultiplier();
        if (ImGui::SliderFloat("Free-fly sprint multiplier", &sprintMultiplier,
                               1.0F, 8.0F, "%.1fx")) {
            portalFreeFlyCamera_.setSprintMultiplier(sprintMultiplier);
        }
        float lookSensitivity = portalFreeFlyCamera_.mouseSensitivity();
        if (ImGui::SliderFloat("Free-fly look sensitivity", &lookSensitivity,
                               0.0005F, 0.015F, "%.4f")) {
            portalFreeFlyCamera_.setMouseSensitivity(lookSensitivity);
        }
        if (!surfaceMouseCaptured_ && ImGui::Button("Capture free-fly mouse")) {
            setSurfaceMouseCapture(true);
        }
        const PortalVector& flyPosition = portalFreeFlyCamera_.position();
        ImGui::Text("Position: %.3f, %.3f, %.3f",
                    flyPosition.x, flyPosition.y, flyPosition.z);
        ImGui::Text("Mouse: %s | collision/gravity: OFF",
                    surfaceMouseCaptured_ ? "captured" : "UI released");
        ImGui::TextDisabled("WASD move | Space/Ctrl vertical | Shift sprint");
        ImGui::TextDisabled("Mouse look | Q/E roll | Escape releases mouse");
        ImGui::TextDisabled("This pose remains inside the shared ray/camera wormhole manifold.");
#endif
#endif
    }
    ImGui::SliderFloat("Hit epsilon", &renderSettings_.hitEpsilon, 0.0001F, 0.01F, "%.4f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderScalar("Maximum steps", ImGuiDataType_U32, &renderSettings_.maxMarchSteps,
                        &kMinMarchSteps, &kMaxMarchSteps);
    ImGui::TextDisabled("Conservative sphere march -> macro occupancy -> voxel DDA");

    ImGui::Spacing();
    ImGui::TextUnformatted("CELLULAR VOXEL SIMULATION");
    ImGui::Separator();
    ImGui::Checkbox("Run material simulation", &renderSettings_.simulateMaterials);
    if (ImGui::Button("Reset material test")) {
        renderSettings_.simulateMaterials = false;
        renderSettings_.simulationSteps = 0;
        renderSettings_.resetVoxelVolume = true;
        cellularAccumulator_ = 0.0;
    }
    ImGui::SameLine();
    if (ImGui::Button("Single step")) {
        renderSettings_.simulateMaterials = false;
        renderSettings_.simulationSteps = 1;
        cellularAccumulator_ = 0.0;
    }
    ImGui::TextDisabled("Double-buffered materials: active");
    ImGui::TextDisabled("Deterministic movement claims: active");
    ImGui::TextDisabled("Radial planetary gravity: active");
    ImGui::TextDisabled("Fixed simulation rate: 60 Hz");
    ImGui::TextDisabled("Automata scheduling deferred until topology evaluation.");

    ImGui::Spacing();
    ImGui::TextUnformatted("VOXEL BRUSH");
    ImGui::Separator();
    if (renderSettings_.visualizationMode == 0U) {
        ImGui::BulletText("Left click: place sand");
        ImGui::BulletText("Right click: remove voxel");
    } else {
        ImGui::BulletText("Left click: raise prism column");
        ImGui::BulletText("Right click: lower prism column");
        ImGui::TextDisabled("Picking and edits remain entirely GPU-side.");
    }
    if (renderSettings_.cameraMode == CameraControllerMode::Orbit) {
        ImGui::BulletText("Middle drag: orbit camera");
        ImGui::BulletText("Mouse wheel: zoom");
    } else if (renderSettings_.cameraMode == CameraControllerMode::SurfaceTraversal) {
        ImGui::BulletText("WASD: traverse surface; Shift: sprint");
        ImGui::BulletText(surfaceCamera_.collisionEnabled()
            ? "Mouse: first-person look; Space: jump"
            : "Mouse: first-person look; Q/E or wheel: altitude");
        ImGui::BulletText("Escape: release mouse; F6: orbital mode");
#if VOXEL_PORTAL_LAB
    } else {
        ImGui::BulletText("WASD + Space/Ctrl: six-DOF flight; Shift: sprint");
#if VOXEL_INTRINSIC_PORTAL_LAB
        ImGui::BulletText("Mouse: intrinsic look; Q/E: roll; Home: +l reset");
#else
        ImGui::BulletText("Mouse: look; Q/E: roll; Home: portal-A reset");
#endif
        ImGui::BulletText("Escape: release mouse; F7: orbital mode");
#endif
    }

    ImGui::End();

#if VOXEL_PORTAL_LAB
    {
        ImDrawList* foreground = ImGui::GetForegroundDrawList();
        const ImVec2 display = ImGui::GetIO().DisplaySize;
        const float bannerWidth = std::min(display.x - 24.0F, 720.0F);
        const ImVec2 bannerMin((display.x - bannerWidth) * 0.5F, 10.0F);
        const ImVec2 bannerMax(bannerMin.x + bannerWidth, 50.0F);
        foreground->AddRectFilled(bannerMin, bannerMax,
                                  IM_COL32(20, 5, 35, 238), 7.0F);
        foreground->AddRect(bannerMin, bannerMax,
                            IM_COL32(165, 90, 255, 255), 7.0F, 0, 2.0F);
        foreground->AddText(ImVec2(bannerMin.x + 14.0F, bannerMin.y + 11.0F),
                            IM_COL32(210, 175, 255, 255),
#if VOXEL_INTRINSIC_PORTAL_LAB
#if VOXEL_GLOBAL_METRIC_LAB
                            "GLOBAL STATIC SPACETIME LAB - NATIVE ELLIS ATLAS (NO MOUTH SPHERE)");
#else
                            "INTRINSIC ELLIS MANIFOLD LAB - NATIVE l, n, TETRAD");
#endif
#else
                            "SPHERICAL WORMHOLE PORTAL LAB - CURVED WORLD-SPACE RAYS");
#endif
    }
#endif

#if VOXEL_FRACTAL_VOXEL_SDF_LAB
    ImDrawList* foreground = ImGui::GetForegroundDrawList();
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float bannerWidth = std::min(display.x - 24.0F, 820.0F);
    const ImVec2 bannerMin((display.x - bannerWidth) * 0.5F, 10.0F);
    const ImVec2 bannerMax(bannerMin.x + bannerWidth, 55.0F);
    foreground->AddRectFilled(bannerMin, bannerMax,
                              IM_COL32(4, 42, 55, 244), 7.0F);
    foreground->AddRect(bannerMin, bannerMax,
                        IM_COL32(72, 220, 255, 255), 7.0F, 0, 2.0F);
    foreground->AddText(ImVec2(bannerMin.x + 14.0F, bannerMin.y + 7.0F),
                        IM_COL32(105, 235, 255, 255),
                        "ADAPTIVE VOXELIZED FRACTAL SDF LAB - DISCRETE LEAF SURFACE");
    char fractalStatus[192]{};
    std::snprintf(fractalStatus, sizeof(fractalStatus),
                  "VOXEL SDF %s | %s | L0..L%u | %.2f px target | output %u",
                  renderSettings_.fractalPlanetSdfEnabled ? "ACTIVE" : "DISABLED",
                  renderSettings_.fractalVoxelDiscreteSurface
                      ? "DISCRETE CELLS" : "SMOOTH A/B",
                  renderSettings_.fractalPlanetSdfMaximumLevels,
                  renderSettings_.fractalPlanetSdfTargetPixels,
                  renderSettings_.fractalPlanetSdfDebugMode);
    foreground->AddText(ImVec2(bannerMin.x + 14.0F, bannerMin.y + 26.0F),
                        IM_COL32(255, 255, 255, 245), fractalStatus);
#elif VOXEL_FRACTAL_PLANET_SDF_LAB
    ImDrawList* foreground = ImGui::GetForegroundDrawList();
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float bannerWidth = std::min(display.x - 24.0F, 760.0F);
    const ImVec2 bannerMin((display.x - bannerWidth) * 0.5F, 10.0F);
    const ImVec2 bannerMax(bannerMin.x + bannerWidth, 55.0F);
    foreground->AddRectFilled(bannerMin, bannerMax,
                              IM_COL32(5, 48, 25, 242), 7.0F);
    foreground->AddRect(bannerMin, bannerMax,
                        IM_COL32(70, 255, 130, 255), 7.0F, 0, 2.0F);
    foreground->AddText(ImVec2(bannerMin.x + 14.0F, bannerMin.y + 7.0F),
                        IM_COL32(102, 255, 150, 255),
                        "TRUE FRACTAL PLANET SDF - HIT/SHAPE FROM MULTISCALE FIELD");
    char fractalStatus[192]{};
    std::snprintf(fractalStatus, sizeof(fractalStatus),
                  "SDF %s | virtual L0..L%u | %.2f px leaves | 8-step root | output mode %u",
                  renderSettings_.fractalPlanetSdfEnabled ? "ACTIVE" : "DISABLED",
                  renderSettings_.fractalPlanetSdfMaximumLevels,
                  renderSettings_.fractalPlanetSdfTargetPixels,
                  renderSettings_.fractalPlanetSdfDebugMode);
    foreground->AddText(ImVec2(bannerMin.x + 14.0F, bannerMin.y + 26.0F),
                        IM_COL32(255, 255, 255, 245), fractalStatus);
#elif VOXEL_ADAPTIVE_SDF_LAB
    ImDrawList* foreground = ImGui::GetForegroundDrawList();
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float bannerWidth = std::min(display.x - 24.0F, 720.0F);
    const ImVec2 bannerMin((display.x - bannerWidth) * 0.5F, 10.0F);
    const ImVec2 bannerMax(bannerMin.x + bannerWidth, 55.0F);
    foreground->AddRectFilled(bannerMin, bannerMax,
                              IM_COL32(74, 20, 8, 238), 7.0F);
    foreground->AddRect(bannerMin, bannerMax,
                        IM_COL32(255, 184, 35, 255), 7.0F, 0, 2.0F);
    foreground->AddText(ImVec2(bannerMin.x + 14.0F, bannerMin.y + 7.0F),
                        IM_COL32(255, 207, 86, 255),
                        "ADAPTIVE FRACTAL SDF LAB - NOT PRODUCTION");
    char adaptiveStatus[192]{};
    std::snprintf(adaptiveStatus, sizeof(adaptiveStatus),
                  "SDF %s | dynamic L0..L%u | %.2f px target | 7 root steps | output: %s",
                  renderSettings_.adaptivePlanetSdfEnabled ? "ACTIVE" : "DISABLED",
                  renderSettings_.adaptivePlanetSdfMaximumLevels,
                  renderSettings_.adaptivePlanetSdfTargetPixels,
                  renderSettings_.adaptivePlanetSdfDebugMode == 0U
                      ? (renderSettings_.adaptivePlanetSdfComparisonView
                             ? "A/B shaded relief"
                             : "shaded relief")
                      : renderSettings_.adaptivePlanetSdfDebugMode == 1U
                            ? "displacement field"
                            : renderSettings_.adaptivePlanetSdfDebugMode == 2U
                                  ? "active level"
                                  : "support mask");
    foreground->AddText(ImVec2(bannerMin.x + 14.0F, bannerMin.y + 26.0F),
                        IM_COL32(255, 255, 255, 245), adaptiveStatus);
    if (renderSettings_.adaptivePlanetSdfComparisonView &&
        renderSettings_.adaptivePlanetSdfDebugMode == 0U) {
        const float splitX = display.x * 0.5F;
        foreground->AddLine(ImVec2(splitX, 58.0F),
                            ImVec2(splitX, display.y),
                            IM_COL32(255, 205, 60, 190), 2.0F);
        foreground->AddText(ImVec2(splitX - 175.0F, 62.0F),
                            IM_COL32(255, 255, 255, 235),
                            "LEFT: EXACT PARENT CAPS");
        foreground->AddText(ImVec2(splitX + 14.0F, 62.0F),
                            IM_COL32(255, 224, 102, 255),
                            "RIGHT: ADAPTIVE FRACTAL SDF");
    }
#endif

    if (!ImGui::GetIO().WantCaptureMouse) {
        const ImVec2 cursor = surfaceMouseCaptured_
                                  ? ImVec2(ImGui::GetIO().DisplaySize.x * 0.5F,
                                           ImGui::GetIO().DisplaySize.y * 0.5F)
                                  : ImGui::GetMousePos();
        ImGui::GetForegroundDrawList()->AddCircle(cursor, 7.0F, IM_COL32(70, 235, 205, 220), 16, 1.5F);
        ImGui::GetForegroundDrawList()->AddLine(
            ImVec2(cursor.x - 10.0F, cursor.y), ImVec2(cursor.x + 10.0F, cursor.y),
            IM_COL32(70, 235, 205, 180), 1.0F);
        ImGui::GetForegroundDrawList()->AddLine(
            ImVec2(cursor.x, cursor.y - 10.0F), ImVec2(cursor.x, cursor.y + 10.0F),
            IM_COL32(70, 235, 205, 180), 1.0F);
    }
}

} // namespace voxel
