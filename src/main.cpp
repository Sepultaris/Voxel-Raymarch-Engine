#include "app/application.hpp"
#include "planet/geodesic_topology_cache.hpp"

#include <SDL.h>

#include <cmath>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

int main(int argumentCount, char** arguments) {
    bool headlessRequested = false;
    for (int index = 1; index < argumentCount; ++index) {
        headlessRequested = headlessRequested ||
            std::string_view(arguments[index]) == "--headless-capture";
    }
    try {
        const auto hasArgument = [&](std::string_view expected) {
            for (int index = 1; index < argumentCount; ++index) {
                if (std::string_view(arguments[index]) == expected) {
                    return true;
                }
            }
            return false;
        };
        const auto argumentValue = [&](std::string_view option)
            -> std::optional<std::string_view> {
            for (int index = 1; index + 1 < argumentCount; ++index) {
                if (std::string_view(arguments[index]) == option) {
                    return std::string_view(arguments[index + 1]);
                }
            }
            return std::nullopt;
        };
        const auto captureOutput = argumentValue("--headless-capture");
        if (headlessRequested && !captureOutput.has_value()) {
            std::cerr << "--headless-capture requires an output .png path.\n";
            return 2;
        }
        std::uint32_t captureFrames = 8U;
        if (const auto value = argumentValue("--capture-frames")) {
            try {
                const unsigned long parsed = std::stoul(std::string(*value));
                if (parsed == 0UL || parsed > 600UL) {
                    throw std::out_of_range("capture frame count");
                }
                captureFrames = static_cast<std::uint32_t>(parsed);
            } catch (...) {
                std::cerr << "--capture-frames must be in [1, 600].\n";
                return 2;
            }
        }
#if VOXEL_GLOBAL_METRIC_LAB
        std::uint32_t capturePreset = 0U;
        if (const auto value = argumentValue("--capture-preset")) {
            if (*value == "balanced") {
                capturePreset = 0U;
            } else if (*value == "no-handle") {
                capturePreset = 1U;
            } else if (*value == "rejected-broad") {
                capturePreset = 2U;
            } else if (*value == "rejected-hard-aperture") {
                capturePreset = 3U;
            } else {
                std::cerr << "Unknown --capture-preset. Use balanced, no-handle, "
                             "rejected-broad, or rejected-hard-aperture.\n";
                return 2;
            }
        }
        std::optional<float> captureTailScale;
        if (const auto value = argumentValue("--capture-tail-scale")) {
            try {
                const float parsed = std::stof(std::string(*value));
                if (!(parsed > 0.0F) || !std::isfinite(parsed)) {
                    throw std::out_of_range("capture tail scale");
                }
                captureTailScale = parsed;
            } catch (...) {
                std::cerr << "--capture-tail-scale must be a positive finite value.\n";
                return 2;
            }
        }
        std::uint32_t captureCamera = 0U;
        if (const auto value = argumentValue("--capture-camera")) {
            if (*value == "reset") captureCamera = 0U;
            else if (*value == "close-a") captureCamera = 1U;
            else if (*value == "close-b") captureCamera = 2U;
            else if (*value == "oblique-a") captureCamera = 3U;
            else {
                std::cerr << "Unknown --capture-camera. Use reset, close-a, "
                             "close-b, or oblique-a.\n";
                return 2;
            }
        }
        std::optional<std::uint32_t> captureDebug;
        if (const auto value = argumentValue("--capture-debug")) {
            if (*value == "normal") captureDebug = 0U;
            else if (*value == "bending") captureDebug = 6U;
            else if (*value == "aa-samples") captureDebug = 7U;
            else if (*value == "jacobian") captureDebug = 8U;
            else {
                std::cerr << "Unknown --capture-debug. Use normal, bending, "
                             "aa-samples, or jacobian.\n";
                return 2;
            }
        }
#endif
#if VOXEL_EIGHT_PLANET_SYSTEM_LAB
        std::uint32_t systemCapturePreset = 0U;
        if (const auto value = argumentValue("--system-capture")) {
            if (*value == "overview") systemCapturePreset = 0U;
            else if (*value == "near") systemCapturePreset = 1U;
            else if (*value == "mixed-lod") systemCapturePreset = 2U;
            else if (*value == "traversal-8-1") systemCapturePreset = 3U;
            else if (*value == "cycle-audit") systemCapturePreset = 4U;
            else if (*value == "edit-persistence") systemCapturePreset = 5U;
            else if (*value == "differential") systemCapturePreset = 6U;
            else {
                std::cerr << "Unknown --system-capture. Use overview, near, "
                             "mixed-lod, traversal-8-1, cycle-audit, "
                             "edit-persistence, or differential.\n";
                return 2;
            }
        }
#endif
#if VOXEL_SCALE_LAB
        const std::string_view command = argumentCount > 1
            ? std::string_view(arguments[1]) : std::string_view{};
        const bool scaleLabOrbit = command == "--scale-lab-orbit";
        const bool scaleLabRotation = command == "--scale-lab-rotation";
        const bool scaleLabSurface = command == "--scale-lab-surface";
        const bool scaleLabArtifact = command == "--scale-lab-artifact";
        const bool scaleLabInteractive = command == "--scale-lab-interactive";
        const bool clearScaleLabCache = command == "--scale-lab-clear-cache";
        if (!scaleLabOrbit && !scaleLabRotation && !scaleLabSurface &&
            !scaleLabArtifact && !scaleLabInteractive && !clearScaleLabCache) {
            std::cerr << "This isolated executable requires one explicit scale-lab command: "
                         "--scale-lab-orbit, --scale-lab-rotation, --scale-lab-surface, "
                         "--scale-lab-artifact, --scale-lab-interactive, or "
                         "--scale-lab-clear-cache.\n";
            return 2;
        }
        if (clearScaleLabCache) {
            std::string error;
            const auto path = voxel::scaleLabTopologyCachePath();
            const bool removed = voxel::clearExactGeodesicTopologyCache(path, error);
            if (!error.empty()) {
                std::cerr << "Unable to clear exact scale-lab cache " << path
                          << ": " << error << '\n';
                return 3;
            }
            std::cout << (removed ? "Cleared " : "No cache existed at ") << path << '\n';
            return 0;
        }
#else
        constexpr bool scaleLabOrbit = false;
        constexpr bool scaleLabRotation = false;
        constexpr bool scaleLabSurface = false;
        constexpr bool scaleLabArtifact = false;
#endif
#if VOXEL_ADAPTIVE_SDF_LAB
        const std::string_view adaptiveCommand = argumentCount > 1
            ? std::string_view(arguments[1]) : std::string_view{};
        const bool adaptiveSdfOrbit = adaptiveCommand == "--adaptive-sdf-orbit";
        const bool adaptiveSdfRotation = adaptiveCommand == "--adaptive-sdf-rotation";
        const bool adaptiveSdfSurface = adaptiveCommand == "--adaptive-sdf-surface";
        const bool adaptiveSdfArtifact = adaptiveCommand == "--adaptive-sdf-artifact";
        const bool adaptiveSdfInteractive = adaptiveCommand == "--adaptive-sdf-interactive";
        if (!adaptiveSdfOrbit && !adaptiveSdfRotation && !adaptiveSdfSurface &&
            !adaptiveSdfArtifact && !adaptiveSdfInteractive) {
            std::cerr << "This isolated executable requires one explicit adaptive-SDF command: "
                         "--adaptive-sdf-orbit, --adaptive-sdf-rotation, "
                         "--adaptive-sdf-surface, --adaptive-sdf-artifact, or "
                         "--adaptive-sdf-interactive.\n";
            return 2;
        }
#else
        constexpr bool adaptiveSdfOrbit = false;
        constexpr bool adaptiveSdfRotation = false;
        constexpr bool adaptiveSdfSurface = false;
        constexpr bool adaptiveSdfArtifact = false;
#endif
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
        const std::string_view fractalCommand = argumentCount > 1
            ? std::string_view(arguments[1]) : std::string_view{};
        const bool fractalSdfOrbit = fractalCommand == "--fractal-voxel-orbit";
        const bool fractalSdfRotation = fractalCommand == "--fractal-voxel-rotation";
        const bool fractalSdfSurface = fractalCommand == "--fractal-voxel-surface";
        const bool fractalSdfArtifact = fractalCommand == "--fractal-voxel-artifact";
        const bool fractalSdfInteractive = fractalCommand.empty() ||
            fractalCommand == "--fractal-voxel-interactive";
        if (!fractalSdfOrbit && !fractalSdfRotation && !fractalSdfSurface &&
            !fractalSdfArtifact && !fractalSdfInteractive) {
            std::cerr << "This isolated executable requires a fractal-voxel command: "
                         "--fractal-voxel-orbit, --fractal-voxel-rotation, "
                         "--fractal-voxel-surface, --fractal-voxel-artifact, or "
                         "--fractal-voxel-interactive.\n";
            return 2;
        }
#elif VOXEL_FRACTAL_PLANET_SDF_LAB
        const std::string_view fractalCommand = argumentCount > 1
            ? std::string_view(arguments[1]) : std::string_view{};
        const bool fractalSdfOrbit = fractalCommand == "--fractal-planet-orbit";
        const bool fractalSdfRotation = fractalCommand == "--fractal-planet-rotation";
        const bool fractalSdfSurface = fractalCommand == "--fractal-planet-surface";
        const bool fractalSdfArtifact = fractalCommand == "--fractal-planet-artifact";
        // Explorer/double-click execution supplies no command line. The lab is
        // an interactive application first, so that ordinary launch must stay
        // open instead of returning the old command-validation error code 2.
        const bool fractalSdfInteractive = fractalCommand.empty() ||
            fractalCommand == "--fractal-planet-interactive";
        if (!fractalSdfOrbit && !fractalSdfRotation && !fractalSdfSurface &&
            !fractalSdfArtifact && !fractalSdfInteractive) {
            std::cerr << "This isolated executable requires one explicit fractal-planet command: "
                         "--fractal-planet-orbit, --fractal-planet-rotation, "
                         "--fractal-planet-surface, --fractal-planet-artifact, or "
                         "--fractal-planet-interactive.\n";
            return 2;
        }
#else
        constexpr bool fractalSdfOrbit = false;
        constexpr bool fractalSdfRotation = false;
        constexpr bool fractalSdfSurface = false;
        constexpr bool fractalSdfArtifact = false;
#endif
        const bool smokeTest = argumentCount > 1 &&
                               std::string_view(arguments[1]) == "--smoke-test";
        const bool benchmark = argumentCount > 1 &&
                               std::string_view(arguments[1]) == "--benchmark";
        const bool bvhBenchmark = argumentCount > 1 &&
                                  std::string_view(arguments[1]) == "--benchmark-bvh";
        const bool wideBenchmark = argumentCount > 1 &&
                                   std::string_view(arguments[1]) == "--benchmark-wide";
        const bool ddaBenchmark = argumentCount > 1 &&
                                  std::string_view(arguments[1]) == "--benchmark-dda";
        const bool streamingTest = argumentCount > 1 &&
                                   std::string_view(arguments[1]) == "--stream-test";
        const bool rotationBenchmark = argumentCount > 1 &&
                                       std::string_view(arguments[1]) == "--rotation-benchmark";
        const bool surfaceBenchmark = argumentCount > 1 &&
                                      std::string_view(arguments[1]) == "--surface-benchmark";
        const bool artifactRegression = argumentCount > 1 &&
                                        std::string_view(arguments[1]) == "--artifact-regression";
        const bool rayStatusCapture = argumentCount > 1 &&
                                      std::string_view(arguments[1]) == "--ray-status-capture";
        const bool rayStatusInteractive = hasArgument("--ray-status");
        const bool rayFailureOverlay = argumentCount > 1 &&
                                       std::string_view(arguments[1]) == "--ray-failure-overlay";
        const bool rayFailureBenchmark = argumentCount > 1 &&
                                         std::string_view(arguments[1]) == "--ray-failure-benchmark";
        const bool rayFailureSurfaceBenchmark = argumentCount > 1 &&
            std::string_view(arguments[1]) == "--ray-failure-surface-benchmark";
#if VOXEL_INTRINSIC_PORTAL_LAB
        const bool intrinsicTraversal =
            hasArgument("--intrinsic-traversal-regression");
#else
        constexpr bool intrinsicTraversal = false;
#endif
#if VOXEL_GLOBAL_METRIC_LAB
        const bool globalVisualRegression = argumentCount > 1 &&
            std::string_view(arguments[1]) == "--global-visual-regression";
        const bool globalVisualRegressionNoMedia = argumentCount > 1 &&
            std::string_view(arguments[1]) ==
                "--global-visual-regression-no-media";
#else
        constexpr bool globalVisualRegression = false;
        constexpr bool globalVisualRegressionNoMedia = false;
#endif
#if !VOXEL_ENABLE_REFERENCE_TRAVERSAL
        if (bvhBenchmark || wideBenchmark) {
            std::cerr << "Reference traversal is not present in this production build. "
                         "Configure with -DVOXEL_ENABLE_REFERENCE_TRAVERSAL=ON.\n";
            return 2;
        }
#endif
        voxel::Application application(
            hasArgument("--force-regenerate-topology"), headlessRequested);
#if VOXEL_EIGHT_PLANET_SYSTEM_LAB
        application.setEightPlanetCapturePreset(systemCapturePreset);
#endif
        if (hasArgument("--no-micro-sdf")) {
            application.setTerrainMicroSdfEnabled(false);
        }
#if VOXEL_GLOBAL_METRIC_LAB
        if (headlessRequested) {
            application.setGlobalLensCapturePreset(capturePreset);
            if (captureTailScale.has_value()) {
                application.setGlobalCaptureTailScale(*captureTailScale);
            }
            application.setGlobalCaptureCameraPreset(captureCamera);
            // no-handle is itself the reference visualization; a generic
            // --capture-debug normal must not silently turn the handle back on.
            if (captureDebug.has_value() && capturePreset != 1U) {
                application.setGlobalCaptureDebugMode(*captureDebug);
            }
        }
        if (hasArgument("--no-global-spatial-aa")) {
            application.setGlobalSpatialAaEnabled(false);
        }
        if (hasArgument("--global-no-handle-reference")) {
            application.setGlobalNoHandleReference(true);
        }
#endif
#if VOXEL_INTRINSIC_PORTAL_LAB
        if (intrinsicTraversal) {
            application.beginIntrinsicEllisTraversalRegression();
        }
#endif
#if VOXEL_GLOBAL_METRIC_LAB
        if (globalVisualRegression || globalVisualRegressionNoMedia) {
            application.beginGlobalMetricVisualRegression(
                globalVisualRegression);
        }
#endif
#if VOXEL_ADAPTIVE_SDF_LAB
        if (adaptiveSdfInteractive) {
            // A full-globe view intentionally merges subpixel SDF octaves and
            // therefore resembles the exact parent voxels. Start the visual
            // lab at player scale where live subdivision is observable.
            application.beginAdaptiveSdfInspection();
        }
#endif
#if VOXEL_FRACTAL_PLANET_SDF_LAB
        if (fractalSdfInteractive) {
            application.beginAdaptiveSdfInspection();
        }
#endif
        const bool adaptiveSdfBoundedRun = adaptiveSdfOrbit || adaptiveSdfRotation ||
                                           adaptiveSdfSurface;
        const bool fractalSdfBoundedRun = fractalSdfOrbit || fractalSdfRotation ||
                                          fractalSdfSurface;
        const bool artifactRun = artifactRegression || adaptiveSdfArtifact ||
                                 fractalSdfArtifact;
        const int runResult = application.run(headlessRequested ? captureFrames
                                         : smokeTest ? 3U
                                         : benchmark || bvhBenchmark || wideBenchmark ||
                                                   ddaBenchmark || rotationBenchmark ||
                                                   surfaceBenchmark || rayFailureBenchmark ||
                                                   rayFailureSurfaceBenchmark || scaleLabOrbit ||
                                                   scaleLabRotation || scaleLabSurface ||
                                                   adaptiveSdfBoundedRun ||
                                                   fractalSdfBoundedRun ? 120U
                                         : intrinsicTraversal ? 160U
                                         : globalVisualRegression ||
                                                   globalVisualRegressionNoMedia ? 180U
                                         : artifactRun ? 720U
                                         : scaleLabArtifact ? 720U
                                         : rayStatusCapture ? 240U
                                         : streamingTest ? 20U : 0U,
                               streamingTest || rotationBenchmark || surfaceBenchmark ||
                                   scaleLabRotation || scaleLabSurface ||
                                   adaptiveSdfRotation || adaptiveSdfSurface ||
                                   fractalSdfRotation || fractalSdfSurface,
                               streamingTest || rotationBenchmark || scaleLabRotation ||
                                   adaptiveSdfRotation || fractalSdfRotation,
                               rayFailureOverlay || rayFailureBenchmark ||
                                       rayFailureSurfaceBenchmark ? 7U
                                                  : rayStatusCapture || rayStatusInteractive ? 6U
                                                  : scaleLabArtifact ? 8U
                                                  : globalVisualRegression ||
                                                        globalVisualRegressionNoMedia ? 5U
                                                  : artifactRun ? 5U
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
                                                  : fractalSdfBoundedRun ? 5U
#endif
#if VOXEL_GLOBAL_METRIC_LAB
                                                  : intrinsicTraversal ? 5U
#endif
                                                  : bvhBenchmark ? 0U : wideBenchmark ? 1U : 2U,
                               smokeTest || surfaceBenchmark || rayFailureSurfaceBenchmark ||
                                   scaleLabSurface || adaptiveSdfSurface ||
                                   fractalSdfSurface,
                               artifactRun || rayStatusCapture || scaleLabArtifact);
        if (runResult != 0 || !headlessRequested) {
            return runResult;
        }
        if (!application.headlessWindowStayedHidden()) {
            std::cerr << "Headless capture aborted: SDL window became visible.\n";
            return 16;
        }
        std::string captureError;
        const std::filesystem::path outputPath{
            std::string(*captureOutput)};
        std::cout << "Headless capture: reading deterministic GPU frame...\n";
        if (!application.captureFrame(outputPath, captureError)) {
            std::cerr << "Headless capture failed: " << captureError << '\n';
            return 17;
        }
        std::cout << "Headless capture complete: " << outputPath.string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal error: " << error.what() << '\n';
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
        SDL_ShowSimpleMessageBox(
            SDL_MESSAGEBOX_ERROR,
            "ADAPTIVE VOXELIZED FRACTAL SDF Lab - Startup Error",
            error.what(), nullptr);
#elif VOXEL_FRACTAL_PLANET_SDF_LAB
        // A console created by Explorer closes with the process. Keep startup
        // failures visible to a double-click user even when stderr disappears.
        SDL_ShowSimpleMessageBox(
            SDL_MESSAGEBOX_ERROR,
            "TRUE FRACTAL PLANET SDF Lab - Startup Error",
            error.what(), nullptr);
#elif VOXEL_PORTAL_LAB
        if (!headlessRequested) {
        SDL_ShowSimpleMessageBox(
            SDL_MESSAGEBOX_ERROR,
#if VOXEL_GLOBAL_METRIC_LAB
            "GLOBAL STATIC SPACETIME Lab - Startup Error",
#elif VOXEL_INTRINSIC_PORTAL_LAB
            "INTRINSIC ELLIS MANIFOLD Lab - Startup Error",
#else
            "SPHERICAL WORMHOLE PORTAL Lab - Startup Error",
#endif
            error.what(), nullptr);
        }
#endif
        return 1;
    }
}
