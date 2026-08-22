#include "app/application.hpp"
#include "planet/geodesic_topology_cache.hpp"

#include <SDL.h>

#include <exception>
#include <iostream>
#include <string_view>

int main(int argumentCount, char** arguments) {
    try {
        const auto hasArgument = [&](std::string_view expected) {
            for (int index = 1; index < argumentCount; ++index) {
                if (std::string_view(arguments[index]) == expected) {
                    return true;
                }
            }
            return false;
        };
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
        const bool rayStatusInteractive = argumentCount > 1 &&
                                          std::string_view(arguments[1]) == "--ray-status";
        const bool rayFailureOverlay = argumentCount > 1 &&
                                       std::string_view(arguments[1]) == "--ray-failure-overlay";
        const bool rayFailureBenchmark = argumentCount > 1 &&
                                         std::string_view(arguments[1]) == "--ray-failure-benchmark";
        const bool rayFailureSurfaceBenchmark = argumentCount > 1 &&
            std::string_view(arguments[1]) == "--ray-failure-surface-benchmark";
#if VOXEL_INTRINSIC_PORTAL_LAB
        const bool intrinsicTraversal = argumentCount > 1 &&
            std::string_view(arguments[1]) == "--intrinsic-traversal-regression";
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
        voxel::Application application(hasArgument("--force-regenerate-topology"));
        if (hasArgument("--no-micro-sdf")) {
            application.setTerrainMicroSdfEnabled(false);
        }
#if VOXEL_GLOBAL_METRIC_LAB
        if (hasArgument("--no-global-spatial-aa")) {
            application.setGlobalSpatialAaEnabled(false);
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
        return application.run(smokeTest ? 3U
                                         : benchmark || bvhBenchmark || wideBenchmark ||
                                                   ddaBenchmark || rotationBenchmark ||
                                                   surfaceBenchmark || rayFailureBenchmark ||
                                                   rayFailureSurfaceBenchmark || scaleLabOrbit ||
                                                   scaleLabRotation || scaleLabSurface ||
                                                   adaptiveSdfBoundedRun ||
                                                   fractalSdfBoundedRun ? 120U
                                         : intrinsicTraversal ? 80U
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
#endif
        return 1;
    }
}
