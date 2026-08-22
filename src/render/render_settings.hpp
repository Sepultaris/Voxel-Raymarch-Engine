#pragma once

#include "render/planet_lighting.hpp"
#include "render/planet_atmosphere.hpp"
#include "render/intrinsic_ellis_manifold.hpp"
#include "render/portal_lab.hpp"
#include "render/space_environment.hpp"
#include "render/volumetric_clouds.hpp"

#include <algorithm>
#include <array>
#include <cstdint>

namespace voxel {

#ifndef VOXEL_DEFAULT_PLANET_RADIUS
#define VOXEL_DEFAULT_PLANET_RADIUS 1.0F
#endif

[[nodiscard]] inline float minimumCameraDistance(float planetRadius,
                                                 float planetOuterScale) noexcept {
    const float clearance = std::max(planetRadius * 0.01F, 0.005F);
    return planetRadius * planetOuterScale + clearance;
}

[[nodiscard]] inline float minimumSurfaceClearance(float planetRadius) noexcept {
    return std::max(planetRadius * 0.01F, 0.005F);
}

enum class CameraControllerMode : std::uint32_t {
    Orbit = 0U,
    SurfaceTraversal = 1U,
    PortalFreeFly = 2U,
};

[[nodiscard]] inline float cameraZoomStep(float cameraDistance) noexcept {
    return std::max(cameraDistance * 0.08F, 0.025F);
}

struct TerrainGenerationSettings {
    std::uint32_t seed{1337U};
    float continentScale{1.35F};
    float continentStrength{0.95F};
    float mountainScale{5.5F};
    float mountainStrength{0.52F};
    float roughness{0.60F};
    float oceanLevel{0.46F};
    float polarStrength{0.20F};
};

struct RenderSettings {
    float planetRadius{VOXEL_DEFAULT_PLANET_RADIUS};
    float hitEpsilon{0.001F};
    float cameraDistance{3.0F};
    float cameraYaw{};
    float cameraPitch{0.16F};
    CameraControllerMode cameraMode{CameraControllerMode::Orbit};
    float surfaceRadialX{};
    float surfaceRadialY{};
    float surfaceRadialZ{1.0F};
    float surfaceForwardX{};
    float surfaceForwardY{1.0F};
    float surfaceForwardZ{};
    float surfaceClearance{0.025F};
    float surfaceCameraRadius{};
    float surfaceLookPitch{};
    float timeScale{0.2F};
    std::uint32_t maxMarchSteps{160};
    bool localGeodesicAoEnabled{true};
    float localGeodesicAoStrength{0.12F};
    bool localGeodesicAoDebug{};
    bool ddaWorkHeatmapDebug{};
    PlanetLightingSettings lighting{};
    PlanetAtmosphereSettings atmosphere{};
    VolumetricCloudSettings clouds{};
    SpaceEnvironmentSettings spaceEnvironment{};
    PortalLabSettings portal{};
    IntrinsicEllisSettings intrinsicEllis{};
    bool terrainMicroSdfEnabled{true};
    float terrainMicroSdfStrength{0.075F};
    bool terrainMicroSdfDebug{};
    bool adaptivePlanetSdfEnabled{true};
    std::uint32_t adaptivePlanetSdfMaximumLevels{9U};
    float adaptivePlanetSdfTargetPixels{1.25F};
    float adaptivePlanetSdfLayerFraction{0.15F};
    std::uint32_t adaptivePlanetSdfDebugMode{};
    bool adaptivePlanetSdfComparisonView{true};
    bool fractalPlanetSdfEnabled{true};
    std::uint32_t fractalPlanetSdfMaximumLevels{10U};
    float fractalPlanetSdfTargetPixels{1.25F};
    float fractalPlanetSdfMicroRelief{0.012F};
    std::uint32_t fractalPlanetSdfDebugMode{};
    float fractalVoxelSunAzimuth{-0.65F};
    float fractalVoxelSunElevation{0.65F};
    float fractalVoxelSunStrength{1.10F};
    float fractalVoxelAmbientStrength{0.30F};
    float fractalVoxelCavityStrength{0.22F};
    float fractalVoxelCellEdgeStrength{0.10F};
    float fractalVoxelNormalSharpness{0.45F};
    bool fractalVoxelDiscreteSurface{true};
    bool fractalVoxelCellEdgesEnabled{true};
    bool fractalVoxelCavityDebug{};
    float brushNdcX{};
    float brushNdcY{};
    std::uint32_t editMode{};
    std::uint32_t brushMaterial{2};
    std::uint32_t simulationSteps{};
    std::uint32_t visualizationMode{1};
    // 0: reference BVH, 1: wide masks, 2: DDA, 3-6: debug/capture modes.
    std::uint32_t geodesicTraversalMode{2};
    TerrainGenerationSettings terrain{};
    bool resetVoxelVolume{};
    bool regenerateTerrain{};
    bool animate{false};
    bool simulateMaterials{false};
};

struct RendererStats {
    const char* deviceName{"Not initialized"};
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint32_t swapchainImages{};
    std::uint32_t voxelResolution{};
    std::uint32_t voxelCount{};
    std::uint32_t macrocellCount{};
    std::uint32_t simulationTick{};
    std::uint32_t geodesicFrequency{};
    std::uint32_t surfaceTileCount{};
    std::uint32_t pentagonCount{};
    std::uint32_t hexagonCount{};
    std::uint32_t residentTileBricks{};
    std::uint32_t tileBrickResolution{};
    float averageSurfaceCellWidth{};
    float averageRadialLayerHeight{};
    float radialHeightWidthRatio{};
    std::uint64_t tileBrickCellCapacity{};
    std::uint64_t residentBrickCellCapacity{};
    std::uint32_t geodesicBvhNodeCount{};
    std::uint32_t geodesicWideNodeCount{};
    std::uint32_t geodesicTraversalMode{};
    std::uint32_t streamRequestSamples{};
    std::uint32_t pageRequests{};
    std::uint32_t pageRequestOverflow{};
    std::uint32_t pagesStreamed{};
    std::uint32_t stalePageRequests{};
    std::uint64_t totalPageRequests{};
    std::uint64_t totalPagesStreamed{};
    std::uint64_t totalPageRequestOverflow{};
    std::uint64_t totalStalePageRequests{};
    std::uint32_t terrainGeneration{};
    std::uint32_t terrainSeed{1337U};
    std::uint64_t artifactDiagnosticRecords{};
    std::uint64_t artifactDiagnosticOverflow{};
    std::uint64_t artifactCapHits{};
    std::uint64_t artifactNonExposedSides{};
    std::uint64_t artifactFilteredSides{};
    std::uint64_t artifactPartialSides{};
    std::uint64_t artifactResolvedSides{};
    std::uint64_t artifactInvalidNeighbors{};
    std::uint64_t artifactClosedShellMisses{};
    std::uint64_t artifactRecoverableHorizonMisses{};
    std::uint64_t artifactAllNoOwnerMisses{};
    std::uint64_t artifactAllFarMisses{};
    std::array<std::uint64_t, 4> artifactRecoverableClosestApproachBins{};
    std::array<std::uint64_t, 4> artifactNoOwnerClosestApproachBins{};
    std::uint64_t artifactNearHorizonStarPixels{};
    std::uint32_t artifactFirstHorizonPixelX{};
    std::uint32_t artifactFirstHorizonPixelY{};
    std::uint32_t artifactFirstHorizonTermination{};
    std::uint32_t artifactFirstHorizonTile{0xffffffffU};
    float artifactFirstHorizonDistance{};
    float artifactFirstHorizonClosestApproach{};
    float artifactFirstHorizonCapDenominator{};
    float artifactFirstHorizonCapDistance{};
    float artifactFirstHorizonTravel{};
    float artifactFirstHorizonFar{};
    std::uint32_t artifactMaximumIsolatedSidePixels{};
    std::uint32_t artifactMaximumDottedChainPixels{};
    std::uint32_t artifactFirstChainPixelX{};
    std::uint32_t artifactFirstChainPixelY{};
    std::uint32_t artifactFirstChainClass{};
    std::uint32_t artifactFirstChainTile{0xffffffffU};
    float artifactFirstChainDistance{};
    float artifactFirstChainProjectedWidth{};
    float artifactFirstChainProjectedHeight{};
    float artifactFirstChainLocalInterior{};
    std::uint32_t artifactFirstPixelX{};
    std::uint32_t artifactFirstPixelY{};
    std::uint32_t artifactFirstClass{};
    std::uint32_t artifactFirstTile{0xffffffffU};
    std::uint32_t artifactFirstLayer{};
    std::uint32_t artifactFirstNeighbor{0xffffffffU};
    std::uint32_t artifactFirstColumnHeight{};
    std::uint32_t artifactFirstNeighborHeight{};
    std::uint32_t artifactFirstMaterial{};
    std::uint32_t artifactFirstTermination{};
    float artifactFirstDistance{};
    float artifactFirstGeometryDetail{};
    float artifactFirstSideIncidence{};
    float artifactFirstProjectedWidth{};
    float artifactFirstProjectedHeight{};
    float artifactFirstSideMask{};
    float artifactFirstLocalInteriorPixels{};
    float artifactFirstRayX{};
    float artifactFirstRayY{};
    float artifactFirstRayZ{};
    std::array<std::uint64_t, 6> artifactIsolatedMinorPixelHistogram{};
    std::array<std::uint64_t, 6> artifactTerminationCounts{};
    std::uint32_t artifactFirstMissPixelX{};
    std::uint32_t artifactFirstMissPixelY{};
    std::uint32_t artifactFirstMissTermination{};
    std::uint32_t artifactFirstMissTile{0xffffffffU};
    std::uint32_t artifactFirstMissColumnHeight{};
    float artifactFirstMissTravel{};
    float artifactFirstMissFar{};
    float artifactFirstMissCapDenominator{};
    float artifactFirstMissCapDistance{};
    float artifactFirstMissConeMargin{};
    float artifactFirstMissRayX{};
    float artifactFirstMissRayY{};
    float artifactFirstMissRayZ{};
    bool rayProbeValid{};
    bool rayProbeHit{};
    std::uint32_t rayProbeClass{};
    std::uint32_t rayProbeTermination{};
    std::uint32_t rayProbeTile{0xffffffffU};
    std::uint32_t rayProbeLayer{};
    std::uint32_t rayProbeMaterial{};
    float rayProbeTravel{};
    float rayProbeIntervalNear{};
    float rayProbeIntervalFar{};
    float rayProbeCandidateDistance{};
    float rayProbeClosestApproach{};
    float rayProbeNormalX{};
    float rayProbeNormalY{};
    float rayProbeNormalZ{};
    float planetOuterScale{1.0F};
    float gpuStreamingMilliseconds{};
    float gpuRenderMilliseconds{};
    float gpuTotalComputeMilliseconds{};
    bool gpuTimestampsSupported{};
    std::uint32_t ddaEventsP50{};
    std::uint32_t ddaEventsP95{};
    std::uint32_t ddaEventsP99{};
    std::uint32_t ddaEventsMaximum{};
    std::uint32_t atlasRefinementsP50{};
    std::uint32_t atlasRefinementsP95{};
    std::uint32_t atlasRefinementsP99{};
    std::uint32_t atlasRefinementsMaximum{};
    std::uint64_t telemetryRaySamples{};
    std::uint64_t conservativeBoundSkips{};
    std::uint64_t conservativeBoundNonfinite{};
    std::uint64_t macroNodeTests{};
    std::uint64_t macroCandidates{};
    std::uint64_t macroFallbacks{};
    std::uint64_t compactTileLoads{};
    std::uint64_t microSdfActiveSamples{};
    std::uint64_t microSdfNonfiniteRejects{};
    std::uint64_t microSdfBoundaryRejects{};
    std::uint64_t microSdfOversteps{};
    std::uint64_t adaptiveSdfActiveSamples{};
    std::uint64_t adaptiveSdfNonfiniteRejects{};
    std::uint64_t adaptiveSdfBoundaryRejects{};
    std::uint64_t adaptiveSdfRootIterations{};
    std::uint32_t adaptiveSdfLevelP50{};
    std::uint32_t adaptiveSdfLevelP95{};
    std::uint32_t adaptiveSdfLevelMaximum{};
    std::uint64_t fractalPlanetHits{};
    std::uint64_t fractalPlanetIntervalSkips{};
    std::uint64_t fractalPlanetNonfiniteRejects{};
    std::uint64_t fractalPlanetRootIterations{};
    std::uint64_t fractalPlanetRecoveredHits{};
    std::uint64_t fractalPlanetClosedShellMisses{};
    std::uint32_t fractalPlanetLevelP50{};
    std::uint32_t fractalPlanetLevelP95{};
    std::uint32_t fractalPlanetLevelMaximum{};
    std::uint32_t fractalVoxelCacheLevel{};
    std::uint64_t fractalVoxelRefineCount{};
    std::uint64_t fractalVoxelMergeCount{};
    float fractalVoxelProjectedPixels{};
    float fractalVoxelTransition{};
    std::uint64_t differentialSamples{};
    std::uint64_t differentialHitMismatches{};
    std::uint64_t differentialDistanceMismatches{};
    std::uint32_t differentialMaximumDistanceUlps{};
    double startupSeconds{};
    std::uint64_t cpuPeakBytes{};
    bool topologyCacheHit{};
    std::uint64_t topologyCacheBytes{};
    double topologyCacheSeconds{};
    bool portalGrPrecomputeReady{};
    bool portalGrCacheHit{};
    std::uint64_t portalGrTableBytes{};
    double portalGrPrecomputeSeconds{};
    float portalGrMaximumError{};
    std::uint64_t portalGrTableLookups{};
    std::uint64_t portalGrLocalRefinements{};
    std::uint64_t portalGrNonfiniteRays{};
    std::uint64_t portalGrNegativeEndRays{};
    std::uint64_t globalMappedOriginRepairs{};
    std::uint64_t globalMappedFiniteRays{};
    std::uint64_t globalRadialPositiveRays{};
    std::uint64_t globalRadialNonPositiveRays{};
    std::uint64_t globalNearTangentRays{};
    std::uint64_t globalExactTangentRays{};
    std::uint64_t globalTerrainIntervalRays{};
    std::uint64_t globalDdaCandidateRays{};
    std::uint64_t globalFinalSkyRays{};
    std::uint64_t globalAtmosphereIntervalRays{};
    std::uint64_t globalCloudIntervalRays{};
    std::uint64_t globalBackFacingHitRejects{};
    std::uint64_t globalBackFacingHitRecoveries{};
    std::uint64_t globalFrontFacingHits{};
    std::uint64_t globalObserverBroadCandidates{};
    std::uint64_t globalObserverExactHits{};
    std::uint64_t globalObserverSelectedHits{};
    std::uint64_t globalCurvedSelectedRays{};
    std::uint64_t globalObserverMediaSegments{};
    std::uint64_t globalMouthARays{};
    std::uint64_t globalMouthBRays{};
    std::uint64_t globalExteriorDirectRays{};
    std::uint64_t globalObserverHitsBehindMouth{};
    std::uint64_t globalBroadShellCurvedRays{};
    std::uint64_t globalBroadShellMouthARays{};
    std::uint64_t globalBroadShellMouthBRays{};
    std::uint64_t globalContentShellCrossingRays{};
    std::uint32_t globalFrameRaySamples{};
    std::uint32_t globalFrameTerrainIntervals{};
    std::uint32_t globalFrameAtmosphereIntervals{};
    std::uint32_t globalFrameCloudIntervals{};
    std::uint32_t globalFrameCurvedSelectedRays{};
    std::uint32_t globalFrameMouthARays{};
    std::uint32_t globalFrameMouthBRays{};
    std::uint32_t globalFrameContentShellCrossingRays{};
    bool globalFrameCenterSampleValid{};
    float globalFrameCenterLinearR{};
    float globalFrameCenterLinearG{};
    float globalFrameCenterLinearB{};
    bool globalFrameCenterHit{};
    float globalFrameCenterDepth{};
    float globalFrameCenterMappedOriginX{};
    float globalFrameCenterMappedOriginY{};
    float globalFrameCenterMappedOriginZ{};
    float globalFrameCenterMappedDirectionX{};
    float globalFrameCenterMappedDirectionY{};
    float globalFrameCenterMappedDirectionZ{};
    float globalFrameCenterStarFootprint{};
    std::uint32_t globalFrameCenterHitClass{};
    std::uint32_t globalFrameCenterMouthOwner{2U};
    std::uint64_t globalAffineBudgetExhaustions{};
    std::uint32_t globalFrameAffineBudgetExhaustions{};
    std::uint64_t globalSpatialAaCandidatePixels{};
    std::uint64_t globalSpatialAaPixels{};
    std::uint64_t globalSpatialAaFamilyTransitions{};
    std::uint64_t globalSpatialAaSubrays{};
    std::uint64_t globalSpatialAaRejectedPixels{};
    std::uint64_t globalSpatialAaPixelSamples{};
    std::uint32_t globalFrameSpatialAaPixels{};
    std::uint32_t globalFrameSpatialAaPixelSamples{};
    std::uint64_t globalMetricPathSteps{};
    std::uint64_t globalMetricPathSamples{};
    std::uint64_t globalSpatialAaPathSteps{};
    std::uint64_t globalSpatialAaPathSamples{};
};

} // namespace voxel
