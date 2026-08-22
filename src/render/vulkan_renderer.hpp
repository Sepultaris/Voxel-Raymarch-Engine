#pragma once

#include "app/surface_player_collision.hpp"
#include "planet/geodesic_topology.hpp"
#include "render/render_settings.hpp"

#include <vulkan/vulkan.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

struct SDL_Window;

namespace voxel {

class VulkanRenderer final {
public:
    explicit VulkanRenderer(SDL_Window* window, bool forceTopologyRegeneration = false);
    ~VulkanRenderer();

    VulkanRenderer(const VulkanRenderer&) = delete;
    VulkanRenderer& operator=(const VulkanRenderer&) = delete;

    [[nodiscard]] bool beginUiFrame();
    void render(float time, const RenderSettings& settings);
    void requestResize() noexcept { resizeRequested_ = true; }
    [[nodiscard]] const RendererStats& stats() const noexcept { return stats_; }
    [[nodiscard]] const PortalGrPrecomputedTable& portalGrTable() const noexcept {
        return portalGrTable_;
    }
    [[nodiscard]] SurfaceTerrainContact querySurfaceTerrain(
        const std::array<float, 3>& radial, float planetRadius,
        float capsuleRadius, float sweepDistance = 0.0F) const noexcept;

private:
    struct SwapchainSupport {
        VkSurfaceCapabilitiesKHR capabilities{};
        std::vector<VkSurfaceFormatKHR> formats;
        std::vector<VkPresentModeKHR> presentModes;
    };

    struct FrameConstants {
        float time{};
        float planetRadius{};
        std::uint32_t maxMarchSteps{};
        float hitEpsilon{};
        std::uint32_t width{};
        std::uint32_t height{};
        float cameraDistance{};
        std::uint32_t voxelResolution{};
        float cameraYaw{};
        float cameraPitch{};
        float brushNdcX{};
        float brushNdcY{};
        std::uint32_t editMode{};
        std::uint32_t brushMaterial{};
        std::uint32_t simulationTick{};
        std::uint32_t visualizationMode{};
        std::uint32_t surfaceTileCount{};
        std::uint32_t residentPageCount{};
        float planetOuterScale{1.0F};
        std::uint32_t geodesicTraversalMode{2U};
        std::uint32_t streamSamplePhase{};
        std::uint32_t terrainSeed{1337U};
        float terrainContinentScale{1.35F};
        float terrainContinentStrength{0.95F};
        float terrainMountainScale{5.5F};
        float terrainMountainStrength{0.52F};
        float terrainRoughness{0.60F};
        float terrainOceanLevel{0.46F};
        float terrainPolarStrength{0.20F};
        std::uint32_t cameraMode{};
        float surfaceHeading{};
        float surfaceLookPitch{};
    };
    static_assert(sizeof(FrameConstants) == 128);

    struct alignas(16) PageRequestFeedbackHeader {
        std::uint32_t requestCount{};
        std::uint32_t overflowCount{};
        std::uint32_t frameGeneration{};
        std::uint32_t reserved{};
    };
    static_assert(sizeof(PageRequestFeedbackHeader) == 16);

    struct PageRequestGpu {
        std::uint32_t tileIndex{};
        std::uint32_t expectedGeneration{};
        std::uint32_t occupancyMask{};
        std::uint32_t packedColumnState{};
    };
    static_assert(sizeof(PageRequestGpu) == 16);

    struct alignas(16) ArtifactDiagnosticHeaderGpu {
        std::array<std::uint32_t, 4> control{};
        std::array<std::uint32_t, 4> classCounts0{};
        std::array<std::uint32_t, 4> classCounts1{};
        std::array<std::uint32_t, 4> horizonCounts{};
        std::array<std::uint32_t, 4> noOwnerApproachCounts{};
        std::array<std::uint32_t, 4> skyCounts{};
        std::array<std::uint32_t, 2049> scaleDdaHistogram{};
        std::array<std::uint32_t, 33> scaleAtlasHistogram{};
        std::array<std::uint32_t, 6> scalePadding{};
        std::array<std::uint32_t, 4> scaleMacroStats{};
    };
    static_assert(sizeof(ArtifactDiagnosticHeaderGpu) == 8464);

    struct alignas(16) ArtifactDiagnosticRecordGpu {
        std::array<std::uint32_t, 4> pixelClassTermination{};
        std::array<std::uint32_t, 4> cell{};
        std::array<std::uint32_t, 4> footprint{};
        std::array<std::uint32_t, 4> ray{};
        std::array<std::uint32_t, 4> state{};
        std::array<std::uint32_t, 4> intervalCandidate{};
        std::array<std::uint32_t, 4> normalMaterial{};
    };
    static_assert(sizeof(ArtifactDiagnosticRecordGpu) == 112);

    struct PendingPageUpload {
        std::uint32_t physicalPage{};
        std::uint32_t incomingTile{};
        std::uint32_t evictedTile{};
        VkDeviceSize pageSourceOffset{};
        VkDeviceSize incomingEntrySourceOffset{};
        VkDeviceSize evictedEntrySourceOffset{};
    };

    void initialize();
    void shutdown() noexcept;
    void createInstance();
    void createDebugMessenger();
    void createSurface();
    void choosePhysicalDevice();
    void createDevice();
    void createSwapchain();
    void createRenderPass();
    void createSwapchainViews();
    void createOutputImage();
    void createVoxelResources();
    void createPlanetTopologyResources();
    void createComputeResources();
    void createFramebuffers();
    void createCommandResources();
    void createSynchronization();
    void createPresentSemaphores();
    void initializeVoxelVolume();
    void createImGuiContext();
    void initializeImGuiBackends();
    void recreateSwapchain();
    void processPageRequests(float time, const RenderSettings& settings);
    void processArtifactDiagnostics();
    void reportScaleLabStartup(std::string_view stage, std::uint64_t current,
                               std::uint64_t total);
    void recordCommands(
        std::uint32_t imageIndex,
        const FrameConstants& constants,
        std::uint32_t simulationSteps,
        bool resetVoxelVolume,
        bool regenerateTerrain);

    void destroySwapchainResources() noexcept;
    void destroyImGuiBackends() noexcept;

    [[nodiscard]] SwapchainSupport querySwapchainSupport(VkPhysicalDevice device) const;
    [[nodiscard]] bool supportsRequiredDeviceFeatures(VkPhysicalDevice device, std::uint32_t& family) const;
    [[nodiscard]] std::uint32_t findMemoryType(std::uint32_t typeBits, VkMemoryPropertyFlags flags) const;

    SDL_Window* window_{};
    bool validationEnabled_{};
    bool resizeRequested_{};
    bool imguiContextCreated_{};
    bool imguiBackendsReady_{};
    bool outputImageInitialized_{};

    VkInstance instance_{};
    VkDebugUtilsMessengerEXT debugMessenger_{};
    VkSurfaceKHR surface_{};
    VkPhysicalDevice physicalDevice_{};
    VkDevice device_{};
    std::uint32_t queueFamily_{};
    VkQueue queue_{};

    VkSwapchainKHR swapchain_{};
    VkFormat swapchainFormat_{VK_FORMAT_UNDEFINED};
    VkExtent2D swapchainExtent_{};
    std::uint32_t swapchainMinImageCount_{};
    std::vector<VkImage> swapchainImages_;
    std::vector<VkImageView> swapchainViews_;
    std::vector<VkFramebuffer> framebuffers_;
    VkRenderPass renderPass_{};

    VkImage outputImage_{};
    VkDeviceMemory outputMemory_{};
    VkImageView outputView_{};

    std::array<VkBuffer, 2> cellBuffers_{};
    std::array<VkDeviceMemory, 2> cellMemories_{};
    VkBuffer occupancyBuffer_{};
    VkDeviceMemory occupancyMemory_{};
    VkBuffer claimBuffer_{};
    VkDeviceMemory claimMemory_{};
    GeodesicTopology planetTopology_;
    VkBuffer geodesicTileBuffer_{};
    VkDeviceMemory geodesicTileMemory_{};
    VkBuffer tileLookupBuffer_{};
    VkDeviceMemory tileLookupMemory_{};
    VkBuffer tileBrickBuffer_{};
    VkDeviceMemory tileBrickMemory_{};
    VkBuffer geodesicBvhBuffer_{};
    VkDeviceMemory geodesicBvhMemory_{};
    VkBuffer geodesicWideNodeBuffer_{};
    VkDeviceMemory geodesicWideNodeMemory_{};
    VkBuffer geodesicWideItemBuffer_{};
    VkDeviceMemory geodesicWideItemMemory_{};
    VkBuffer geodesicColumnStateBuffer_{};
    VkDeviceMemory geodesicColumnStateMemory_{};
    VkBuffer geodesicPageTableBuffer_{};
    VkDeviceMemory geodesicPageTableMemory_{};
    VkBuffer geodesicPageRequestBuffer_{};
    VkDeviceMemory geodesicPageRequestMemory_{};
    void* geodesicPageRequestMapped_{};
    VkBuffer geodesicFarFieldBuffer_{};
    VkDeviceMemory geodesicFarFieldMemory_{};
    VkBuffer geodesicMacroHierarchyBuffer_{};
    VkDeviceMemory geodesicMacroHierarchyMemory_{};
    VkBuffer artifactDiagnosticBuffer_{};
    VkDeviceMemory artifactDiagnosticMemory_{};
    void* artifactDiagnosticMapped_{};
    VkBuffer portalLabParameterBuffer_{};
    VkDeviceMemory portalLabParameterMemory_{};
    void* portalLabParameterMapped_{};
    PortalGrPrecomputedTable portalGrTable_{};
    VkBuffer geodesicStreamUploadBuffer_{};
    VkDeviceMemory geodesicStreamUploadMemory_{};
    void* geodesicStreamUploadMapped_{};
    std::vector<std::uint32_t> physicalPageOwners_;
    std::vector<std::uint8_t> freshCollisionColumns_;
    std::vector<PendingPageUpload> pendingPageUploads_;

    VkDescriptorSetLayout computeDescriptorSetLayout_{};
    VkDescriptorPool computeDescriptorPool_{};
    std::array<VkDescriptorSet, 2> computeDescriptorSets_{};
    VkPipelineLayout computePipelineLayout_{};
    VkPipeline computePipeline_{};
    VkPipeline globalSpatialAaPipeline_{};
    VkPipeline globalSpatialAaResolvePipeline_{};
    VkPipeline voxelInitPipeline_{};
    VkPipeline voxelEditPipeline_{};
    VkPipeline geodesicEditPipeline_{};
    VkPipeline geodesicTerrainInitPipeline_{};
    VkPipeline geodesicFarFieldInitPipeline_{};
    VkPipeline geodesicStreamRequestPipeline_{};
    VkPipeline sandClaimPipeline_{};
    VkPipeline sandCommitPipeline_{};
    std::uint32_t activeCellBuffer_{};
    std::uint32_t simulationTick_{};

    VkDescriptorPool imguiDescriptorPool_{};
    VkCommandPool commandPool_{};
    VkCommandBuffer commandBuffer_{};
    VkSemaphore imageAvailable_{};
    std::vector<VkSemaphore> renderingFinished_;
    VkFence frameFence_{};
    VkQueryPool timestampQueryPool_{};
    float timestampPeriodNanoseconds_{};
    std::uint32_t timestampValidBits_{};
    bool timestampsSupported_{};
    bool timestampPending_{};
    bool pageFeedbackPending_{};
    bool artifactDiagnosticPending_{};
    bool forceTopologyRegeneration_{};
    std::uint32_t pageFeedbackGeneration_{};
    std::array<std::uint64_t, 2049> scaleDdaHistogram_{};
    std::array<std::uint64_t, 33> scaleAtlasHistogram_{};
    std::uint64_t scaleBoundSkips_{};
    std::uint64_t scaleBoundNonfinite_{};
    std::uint64_t scaleMacroNodeTests_{};
    std::uint64_t scaleMacroCandidates_{};
    std::uint64_t scaleMacroFallbacks_{};
    std::uint64_t scaleCompactTileLoads_{};
    std::uint64_t microSdfActiveSamples_{};
    std::uint64_t microSdfNonfiniteRejects_{};
    std::uint64_t microSdfBoundaryRejects_{};
    std::uint64_t microSdfOversteps_{};
    std::uint64_t adaptiveSdfActiveSamples_{};
    std::uint64_t adaptiveSdfNonfiniteRejects_{};
    std::uint64_t adaptiveSdfBoundaryRejects_{};
    std::uint64_t adaptiveSdfRootIterations_{};
    std::array<std::uint64_t, 33> adaptiveSdfLevelHistogram_{};
    bool adaptiveSdfEnabled_{true};
    std::uint32_t adaptiveSdfSeed_{1337U};
    std::uint32_t adaptiveSdfMaximumLevels_{9U};
    float adaptiveSdfLayerFraction_{0.15F};
    bool fractalPlanetSdfEnabled_{true};
    std::uint32_t fractalPlanetSdfSeed_{1337U};
    std::uint32_t fractalPlanetSdfMaximumLevels_{10U};
    float fractalPlanetSdfMicroRelief_{0.012F};
    std::uint64_t fractalPlanetHits_{};
    std::uint64_t fractalPlanetIntervalSkips_{};
    std::uint64_t fractalPlanetNonfiniteRejects_{};
    std::uint64_t fractalPlanetRootIterations_{};
    std::uint64_t fractalPlanetRecoveredHits_{};
    std::uint64_t fractalPlanetClosedShellMisses_{};
    std::array<std::uint64_t, 33> fractalPlanetLevelHistogram_{};
    bool fractalVoxelCacheInitialized_{};
    std::uint32_t fractalVoxelCacheLevel_{};
    std::uint64_t fractalVoxelRefineCount_{};
    std::uint64_t fractalVoxelMergeCount_{};
    float fractalVoxelProjectedPixels_{};
    float fractalVoxelTransition_{};
    std::array<float, 3> fractalVoxelCacheRadial_{0.0F, 0.0F, 1.0F};
    std::uint32_t fractalVoxelCacheMorphQuantized_{0xffffffffU};
    bool fractalVoxelCacheDirty_{true};
    FrameConstants lightingConstants_{};
    std::uint64_t scaleDifferentialSamples_{};
    std::uint64_t scaleDifferentialHitMismatches_{};
    std::uint64_t scaleDifferentialDistanceMismatches_{};
    std::uint32_t scaleDifferentialMaximumDistanceUlps_{};

    std::string deviceName_;
    std::string startupProgressStage_;
    std::uint32_t startupProgressPercent_{0xffffffffU};
    std::chrono::steady_clock::time_point startupBegin_{};
    RendererStats stats_{};
};

} // namespace voxel
