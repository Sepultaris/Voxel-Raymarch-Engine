#include "render/vulkan_renderer.hpp"
#include "planet/scale_lab_guardrails.hpp"
#include "planet/geodesic_topology_cache.hpp"
#include "planet/geodesic_local_ao.hpp"
#include "planet/adaptive_planet_sdf.hpp"
#include "planet/fractal_planet_sdf.hpp"
#include "planet/fractal_voxel_sdf.hpp"
#include "render/portal_lab.hpp"
#include "render/intrinsic_ellis_manifold.hpp"

#include <SDL.h>
#include <SDL_vulkan.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_vulkan.h>

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#if defined(_WIN32)
#include <windows.h>
#include <psapi.h>
#endif

namespace voxel {
namespace {

constexpr std::array<const char*, 1> kValidationLayers{"VK_LAYER_KHRONOS_validation"};
constexpr std::array<const char*, 1> kDeviceExtensions{VK_KHR_SWAPCHAIN_EXTENSION_NAME};
constexpr std::uint32_t kVoxelResolution = 96;
constexpr std::uint32_t kMacrocellSize = 8;
constexpr std::uint32_t kMacrocellResolution = kVoxelResolution / kMacrocellSize;
constexpr std::uint32_t kGeodesicFrequency = VOXEL_GEODESIC_FREQUENCY;
constexpr std::uint32_t kTileBrickResolution = VOXEL_TILE_RADIAL_LAYERS;
constexpr std::uint32_t kResidentGeodesicPages = VOXEL_RESIDENT_GEODESIC_PAGES;
constexpr bool kReferenceTraversalEnabled = VOXEL_ENABLE_REFERENCE_TRAVERSAL != 0;
constexpr bool kScaleLabBuild = VOXEL_SCALE_LAB != 0;
constexpr bool kAdaptiveSdfBuild = VOXEL_ADAPTIVE_SDF_LAB != 0;
constexpr bool kFractalPlanetSdfBuild = VOXEL_FRACTAL_PLANET_SDF_LAB != 0;
constexpr bool kFractalVoxelSdfBuild = VOXEL_FRACTAL_VOXEL_SDF_LAB != 0;
constexpr bool kPortalLabBuild = VOXEL_PORTAL_LAB != 0;
constexpr bool kIntrinsicPortalLabBuild = VOXEL_INTRINSIC_PORTAL_LAB != 0;
#if defined(VOXEL_EIGHT_PLANET_SYSTEM_LAB)
constexpr bool kEightPlanetSystemLabBuild = VOXEL_EIGHT_PLANET_SYSTEM_LAB != 0;
#else
constexpr bool kEightPlanetSystemLabBuild = false;
#endif
#if defined(VOXEL_GLOBAL_METRIC_LAB)
constexpr bool kGlobalMetricLabBuild = VOXEL_GLOBAL_METRIC_LAB != 0;
#else
constexpr bool kGlobalMetricLabBuild = false;
#endif
static_assert(kGeodesicFrequency > 0U);
static_assert(kTileBrickResolution > 0U);
static_assert(kResidentGeodesicPages > 0U);
static_assert(kTileBrickResolution <= 32U, "Column occupancy masks currently support up to 32 layers");
constexpr std::uint32_t kTileLookupWidth = VOXEL_TILE_LOOKUP_WIDTH;
constexpr std::uint32_t kTileLookupHeight = VOXEL_TILE_LOOKUP_HEIGHT;
constexpr std::uint32_t kFarFieldLodLevels = 11U;
constexpr std::uint32_t kScaleLabHierarchyPasses = kFarFieldLodLevels + 2U;
constexpr std::uint32_t kScaleLabAoInitLevel = kFarFieldLodLevels + 1U;
constexpr std::uint32_t farFieldNodeCount() {
    std::uint32_t total = 0U;
    std::uint32_t width = kTileLookupWidth;
    std::uint32_t height = kTileLookupHeight;
    for (std::uint32_t level = 0U; level < kFarFieldLodLevels; ++level) {
        total += width * height;
        width = std::max(width >> 1U, 1U);
        height = std::max(height >> 1U, 1U);
    }
    return total;
}
constexpr VkDeviceSize kFarFieldBytes =
    static_cast<VkDeviceSize>(farFieldNodeCount() + 1U) * sizeof(std::uint32_t);
constexpr VkDeviceSize kMacroHierarchyBytes =
    static_cast<VkDeviceSize>(farFieldNodeCount() + 1U) *
    sizeof(std::array<std::uint32_t, 4>);
constexpr std::uint32_t kPageRequestCapacity = 16384;
constexpr std::uint32_t kCollisionProfileRequestCount = 43;
constexpr std::uint32_t kArtifactDiagnosticCapacity = 262144;
constexpr std::uint32_t kPageStreamBudget = 256;
constexpr std::uint32_t kStreamRequestSamples = kPageRequestCapacity;
constexpr VkDeviceSize kCellBufferSize =
    static_cast<VkDeviceSize>(kVoxelResolution) * kVoxelResolution * kVoxelResolution * sizeof(std::uint32_t);
constexpr VkDeviceSize kOccupancyBufferSize =
    static_cast<VkDeviceSize>(kMacrocellResolution) * kMacrocellResolution * kMacrocellResolution * sizeof(std::uint32_t);

#if VOXEL_EIGHT_PLANET_SYSTEM_LAB
constexpr std::uint32_t kSystemHierarchyHeaderWords = 32U;
constexpr std::uint32_t kSystemHierarchyNodeWords = 16U;
constexpr std::uint32_t kSystemAuthorityHeaderWords = 32U;
constexpr std::uint32_t kSystemAggregateWords = 2U;
constexpr std::uint32_t kSystemPlanetDescriptorWords = 16U;
constexpr std::uint32_t kSystemPageTableCapacity = 1024U;
constexpr std::uint32_t kSystemPageTableEntryWords = 4U;
constexpr std::uint32_t kSystemEditCapacity = 256U;
constexpr std::uint32_t kSystemEditWords = 4U;
constexpr std::uint32_t kSystemTelemetryWords = 16U;

[[nodiscard]] std::vector<std::uint32_t> packSystemHierarchy(
    const system_lab::SharedLodHierarchy& hierarchy) {
    const std::uint32_t nodeWords = static_cast<std::uint32_t>(
        hierarchy.nodes.size()) * kSystemHierarchyNodeWords;
    const std::uint32_t leafMapOffset = kSystemHierarchyHeaderWords + nodeWords;
    std::vector<std::uint32_t> words(
        static_cast<std::size_t>(leafMapOffset) +
        hierarchy.leafToFinestCoarse.size(), 0U);
    words[0] = 0x48584c32U; // HXL2
    words[1] = 2U;
    words[2] = static_cast<std::uint32_t>(hierarchy.nodes.size());
    words[3] = static_cast<std::uint32_t>(hierarchy.leafToFinestCoarse.size());
    words[4] = leafMapOffset;
    words[5] = hierarchy.activeLevels;
    for (std::uint32_t level = 0U;
         level < system_lab::kSystemCoarseLodLevels; ++level) {
        words[6U + level] = hierarchy.levelOffsets[level];
        words[16U + level] = hierarchy.levelCounts[level];
    }
    for (std::uint32_t index = 0U; index < hierarchy.nodes.size(); ++index) {
        const system_lab::SharedLodNode& node = hierarchy.nodes[index];
        const std::uint32_t base = kSystemHierarchyHeaderWords +
            index * kSystemHierarchyNodeWords;
        words[base + 0U] = std::bit_cast<std::uint32_t>(node.center[0]);
        words[base + 1U] = std::bit_cast<std::uint32_t>(node.center[1]);
        words[base + 2U] = std::bit_cast<std::uint32_t>(node.center[2]);
        words[base + 3U] = std::bit_cast<std::uint32_t>(node.surfaceWidth);
        std::copy(node.neighbors.begin(), node.neighbors.end(),
                  words.begin() + base + 4U);
        words[base + 10U] = node.parent;
        words[base + 11U] = node.level;
        words[base + 12U] = node.pentagon ? 1U : 0U;
    }
    std::copy(hierarchy.leafToFinestCoarse.begin(),
              hierarchy.leafToFinestCoarse.end(),
              words.begin() + leafMapOffset);
    return words;
}

[[nodiscard]] std::vector<std::uint32_t> gatherPinnedSystemPages(
    const GeodesicTopology& topology, std::array<float, 3> direction) {
    const std::uint32_t center = system_lab::locateGeodesicTile(topology, direction);
    std::set<std::uint32_t> visited{center};
    std::set<std::uint32_t> frontier{center};
    for (std::uint32_t ring = 0U; ring < 2U; ++ring) {
        std::set<std::uint32_t> next;
        for (const std::uint32_t tileIndex : frontier) {
            const GeodesicTileGpu& tile = topology.tiles[tileIndex];
            for (std::uint32_t slot = 0U; slot < tile.brickInfo[2]; ++slot) {
                const std::uint32_t neighbor = slot < 4U
                    ? tile.neighborsLow[slot]
                    : tile.neighborsHigh[slot - 4U];
                if (visited.insert(neighbor).second) next.insert(neighbor);
            }
        }
        frontier = std::move(next);
    }
    std::set<std::uint32_t> pages;
    for (const std::uint32_t tileIndex : visited) {
        pages.insert(tileIndex / system_lab::kSystemPageColumns);
    }
    return {pages.begin(), pages.end()};
}

[[nodiscard]] std::vector<std::uint32_t> packSystemAuthorities(
    const GeodesicTopology& topology,
    const system_lab::SharedLodHierarchy& hierarchy,
    const system_lab::Settings& settings) {
    const auto aggregates = system_lab::buildPlanetAggregates(
        topology, hierarchy, settings.planetSeeds);
    const std::uint32_t aggregateOffset = kSystemAuthorityHeaderWords;
    const std::uint32_t aggregateStride = static_cast<std::uint32_t>(
        hierarchy.nodes.size()) * kSystemAggregateWords;
    const std::uint32_t descriptorOffset = aggregateOffset +
        system_lab::kPlanetCount * aggregateStride;
    const std::uint32_t pageTableOffset = descriptorOffset +
        system_lab::kPlanetCount * kSystemPlanetDescriptorWords;
    const std::uint32_t pageTableStride =
        kSystemPageTableCapacity * kSystemPageTableEntryWords;
    const std::uint32_t residentOffset = pageTableOffset +
        system_lab::kPlanetCount * pageTableStride;
    const std::uint32_t residentStride =
        system_lab::kSystemResidentPagesPerPlanet *
        system_lab::kSystemPageColumns;
    const std::uint32_t editOffset = residentOffset +
        system_lab::kPlanetCount * residentStride;
    const std::uint32_t editStride = kSystemEditCapacity * kSystemEditWords;
    const std::uint32_t telemetryOffset = editOffset +
        system_lab::kPlanetCount * editStride;
    const std::uint32_t totalWords = telemetryOffset +
        system_lab::kPlanetCount * kSystemTelemetryWords;
    std::vector<std::uint32_t> words(totalWords, 0U);
    words[0] = 0x41555432U; // AUT2
    words[1] = 3U;
    words[2] = static_cast<std::uint32_t>(hierarchy.nodes.size());
    words[3] = system_lab::kPlanetCount;
    words[4] = aggregateOffset;
    words[5] = aggregateStride;
    words[6] = descriptorOffset;
    words[7] = kSystemPlanetDescriptorWords;
    words[8] = pageTableOffset;
    words[9] = pageTableStride;
    words[10] = kSystemPageTableCapacity;
    words[11] = residentOffset;
    words[12] = residentStride;
    words[13] = system_lab::kSystemResidentPagesPerPlanet;
    words[14] = editOffset;
    words[15] = editStride;
    words[16] = kSystemEditCapacity;
    words[17] = telemetryOffset;
    words[18] = kSystemTelemetryWords;
    words[19] = totalWords;
    words[20] = std::bit_cast<std::uint32_t>(0.92F);
    words[21] = std::bit_cast<std::uint32_t>(0.12F);
    words[22] = kSystemAggregateWords;

    for (std::uint32_t planet = 0U; planet < system_lab::kPlanetCount; ++planet) {
        for (std::uint32_t node = 0U; node < hierarchy.nodes.size(); ++node) {
            const system_lab::Aggregate& aggregate = aggregates[planet][node];
            const std::uint32_t base = aggregateOffset + planet * aggregateStride +
                node * kSystemAggregateWords;
            const system_lab::PackedAggregateGpu packed =
                system_lab::packAggregateGpu(aggregate);
            words[base + 0U] = packed.heights;
            words[base + 1U] = packed.materialError;
        }

        const std::uint32_t descriptor = descriptorOffset +
            planet * kSystemPlanetDescriptorWords;
        words[descriptor + 0U] = settings.planetSeeds[planet];
        words[descriptor + 1U] = 1U;
        words[descriptor + 2U] = system_lab::kSystemResidentPagesPerPlanet;
        words[descriptor + 3U] = 0U; // active sparse edits; capacity is in header[16]
        const system_lab::AtmosphereCloudControls media{
            0.16F + 0.018F * static_cast<float>(planet),
            0.055F + 0.004F * static_cast<float>(planet % 3U),
            0.34F + 0.055F * static_cast<float>(planet % 4U),
            0.48F + 0.045F * static_cast<float>(planet % 3U),
            0.038F, 0.082F};
        words[descriptor + 4U] = std::bit_cast<std::uint32_t>(media.atmosphereDensity);
        words[descriptor + 5U] = std::bit_cast<std::uint32_t>(media.atmosphereHeight);
        words[descriptor + 6U] = std::bit_cast<std::uint32_t>(media.cloudCoverage);
        words[descriptor + 7U] = std::bit_cast<std::uint32_t>(media.cloudDensity);
        words[descriptor + 8U] = std::bit_cast<std::uint32_t>(media.cloudBase);
        words[descriptor + 9U] = std::bit_cast<std::uint32_t>(media.cloudTop);

        const std::uint32_t tableBase = pageTableOffset + planet * pageTableStride;
        for (std::uint32_t entry = 0U; entry < kSystemPageTableCapacity; ++entry) {
            words[tableBase + entry * kSystemPageTableEntryWords] = 0xffffffffU;
        }
        const auto planets = system_lab::defaultPlanets();
        const system_lab::BodyFrame frame = system_lab::evaluateOrbit(
            planets[planet].orbit, 0.0, 0.0);
        const std::array<float, 3> pinnedDirection{
            static_cast<float>(frame.radial.x),
            static_cast<float>(frame.radial.y),
            static_cast<float>(frame.radial.z)};
        const std::uint32_t editedTile = system_lab::locateGeodesicTile(
            topology, pinnedDirection);
        std::vector<std::uint32_t> virtualPages = gatherPinnedSystemPages(
            topology, pinnedDirection);
        const std::uint32_t virtualPageCount = static_cast<std::uint32_t>(
            (topology.tiles.size() + system_lab::kSystemPageColumns - 1U) /
            system_lab::kSystemPageColumns);
        std::set<std::uint32_t> used(virtualPages.begin(), virtualPages.end());
        for (std::uint32_t physical = static_cast<std::uint32_t>(virtualPages.size());
             physical < system_lab::kSystemResidentPagesPerPlanet; ++physical) {
            std::uint32_t candidate = (physical * 7919U + planet * 977U) %
                virtualPageCount;
            while (!used.insert(candidate).second) {
                candidate = (candidate + 1U) % virtualPageCount;
            }
            virtualPages.push_back(candidate);
        }
        virtualPages.resize(system_lab::kSystemResidentPagesPerPlanet);
        const std::uint32_t poolBase = residentOffset + planet * residentStride;
        for (std::uint32_t physical = 0U;
             physical < system_lab::kSystemResidentPagesPerPlanet; ++physical) {
            const std::uint32_t virtualPage = virtualPages[physical];
            std::uint32_t slot = system_lab::systemAvalanche(virtualPage) &
                (kSystemPageTableCapacity - 1U);
            while (words[tableBase + slot * kSystemPageTableEntryWords] != 0xffffffffU) {
                slot = (slot + 1U) & (kSystemPageTableCapacity - 1U);
            }
            const std::uint32_t entry = tableBase + slot * kSystemPageTableEntryWords;
            words[entry + 0U] = virtualPage;
            words[entry + 1U] = physical;
            words[entry + 2U] = 1U;
            words[entry + 3U] = physical < virtualPages.size() ? 1U : 0U;
            for (std::uint32_t column = 0U;
                 column < system_lab::kSystemPageColumns; ++column) {
                const std::uint32_t tile = virtualPage * system_lab::kSystemPageColumns +
                    column;
                const system_lab::CompactColumnState state =
                    system_lab::generatedColumn(settings.planetSeeds[planet], tile);
                words[poolBase + physical * system_lab::kSystemPageColumns + column] =
                    system_lab::packCompactColumnGpu(state);
            }
        }
        const std::uint32_t editsBase = editOffset + planet * editStride;
        for (std::uint32_t edit = 0U; edit < kSystemEditCapacity; ++edit) {
            words[editsBase + edit * kSystemEditWords] = 0xffffffffU;
        }
        const std::uint32_t editedLayers = 14U + planet % 7U;
        words[editsBase + 0U] = editedTile;
        words[editsBase + 1U] = editedLayers | (7U << 8U) | (2U << 16U);
        words[editsBase + 2U] = 1U;
        words[editsBase + 3U] = 0U;
        words[descriptor + 3U] = 1U;
        std::uint32_t dirtyNode = hierarchy.leafToFinestCoarse[editedTile];
        for (;;) {
            const std::uint32_t aggregateBase = aggregateOffset +
                planet * aggregateStride + dirtyNode * kSystemAggregateWords;
            words[aggregateBase + 1U] |= 0x80000000U;
            if (hierarchy.nodes[dirtyNode].level == 0U) break;
            dirtyNode = hierarchy.nodes[dirtyNode].parent;
        }
    }
    return words;
}
#endif

using CameraVector = std::array<float, 3>;

[[nodiscard]] float cameraDot(const CameraVector& left, const CameraVector& right) noexcept {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

[[nodiscard]] CameraVector cameraCross(const CameraVector& left,
                                       const CameraVector& right) noexcept {
    return {left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0]};
}

[[nodiscard]] CameraVector cameraNormalized(const CameraVector& value,
                                            CameraVector fallback) noexcept {
    const float lengthSquared = cameraDot(value, value);
    if (!(lengthSquared > 1e-12F) || !std::isfinite(lengthSquared)) {
        return fallback;
    }
    const float inverseLength = 1.0F / std::sqrt(lengthSquared);
    return {value[0] * inverseLength, value[1] * inverseLength,
            value[2] * inverseLength};
}

[[nodiscard]] CameraVector surfaceRadial(const RenderSettings& settings) noexcept {
    return cameraNormalized(
        {settings.surfaceRadialX, settings.surfaceRadialY, settings.surfaceRadialZ},
        {0.0F, 0.0F, 1.0F});
}

void surfaceTangentBasis(const CameraVector& radial, CameraVector& forward,
                         CameraVector& right) noexcept {
    const CameraVector reference = std::abs(radial[1]) < 0.95F
                                       ? CameraVector{0.0F, 1.0F, 0.0F}
                                       : CameraVector{0.0F, 0.0F, 1.0F};
    right = cameraNormalized(cameraCross(reference, radial), {1.0F, 0.0F, 0.0F});
    forward = cameraNormalized(cameraCross(radial, right), {0.0F, 1.0F, 0.0F});
}

[[nodiscard]] CameraVector cameraRadialDirection(float time,
                                                 const RenderSettings& settings) noexcept {
    if (settings.cameraMode == CameraControllerMode::SurfaceTraversal) {
        return surfaceRadial(settings);
    }
    if constexpr (kPortalLabBuild) {
        if (settings.cameraMode == CameraControllerMode::PortalFreeFly) {
            return cameraNormalized(
                {settings.portal.freeFlyPosition.x,
                 settings.portal.freeFlyPosition.y,
                 settings.portal.freeFlyPosition.z},
                {0.0F, 0.0F, 1.0F});
        }
    }
    const float orbit = settings.cameraYaw + time;
    const float cosinePitch = std::cos(settings.cameraPitch);
    return {std::sin(orbit) * cosinePitch, std::sin(settings.cameraPitch),
            std::cos(orbit) * cosinePitch};
}

void checkVk(VkResult result, const char* operation) {
    if (result != VK_SUCCESS) {
        throw std::runtime_error(std::string(operation) + " failed with Vulkan result " +
                                 std::to_string(static_cast<int>(result)));
    }
}

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT,
    const VkDebugUtilsMessengerCallbackDataEXT* callbackData,
    void*) {
    std::ostream& stream = severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT
                               ? std::cerr
                               : std::clog;
    stream << "Vulkan validation: " << callbackData->pMessage << '\n';
    return VK_FALSE;
}

VkDebugUtilsMessengerCreateInfoEXT debugMessengerInfo() {
    VkDebugUtilsMessengerCreateInfoEXT info{VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
    info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                           VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                       VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                       VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    info.pfnUserCallback = debugCallback;
    return info;
}

bool hasValidationLayer() {
    std::uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> layers(count);
    vkEnumerateInstanceLayerProperties(&count, layers.data());
    return std::ranges::any_of(layers, [](const VkLayerProperties& layer) {
        return std::strcmp(layer.layerName, kValidationLayers.front()) == 0;
    });
}

std::vector<std::byte> readBinaryFile(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("Unable to open shader: " + path.string());
    }
    const auto size = file.tellg();
    if (size <= 0 || size % 4 != 0) {
        throw std::runtime_error("Shader has invalid SPIR-V byte size: " + path.string());
    }
    std::vector<std::byte> bytes(static_cast<std::size_t>(size));
    file.seekg(0);
    file.read(reinterpret_cast<char*>(bytes.data()), size);
    if (!file) {
        throw std::runtime_error("Unable to read shader: " + path.string());
    }
    return bytes;
}

[[nodiscard]] std::uint64_t processPeakBytes() noexcept {
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS_EX counters{};
    counters.cb = sizeof(counters);
    if (GetProcessMemoryInfo(GetCurrentProcess(),
                             reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                             sizeof(counters)) != 0) {
        return static_cast<std::uint64_t>(counters.PeakWorkingSetSize);
    }
#endif
    return 0U;
}

[[nodiscard]] std::uint32_t pngCrc32(const std::uint8_t* data,
                                     std::size_t size) noexcept {
    std::uint32_t crc = 0xffffffffU;
    for (std::size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (std::uint32_t bit = 0; bit < 8U; ++bit) {
            const std::uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xedb88320U & mask);
        }
    }
    return ~crc;
}

void appendBigEndian(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    bytes.push_back(static_cast<std::uint8_t>(value >> 24U));
    bytes.push_back(static_cast<std::uint8_t>(value >> 16U));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8U));
    bytes.push_back(static_cast<std::uint8_t>(value));
}

void appendPngChunk(std::vector<std::uint8_t>& png,
                    const std::array<char, 4>& type,
                    const std::vector<std::uint8_t>& payload) {
    appendBigEndian(png, static_cast<std::uint32_t>(payload.size()));
    const std::size_t crcBegin = png.size();
    for (const char value : type) {
        png.push_back(static_cast<std::uint8_t>(value));
    }
    png.insert(png.end(), payload.begin(), payload.end());
    appendBigEndian(png, pngCrc32(png.data() + crcBegin,
                                 png.size() - crcBegin));
}

[[nodiscard]] bool writePngRgba8(const std::filesystem::path& outputPath,
                                 std::uint32_t width, std::uint32_t height,
                                 const std::uint8_t* rgba,
                                 std::string& error) {
    if (width == 0U || height == 0U || rgba == nullptr) {
        error = "capture image is empty";
        return false;
    }
    const std::size_t rowBytes = static_cast<std::size_t>(width) * 4U;
    std::vector<std::uint8_t> scanlines;
    scanlines.reserve((rowBytes + 1U) * static_cast<std::size_t>(height));
    for (std::uint32_t y = 0U; y < height; ++y) {
        scanlines.push_back(0U); // PNG filter: None
        const std::uint8_t* row = rgba + static_cast<std::size_t>(y) * rowBytes;
        scanlines.insert(scanlines.end(), row, row + rowBytes);
    }

    std::vector<std::uint8_t> zlib;
    zlib.reserve(scanlines.size() + scanlines.size() / 65535U * 5U + 8U);
    zlib.push_back(0x78U);
    zlib.push_back(0x01U); // deflate, no compression; deterministic
    std::size_t offset = 0U;
    while (offset < scanlines.size()) {
        const std::size_t count = std::min<std::size_t>(
            65535U, scanlines.size() - offset);
        const bool finalBlock = offset + count == scanlines.size();
        zlib.push_back(finalBlock ? 1U : 0U);
        const std::uint16_t length = static_cast<std::uint16_t>(count);
        const std::uint16_t inverse = static_cast<std::uint16_t>(~length);
        zlib.push_back(static_cast<std::uint8_t>(length));
        zlib.push_back(static_cast<std::uint8_t>(length >> 8U));
        zlib.push_back(static_cast<std::uint8_t>(inverse));
        zlib.push_back(static_cast<std::uint8_t>(inverse >> 8U));
        zlib.insert(zlib.end(), scanlines.begin() +
                    static_cast<std::ptrdiff_t>(offset),
                    scanlines.begin() +
                    static_cast<std::ptrdiff_t>(offset + count));
        offset += count;
    }
    std::uint32_t adlerA = 1U;
    std::uint32_t adlerB = 0U;
    for (const std::uint8_t value : scanlines) {
        adlerA = (adlerA + value) % 65521U;
        adlerB = (adlerB + adlerA) % 65521U;
    }
    appendBigEndian(zlib, (adlerB << 16U) | adlerA);

    std::vector<std::uint8_t> png{
        137U, 80U, 78U, 71U, 13U, 10U, 26U, 10U};
    std::vector<std::uint8_t> ihdr;
    appendBigEndian(ihdr, width);
    appendBigEndian(ihdr, height);
    ihdr.insert(ihdr.end(), {8U, 6U, 0U, 0U, 0U});
    appendPngChunk(png, {'I', 'H', 'D', 'R'}, ihdr);
    appendPngChunk(png, {'I', 'D', 'A', 'T'}, zlib);
    appendPngChunk(png, {'I', 'E', 'N', 'D'}, {});

    std::error_code filesystemError;
    if (!outputPath.parent_path().empty()) {
        std::filesystem::create_directories(
            outputPath.parent_path(), filesystemError);
        if (filesystemError) {
            error = "unable to create capture directory: " +
                filesystemError.message();
            return false;
        }
    }
    const std::filesystem::path temporary = outputPath.string() + ".partial";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) {
            error = "unable to open capture output";
            return false;
        }
        output.write(reinterpret_cast<const char*>(png.data()),
                     static_cast<std::streamsize>(png.size()));
        if (!output) {
            error = "unable to write capture output";
            return false;
        }
    }
    std::filesystem::remove(outputPath, filesystemError);
    filesystemError.clear();
    std::filesystem::rename(temporary, outputPath, filesystemError);
    if (filesystemError) {
        std::filesystem::remove(temporary);
        error = "unable to finalize capture: " + filesystemError.message();
        return false;
    }
    return true;
}

} // namespace

VulkanRenderer::VulkanRenderer(SDL_Window* window, bool forceTopologyRegeneration,
                               bool headlessCapture)
    : window_(window), headlessCapture_(headlessCapture),
      forceTopologyRegeneration_(forceTopologyRegeneration) {
    try {
        initialize();
    } catch (...) {
        shutdown();
        throw;
    }
}

VulkanRenderer::~VulkanRenderer() {
    shutdown();
}

bool VulkanRenderer::captureOutputPng(const std::filesystem::path& outputPath,
                                      std::string& error) {
    error.clear();
    if (!outputImageInitialized_) {
        error = "no completed frame is available";
        return false;
    }
    try {
        checkVk(vkWaitForFences(device_, 1, &frameFence_, VK_TRUE,
                                std::numeric_limits<std::uint64_t>::max()),
                "Waiting for capture frame");
        const VkDeviceSize captureBytes =
            static_cast<VkDeviceSize>(swapchainExtent_.width) *
            static_cast<VkDeviceSize>(swapchainExtent_.height) * 4U;
        VkBuffer captureBuffer = VK_NULL_HANDLE;
        VkDeviceMemory captureMemory = VK_NULL_HANDLE;
        const auto cleanup = [&]() {
            if (captureBuffer != VK_NULL_HANDLE) {
                vkDestroyBuffer(device_, captureBuffer, nullptr);
            }
            if (captureMemory != VK_NULL_HANDLE) {
                vkFreeMemory(device_, captureMemory, nullptr);
            }
        };
        try {
            VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
            bufferInfo.size = captureBytes;
            bufferInfo.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
            bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            checkVk(vkCreateBuffer(device_, &bufferInfo, nullptr, &captureBuffer),
                    "Creating capture readback buffer");
            VkMemoryRequirements requirements{};
            vkGetBufferMemoryRequirements(device_, captureBuffer, &requirements);
            VkMemoryAllocateInfo allocation{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
            allocation.allocationSize = requirements.size;
            allocation.memoryTypeIndex = findMemoryType(
                requirements.memoryTypeBits,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            checkVk(vkAllocateMemory(device_, &allocation, nullptr, &captureMemory),
                    "Allocating capture readback memory");
            checkVk(vkBindBufferMemory(device_, captureBuffer, captureMemory, 0U),
                    "Binding capture readback memory");

            checkVk(vkResetFences(device_, 1, &frameFence_),
                    "Resetting capture fence");
            checkVk(vkResetCommandBuffer(commandBuffer_, 0U),
                    "Resetting capture command buffer");
            VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
            checkVk(vkBeginCommandBuffer(commandBuffer_, &begin),
                    "Beginning capture commands");
            VkBufferImageCopy copy{};
            copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            copy.imageSubresource.layerCount = 1U;
            copy.imageExtent = {swapchainExtent_.width, swapchainExtent_.height, 1U};
            vkCmdCopyImageToBuffer(commandBuffer_, outputImage_,
                                   VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                   captureBuffer, 1U, &copy);
            VkBufferMemoryBarrier ready{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
            ready.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            ready.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
            ready.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            ready.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            ready.buffer = captureBuffer;
            ready.size = VK_WHOLE_SIZE;
            vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 VK_PIPELINE_STAGE_HOST_BIT, 0U, 0U, nullptr,
                                 1U, &ready, 0U, nullptr);
            checkVk(vkEndCommandBuffer(commandBuffer_),
                    "Ending capture commands");
            VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            submit.commandBufferCount = 1U;
            submit.pCommandBuffers = &commandBuffer_;
            checkVk(vkQueueSubmit(queue_, 1U, &submit, frameFence_),
                    "Submitting capture commands");
            checkVk(vkWaitForFences(device_, 1, &frameFence_, VK_TRUE,
                                    std::numeric_limits<std::uint64_t>::max()),
                    "Waiting for capture readback");
            void* mapped = nullptr;
            checkVk(vkMapMemory(device_, captureMemory, 0U, captureBytes, 0U,
                                &mapped), "Mapping capture readback");
            const bool written = writePngRgba8(
                outputPath, swapchainExtent_.width, swapchainExtent_.height,
                static_cast<const std::uint8_t*>(mapped), error);
            vkUnmapMemory(device_, captureMemory);
            cleanup();
            return written;
        } catch (...) {
            cleanup();
            throw;
        }
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

void VulkanRenderer::initialize() {
    startupBegin_ = std::chrono::steady_clock::now();
    if constexpr (kPortalLabBuild) {
        if (!headlessCapture_) {
            SDL_ShowWindow(window_);
            SDL_RaiseWindow(window_);
        }
        SDL_SetWindowTitle(window_,
#if VOXEL_INTRINSIC_PORTAL_LAB
#if VOXEL_GLOBAL_METRIC_LAB
            "Global Metric Lab - metric/connection precompute 10%");
#else
            "Intrinsic Ellis Lab - metric/connection precompute 10%");
#endif
#else
            "Voxel Planet Engine - GR wormhole precompute: metric/Christoffel 10%");
#endif
        SDL_PumpEvents();
        const std::filesystem::path grCache =
            std::filesystem::current_path() / "cache" / "portal_gr_ellis_v4.bin";
        SDL_SetWindowTitle(window_,
#if VOXEL_INTRINSIC_PORTAL_LAB
#if VOXEL_GLOBAL_METRIC_LAB
            "Global Metric Lab - all-space geodesic table 35%");
#else
            "Intrinsic Ellis Lab - native geodesic exit table 35%");
#endif
#else
            "Voxel Planet Engine - GR wormhole precompute: geodesics/frames 35%");
#endif
        const PortalGrBuildResult gr = buildPortalGrPrecompute(
            grCache, forceTopologyRegeneration_);
        if (gr.seconds >= 55.0) {
            throw std::runtime_error(
                "Portal GR precompute exceeded the 55 second watchdog");
        }
        portalGrTable_ = gr.table;
        stats_.portalGrPrecomputeReady = true;
        stats_.portalGrCacheHit = gr.cacheHit;
        stats_.portalGrTableBytes = sizeof(PortalGrPrecomputedTable);
        stats_.portalGrPrecomputeSeconds = gr.seconds;
        stats_.portalGrMaximumError = gr.maximumError;
        SDL_SetWindowTitle(window_,
#if VOXEL_INTRINSIC_PORTAL_LAB
#if VOXEL_GLOBAL_METRIC_LAB
            "Global Metric Lab - GPU metric/table upload 80%");
#else
            "Intrinsic Ellis Lab - GPU metric/table upload 80%");
#endif
#else
            "Voxel Planet Engine - GR wormhole precompute: GPU upload 80%");
#endif
        SDL_PumpEvents();
    }
    reportScaleLabStartup("Vulkan setup", 0U, 1U);
#if defined(VOXEL_ENABLE_VALIDATION)
    validationEnabled_ = hasValidationLayer();
    if (!validationEnabled_) {
        std::clog << "Vulkan validation layer is unavailable; continuing without validation.\n";
    }
#endif
    createInstance();
    createDebugMessenger();
    createSurface();
    choosePhysicalDevice();
    createDevice();
    reportScaleLabStartup("Vulkan setup", 1U, 1U);
    createSwapchain();
    createRenderPass();
    createSwapchainViews();
    createOutputImage();
    createVoxelResources();
    createPlanetTopologyResources();
    reportScaleLabStartup("GPU topology upload complete", 1U, 1U);
    createComputeResources();
    createFramebuffers();
    createCommandResources();
    initializeVoxelVolume();
    reportScaleLabStartup("terrain initialization", 1U, 1U);
    createSynchronization();
    createImGuiContext();
    initializeImGuiBackends();
    if constexpr (kPortalLabBuild) {
        SDL_SetWindowTitle(window_,
#if VOXEL_INTRINSIC_PORTAL_LAB
#if VOXEL_GLOBAL_METRIC_LAB
            "Voxel Planet Engine - GLOBAL STATIC SPACETIME Lab - SAME-EXTERIOR SMOOTH HANDLE ATLAS");
#else
            "Voxel Planet Engine - INTRINSIC ELLIS MANIFOLD Lab");
#endif
#else
            "Voxel Planet Engine - STATIC ELLIS GR WORMHOLE PORTAL Lab");
#endif
    }
    stats_.startupSeconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - startupBegin_).count();
    stats_.cpuPeakBytes = processPeakBytes();
    if constexpr (kScaleLabBuild) {
        if (stats_.startupSeconds >= kScaleLabStartupLimitSeconds) {
            throw std::runtime_error("Scale-lab startup guardrail exceeded after initialization: " +
                std::to_string(stats_.startupSeconds) + " s >= 60 s");
        }
        if (stats_.cpuPeakBytes > kScaleLabCpuPeakLimit) {
            throw std::runtime_error("Scale-lab CPU peak guardrail exceeded after initialization");
        }
        std::cout << "Scale-lab startup: " << stats_.startupSeconds << " s, CPU peak "
                  << static_cast<double>(stats_.cpuPeakBytes) /
                         static_cast<double>(1024ULL * 1024ULL * 1024ULL)
                  << " GiB\n";
        reportScaleLabStartup("ready", 1U, 1U);
    } else if constexpr (kFractalPlanetSdfBuild) {
        std::cout << "True fractal planet lab startup: " << stats_.startupSeconds
                  << " s, CPU peak "
                  << static_cast<double>(stats_.cpuPeakBytes) /
                         static_cast<double>(1024ULL * 1024ULL * 1024ULL)
                  << " GiB\n";
        reportScaleLabStartup("ready", 1U, 1U);
    }
}

void VulkanRenderer::reportScaleLabStartup(std::string_view stage,
                                            std::uint64_t current,
                                            std::uint64_t total) {
    if constexpr (!kScaleLabBuild && !kFractalPlanetSdfBuild) {
        return;
    }
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - startupBegin_).count();
    if constexpr (kScaleLabBuild) {
        if (elapsed >= kScaleLabStartupWatchdogSeconds) {
        throw std::runtime_error("Scale-lab startup watchdog stopped stage '" +
            std::string(stage) + "' at " + std::to_string(elapsed) +
            " s; cache was not published and the 60 s wall-clock cap was preserved");
        }
    }
    const std::uint32_t percent = total == 0U ? 0U : static_cast<std::uint32_t>(
        std::min<std::uint64_t>(100U, current * 100U / total));
    if (startupProgressStage_ != stage || startupProgressPercent_ == 0xffffffffU ||
        percent >= startupProgressPercent_ + 5U || percent == 100U) {
        startupProgressStage_ = stage;
        startupProgressPercent_ = percent;
        const std::string prefix = kFractalPlanetSdfBuild
            ? "True fractal planet startup - " : "Scale lab startup - ";
        const std::string message = prefix + std::string(stage) +
            " " + std::to_string(percent) + "% (" + std::to_string(elapsed) + " s)";
        SDL_SetWindowTitle(window_, message.c_str());
        SDL_PumpEvents();
        std::cout << message << '\n';
    }
}

void VulkanRenderer::createInstance() {
    unsigned int sdlExtensionCount = 0;
    if (SDL_Vulkan_GetInstanceExtensions(window_, &sdlExtensionCount, nullptr) != SDL_TRUE) {
        throw std::runtime_error(std::string("SDL could not enumerate Vulkan extensions: ") + SDL_GetError());
    }
    std::vector<const char*> extensions(sdlExtensionCount);
    if (SDL_Vulkan_GetInstanceExtensions(window_, &sdlExtensionCount, extensions.data()) != SDL_TRUE) {
        throw std::runtime_error(std::string("SDL could not provide Vulkan extensions: ") + SDL_GetError());
    }
    if (validationEnabled_) {
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }

    VkApplicationInfo applicationInfo{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    applicationInfo.pApplicationName = "Voxel Planet Engine";
    applicationInfo.applicationVersion = VK_MAKE_API_VERSION(0, 0, 1, 0);
    applicationInfo.pEngineName = "Voxel Planet Engine";
    applicationInfo.engineVersion = VK_MAKE_API_VERSION(0, 0, 1, 0);
    applicationInfo.apiVersion = VK_API_VERSION_1_2;

    VkDebugUtilsMessengerCreateInfoEXT debugInfo = debugMessengerInfo();
    VkInstanceCreateInfo createInfo{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    createInfo.pApplicationInfo = &applicationInfo;
    createInfo.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    createInfo.ppEnabledExtensionNames = extensions.data();
    if (validationEnabled_) {
        createInfo.enabledLayerCount = static_cast<std::uint32_t>(kValidationLayers.size());
        createInfo.ppEnabledLayerNames = kValidationLayers.data();
        createInfo.pNext = &debugInfo;
    }
    checkVk(vkCreateInstance(&createInfo, nullptr, &instance_), "Creating Vulkan instance");
}

void VulkanRenderer::createDebugMessenger() {
    if (!validationEnabled_) {
        return;
    }
    const auto createFunction = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance_, "vkCreateDebugUtilsMessengerEXT"));
    if (createFunction == nullptr) {
        throw std::runtime_error("Vulkan debug utils extension function is unavailable");
    }
    auto info = debugMessengerInfo();
    checkVk(createFunction(instance_, &info, nullptr, &debugMessenger_), "Creating debug messenger");
}

void VulkanRenderer::createSurface() {
    if (SDL_Vulkan_CreateSurface(window_, instance_, &surface_) != SDL_TRUE) {
        throw std::runtime_error(std::string("SDL could not create the Vulkan surface: ") + SDL_GetError());
    }
}

bool VulkanRenderer::supportsRequiredDeviceFeatures(VkPhysicalDevice device, std::uint32_t& family) const {
    std::uint32_t extensionCount = 0;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);
    std::vector<VkExtensionProperties> extensions(extensionCount);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, extensions.data());
    for (const char* required : kDeviceExtensions) {
        const bool found = std::ranges::any_of(extensions, [required](const VkExtensionProperties& extension) {
            return std::strcmp(required, extension.extensionName) == 0;
        });
        if (!found) {
            return false;
        }
    }

    std::uint32_t familyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &familyCount, nullptr);
    std::vector<VkQueueFamilyProperties> families(familyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &familyCount, families.data());
    for (std::uint32_t index = 0; index < familyCount; ++index) {
        VkBool32 canPresent = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, index, surface_, &canPresent);
        const VkQueueFlags requiredFlags = VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT;
        if ((families[index].queueFlags & requiredFlags) == requiredFlags && canPresent == VK_TRUE) {
            family = index;
            return !querySwapchainSupport(device).formats.empty() &&
                   !querySwapchainSupport(device).presentModes.empty();
        }
    }
    return false;
}

void VulkanRenderer::choosePhysicalDevice() {
    std::uint32_t count = 0;
    checkVk(vkEnumeratePhysicalDevices(instance_, &count, nullptr), "Enumerating Vulkan devices");
    if (count == 0) {
        throw std::runtime_error("No Vulkan physical device was found");
    }
    std::vector<VkPhysicalDevice> devices(count);
    checkVk(vkEnumeratePhysicalDevices(instance_, &count, devices.data()), "Reading Vulkan devices");

    int bestScore = -1;
    for (VkPhysicalDevice device : devices) {
        std::uint32_t family = 0;
        if (!supportsRequiredDeviceFeatures(device, family)) {
            continue;
        }
        VkFormatProperties outputFormatProperties{};
        vkGetPhysicalDeviceFormatProperties(device, VK_FORMAT_R8G8B8A8_UNORM, &outputFormatProperties);
        const VkFormatFeatureFlags requiredFormatFeatures =
            VK_FORMAT_FEATURE_STORAGE_IMAGE_BIT | VK_FORMAT_FEATURE_BLIT_SRC_BIT;
        if ((outputFormatProperties.optimalTilingFeatures & requiredFormatFeatures) != requiredFormatFeatures) {
            continue;
        }

        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(device, &properties);
        int score = static_cast<int>(properties.limits.maxImageDimension2D);
        if (properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            score += 100'000;
        }
        if (score > bestScore) {
            bestScore = score;
            physicalDevice_ = device;
            queueFamily_ = family;
            deviceName_ = properties.deviceName;
        }
    }
    if (physicalDevice_ == VK_NULL_HANDLE) {
        throw std::runtime_error("No GPU supports graphics, compute, presentation, and the required image formats");
    }
    stats_.deviceName = deviceName_.c_str();
    stats_.voxelResolution = kVoxelResolution;
    stats_.voxelCount = kVoxelResolution * kVoxelResolution * kVoxelResolution;
    stats_.macrocellCount = kMacrocellResolution * kMacrocellResolution * kMacrocellResolution;

    VkPhysicalDeviceProperties selectedProperties{};
    vkGetPhysicalDeviceProperties(physicalDevice_, &selectedProperties);
    std::uint32_t familyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &familyCount, nullptr);
    std::vector<VkQueueFamilyProperties> families(familyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice_, &familyCount, families.data());
    timestampsSupported_ = queueFamily_ < families.size() &&
                           families[queueFamily_].timestampValidBits > 0U &&
                           selectedProperties.limits.timestampComputeAndGraphics == VK_TRUE;
    timestampValidBits_ = timestampsSupported_ ? families[queueFamily_].timestampValidBits : 0U;
    timestampPeriodNanoseconds_ = selectedProperties.limits.timestampPeriod;
    stats_.gpuTimestampsSupported = timestampsSupported_;
}

void VulkanRenderer::createDevice() {
    constexpr float priority = 1.0F;
    VkDeviceQueueCreateInfo queueInfo{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queueInfo.queueFamilyIndex = queueFamily_;
    queueInfo.queueCount = 1;
    queueInfo.pQueuePriorities = &priority;

    VkPhysicalDeviceFeatures features{};
    VkDeviceCreateInfo createInfo{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    createInfo.queueCreateInfoCount = 1;
    createInfo.pQueueCreateInfos = &queueInfo;
    createInfo.enabledExtensionCount = static_cast<std::uint32_t>(kDeviceExtensions.size());
    createInfo.ppEnabledExtensionNames = kDeviceExtensions.data();
    createInfo.pEnabledFeatures = &features;
    checkVk(vkCreateDevice(physicalDevice_, &createInfo, nullptr, &device_), "Creating Vulkan device");
    vkGetDeviceQueue(device_, queueFamily_, 0, &queue_);
}

VulkanRenderer::SwapchainSupport VulkanRenderer::querySwapchainSupport(VkPhysicalDevice device) const {
    SwapchainSupport support;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface_, &support.capabilities);
    std::uint32_t count = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface_, &count, nullptr);
    support.formats.resize(count);
    if (count > 0) {
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface_, &count, support.formats.data());
    }
    count = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface_, &count, nullptr);
    support.presentModes.resize(count);
    if (count > 0) {
        vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface_, &count, support.presentModes.data());
    }
    return support;
}

void VulkanRenderer::createSwapchain() {
    const SwapchainSupport support = querySwapchainSupport(physicalDevice_);
    const auto formatIterator = std::ranges::find_if(support.formats, [](const VkSurfaceFormatKHR& candidate) {
        return candidate.format == VK_FORMAT_B8G8R8A8_SRGB &&
               candidate.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    });
    const VkSurfaceFormatKHR surfaceFormat = formatIterator != support.formats.end()
                                                       ? *formatIterator
                                                       : support.formats.front();

    VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
    if (std::ranges::find(support.presentModes, VK_PRESENT_MODE_MAILBOX_KHR) != support.presentModes.end()) {
        presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
    }

    VkExtent2D extent = support.capabilities.currentExtent;
    if (extent.width == std::numeric_limits<std::uint32_t>::max()) {
        int drawableWidth = 0;
        int drawableHeight = 0;
        SDL_Vulkan_GetDrawableSize(window_, &drawableWidth, &drawableHeight);
        extent.width = std::clamp(static_cast<std::uint32_t>(std::max(drawableWidth, 1)),
                                  support.capabilities.minImageExtent.width,
                                  support.capabilities.maxImageExtent.width);
        extent.height = std::clamp(static_cast<std::uint32_t>(std::max(drawableHeight, 1)),
                                   support.capabilities.minImageExtent.height,
                                   support.capabilities.maxImageExtent.height);
    }

    constexpr VkImageUsageFlags requiredUsage =
        VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    if ((support.capabilities.supportedUsageFlags & requiredUsage) != requiredUsage) {
        throw std::runtime_error("The presentation surface cannot receive the compute image and ImGui overlay");
    }

    std::uint32_t imageCount = std::max(2U, support.capabilities.minImageCount + 1U);
    if (support.capabilities.maxImageCount > 0) {
        imageCount = std::min(imageCount, support.capabilities.maxImageCount);
    }

    VkSwapchainCreateInfoKHR createInfo{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    createInfo.surface = surface_;
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = surfaceFormat.format;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = extent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = requiredUsage;
    createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    createInfo.preTransform = support.capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = presentMode;
    createInfo.clipped = VK_TRUE;
    checkVk(vkCreateSwapchainKHR(device_, &createInfo, nullptr, &swapchain_), "Creating swapchain");

    checkVk(vkGetSwapchainImagesKHR(device_, swapchain_, &imageCount, nullptr), "Counting swapchain images");
    swapchainImages_.resize(imageCount);
    checkVk(vkGetSwapchainImagesKHR(device_, swapchain_, &imageCount, swapchainImages_.data()),
            "Reading swapchain images");
    swapchainFormat_ = surfaceFormat.format;
    swapchainExtent_ = extent;
    swapchainMinImageCount_ = support.capabilities.minImageCount;
    stats_.width = extent.width;
    stats_.height = extent.height;
    stats_.swapchainImages = imageCount;
}

void VulkanRenderer::createRenderPass() {
    VkAttachmentDescription attachment{};
    attachment.format = swapchainFormat_;
    attachment.samples = VK_SAMPLE_COUNT_1_BIT;
    attachment.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachment.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    attachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference colorReference{};
    colorReference.attachment = 0;
    colorReference.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorReference;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo createInfo{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    createInfo.attachmentCount = 1;
    createInfo.pAttachments = &attachment;
    createInfo.subpassCount = 1;
    createInfo.pSubpasses = &subpass;
    createInfo.dependencyCount = 1;
    createInfo.pDependencies = &dependency;
    checkVk(vkCreateRenderPass(device_, &createInfo, nullptr, &renderPass_), "Creating ImGui render pass");
}

void VulkanRenderer::createSwapchainViews() {
    swapchainViews_.resize(swapchainImages_.size());
    for (std::size_t index = 0; index < swapchainImages_.size(); ++index) {
        VkImageViewCreateInfo createInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        createInfo.image = swapchainImages_[index];
        createInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        createInfo.format = swapchainFormat_;
        createInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        createInfo.subresourceRange.levelCount = 1;
        createInfo.subresourceRange.layerCount = 1;
        checkVk(vkCreateImageView(device_, &createInfo, nullptr, &swapchainViews_[index]),
                "Creating swapchain image view");
    }
}

std::uint32_t VulkanRenderer::findMemoryType(std::uint32_t typeBits, VkMemoryPropertyFlags flags) const {
    VkPhysicalDeviceMemoryProperties properties{};
    vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &properties);
    for (std::uint32_t index = 0; index < properties.memoryTypeCount; ++index) {
        if ((typeBits & (1U << index)) != 0 &&
            (properties.memoryTypes[index].propertyFlags & flags) == flags) {
            return index;
        }
    }
    throw std::runtime_error("No compatible Vulkan memory type was found");
}

void VulkanRenderer::createOutputImage() {
    VkImageCreateInfo imageInfo{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    imageInfo.extent = {swapchainExtent_.width, swapchainExtent_.height, 1};
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    checkVk(vkCreateImage(device_, &imageInfo, nullptr, &outputImage_), "Creating compute output image");

    VkMemoryRequirements requirements{};
    vkGetImageMemoryRequirements(device_, outputImage_, &requirements);
    VkMemoryAllocateInfo allocationInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    allocationInfo.allocationSize = requirements.size;
    allocationInfo.memoryTypeIndex = findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    checkVk(vkAllocateMemory(device_, &allocationInfo, nullptr, &outputMemory_), "Allocating compute output memory");
    checkVk(vkBindImageMemory(device_, outputImage_, outputMemory_, 0), "Binding compute output memory");

    VkImageViewCreateInfo viewInfo{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    viewInfo.image = outputImage_;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = VK_FORMAT_R8G8B8A8_UNORM;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.layerCount = 1;
    checkVk(vkCreateImageView(device_, &viewInfo, nullptr, &outputView_), "Creating compute output view");
    outputImageInitialized_ = false;
}

void VulkanRenderer::createVoxelResources() {
    const auto createBuffer = [this](VkDeviceSize size, VkBuffer& buffer, VkDeviceMemory& memory) {
        VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufferInfo.size = size;
        bufferInfo.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        checkVk(vkCreateBuffer(device_, &bufferInfo, nullptr, &buffer), "Creating voxel storage buffer");

        VkMemoryRequirements requirements{};
        vkGetBufferMemoryRequirements(device_, buffer, &requirements);
        VkMemoryAllocateInfo allocationInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocationInfo.allocationSize = requirements.size;
        allocationInfo.memoryTypeIndex =
            findMemoryType(requirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        checkVk(vkAllocateMemory(device_, &allocationInfo, nullptr, &memory),
                "Allocating voxel storage memory");
        checkVk(vkBindBufferMemory(device_, buffer, memory, 0), "Binding voxel storage memory");
    };

    for (std::size_t index = 0; index < cellBuffers_.size(); ++index) {
        createBuffer(kCellBufferSize, cellBuffers_[index], cellMemories_[index]);
    }
    createBuffer(kOccupancyBufferSize, occupancyBuffer_, occupancyMemory_);
    createBuffer(kCellBufferSize, claimBuffer_, claimMemory_);
}

void VulkanRenderer::createPlanetTopologyResources() {
    if constexpr (kScaleLabBuild) {
        const ScaleLabBudget budget = estimateScaleLabBudget(
            kGeodesicFrequency, kTileBrickResolution, kTileLookupWidth,
            kTileLookupHeight, kResidentGeodesicPages, sizeof(FrameConstants),
            sizeof(GeodesicRayTileGpu),
            static_cast<std::uint64_t>(kFarFieldBytes + kMacroHierarchyBytes));
        std::cout << "Scale-lab preflight: " << budget.report() << '\n';
        if (!budget.accepted()) {
            throw std::runtime_error("Scale-lab preflight refused unsafe allocation: " +
                                     budget.report());
        }
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(physicalDevice_, &properties);
        if (properties.limits.maxStorageBufferRange < budget.gpuMetadataBytes) {
            throw std::runtime_error(
                "Scale-lab preflight refused: device maxStorageBufferRange is below the "
                "dense metadata addressing requirement");
        }
        if (properties.limits.maxPushConstantsSize < sizeof(FrameConstants)) {
            throw std::runtime_error(
                "Scale-lab preflight refused: device push-constant limit is below 128 bytes");
        }
    }
    const auto topologyBegin = std::chrono::steady_clock::now();
    const auto progress = [this](std::string_view stage, std::uint64_t current,
                                 std::uint64_t total) {
        reportScaleLabStartup(stage, current, total);
    };
    if constexpr (kScaleLabBuild) {
        const GeodesicTopologyCacheConfig cacheConfig{
            kGeodesicFrequency, kTileBrickResolution, kTileLookupWidth,
            kTileLookupHeight, kResidentGeodesicPages, kReferenceTraversalEnabled};
        const std::filesystem::path cachePath = scaleLabTopologyCachePath();
        std::optional<GeodesicTopologyCacheLoad> cached;
        if (!forceTopologyRegeneration_) {
            reportScaleLabStartup("topology cache validation", 0U, 1U);
            cached = loadGeodesicTopologyCache(cachePath, cacheConfig, progress);
        }
        if (cached.has_value()) {
            planetTopology_ = std::move(cached->topology);
            stats_.topologyCacheHit = true;
            stats_.topologyCacheBytes = cached->fileBytes;
            reportScaleLabStartup("topology cache validated", 1U, 1U);
        } else {
            reportScaleLabStartup(forceTopologyRegeneration_
                ? "forced topology regeneration" : "cache missing/invalid; regenerating",
                0U, 1U);
            planetTopology_ = GeodesicTopology::build(
                kGeodesicFrequency, kTileBrickResolution, kTileLookupWidth,
                kTileLookupHeight, kResidentGeodesicPages,
                kReferenceTraversalEnabled, progress);
            reportScaleLabStartup("topology cache write", 0U, 1U);
            stats_.topologyCacheBytes = writeGeodesicTopologyCache(
                cachePath, cacheConfig, planetTopology_, progress);
            reportScaleLabStartup("topology cache published", 1U, 1U);
        }
        const double topologySeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - topologyBegin).count();
        stats_.topologyCacheSeconds = topologySeconds;
        const std::uint64_t peak = processPeakBytes();
        std::cout << "Scale-lab topology/cache: " << topologySeconds << " s ("
                  << (stats_.topologyCacheHit ? "warm" : "cold") << "), CPU peak "
                  << static_cast<double>(peak) /
                         static_cast<double>(1024ULL * 1024ULL * 1024ULL)
                  << " GiB\n";
        if (peak > kScaleLabCpuPeakLimit) {
            throw std::runtime_error(
                "Scale-lab topology exceeded CPU guardrail; refusing GPU allocation");
        }
        reportScaleLabStartup("GPU topology allocation/upload", 0U, 1U);
    } else {
        if constexpr (kFractalPlanetSdfBuild) {
            reportScaleLabStartup("geodesic topology generation", 0U, 1U);
        }
        if constexpr (kFractalPlanetSdfBuild) {
            planetTopology_ = GeodesicTopology::build(
                kGeodesicFrequency, kTileBrickResolution, kTileLookupWidth,
                kTileLookupHeight, kResidentGeodesicPages,
                kReferenceTraversalEnabled, progress);
        } else {
            planetTopology_ = GeodesicTopology::build(
                kGeodesicFrequency, kTileBrickResolution, kTileLookupWidth,
                kTileLookupHeight, kResidentGeodesicPages,
                kReferenceTraversalEnabled);
        }
        if constexpr (kFractalPlanetSdfBuild) {
            reportScaleLabStartup("geodesic topology generation", 1U, 1U);
            reportScaleLabStartup("GPU topology allocation/upload", 0U, 1U);
        }
    }
    freshCollisionColumns_.assign(planetTopology_.tiles.size(), 0U);

    const VkDeviceSize tileBytes = static_cast<VkDeviceSize>(
        planetTopology_.tiles.size() *
        (kScaleLabBuild ? sizeof(GeodesicRayTileGpu) : sizeof(GeodesicTileGpu)));
    const VkDeviceSize lookupBytes =
        static_cast<VkDeviceSize>(planetTopology_.directionLookup.size() * sizeof(std::uint32_t));
    const VkDeviceSize brickBytes =
        static_cast<VkDeviceSize>(planetTopology_.residentBrickCellCapacity() * sizeof(std::uint32_t));
    const VkDeviceSize bvhBytes =
        static_cast<VkDeviceSize>(planetTopology_.bvhNodes.size() * sizeof(GeodesicBvhNodeGpu));
    const VkDeviceSize wideNodeBytes = static_cast<VkDeviceSize>(
        planetTopology_.wideNodes.size() * sizeof(GeodesicWideNodeGpu));
    const VkDeviceSize wideItemBytes = static_cast<VkDeviceSize>(
        planetTopology_.wideItems.size() * sizeof(std::uint32_t));
    const VkDeviceSize columnStateBytes = static_cast<VkDeviceSize>(
        planetTopology_.columnStates.size() * sizeof(GeodesicColumnStateGpu));
    const VkDeviceSize pageTableBytes = static_cast<VkDeviceSize>(
        planetTopology_.pageTable.size() * sizeof(GeodesicPageTableEntryGpu));
    const VkDeviceSize pageRequestBytes = sizeof(PageRequestFeedbackHeader) +
        static_cast<VkDeviceSize>(kPageRequestCapacity) * sizeof(PageRequestGpu);
    const VkDeviceSize artifactDiagnosticBytes = sizeof(ArtifactDiagnosticHeaderGpu) +
        static_cast<VkDeviceSize>(kArtifactDiagnosticCapacity) *
            sizeof(ArtifactDiagnosticRecordGpu);
    const VkDeviceSize streamUploadBytes = static_cast<VkDeviceSize>(kPageStreamBudget) *
        (static_cast<VkDeviceSize>(planetTopology_.brickResolution) * sizeof(std::uint32_t) +
         2U * sizeof(GeodesicPageTableEntryGpu));
    VkDeviceSize eightPlanetHierarchyBytes = 0U;
    VkDeviceSize eightPlanetAuthorityBytes = 0U;
#if VOXEL_EIGHT_PLANET_SYSTEM_LAB
    if constexpr (kEightPlanetSystemLabBuild) {
        eightPlanetHierarchy_ = system_lab::buildSharedLodHierarchy(planetTopology_);
        eightPlanetHierarchyWords_ = packSystemHierarchy(eightPlanetHierarchy_);
        eightPlanetAuthorityWords_ = packSystemAuthorities(
            planetTopology_, eightPlanetHierarchy_, system_lab::Settings{});
        eightPlanetAuthoritySeeds_ = system_lab::Settings{}.planetSeeds;
        eightPlanetHierarchyBytes = static_cast<VkDeviceSize>(
            eightPlanetHierarchyWords_.size() * sizeof(std::uint32_t));
        eightPlanetAuthorityBytes = static_cast<VkDeviceSize>(
            eightPlanetAuthorityWords_.size() * sizeof(std::uint32_t));
        const std::uint64_t topologyBytes =
            planetTopology_.tiles.size() * sizeof(GeodesicTileGpu) +
            planetTopology_.directionLookup.size() * sizeof(std::uint32_t);
        const std::uint64_t totalBytes = topologyBytes + eightPlanetHierarchyBytes +
            eightPlanetAuthorityBytes;
        stats_.systemImmutableTopologyBytes = topologyBytes;
        stats_.systemSharedHierarchyBytes = eightPlanetHierarchyBytes;
        stats_.systemAuthorityBytes = eightPlanetAuthorityBytes;
        std::cout << "Eight-planet authority memory: immutable topology "
                  << topologyBytes / (1024U * 1024U) << " MiB, shared LOD "
                  << eightPlanetHierarchyBytes / (1024U * 1024U)
                  << " MiB, eight sparse authorities "
                  << eightPlanetAuthorityBytes / (1024U * 1024U)
                  << " MiB, total " << totalBytes / (1024U * 1024U)
                  << " MiB\n";
        if (totalBytes > system_lab::kSystemAuthorityMemoryGuardrail) {
            throw std::runtime_error(
                "Eight-planet authority memory guardrail exceeded");
        }
    }
#endif

    const auto createBuffer = [this](VkDeviceSize size, VkBufferUsageFlags usage,
                                     VkMemoryPropertyFlags memoryFlags,
                                     VkBuffer& buffer, VkDeviceMemory& memory) {
        VkBufferCreateInfo bufferInfo{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        checkVk(vkCreateBuffer(device_, &bufferInfo, nullptr, &buffer), "Creating planet topology buffer");

        VkMemoryRequirements requirements{};
        vkGetBufferMemoryRequirements(device_, buffer, &requirements);
        VkMemoryAllocateInfo allocationInfo{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        allocationInfo.allocationSize = requirements.size;
        allocationInfo.memoryTypeIndex = findMemoryType(requirements.memoryTypeBits, memoryFlags);
        checkVk(vkAllocateMemory(device_, &allocationInfo, nullptr, &memory),
                "Allocating planet topology memory");
        checkVk(vkBindBufferMemory(device_, buffer, memory, 0), "Binding planet topology memory");
    };

    createBuffer(tileBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, geodesicTileBuffer_, geodesicTileMemory_);
    createBuffer(lookupBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, tileLookupBuffer_, tileLookupMemory_);
    createBuffer(brickBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, tileBrickBuffer_, tileBrickMemory_);
    if (kReferenceTraversalEnabled) {
        createBuffer(bvhBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, geodesicBvhBuffer_, geodesicBvhMemory_);
        createBuffer(wideNodeBytes,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                     geodesicWideNodeBuffer_, geodesicWideNodeMemory_);
        createBuffer(wideItemBytes,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                     geodesicWideItemBuffer_, geodesicWideItemMemory_);
    }
    createBuffer(columnStateBytes,
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                 geodesicColumnStateBuffer_, geodesicColumnStateMemory_);
    createBuffer(pageTableBytes,
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                 geodesicPageTableBuffer_, geodesicPageTableMemory_);
    createBuffer(pageRequestBytes,
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 geodesicPageRequestBuffer_, geodesicPageRequestMemory_);
    createBuffer(kFarFieldBytes,
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                 geodesicFarFieldBuffer_, geodesicFarFieldMemory_);
    if constexpr (kScaleLabBuild) {
        createBuffer(kMacroHierarchyBytes,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                         VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                     geodesicMacroHierarchyBuffer_,
                     geodesicMacroHierarchyMemory_);
    }
    if constexpr (kEightPlanetSystemLabBuild) {
        createBuffer(eightPlanetHierarchyBytes,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     eightPlanetHierarchyBuffer_, eightPlanetHierarchyMemory_);
        void* hierarchyMapped = nullptr;
        checkVk(vkMapMemory(device_, eightPlanetHierarchyMemory_, 0,
                            eightPlanetHierarchyBytes, 0, &hierarchyMapped),
                "Mapping shared eight-planet LOD hierarchy");
        std::memcpy(hierarchyMapped, eightPlanetHierarchyWords_.data(),
                    static_cast<std::size_t>(eightPlanetHierarchyBytes));
        vkUnmapMemory(device_, eightPlanetHierarchyMemory_);
        createBuffer(eightPlanetAuthorityBytes,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     eightPlanetAuthorityBuffer_, eightPlanetAuthorityMemory_);
        checkVk(vkMapMemory(device_, eightPlanetAuthorityMemory_, 0,
                            eightPlanetAuthorityBytes, 0,
                            &eightPlanetAuthorityMapped_),
                "Mapping eight independent planet authorities");
        std::memcpy(eightPlanetAuthorityMapped_,
                    eightPlanetAuthorityWords_.data(),
                    static_cast<std::size_t>(eightPlanetAuthorityBytes));
    }
    checkVk(vkMapMemory(device_, geodesicPageRequestMemory_, 0, pageRequestBytes, 0,
                        &geodesicPageRequestMapped_),
            "Mapping geodesic page-request feedback");
    std::memset(geodesicPageRequestMapped_, 0, static_cast<std::size_t>(pageRequestBytes));
    createBuffer(artifactDiagnosticBytes,
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                     VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 artifactDiagnosticBuffer_, artifactDiagnosticMemory_);
    checkVk(vkMapMemory(device_, artifactDiagnosticMemory_, 0,
                        artifactDiagnosticBytes, 0, &artifactDiagnosticMapped_),
            "Mapping artifact diagnostic feedback");
    std::memset(artifactDiagnosticMapped_, 0,
                static_cast<std::size_t>(artifactDiagnosticBytes));
    if constexpr (kPortalLabBuild) {
        createBuffer(sizeof(PortalLabGpuBuffer),
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     portalLabParameterBuffer_, portalLabParameterMemory_);
        checkVk(vkMapMemory(device_, portalLabParameterMemory_, 0,
                            sizeof(PortalLabGpuBuffer), 0,
                            &portalLabParameterMapped_),
                "Mapping portal-lab parameters");
#if VOXEL_INTRINSIC_PORTAL_LAB
        const IntrinsicEllisSettings intrinsicSettings{};
        const IntrinsicEllisGpuBuffer defaults{
            intrinsicEllisGpuParameters(
                intrinsicSettings, intrinsicSettings.cameraState,
                intrinsicSettings.cameraFrame),
            portalGrTable_};
#else
        const PortalLabGpuBuffer defaults{portalGpuParameters({}), portalGrTable_};
#endif
        std::memcpy(portalLabParameterMapped_, &defaults, sizeof(defaults));
    }
    createBuffer(streamUploadBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 geodesicStreamUploadBuffer_, geodesicStreamUploadMemory_);
    checkVk(vkMapMemory(device_, geodesicStreamUploadMemory_, 0, streamUploadBytes, 0,
                        &geodesicStreamUploadMapped_),
            "Mapping geodesic stream upload buffer");

    physicalPageOwners_.assign(planetTopology_.residentPageCount(), kInvalidTile);
    for (std::uint32_t tileIndex = 0; tileIndex < planetTopology_.pageTable.size(); ++tileIndex) {
        const auto& page = planetTopology_.pageTable[tileIndex].data;
        if ((page[2] & kPageResident) != 0U) {
            physicalPageOwners_[page[0]] = tileIndex;
        }
    }

    const VkDeviceSize stagingBytes = tileBytes + lookupBytes + brickBytes + bvhBytes +
        wideNodeBytes + wideItemBytes + columnStateBytes + pageTableBytes;
    VkBuffer stagingBuffer = VK_NULL_HANDLE;
    VkDeviceMemory stagingMemory = VK_NULL_HANDLE;
    createBuffer(stagingBytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 stagingBuffer, stagingMemory);
    void* mapped = nullptr;
    checkVk(vkMapMemory(device_, stagingMemory, 0, stagingBytes, 0, &mapped),
            "Mapping planet topology upload");
    if constexpr (kScaleLabBuild) {
        reportScaleLabStartup("compact exact ray-query records", 0U,
                              planetTopology_.tiles.size());
        auto* compactTiles = static_cast<GeodesicRayTileGpu*>(mapped);
        for (std::size_t tileIndex = 0U;
             tileIndex < planetTopology_.tiles.size(); ++tileIndex) {
            const GeodesicTileGpu& source = planetTopology_.tiles[tileIndex];
            compactTiles[tileIndex] = compactGeodesicRayTile(source);
            if ((tileIndex & 0x3ffffU) == 0U) {
                reportScaleLabStartup("compact exact ray-query records",
                                      tileIndex, planetTopology_.tiles.size());
            }
        }
        reportScaleLabStartup("compact exact ray-query records",
                              planetTopology_.tiles.size(),
                              planetTopology_.tiles.size());
    } else {
        std::memcpy(mapped, planetTopology_.tiles.data(),
                    static_cast<std::size_t>(tileBytes));
    }
    std::memcpy(static_cast<std::byte*>(mapped) + tileBytes,
                planetTopology_.directionLookup.data(), static_cast<std::size_t>(lookupBytes));
    std::memcpy(static_cast<std::byte*>(mapped) + tileBytes + lookupBytes,
                planetTopology_.brickCells.data(), static_cast<std::size_t>(brickBytes));
    if (kReferenceTraversalEnabled) {
        std::memcpy(static_cast<std::byte*>(mapped) + tileBytes + lookupBytes + brickBytes,
                    planetTopology_.bvhNodes.data(), static_cast<std::size_t>(bvhBytes));
        std::memcpy(static_cast<std::byte*>(mapped) + tileBytes + lookupBytes + brickBytes + bvhBytes,
                    planetTopology_.wideNodes.data(), static_cast<std::size_t>(wideNodeBytes));
        std::memcpy(static_cast<std::byte*>(mapped) + tileBytes + lookupBytes + brickBytes + bvhBytes +
                        wideNodeBytes,
                    planetTopology_.wideItems.data(), static_cast<std::size_t>(wideItemBytes));
    }
    std::memcpy(static_cast<std::byte*>(mapped) + tileBytes + lookupBytes + brickBytes + bvhBytes +
                    wideNodeBytes + wideItemBytes,
                planetTopology_.columnStates.data(), static_cast<std::size_t>(columnStateBytes));
    std::memcpy(static_cast<std::byte*>(mapped) + tileBytes + lookupBytes + brickBytes + bvhBytes +
                    wideNodeBytes + wideItemBytes + columnStateBytes,
                planetTopology_.pageTable.data(), static_cast<std::size_t>(pageTableBytes));
    vkUnmapMemory(device_, stagingMemory);

    VkCommandPool uploadPool = VK_NULL_HANDLE;
    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    poolInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
    poolInfo.queueFamilyIndex = queueFamily_;
    checkVk(vkCreateCommandPool(device_, &poolInfo, nullptr, &uploadPool),
            "Creating planet topology upload pool");
    VkCommandBuffer uploadCommands = VK_NULL_HANDLE;
    VkCommandBufferAllocateInfo commandInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    commandInfo.commandPool = uploadPool;
    commandInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    commandInfo.commandBufferCount = 1;
    checkVk(vkAllocateCommandBuffers(device_, &commandInfo, &uploadCommands),
            "Allocating planet topology upload commands");
    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    checkVk(vkBeginCommandBuffer(uploadCommands, &beginInfo), "Beginning planet topology upload");
    VkBufferCopy tileCopy{0, 0, tileBytes};
    vkCmdCopyBuffer(uploadCommands, stagingBuffer, geodesicTileBuffer_, 1, &tileCopy);
    VkBufferCopy lookupCopy{tileBytes, 0, lookupBytes};
    vkCmdCopyBuffer(uploadCommands, stagingBuffer, tileLookupBuffer_, 1, &lookupCopy);
    VkBufferCopy brickCopy{tileBytes + lookupBytes, 0, brickBytes};
    vkCmdCopyBuffer(uploadCommands, stagingBuffer, tileBrickBuffer_, 1, &brickCopy);
    if (kReferenceTraversalEnabled) {
        VkBufferCopy bvhCopy{tileBytes + lookupBytes + brickBytes, 0, bvhBytes};
        vkCmdCopyBuffer(uploadCommands, stagingBuffer, geodesicBvhBuffer_, 1, &bvhCopy);
        VkBufferCopy wideNodeCopy{
            tileBytes + lookupBytes + brickBytes + bvhBytes, 0, wideNodeBytes};
        vkCmdCopyBuffer(uploadCommands, stagingBuffer, geodesicWideNodeBuffer_, 1, &wideNodeCopy);
        VkBufferCopy wideItemCopy{
            tileBytes + lookupBytes + brickBytes + bvhBytes + wideNodeBytes, 0, wideItemBytes};
        vkCmdCopyBuffer(uploadCommands, stagingBuffer, geodesicWideItemBuffer_, 1, &wideItemCopy);
    }
    VkBufferCopy columnStateCopy{
        tileBytes + lookupBytes + brickBytes + bvhBytes + wideNodeBytes + wideItemBytes,
        0, columnStateBytes};
    vkCmdCopyBuffer(uploadCommands, stagingBuffer, geodesicColumnStateBuffer_, 1,
                    &columnStateCopy);
    VkBufferCopy pageTableCopy{
        tileBytes + lookupBytes + brickBytes + bvhBytes + wideNodeBytes + wideItemBytes +
            columnStateBytes,
        0, pageTableBytes};
    vkCmdCopyBuffer(uploadCommands, stagingBuffer, geodesicPageTableBuffer_, 1,
                    &pageTableCopy);
    checkVk(vkEndCommandBuffer(uploadCommands), "Ending planet topology upload");
    VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &uploadCommands;
    checkVk(vkQueueSubmit(queue_, 1, &submitInfo, VK_NULL_HANDLE), "Submitting planet topology upload");
    checkVk(vkQueueWaitIdle(queue_), "Waiting for planet topology upload");
    vkDestroyCommandPool(device_, uploadPool, nullptr);
    vkDestroyBuffer(device_, stagingBuffer, nullptr);
    vkFreeMemory(device_, stagingMemory, nullptr);
    reportScaleLabStartup("GPU topology allocation/upload", 1U, 1U);

    stats_.geodesicFrequency = planetTopology_.frequency;
    stats_.surfaceTileCount = static_cast<std::uint32_t>(planetTopology_.tiles.size());
    stats_.pentagonCount = planetTopology_.pentagonCount;
    stats_.hexagonCount = planetTopology_.hexagonCount;
    stats_.residentTileBricks = planetTopology_.residentPageCount();
    stats_.tileBrickResolution = planetTopology_.brickResolution;
    double surfaceWidthSum = 0.0;
    double radialLayerHeightSum = 0.0;
    for (const GeodesicTileGpu& tile : planetTopology_.tiles) {
        surfaceWidthSum += static_cast<double>(tile.center[3]) * 2.0;
        radialLayerHeightSum += static_cast<double>(tile.tangent[3]) /
                                static_cast<double>(planetTopology_.brickResolution);
    }
    const double inverseTileCount = 1.0 / static_cast<double>(planetTopology_.tiles.size());
    stats_.averageSurfaceCellWidth = static_cast<float>(surfaceWidthSum * inverseTileCount);
    stats_.averageRadialLayerHeight =
        static_cast<float>(radialLayerHeightSum * inverseTileCount);
    stats_.radialHeightWidthRatio = stats_.averageRadialLayerHeight /
                                    stats_.averageSurfaceCellWidth;
    stats_.tileBrickCellCapacity = planetTopology_.brickCellCapacity();
    stats_.residentBrickCellCapacity = planetTopology_.residentBrickCellCapacity();
    stats_.planetOuterScale = planetTopology_.outerBoundingRadius;
    stats_.geodesicBvhNodeCount = static_cast<std::uint32_t>(planetTopology_.bvhNodes.size());
    stats_.geodesicWideNodeCount = static_cast<std::uint32_t>(planetTopology_.wideNodes.size());
}

void VulkanRenderer::createComputeResources() {
    std::array<VkDescriptorSetLayoutBinding, 18> bindings{};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bindings[2] = bindings[1];
    bindings[2].binding = 2;
    bindings[3] = bindings[1];
    bindings[3].binding = 3;
    bindings[4] = bindings[1];
    bindings[4].binding = 4;
    bindings[5] = bindings[1];
    bindings[5].binding = 5;
    bindings[6] = bindings[1];
    bindings[6].binding = 6;
    bindings[7] = bindings[1];
    bindings[7].binding = 7;
    bindings[8] = bindings[1];
    bindings[8].binding = 8;
    bindings[9] = bindings[1];
    bindings[9].binding = 9;
    bindings[10] = bindings[1];
    bindings[10].binding = 10;
    bindings[11] = bindings[1];
    bindings[11].binding = 11;
    bindings[12] = bindings[1];
    bindings[12].binding = 12;
    bindings[13] = bindings[1];
    bindings[13].binding = 13;
    bindings[14] = bindings[1];
    bindings[14].binding = 14;
    bindings[15] = bindings[1];
    bindings[15].binding = 15;
    bindings[16] = bindings[1];
    bindings[16].binding = 16;
    bindings[17] = bindings[1];
    bindings[17].binding = 17;
    VkDescriptorSetLayoutCreateInfo layoutInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layoutInfo.bindingCount = static_cast<std::uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    checkVk(vkCreateDescriptorSetLayout(device_, &layoutInfo, nullptr, &computeDescriptorSetLayout_),
            "Creating compute descriptor layout");

    const std::array<VkDescriptorPoolSize, 2> poolSizes{{
        {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 2},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 34},
    }};
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.maxSets = 2;
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    checkVk(vkCreateDescriptorPool(device_, &poolInfo, nullptr, &computeDescriptorPool_),
            "Creating compute descriptor pool");
    VkDescriptorSetAllocateInfo allocationInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    allocationInfo.descriptorPool = computeDescriptorPool_;
    const std::array layouts{computeDescriptorSetLayout_, computeDescriptorSetLayout_};
    allocationInfo.descriptorSetCount = static_cast<std::uint32_t>(layouts.size());
    allocationInfo.pSetLayouts = layouts.data();
    checkVk(vkAllocateDescriptorSets(device_, &allocationInfo, computeDescriptorSets_.data()),
            "Allocating compute descriptor set");

    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pushConstantRange.size = sizeof(FrameConstants);
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &computeDescriptorSetLayout_;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;
    checkVk(vkCreatePipelineLayout(device_, &pipelineLayoutInfo, nullptr, &computePipelineLayout_),
            "Creating compute pipeline layout");

    const auto createComputePipeline = [this](const char* filename, VkPipeline& pipeline) {
        const auto shaderBytes = readBinaryFile(std::filesystem::path(VOXEL_SHADER_DIR) / filename);
        VkShaderModuleCreateInfo shaderInfo{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        shaderInfo.codeSize = shaderBytes.size();
        shaderInfo.pCode = reinterpret_cast<const std::uint32_t*>(shaderBytes.data());
        VkShaderModule shaderModule = VK_NULL_HANDLE;
        checkVk(vkCreateShaderModule(device_, &shaderInfo, nullptr, &shaderModule),
                "Creating compute shader module");
        VkComputePipelineCreateInfo pipelineInfo{VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
        pipelineInfo.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        pipelineInfo.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        pipelineInfo.stage.module = shaderModule;
        pipelineInfo.stage.pName = "main";
        pipelineInfo.layout = computePipelineLayout_;
        const VkResult result =
            vkCreateComputePipelines(device_, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &pipeline);
        vkDestroyShaderModule(device_, shaderModule, nullptr);
        checkVk(result, "Creating compute pipeline");
    };
    createComputePipeline("planet.comp.spv", computePipeline_);
    if constexpr (kGlobalMetricLabBuild) {
        createComputePipeline("planet_spatial_aa.comp.spv",
                              globalSpatialAaPipeline_);
        createComputePipeline("planet_spatial_aa_resolve.comp.spv",
                              globalSpatialAaResolvePipeline_);
    }
    createComputePipeline("voxel_init.comp.spv", voxelInitPipeline_);
    createComputePipeline("voxel_edit.comp.spv", voxelEditPipeline_);
    createComputePipeline("geodesic_edit.comp.spv", geodesicEditPipeline_);
    createComputePipeline("geodesic_terrain_init.comp.spv", geodesicTerrainInitPipeline_);
    createComputePipeline("geodesic_far_field_init.comp.spv", geodesicFarFieldInitPipeline_);
    createComputePipeline("geodesic_stream_requests.comp.spv", geodesicStreamRequestPipeline_);
    createComputePipeline("sand_claim.comp.spv", sandClaimPipeline_);
    createComputePipeline("sand_commit.comp.spv", sandCommitPipeline_);

    VkDescriptorImageInfo imageInfo{};
    imageInfo.imageView = outputView_;
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    VkDescriptorBufferInfo occupancyInfo{occupancyBuffer_, 0, kOccupancyBufferSize};
    VkDescriptorBufferInfo claimInfo{claimBuffer_, 0, kCellBufferSize};
    const VkDeviceSize tileBytes = static_cast<VkDeviceSize>(
        planetTopology_.tiles.size() *
        (kScaleLabBuild ? sizeof(GeodesicRayTileGpu) : sizeof(GeodesicTileGpu)));
    const VkDeviceSize lookupBytes =
        static_cast<VkDeviceSize>(planetTopology_.directionLookup.size() * sizeof(std::uint32_t));
    const VkDeviceSize brickBytes =
        static_cast<VkDeviceSize>(planetTopology_.residentBrickCellCapacity() * sizeof(std::uint32_t));
    const VkDeviceSize bvhBytes =
        static_cast<VkDeviceSize>(planetTopology_.bvhNodes.size() * sizeof(GeodesicBvhNodeGpu));
    const VkDeviceSize wideNodeBytes = static_cast<VkDeviceSize>(
        planetTopology_.wideNodes.size() * sizeof(GeodesicWideNodeGpu));
    const VkDeviceSize wideItemBytes = static_cast<VkDeviceSize>(
        planetTopology_.wideItems.size() * sizeof(std::uint32_t));
    const VkDeviceSize columnStateBytes = static_cast<VkDeviceSize>(
        planetTopology_.columnStates.size() * sizeof(GeodesicColumnStateGpu));
    const VkDeviceSize pageTableBytes = static_cast<VkDeviceSize>(
        planetTopology_.pageTable.size() * sizeof(GeodesicPageTableEntryGpu));
    const VkDeviceSize pageRequestBytes = sizeof(PageRequestFeedbackHeader) +
        static_cast<VkDeviceSize>(kPageRequestCapacity) * sizeof(PageRequestGpu);
    const VkDeviceSize artifactDiagnosticBytes = sizeof(ArtifactDiagnosticHeaderGpu) +
        static_cast<VkDeviceSize>(kArtifactDiagnosticCapacity) *
            sizeof(ArtifactDiagnosticRecordGpu);
    VkDescriptorBufferInfo tileInfo{geodesicTileBuffer_, 0, tileBytes};
    VkDescriptorBufferInfo lookupInfo{tileLookupBuffer_, 0, lookupBytes};
    VkDescriptorBufferInfo tileBrickInfo{tileBrickBuffer_, 0, brickBytes};
    // Bind a valid existing buffer to statically unused diagnostic bindings in
    // the production DDA shader. This keeps one descriptor layout for every
    // compute pipeline without allocating the reference acceleration data.
    VkDescriptorBufferInfo bvhInfo{
        kReferenceTraversalEnabled ? geodesicBvhBuffer_ : geodesicTileBuffer_, 0,
        kReferenceTraversalEnabled ? bvhBytes : tileBytes};
    VkDescriptorBufferInfo wideNodeInfo{
        kReferenceTraversalEnabled ? geodesicWideNodeBuffer_ : geodesicTileBuffer_, 0,
        kReferenceTraversalEnabled ? wideNodeBytes : tileBytes};
    VkDescriptorBufferInfo wideItemInfo{
        kReferenceTraversalEnabled ? geodesicWideItemBuffer_ : geodesicTileBuffer_, 0,
        kReferenceTraversalEnabled ? wideItemBytes : tileBytes};
    VkDescriptorBufferInfo columnStateInfo{geodesicColumnStateBuffer_, 0, columnStateBytes};
    VkDescriptorBufferInfo pageTableInfo{geodesicPageTableBuffer_, 0, pageTableBytes};
    VkDescriptorBufferInfo pageRequestInfo{geodesicPageRequestBuffer_, 0, pageRequestBytes};
    VkDescriptorBufferInfo artifactDiagnosticInfo{
        artifactDiagnosticBuffer_, 0, artifactDiagnosticBytes};
    VkDescriptorBufferInfo farFieldInfo{geodesicFarFieldBuffer_, 0, kFarFieldBytes};
    VkDescriptorBufferInfo macroHierarchyInfo{
        kScaleLabBuild ? geodesicMacroHierarchyBuffer_
                       : kEightPlanetSystemLabBuild ? eightPlanetHierarchyBuffer_
                                                    : geodesicTileBuffer_,
        0,
        kScaleLabBuild ? kMacroHierarchyBytes
                       : kEightPlanetSystemLabBuild
                             ? static_cast<VkDeviceSize>(
                                   eightPlanetHierarchyWords_.size() *
                                   sizeof(std::uint32_t))
                             : tileBytes};
    VkDescriptorBufferInfo portalParameterInfo{
        kPortalLabBuild ? portalLabParameterBuffer_
                        : kEightPlanetSystemLabBuild ? eightPlanetAuthorityBuffer_
                                                     : geodesicTileBuffer_,
        0,
        kPortalLabBuild ? static_cast<VkDeviceSize>(sizeof(PortalLabGpuBuffer))
                        : kEightPlanetSystemLabBuild
                              ? static_cast<VkDeviceSize>(
                                    eightPlanetAuthorityWords_.size() *
                                    sizeof(std::uint32_t))
                              : tileBytes};
    for (std::uint32_t setIndex = 0; setIndex < computeDescriptorSets_.size(); ++setIndex) {
        VkDescriptorBufferInfo sourceInfo{cellBuffers_[setIndex], 0, kCellBufferSize};
        VkDescriptorBufferInfo destinationInfo{cellBuffers_[1U - setIndex], 0, kCellBufferSize};
        std::array<VkWriteDescriptorSet, 18> writes{};
        for (auto& write : writes) {
            write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            write.dstSet = computeDescriptorSets_[setIndex];
            write.descriptorCount = 1;
        }
        writes[0].dstBinding = 0;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        writes[0].pImageInfo = &imageInfo;
        writes[1].dstBinding = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[1].pBufferInfo = &sourceInfo;
        writes[2].dstBinding = 2;
        writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[2].pBufferInfo = &occupancyInfo;
        writes[3].dstBinding = 3;
        writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[3].pBufferInfo = &destinationInfo;
        writes[4].dstBinding = 4;
        writes[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[4].pBufferInfo = &claimInfo;
        writes[5].dstBinding = 5;
        writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[5].pBufferInfo = &tileInfo;
        writes[6].dstBinding = 6;
        writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[6].pBufferInfo = &lookupInfo;
        writes[7].dstBinding = 7;
        writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[7].pBufferInfo = &tileBrickInfo;
        writes[8].dstBinding = 8;
        writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[8].pBufferInfo = &bvhInfo;
        writes[9].dstBinding = 9;
        writes[9].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[9].pBufferInfo = &columnStateInfo;
        writes[10].dstBinding = 10;
        writes[10].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[10].pBufferInfo = &pageTableInfo;
        writes[11].dstBinding = 11;
        writes[11].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[11].pBufferInfo = &pageRequestInfo;
        writes[12].dstBinding = 12;
        writes[12].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[12].pBufferInfo = &wideNodeInfo;
        writes[13].dstBinding = 13;
        writes[13].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[13].pBufferInfo = &wideItemInfo;
        writes[14].dstBinding = 14;
        writes[14].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[14].pBufferInfo = &artifactDiagnosticInfo;
        writes[15].dstBinding = 15;
        writes[15].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[15].pBufferInfo = &farFieldInfo;
        writes[16].dstBinding = 16;
        writes[16].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[16].pBufferInfo = &macroHierarchyInfo;
        writes[17].dstBinding = 17;
        writes[17].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[17].pBufferInfo = &portalParameterInfo;
        vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }
}

void VulkanRenderer::createFramebuffers() {
    framebuffers_.resize(swapchainViews_.size());
    for (std::size_t index = 0; index < swapchainViews_.size(); ++index) {
        VkFramebufferCreateInfo createInfo{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        createInfo.renderPass = renderPass_;
        createInfo.attachmentCount = 1;
        createInfo.pAttachments = &swapchainViews_[index];
        createInfo.width = swapchainExtent_.width;
        createInfo.height = swapchainExtent_.height;
        createInfo.layers = 1;
        checkVk(vkCreateFramebuffer(device_, &createInfo, nullptr, &framebuffers_[index]),
                "Creating swapchain framebuffer");
    }
}

void VulkanRenderer::createCommandResources() {
    VkCommandPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = queueFamily_;
    checkVk(vkCreateCommandPool(device_, &poolInfo, nullptr, &commandPool_), "Creating command pool");

    VkCommandBufferAllocateInfo allocationInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    allocationInfo.commandPool = commandPool_;
    allocationInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocationInfo.commandBufferCount = 1;
    checkVk(vkAllocateCommandBuffers(device_, &allocationInfo, &commandBuffer_), "Allocating command buffer");
}

void VulkanRenderer::initializeVoxelVolume() {
    checkVk(vkResetCommandBuffer(commandBuffer_, 0), "Resetting voxel initialization commands");
    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    checkVk(vkBeginCommandBuffer(commandBuffer_, &beginInfo), "Beginning voxel initialization");
    vkCmdFillBuffer(commandBuffer_, occupancyBuffer_, 0, kOccupancyBufferSize, 0);
    if constexpr (kScaleLabBuild) {
        vkCmdFillBuffer(commandBuffer_, geodesicFarFieldBuffer_, 0,
                        kFarFieldBytes, 0U);
    }

    VkBufferMemoryBarrier clearBarrier{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
    clearBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    clearBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
    clearBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    clearBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    clearBarrier.buffer = occupancyBuffer_;
    clearBarrier.size = kOccupancyBufferSize;
    vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                         0, nullptr, 1, &clearBarrier, 0, nullptr);
    if constexpr (kScaleLabBuild) {
        VkBufferMemoryBarrier hierarchyClearBarrier = clearBarrier;
        hierarchyClearBarrier.buffer = geodesicFarFieldBuffer_;
        hierarchyClearBarrier.size = kFarFieldBytes;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             0, nullptr, 1, &hierarchyClearBarrier, 0, nullptr);
    }

    FrameConstants constants{};
    constants.planetRadius = VOXEL_DEFAULT_PLANET_RADIUS;
    constants.planetOuterScale = planetTopology_.outerBoundingRadius;
    constants.voxelResolution = kVoxelResolution;
    constants.surfaceTileCount = static_cast<std::uint32_t>(planetTopology_.tiles.size());
    constants.residentPageCount = planetTopology_.residentPageCount();
    const TerrainGenerationSettings terrain{};
    constants.terrainSeed = terrain.seed;
    constants.terrainContinentScale = terrain.continentScale;
    constants.terrainContinentStrength = terrain.continentStrength;
    constants.terrainMountainScale = terrain.mountainScale;
    constants.terrainMountainStrength = terrain.mountainStrength;
    constants.terrainRoughness = terrain.roughness;
    constants.terrainOceanLevel = terrain.oceanLevel;
    constants.terrainPolarStrength = terrain.polarStrength;
    vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, voxelInitPipeline_);
    vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout_,
                            0, 1, &computeDescriptorSets_[0], 0, nullptr);
    vkCmdPushConstants(commandBuffer_, computePipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                       0, sizeof(constants), &constants);
    constexpr std::uint32_t groupCount = (kVoxelResolution + 3U) / 4U;
    vkCmdDispatch(commandBuffer_, groupCount, groupCount, groupCount);

    vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                      geodesicTerrainInitPipeline_);
    vkCmdPushConstants(commandBuffer_, computePipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                       0, sizeof(constants), &constants);
    vkCmdDispatch(commandBuffer_, (constants.surfaceTileCount + 255U) / 256U, 1U, 1U);

    if constexpr (kScaleLabBuild) {
        reportScaleLabStartup("local AO + two-stage macro hierarchy", 0U,
                              kScaleLabHierarchyPasses);
        VkBufferMemoryBarrier terrainToHierarchy{
            VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        terrainToHierarchy.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        terrainToHierarchy.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        terrainToHierarchy.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        terrainToHierarchy.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        terrainToHierarchy.buffer = geodesicColumnStateBuffer_;
        terrainToHierarchy.size = VK_WHOLE_SIZE;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             0, nullptr, 1, &terrainToHierarchy, 0, nullptr);
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                          geodesicFarFieldInitPipeline_);
        for (std::uint32_t pass = 0U; pass < kScaleLabHierarchyPasses; ++pass) {
            const std::uint32_t level = pass == 0U
                ? kScaleLabAoInitLevel
                : (pass == 1U ? kFarFieldLodLevels : pass - 2U);
            constants.maxMarchSteps = level;
            vkCmdPushConstants(commandBuffer_, computePipelineLayout_,
                               VK_SHADER_STAGE_COMPUTE_BIT,
                               0, sizeof(constants), &constants);
            std::uint32_t itemCount = constants.surfaceTileCount;
            if (level == kFarFieldLodLevels) {
                itemCount = farFieldNodeCount();
            } else if (level > 0U && level < kFarFieldLodLevels) {
                itemCount = std::max(kTileLookupWidth >> level, 1U) *
                            std::max(kTileLookupHeight >> level, 1U);
            }
            vkCmdDispatch(commandBuffer_, (itemCount + 255U) / 256U, 1U, 1U);
            std::array<VkBufferMemoryBarrier, 3> hierarchyLevelReady{};
            hierarchyLevelReady[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            hierarchyLevelReady[0].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            hierarchyLevelReady[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT |
                                                   VK_ACCESS_SHADER_WRITE_BIT;
            hierarchyLevelReady[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            hierarchyLevelReady[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            hierarchyLevelReady[0].buffer = geodesicFarFieldBuffer_;
            hierarchyLevelReady[0].size = kFarFieldBytes;
            hierarchyLevelReady[1] = hierarchyLevelReady[0];
            hierarchyLevelReady[1].buffer = geodesicMacroHierarchyBuffer_;
            hierarchyLevelReady[1].size = kMacroHierarchyBytes;
            hierarchyLevelReady[2] = hierarchyLevelReady[0];
            hierarchyLevelReady[2].buffer = geodesicColumnStateBuffer_;
            hierarchyLevelReady[2].size = VK_WHOLE_SIZE;
            vkCmdPipelineBarrier(commandBuffer_,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                 0, nullptr,
                                 static_cast<std::uint32_t>(hierarchyLevelReady.size()),
                                 hierarchyLevelReady.data(), 0, nullptr);
            reportScaleLabStartup("local AO + two-stage macro hierarchy", pass + 1U,
                                  kScaleLabHierarchyPasses);
        }
    }

    std::array<VkBufferMemoryBarrier, 6> readyBarriers{};
    readyBarriers[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    readyBarriers[0].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    readyBarriers[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    readyBarriers[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    readyBarriers[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    readyBarriers[0].buffer = cellBuffers_[0];
    readyBarriers[0].size = kCellBufferSize;
    readyBarriers[1] = readyBarriers[0];
    readyBarriers[1].buffer = occupancyBuffer_;
    readyBarriers[1].size = kOccupancyBufferSize;
    readyBarriers[2] = readyBarriers[0];
    readyBarriers[2].buffer = geodesicColumnStateBuffer_;
    readyBarriers[2].size = VK_WHOLE_SIZE;
    readyBarriers[3] = readyBarriers[0];
    readyBarriers[3].buffer = tileBrickBuffer_;
    readyBarriers[3].size = VK_WHOLE_SIZE;
    readyBarriers[4] = readyBarriers[0];
    readyBarriers[4].buffer = geodesicFarFieldBuffer_;
    readyBarriers[4].size = VK_WHOLE_SIZE;
    readyBarriers[5] = readyBarriers[0];
    readyBarriers[5].buffer = geodesicMacroHierarchyBuffer_;
    readyBarriers[5].size = VK_WHOLE_SIZE;
    const std::uint32_t readyBarrierCount = kScaleLabBuild ? 6U : 4U;
    vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                         0, nullptr, readyBarrierCount,
                         readyBarriers.data(), 0, nullptr);
    checkVk(vkEndCommandBuffer(commandBuffer_), "Ending voxel initialization");

    VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer_;
    checkVk(vkQueueSubmit(queue_, 1, &submitInfo, VK_NULL_HANDLE), "Submitting voxel initialization");
    checkVk(vkQueueWaitIdle(queue_), "Waiting for voxel initialization");
    reportScaleLabStartup("local AO + two-stage macro hierarchy",
                          kScaleLabHierarchyPasses,
                          kScaleLabHierarchyPasses);
    stats_.terrainGeneration = 1U;
    stats_.terrainSeed = terrain.seed;
}

void VulkanRenderer::createSynchronization() {
    if (timestampsSupported_) {
        VkQueryPoolCreateInfo queryInfo{VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO};
        queryInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
        queryInfo.queryCount = 3U;
        checkVk(vkCreateQueryPool(device_, &queryInfo, nullptr, &timestampQueryPool_),
                "Creating GPU timestamp query pool");
    }
    VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    checkVk(vkCreateSemaphore(device_, &semaphoreInfo, nullptr, &imageAvailable_), "Creating acquire semaphore");
    createPresentSemaphores();
    VkFenceCreateInfo fenceInfo{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    checkVk(vkCreateFence(device_, &fenceInfo, nullptr, &frameFence_), "Creating frame fence");
}

void VulkanRenderer::createPresentSemaphores() {
    renderingFinished_.resize(swapchainImages_.size());
    VkSemaphoreCreateInfo semaphoreInfo{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    for (VkSemaphore& semaphore : renderingFinished_) {
        checkVk(vkCreateSemaphore(device_, &semaphoreInfo, nullptr, &semaphore),
                "Creating per-image presentation semaphore");
    }
}

void VulkanRenderer::createImGuiContext() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    imguiContextCreated_ = true;
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 7.0F;
    style.FrameRounding = 4.0F;
    style.GrabRounding = 4.0F;
    style.Colors[ImGuiCol_Header] = ImVec4(0.02F, 0.43F, 0.42F, 0.82F);
    style.Colors[ImGuiCol_Button] = ImVec4(0.02F, 0.32F, 0.36F, 0.90F);
    style.Colors[ImGuiCol_CheckMark] = ImVec4(0.18F, 0.90F, 0.76F, 1.0F);

    const std::array<VkDescriptorPoolSize, 4> poolSizes{{
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 256},
        {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 256},
        {VK_DESCRIPTOR_TYPE_SAMPLER, 256},
        {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 64},
    }};
    VkDescriptorPoolCreateInfo poolInfo{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets = 512;
    poolInfo.poolSizeCount = static_cast<std::uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    checkVk(vkCreateDescriptorPool(device_, &poolInfo, nullptr, &imguiDescriptorPool_),
            "Creating ImGui descriptor pool");
}

void VulkanRenderer::initializeImGuiBackends() {
    if (!ImGui_ImplSDL2_InitForVulkan(window_)) {
        throw std::runtime_error("Dear ImGui SDL backend initialization failed");
    }

    ImGui_ImplVulkan_InitInfo initInfo{};
    initInfo.ApiVersion = VK_API_VERSION_1_2;
    initInfo.Instance = instance_;
    initInfo.PhysicalDevice = physicalDevice_;
    initInfo.Device = device_;
    initInfo.QueueFamily = queueFamily_;
    initInfo.Queue = queue_;
    initInfo.DescriptorPool = imguiDescriptorPool_;
    initInfo.MinImageCount = std::max(2U, swapchainMinImageCount_);
    initInfo.ImageCount = static_cast<std::uint32_t>(swapchainImages_.size());
#if IMGUI_VERSION_NUM >= 19200
    initInfo.PipelineInfoMain.RenderPass = renderPass_;
    initInfo.PipelineInfoMain.Subpass = 0;
    initInfo.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
#else
    initInfo.RenderPass = renderPass_;
    initInfo.Subpass = 0;
    initInfo.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
#endif
    if (!ImGui_ImplVulkan_Init(&initInfo)) {
        ImGui_ImplSDL2_Shutdown();
        throw std::runtime_error("Dear ImGui Vulkan backend initialization failed");
    }
    imguiBackendsReady_ = true;
}

bool VulkanRenderer::beginUiFrame() {
    int drawableWidth = 0;
    int drawableHeight = 0;
    SDL_Vulkan_GetDrawableSize(window_, &drawableWidth, &drawableHeight);
    if (drawableWidth <= 0 || drawableHeight <= 0) {
        resizeRequested_ = true;
        return false;
    }
    if (resizeRequested_) {
        recreateSwapchain();
    }
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    return true;
}

void VulkanRenderer::processPageRequests(float time, const RenderSettings& settings) {
    stats_.pageRequests = 0U;
    stats_.pageRequestOverflow = 0U;
    stats_.pagesStreamed = 0U;
    stats_.stalePageRequests = 0U;
    if (!pendingPageUploads_.empty()) {
        stats_.pagesStreamed = static_cast<std::uint32_t>(pendingPageUploads_.size());
        return;
    }
    if (!pageFeedbackPending_ || geodesicPageRequestMapped_ == nullptr ||
        physicalPageOwners_.empty()) {
        return;
    }
    pageFeedbackPending_ = false;

    const auto* header = static_cast<const PageRequestFeedbackHeader*>(
        geodesicPageRequestMapped_);
    const auto* requests = reinterpret_cast<const PageRequestGpu*>(
        static_cast<const std::byte*>(geodesicPageRequestMapped_) +
        sizeof(PageRequestFeedbackHeader));
    const std::uint32_t availableRequests =
        std::min(header->requestCount, kPageRequestCapacity);
    stats_.pageRequests = availableRequests;
    stats_.pageRequestOverflow = header->overflowCount;
    stats_.totalPageRequests += availableRequests;
    stats_.totalPageRequestOverflow += header->overflowCount;
    if (availableRequests == 0U) {
        return;
    }

    const CameraVector cameraDirection = cameraRadialDirection(time, settings);
    struct EvictionCandidate {
        float cameraAlignment{};
        std::uint32_t physicalPage{};
    };
    std::vector<EvictionCandidate> evictionCandidates;
    evictionCandidates.reserve(physicalPageOwners_.size());
    for (std::uint32_t physicalPage = 0; physicalPage < physicalPageOwners_.size();
         ++physicalPage) {
        const std::uint32_t owner = physicalPageOwners_[physicalPage];
        if (owner == kInvalidTile) {
            evictionCandidates.push_back({-2.0F, physicalPage});
            continue;
        }
        const auto& center = planetTopology_.tiles[owner].center;
        const float alignment = center[0] * cameraDirection[0] +
                                center[1] * cameraDirection[1] +
                                center[2] * cameraDirection[2];
        evictionCandidates.push_back({alignment, physicalPage});
    }
    std::ranges::sort(evictionCandidates, {}, &EvictionCandidate::cameraAlignment);

    std::byte* upload = static_cast<std::byte*>(geodesicStreamUploadMapped_);
    VkDeviceSize uploadCursor = 0;
    std::size_t evictionIndex = 0;
    pendingPageUploads_.reserve(kPageStreamBudget);
    for (std::uint32_t requestIndex = 0;
         requestIndex < availableRequests &&
         (requestIndex < kCollisionProfileRequestCount ||
          pendingPageUploads_.size() < kPageStreamBudget);
         ++requestIndex) {
        const PageRequestGpu& request = requests[requestIndex];
        if (request.tileIndex == kInvalidTile) {
            continue;
        }
        if (request.tileIndex >= planetTopology_.tiles.size()) {
            ++stats_.stalePageRequests;
            continue;
        }
        auto& pageEntry = planetTopology_.pageTable[request.tileIndex];
        if (pageEntry.data[1] != request.expectedGeneration) {
            ++stats_.stalePageRequests;
            continue;
        }
        const std::uint32_t occupiedLayers = request.packedColumnState & 0xffU;
        const std::uint32_t material = request.packedColumnState >> 8U;
        if (occupiedLayers == 0U || occupiedLayers > planetTopology_.brickResolution ||
            material == 0U) {
            ++stats_.stalePageRequests;
            continue;
        }
        planetTopology_.columnStates[request.tileIndex].data = {
            request.occupancyMask, occupiedLayers, material, 0U};
        freshCollisionColumns_[request.tileIndex] = 1U;
        if ((pageEntry.data[2] & kPageResident) != 0U) {
            continue;
        }

        if (pendingPageUploads_.size() >= kPageStreamBudget) {
            continue;
        }

        if (evictionIndex >= evictionCandidates.size()) {
            break;
        }
        const std::uint32_t physicalPage =
            evictionCandidates[evictionIndex++].physicalPage;
        const std::uint32_t currentOwner = physicalPageOwners_[physicalPage];
        const std::uint32_t evictedTile = planetTopology_.remapResidentPageFromOwner(
            physicalPage, request.tileIndex, currentOwner);
        physicalPageOwners_[physicalPage] = request.tileIndex;

        PendingPageUpload pending{};
        pending.physicalPage = physicalPage;
        pending.incomingTile = request.tileIndex;
        pending.evictedTile = evictedTile;
        pending.pageSourceOffset = uploadCursor;
        const std::size_t physicalCellOffset =
            static_cast<std::size_t>(physicalPage) * planetTopology_.brickResolution;
        const VkDeviceSize physicalPageBytes =
            static_cast<VkDeviceSize>(planetTopology_.brickResolution) * sizeof(std::uint32_t);
        std::memcpy(upload + uploadCursor,
                    planetTopology_.brickCells.data() + physicalCellOffset,
                    static_cast<std::size_t>(physicalPageBytes));
        uploadCursor += physicalPageBytes;

        pending.incomingEntrySourceOffset = uploadCursor;
        std::memcpy(upload + uploadCursor,
                    &planetTopology_.pageTable[pending.incomingTile],
                    sizeof(GeodesicPageTableEntryGpu));
        uploadCursor += sizeof(GeodesicPageTableEntryGpu);
        pending.evictedEntrySourceOffset = uploadCursor;
        std::memcpy(upload + uploadCursor,
                    &planetTopology_.pageTable[pending.evictedTile],
                    sizeof(GeodesicPageTableEntryGpu));
        uploadCursor += sizeof(GeodesicPageTableEntryGpu);
        pendingPageUploads_.push_back(pending);
    }
    stats_.pagesStreamed = static_cast<std::uint32_t>(pendingPageUploads_.size());
    stats_.totalPagesStreamed += stats_.pagesStreamed;
    stats_.totalStalePageRequests += stats_.stalePageRequests;
}

void VulkanRenderer::processArtifactDiagnostics() {
    if (!artifactDiagnosticPending_ || artifactDiagnosticMapped_ == nullptr) {
        return;
    }
    artifactDiagnosticPending_ = false;
    stats_.rayProbeValid = false;
    const auto* header = static_cast<const ArtifactDiagnosticHeaderGpu*>(
        artifactDiagnosticMapped_);
    const auto* records = reinterpret_cast<const ArtifactDiagnosticRecordGpu*>(
        static_cast<const std::byte*>(artifactDiagnosticMapped_) +
        sizeof(ArtifactDiagnosticHeaderGpu));
    if constexpr (kGlobalMetricLabBuild) {
        const std::uint32_t frameSamples =
            header->scalePadding[1] + header->scalePadding[2];
        const std::uint32_t frameHits = frameSamples >= header->scalePadding[5]
            ? frameSamples - header->scalePadding[5] : 0U;
        stats_.globalMappedFiniteRays += header->scalePadding[0];
        stats_.globalRadialPositiveRays += header->scalePadding[1];
        stats_.globalRadialNonPositiveRays += header->scalePadding[2];
        stats_.globalTerrainIntervalRays += header->scalePadding[3];
        stats_.globalDdaCandidateRays += header->scalePadding[4];
        stats_.globalFinalSkyRays += header->scalePadding[5];
        stats_.globalMappedOriginRepairs += header->scaleMacroStats[0];
        stats_.globalBackFacingHitRejects += header->scaleMacroStats[1];
        stats_.globalBackFacingHitRecoveries += header->scaleMacroStats[2];
        stats_.globalNearTangentRays += header->scaleMacroStats[3];
        stats_.globalExactTangentRays += header->scaleDdaHistogram[0];
        stats_.globalAtmosphereIntervalRays += header->scaleDdaHistogram[1];
        stats_.globalCloudIntervalRays += header->scaleDdaHistogram[2];
        stats_.globalAffineBudgetExhaustions +=
            header->scaleDdaHistogram[4];
        stats_.globalFrontFacingHits += frameHits;
        stats_.globalFrameRaySamples = frameSamples;
        stats_.globalFrameTerrainIntervals = header->scalePadding[3];
        stats_.globalFrameAtmosphereIntervals = header->scaleDdaHistogram[1];
        stats_.globalFrameCloudIntervals = header->scaleDdaHistogram[2];
        stats_.globalFrameAffineBudgetExhaustions =
            header->scaleDdaHistogram[4];
        stats_.globalSpatialAaCandidatePixels +=
            header->scaleDdaHistogram[5];
        stats_.globalSpatialAaPixels += header->scaleDdaHistogram[6];
        stats_.globalSpatialAaFamilyTransitions +=
            header->scaleDdaHistogram[7];
        stats_.globalSpatialAaSubrays += header->scaleDdaHistogram[8];
        stats_.globalSpatialAaRejectedPixels +=
            header->scaleDdaHistogram[9];
        stats_.globalSpatialAaPixelSamples +=
            header->scaleDdaHistogram[10];
        stats_.globalFrameSpatialAaPixels = header->scaleDdaHistogram[6];
        stats_.globalFrameSpatialAaPixelSamples =
            header->scaleDdaHistogram[10];
        stats_.globalMetricPathSteps += header->scaleDdaHistogram[18];
        stats_.globalMetricPathSamples += header->scaleDdaHistogram[19];
        stats_.globalSpatialAaPathSteps += header->scaleDdaHistogram[20];
        stats_.globalSpatialAaPathSamples += header->scaleDdaHistogram[21];
        stats_.globalObserverBroadCandidates += header->scaleAtlasHistogram[0];
        stats_.globalObserverExactHits += header->scaleAtlasHistogram[1];
        stats_.globalObserverSelectedHits += header->scaleAtlasHistogram[2];
        stats_.globalCurvedSelectedRays += header->scaleAtlasHistogram[3];
        stats_.globalObserverMediaSegments += header->scaleAtlasHistogram[4];
        stats_.globalMouthARays += header->scaleAtlasHistogram[5];
        stats_.globalMouthBRays += header->scaleAtlasHistogram[6];
        stats_.globalExteriorDirectRays += header->scaleAtlasHistogram[7];
        stats_.globalObserverHitsBehindMouth += header->scaleAtlasHistogram[8];
        stats_.globalBroadShellCurvedRays += header->scaleAtlasHistogram[9];
        stats_.globalBroadShellMouthARays += header->scaleAtlasHistogram[10];
        stats_.globalBroadShellMouthBRays += header->scaleAtlasHistogram[11];
        stats_.globalContentShellCrossingRays +=
            header->scaleAtlasHistogram[12];
        stats_.globalFrameCurvedSelectedRays = header->scaleAtlasHistogram[3];
        stats_.globalFrameMouthARays = header->scaleAtlasHistogram[5];
        stats_.globalFrameMouthBRays = header->scaleAtlasHistogram[6];
        stats_.globalFrameContentShellCrossingRays =
            header->scaleAtlasHistogram[12];
        stats_.globalFrameCenterSampleValid =
            header->scaleAtlasHistogram[13] != 0U;
        stats_.globalFrameCenterLinearR = std::bit_cast<float>(
            header->scaleAtlasHistogram[14]);
        stats_.globalFrameCenterLinearG = std::bit_cast<float>(
            header->scaleAtlasHistogram[15]);
        stats_.globalFrameCenterLinearB = std::bit_cast<float>(
            header->scaleAtlasHistogram[16]);
        stats_.globalFrameCenterHit = header->scaleAtlasHistogram[17] != 0U;
        stats_.globalFrameCenterDepth = std::bit_cast<float>(
            header->scaleAtlasHistogram[18]);
        stats_.globalFrameCenterMappedOriginX = std::bit_cast<float>(
            header->scaleAtlasHistogram[19]);
        stats_.globalFrameCenterMappedOriginY = std::bit_cast<float>(
            header->scaleAtlasHistogram[20]);
        stats_.globalFrameCenterMappedOriginZ = std::bit_cast<float>(
            header->scaleAtlasHistogram[21]);
        stats_.globalFrameCenterMappedDirectionX = std::bit_cast<float>(
            header->scaleAtlasHistogram[22]);
        stats_.globalFrameCenterMappedDirectionY = std::bit_cast<float>(
            header->scaleAtlasHistogram[23]);
        stats_.globalFrameCenterMappedDirectionZ = std::bit_cast<float>(
            header->scaleAtlasHistogram[24]);
        stats_.globalFrameCenterHitClass = header->scaleAtlasHistogram[25];
        stats_.globalFrameCenterMouthOwner = header->scaleAtlasHistogram[26];
        stats_.globalFrameCenterStarFootprint = std::bit_cast<float>(
            header->scaleAtlasHistogram[27]);
    } else if constexpr (kFractalPlanetSdfBuild) {
        for (std::size_t index = 0U;
             index < fractalPlanetLevelHistogram_.size(); ++index) {
            fractalPlanetLevelHistogram_[index] +=
                header->scaleAtlasHistogram[index];
        }
        fractalPlanetHits_ += header->scalePadding[0];
        fractalPlanetIntervalSkips_ += header->scalePadding[1];
        fractalPlanetNonfiniteRejects_ += header->scalePadding[2];
        fractalPlanetRootIterations_ += header->scalePadding[3];
        fractalPlanetRecoveredHits_ += header->scalePadding[4];
        fractalPlanetClosedShellMisses_ += header->scalePadding[5];
        const auto percentile = [](const auto& histogram, std::uint32_t percent) {
            const std::uint64_t total = std::accumulate(
                histogram.begin(), histogram.end(), std::uint64_t{0});
            const std::uint64_t target = (total * percent + 99U) / 100U;
            std::uint64_t running = 0U;
            for (std::size_t index = 0U; index < histogram.size(); ++index) {
                running += histogram[index];
                if (running >= target) {
                    return static_cast<std::uint32_t>(index);
                }
            }
            return 0U;
        };
        stats_.fractalPlanetHits = fractalPlanetHits_;
        stats_.fractalPlanetIntervalSkips = fractalPlanetIntervalSkips_;
        stats_.fractalPlanetNonfiniteRejects = fractalPlanetNonfiniteRejects_;
        stats_.fractalPlanetRootIterations = fractalPlanetRootIterations_;
        stats_.fractalPlanetRecoveredHits = fractalPlanetRecoveredHits_;
        stats_.fractalPlanetClosedShellMisses = fractalPlanetClosedShellMisses_;
        stats_.fractalPlanetLevelP50 = percentile(fractalPlanetLevelHistogram_, 50U);
        stats_.fractalPlanetLevelP95 = percentile(fractalPlanetLevelHistogram_, 95U);
        stats_.fractalPlanetLevelMaximum = percentile(fractalPlanetLevelHistogram_, 100U);
    } else if constexpr (kAdaptiveSdfBuild) {
        for (std::size_t index = 0U;
             index < adaptiveSdfLevelHistogram_.size(); ++index) {
            adaptiveSdfLevelHistogram_[index] +=
                header->scaleAtlasHistogram[index];
        }
        adaptiveSdfActiveSamples_ += header->scalePadding[0];
        adaptiveSdfNonfiniteRejects_ += header->scalePadding[1];
        adaptiveSdfBoundaryRejects_ += header->scalePadding[2];
        adaptiveSdfRootIterations_ += header->scalePadding[3];
        const auto percentile = [](const auto& histogram, std::uint32_t percent) {
            const std::uint64_t total = std::accumulate(
                histogram.begin(), histogram.end(), std::uint64_t{0});
            const std::uint64_t target = (total * percent + 99U) / 100U;
            std::uint64_t running = 0U;
            for (std::size_t index = 0U; index < histogram.size(); ++index) {
                running += histogram[index];
                if (running >= target) {
                    return static_cast<std::uint32_t>(index);
                }
            }
            return 0U;
        };
        stats_.adaptiveSdfActiveSamples = adaptiveSdfActiveSamples_;
        stats_.adaptiveSdfNonfiniteRejects = adaptiveSdfNonfiniteRejects_;
        stats_.adaptiveSdfBoundaryRejects = adaptiveSdfBoundaryRejects_;
        stats_.adaptiveSdfRootIterations = adaptiveSdfRootIterations_;
        stats_.adaptiveSdfLevelP50 = percentile(adaptiveSdfLevelHistogram_, 50U);
        stats_.adaptiveSdfLevelP95 = percentile(adaptiveSdfLevelHistogram_, 95U);
        stats_.adaptiveSdfLevelMaximum = percentile(adaptiveSdfLevelHistogram_, 100U);
    } else if constexpr (kScaleLabBuild) {
        for (std::size_t index = 0; index < scaleDdaHistogram_.size(); ++index) {
            scaleDdaHistogram_[index] += header->scaleDdaHistogram[index];
        }
        for (std::size_t index = 0; index < scaleAtlasHistogram_.size(); ++index) {
            scaleAtlasHistogram_[index] += header->scaleAtlasHistogram[index];
        }
        scaleBoundSkips_ += header->scalePadding[0];
        scaleBoundNonfinite_ += header->scalePadding[1];
        scaleDifferentialSamples_ += header->scalePadding[2];
        scaleDifferentialHitMismatches_ += header->scalePadding[3];
        scaleDifferentialDistanceMismatches_ += header->scalePadding[4];
        scaleDifferentialMaximumDistanceUlps_ = std::max(
            scaleDifferentialMaximumDistanceUlps_, header->scalePadding[5]);
        const auto percentile = [](const auto& histogram, std::uint32_t percent) {
            const std::uint64_t total = std::accumulate(
                histogram.begin(), histogram.end(), std::uint64_t{0});
            const std::uint64_t target = (total * percent + 99U) / 100U;
            std::uint64_t running = 0U;
            for (std::size_t index = 0; index < histogram.size(); ++index) {
                running += histogram[index];
                if (running >= target) {
                    return static_cast<std::uint32_t>(index);
                }
            }
            return 0U;
        };
        stats_.telemetryRaySamples = std::accumulate(
            scaleDdaHistogram_.begin(), scaleDdaHistogram_.end(), std::uint64_t{0});
        std::uint64_t sampledDdaEvents = 0U;
        for (std::size_t index = 0; index < scaleDdaHistogram_.size(); ++index) {
            sampledDdaEvents += static_cast<std::uint64_t>(index) *
                                scaleDdaHistogram_[index];
        }
        stats_.ddaEventsP50 = percentile(scaleDdaHistogram_, 50U);
        stats_.ddaEventsP95 = percentile(scaleDdaHistogram_, 95U);
        stats_.ddaEventsP99 = percentile(scaleDdaHistogram_, 99U);
        stats_.ddaEventsMaximum = percentile(scaleDdaHistogram_, 100U);
        stats_.atlasRefinementsP50 = percentile(scaleAtlasHistogram_, 50U);
        stats_.atlasRefinementsP95 = percentile(scaleAtlasHistogram_, 95U);
        stats_.atlasRefinementsP99 = percentile(scaleAtlasHistogram_, 99U);
        stats_.atlasRefinementsMaximum = percentile(scaleAtlasHistogram_, 100U);
        stats_.conservativeBoundSkips = scaleBoundSkips_;
        stats_.conservativeBoundNonfinite = scaleBoundNonfinite_;
        // Derive broad/narrow traffic from the existing sampled histograms so
        // instrumentation does not add four live counters to the register-heavy
        // f1024 ray shader. Each conservative skip evaluates three macro levels;
        // each exact hex event consumes at most seven compact records (owner +
        // six neighbors; pentagons consume one fewer).
        stats_.macroNodeTests = 3U *
            (scaleBoundSkips_ + stats_.telemetryRaySamples);
        stats_.macroCandidates = stats_.telemetryRaySamples;
        stats_.macroFallbacks = scaleBoundNonfinite_;
        stats_.compactTileLoads = 7U * sampledDdaEvents;
        stats_.differentialSamples = scaleDifferentialSamples_;
        stats_.differentialHitMismatches = scaleDifferentialHitMismatches_;
        stats_.differentialDistanceMismatches =
            scaleDifferentialDistanceMismatches_;
        stats_.differentialMaximumDistanceUlps =
            scaleDifferentialMaximumDistanceUlps_;
    } else {
        microSdfActiveSamples_ += header->scaleMacroStats[0];
        microSdfNonfiniteRejects_ += header->scaleMacroStats[1];
        microSdfBoundaryRejects_ += header->scaleMacroStats[2];
        microSdfOversteps_ += header->scaleMacroStats[3];
        stats_.microSdfActiveSamples = microSdfActiveSamples_;
        stats_.microSdfNonfiniteRejects = microSdfNonfiniteRejects_;
        stats_.microSdfBoundaryRejects = microSdfBoundaryRejects_;
        stats_.microSdfOversteps = microSdfOversteps_;
    }
    const std::uint32_t available = std::min(
        header->control[0], kArtifactDiagnosticCapacity);
    stats_.artifactDiagnosticRecords += available;
    stats_.artifactDiagnosticOverflow += header->control[1];
    stats_.artifactAllNoOwnerMisses += header->control[2];
    stats_.artifactAllFarMisses += header->control[3];
    stats_.artifactCapHits += header->classCounts0[0];
    stats_.artifactNonExposedSides += header->classCounts0[1];
    stats_.artifactFilteredSides += header->classCounts0[2];
    stats_.artifactPartialSides += header->classCounts0[3];
    stats_.artifactResolvedSides += header->classCounts1[0];
    stats_.artifactInvalidNeighbors += header->classCounts1[1];
    stats_.artifactClosedShellMisses += header->classCounts1[2];
    stats_.artifactRecoverableHorizonMisses += header->classCounts1[3];
    for (std::size_t bin = 0U; bin < header->horizonCounts.size(); ++bin) {
        stats_.artifactRecoverableClosestApproachBins[bin] +=
            header->horizonCounts[bin];
    }
    for (std::size_t bin = 0U; bin < header->noOwnerApproachCounts.size(); ++bin) {
        stats_.artifactNoOwnerClosestApproachBins[bin] +=
            header->noOwnerApproachCounts[bin];
    }
    stats_.artifactNearHorizonStarPixels += header->skyCounts[0];

    if (available == 0U || stats_.width == 0U || stats_.height == 0U) {
        return;
    }
    std::vector<std::uint8_t> highContrast(
        static_cast<std::size_t>(stats_.width) * stats_.height, 0U);
    std::vector<std::uint8_t> isolatedMask(
        static_cast<std::size_t>(stats_.width) * stats_.height, 0U);
    std::vector<std::uint32_t> candidates;
    candidates.reserve(available);
    for (std::uint32_t index = 0U; index < available; ++index) {
        const ArtifactDiagnosticRecordGpu& record = records[index];
        const std::uint32_t x = record.pixelClassTermination[0];
        const std::uint32_t y = record.pixelClassTermination[1];
        const std::uint32_t classification = record.pixelClassTermination[2];
        if (x == stats_.width / 2U && y == stats_.height / 2U) {
            stats_.rayProbeValid = true;
            stats_.rayProbeHit = record.pixelClassTermination[3] == 2U;
            stats_.rayProbeClass = classification;
            stats_.rayProbeTermination = record.pixelClassTermination[3];
            stats_.rayProbeTile = record.cell[0];
            stats_.rayProbeLayer = record.cell[1];
            stats_.rayProbeMaterial = record.normalMaterial[3];
            stats_.rayProbeTravel = std::bit_cast<float>(record.footprint[0]);
            stats_.rayProbeIntervalNear =
                std::bit_cast<float>(record.intervalCandidate[0]);
            stats_.rayProbeIntervalFar =
                std::bit_cast<float>(record.intervalCandidate[1]);
            stats_.rayProbeCandidateDistance =
                std::bit_cast<float>(record.intervalCandidate[2]);
            stats_.rayProbeClosestApproach =
                std::bit_cast<float>(record.intervalCandidate[3]);
            stats_.rayProbeNormalX =
                std::bit_cast<float>(record.normalMaterial[0]);
            stats_.rayProbeNormalY =
                std::bit_cast<float>(record.normalMaterial[1]);
            stats_.rayProbeNormalZ =
                std::bit_cast<float>(record.normalMaterial[2]);
        }
        const float geometryDetail = std::bit_cast<float>(record.footprint[1]);
        const float hitDistance = std::bit_cast<float>(record.footprint[0]);
        const bool unstableSideSample = classification == 3U ||
            (classification == 4U && hitDistance > 0.15F);
        if (x >= stats_.width || y >= stats_.height ||
            !unstableSideSample ||
            !(geometryDetail > 0.05F)) {
            continue;
        }
        highContrast[static_cast<std::size_t>(y) * stats_.width + x] = 1U;
        candidates.push_back(index);
    }

    std::uint32_t isolated = 0U;
    const ArtifactDiagnosticRecordGpu* firstIsolated = nullptr;
    for (const std::uint32_t recordIndex : candidates) {
        const ArtifactDiagnosticRecordGpu& record = records[recordIndex];
        const int x = static_cast<int>(record.pixelClassTermination[0]);
        const int y = static_cast<int>(record.pixelClassTermination[1]);
        std::uint32_t neighbors = 0U;
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                if ((dx == 0 && dy == 0) || x + dx < 0 || y + dy < 0 ||
                    x + dx >= static_cast<int>(stats_.width) ||
                    y + dy >= static_cast<int>(stats_.height)) {
                    continue;
                }
                neighbors += highContrast[
                    static_cast<std::size_t>(y + dy) * stats_.width +
                    static_cast<std::size_t>(x + dx)];
            }
        }
        if (neighbors <= 2U) {
            ++isolated;
            isolatedMask[static_cast<std::size_t>(y) * stats_.width +
                         static_cast<std::size_t>(x)] = 1U;
            const float projectedWidth = std::bit_cast<float>(record.footprint[3]);
            const float projectedHeight = std::bit_cast<float>(record.ray[0]);
            const float minorPixels = std::min(projectedWidth, projectedHeight);
            const std::size_t histogramBucket = minorPixels < 1.0F ? 0U
                                                : minorPixels < 2.0F ? 1U
                                                : minorPixels < 3.0F ? 2U
                                                : minorPixels < 4.0F ? 3U
                                                : minorPixels < 6.0F ? 4U : 5U;
            ++stats_.artifactIsolatedMinorPixelHistogram[histogramBucket];
            if (firstIsolated == nullptr) {
                firstIsolated = &record;
            }
        }
    }
    constexpr std::array<std::array<int, 2>, 12> kChainDirections{{
        {{1, 0}}, {{0, 1}}, {{1, 1}}, {{1, -1}},
        {{2, 1}}, {{2, -1}}, {{1, 2}}, {{1, -2}},
        {{3, 1}}, {{3, -1}}, {{1, 3}}, {{1, -3}}}};
    std::uint32_t dottedChainPixels = 0U;
    int firstChainX = -1;
    int firstChainY = -1;
    const auto hasIsolated = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < static_cast<int>(stats_.width) &&
               y < static_cast<int>(stats_.height) &&
               isolatedMask[static_cast<std::size_t>(y) * stats_.width +
                            static_cast<std::size_t>(x)] != 0U;
    };
    for (std::uint32_t y = 0U; y < stats_.height; ++y) {
        for (std::uint32_t x = 0U; x < stats_.width; ++x) {
            if (!hasIsolated(static_cast<int>(x), static_cast<int>(y))) {
                continue;
            }
            bool belongsToChain = false;
            for (const auto& direction : kChainDirections) {
                bool foundBefore = false;
                bool foundAfter = false;
                for (int gap = 2; gap <= 8; ++gap) {
                    foundBefore = foundBefore || hasIsolated(
                        static_cast<int>(x) - direction[0] * gap,
                        static_cast<int>(y) - direction[1] * gap);
                    foundAfter = foundAfter || hasIsolated(
                        static_cast<int>(x) + direction[0] * gap,
                        static_cast<int>(y) + direction[1] * gap);
                }
                if (foundBefore && foundAfter) {
                    belongsToChain = true;
                    break;
                }
            }
            dottedChainPixels += belongsToChain ? 1U : 0U;
            if (belongsToChain && firstChainX < 0) {
                firstChainX = static_cast<int>(x);
                firstChainY = static_cast<int>(y);
            }
        }
    }
    stats_.artifactMaximumIsolatedSidePixels = std::max(
        stats_.artifactMaximumIsolatedSidePixels, isolated);
    stats_.artifactMaximumDottedChainPixels = std::max(
        stats_.artifactMaximumDottedChainPixels, dottedChainPixels);
    if (firstChainX >= 0 && stats_.artifactFirstChainTile == 0xffffffffU) {
        for (const std::uint32_t recordIndex : candidates) {
            const ArtifactDiagnosticRecordGpu& record = records[recordIndex];
            if (record.pixelClassTermination[0] ==
                    static_cast<std::uint32_t>(firstChainX) &&
                record.pixelClassTermination[1] ==
                    static_cast<std::uint32_t>(firstChainY)) {
                stats_.artifactFirstChainPixelX = record.pixelClassTermination[0];
                stats_.artifactFirstChainPixelY = record.pixelClassTermination[1];
                stats_.artifactFirstChainClass = record.pixelClassTermination[2];
                stats_.artifactFirstChainTile = record.cell[0];
                stats_.artifactFirstChainDistance =
                    std::bit_cast<float>(record.footprint[0]);
                stats_.artifactFirstChainProjectedWidth =
                    std::bit_cast<float>(record.footprint[3]);
                stats_.artifactFirstChainProjectedHeight =
                    std::bit_cast<float>(record.ray[0]);
                stats_.artifactFirstChainLocalInterior =
                    std::bit_cast<float>(record.state[3]);
                break;
            }
        }
    }

    for (std::uint32_t index = 0U; index < available; ++index) {
        const ArtifactDiagnosticRecordGpu& record = records[index];
        const std::uint32_t termination = record.pixelClassTermination[3];
        if (termination < stats_.artifactTerminationCounts.size()) {
            ++stats_.artifactTerminationCounts[termination];
        }
        if (record.pixelClassTermination[2] == 6U &&
            stats_.artifactFirstMissTermination == 0U) {
            stats_.artifactFirstMissPixelX = record.pixelClassTermination[0];
            stats_.artifactFirstMissPixelY = record.pixelClassTermination[1];
            stats_.artifactFirstMissTermination = termination;
            stats_.artifactFirstMissTile = record.cell[0];
            stats_.artifactFirstMissColumnHeight = record.cell[3];
            stats_.artifactFirstMissTravel = std::bit_cast<float>(record.footprint[0]);
            stats_.artifactFirstMissFar = std::bit_cast<float>(record.ray[0]);
            stats_.artifactFirstMissCapDenominator =
                std::bit_cast<float>(record.footprint[1]);
            stats_.artifactFirstMissCapDistance =
                std::bit_cast<float>(record.footprint[2]);
            stats_.artifactFirstMissConeMargin =
                std::bit_cast<float>(record.footprint[3]);
            stats_.artifactFirstMissRayX = std::bit_cast<float>(record.ray[1]);
            stats_.artifactFirstMissRayY = std::bit_cast<float>(record.ray[2]);
            stats_.artifactFirstMissRayZ = std::bit_cast<float>(record.ray[3]);
        }
        if (record.pixelClassTermination[2] == 8U &&
            stats_.artifactFirstHorizonTile == 0xffffffffU) {
            stats_.artifactFirstHorizonPixelX = record.pixelClassTermination[0];
            stats_.artifactFirstHorizonPixelY = record.pixelClassTermination[1];
            stats_.artifactFirstHorizonTermination = termination;
            stats_.artifactFirstHorizonTile = record.cell[0];
            stats_.artifactFirstHorizonDistance =
                std::bit_cast<float>(record.footprint[0]);
            stats_.artifactFirstHorizonCapDenominator =
                std::bit_cast<float>(record.footprint[1]);
            stats_.artifactFirstHorizonCapDistance =
                std::bit_cast<float>(record.footprint[2]);
            stats_.artifactFirstHorizonTravel =
                std::bit_cast<float>(record.footprint[3]);
            stats_.artifactFirstHorizonFar =
                std::bit_cast<float>(record.ray[0]);
            stats_.artifactFirstHorizonClosestApproach =
                std::bit_cast<float>(record.state[3]);
        }
    }
    if (firstIsolated != nullptr && stats_.artifactFirstTile == 0xffffffffU) {
        stats_.artifactFirstPixelX = firstIsolated->pixelClassTermination[0];
        stats_.artifactFirstPixelY = firstIsolated->pixelClassTermination[1];
        stats_.artifactFirstClass = firstIsolated->pixelClassTermination[2];
        stats_.artifactFirstTile = firstIsolated->cell[0];
        stats_.artifactFirstLayer = firstIsolated->cell[1];
        stats_.artifactFirstNeighbor = firstIsolated->cell[2];
        stats_.artifactFirstColumnHeight = firstIsolated->cell[3];
        stats_.artifactFirstNeighborHeight = firstIsolated->state[0];
        stats_.artifactFirstMaterial = firstIsolated->state[1];
        stats_.artifactFirstTermination = firstIsolated->pixelClassTermination[3];
        stats_.artifactFirstDistance = std::bit_cast<float>(firstIsolated->footprint[0]);
        stats_.artifactFirstGeometryDetail = std::bit_cast<float>(firstIsolated->footprint[1]);
        stats_.artifactFirstSideIncidence = std::bit_cast<float>(firstIsolated->footprint[2]);
        stats_.artifactFirstProjectedWidth = std::bit_cast<float>(firstIsolated->footprint[3]);
        stats_.artifactFirstProjectedHeight = std::bit_cast<float>(firstIsolated->ray[0]);
        stats_.artifactFirstRayX = std::bit_cast<float>(firstIsolated->ray[1]);
        stats_.artifactFirstRayY = std::bit_cast<float>(firstIsolated->ray[2]);
        stats_.artifactFirstRayZ = std::bit_cast<float>(firstIsolated->ray[3]);
        stats_.artifactFirstSideMask = std::bit_cast<float>(firstIsolated->state[2]);
        stats_.artifactFirstLocalInteriorPixels =
            std::bit_cast<float>(firstIsolated->state[3]);
    }
}

void VulkanRenderer::render(float time, const RenderSettings& settings) {
    ImGui::Render();
    checkVk(vkWaitForFences(device_, 1, &frameFence_, VK_TRUE, std::numeric_limits<std::uint64_t>::max()),
            "Waiting for frame fence");
    if (timestampsSupported_ && timestampPending_) {
        std::array<std::uint64_t, 3> timestamps{};
        const VkResult queryResult = vkGetQueryPoolResults(
            device_, timestampQueryPool_, 0U, static_cast<std::uint32_t>(timestamps.size()),
            sizeof(timestamps), timestamps.data(), sizeof(std::uint64_t),
            VK_QUERY_RESULT_64_BIT);
        if (queryResult == VK_SUCCESS) {
            const std::uint64_t mask = timestampValidBits_ >= 64U
                                           ? std::numeric_limits<std::uint64_t>::max()
                                           : (1ULL << timestampValidBits_) - 1ULL;
            const std::uint64_t streamingTicks = (timestamps[1] - timestamps[0]) & mask;
            const std::uint64_t renderTicks = (timestamps[2] - timestamps[1]) & mask;
            const double millisecondsPerTick =
                static_cast<double>(timestampPeriodNanoseconds_) / 1'000'000.0;
            stats_.gpuStreamingMilliseconds = static_cast<float>(
                static_cast<double>(streamingTicks) * millisecondsPerTick);
            stats_.gpuRenderMilliseconds = static_cast<float>(
                static_cast<double>(renderTicks) * millisecondsPerTick);
            stats_.gpuTotalComputeMilliseconds =
                stats_.gpuStreamingMilliseconds + stats_.gpuRenderMilliseconds;
        } else if (queryResult != VK_NOT_READY) {
            checkVk(queryResult, "Reading GPU timestamp queries");
        }
        timestampPending_ = false;
    }
    processArtifactDiagnostics();
    processPageRequests(time, settings);
    if constexpr (kEightPlanetSystemLabBuild) {
        auto* words = static_cast<std::uint32_t*>(eightPlanetAuthorityMapped_);
        const std::uint32_t telemetryOffset = words[17];
        const std::uint32_t telemetryStride = words[18];
        for (std::uint32_t planet = 0U;
             planet < system_lab::kPlanetCount; ++planet) {
            std::uint32_t* telemetry = words + telemetryOffset +
                planet * telemetryStride;
            stats_.systemGeneratedPageEvaluations += telemetry[1];
            stats_.systemStalePageRejects += telemetry[2];
            stats_.systemConservativeBoundaryRefinements += telemetry[3];
            stats_.systemCoarseHits += telemetry[4];
            stats_.systemExactHits += telemetry[5];
            stats_.systemEditHits += telemetry[6];
            stats_.systemNonfiniteOutputs += telemetry[7];
            stats_.systemCracksOrClosedShellMisses += telemetry[8];
            stats_.systemDifferentialSamples += telemetry[9];
            stats_.systemDifferentialHitMismatches += telemetry[10];
            stats_.systemDifferentialDepthMismatches += telemetry[11];
            stats_.systemDifferentialMaterialMismatches += telemetry[12];
            stats_.systemAdaptiveBudgetFallbacks += telemetry[13];
            stats_.systemReferenceBudgetFallbacks += telemetry[14];
            std::fill_n(telemetry, telemetryStride, 0U);
        }
#if VOXEL_EIGHT_PLANET_SYSTEM_LAB
        if (eightPlanetAuthoritySeeds_ !=
            settings.eightPlanetSystem.planetSeeds) {
            const std::vector<std::uint32_t> replacement =
                packSystemAuthorities(planetTopology_, eightPlanetHierarchy_,
                                      settings.eightPlanetSystem);
            if (replacement.size() != eightPlanetAuthorityWords_.size()) {
                throw std::runtime_error(
                    "Eight-planet authority rebuild changed fixed GPU layout");
            }
            eightPlanetAuthorityWords_ = replacement;
            std::memcpy(eightPlanetAuthorityMapped_,
                        eightPlanetAuthorityWords_.data(),
                        eightPlanetAuthorityWords_.size() *
                            sizeof(std::uint32_t));
            eightPlanetAuthoritySeeds_ =
                settings.eightPlanetSystem.planetSeeds;
        }
#endif
    }
    if constexpr (kPortalLabBuild) {
#if VOXEL_INTRINSIC_PORTAL_LAB
        auto* portalBuffer = static_cast<IntrinsicEllisGpuBuffer*>(
            portalLabParameterMapped_);
        stats_.portalGrTableLookups = portalBuffer->telemetry[0];
        stats_.portalGrLocalRefinements = portalBuffer->telemetry[1];
        stats_.portalGrNonfiniteRays = portalBuffer->telemetry[2];
        stats_.portalGrNegativeEndRays = portalBuffer->telemetry[3];
        portalBuffer->telemetry = {};
        const IntrinsicEllisGpuParameters parameters =
            intrinsicEllisGpuParameters(
                settings.intrinsicEllis,
                settings.intrinsicEllis.cameraState,
                settings.intrinsicEllis.cameraFrame);
        std::memcpy(portalLabParameterMapped_, &parameters,
                    sizeof(parameters));
#else
        auto* portalBuffer = static_cast<PortalLabGpuBuffer*>(
            portalLabParameterMapped_);
        stats_.portalGrTableLookups = portalBuffer->telemetry[0];
        stats_.portalGrLocalRefinements = portalBuffer->telemetry[1];
        stats_.portalGrNonfiniteRays = portalBuffer->telemetry[2];
        stats_.portalGrNegativeEndRays = portalBuffer->telemetry[3];
        portalBuffer->telemetry = {};
        const PortalLabGpuParameters parameters = portalGpuParameters(settings.portal);
        std::memcpy(portalLabParameterMapped_, &parameters, sizeof(parameters));
#endif
    }
    adaptiveSdfEnabled_ = settings.adaptivePlanetSdfEnabled;
    adaptiveSdfSeed_ = settings.terrain.seed;
    adaptiveSdfMaximumLevels_ = std::min(
        settings.adaptivePlanetSdfMaximumLevels,
        kAdaptivePlanetSdfMaximumLevels);
    adaptiveSdfLayerFraction_ = std::clamp(
        settings.adaptivePlanetSdfLayerFraction, 0.0F,
        kAdaptivePlanetSdfMaximumLayerFraction);
    fractalPlanetSdfEnabled_ = settings.fractalPlanetSdfEnabled;
    fractalPlanetSdfSeed_ = settings.terrain.seed;
    fractalPlanetSdfMaximumLevels_ = std::min(
        settings.fractalPlanetSdfMaximumLevels,
        kFractalPlanetMaximumLevels);
    fractalPlanetSdfMicroRelief_ = std::clamp(
        settings.fractalPlanetSdfMicroRelief, 0.0F, 0.018F);

    std::uint32_t imageIndex = 0;
    const VkResult acquireResult = vkAcquireNextImageKHR(
        device_, swapchain_, std::numeric_limits<std::uint64_t>::max(), imageAvailable_, VK_NULL_HANDLE, &imageIndex);
    if (acquireResult == VK_ERROR_OUT_OF_DATE_KHR) {
        resizeRequested_ = true;
        return;
    }
    if (acquireResult != VK_SUCCESS && acquireResult != VK_SUBOPTIMAL_KHR) {
        checkVk(acquireResult, "Acquiring swapchain image");
    }

    checkVk(vkResetFences(device_, 1, &frameFence_), "Resetting frame fence");
    checkVk(vkResetCommandBuffer(commandBuffer_, 0), "Resetting frame command buffer");
    FrameConstants constants{};
    constants.time = time;
    constants.planetRadius = settings.planetRadius;
    constants.maxMarchSteps = packMarchAndGeodesicShading(
        settings.maxMarchSteps, settings.localGeodesicAoEnabled,
        settings.localGeodesicAoStrength, settings.localGeodesicAoDebug,
        kScaleLabBuild && settings.ddaWorkHeatmapDebug,
        !kScaleLabBuild && !kAdaptiveSdfBuild && settings.terrainMicroSdfEnabled,
        settings.terrainMicroSdfStrength,
        !kScaleLabBuild && !kAdaptiveSdfBuild && settings.terrainMicroSdfDebug);
    constants.hitEpsilon = settings.hitEpsilon;
    constants.width = swapchainExtent_.width;
    constants.height = swapchainExtent_.height;
    constants.cameraMode = static_cast<std::uint32_t>(settings.cameraMode);
    if (settings.cameraMode == CameraControllerMode::SurfaceTraversal) {
        const CameraVector radial = surfaceRadial(settings);
        const CameraVector tangentForward = cameraNormalized(
            {settings.surfaceForwardX, settings.surfaceForwardY, settings.surfaceForwardZ},
            {0.0F, 1.0F, 0.0F});
        CameraVector basisForward{};
        CameraVector basisRight{};
        surfaceTangentBasis(radial, basisForward, basisRight);
        constants.cameraDistance = settings.surfaceCameraRadius > 0.0F
            ? settings.surfaceCameraRadius
            : settings.planetRadius * planetTopology_.outerBoundingRadius +
                std::max(settings.surfaceClearance,
                         minimumSurfaceClearance(settings.planetRadius));
        constants.cameraYaw = std::atan2(radial[0], radial[2]);
        constants.cameraPitch = std::asin(std::clamp(radial[1], -1.0F, 1.0F));
        constants.surfaceHeading = std::atan2(cameraDot(tangentForward, basisRight),
                                              cameraDot(tangentForward, basisForward));
        constants.surfaceLookPitch = std::clamp(settings.surfaceLookPitch, -1.45F, 1.45F);
    } else if constexpr (kPortalLabBuild) {
        if (settings.cameraMode == CameraControllerMode::PortalFreeFly) {
#if VOXEL_INTRINSIC_PORTAL_LAB
            // The shader builds this camera from the intrinsic SSBO tetrad.
            // Push constants intentionally carry no Euclidean observer pose.
            constants.cameraDistance = settings.planetRadius;
            constants.cameraYaw = 0.0F;
            constants.cameraPitch = 0.0F;
#else
            const PortalVector normalizedPosition = settings.portal.freeFlyPosition;
            const float normalizedRadius = std::max(
                portalLength(normalizedPosition), 1.0e-6F);
            const PortalVector radial = normalizedPosition * (1.0F / normalizedRadius);
            constants.cameraDistance = normalizedRadius * settings.planetRadius;
            constants.cameraYaw = std::atan2(radial.x, radial.z);
            constants.cameraPitch = std::asin(std::clamp(radial.y, -1.0F, 1.0F));
#endif
        } else {
            constants.cameraDistance = std::max(
                settings.cameraDistance,
                minimumCameraDistance(settings.planetRadius, planetTopology_.outerBoundingRadius));
            constants.cameraYaw = settings.cameraYaw;
            constants.cameraPitch = settings.cameraPitch;
        }
    } else {
        constants.cameraDistance = std::max(
            settings.cameraDistance,
            minimumCameraDistance(settings.planetRadius, planetTopology_.outerBoundingRadius));
        constants.cameraYaw = settings.cameraYaw;
        constants.cameraPitch = settings.cameraPitch;
    }
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
    // The camera-centred virtual brick cache retains its leaf level until a
    // projected leaf crosses the 1.6 px split or 0.8 px merge boundary. The
    // global parent/child morph is continuous, so a split at 1.6 px is exactly
    // the same field as its 0.8 px child and cannot pop or crack.
    const float cameraAltitude = std::max(
        constants.cameraDistance -
            settings.planetRadius * kFractalPlanetSeaRadiusScale,
        settings.planetRadius * 1e-5F);
    const float projectedParentPixels =
        settings.planetRadius * kFractalPlanetParentAngularWidth *
        (static_cast<float>(swapchainExtent_.height) * 0.5F) /
        cameraAltitude;
    const std::uint32_t configuredMaximum = std::min(
        settings.fractalPlanetSdfMaximumLevels,
        kFractalPlanetMaximumLevels);
    if (!fractalVoxelCacheInitialized_) {
        fractalVoxelCacheLevel_ = 0U;
        fractalVoxelCacheInitialized_ = true;
    }
    auto projectedLeafPixels = [&]() {
        return projectedParentPixels /
            static_cast<float>(std::uint32_t{1U} << fractalVoxelCacheLevel_);
    };
    while (fractalVoxelCacheLevel_ < configuredMaximum &&
           projectedLeafPixels() > 1.6F) {
        ++fractalVoxelCacheLevel_;
        ++fractalVoxelRefineCount_;
        fractalVoxelCacheDirty_ = true;
    }
    while (fractalVoxelCacheLevel_ > 0U &&
           projectedLeafPixels() < kFractalVoxelMergePixels) {
        --fractalVoxelCacheLevel_;
        ++fractalVoxelMergeCount_;
        fractalVoxelCacheDirty_ = true;
    }
    fractalVoxelProjectedPixels_ = projectedLeafPixels();
    if (fractalVoxelCacheLevel_ < configuredMaximum) {
        const float transitionT = std::clamp(
            (fractalVoxelProjectedPixels_ -
             settings.fractalPlanetSdfTargetPixels) /
                std::max(1.6F - settings.fractalPlanetSdfTargetPixels, 0.05F),
            0.0F, 1.0F);
        fractalVoxelTransition_ = transitionT * transitionT *
            (3.0F - 2.0F * transitionT);
    } else {
        fractalVoxelTransition_ = 0.0F;
    }
    stats_.fractalVoxelCacheLevel = fractalVoxelCacheLevel_;
    stats_.fractalVoxelRefineCount = fractalVoxelRefineCount_;
    stats_.fractalVoxelMergeCount = fractalVoxelMergeCount_;
    stats_.fractalVoxelProjectedPixels = fractalVoxelProjectedPixels_;
    stats_.fractalVoxelTransition = fractalVoxelTransition_;
    const float cacheCosinePitch = std::cos(constants.cameraPitch);
    const std::array<float, 3> cacheRadial{
        std::sin(constants.cameraYaw) * cacheCosinePitch,
        std::sin(constants.cameraPitch),
        std::cos(constants.cameraYaw) * cacheCosinePitch};
    const std::uint32_t morphQuantized = static_cast<std::uint32_t>(
        std::lround(std::clamp(fractalVoxelTransition_, 0.0F, 1.0F) * 32.0F));
    const float radialDot = cacheRadial[0] * fractalVoxelCacheRadial_[0] +
        cacheRadial[1] * fractalVoxelCacheRadial_[1] +
        cacheRadial[2] * fractalVoxelCacheRadial_[2];
    const float cacheAngularRadius =
        kFractalPlanetParentAngularWidth /
        static_cast<float>(std::uint32_t{1U} << fractalVoxelCacheLevel_) *
        300.0F;
    if (morphQuantized != fractalVoxelCacheMorphQuantized_ ||
        radialDot < 1.0F - 0.02F * cacheAngularRadius * cacheAngularRadius) {
        fractalVoxelCacheDirty_ = true;
        fractalVoxelCacheMorphQuantized_ = morphQuantized;
        fractalVoxelCacheRadial_ = cacheRadial;
    }
#endif
    constants.voxelResolution = kVoxelResolution;
    constants.brushNdcX = settings.brushNdcX;
    constants.brushNdcY = settings.brushNdcY;
    constants.editMode = settings.editMode;
    constants.brushMaterial = settings.brushMaterial;
    constants.simulationTick = simulationTick_;
    constants.visualizationMode = kAdaptiveSdfBuild
        ? packVisualizationAndAdaptiveSdf(
              settings.visualizationMode, settings.adaptivePlanetSdfEnabled,
              settings.adaptivePlanetSdfMaximumLevels,
              settings.adaptivePlanetSdfTargetPixels,
              settings.adaptivePlanetSdfDebugMode,
              settings.adaptivePlanetSdfLayerFraction,
              settings.adaptivePlanetSdfComparisonView)
        : kFractalPlanetSdfBuild
              ? packVisualizationAndAdaptiveSdf(
                    settings.visualizationMode,
                    settings.fractalPlanetSdfEnabled,
                    kFractalVoxelSdfBuild ? fractalVoxelCacheLevel_
                                          : settings.fractalPlanetSdfMaximumLevels,
                    kFractalVoxelSdfBuild ? fractalVoxelTransition_ * 31.875F
                                          : settings.fractalPlanetSdfTargetPixels,
                    settings.fractalPlanetSdfDebugMode,
                    settings.fractalPlanetSdfMicroRelief / 0.09F,
                    kFractalVoxelSdfBuild
                        ? settings.fractalVoxelCavityDebug
                        : false)
              : settings.visualizationMode;
    constants.surfaceTileCount = static_cast<std::uint32_t>(planetTopology_.tiles.size());
    constants.residentPageCount = planetTopology_.residentPageCount();
    constants.planetOuterScale = planetTopology_.outerBoundingRadius;
    constants.geodesicTraversalMode = kEightPlanetSystemLabBuild
        ? settings.geodesicTraversalMode
        : kReferenceTraversalEnabled
                                          ? settings.geodesicTraversalMode
                                          : std::clamp(settings.geodesicTraversalMode, 2U,
                                                       kScaleLabBuild ? 8U : 7U);
#if VOXEL_EIGHT_PLANET_SYSTEM_LAB
    const auto& system = settings.eightPlanetSystem;
    constants.systemStar = {system.starRadius, system.starRadiance,
                            settings.timeScale, 0.0F};
    constants.systemStarColor = {system.starColor[0], system.starColor[1],
                                 system.starColor[2], 0.0F};
    constants.systemAmbient = {system.ambientColor[0], system.ambientColor[1],
                               system.ambientColor[2], system.ambientStrength};
    const std::uint32_t systemDebugView =
        settings.visualizationMode == 106U ? 4U
        : settings.visualizationMode == 103U ? 2U
                                            : system.debugView;
    constants.systemControl = {system.shadowSamples, systemDebugView,
                               system.selectedPlanet, 0U};
    std::copy_n(system.planetRadii.begin(), 4U,
                constants.systemPlanetRadii0.begin());
    std::copy_n(system.planetRadii.begin() + 4, 4U,
                constants.systemPlanetRadii1.begin());
    std::copy_n(system.planetSeeds.begin(), 4U,
                constants.systemPlanetSeeds0.begin());
    std::copy_n(system.planetSeeds.begin() + 4, 4U,
                constants.systemPlanetSeeds1.begin());
#endif
    ++pageFeedbackGeneration_;
    if (pageFeedbackGeneration_ == 0U) {
        ++pageFeedbackGeneration_;
    }
    constants.streamSamplePhase = pageFeedbackGeneration_;
    constants.terrainSeed = settings.terrain.seed;
    constants.terrainContinentScale = settings.terrain.continentScale;
    constants.terrainContinentStrength = settings.terrain.continentStrength;
    constants.terrainMountainScale = settings.terrain.mountainScale;
    constants.terrainMountainStrength = settings.terrain.mountainStrength;
    constants.terrainRoughness = settings.terrain.roughness;
    constants.terrainOceanLevel = settings.terrain.oceanLevel;
    constants.terrainPolarStrength = settings.terrain.polarStrength;
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
    // The true-fractal field only consumes terrainSeed. Reuse the otherwise
    // dormant terrain-generation floats for lab-only post-hit shading without
    // expanding the already-full 128-byte Vulkan push-constant block.
    const float sunStrength = std::clamp(
        settings.fractalVoxelSunStrength, 0.0F, 2.0F);
    const float sunElevation = std::clamp(
        settings.fractalVoxelSunElevation, 0.08726646F, 1.48352986F);
    const float sunElevationCosine = std::cos(sunElevation);
    constants.terrainContinentScale = sunStrength * sunElevationCosine *
        std::sin(settings.fractalVoxelSunAzimuth);
    constants.terrainContinentStrength = sunStrength * std::sin(sunElevation);
    constants.terrainMountainScale = sunStrength * sunElevationCosine *
        std::cos(settings.fractalVoxelSunAzimuth);
    constants.terrainMountainStrength = std::clamp(
        settings.fractalVoxelAmbientStrength, 0.05F, 0.65F);
    constants.terrainRoughness = std::clamp(
        settings.fractalVoxelCavityStrength, 0.0F, 0.45F);
    constants.terrainOceanLevel = settings.fractalVoxelCellEdgesEnabled
        ? std::clamp(settings.fractalVoxelCellEdgeStrength, 0.0F, 0.40F)
        : 0.0F;
    // Sign is a lab-only geometry selector: positive means a true half-open
    // constant-height leaf surface, negative retains smooth reconstruction
    // for A/B. The magnitude still carries normal-detail strength.
    const float encodedNormalSharpness = 1.0F + std::clamp(
        settings.fractalVoxelNormalSharpness, 0.0F, 1.0F);
    constants.terrainPolarStrength = settings.fractalVoxelDiscreteSurface
        ? encodedNormalSharpness : -encodedNormalSharpness;
#endif
    lightingConstants_ = constants;
    if constexpr (!kScaleLabBuild && !kAdaptiveSdfBuild &&
                  !kFractalPlanetSdfBuild) {
        const auto& lighting = settings.lighting;
        lightingConstants_.terrainContinentScale = lighting.sunAzimuth;
        lightingConstants_.terrainContinentStrength = std::clamp(
            lighting.sunElevation, 0.01745329F, 1.55334306F);
        lightingConstants_.terrainMountainScale = std::clamp(
            lighting.sunIntensity, 0.0F, 3.0F);
        lightingConstants_.terrainMountainStrength = std::clamp(
            lighting.skyIntensity, 0.0F, 1.25F);
        lightingConstants_.terrainRoughness = std::clamp(
            lighting.roughness, 0.12F, 1.0F);
        lightingConstants_.terrainOceanLevel = std::clamp(
            lighting.exposure, 0.25F, 2.5F);
        lightingConstants_.terrainPolarStrength = std::clamp(
            lighting.contactShadowStrength, 0.0F, 0.65F);
        const auto& atmosphere = settings.atmosphere;
        lightingConstants_.terrainSeed = packLightingRgb8(
            lighting.sunColor, packAtmosphereTerrainSeedHighBits(
                atmosphere, std::min(lighting.debugMode, 3U)));
        lightingConstants_.brushMaterial = packLightingRgb8(
            lighting.skyZenithColor,
            quantizeAtmosphere(atmosphere.endHeight,
                               kAtmosphereMaximumEndHeight, 255U));
        lightingConstants_.editMode = packLightingRgb8(
            lighting.skyHorizonColor,
            quantizeAtmosphere(atmosphere.density,
                               kAtmosphereMaximumDensity, 255U));
        const auto quantized = [](float value, float maximum) {
            return static_cast<std::uint32_t>(std::lround(
                std::clamp(value / maximum, 0.0F, 1.0F) * 255.0F));
        };
        lightingConstants_.simulationTick =
            quantized(lighting.specularStrength, 1.0F) |
            (quantized(lighting.rimStrength, 0.25F) << 8U) |
            packSpaceEnvironmentFlags(settings.spaceEnvironment) |
            packAtmosphereSimulationFlags(atmosphere);
        lightingConstants_.brushNdcX = std::bit_cast<float>(
            packAtmosphereRenderWord0(
                settings.spaceEnvironment.brightness,
                settings.spaceEnvironment.size, atmosphere));
        lightingConstants_.brushNdcY = std::bit_cast<float>(
            packAtmosphereRenderWord1(atmosphere));
        lightingConstants_.surfaceTileCount = packCloudWord0(settings.clouds);
        lightingConstants_.residentPageCount = packCloudWord1(settings.clouds);
        lightingConstants_.streamSamplePhase =
            packAtmosphereIntegrationPhaseWord(
                packCloudPhaseWord(constants.streamSamplePhase,
                                   settings.clouds),
                atmosphere);
    }
    stats_.geodesicTraversalMode = constants.geodesicTraversalMode;
    stats_.streamRequestSamples = settings.visualizationMode != 0U
                                      ? kStreamRequestSamples : 0U;
    if (settings.regenerateTerrain) {
        std::ranges::fill(freshCollisionColumns_, 0U);
    }
    recordCommands(imageIndex, constants, settings.simulationSteps,
                   settings.resetVoxelVolume ||
                       (kFractalVoxelSdfBuild && fractalVoxelCacheDirty_),
                   settings.regenerateTerrain);
#if VOXEL_FRACTAL_VOXEL_SDF_LAB
    fractalVoxelCacheDirty_ = false;
#endif

    constexpr VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    VkSubmitInfo submitInfo{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = &imageAvailable_;
    submitInfo.pWaitDstStageMask = &waitStage;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer_;
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = &renderingFinished_[imageIndex];
    checkVk(vkQueueSubmit(queue_, 1, &submitInfo, frameFence_), "Submitting frame");
    timestampPending_ = timestampsSupported_;
    pageFeedbackPending_ = true;
    artifactDiagnosticPending_ = kScaleLabBuild || kGlobalMetricLabBuild ||
        constants.geodesicTraversalMode == 5U ||
        constants.geodesicTraversalMode == 6U ||
        constants.geodesicTraversalMode == 8U;

    VkPresentInfoKHR presentInfo{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = &renderingFinished_[imageIndex];
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swapchain_;
    presentInfo.pImageIndices = &imageIndex;
    const VkResult presentResult = vkQueuePresentKHR(queue_, &presentInfo);
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR ||
        acquireResult == VK_SUBOPTIMAL_KHR) {
        resizeRequested_ = true;
    } else if (presentResult != VK_SUCCESS) {
        checkVk(presentResult, "Presenting frame");
    }
}

SurfaceTerrainContact VulkanRenderer::querySurfaceTerrain(
    const std::array<float, 3>& radial, float planetRadius,
    float capsuleRadius, float sweepDistance) const noexcept {
    return queryGeodesicTerrain(
        planetTopology_, freshCollisionColumns_, radial,
        planetRadius, capsuleRadius, sweepDistance,
        kAdaptiveSdfBuild && adaptiveSdfEnabled_, adaptiveSdfSeed_,
        adaptiveSdfMaximumLevels_, adaptiveSdfLayerFraction_,
        kFractalPlanetSdfBuild && fractalPlanetSdfEnabled_,
        fractalPlanetSdfSeed_, fractalPlanetSdfMaximumLevels_,
        fractalPlanetSdfMicroRelief_);
}

void VulkanRenderer::recordCommands(
    std::uint32_t imageIndex,
    const FrameConstants& constants,
    std::uint32_t simulationSteps,
    bool resetVoxelVolume,
    bool regenerateTerrain) {
    VkCommandBufferBeginInfo beginInfo{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    checkVk(vkBeginCommandBuffer(commandBuffer_, &beginInfo), "Beginning frame commands");

    vkCmdFillBuffer(commandBuffer_, geodesicPageRequestBuffer_, 0,
                    2U * sizeof(std::uint32_t), 0U);
    if (constants.cameraMode == 1U) {
        const std::uint32_t reservedCollisionProfiles =
            kCollisionProfileRequestCount;
        vkCmdUpdateBuffer(commandBuffer_, geodesicPageRequestBuffer_, 0,
                          sizeof(reservedCollisionProfiles),
                          &reservedCollisionProfiles);
    }
    const bool diagnosticsEnabled = constants.geodesicTraversalMode == 5U ||
        constants.geodesicTraversalMode == 6U ||
        constants.geodesicTraversalMode == 8U;
    const bool diagnosticBufferWritten = diagnosticsEnabled || kScaleLabBuild ||
        kGlobalMetricLabBuild;
    if (diagnosticBufferWritten) {
        vkCmdFillBuffer(commandBuffer_, artifactDiagnosticBuffer_, 0,
                        sizeof(ArtifactDiagnosticHeaderGpu), 0U);
    }

    const bool hasPageUploads = !pendingPageUploads_.empty();
    if (hasPageUploads) {
        std::vector<VkBufferCopy> brickCopies;
        std::vector<VkBufferCopy> pageTableCopies;
        brickCopies.reserve(pendingPageUploads_.size());
        pageTableCopies.reserve(pendingPageUploads_.size() * 2U);
        const VkDeviceSize physicalPageBytes =
            static_cast<VkDeviceSize>(planetTopology_.brickResolution) * sizeof(std::uint32_t);
        for (const PendingPageUpload& pending : pendingPageUploads_) {
            brickCopies.push_back({
                pending.pageSourceOffset,
                static_cast<VkDeviceSize>(pending.physicalPage) * physicalPageBytes,
                physicalPageBytes});
            pageTableCopies.push_back({
                pending.incomingEntrySourceOffset,
                static_cast<VkDeviceSize>(pending.incomingTile) *
                    sizeof(GeodesicPageTableEntryGpu),
                sizeof(GeodesicPageTableEntryGpu)});
            pageTableCopies.push_back({
                pending.evictedEntrySourceOffset,
                static_cast<VkDeviceSize>(pending.evictedTile) *
                    sizeof(GeodesicPageTableEntryGpu),
                sizeof(GeodesicPageTableEntryGpu)});
        }
        vkCmdCopyBuffer(commandBuffer_, geodesicStreamUploadBuffer_, tileBrickBuffer_,
                        static_cast<std::uint32_t>(brickCopies.size()), brickCopies.data());
        vkCmdCopyBuffer(commandBuffer_, geodesicStreamUploadBuffer_, geodesicPageTableBuffer_,
                        static_cast<std::uint32_t>(pageTableCopies.size()), pageTableCopies.data());
    }

    std::array<VkBufferMemoryBarrier, 4> streamingReady{};
    std::uint32_t streamingBarrierCount = 1U;
    for (auto& barrier : streamingReady) {
        barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    }
    streamingReady[0].buffer = geodesicPageRequestBuffer_;
    streamingReady[0].size = 2U * sizeof(std::uint32_t);
    if (diagnosticBufferWritten) {
        streamingReady[streamingBarrierCount].buffer = artifactDiagnosticBuffer_;
        streamingReady[streamingBarrierCount].size =
            sizeof(ArtifactDiagnosticHeaderGpu);
        ++streamingBarrierCount;
    }
    if (hasPageUploads) {
        streamingReady[streamingBarrierCount].buffer = tileBrickBuffer_;
        streamingReady[streamingBarrierCount].size = VK_WHOLE_SIZE;
        ++streamingBarrierCount;
        streamingReady[streamingBarrierCount].buffer = geodesicPageTableBuffer_;
        streamingReady[streamingBarrierCount].size = VK_WHOLE_SIZE;
        ++streamingBarrierCount;
    }
    vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                         0, nullptr, streamingBarrierCount, streamingReady.data(), 0, nullptr);
    pendingPageUploads_.clear();

    if (regenerateTerrain) {
        VkDescriptorSet terrainSet = computeDescriptorSets_[activeCellBuffer_];
        if constexpr (kScaleLabBuild) {
            vkCmdFillBuffer(commandBuffer_, geodesicFarFieldBuffer_, 0,
                            kFarFieldBytes, 0U);
        }
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                          geodesicTerrainInitPipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                                computePipelineLayout_, 0, 1, &terrainSet, 0, nullptr);
        vkCmdPushConstants(commandBuffer_, computePipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(constants), &constants);
        vkCmdDispatch(commandBuffer_, (constants.surfaceTileCount + 255U) / 256U, 1U, 1U);

        if constexpr (kScaleLabBuild) {
            std::array<VkBufferMemoryBarrier, 2> terrainToHierarchy{};
            terrainToHierarchy[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            terrainToHierarchy[0].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            terrainToHierarchy[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            terrainToHierarchy[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            terrainToHierarchy[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            terrainToHierarchy[0].buffer = geodesicColumnStateBuffer_;
            terrainToHierarchy[0].size = VK_WHOLE_SIZE;
            terrainToHierarchy[1] = terrainToHierarchy[0];
            terrainToHierarchy[1].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            terrainToHierarchy[1].buffer = geodesicFarFieldBuffer_;
            terrainToHierarchy[1].size = kFarFieldBytes;
            vkCmdPipelineBarrier(commandBuffer_,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
                                     VK_PIPELINE_STAGE_TRANSFER_BIT,
                                 VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                 0, nullptr,
                                 static_cast<std::uint32_t>(terrainToHierarchy.size()),
                                 terrainToHierarchy.data(), 0, nullptr);
            vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                              geodesicFarFieldInitPipeline_);
            FrameConstants hierarchyConstants = constants;
            for (std::uint32_t pass = 0U; pass < kScaleLabHierarchyPasses; ++pass) {
                const std::uint32_t level = pass == 0U
                    ? kScaleLabAoInitLevel
                    : (pass == 1U ? kFarFieldLodLevels : pass - 2U);
                hierarchyConstants.maxMarchSteps = level;
                vkCmdPushConstants(commandBuffer_, computePipelineLayout_,
                                   VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(hierarchyConstants),
                                   &hierarchyConstants);
                std::uint32_t itemCount = hierarchyConstants.surfaceTileCount;
                if (level == kFarFieldLodLevels) {
                    itemCount = farFieldNodeCount();
                } else if (level > 0U && level < kFarFieldLodLevels) {
                    itemCount = std::max(kTileLookupWidth >> level, 1U) *
                                std::max(kTileLookupHeight >> level, 1U);
                }
                vkCmdDispatch(commandBuffer_, (itemCount + 255U) / 256U, 1U, 1U);
                std::array<VkBufferMemoryBarrier, 3> hierarchyLevelReady{};
                hierarchyLevelReady[0].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
                hierarchyLevelReady[0].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
                hierarchyLevelReady[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT |
                                                       VK_ACCESS_SHADER_WRITE_BIT;
                hierarchyLevelReady[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                hierarchyLevelReady[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                hierarchyLevelReady[0].buffer = geodesicFarFieldBuffer_;
                hierarchyLevelReady[0].size = kFarFieldBytes;
                hierarchyLevelReady[1] = hierarchyLevelReady[0];
                hierarchyLevelReady[1].buffer = geodesicMacroHierarchyBuffer_;
                hierarchyLevelReady[1].size = kMacroHierarchyBytes;
                hierarchyLevelReady[2] = hierarchyLevelReady[0];
                hierarchyLevelReady[2].buffer = geodesicColumnStateBuffer_;
                hierarchyLevelReady[2].size = VK_WHOLE_SIZE;
                vkCmdPipelineBarrier(commandBuffer_,
                                     VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                     VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                     0, nullptr,
                                     static_cast<std::uint32_t>(hierarchyLevelReady.size()),
                                     hierarchyLevelReady.data(), 0, nullptr);
            }
        }

        std::array<VkBufferMemoryBarrier, 4> terrainReady{};
        for (auto& barrier : terrainReady) {
            barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT |
                                    VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        }
        terrainReady[0].buffer = geodesicColumnStateBuffer_;
        terrainReady[0].size = VK_WHOLE_SIZE;
        terrainReady[1].buffer = tileBrickBuffer_;
        terrainReady[1].size = VK_WHOLE_SIZE;
        terrainReady[2] = terrainReady[0];
        terrainReady[2].buffer = geodesicFarFieldBuffer_;
        terrainReady[2].size = VK_WHOLE_SIZE;
        terrainReady[3] = terrainReady[0];
        terrainReady[3].buffer = geodesicMacroHierarchyBuffer_;
        terrainReady[3].size = VK_WHOLE_SIZE;
        const std::uint32_t terrainBarrierCount = kScaleLabBuild ? 4U : 2U;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
                                 VK_PIPELINE_STAGE_TRANSFER_BIT,
                             0, 0, nullptr, terrainBarrierCount,
                             terrainReady.data(), 0, nullptr);
        ++stats_.terrainGeneration;
        stats_.terrainSeed = constants.terrainSeed;
    }

    std::uint32_t renderBufferIndex = activeCellBuffer_;
    if (resetVoxelVolume) {
        vkCmdFillBuffer(commandBuffer_, occupancyBuffer_, 0, kOccupancyBufferSize, 0U);
        VkBufferMemoryBarrier occupancyCleared{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        occupancyCleared.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        occupancyCleared.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
        occupancyCleared.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        occupancyCleared.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        occupancyCleared.buffer = occupancyBuffer_;
        occupancyCleared.size = kOccupancyBufferSize;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             0, nullptr, 1, &occupancyCleared, 0, nullptr);

        VkDescriptorSet resetSet = computeDescriptorSets_[activeCellBuffer_];
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, voxelInitPipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout_,
                                0, 1, &resetSet, 0, nullptr);
        vkCmdPushConstants(commandBuffer_, computePipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(constants), &constants);
        constexpr std::uint32_t resetGroupCount = (kVoxelResolution + 3U) / 4U;
        vkCmdDispatch(commandBuffer_, resetGroupCount, resetGroupCount, resetGroupCount);

        std::array<VkBufferMemoryBarrier, 2> resetReady{};
        for (auto& barrier : resetReady) {
            barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT |
                                    VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        }
        resetReady[0].buffer = cellBuffers_[activeCellBuffer_];
        resetReady[0].size = kCellBufferSize;
        resetReady[1].buffer = occupancyBuffer_;
        resetReady[1].size = kOccupancyBufferSize;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
                             0, 0, nullptr, static_cast<std::uint32_t>(resetReady.size()),
                             resetReady.data(), 0, nullptr);
        simulationTick_ = 0;
        stats_.simulationTick = 0;
    }
    if (constants.editMode != 0U) {
        VkDescriptorSet editSet = computeDescriptorSets_[activeCellBuffer_];
        const VkPipeline editPipeline = constants.visualizationMode == 0U
                                            ? voxelEditPipeline_
                                            : geodesicEditPipeline_;
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, editPipeline);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout_,
                                0, 1, &editSet, 0, nullptr);
        vkCmdPushConstants(commandBuffer_, computePipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(constants), &constants);
        vkCmdDispatch(commandBuffer_, 1, 1, 1);

        std::array<VkBufferMemoryBarrier, 4> editReady{};
        for (auto& barrier : editReady) {
            barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT |
                                    VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        }
        editReady[0].buffer = cellBuffers_[activeCellBuffer_];
        editReady[0].size = kCellBufferSize;
        editReady[1].buffer = occupancyBuffer_;
        editReady[1].size = kOccupancyBufferSize;
        editReady[2].buffer = tileBrickBuffer_;
        editReady[2].size = static_cast<VkDeviceSize>(
            planetTopology_.residentBrickCellCapacity() * sizeof(std::uint32_t));
        editReady[3].buffer = geodesicColumnStateBuffer_;
        editReady[3].size = static_cast<VkDeviceSize>(
            planetTopology_.columnStates.size() * sizeof(GeodesicColumnStateGpu));
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
                             0, 0, nullptr, static_cast<std::uint32_t>(editReady.size()),
                             editReady.data(), 0, nullptr);
    }
    for (std::uint32_t step = 0; step < simulationSteps; ++step) {
        vkCmdFillBuffer(commandBuffer_, claimBuffer_, 0, kCellBufferSize, 0xffffffffU);
        vkCmdFillBuffer(commandBuffer_, occupancyBuffer_, 0, kOccupancyBufferSize, 0U);

        std::array<VkBufferMemoryBarrier, 2> clearedBarriers{};
        for (auto& barrier : clearedBarriers) {
            barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        }
        clearedBarriers[0].buffer = claimBuffer_;
        clearedBarriers[0].size = kCellBufferSize;
        clearedBarriers[1].buffer = occupancyBuffer_;
        clearedBarriers[1].size = kOccupancyBufferSize;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             0, nullptr, static_cast<std::uint32_t>(clearedBarriers.size()),
                             clearedBarriers.data(), 0, nullptr);

        FrameConstants simulationConstants = constants;
        simulationConstants.simulationTick = simulationTick_;
        constexpr std::uint32_t groupCount = (kVoxelResolution + 3U) / 4U;
        VkDescriptorSet simulationSet = computeDescriptorSets_[activeCellBuffer_];
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, sandClaimPipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout_,
                                0, 1, &simulationSet, 0, nullptr);
        vkCmdPushConstants(commandBuffer_, computePipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(simulationConstants), &simulationConstants);
        vkCmdDispatch(commandBuffer_, groupCount, groupCount, groupCount);

        VkBufferMemoryBarrier claimReady{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        claimReady.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        claimReady.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        claimReady.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        claimReady.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        claimReady.buffer = claimBuffer_;
        claimReady.size = kCellBufferSize;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             0, nullptr, 1, &claimReady, 0, nullptr);

        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, sandCommitPipeline_);
        vkCmdDispatch(commandBuffer_, groupCount, groupCount, groupCount);

        std::array<VkBufferMemoryBarrier, 2> simulationReady{};
        for (auto& barrier : simulationReady) {
            barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
            barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        }
        simulationReady[0].buffer = cellBuffers_[1U - activeCellBuffer_];
        simulationReady[0].size = kCellBufferSize;
        simulationReady[1].buffer = occupancyBuffer_;
        simulationReady[1].size = kOccupancyBufferSize;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             0, nullptr, static_cast<std::uint32_t>(simulationReady.size()),
                             simulationReady.data(), 0, nullptr);

        renderBufferIndex = 1U - activeCellBuffer_;
        activeCellBuffer_ = renderBufferIndex;
        ++simulationTick_;
        stats_.simulationTick = simulationTick_;
    }

    VkImageMemoryBarrier outputToGeneral{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    outputToGeneral.srcAccessMask = outputImageInitialized_ ? VK_ACCESS_TRANSFER_READ_BIT : 0;
    outputToGeneral.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    outputToGeneral.oldLayout = outputImageInitialized_ ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL
                                                       : VK_IMAGE_LAYOUT_UNDEFINED;
    outputToGeneral.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    outputToGeneral.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    outputToGeneral.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    outputToGeneral.image = outputImage_;
    outputToGeneral.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    outputToGeneral.subresourceRange.levelCount = 1;
    outputToGeneral.subresourceRange.layerCount = 1;
    vkCmdPipelineBarrier(commandBuffer_,
                         outputImageInitialized_ ? VK_PIPELINE_STAGE_TRANSFER_BIT : VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &outputToGeneral);

    if (timestampsSupported_) {
        vkCmdResetQueryPool(commandBuffer_, timestampQueryPool_, 0U, 3U);
        vkCmdWriteTimestamp(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                            timestampQueryPool_, 0U);
    }
    if (constants.visualizationMode != 0U) {
        VkDescriptorSet requestSet = computeDescriptorSets_[renderBufferIndex];
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                          geodesicStreamRequestPipeline_);
        vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                                computePipelineLayout_, 0, 1, &requestSet, 0, nullptr);
        vkCmdPushConstants(commandBuffer_, computePipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                           0, sizeof(constants), &constants);
        vkCmdDispatch(commandBuffer_, (kStreamRequestSamples + 255U) / 256U, 1U, 1U);

        VkBufferMemoryBarrier requestDedupReady{VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        requestDedupReady.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        requestDedupReady.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        requestDedupReady.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        requestDedupReady.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        requestDedupReady.buffer = geodesicPageTableBuffer_;
        requestDedupReady.size = VK_WHOLE_SIZE;
        vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                             0, nullptr, 1, &requestDedupReady, 0, nullptr);
    }
    if (timestampsSupported_) {
        vkCmdWriteTimestamp(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                            timestampQueryPool_, 1U);
    }
    vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, computePipeline_);
    VkDescriptorSet renderSet = computeDescriptorSets_[renderBufferIndex];
    vkCmdBindDescriptorSets(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE, computePipelineLayout_,
                            0, 1, &renderSet, 0, nullptr);
    vkCmdPushConstants(commandBuffer_, computePipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT,
                       0, sizeof(lightingConstants_), &lightingConstants_);
    vkCmdDispatch(commandBuffer_, (swapchainExtent_.width + 7U) / 8U,
                  (swapchainExtent_.height + 7U) / 8U, 1);
    if constexpr (kGlobalMetricLabBuild) {
        // The first pass appends only potentially discontinuous pixels to the
        // reverse end of the existing diagnostic buffer.  This barrier keeps
        // both the queue and central image visible before the sparse 4x pass;
        // no full-screen AA shader executes.
        VkBufferMemoryBarrier aaQueueReady{
            VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        aaQueueReady.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        aaQueueReady.dstAccessMask = VK_ACCESS_SHADER_READ_BIT |
                                     VK_ACCESS_SHADER_WRITE_BIT |
                                     VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
        aaQueueReady.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        aaQueueReady.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        aaQueueReady.buffer = artifactDiagnosticBuffer_;
        aaQueueReady.size = VK_WHOLE_SIZE;
        VkImageMemoryBarrier aaImageReady{
            VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        aaImageReady.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        aaImageReady.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        aaImageReady.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
        aaImageReady.newLayout = VK_IMAGE_LAYOUT_GENERAL;
        aaImageReady.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        aaImageReady.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        aaImageReady.image = outputImage_;
        aaImageReady.subresourceRange = outputToGeneral.subresourceRange;
        vkCmdPipelineBarrier(commandBuffer_,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
                                 VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT, 0,
                             0, nullptr, 1, &aaQueueReady, 1, &aaImageReady);
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                          globalSpatialAaPipeline_);
        vkCmdBindDescriptorSets(commandBuffer_,
                                VK_PIPELINE_BIND_POINT_COMPUTE,
                                computePipelineLayout_, 0, 1,
                                &renderSet, 0, nullptr);
        vkCmdPushConstants(commandBuffer_, computePipelineLayout_,
                           VK_SHADER_STAGE_COMPUTE_BIT, 0,
                           sizeof(lightingConstants_), &lightingConstants_);
        constexpr VkDeviceSize aaIndirectOffset =
            offsetof(ArtifactDiagnosticHeaderGpu, scaleDdaHistogram) +
            12U * sizeof(std::uint32_t);
        vkCmdDispatchIndirect(commandBuffer_, artifactDiagnosticBuffer_,
                              aaIndirectOffset);
        VkBufferMemoryBarrier aaSamplesReady{
            VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER};
        aaSamplesReady.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        aaSamplesReady.dstAccessMask = VK_ACCESS_SHADER_READ_BIT |
                                       VK_ACCESS_SHADER_WRITE_BIT |
                                       VK_ACCESS_INDIRECT_COMMAND_READ_BIT;
        aaSamplesReady.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        aaSamplesReady.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        aaSamplesReady.buffer = artifactDiagnosticBuffer_;
        aaSamplesReady.size = VK_WHOLE_SIZE;
        vkCmdPipelineBarrier(commandBuffer_,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT |
                                 VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT, 0,
                             0, nullptr, 1, &aaSamplesReady, 0, nullptr);
        vkCmdBindPipeline(commandBuffer_, VK_PIPELINE_BIND_POINT_COMPUTE,
                          globalSpatialAaResolvePipeline_);
        vkCmdBindDescriptorSets(commandBuffer_,
                                VK_PIPELINE_BIND_POINT_COMPUTE,
                                computePipelineLayout_, 0, 1,
                                &renderSet, 0, nullptr);
        vkCmdPushConstants(commandBuffer_, computePipelineLayout_,
                           VK_SHADER_STAGE_COMPUTE_BIT, 0,
                           sizeof(lightingConstants_), &lightingConstants_);
        constexpr VkDeviceSize aaResolveIndirectOffset =
            offsetof(ArtifactDiagnosticHeaderGpu, scaleDdaHistogram) +
            15U * sizeof(std::uint32_t);
        vkCmdDispatchIndirect(commandBuffer_, artifactDiagnosticBuffer_,
                              aaResolveIndirectOffset);
    }
    if (timestampsSupported_) {
        vkCmdWriteTimestamp(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                            timestampQueryPool_, 2U);
    }

    std::array<VkBufferMemoryBarrier, 2> feedbackReady{};
    for (auto& barrier : feedbackReady) {
        barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    }
    feedbackReady[0].buffer = geodesicPageRequestBuffer_;
    feedbackReady[0].size = VK_WHOLE_SIZE;
    std::uint32_t feedbackBarrierCount = 1U;
    if (diagnosticBufferWritten) {
        feedbackReady[1].buffer = artifactDiagnosticBuffer_;
        feedbackReady[1].size = VK_WHOLE_SIZE;
        feedbackBarrierCount = 2U;
    }
    vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_HOST_BIT, 0, 0, nullptr,
                         feedbackBarrierCount,
                         feedbackReady.data(), 0, nullptr);

    VkImageMemoryBarrier outputToTransfer{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    outputToTransfer.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    outputToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    outputToTransfer.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
    outputToTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    outputToTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    outputToTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    outputToTransfer.image = outputImage_;
    outputToTransfer.subresourceRange = outputToGeneral.subresourceRange;

    VkImageMemoryBarrier swapchainToTransfer{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    swapchainToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    swapchainToTransfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    swapchainToTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    swapchainToTransfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    swapchainToTransfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    swapchainToTransfer.image = swapchainImages_[imageIndex];
    swapchainToTransfer.subresourceRange = outputToGeneral.subresourceRange;
    const std::array transferBarriers{outputToTransfer, swapchainToTransfer};
    vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr,
                         static_cast<std::uint32_t>(transferBarriers.size()), transferBarriers.data());

    VkImageBlit blit{};
    blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.srcSubresource.layerCount = 1;
    blit.srcOffsets[1] = {static_cast<std::int32_t>(swapchainExtent_.width),
                          static_cast<std::int32_t>(swapchainExtent_.height), 1};
    blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    blit.dstSubresource.layerCount = 1;
    blit.dstOffsets[1] = blit.srcOffsets[1];
    vkCmdBlitImage(commandBuffer_, outputImage_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                   swapchainImages_[imageIndex], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                   1, &blit, VK_FILTER_NEAREST);

    VkImageMemoryBarrier swapchainToColor{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    swapchainToColor.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    swapchainToColor.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    swapchainToColor.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    swapchainToColor.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    swapchainToColor.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    swapchainToColor.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    swapchainToColor.image = swapchainImages_[imageIndex];
    swapchainToColor.subresourceRange = outputToGeneral.subresourceRange;
    vkCmdPipelineBarrier(commandBuffer_, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                         0, nullptr, 0, nullptr, 1, &swapchainToColor);

    VkRenderPassBeginInfo renderPassInfo{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    renderPassInfo.renderPass = renderPass_;
    renderPassInfo.framebuffer = framebuffers_[imageIndex];
    renderPassInfo.renderArea.extent = swapchainExtent_;
    vkCmdBeginRenderPass(commandBuffer_, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);
    ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer_);
    vkCmdEndRenderPass(commandBuffer_);

    checkVk(vkEndCommandBuffer(commandBuffer_), "Ending frame commands");
    outputImageInitialized_ = true;
}

void VulkanRenderer::recreateSwapchain() {
    int width = 0;
    int height = 0;
    SDL_Vulkan_GetDrawableSize(window_, &width, &height);
    if (width <= 0 || height <= 0) {
        return;
    }
    checkVk(vkDeviceWaitIdle(device_), "Waiting to resize swapchain");
    destroyImGuiBackends();
    for (VkSemaphore semaphore : renderingFinished_) {
        vkDestroySemaphore(device_, semaphore, nullptr);
    }
    renderingFinished_.clear();
    destroySwapchainResources();
    createSwapchain();
    createRenderPass();
    createSwapchainViews();
    createOutputImage();
    createFramebuffers();
    createPresentSemaphores();

    VkDescriptorImageInfo imageInfo{};
    imageInfo.imageView = outputView_;
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    std::array<VkWriteDescriptorSet, 2> writes{};
    for (std::uint32_t index = 0; index < writes.size(); ++index) {
        writes[index].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[index].dstSet = computeDescriptorSets_[index];
        writes[index].dstBinding = 0;
        writes[index].descriptorCount = 1;
        writes[index].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        writes[index].pImageInfo = &imageInfo;
    }
    vkUpdateDescriptorSets(device_, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
    initializeImGuiBackends();
    resizeRequested_ = false;
}

void VulkanRenderer::destroyImGuiBackends() noexcept {
    if (imguiBackendsReady_) {
        ImGui_ImplVulkan_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        imguiBackendsReady_ = false;
    }
}

void VulkanRenderer::destroySwapchainResources() noexcept {
    for (VkFramebuffer framebuffer : framebuffers_) {
        vkDestroyFramebuffer(device_, framebuffer, nullptr);
    }
    framebuffers_.clear();
    if (outputView_ != VK_NULL_HANDLE) {
        vkDestroyImageView(device_, outputView_, nullptr);
        outputView_ = VK_NULL_HANDLE;
    }
    if (outputImage_ != VK_NULL_HANDLE) {
        vkDestroyImage(device_, outputImage_, nullptr);
        outputImage_ = VK_NULL_HANDLE;
    }
    if (outputMemory_ != VK_NULL_HANDLE) {
        vkFreeMemory(device_, outputMemory_, nullptr);
        outputMemory_ = VK_NULL_HANDLE;
    }
    for (VkImageView view : swapchainViews_) {
        vkDestroyImageView(device_, view, nullptr);
    }
    swapchainViews_.clear();
    swapchainImages_.clear();
    if (renderPass_ != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device_, renderPass_, nullptr);
        renderPass_ = VK_NULL_HANDLE;
    }
    if (swapchain_ != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(device_, swapchain_, nullptr);
        swapchain_ = VK_NULL_HANDLE;
    }
    outputImageInitialized_ = false;
}

void VulkanRenderer::shutdown() noexcept {
    if (device_ != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device_);
    }
    destroyImGuiBackends();
    if (imguiContextCreated_) {
        ImGui::DestroyContext();
        imguiContextCreated_ = false;
    }
    if (device_ != VK_NULL_HANDLE && imguiDescriptorPool_ != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device_, imguiDescriptorPool_, nullptr);
    }
    if (device_ != VK_NULL_HANDLE && frameFence_ != VK_NULL_HANDLE) {
        vkDestroyFence(device_, frameFence_, nullptr);
    }
    if (device_ != VK_NULL_HANDLE && timestampQueryPool_ != VK_NULL_HANDLE) {
        vkDestroyQueryPool(device_, timestampQueryPool_, nullptr);
        timestampQueryPool_ = VK_NULL_HANDLE;
    }
    if (device_ != VK_NULL_HANDLE) {
        for (VkSemaphore semaphore : renderingFinished_) {
            vkDestroySemaphore(device_, semaphore, nullptr);
        }
        renderingFinished_.clear();
    }
    if (device_ != VK_NULL_HANDLE && imageAvailable_ != VK_NULL_HANDLE) {
        vkDestroySemaphore(device_, imageAvailable_, nullptr);
    }
    if (device_ != VK_NULL_HANDLE && commandPool_ != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device_, commandPool_, nullptr);
    }
    if (device_ != VK_NULL_HANDLE) {
        destroySwapchainResources();
        if (computePipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, computePipeline_, nullptr);
        }
        if (globalSpatialAaPipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, globalSpatialAaPipeline_, nullptr);
        }
        if (globalSpatialAaResolvePipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, globalSpatialAaResolvePipeline_, nullptr);
        }
        if (voxelInitPipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, voxelInitPipeline_, nullptr);
        }
        if (voxelEditPipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, voxelEditPipeline_, nullptr);
        }
        if (geodesicEditPipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, geodesicEditPipeline_, nullptr);
        }
        if (geodesicTerrainInitPipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, geodesicTerrainInitPipeline_, nullptr);
        }
        if (geodesicFarFieldInitPipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, geodesicFarFieldInitPipeline_, nullptr);
        }
        if (geodesicStreamRequestPipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, geodesicStreamRequestPipeline_, nullptr);
        }
        if (sandClaimPipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, sandClaimPipeline_, nullptr);
        }
        if (sandCommitPipeline_ != VK_NULL_HANDLE) {
            vkDestroyPipeline(device_, sandCommitPipeline_, nullptr);
        }
        if (computePipelineLayout_ != VK_NULL_HANDLE) {
            vkDestroyPipelineLayout(device_, computePipelineLayout_, nullptr);
        }
        if (computeDescriptorPool_ != VK_NULL_HANDLE) {
            vkDestroyDescriptorPool(device_, computeDescriptorPool_, nullptr);
        }
        if (computeDescriptorSetLayout_ != VK_NULL_HANDLE) {
            vkDestroyDescriptorSetLayout(device_, computeDescriptorSetLayout_, nullptr);
        }
        if (occupancyBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, occupancyBuffer_, nullptr);
        }
        if (occupancyMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, occupancyMemory_, nullptr);
        }
        if (claimBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, claimBuffer_, nullptr);
        }
        if (claimMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, claimMemory_, nullptr);
        }
        if (geodesicTileBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicTileBuffer_, nullptr);
        }
        if (geodesicTileMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicTileMemory_, nullptr);
        }
        if (tileLookupBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, tileLookupBuffer_, nullptr);
        }
        if (tileLookupMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, tileLookupMemory_, nullptr);
        }
        if (tileBrickBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, tileBrickBuffer_, nullptr);
        }
        if (tileBrickMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, tileBrickMemory_, nullptr);
        }
        if (geodesicBvhBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicBvhBuffer_, nullptr);
        }
        if (geodesicBvhMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicBvhMemory_, nullptr);
        }
        if (geodesicWideNodeBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicWideNodeBuffer_, nullptr);
        }
        if (geodesicWideNodeMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicWideNodeMemory_, nullptr);
        }
        if (geodesicWideItemBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicWideItemBuffer_, nullptr);
        }
        if (geodesicWideItemMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicWideItemMemory_, nullptr);
        }
        if (geodesicColumnStateBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicColumnStateBuffer_, nullptr);
        }
        if (geodesicColumnStateMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicColumnStateMemory_, nullptr);
        }
        if (geodesicPageTableBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicPageTableBuffer_, nullptr);
        }
        if (geodesicPageTableMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicPageTableMemory_, nullptr);
        }
        if (geodesicPageRequestMapped_ != nullptr) {
            vkUnmapMemory(device_, geodesicPageRequestMemory_);
            geodesicPageRequestMapped_ = nullptr;
        }
        if (geodesicPageRequestBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicPageRequestBuffer_, nullptr);
        }
        if (geodesicPageRequestMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicPageRequestMemory_, nullptr);
        }
        if (geodesicFarFieldBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicFarFieldBuffer_, nullptr);
        }
        if (geodesicFarFieldMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicFarFieldMemory_, nullptr);
        }
        if (geodesicMacroHierarchyBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicMacroHierarchyBuffer_, nullptr);
        }
        if (geodesicMacroHierarchyMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicMacroHierarchyMemory_, nullptr);
        }
        if (eightPlanetHierarchyBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, eightPlanetHierarchyBuffer_, nullptr);
        }
        if (eightPlanetHierarchyMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, eightPlanetHierarchyMemory_, nullptr);
        }
        if (eightPlanetAuthorityMapped_ != nullptr) {
            vkUnmapMemory(device_, eightPlanetAuthorityMemory_);
            eightPlanetAuthorityMapped_ = nullptr;
        }
        if (eightPlanetAuthorityBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, eightPlanetAuthorityBuffer_, nullptr);
        }
        if (eightPlanetAuthorityMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, eightPlanetAuthorityMemory_, nullptr);
        }
        if (artifactDiagnosticMapped_ != nullptr) {
            vkUnmapMemory(device_, artifactDiagnosticMemory_);
            artifactDiagnosticMapped_ = nullptr;
        }
        if (artifactDiagnosticBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, artifactDiagnosticBuffer_, nullptr);
        }
        if (artifactDiagnosticMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, artifactDiagnosticMemory_, nullptr);
        }
        if (portalLabParameterMapped_ != nullptr) {
            vkUnmapMemory(device_, portalLabParameterMemory_);
            portalLabParameterMapped_ = nullptr;
        }
        if (portalLabParameterBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, portalLabParameterBuffer_, nullptr);
        }
        if (portalLabParameterMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, portalLabParameterMemory_, nullptr);
        }
        if (geodesicStreamUploadMapped_ != nullptr) {
            vkUnmapMemory(device_, geodesicStreamUploadMemory_);
            geodesicStreamUploadMapped_ = nullptr;
        }
        if (geodesicStreamUploadBuffer_ != VK_NULL_HANDLE) {
            vkDestroyBuffer(device_, geodesicStreamUploadBuffer_, nullptr);
        }
        if (geodesicStreamUploadMemory_ != VK_NULL_HANDLE) {
            vkFreeMemory(device_, geodesicStreamUploadMemory_, nullptr);
        }
        for (VkBuffer buffer : cellBuffers_) {
            if (buffer != VK_NULL_HANDLE) {
                vkDestroyBuffer(device_, buffer, nullptr);
            }
        }
        for (VkDeviceMemory memory : cellMemories_) {
            if (memory != VK_NULL_HANDLE) {
                vkFreeMemory(device_, memory, nullptr);
            }
        }
        vkDestroyDevice(device_, nullptr);
        device_ = VK_NULL_HANDLE;
    }
    if (surface_ != VK_NULL_HANDLE) {
        vkDestroySurfaceKHR(instance_, surface_, nullptr);
        surface_ = VK_NULL_HANDLE;
    }
    if (debugMessenger_ != VK_NULL_HANDLE) {
        const auto destroyFunction = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance_, "vkDestroyDebugUtilsMessengerEXT"));
        if (destroyFunction != nullptr) {
            destroyFunction(instance_, debugMessenger_, nullptr);
        }
        debugMessenger_ = VK_NULL_HANDLE;
    }
    if (instance_ != VK_NULL_HANDLE) {
        vkDestroyInstance(instance_, nullptr);
        instance_ = VK_NULL_HANDLE;
    }
}

} // namespace voxel
