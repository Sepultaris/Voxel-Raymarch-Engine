#pragma once

#include "planet/geodesic_topology.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string_view>

namespace voxel {

struct GeodesicTopologyCacheConfig {
    std::uint32_t frequency{};
    std::uint32_t brickResolution{};
    std::uint32_t lookupWidth{};
    std::uint32_t lookupHeight{};
    std::uint32_t residentPages{};
    bool referenceTraversal{};
};

using TopologyProgress = std::function<void(
    std::string_view stage, std::uint64_t current, std::uint64_t total)>;

struct GeodesicTopologyCacheLoad {
    GeodesicTopology topology;
    std::uint64_t fileBytes{};
};

inline constexpr std::uint32_t kGeodesicTopologyCacheVersion = 1U;

[[nodiscard]] std::filesystem::path scaleLabTopologyCachePath();
[[nodiscard]] std::optional<GeodesicTopologyCacheLoad> loadGeodesicTopologyCache(
    const std::filesystem::path& path, const GeodesicTopologyCacheConfig& config,
    const TopologyProgress& progress = {});
[[nodiscard]] std::uint64_t writeGeodesicTopologyCache(
    const std::filesystem::path& path, const GeodesicTopologyCacheConfig& config,
    const GeodesicTopology& topology, const TopologyProgress& progress = {});
[[nodiscard]] bool clearExactGeodesicTopologyCache(
    const std::filesystem::path& path, std::string& error);

} // namespace voxel
