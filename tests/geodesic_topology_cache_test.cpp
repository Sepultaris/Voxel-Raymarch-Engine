#include "planet/geodesic_topology_cache.hpp"

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "topology cache test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

} // namespace

int main() {
    const auto root = std::filesystem::temp_directory_path() /
        "voxel-geodesic-cache-integrity-test";
    const auto path = root / "test-cache.bin";
    std::string ignored;
    (void)voxel::clearExactGeodesicTopologyCache(path, ignored);

    const voxel::GeodesicTopologyCacheConfig config{8U, 32U, 32U, 16U, 128U, false};
    std::uint32_t progressEvents = 0U;
    auto topology = voxel::GeodesicTopology::build(
        config.frequency, config.brickResolution, config.lookupWidth,
        config.lookupHeight, config.residentPages, config.referenceTraversal,
        [&](std::string_view, std::uint64_t, std::uint64_t) { ++progressEvents; });
    require(progressEvents > 10U, "cold generation did not expose useful progress");
    const auto bytes = voxel::writeGeodesicTopologyCache(path, config, topology);
    require(bytes == std::filesystem::file_size(path), "cache size reporting is wrong");
    auto warm = voxel::loadGeodesicTopologyCache(path, config);
    require(warm.has_value(), "valid warm cache was rejected");
    require(warm->topology.tiles.size() == topology.tiles.size() &&
                std::memcmp(warm->topology.tiles.data(), topology.tiles.data(),
                            topology.tiles.size() * sizeof(voxel::GeodesicTileGpu)) == 0,
            "warm cache payload differs from deterministic cold build");

    auto mismatch = config;
    mismatch.frequency = 9U;
    require(!voxel::loadGeodesicTopologyCache(path, mismatch).has_value(),
            "configuration mismatch was accepted");

    {
        std::fstream corrupt(path, std::ios::binary | std::ios::in | std::ios::out);
        corrupt.seekp(-8, std::ios::end);
        const std::uint64_t damage = 0xdecafbadcafebeefULL;
        corrupt.write(reinterpret_cast<const char*>(&damage), sizeof(damage));
    }
    require(!voxel::loadGeodesicTopologyCache(path, config).has_value(),
            "corrupted checksum payload was accepted");

    (void)voxel::writeGeodesicTopologyCache(path, config, topology);
    std::filesystem::resize_file(path, std::filesystem::file_size(path) / 2U);
    require(!voxel::loadGeodesicTopologyCache(path, config).has_value(),
            "partial cache was accepted");

    bool watchdogFired = false;
    try {
        (void)voxel::writeGeodesicTopologyCache(
            path, config, topology,
            [&](std::string_view, std::uint64_t current, std::uint64_t) {
                if (current > 0U) {
                    throw std::runtime_error("synthetic watchdog");
                }
            });
    } catch (const std::runtime_error&) {
        watchdogFired = true;
    }
    require(watchdogFired, "watchdog callback did not abort cache publication");
    require(!std::filesystem::exists(path.string() + ".partial"),
            "aborted write left a partial cache file");

    require(voxel::clearExactGeodesicTopologyCache(path, ignored),
            "exact cache cleanup failed");
    std::filesystem::remove(root);
}
