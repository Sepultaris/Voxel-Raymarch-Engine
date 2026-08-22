#include "planet/geodesic_topology_cache.hpp"

#include <array>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <type_traits>
#if defined(_WIN32)
#include <windows.h>
#endif

namespace voxel {
namespace {

constexpr std::array<char, 8> kMagic{'V', 'X', 'G', 'E', 'O', '1', '0', '2'};
constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr std::size_t kIoChunk = 4U * 1024U * 1024U;

struct PartialFileGuard {
    std::filesystem::path path;
    bool published{};
    ~PartialFileGuard() {
        if (!published) {
            std::error_code ignored;
            std::filesystem::remove(path, ignored);
        }
    }
};

struct CacheHeader {
    std::array<char, 8> magic{};
    std::uint32_t version{};
    std::uint32_t endian{};
    std::uint32_t tileSize{};
    std::uint32_t columnStateSize{};
    std::uint32_t pageTableSize{};
    std::uint32_t bvhNodeSize{};
    std::uint32_t wideNodeSize{};
    std::uint32_t frequency{};
    std::uint32_t brickResolution{};
    std::uint32_t lookupWidth{};
    std::uint32_t lookupHeight{};
    std::uint32_t residentPages{};
    std::uint32_t referenceTraversal{};
    std::uint32_t pentagons{};
    std::uint32_t hexagons{};
    std::uint32_t triangles{};
    std::uint32_t nextPageGeneration{};
    std::uint64_t tileCount{};
    std::uint64_t lookupCount{};
    std::uint64_t brickCellCount{};
    std::uint64_t bvhCount{};
    std::uint64_t wideNodeCount{};
    std::uint64_t wideItemCount{};
    std::uint64_t columnStateCount{};
    std::uint64_t pageTableCount{};
    std::uint64_t payloadBytes{};
    float outerBoundingRadius{};
    float bedrockRadius{};
    std::uint64_t checksum{};
};
static_assert(std::is_trivially_copyable_v<CacheHeader>);

void hashBytes(std::uint64_t& hash, const void* data, std::size_t bytes) {
    const auto* cursor = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < bytes; ++index) {
        hash ^= cursor[index];
        hash *= kFnvPrime;
    }
}

template <typename Stream>
void transfer(Stream& stream, void* data, std::uint64_t bytes, bool writing,
              std::uint64_t& hash, std::uint64_t& completed,
              std::uint64_t total, const TopologyProgress& progress) {
    auto* cursor = static_cast<std::byte*>(data);
    while (bytes > 0U) {
        const std::size_t chunk = static_cast<std::size_t>(
            std::min<std::uint64_t>(bytes, kIoChunk));
        if (writing) {
            stream.write(reinterpret_cast<const char*>(cursor),
                         static_cast<std::streamsize>(chunk));
        } else {
            stream.read(reinterpret_cast<char*>(cursor), static_cast<std::streamsize>(chunk));
        }
        if (!stream) {
            throw std::runtime_error(writing ? "Topology cache write failed"
                                             : "Topology cache is truncated");
        }
        hashBytes(hash, cursor, chunk);
        cursor += chunk;
        bytes -= chunk;
        completed += chunk;
        if (progress) {
            progress(writing ? "topology cache write" : "topology cache read",
                     completed, total);
        }
    }
}

template <typename T, typename Stream>
void transferVector(Stream& stream, std::vector<T>& values, bool writing,
                    std::uint64_t& hash, std::uint64_t& completed,
                    std::uint64_t total, const TopologyProgress& progress) {
    if (!values.empty()) {
        transfer(stream, values.data(), values.size() * sizeof(T), writing,
                 hash, completed, total, progress);
    }
}

CacheHeader makeHeader(const GeodesicTopologyCacheConfig& config,
                       const GeodesicTopology& topology) {
    CacheHeader header{};
    header.magic = kMagic;
    header.version = kGeodesicTopologyCacheVersion;
    header.endian = 0x01020304U;
    header.tileSize = sizeof(GeodesicTileGpu);
    header.columnStateSize = sizeof(GeodesicColumnStateGpu);
    header.pageTableSize = sizeof(GeodesicPageTableEntryGpu);
    header.bvhNodeSize = sizeof(GeodesicBvhNodeGpu);
    header.wideNodeSize = sizeof(GeodesicWideNodeGpu);
    header.frequency = config.frequency;
    header.brickResolution = config.brickResolution;
    header.lookupWidth = config.lookupWidth;
    header.lookupHeight = config.lookupHeight;
    header.residentPages = config.residentPages;
    header.referenceTraversal = config.referenceTraversal ? 1U : 0U;
    header.pentagons = topology.pentagonCount;
    header.hexagons = topology.hexagonCount;
    header.triangles = topology.triangleCount;
    header.nextPageGeneration = topology.nextPageGeneration;
    header.tileCount = topology.tiles.size();
    header.lookupCount = topology.directionLookup.size();
    header.brickCellCount = topology.brickCells.size();
    header.bvhCount = topology.bvhNodes.size();
    header.wideNodeCount = topology.wideNodes.size();
    header.wideItemCount = topology.wideItems.size();
    header.columnStateCount = topology.columnStates.size();
    header.pageTableCount = topology.pageTable.size();
    header.payloadBytes = header.tileCount * sizeof(GeodesicTileGpu) +
        header.lookupCount * sizeof(std::uint32_t) +
        header.brickCellCount * sizeof(std::uint32_t) +
        header.bvhCount * sizeof(GeodesicBvhNodeGpu) +
        header.wideNodeCount * sizeof(GeodesicWideNodeGpu) +
        header.wideItemCount * sizeof(std::uint32_t) +
        header.columnStateCount * sizeof(GeodesicColumnStateGpu) +
        header.pageTableCount * sizeof(GeodesicPageTableEntryGpu);
    header.outerBoundingRadius = topology.outerBoundingRadius;
    header.bedrockRadius = topology.bedrockRadius;
    return header;
}

bool headerMatches(CacheHeader header, const GeodesicTopologyCacheConfig& config,
                   std::uint64_t fileBytes) {
    const std::uint64_t expectedTiles = 10ULL * config.frequency * config.frequency + 2ULL;
    const std::uint64_t expectedLookup =
        static_cast<std::uint64_t>(config.lookupWidth) * config.lookupHeight;
    const std::uint64_t expectedPages = std::min<std::uint64_t>(config.residentPages,
                                                                expectedTiles);
    return header.magic == kMagic && header.version == kGeodesicTopologyCacheVersion &&
        header.endian == 0x01020304U && header.tileSize == sizeof(GeodesicTileGpu) &&
        header.columnStateSize == sizeof(GeodesicColumnStateGpu) &&
        header.pageTableSize == sizeof(GeodesicPageTableEntryGpu) &&
        header.bvhNodeSize == sizeof(GeodesicBvhNodeGpu) &&
        header.wideNodeSize == sizeof(GeodesicWideNodeGpu) &&
        header.frequency == config.frequency &&
        header.brickResolution == config.brickResolution &&
        header.lookupWidth == config.lookupWidth && header.lookupHeight == config.lookupHeight &&
        header.residentPages == config.residentPages &&
        header.referenceTraversal == (config.referenceTraversal ? 1U : 0U) &&
        header.tileCount == expectedTiles && header.lookupCount == expectedLookup &&
        header.brickCellCount == expectedPages * config.brickResolution &&
        header.columnStateCount == expectedTiles && header.pageTableCount == expectedTiles &&
        (config.referenceTraversal ||
         (header.bvhCount == 0U && header.wideNodeCount == 0U && header.wideItemCount == 0U)) &&
        fileBytes == sizeof(CacheHeader) + header.payloadBytes;
}

} // namespace

std::filesystem::path scaleLabTopologyCachePath() {
#ifdef VOXEL_SCALE_LAB_CACHE_DIR
    return std::filesystem::path(VOXEL_SCALE_LAB_CACHE_DIR) /
           "geodesic-f1024-l32-a1024x512-p8192-v1.bin";
#else
    return std::filesystem::path("cache/scale-lab") /
           "geodesic-f1024-l32-a1024x512-p8192-v1.bin";
#endif
}

std::optional<GeodesicTopologyCacheLoad> loadGeodesicTopologyCache(
    const std::filesystem::path& path, const GeodesicTopologyCacheConfig& config,
    const TopologyProgress& progress) {
    std::error_code error;
    const std::uint64_t fileBytes = std::filesystem::file_size(path, error);
    if (error || fileBytes < sizeof(CacheHeader)) {
        return std::nullopt;
    }
    std::fstream input(path, std::ios::binary | std::ios::in);
    CacheHeader header{};
    input.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!input || !headerMatches(header, config, fileBytes)) {
        return std::nullopt;
    }
    const std::uint64_t expectedChecksum = header.checksum;
    header.checksum = 0U;
    std::uint64_t hash = kFnvOffset;
    hashBytes(hash, &header, sizeof(header));

    GeodesicTopology topology{};
    topology.frequency = header.frequency;
    topology.brickResolution = header.brickResolution;
    topology.pentagonCount = header.pentagons;
    topology.hexagonCount = header.hexagons;
    topology.triangleCount = header.triangles;
    topology.lookupWidth = header.lookupWidth;
    topology.lookupHeight = header.lookupHeight;
    topology.outerBoundingRadius = header.outerBoundingRadius;
    topology.bedrockRadius = header.bedrockRadius;
    topology.referenceTraversalBuilt = header.referenceTraversal != 0U;
    topology.nextPageGeneration = header.nextPageGeneration;
    topology.tiles.resize(static_cast<std::size_t>(header.tileCount));
    topology.directionLookup.resize(static_cast<std::size_t>(header.lookupCount));
    topology.brickCells.resize(static_cast<std::size_t>(header.brickCellCount));
    topology.bvhNodes.resize(static_cast<std::size_t>(header.bvhCount));
    topology.wideNodes.resize(static_cast<std::size_t>(header.wideNodeCount));
    topology.wideItems.resize(static_cast<std::size_t>(header.wideItemCount));
    topology.columnStates.resize(static_cast<std::size_t>(header.columnStateCount));
    topology.pageTable.resize(static_cast<std::size_t>(header.pageTableCount));
    std::uint64_t completed = 0U;
    transferVector(input, topology.tiles, false, hash, completed, header.payloadBytes, progress);
    transferVector(input, topology.directionLookup, false, hash, completed, header.payloadBytes, progress);
    transferVector(input, topology.brickCells, false, hash, completed, header.payloadBytes, progress);
    transferVector(input, topology.bvhNodes, false, hash, completed, header.payloadBytes, progress);
    transferVector(input, topology.wideNodes, false, hash, completed, header.payloadBytes, progress);
    transferVector(input, topology.wideItems, false, hash, completed, header.payloadBytes, progress);
    transferVector(input, topology.columnStates, false, hash, completed, header.payloadBytes, progress);
    transferVector(input, topology.pageTable, false, hash, completed, header.payloadBytes, progress);
    if (hash != expectedChecksum) {
        return std::nullopt;
    }
    topology.validate();
    return GeodesicTopologyCacheLoad{std::move(topology), fileBytes};
}

std::uint64_t writeGeodesicTopologyCache(
    const std::filesystem::path& path, const GeodesicTopologyCacheConfig& config,
    const GeodesicTopology& topology, const TopologyProgress& progress) {
    topology.validate();
    std::filesystem::create_directories(path.parent_path());
    const std::filesystem::path temporary = path.string() + ".partial";
    PartialFileGuard partialGuard{temporary};
    std::fstream output(temporary, std::ios::binary | std::ios::in |
                                      std::ios::out | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("Cannot create topology cache temporary file: " + temporary.string());
    }
    CacheHeader header = makeHeader(config, topology);
    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
    std::uint64_t hash = kFnvOffset;
    hashBytes(hash, &header, sizeof(header));
    std::uint64_t completed = 0U;
    auto& mutableTopology = const_cast<GeodesicTopology&>(topology);
    transferVector(output, mutableTopology.tiles, true, hash, completed, header.payloadBytes, progress);
    transferVector(output, mutableTopology.directionLookup, true, hash, completed, header.payloadBytes, progress);
    transferVector(output, mutableTopology.brickCells, true, hash, completed, header.payloadBytes, progress);
    transferVector(output, mutableTopology.bvhNodes, true, hash, completed, header.payloadBytes, progress);
    transferVector(output, mutableTopology.wideNodes, true, hash, completed, header.payloadBytes, progress);
    transferVector(output, mutableTopology.wideItems, true, hash, completed, header.payloadBytes, progress);
    transferVector(output, mutableTopology.columnStates, true, hash, completed, header.payloadBytes, progress);
    transferVector(output, mutableTopology.pageTable, true, hash, completed, header.payloadBytes, progress);
    header.checksum = hash;
    output.seekp(0);
    output.write(reinterpret_cast<const char*>(&header), sizeof(header));
    output.flush();
    output.close();
    if (!output) {
        throw std::runtime_error("Finalizing topology cache failed");
    }
#if defined(_WIN32)
    if (MoveFileExW(temporary.c_str(), path.c_str(),
                    MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
        throw std::runtime_error("Atomically publishing topology cache failed");
    }
#else
    std::filesystem::rename(temporary, path);
#endif
    partialGuard.published = true;
    return sizeof(CacheHeader) + header.payloadBytes;
}

bool clearExactGeodesicTopologyCache(const std::filesystem::path& path, std::string& error) {
    std::error_code code;
    const bool removed = std::filesystem::remove(path, code);
    if (code) {
        error = code.message();
        return false;
    }
    const std::filesystem::path partial = path.string() + ".partial";
    std::filesystem::remove(partial, code);
    if (code) {
        error = code.message();
        return false;
    }
    return removed;
}

} // namespace voxel
