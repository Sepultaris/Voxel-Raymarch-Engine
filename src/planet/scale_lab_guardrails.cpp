#include "planet/scale_lab_guardrails.hpp"

#include <iomanip>
#include <limits>
#include <sstream>

namespace voxel {
namespace {

constexpr std::uint64_t kColumnStateGpuBytes = 16ULL;
constexpr std::uint64_t kPageTableGpuBytes = 16ULL;
constexpr std::uint64_t kPageRequestCapacity = 16384ULL;
constexpr std::uint64_t kPageRequestBytes = 16ULL;

double gib(std::uint64_t bytes) {
    return static_cast<double>(bytes) / static_cast<double>(1024ULL * 1024ULL * 1024ULL);
}

} // namespace

ScaleLabBudget estimateScaleLabBudget(
    std::uint32_t frequency, std::uint32_t radialLayers,
    std::uint32_t lookupWidth, std::uint32_t lookupHeight,
    std::uint32_t residentPages, std::uint32_t pushConstantBytes,
    std::uint32_t tileQueryBytes, std::uint64_t acceleratorBytes) noexcept {
    ScaleLabBudget budget{};
    const std::uint64_t f = frequency;
    budget.surfaceTiles = 10ULL * f * f + 2ULL;
    budget.virtualVoxelCells = budget.surfaceTiles * radialLayers;

    const std::uint64_t denseMetadata = budget.surfaceTiles *
        (tileQueryBytes + kColumnStateGpuBytes + kPageTableGpuBytes);
    const std::uint64_t lookup = static_cast<std::uint64_t>(lookupWidth) *
        lookupHeight * sizeof(std::uint32_t);
    const std::uint64_t residentBricks = static_cast<std::uint64_t>(residentPages) *
        radialLayers * sizeof(std::uint32_t);
    const std::uint64_t requestFeedback = 16ULL +
        kPageRequestCapacity * kPageRequestBytes;
    budget.gpuMetadataBytes = denseMetadata + lookup + residentBricks +
                              requestFeedback + acceleratorBytes;

    // Conservative pre-allocation estimate for the current builder. It includes
    // the topology payload, red-black tree nodes used while deduplicating
    // vertices/edges, adjacency scratch, upload staging, and a 25% allocator margin.
    const std::uint64_t persistentAndStaging = budget.surfaceTiles * 256ULL;
    const std::uint64_t orderedBuilderNodes = budget.surfaceTiles * 224ULL;
    const std::uint64_t adjacencyScratch = budget.surfaceTiles * 80ULL;
    budget.estimatedCpuPeakBytes =
        (persistentAndStaging + orderedBuilderNodes + adjacencyScratch) * 5ULL / 4ULL;

    budget.offsetsFit32Bit =
        budget.surfaceTiles <= std::numeric_limits<std::uint32_t>::max() &&
        budget.virtualVoxelCells <= std::numeric_limits<std::uint32_t>::max();
    budget.gpuMetadataFits = budget.gpuMetadataBytes <= kScaleLabGpuMetadataLimit;
    budget.cpuPeakFits = budget.estimatedCpuPeakBytes <= kScaleLabCpuPeakLimit;
    budget.pushConstantsFitPortableLimit = pushConstantBytes <= 128U;
    return budget;
}

std::string ScaleLabBudget::report() const {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "tiles=" << surfaceTiles
        << " virtual-cells=" << virtualVoxelCells
        << " GPU-metadata=" << gib(gpuMetadataBytes) << " GiB/1.500 GiB"
        << " CPU-peak-estimate=" << gib(estimatedCpuPeakBytes) << " GiB/8.000 GiB"
        << " offsets32=" << (offsetsFit32Bit ? "yes" : "NO")
        << " push128=" << (pushConstantsFitPortableLimit ? "yes" : "NO")
        << " accepted=" << (accepted() ? "yes" : "NO");
    return out.str();
}

} // namespace voxel
