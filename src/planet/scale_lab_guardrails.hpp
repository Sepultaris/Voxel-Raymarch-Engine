#pragma once

#include <cstdint>
#include <string>

namespace voxel {

struct ScaleLabBudget {
    std::uint64_t surfaceTiles{};
    std::uint64_t virtualVoxelCells{};
    std::uint64_t gpuMetadataBytes{};
    std::uint64_t estimatedCpuPeakBytes{};
    bool offsetsFit32Bit{};
    bool gpuMetadataFits{};
    bool cpuPeakFits{};
    bool pushConstantsFitPortableLimit{};

    [[nodiscard]] bool accepted() const noexcept {
        return offsetsFit32Bit && gpuMetadataFits && cpuPeakFits &&
               pushConstantsFitPortableLimit;
    }
    [[nodiscard]] std::string report() const;
};

[[nodiscard]] ScaleLabBudget estimateScaleLabBudget(
    std::uint32_t frequency, std::uint32_t radialLayers,
    std::uint32_t lookupWidth, std::uint32_t lookupHeight,
    std::uint32_t residentPages, std::uint32_t pushConstantBytes,
    std::uint32_t tileQueryBytes, std::uint64_t acceleratorBytes) noexcept;

inline constexpr std::uint64_t kScaleLabGpuMetadataLimit = 1536ULL * 1024ULL * 1024ULL;
inline constexpr std::uint64_t kScaleLabCpuPeakLimit = 8ULL * 1024ULL * 1024ULL * 1024ULL;
inline constexpr double kScaleLabStartupLimitSeconds = 60.0;
inline constexpr double kScaleLabStartupWatchdogSeconds = 55.0;

} // namespace voxel
