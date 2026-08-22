#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace voxel {

inline constexpr std::uint32_t kFarFieldLevels = 11U;

[[nodiscard]] constexpr std::uint32_t farFieldLevelOffset(
    std::uint32_t baseWidth, std::uint32_t baseHeight,
    std::uint32_t level) noexcept {
    std::uint32_t offset = 0U;
    for (std::uint32_t current = 0U; current < level; ++current) {
        offset += baseWidth * baseHeight;
        baseWidth = std::max(baseWidth >> 1U, 1U);
        baseHeight = std::max(baseHeight >> 1U, 1U);
    }
    return offset;
}

[[nodiscard]] constexpr std::uint32_t farFieldNodeCount(
    std::uint32_t baseWidth, std::uint32_t baseHeight) noexcept {
    return farFieldLevelOffset(baseWidth, baseHeight, kFarFieldLevels);
}

[[nodiscard]] inline std::uint32_t farFieldLodForProjectedPixels(
    float projectedPixels) noexcept {
    return projectedPixels < 0.35F ? 2U : projectedPixels < 0.70F ? 1U : 0U;
}

[[nodiscard]] inline std::uint32_t halfOpenWrappedCoordinate(
    double unitCoordinate, std::uint32_t extent) noexcept {
    const double wrapped = unitCoordinate - std::floor(unitCoordinate);
    return std::min(static_cast<std::uint32_t>(wrapped * extent), extent - 1U);
}

struct FarFieldHeightRange {
    std::uint32_t minimum{32U};
    std::uint32_t maximum{1U};

    void include(std::uint32_t height) noexcept {
        minimum = std::min(minimum, height);
        maximum = std::max(maximum, height);
    }
    void conservativelyPad(std::uint32_t padding) noexcept {
        minimum = minimum > padding ? minimum - padding : 1U;
        maximum = std::min(32U, maximum + padding);
    }
};

} // namespace voxel
