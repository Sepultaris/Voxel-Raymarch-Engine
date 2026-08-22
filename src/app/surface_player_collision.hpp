#pragma once

#include "planet/geodesic_topology.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace voxel {

struct SurfaceTerrainContact {
    float floorRadius{};
    std::uint32_t supportingTile{kInvalidTile};
    bool exact{};
};

// Queries the compact, residency-independent column profile used by the
// renderer. Candidates are bounded to the local topology ring covered by the
// capsule footprint and sweep distance. Unknown GPU profiles are treated as
// maximum-height solid columns, which is conservative for collision.
[[nodiscard]] SurfaceTerrainContact queryGeodesicTerrain(
    const GeodesicTopology& topology,
    const std::vector<std::uint8_t>& freshColumnStates,
    const std::array<float, 3>& radial,
    float planetRadius,
    float capsuleRadius,
    float sweepDistance = 0.0F,
    bool adaptiveSdfEnabled = false,
    std::uint32_t adaptiveSdfSeed = 1337U,
    std::uint32_t adaptiveSdfMaximumLevels = 9U,
    float adaptiveSdfLayerFraction = 0.15F,
    bool fractalPlanetSdfEnabled = false,
    std::uint32_t fractalPlanetSdfSeed = 1337U,
    std::uint32_t fractalPlanetSdfMaximumLevels = 10U,
    float fractalPlanetSdfMicroRelief = 0.012F) noexcept;

} // namespace voxel
