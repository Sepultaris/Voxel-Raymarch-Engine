#include "app/surface_player_collision.hpp"

#include "planet/adaptive_planet_sdf.hpp"
#include "planet/fractal_planet_sdf.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace voxel {
namespace {

using Vector = std::array<float, 3>;

[[nodiscard]] float dot(const Vector& left, const Vector& right) noexcept {
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2];
}

[[nodiscard]] Vector normalized(Vector value) noexcept {
    const float lengthSquared = dot(value, value);
    if (!(lengthSquared > 1e-12F) || !std::isfinite(lengthSquared)) {
        return {0.0F, 0.0F, 1.0F};
    }
    const float inverseLength = 1.0F / std::sqrt(lengthSquared);
    return {value[0] * inverseLength, value[1] * inverseLength,
            value[2] * inverseLength};
}

[[nodiscard]] Vector centerOf(const GeodesicTileGpu& tile) noexcept {
    return {tile.center[0], tile.center[1], tile.center[2]};
}

[[nodiscard]] std::uint32_t neighbor(const GeodesicTileGpu& tile,
                                     std::uint32_t slot) noexcept {
    return slot < 4U ? tile.neighborsLow[slot] : tile.neighborsHigh[slot - 4U];
}

[[nodiscard]] std::uint32_t locateTile(const GeodesicTopology& topology,
                                       Vector direction) noexcept {
    direction = normalized(direction);
    constexpr float pi = 3.14159265358979323846F;
    const float longitude = std::atan2(direction[2], direction[0]);
    const float latitude = std::asin(std::clamp(direction[1], -1.0F, 1.0F));
    const float u = longitude / (2.0F * pi) + 0.5F;
    const float v = 0.5F - latitude / pi;
    const std::uint32_t x = std::min(
        static_cast<std::uint32_t>(u * static_cast<float>(topology.lookupWidth)),
        topology.lookupWidth - 1U);
    const std::uint32_t y = std::min(
        static_cast<std::uint32_t>(v * static_cast<float>(topology.lookupHeight)),
        topology.lookupHeight - 1U);
    std::uint32_t tileIndex = topology.directionLookup[
        x + static_cast<std::size_t>(topology.lookupWidth) * y];
    for (std::uint32_t walk = 0U; walk < 12U; ++walk) {
        const GeodesicTileGpu& tile = topology.tiles[tileIndex];
        float bestAlignment = dot(direction, centerOf(tile));
        std::uint32_t bestTile = tileIndex;
        for (std::uint32_t slot = 0U; slot < tile.brickInfo[2]; ++slot) {
            const std::uint32_t candidate = neighbor(tile, slot);
            const float alignment = dot(direction, centerOf(topology.tiles[candidate]));
            if (alignment > bestAlignment) {
                bestAlignment = alignment;
                bestTile = candidate;
            }
        }
        if (bestTile == tileIndex) {
            break;
        }
        tileIndex = bestTile;
    }
    return tileIndex;
}

} // namespace

SurfaceTerrainContact queryGeodesicTerrain(
    const GeodesicTopology& topology,
    const std::vector<std::uint8_t>& freshColumnStates,
    const std::array<float, 3>& requestedRadial,
    float planetRadius,
    float capsuleRadius,
    float sweepDistance,
    bool adaptiveSdfEnabled,
    std::uint32_t adaptiveSdfSeed,
    std::uint32_t adaptiveSdfMaximumLevels,
    float adaptiveSdfLayerFraction,
    bool fractalPlanetSdfEnabled,
    std::uint32_t fractalPlanetSdfSeed,
    std::uint32_t fractalPlanetSdfMaximumLevels,
    float fractalPlanetSdfMicroRelief) noexcept {
    SurfaceTerrainContact contact{};
    if (topology.tiles.empty() || topology.columnStates.size() != topology.tiles.size() ||
        !(planetRadius > 0.0F)) {
        return contact;
    }

    const Vector radial = normalized(requestedRadial);
    const std::uint32_t seed = locateTile(topology, radial);
    constexpr std::size_t maximumCandidates = 256U;
    std::array<std::uint32_t, maximumCandidates> candidates{};
    std::array<std::uint8_t, maximumCandidates> depths{};
    std::size_t candidateCount = 1U;
    candidates[0] = seed;
    depths[0] = 0U;

    const float localWidth = topology.tiles[seed].bitangent[3] * planetRadius;
    const float footprint = std::max(capsuleRadius + std::max(sweepDistance, 0.0F), 0.0F);
    const std::uint32_t maximumDepth = std::clamp(
        static_cast<std::uint32_t>(std::ceil(
            footprint / std::max(localWidth, 1e-7F))) + 2U, 2U, 5U);
    for (std::size_t cursor = 0U; cursor < candidateCount; ++cursor) {
        if (depths[cursor] >= maximumDepth) {
            continue;
        }
        const GeodesicTileGpu& tile = topology.tiles[candidates[cursor]];
        for (std::uint32_t slot = 0U; slot < tile.brickInfo[2]; ++slot) {
            const std::uint32_t adjacent = neighbor(tile, slot);
            bool duplicate = false;
            for (std::size_t previous = 0U; previous < candidateCount; ++previous) {
                duplicate = duplicate || candidates[previous] == adjacent;
            }
            if (!duplicate && candidateCount < maximumCandidates) {
                candidates[candidateCount] = adjacent;
                depths[candidateCount] = static_cast<std::uint8_t>(depths[cursor] + 1U);
                ++candidateCount;
            }
        }
    }

    if (fractalPlanetSdfEnabled) {
        contact.floorRadius = 0.0F;
        contact.exact = true;
        const auto considerProceduralDirection = [&](const Vector& sampleDirection,
                                                     std::uint32_t tileIndex) {
            const FractalPlanetField field = evaluateFractalPlanetField(
                normalized(sampleDirection), fractalPlanetSdfSeed,
                static_cast<float>(std::min(fractalPlanetSdfMaximumLevels,
                                            kFractalPlanetMaximumLevels)),
                fractalPlanetSdfMicroRelief);
            if (!field.finite) {
                contact.exact = false;
                return;
            }
            float candidateRadius = planetRadius * field.surfaceRadiusScale;
            const bool fresh = tileIndex < freshColumnStates.size() &&
                freshColumnStates[tileIndex] != 0U;
            const auto& state = topology.columnStates[tileIndex].data;
            if (fresh && state[3] != 0U) {
                if (state[1] == 0U) {
                    candidateRadius = 0.0F;
                } else {
                    const GeodesicTileGpu& tile = topology.tiles[tileIndex];
                    const float layerHeight = tile.tangent[3] /
                        static_cast<float>(tile.brickInfo[1]);
                    const float capPlane = planetRadius *
                        (1.0F - tile.tangent[3] +
                         static_cast<float>(state[1]) * layerHeight);
                    candidateRadius = capPlane /
                        std::max(dot(centerOf(tile), radial), 1e-5F);
                }
            }
            if (candidateRadius > contact.floorRadius) {
                contact.floorRadius = candidateRadius;
                contact.supportingTile = tileIndex;
            }
        };
        considerProceduralDirection(radial, seed);
        const float referenceRadius = std::max(planetRadius -
            topology.tiles[seed].tangent[3] * planetRadius,
            planetRadius * 0.5F);
        for (std::size_t index = 0U; index < candidateCount; ++index) {
            const std::uint32_t tileIndex = candidates[index];
            const GeodesicTileGpu& tile = topology.tiles[tileIndex];
            const float alignment = std::clamp(
                dot(centerOf(tile), radial), -1.0F, 1.0F);
            const float centerAngle = std::acos(alignment);
            const float cellAngularRadius = tile.center[3] / 0.62F;
            const float lateralDistance = referenceRadius *
                std::sin(std::max(centerAngle - cellAngularRadius, 0.0F));
            if (lateralDistance > footprint + localWidth * 0.05F) {
                continue;
            }
            considerProceduralDirection(centerOf(topology.tiles[tileIndex]), tileIndex);
        }
        return contact;
    }

    contact.floorRadius = 0.0F;
    contact.exact = true;
    const float referenceRadius = std::max(planetRadius -
        topology.tiles[seed].tangent[3] * planetRadius, planetRadius * 0.5F);
    for (std::size_t index = 0U; index < candidateCount; ++index) {
        const std::uint32_t tileIndex = candidates[index];
        const GeodesicTileGpu& tile = topology.tiles[tileIndex];
        const Vector center = centerOf(tile);
        const float alignment = std::clamp(dot(center, radial), -1.0F, 1.0F);
        const float centerAngle = std::acos(alignment);
        // center.w is 0.62 of the farthest one-ring angle and therefore a
        // conservative dual-cell circumscribed radius after this conversion.
        const float cellAngularRadius = tile.center[3] / 0.62F;
        const float lateralDistance = referenceRadius *
            std::sin(std::max(centerAngle - cellAngularRadius, 0.0F));
        if (lateralDistance > footprint + localWidth * 0.05F) {
            continue;
        }

        const bool fresh = tileIndex < freshColumnStates.size() &&
            freshColumnStates[tileIndex] != 0U;
        const std::uint32_t filledLayers = fresh
            ? std::min(topology.columnStates[tileIndex].data[1],
                       topology.brickResolution)
            : topology.brickResolution;
        contact.exact = contact.exact && fresh;
        if (filledLayers == 0U) {
            continue;
        }
        const float layerHeight = tile.tangent[3] /
            static_cast<float>(tile.brickInfo[1]);
        const float columnBase = 1.0F - tile.tangent[3];
        // Match the render prism's outward radial overlap exactly.
        const float outerPlane = columnBase +
            static_cast<float>(filledLayers) * layerHeight + layerHeight * 2e-5F;
        const float projectedAlignment = std::max(alignment, 1e-5F);
        float capPlaneWorld = planetRadius * outerPlane;
        if (adaptiveSdfEnabled && fresh &&
            topology.columnStates[tileIndex].data[2] != 3U &&
            topology.columnStates[tileIndex].data[3] == 0U) {
            const AdaptivePlanetSdfField detail = evaluateAdaptivePlanetSdf(
                radial, adaptiveSdfSeed,
                static_cast<float>(std::min(adaptiveSdfMaximumLevels,
                                            kAdaptivePlanetSdfMaximumLevels)),
                adaptiveSdfLayerFraction);
            const float layerHeightWorld = layerHeight * planetRadius;
            capPlaneWorld -= layerHeightWorld * detail.depthLayerFraction;
        }
        const float floorRadius = capPlaneWorld / projectedAlignment;
        if (floorRadius > contact.floorRadius) {
            contact.floorRadius = floorRadius;
            contact.supportingTile = tileIndex;
        }
    }
    return contact;
}

} // namespace voxel
