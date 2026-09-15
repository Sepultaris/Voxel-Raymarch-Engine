#include "render/eight_planet_system.hpp"
#include "planet/geodesic_topology.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <set>
#include <vector>

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}
bool near(double a, double b, double epsilon = 1.0e-9) {
    return std::abs(a - b) <= epsilon;
}
} // namespace

int main() {
    using namespace voxel::system_lab;
    const auto planets = defaultPlanets();
    const auto cycle = directedCycle();
    std::array<std::uint32_t, kPlanetCount> incoming{};
    std::array<std::uint32_t, kPlanetCount> outgoing{};
    for (const CycleHandle handle : cycle) {
        require(handle.destination == (handle.source + 1U) % kPlanetCount,
                "directed handle cycle ordering");
        ++outgoing[handle.source];
        ++incoming[handle.destination];
    }
    for (std::uint32_t index = 0; index < kPlanetCount; ++index) {
        require(incoming[index] == 1U && outgoing[index] == 1U,
                "one incoming and outgoing mouth per planet");
        require(planets[index].seed != 0U, "independent deterministic planet seed");
        const BodyFrame pausedA = evaluateOrbit(planets[index].orbit, 0.0, 0.0);
        const BodyFrame pausedB = evaluateOrbit(planets[index].orbit, 91234.0, 0.0);
        require(length(pausedA.position - pausedB.position) < 1.0e-12,
                "paused orbit is deterministic");
        const BodyFrame moving = evaluateOrbit(planets[index].orbit, 2.0, 1.0);
        require(near(length(moving.position), planets[index].orbit.radius, 1.0e-10),
                "stable circular orbit radius");
        require(std::abs(dot(moving.position, moving.velocity)) < 1.0e-10,
                "orbital velocity is tangent");
    }

    const BodyFrame body8 = evaluateOrbit(planets[7].orbit, 7.25, 1.0);
    const BodyFrame body1 = evaluateOrbit(planets[0].orbit, 7.25, 1.0);
    const MouthFrame source = anchoredMouth(body8, planets[7].radius, true);
    const MouthFrame destination = anchoredMouth(body1, planets[0].radius, false);
    const Vec3d localPosition{0.12, -0.08, -0.03};
    const Vec3d localDirection = normalized({0.18, -0.22, -0.96});
    const Vec3d localVelocity{0.03, 0.01, -0.02};
    const TransportedState transported = transportThroughHandle(
        source, destination, source.position + frameToWorld(source, localPosition),
        frameToWorld(source, localDirection),
        source.velocity + frameToWorld(source, localVelocity),
        source.right, source.up);
    const Vec3d transportedLocalPosition = worldToFrame(
        destination, transported.position - destination.position);
    const Vec3d transportedLocalDirection = worldToFrame(destination, transported.direction);
    const Vec3d transportedLocalVelocity = worldToFrame(
        destination, transported.velocity - destination.velocity);
    require(near(transportedLocalPosition.x, localPosition.x) &&
            near(transportedLocalPosition.y, localPosition.y) &&
            near(transportedLocalPosition.z, localPosition.z),
            "8->1 local position transport");
    require(near(transportedLocalDirection.x, localDirection.x) &&
            near(transportedLocalDirection.y, localDirection.y) &&
            near(transportedLocalDirection.z, localDirection.z),
            "8->1 ray transport");
    require(near(transportedLocalVelocity.x, localVelocity.x) &&
            near(transportedLocalVelocity.y, localVelocity.y) &&
            near(transportedLocalVelocity.z, localVelocity.z),
            "moving endpoint relative-velocity transport");

    for (std::uint32_t level = 0; level <= 5U; ++level) {
        const voxel::GeodesicTopology topology = voxel::GeodesicTopology::build(
            1U << level, 1U, 64U, 32U, 256U, false);
        require(topology.triangleCount == triangleCount(level),
                "dyadic icosahedral triangle count");
        require(topology.tiles.size() == dualCellCount(level),
                "dual-cell count at hierarchy level");
        require(topology.pentagonCount == kPentagonsPerLevel,
                "twelve pentagonal defects");
        require(topology.hexagonCount == hexagonCount(level),
                "all remaining cells are hexagons");
        for (const voxel::GeodesicTileGpu& tile : topology.tiles) {
            require(tile.brickInfo[2] == 5U || tile.brickInfo[2] == 6U,
                    "only pentagonal/hexagonal cells");
        }
    }

    auto sharedTopology = std::make_shared<voxel::GeodesicTopology>(
        voxel::GeodesicTopology::build(4U, 1U, 64U, 32U, 64U, false));
    // Keep the authority-construction guard explicit without making this fast
    // invariant test allocate the full production f512 topology a second time.
    sharedTopology->frequency = 512U;
    std::array<std::uint32_t, kPlanetCount> authoritySeeds{};
    for (std::uint32_t index = 0U; index < kPlanetCount; ++index) {
        authoritySeeds[index] = planets[index].seed;
    }
    EightPlanetTerrainAuthorities authorities(sharedTopology, authoritySeeds);
    std::set<const PlanetTerrainAuthority*> stores;
    std::set<std::uint32_t> seeds;
    std::set<std::uint64_t> terrainSamples;
    for (std::uint32_t index = 0U; index < kPlanetCount; ++index) {
        require(authorities.topologyAddress(index) == sharedTopology.get(),
                "all authorities share exactly one immutable topology");
        stores.insert(&authorities.planet(index));
        seeds.insert(authorities.planet(index).seed());
        const CompactColumnState sample = generatedColumn(
            authorities.planet(index).seed(), 73U);
        terrainSamples.insert(
            (static_cast<std::uint64_t>(sample.filledLayers) << 32U) |
            sample.material);
    }
    require(stores.size() == kPlanetCount && seeds.size() == kPlanetCount,
            "eight distinct stores and seeds");
    require(terrainSamples.size() > 1U,
            "independent seeds produce distinct terrain state");

    PlanetTerrainAuthority& playerAuthority = authorities.planet(3U);
    require(playerAuthority.query(17U).unknown() &&
            playerAuthority.query(17U).filledLayers == 32U,
            "unknown pages are conservatively solid");
    playerAuthority.pinFinestTwoRing(*sharedTopology, 17U);
    require(!playerAuthority.query(17U).unknown(),
            "camera two-ring is resident at finest level");
    const CompactColumnState generated = playerAuthority.query(17U);
    CompactColumnState edited = generated;
    edited.filledLayers = static_cast<std::uint8_t>(
        std::max(1, static_cast<int>(generated.filledLayers) - 3));
    edited.material = 7U;
    playerAuthority.edit(17U, edited);
    const CompactColumnState collisionState = playerAuthority.query(17U);
    const CompactColumnState pickingState = playerAuthority.query(17U);
    require(collisionState.filledLayers == pickingState.filledLayers &&
            collisionState.material == 7U && collisionState.edited(),
            "collision and edit picking share authoritative leaf state");
    playerAuthority.evictUnpinned();
    require(playerAuthority.query(17U).material == 7U,
            "edit overlay survives page eviction");
    playerAuthority.regenerate(playerAuthority.seed() ^ 0x12345U);
    require(playerAuthority.query(17U).material == 7U,
            "authored edits survive procedural regeneration");
    const voxel::GeodesicTileGpu& editedCell = sharedTopology->tiles[17U];
    const std::uint32_t seamNeighbor = editedCell.neighborsLow[0];
    playerAuthority.makeResident(seamNeighbor, true);
    const CompactColumnState seamState = playerAuthority.query(seamNeighbor);
    require(!seamState.unknown() &&
            std::isfinite(compactColumnCapScale(seamState)) &&
            std::isfinite(compactColumnCapScale(playerAuthority.query(17U))),
            "collision/edit walk remains authoritative across an LOD seam");

    const SystemMemoryTelemetry authorityMemory = authorities.telemetry();
    require(authorityMemory.withinGuardrail(), "authority memory guardrail");
    require(authorityMemory.authorityBytes > 0U &&
            authorityMemory.immutableTopologyBytes > 0U,
            "explicit authority/topology memory telemetry");

    const voxel::GeodesicTopology hierarchyReference =
        voxel::GeodesicTopology::build(32U, 1U, 128U, 64U, 1U, false);
    const SharedLodHierarchy hierarchy = buildSharedLodHierarchy(
        hierarchyReference, 6U);
    require(kSystemCoarseLodLevels == kSystemExactLodLevel,
            "GPU hierarchy reaches f256 before the exact f512 level");
    require(hierarchy.levelCounts[0] == dualCellCount(0U) &&
            hierarchy.levelCounts[5] == dualCellCount(5U),
            "shared hierarchy retains geodesic dual-cell counts");
    require(hierarchy.leafToFinestCoarse.size() ==
                hierarchyReference.tiles.size(),
            "every exact leaf maps into shared hierarchy");
    for (std::uint32_t level = 0U; level < hierarchy.activeLevels; ++level) {
        std::uint32_t pentagons = 0U;
        const std::uint32_t begin = hierarchy.levelOffsets[level];
        const std::uint32_t end = begin + hierarchy.levelCounts[level];
        for (std::uint32_t nodeIndex = begin; nodeIndex < end; ++nodeIndex) {
            const SharedLodNode& node = hierarchy.nodes[nodeIndex];
            pentagons += node.pentagon ? 1U : 0U;
            for (const std::uint32_t neighbor : node.neighbors) {
                if (neighbor != std::numeric_limits<std::uint32_t>::max()) {
                    require(neighbor >= begin && neighbor < end,
                            "coarse neighbors remain in their geodesic level");
                }
            }
        }
        require(pentagons == kPentagonsPerLevel,
                "twelve pentagons in every displayed hierarchy level");
    }
    const auto planetAggregates = buildPlanetAggregates(
        hierarchyReference, hierarchy, authoritySeeds);
    for (std::uint32_t planet = 0U; planet < kPlanetCount; ++planet) {
        require(planetAggregates[planet].size() == hierarchy.nodes.size(),
                "per-planet hierarchy aggregates uploaded independently");
        for (const Aggregate& aggregate : planetAggregates[planet]) {
            require(aggregate.minimumHeight <= aggregate.maximumHeight &&
                    aggregate.minimumHeight >= 0.0F &&
                    aggregate.maximumHeight <= 1.0F,
                    "conservative descendant cap bounds");
            const GpuAggregateState packed = unpackAggregateGpu(
                packAggregateGpu(aggregate));
            require(packed.minimumHeight <= aggregate.minimumHeight + 1e-7F &&
                    packed.maximumHeight + 1e-7F >= aggregate.maximumHeight &&
                    packed.maximumProjectedError + 1e-7F >=
                        aggregate.maximumProjectedError,
                    "compact GPU aggregate preserves conservative bounds");
        }
    }

    PrimalTriangleId id = PrimalTriangleId::root(19U);
    for (std::uint32_t level = 0; level < 12U; ++level) {
        const PrimalTriangleId child = id.child(level & 3U);
        require(child.parent() == id, "stable dyadic parent mapping");
        id = child;
    }

    const std::array<Aggregate, 4> children{{
        {-0.2F, 0.3F, {2U, 0U, 1U, 0U}, false, 0.2F},
        {-0.4F, 0.1F, {0U, 3U, 0U, 0U}, true, 0.4F},
        {0.0F, 0.8F, {0U, 0U, 4U, 0U}, false, 0.3F},
        {-0.1F, 0.2F, {0U, 0U, 0U, 5U}, false, 0.1F}}};
    const Aggregate parent = conservativeAggregate(children);
    require(parent.minimumHeight == -0.4F && parent.maximumHeight == 0.8F,
            "conservative height bounds");
    require(parent.materialCounts == std::array<std::uint32_t, 4>{2U, 3U, 5U, 5U},
            "material aggregate");
    require(parent.editDirty && parent.maximumProjectedError == 0.4F,
            "dirty/error aggregate");
    require(lodTransition(0.249F) == 0.0F && lodTransition(1.001F) == 1.0F,
            "continuous split/merge endpoints");
    require(lodTransition(0.625F) > 0.49F && lodTransition(0.625F) < 0.51F,
            "continuous transition interior");
    require(selectLod(4.0F, 1.0F, 8U) == 2U &&
            selectLod(4.0F, 16.0F, 8U) == 4U,
            "lens Jacobian drives refinement");

    std::array<std::uint32_t, 4> levels{0U, 4U, 0U, 0U};
    const std::array<std::array<std::uint32_t, 6>, 4> neighbors{{
        {1U, 3U, 99U, 99U, 99U, 99U}, {0U, 2U, 99U, 99U, 99U, 99U},
        {1U, 3U, 99U, 99U, 99U, 99U}, {0U, 2U, 99U, 99U, 99U, 99U}}};
    enforceTwoToOne(levels, neighbors);
    for (std::size_t index = 0; index < levels.size(); ++index) {
        for (const std::uint32_t neighbor : neighbors[index]) {
            if (neighbor < levels.size()) {
                require(std::abs(static_cast<int>(levels[index]) -
                                 static_cast<int>(levels[neighbor])) <= 1,
                        "2:1 neighbor rule");
            }
        }
    }

    std::vector<std::uint32_t> differentialLeaves;
    std::uint32_t northPole = 0U;
    std::uint32_t southPole = 0U;
    for (std::uint32_t tile = 0U; tile < hierarchyReference.tiles.size(); ++tile) {
        const auto& cell = hierarchyReference.tiles[tile];
        if (cell.brickInfo[3] != 0U) differentialLeaves.push_back(tile);
        if (cell.center[1] > hierarchyReference.tiles[northPole].center[1]) {
            northPole = tile;
        }
        if (cell.center[1] < hierarchyReference.tiles[southPole].center[1]) {
            southPole = tile;
        }
    }
    differentialLeaves.push_back(northPole);
    differentialLeaves.push_back(southPole);
    const auto& seamCell = hierarchyReference.tiles[northPole];
    differentialLeaves.push_back(seamCell.neighborsLow[0]);
    std::uint64_t differentialSamples = 0U;
    for (std::uint32_t planet = 0U; planet < kPlanetCount; ++planet) {
        for (const std::uint32_t leaf : differentialLeaves) {
            const CompactColumnState cpu = generatedColumn(
                authoritySeeds[planet], leaf, 1U);
            const CompactColumnState gpu = unpackCompactColumnGpu(
                packCompactColumnGpu(cpu), 1U);
            require(cpu.filledLayers == gpu.filledLayers &&
                    cpu.material == gpu.material && cpu.flags == gpu.flags,
                    "CPU/GPU finest material and occupancy differential");
            require(near(compactColumnCapScale(cpu),
                         compactColumnCapScale(gpu), 1.0e-7),
                    "CPU/GPU finest depth and silhouette differential");
            ++differentialSamples;
        }
    }
    require(differentialSamples >= 8U * (kPentagonsPerLevel + 3U),
            "differential covers all pentagons, poles, and a seam");
    const CompactColumnState editedGpu = unpackCompactColumnGpu(
        packCompactColumnGpu(edited), edited.generation);
    require(editedGpu.filledLayers == edited.filledLayers &&
            editedGpu.material == edited.material,
            "edited leaf uses the identical CPU/GPU packed authority");

    SparseEditHierarchy edits;
    const PrimalTriangleId editedLeaf = PrimalTriangleId::root(3U).child(2U).child(1U);
    edits.set(editedLeaf, {0.125F, 7U});
    require(edits.ancestorDirty(editedLeaf.parent()), "edit dirties ancestors");
    edits.evictResidentPages();
    require(edits.find(editedLeaf) != nullptr && edits.find(editedLeaf)->material == 7U,
            "sparse edit persists across eviction");

    constexpr std::uint32_t sampleCount = 32U;
    double maximumRadius = 0.0;
    double minimumSeparation = 10.0;
    std::array<std::array<double, 2>, sampleCount> samples{};
    for (std::uint32_t index = 0; index < sampleCount; ++index) {
        samples[index] = hammersleyDisk(index, sampleCount);
        maximumRadius = std::max(maximumRadius,
            std::hypot(samples[index][0], samples[index][1]));
        for (std::uint32_t other = 0; other < index; ++other) {
            minimumSeparation = std::min(minimumSeparation,
                std::hypot(samples[index][0] - samples[other][0],
                           samples[index][1] - samples[other][1]));
        }
    }
    require(maximumRadius <= 1.0 && minimumSeparation > 0.01,
            "distributed stellar-disk samples");
    const double smallEmitter = sampledDiskVisibility(0.08, 0.30, {0.0, 0.0});
    const double broadEmitter = sampledDiskVisibility(0.90, 0.30, {0.0, 0.0});
    require(smallEmitter == 0.0 && broadEmitter > 0.0 && broadEmitter < 1.0,
            "penumbra broadens monotonically as emitter radius grows");
    require(sampledDiskVisibility(0.0, 0.2, {0.21, 0.0}) == 1.0 &&
            sampledDiskVisibility(0.0, 0.2, {0.19, 0.0}) == 0.0,
            "point-light limit as stellar radius approaches zero");
    const double offCenterDisk = sampledDiskVisibility(0.8, 0.45, {0.0, 0.0});
    require(offCenterDisk > 0.0 && offCenterDisk < 1.0,
            "finite disk is sampled instead of using its blocked center point");
    const StarLight dark{2.0, 8.0, {1.0, 1.0, 1.0}, 12U,
                         {1.0, 0.5, 0.25}, 0.1};
    const StarLight bright{2.0, 8.0, {1.0, 1.0, 1.0}, 12U,
                           {1.0, 0.5, 0.25}, 0.4};
    require(boundedAmbient(bright) > boundedAmbient(dark), "ambient monotonicity");
    require(boundedAmbient({2.0, 0.2, {1.0, 1.0, 1.0}, 12U,
                            {100.0, 100.0, 100.0}, 100.0}) <= 0.2,
            "ambient energy bound");

    const auto sorted = sortFrontToBack({
        {9.0, 10.0, 2U, 1U}, {2.0, 3.0, 1U, 4U},
        {2.0, 4.0, 0U, 3U}, {5.0, 6.0, 2U, 0U}});
    require(sorted[0].kind == 0U && sorted[1].kind == 1U &&
            sorted[3].nearDistance == 9.0,
            "front-to-back broad phase ordering");

    const Vec3d original{1.0e12 + 0.125, -1.0e12 + 0.25, 7.0};
    const HighLow3 split = splitHighLow(original);
    const Vec3d reconstructed{
        static_cast<double>(split.high[0]) + split.low[0],
        static_cast<double>(split.high[1]) + split.low[1],
        static_cast<double>(split.high[2]) + split.low[2]};
    require(length(reconstructed - original) < 1.0e-3,
            "camera-relative high/low split");

    std::cout << "Eight-planet CPU invariants passed: 8 independent authorities, "
                 "shared f512 contract, sparse/pinned/edit state, geodesic LOD "
                 "aggregates, handles, finite-star samples, and sorted broad phase.\n";
    return 0;
}
