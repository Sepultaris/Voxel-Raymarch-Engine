#include "planet/geodesic_topology.hpp"
#include "planet/geodesic_material.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void verifyResidentPage(const voxel::GeodesicTopology& topology, std::uint32_t tileIndex) {
    const auto& page = topology.pageTable[tileIndex].data;
    require((page[2] & voxel::kPageResident) != 0U, "Expected a resident page");
    require(page[0] < topology.residentPageCount(), "Physical page index is out of range");
    const auto& state = topology.columnStates[tileIndex].data;
    const std::size_t offset = static_cast<std::size_t>(page[0]) * topology.brickResolution;
    for (std::uint32_t layer = 0; layer < topology.brickResolution; ++layer) {
        const std::uint32_t expectedMaterial = voxel::geodesicMaterialForLayer(
            state[1], state[2], layer);
        require(topology.brickCells[offset + layer] == expectedMaterial,
                "Resident material differs from nonresident column reconstruction");
    }
}

} // namespace

int main() {
    try {
        constexpr std::uint32_t frequency = 8U;
        constexpr std::uint32_t layers = 16U;
        constexpr std::uint32_t physicalPages = 7U;
        voxel::GeodesicTopology topology =
            voxel::GeodesicTopology::build(frequency, layers, 32U, 16U, physicalPages);

        require(topology.tiles.size() == 642U, "Unexpected virtual tile count");
        require(topology.brickCellCapacity() == 642ULL * layers,
                "Virtual capacity does not cover every tile");
        require(topology.residentPageCount() == physicalPages,
                "Physical page pool ignored its configured bound");
        require(topology.residentBrickCellCapacity() == physicalPages * layers,
                "Physical cell allocation is not page bounded");
        require(topology.pageTable.size() == topology.tiles.size(),
                "Page table does not cover the virtual address space");

        for (std::uint32_t tile = 0; tile < physicalPages; ++tile) {
            require(topology.pageTable[tile].data[0] == tile,
                    "Initial physical-page ownership is not deterministic");
            verifyResidentPage(topology, tile);
        }
        for (std::uint32_t tile = physicalPages; tile < topology.tiles.size(); ++tile) {
            require(topology.pageTable[tile].data[0] == voxel::kInvalidPage &&
                        topology.pageTable[tile].data[2] == 0U,
                    "A nonresident virtual page has physical storage");
        }

        const std::uint32_t previousGeneration = topology.pageTable[0].data[1];
        constexpr std::uint32_t incomingTile = 100U;
        const std::uint32_t evictedTile = topology.remapResidentPage(0U, incomingTile);
        require(evictedTile == 0U, "Remap returned the wrong evicted tile");
        require(topology.pageTable[evictedTile].data[0] == voxel::kInvalidPage &&
                    topology.pageTable[evictedTile].data[2] == 0U,
                "Evicted virtual page remains resident");
        require(topology.pageTable[incomingTile].data[0] == 0U,
                "Incoming virtual page did not acquire the physical page");
        require(topology.pageTable[incomingTile].data[1] > previousGeneration,
                "Physical page generation did not advance after reuse");
        require(topology.residentPageCount() == physicalPages,
                "Eviction changed the bounded physical allocation");
        verifyResidentPage(topology, incomingTile);
        topology.validate();

        bool rejectedDuplicateOwner = false;
        try {
            (void)topology.remapResidentPage(1U, incomingTile);
        } catch (const std::invalid_argument&) {
            rejectedDuplicateOwner = true;
        }
        require(rejectedDuplicateOwner, "Duplicate physical ownership was accepted");

        bool rejectedBadPhysicalPage = false;
        try {
            (void)topology.remapResidentPage(physicalPages, 200U);
        } catch (const std::out_of_range&) {
            rejectedBadPhysicalPage = true;
        }
        require(rejectedBadPhysicalPage, "Out-of-range physical page was accepted");

        constexpr std::uint32_t staleTile = 200U;
        std::uint32_t firstEviction = voxel::kInvalidTile;
        require(topology.remapResidentPageIfGeneration(
                    1U, staleTile, topology.pageTable[staleTile].data[1], firstEviction),
                "A current generation was incorrectly rejected");
        const std::uint32_t staleGeneration = 0U;
        const auto pageBeforeStaleRequest = topology.pageTable[staleTile].data;
        std::uint32_t ignoredEviction = voxel::kInvalidTile;
        require(!topology.remapResidentPageIfGeneration(
                    2U, staleTile, staleGeneration, ignoredEviction),
                "A stale request generation was accepted");
        require(topology.pageTable[staleTile].data == pageBeforeStaleRequest,
                "Rejected stale request changed page ownership");

        constexpr std::uint32_t directIncomingTile = 300U;
        const std::uint32_t directEviction =
            topology.remapResidentPageFromOwner(3U, directIncomingTile, 3U);
        require(directEviction == 3U &&
                    topology.pageTable[directIncomingTile].data[0] == 3U,
                "Constant-time inverse-owner remap failed");
        bool rejectedStaleInverseOwner = false;
        try {
            (void)topology.remapResidentPageFromOwner(4U, 301U, 3U);
        } catch (const std::runtime_error&) {
            rejectedStaleInverseOwner = true;
        }
        require(rejectedStaleInverseOwner, "A stale inverse page owner was accepted");
        topology.validate();

        std::cout << "Sparse geodesic residency invariants passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Sparse residency gate failed: " << error.what() << '\n';
        return 1;
    }
}
