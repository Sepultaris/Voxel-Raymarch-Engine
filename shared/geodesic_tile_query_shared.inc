#ifndef VOXEL_SCALE_LAB
#define VOXEL_SCALE_LAB 0
#endif

#if VOXEL_SCALE_LAB
struct GeodesicTile {
    vec4 center;
    uvec4 neighborsLow;
    // x/y: neighbors 4/5, z: radial-depth bits, w: surface-width bits.
    uvec4 query;
};

float geodesicTileRadialDepth(GeodesicTile tile) {
    return uintBitsToFloat(tile.query.z);
}
float geodesicTileSurfaceWidth(GeodesicTile tile) {
    return uintBitsToFloat(tile.query.w);
}
uint geodesicTileLayerCount(GeodesicTile tile) {
    return 32u;
}
uint geodesicTileNeighborCount(GeodesicTile tile) {
    return tile.query.y == 0xffffffffu ? 5u : 6u;
}
uint geodesicTileBrickOffset(uint tileIndex, GeodesicTile tile) {
    return tileIndex * 32u;
}
uint geodesicTilePentagon(GeodesicTile tile) {
    return tile.query.y == 0xffffffffu ? 1u : 0u;
}
uint geodesicTileNeighbor(GeodesicTile tile, uint slot) {
    return slot < 4u ? tile.neighborsLow[slot]
         : slot == 4u ? tile.query.x : tile.query.y;
}
#else
struct GeodesicTile {
    vec4 center;
    vec4 tangent;
    vec4 bitangent;
    uvec4 neighborsLow;
    uvec4 neighborsHigh;
    uvec4 brickInfo;
};

float geodesicTileRadialDepth(GeodesicTile tile) {
    return tile.tangent.w;
}
float geodesicTileSurfaceWidth(GeodesicTile tile) {
    return tile.bitangent.w;
}
uint geodesicTileLayerCount(GeodesicTile tile) {
    return tile.brickInfo.y;
}
uint geodesicTileNeighborCount(GeodesicTile tile) {
    return tile.brickInfo.z;
}
uint geodesicTileBrickOffset(uint tileIndex, GeodesicTile tile) {
    return tile.brickInfo.x;
}
uint geodesicTilePentagon(GeodesicTile tile) {
    return tile.brickInfo.w;
}
uint geodesicTileNeighbor(GeodesicTile tile, uint slot) {
    return slot < 4u ? tile.neighborsLow[slot] : tile.neighborsHigh[slot - 4u];
}
#endif
