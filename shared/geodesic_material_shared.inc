VOXEL_SHARED_INLINE VOXEL_SHARED_UINT geodesicMaterialForLayer(
    VOXEL_SHARED_UINT filledLayers, VOXEL_SHARED_UINT surfaceMaterial,
    VOXEL_SHARED_UINT layer) {
    return layer >= filledLayers || surfaceMaterial == 0u
               ? 0u
               : (layer + 3u < filledLayers ? 5u : surfaceMaterial);
}
