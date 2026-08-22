# Traversal references and provenance

The traversal bake-off was informed by the data layouts and published benchmark discussion in these permissively licensed projects:

- [VoxelRT](https://github.com/dubiousconst282/VoxelRT) — MIT; Tree64, XBrickMap, multilevel DDA, bit-mask traversal, and comparative voxel-ray benchmarks.
- [BrickMap](https://github.com/stijnherfst/BrickMap) — MIT; sparse brick streaming, GPU feedback, and level-of-detail organization.
- [SparseVoxelOctree](https://github.com/AdamYuan/SparseVoxelOctree) — MIT; Vulkan sparse-voxel-octree construction and ray traversal.
- [voxel_ray_traversal](https://github.com/DeadlockCode/voxel_ray_traversal) — MIT or Apache-2.0; a compact Vulkan/GLSL implementation of Amanatides–Woo DDA, cached 128-bit occupancy texels, hit-axis reconstruction, and an empirical branched-versus-branchless comparison.

No source file from these repositories is copied into this project. The implementation is original and adapts the general wide-mask, spatial-ordering, stackless escape-link, and DDA concepts to the engine's five/six-neighbor convex geodesic cells. If source is directly ported later, its copyright and full license text must be added to a root `THIRD_PARTY_NOTICES.md` before that change is accepted.

The experimental wide hierarchy exists to test whether wide masked structures transfer to irregular geodesic prisms. Its same-scene result is deliberately retained even though it is slower: the result prevents an attractive cubic-grid optimization from being adopted without evidence. Direct geodesic neighbor DDA is the selected production path.

The DeadlockCode implementation reinforces two choices in the geodesic path. Boundary selection remains explicitly branched because branch removal is not assumed to reduce divergence without a same-GPU measurement. Its cached bit-packed texel read maps to our compact per-column occupancy mask: later non-contiguous radial automata traversal should consume that mask in registers and only read the detailed page for the final material hit. The Cartesian coordinate update itself is not copied because a global XYZ grid cannot represent five/six-neighbor planetary cells.

A minimum-distance guard was also tested inside the shared convex-leaf routine. Although intended to reject a candidate infinitesimally behind a DDA boundary, placing the condition in the routine used by every backend changed shader inlining and regressed the DDA measurement from 0.796 ms to roughly 4.6 ms. CPU exhaustive tests already cover ordered nearest hits, so the guard was rejected. This is exactly why traversal micro-optimizations require isolated same-scene measurements rather than source-level intuition.
