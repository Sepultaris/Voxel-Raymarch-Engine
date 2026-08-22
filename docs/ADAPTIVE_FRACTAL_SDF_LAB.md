# Rejected cap-local adaptive SDF laboratory

> **Rejected concept; never promoted.** This experiment refines shading and a
> bounded root only after the fixed geodesic prism cap has already selected the
> hit. It does not make the planet SDF itself fractal and cannot reveal new
> coherent terrain geometry through real-time voxel subdivision. It remains in
> the tree only as regression/reference code. The replacement is documented in
> `FRACTAL_PLANET_SDF_LAB.md`.

## Status and non-negotiable boundary

This is an isolated frequency-512 laboratory target. It is not production and
does not replace the normal `voxel_engine` executable. The rejected square
far-field surface is neither compiled nor callable here. Event-DDA and exact
convex geodesic prism intersection still select the macro tile, radial layer,
material, side/cap classification, and edit owner before adaptive work starts.
The adaptive hierarchy can refine an accepted top cap; it cannot emit a coarse
surface, color, normal, material, or hit by itself.

The bounded viable milestone is deliberately cap-local. It does not yet create
caves, overhangs, independent subcell edit addresses, or persistent materialized
SDF bricks. Those require the next brick-cache phase described below. It does
prove the live split/merge policy, deterministic fractal source, conservative
local root, topology guards, edit authority, finest player query, artifact
behavior, and performance before committing memory to a planet-wide cache.

The interactive command starts at F6/player scale. A full-globe orbit makes an
f512 parent cell smaller than the default 1.25-pixel refinement target, so all
adaptive octaves correctly merge back to the exact parent cap and the planet
looks identical there. The permanent orange lab banner makes this state
explicit. The default A/B view shows exact parent shading on the left and the
actual bounded fractal root/gradient on the right with demonstration contrast;
it never substitutes a coarse surface. `SHOW FRACTAL FIELD` displays normalized
inward root displacement directly. The development panel labels frequency 512
and 32 radial layers as the exact parent topology and reports the virtual L0-L9
refinement policy, target footprint, and fixed seven-step local root solve.

## Implemented virtual hierarchy

- The f512 geodesic tile is level zero. Its exact five/six-neighbor topology is
  retained, including all twelve pentagons.
- A virtual cell splits 2:1 while its parent footprint exceeds the target of
  1.25 pixels. `log2(parentPixels / targetPixels)` supplies a continuous level;
  `smoothstep` fades each new octave in and reverses the same path on recession.
- A tile is never more than one requested level finer than the coarsest member
  of its exact topology ring. This is the implemented 2:1 balance rule.
- Nine bounded levels provide 512x linear refinement under an f512 parent. A
  deterministic cube-face address exists for cache keys and uses X/Y/Z tie
  priority plus half-open integer coordinates. These cube cells are indexing
  and future conservative-bound cells only; they are never rendered.
- The SDF is a world-direction analytic fractal. Nine frequency-doubling
  octaves have geometrically decreasing amplitude. Evaluated amplitude plus
  the unresolved infinite-tail bound never exceeds the configured layer
  fraction (0.15 default, 0.20 hard maximum).
- When a level is between integers, only its next octave morphs. The surface
  field and derived normal therefore vary continuously rather than popping.

## Conservative narrow phase

The exact prism cap is the entry plane. Detail is inward-only and its maximum
distance is the radial-layer height times the proven amplitude bound and the
topology support. This produces a closed ray interval. Seven fixed bisection
steps solve the analytic local implicit surface inside that interval; nonfinite
input fails to the exact cap. The solve reaches zero support before the parent
side planes and rejects incidence at or below 0.35, preserving half-open macro
ownership and the exact silhouette. Side hits, water, and edited columns remain
the original voxel surface.

The filtered level policy is also the micro-SDF policy. Distant/subpixel detail
merges to the exact cap; it is never replaced by a box. At close range the
band-limited field adds successively smaller features. Debug views separately
show applied depth, active level, and support.

## Collision and edits

F6 collision evaluates the same CPU reference field at the finest configured
level for every fresh candidate in the bounded capsule ring. Unknown profiles
remain conservative solid. The displacement is inward, so the exact macro cap
still supplies a conservative fallback. GPU edit state is authoritative:
`columnState.w != 0` suppresses the procedural SDF in both rendering and CPU
collision, and the stored edited occupancy/material remains valid across page
eviction. Edits currently address the exact macro voxel, not a virtual subcell.

## Research decisions

The architecture uses the papers as constraints rather than copying a data
structure that does not fit a geodesic planet:

- [Ray Tracing of Signed Distance Function Grids (JCGT 2022)](https://research.nvidia.com/publication/2022-09_ray-tracing-signed-distance-function-grids)
  motivates a hierarchy around potentially intersecting SDF cells followed by
  a local analytic/repeated interpolation solve. Here the existing exact DDA is
  the macro broad phase and bounded bisection is the first local solver.
- [Tight Bounding Boxes for Voxels and Bricks in an SDF Ray Tracer (Eurographics 2023)](https://research.nvidia.com/publication/2023-05_tight-bounding-boxes-voxels-and-bricks-signed-distance-field-ray-tracer)
  motivates storing tight conservative radial/angular intervals in the future
  materialized brick cache, because acceleration bounds—not coarse rendered
  geometry—are the safe place to gain performance.
- [Representing Appearance and Pre-filtering Subpixel Data in Sparse Voxel Octrees (HPG 2012)](https://diglib.eg.org/items/5d6a143a-cd1c-4a0d-a70d-e0df2c128a61)
  motivates separate macro SDF and filtered microappearance. The current
  continuous octave fade is the minimal scale-continuous implementation; a
  cached microdescriptor is future work.
- [Vector-to-Closest-Point Octree for Surface Ray-Casting (VMV 2015)](https://diglib.eg.org/items/2a189e43-92e4-403d-b91a-a29ed07137db)
  supports keeping vector/gradient information for smoother results with a
  shallower tree. The lab derives an analytic closest-surface normal but does
  not let that approximation replace an exact close hit.
- [The HERO Algorithm for Ray-Tracing Octrees](https://diglib.eg.org/items/644e2d48-32f5-4618-bb89-6cf2a4d6933a)
  motivates front-to-back ordered child addresses and first-terminal-hit
  traversal for the future materialized local bricks.
- [CPU Ray Tracing of Tree-Based Adaptive Mesh Refinement Data](https://pmc.ncbi.nlm.nih.gov/articles/PMC8525884/)
  reinforces explicit mixed-resolution reconstruction and crack tests. This
  lab applies a 2:1 ring balance and a continuous common world field; future
  sampled bricks must add boundary reconstruction equivalent in purpose to the
  paper's mixed-resolution interpolation.

## Next materialized-brick phase

If this lab is visually accepted, the next isolated step is a sparse GPU pool
of active geodesic SDF bricks. Each node stores scalar min/max, a residual-tail
bound, edit-dirty state, and an ordered child mask. Tight radial/angular bounds
skip empty nodes. Rays visit children front-to-back and the first candidate
leaf performs an exact trilinear/analytic local solve. A two-ring around F6 and
edited cells is pinned at the finest level. Neighbor faces share canonical
samples and enforce 2:1 balance. Far nodes provide bounds and filtered
appearance only—never a rendered square surface.

## Commands and acceptance

The isolated executable accepts only explicit commands:

```text
voxel_engine_adaptive_sdf_lab --adaptive-sdf-orbit
voxel_engine_adaptive_sdf_lab --adaptive-sdf-rotation
voxel_engine_adaptive_sdf_lab --adaptive-sdf-surface
voxel_engine_adaptive_sdf_lab --adaptive-sdf-artifact
voxel_engine_adaptive_sdf_lab --adaptive-sdf-interactive
```

Budgets match the established f512/f1024 experiments: <=4 ms stationary,
<=5 ms rotating, <=12 ms F6, exact collision no later than frame two, zero
streaming overflow/stale requests, and zero nonfinite, boundary, closed-shell,
recoverable-horizon, or dotted-chain failures. Passing this bounded prototype
does not automatically promote it to production.

## Validated bounded-milestone result

On the development RTX 5070, clean sequential runs measured 2.683 ms total
compute for stationary orbit (2.667 ms planet), 2.791 ms while rotating and
streaming (2.755 ms planet), and 7.623 ms for grounded F6 traversal (7.612 ms
planet). The exact adaptive collision profile was available on frame two. All
three stayed inside their 4/5/12 ms budgets with zero request overflow or stale
acceptance.

The 720-frame grazing/horizon replay exercised 436,894 refined cap samples and
3,058,258 bounded root iterations. Active refinement levels were p50 4, p95 5,
maximum 5. It reported zero nonfinite values, owner-boundary rejects,
closed-shell misses, recoverable horizon misses, or dotted side chains; all
49,718,977 sampled termination records were exact hits. The full 20-test suite
passed, including the unchanged production topology, residency, streaming,
artifact, local-AO, micro-SDF, camera, and collision gates.

The rebuilt production f512 executable measured 2.846 ms total compute in the
same stationary benchmark. The adaptive executable remains an isolated lab and
still requires visual acceptance before any promotion or materialized-brick
phase.
