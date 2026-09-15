# Eight-Planet Hex-LOD Wormhole System Lab

Status: **PHASE TWO LAB / UNPROMOTED**. This executable is compile-time
isolated from stable f512 production and from the global-static-spacetime lab.
It is an evidence-bearing system laboratory, not a production promotion.

## Build and hidden capture

Configure with `VOXEL_BUILD_EIGHT_PLANET_SYSTEM_LAB=ON` and build the standalone
`voxel_engine_eight_planet_system_lab` target. Automated rendering must use the
hidden path:

```text
voxel_engine_eight_planet_system_lab --headless-capture <output.png> \
  --capture-frames 12 --system-capture overview
```

`--system-capture` accepts `overview`, `near`, `mixed-lod`, `traversal-8-1`,
`cycle-audit`, `edit-persistence`, and `differential`. The cycle preset is a
deterministic 4×4 audit of outgoing and incoming mouths for planets 1 through
8. The traversal preset uses twelve frames per source and records the full
moving 1→2→…→8→1 sequence; its final frame is the required media-bearing 8→1
view. The SDL window is created hidden, is checked every frame, and
the run fails if it ever becomes visible. Do not use the interactive executable
for automated validation.

## Implemented isolation and data contracts

- One immutable f512 `GeodesicTopology` (2,621,442 dual cells) is uploaded once.
  Its centers, tile frames, 5/6-neighbor graph, and twelve defects are shared by
  all eight authorities. Immutable topology is never duplicated per planet.
- A second shared GPU buffer contains dual-cell hierarchy frames and neighbors
  for f1, f2, f4, f8, f16, f32, f64, f128, and f256 plus the
  f512-leaf→f256 owner map. The exact boundary is therefore one dyadic level,
  rather than the checkpoint's incorrect f64→f512 jump. Every
  displayed level has `10*f^2+2` spherical dual cells, exactly twelve pentagons,
  and otherwise hexagons. There is no cube, square, or latitude/longitude
  surface path.
- Each planet has an independent seed and generation, conservative hierarchy
  aggregates, a 1024-slot sparse page table, 512 physical pages of 32 compact
  columns, a 256-entry edit overlay, atmosphere/cloud controls, and its moving
  transform. The deterministic generator is authoritative and can be evaluated
  on a cache miss; stale or genuinely unknown generations are rejected and
  treated as full conservative columns. Edits are checked before residency and
  survive split/merge, regeneration, and eviction. The camera/player radial
  tile and its finest two-ring are pinned.
- Coarse nodes carry descendant min/max caps, a material mask/dominant
  aggregate, edit-dirty state, and maximum cap error. Below 0.25 px the GPU
  intersects conservative coarse geodesic prisms. From 0.25 to 1 px cap,
  material, and conservative side dilation morph continuously; at/above 1 px
  the f512 convex-prism DDA result is authoritative. Neighbor selection uses
  dyadic thresholds and the tested 2:1 relaxation contract. Normal shell exits
  return sky; an analytic sphere never supplies a terrain silhouette. Only an
  actual bounded-walk exhaustion can return a counted conservative solid, and
  every accepted capture reports zero such adaptive/reference fallbacks. There
  is no dither, screen crossfade, or stochastic coverage.
- Wormhole transport propagates the ray-area Jacobian, and LOD multiplies the
  projected footprint by its square root. The same packed leaf state is used by
  collision, edit picking, the CPU oracle, and GPU resident columns.
- Eight deterministic circular test orbits are separated from 7.0 to 26.25
  scene units. The default is paused. Simulation time and orbit speed are
  adjustable.
- Each planet owns a distinct incoming and outgoing mouth. The directed map is
  1→2→3→4→5→6→7→8→1 and is traversable in both directions. Mouth frames are
  derived from planet-local radial/orbit-normal axes, so they follow orbital
  motion. Position, ray, camera frame, and velocity are transported through a
  proper orthonormal endpoint-frame map, including explicit 8→1 closure.
- GPU scene ownership compares all eight conservative terrain bounds, the
  finite star, and all sixteen mouth disks before a traversal is accepted.
  Atmosphere and cloud intervals are sorted front-to-back. Source media is
  composited up to a mouth, then destination media and opaque content continue
  with the transported ray and footprint.
- The star is an emissive finite sphere. Direct light uses deterministic
  Hammersley samples distributed over its visible disk and tests planet
  occluders per sample. Star radius, radiance/color, sample count, ambient
  color/strength, all eight planet seeds/radii, orbit speed, camera preset, and
  debug output are lab controls. Ambient is independent of direct visibility.
- Finite spherical-star lighting uses deterministic Hammersley disk samples.
  Terrain uses the selected quality; cloud lighting uses a separately bounded
  deterministic subset. Ambient color/strength remains independent and energy
  bounded.
- Debug outputs cover footprint/LOD, pentagons and mixed ownership, stellar
  visibility, edited leaves, and the finest differential. The differential
  records hit, depth, and material counters and paints a mismatch red.

The lab-only push constants are 256 bytes. Production retains its existing
128-byte frame layout and compiles the system macro to zero.

## Tests

`eight_planet_system_invariants` checks:

- eight independently seeded bodies and one incoming/outgoing handle per body;
- paused determinism, stable orbital radius, and tangent velocity;
- moving-frame position/ray/velocity transport and 8→1 closure;
- eight distinct authority objects/seeds sharing one topology address, compact
  page/state round trips, conservative unknown pages, two-ring pinning, edit
  persistence through eviction/regeneration, and a collision/edit seam walk;
- exact hex/pent counts through dyadic level 6, seam neighbor valence, stable
  parent IDs, conservative aggregates, 2:1 relaxation, continuous split/merge,
  Jacobian-driven refinement, and sparse-edit persistence;
- CPU/GPU packed finest-reference cap/material equivalence for all pentagons,
  both poles, a seam, all eight seeds, and an edited leaf; the hidden GPU gate
  independently compares adaptive coverage/depth/material with a forced-f512
  trace over every pixel and all eight planet candidates;
- deterministic stellar-disk distribution, finite-radius penumbra behavior,
  the point-light limit, rejection of a center-point shortcut, bounded ambient,
  high/low position reconstruction, and front-to-back segment ordering.

The system label currently runs nine tests: one CPU oracle plus hidden 1600×900
overview, close exact voxels, mixed hex LOD, 96-frame moving traversal,
48-frame edit persistence, magnified differential, representative benchmark,
and sixteen-mouth audit captures. A bounded hidden run fails on any overflow,
stale acceptance, nonfinite output, crack/closed-shell miss, absent coarse or
exact path, or differential mismatch.

## Measured phase-two checkpoint

On the checkpoint GPU at 1600×900, the allocated data reported by the lab is
240 MiB immutable f512 topology, 63 MiB shared hierarchy/mapping, and 53 MiB
for all eight sparse authorities: 357 MiB total. This accounting is printed at
startup and guarded at 384 MiB. The increase from the earlier checkpoint is the
honest cost of retaining f128/f256 and independently packed per-planet
aggregates instead of skipping six refinement ratios at the GPU boundary.

The explicit 120-frame representative overview measured 4.788 ms terrain
compute and 4.800 ms total GPU compute at 1600×900, inside the 16.67 ms target.
The 12-frame nearby exact-voxel capture measured 174.374 ms terrain and
174.405 ms total, missing the 12 ms aspiration. The corrected differential
measured 567.283 ms terrain and 567.321 ms total because it evaluates all eight
planet candidates through both the adaptive and forced-f512 paths for every
pixel. These are hardware/driver-specific measurements; correctness and
closed-shell gates were not weakened to recover the old analytic-fallback
timing.

The captured near run reported 145,970 coarse hits, 769,494 exact hits,
166,924,010 deterministic cache-miss evaluations, and zero overflow, stale
acceptance, nonfinite output, cracks/closed-shell misses, or trace-budget
fallbacks. The magnified through-mouth differential reported 7,200,000 samples
and zero hit/depth/material mismatches, with zero adaptive or reference
fallbacks.

The final evidence run passed all nine system tests. Stable f512 subsequently
passed its complete 26/26 milestone gate, and the separately compiled global
static-spacetime lab passed all 6/6 global-metric tests. Eight distinct
1600×900 hidden artifacts were emitted for overview/benchmark, close exact
voxels, mixed LOD, moving traversal, edit persistence, differential, and the
sixteen-mouth cycle audit; automation never maps or focuses its SDL window.

The lab remains unpromoted. Its edit overlay is deliberately bounded, page
generation is deterministic rather than disk-backed, and manual subjective
moving-camera acceptance remains outside the automated evidence. Stable f512
and the global-metric laboratory retain their own compile-time layouts,
descriptors, shaders, executables, and gates.
