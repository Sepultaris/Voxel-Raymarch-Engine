# Adaptive voxelized fractal-SDF laboratory

## Status and scope

`voxel_engine_fractal_voxel_sdf_lab` is an isolated experiment. It does not
change or promote the f512 production renderer, the rejected cap-local SDF
experiment, or the earlier direct true-SDF lab. The fractal planet field remains
the source of truth, but visible hits inside a bounded camera patch are now
resolved from a materialized sampled SDF/height cache rather than by relabelling
the direct analytic hit.

This first cache milestone is intentionally a radial 2.5D brick, not a complete
planet-wide 3D sparse volume. It supports one connected terrain surface and
does not yet represent caves or overhangs. Outside the materialized patch, the
existing exact true-SDF path remains active. This keeps the demonstration
bounded while proving actual sampled voxel refinement, transition stability,
edit authority, and render/oracle agreement.

## Materialized adaptive cache

The GPU materializes two shared-corner `664 x 664` float grids in the existing
voxel storage buffer: the selected parent level and its 2:1 child. A compact
header records their common tangent frame, radial origin, cell width, virtual
level, morph, seed, and generation. The cache covers a camera-centred tangent
patch; half-open integer addressing and shared corner samples make adjacent
leaf cells continuous.

The parent and child sample the same deterministic multiscale planet field.
Approach reveals the next coherent spectral band while the parent/child field
morphs continuously. Retreat reverses that exact path. A persistent controller
splits when the projected leaf exceeds `1.60 px` and merges below `0.72 px`;
the default target is `1.25 px`. At most one 2:1 level changes per update, so
camera jitter around a threshold cannot cause unbounded refinement thrash.

The direct implicit traversal remains a conservative broad/reference query. The
lab now defaults to **Voxel surface (visible cells)**. Each half-open tangent
leaf owns a constant radial cap obtained from its four materialized source
samples. A height change between adjacent leaves therefore creates a real
radial side wall and changes ray-hit depth; this is not a grid line or normal
effect painted over a smooth surface. The root bracket follows the resulting
piecewise field and reconstructs the exact gnomonic side plane when it crosses
a leaf boundary. Tied x/y crossings combine both planes deterministically.

**Smooth SDF reconstruction** remains in the UI solely as an A/B reference. It
uses the former bilinear height and derivative. Parent-to-child cap heights
morph from the common deterministic field, and promotion makes the completed
child the new parent. A valid cache cell that cannot form a root is reported as
a closed-shell failure and fails the GPU regression. Existing edited columns
bypass procedural cache shading, so the edit overlay remains authoritative. F6
collision evaluates the same procedural field at its finest local band and is
not reduced to a coarse cache parent.

## UI and debug views

The cyan banner and window title contain `ADAPTIVE VOXELIZED FRACTAL SDF LAB`.
The lab starts at surface-player scale in the materialized-cell debug view so
it cannot be confused with production. The development panel offers:

- shaded voxelized SDF;
- projected leaf footprint;
- virtual brick level and transition;
- sampled voxel-cell grid.

It also reports selected cache level, projected leaf pixels, transition amount,
and cumulative refine/merge counts. These visualizers do not influence hit
selection.

The default is now the readable, discrete voxel surface rather than the smooth
reconstruction or grid diagnostic. The separate `Surface geometry` selector
makes this explicit; `Voxelized SDF output` continues to select diagnostic
color views and never changes geometry.
Lab-only controls expose sun azimuth/elevation, sun and ambient strength,
interpolated-normal detail, local field cavity AO, and an optional cell-edge
overlay. Water, shore/terrain, rock, and high snow use separate palettes. The
directional diffuse/specular response and compact filmic tone map execute only
after the materialized leaf root has fixed depth.

The cavity cue is not traversal-work AO. Cache generation records the negative
lobe of the finest active deterministic field band, smoothly weighted by its
octave activation. This is a stable local curvature/valley proxy at the selected
leaf scale. It is packed into the high eight bits of each cache word; height is
stored as 24-bit fixed point over the conservative `[0.96, 1.06] R` range. The
height quantization step is about `6e-9 R`, finer than float precision at the
planet radius, so the shading metadata does not alter root classification.
Runtime shading bilinearly filters both values from the same four already-read
corners and performs no additional neighborhood-buffer reads. Flat water and
convex/high field lobes remain open; valleys darken subtly. A dedicated AO
debug checkbox displays the cached cue. Cell edges fade completely below 1.45
projected pixels, preventing the target 1.25-pixel leaves from becoming a
camera-dependent stipple pattern.

The bounded cache must never become visible as a rectangular surface. Cached
and direct-SDF fallback hits now pass through the same material, light, and tone
mapping function. Normal, cavity, and grid metadata fade in across an 18-cell
interior guard band; hit depth remains the selected exact/direct result. A zero
cache word is reserved for an ungenerated sample, rejects the four-corner
interpolation, and immediately selects the direct SDF. Thus a moving, partial,
or invalid brick cannot create a white quad or false first hit. The cache test
exercises just-outside, boundary, guard-band, and deep-interior coordinates and
requires monotonic fallback-to-detail blending.

## Research basis

[Gamito, *Techniques for Stochastic Implicit Surface Modelling and Rendering*
(2009)](https://diglib.eg.org/bitstream/handle/10.2312/8218/gamito.pdf?isAllowed=y&sequence=1)
provides the deterministic, band-limited stochastic implicit field, common
low-frequency bands, smooth activation of higher bands, analytic derivatives,
and bounded root model. [Ray Tracing of Signed Distance Function Grids
(JCGT 2022)](https://research.nvidia.com/publication/2022-09_ray-tracing-signed-distance-function-grids)
motivates sampled SDF cells followed by a local interpolated surface solve.
[Tight Bounding Boxes for Voxels and Bricks in an SDF Ray Tracer
(Eurographics 2023)](https://research.nvidia.com/publication/2023-05_tight-bounding-boxes-voxels-and-bricks-signed-distance-field-ray-tracer)
supports retaining conservative bounds around materialized bricks rather than
rendering a coarse proxy. [Heitz and Neyret, *Representing Appearance and
Pre-filtering Subpixel Data in Sparse Voxel Octrees*
(HPG 2012)](https://diglib.eg.org/items/5d6a143a-cd1c-4a0d-a70d-e0df2c128a61)
motivates continuous filtering of newly visible bands instead of abrupt octave
switches.

## Commands and current measurements

Double-clicking the executable starts the interactive surface-scale lab. The
automated modes are:

```text
voxel_engine_fractal_voxel_sdf_lab --fractal-voxel-orbit
voxel_engine_fractal_voxel_sdf_lab --fractal-voxel-rotation
voxel_engine_fractal_voxel_sdf_lab --fractal-voxel-surface
voxel_engine_fractal_voxel_sdf_lab --fractal-voxel-artifact
voxel_engine_fractal_voxel_sdf_lab --fractal-voxel-interactive
```

On the development RTX 5070, the discrete-surface build measured 4.764 ms
stationary orbit and 13.454 ms grounded surface view. The moving rotation and
horizon gates are rerun for every lab change. All runs reported
zero nonfinite events and zero closed-shell misses. These remain within the
5/5/15 ms gates and are within normal run variance of the pre-shading
4.60/4.60/13.42/12.56 ms measurements. The CPU cache test completed ten splits,
ten merges, and 2,401 direct-field oracle rays with no hit mismatch, seam
discontinuity, or hysteresis thrash. It additionally verifies bounded,
monotonic field-cavity shading, smooth octave activation, subpixel edge
suppression, packed-height error below `1e-7 R`, constant within-cell cap
height, distinct neighboring step heights, smooth cache-edge fallback, and new
deterministic geometry when a leaf level refines.

These results meet this bounded experiment's 5/5/15 ms gates. They do not make
the cache planet-wide, provide subleaf edits, or promote it to production. The
next step, if the visual result is accepted, is a sparse 3D brick pool with
canonical shared face samples, conservative SDF ranges, edit-dirty pages, and a
finest pinned collision ring.
