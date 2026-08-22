# True multiscale fractal-planet SDF laboratory

## Status

This is a new isolated target, `voxel_engine_fractal_planet_sdf_lab`. The prior
cap-local post-hit experiment is rejected and not promoted. Production f512 is
unchanged. In this lab the planet ray hit, depth, and differential normal come
from the procedural implicit field itself; an exact geodesic cap does not select
the surface first and no coarse square proxy is ever rendered.

This is the smallest real viable milestone, not literal infinite detail. It has
ten finite virtual levels and a 256-step bounded GPU traversal. The addressing
and spectral construction scale to additional levels, but every frame keeps a
finite band limit and work cap.

## Field and adaptive voxelization

The implicit surface is

```text
F(p) = length(p) - R * max(seaRadius, terrainRadius(normalize(p), bandLimit))
```

Three low-frequency, zero-mean planet bands are common to every scale. Ten
nested procedural bands double frequency and halve amplitude. A ray-cone pixel
footprint selects a continuous virtual leaf level. Only the newest band uses a
smooth transition, so approach reveals coherent smaller terrain features and
retreat reverses the same morph; common lower bands are bit-stable.

Leaves are procedural virtual addresses rather than stored meshes or a dense
volume. Level `L` has parent width divided by `2^L`; the default target is 1.25
pixels. The player collision query always evaluates the finest configured local
field. A future materialized cache can use the same deterministic address as its
page key without changing the field.

## Conservative traversal

At each potentially intersecting sample, the broad phase evaluates two detail
bands plus the exact amplitude tail of unresolved active bands. This creates a
conservative scalar interval. A range that excludes zero may advance using a
closed-form Lipschitz bound derived from the analytic band derivatives. A range
that can contain zero evaluates the selected leaf's complete band-limited field.
An eight-step bisection then returns the actual implicit root. The analytic
tangent gradient supplies the normal.

The bound is deliberately independent of leaf boundaries: the Lipschitz
inequality proves safe progress across multiple virtual addresses. Leaf scale
chooses field bandwidth, not a dangerously tiny march clamp. Nonfinite values
fail closed and telemetry reports interval skips, root work, and active leaf
levels.

The first two interactive builds failed visual validation: shallow/grazing rays
could consume the 256-step fast-path budget while converging toward the surface
without ever forming a sign bracket. More importantly, the interval skip used
`abs(coarseSigned) - unresolvedBound`. That incorrectly treated an interval
proven entirely *inside* solid terrain as empty, so a ray could skip its nearest
entry and return a distant exit sheet or the space background. This is the
concrete cause of the detached horizon sheets and black gaps in the 2026-08-16
recording; the deterministic radial field itself remains one connected closed
surface.

The corrected fast path uses the signed positive lower bound
`coarseSigned - unresolvedBound`: only a strictly positive interval is proven
empty. It retains the last proven-outside sample, brackets the first transition
to a non-positive exact field value, and never advances through a sample already
inside solid terrain. If the bounded fast march still exhausts its budget, a
front-to-back scan covers the complete relevant outer interval, including
terrain-only silhouettes above the analytic sea horizon. Coarse interval tests
prune samples before exact field evaluation; the earliest positive-to-
nonpositive bracket receives the same eight-step root solve. This recovery
never shades a sea/outer proxy and never supplies material or depth from a
coarse surface. A dedicated debug view marks recovered true roots yellow and
any unrecovered guaranteed-sea miss red.

## Collision and edits

F6 collision evaluates the same deterministic field at the finest local level
over the capsule footprint. It is not collision against a smooth sphere or the
old cap-local perturbation. Existing edits remain authoritative at their current
macro address: an edited column replaces the procedural field locally with its
exact stored geodesic cap; an empty edited column removes that local support.
Sub-leaf edit addresses and persistent dirty SDF bricks are the next edit-cache
milestone.

## Research basis

[Gamito, *Techniques for Stochastic Implicit Surface Modelling and Rendering*
(2009)](https://diglib.eg.org/bitstream/handle/10.2312/8218/gamito.pdf?isAllowed=y&sequence=1)
is the central model: point-independent deterministic band-limited stochastic
fields, analytic derivatives, fBm-style spectral refinement, and Lipschitz-safe
ray steps. The lab follows that model with common low bands, smooth spectral
activation, a closed-form derivative bound, and interval cells whose range must
contain zero before refinement. The first milestone remains a radial stochastic
implicit surface; later general implicit bands may add controlled overhangs or
caves while topological guards keep the connected main planet and reject
unintended detached noise islands.

[Ray Tracing of Signed Distance Function Grids](https://research.nvidia.com/publication/2022-09_ray-tracing-signed-distance-function-grids)
motivates broad conservative cells followed by a true local surface solve.
[Tight Bounding Boxes for Voxels and Bricks in an SDF Ray Tracer](https://research.nvidia.com/publication/2023-05_tight-bounding-boxes-voxels-and-bricks-signed-distance-field-ray-tracer)
motivates the next cache storing tight angular/radial bounds rather than visual
coarse geometry. [Heitz and Neyret's scale-continuous sparse-voxel
appearance](https://diglib.eg.org/items/5d6a143a-cd1c-4a0d-a70d-e0df2c128a61)
supports footprint-filtered high bands rather than abrupt octave toggles.
[HERO](https://diglib.eg.org/items/644e2d48-32f5-4618-bb89-6cf2a4d6933a)
and [tree-based AMR ray rendering](https://pmc.ncbi.nlm.nih.gov/articles/PMC8525884/)
inform the future ordered materialized hierarchy and 2:1 neighboring pages.

## Controls and commands

The green permanent banner states that the true field is active. The UI exposes
maximum virtual level, target pixel footprint, fine-octave relief, true shaded
surface, active leaf level, conservative interval work, and terrain-height
views. Double-clicking the executable, or launching it with no arguments,
starts the interactive lab. Automated modes remain available through explicit
commands:

```text
voxel_engine_fractal_planet_sdf_lab --fractal-planet-orbit
voxel_engine_fractal_planet_sdf_lab --fractal-planet-rotation
voxel_engine_fractal_planet_sdf_lab --fractal-planet-surface
voxel_engine_fractal_planet_sdf_lab --fractal-planet-artifact
voxel_engine_fractal_planet_sdf_lab --fractal-planet-interactive
```

The standalone executable is
`build-fractal-sdf-mingw/voxel_engine_fractal_planet_sdf_lab.exe`. Startup
failures are shown in a persistent SDL error dialog so Explorer launches do not
close without an explanation.

No result from this lab promotes production automatically.

## Validated first-milestone measurements

On the development RTX 5070 after the signed-interval and full-silhouette
correction, the 720-frame moving shallow-horizon replay measured 11.966 ms for
planet compute and 11.981 ms total compute (68.4 application FPS). It sampled
8,264,303 true SDF hits, used 546,274,128 conservative interval skips and
66,114,424 bounded root iterations, selected leaf levels p50/p95/max 6/7/8,
recovered 393,583 exact-field hits, and reported zero nonfinite events and zero
unrecovered closed-shell misses.

The deterministic video-path regression reconstructs the default surface-player
camera, 1600x900 projection and shallow horizon sweep from the recording. A
dense 4,096-sample front-to-back oracle plus 20-step bracket solve finds 183
terrain hits in the selected grid; 163 are terrain-only silhouettes which do
not intersect the analytic sea sphere. The production 256-step fast path plus
64/192-sample bounded recovery returns all 183 with zero background
misclassification. Its maximum first-hit distance error is 0.000063 world units
against the dense oracle. An independent 1,024-step signed marcher also returns
all 183. This regression would fail the old sea-only recovery and the old
absolute-value interval rule.

The lab remains isolated and visually unaccepted until the corrected
interactive build is confirmed by the user. Production f512 remains the
default.

The current virtual hierarchy is generated procedurally rather than persisted.
The next scalability step is a sparse materialized range cache keyed by the same
leaf address, with tight cell min/max and edit-dirty pages. General non-radial
implicit bands for controlled overhangs/caves are also future work; this first
accepted technical gate establishes the stochastic planet field, real adaptive
band-limited geometry, safe interval traversal, and render/collision agreement.
