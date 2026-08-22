# Voxel Planet Engine Architecture

## Non-negotiable constraints

- Rendering traversal and voxel intersection execute on the GPU.
- Planet-scale positions never depend on a single world-space `float`.
- The geodesic grid partitions ownership and streaming. Local regular bricks provide efficient storage and DDA traversal.
- Every long-range marching distance is conservative. Approximate fractal distance estimators must be bounded before they can control a step.
- Simulation state remains GPU-resident during normal play. CPU readback is diagnostic or persistence work, never part of the frame loop.

## Frame graph direction

1. Apply queued player and world edits to resident voxel bricks.
2. Build or update the active-brick list.
3. Run cellular automata compute passes against double-buffered material state.
4. Rebuild occupancy masks, macro distance bounds, and dirty acceleration data.
5. Ray march planetary bounds and empty space.
6. Enter a local brick and perform exact DDA traversal.
7. Refine intersections against the cell's analytic or sampled SDF.
8. Shade into a compute storage image, composite development UI, and present.

The initial executable now implements the first hybrid traversal slice. It conservatively marches an analytic macro sphere, switches to a GPU-resident voxel volume near the surface, skips empty 8x8x8 macrocells, and performs fine DDA against material cells. A GPU initialization pass creates both material and occupancy data without CPU staging.

Camera rays explicitly convert top-left Vulkan image coordinates into camera space with positive Y upward. The GPU voxel brush uses the same conversion, keeping visual output and edit picking aligned.

The camera has independent orbital-inspection and surface-player states. The surface controller stores a unit radial direction and a unit tangent-forward direction; it does not store latitude as navigation state. `WASD` input becomes a tangent vector, and movement applies a great-circle rotation to both radial and forward vectors. Rotating the complete frame parallel-transports heading along the sphere, while a subsequent projection removes accumulated floating-point drift. Planet radial is always local up/gravity. This construction remains finite and continuous at the poles, anti-meridian, and geodesic seams.

F6 surface mode owns a physical radial capsule instead of moving a free camera. Its total height is four radial layers, radius is 45% of local cell width, eye position remains inside the upper capsule, standing skin is 3% of a layer, and the maximum automatic step is 1.05 layers. Gravity and jump speed scale from radial layer height. Tangent motion is divided into sweeps no longer than 45% of the capsule radius. For each candidate radial direction, a bounded topology walk gathers only prisms whose conservative dual-polygon extent intersects the swept footprint. Their top planes use the renderer's exact brick depth, occupied-layer count, and radial overlap. A rise above the step allowance blocks the great-circle substep; lower caps are reached through radial gravity.

The compact column profile remains GPU-authoritative. Forty-three low-cost streaming-feedback lanes report the current tile, its one-ring, and its two-ring every surface frame, including already-resident pages. CPU collision consumes those exact occupancy masks/layer counts one frame later. Terrain regeneration invalidates the snapshot, and a missing profile is treated as a fully occupied column until refreshed; this may briefly hold the player high but cannot permit penetration. GPU edits become visible through the same feedback path. Jolt is deliberately not used for this bounded controller because duplicating millions of changing geodesic prisms as collision bodies would create a second terrain authority and unnecessary broadphase work.

For GPU camera construction, the radial direction is encoded as yaw/pitch only at submission time. The parallel-transported forward vector becomes a heading relative to a matching stable tangent basis. Recomputing that heading every frame cancels the basis's pole fallback transition, so it cannot produce a visible orientation jump. Collision mode submits the capsule's exact eye radius; noclip mode retains the conservative maximum-height clearance. First-person look, edit picking, render rays, streaming selection, collision feedback, and CPU page eviction all consume the same camera radial/frame state.

## Falling-sand voxel model

Each logical cell contains compact material and state data: material identifier, phase, temperature, velocity hint, and material-specific state. Storage will be structure-of-arrays where independent passes benefit from narrower bandwidth.

Updates use source and destination buffers. A source cell proposes movement or reaction; a deterministic destination-claim pass resolves collisions; a commit pass writes the winning result. Checkerboard or multi-color scheduling may be used for materials whose behavior benefits from immediate local propagation, but results must not depend on GPU workgroup execution order.

The current prototype implements this algorithm for granular material under radial gravity. Each fixed 60 Hz simulation tick clears destination claims, sand cells atomically claim empty inward neighbors, and a commit pass writes the winning moves into the alternate material buffer while rebuilding macrocell occupancy. If the direct inward cell is blocked, a deterministic tick-varying search ranks four inward-diagonal cells so grains can slide around curved surfaces. Every accepted move must strictly reduce squared distance to the planet center; therefore grains cannot crawl laterally forever or oscillate between equal-potential cells. Rendering consumes the newly committed buffer in the same command stream. Up to four ticks may be recorded after a hitch; excess accumulated time is discarded to prevent a simulation spiral.

The development reset scene uses a sparse granular cap separated from a smaller solid core. This provides a short visible fall and a bounded settling test without the long congestion tail caused by filling an entire planetary hemisphere with sand.

Only active bricks are scheduled. Sleeping bricks retain state without consuming per-frame simulation work. Activity propagates across brick and planetary-tile borders through halo cells and explicit neighbor tables; no shader infers geodesic adjacency from floating-point positions.

## Rigid-body physics boundary

Jolt Physics 5.6.0 provides CPU-side rigid bodies, constraints, characters, vehicles, triggers, and broad/narrow-phase collision queries. It is built in double-precision mode and stepped at a fixed 60 Hz independently of rendering. The GPU cellular automata remains authoritative for granular materials, liquids, gases, heat, and reactions.

Nearby resident voxel bricks will asynchronously produce simplified Jolt collision shapes when their solid occupancy changes. Those shapes are streaming proxies, not the authoritative voxel representation. Dynamic Jolt bodies can enqueue bounded voxel impulses or material edits for a later GPU pass; the frame loop must never synchronously read the voxel field back from the GPU for collision construction.

Planetary gravity is applied per rigid body toward its active planet center. Physics operates in rebased planet-local frames; double precision extends the safe range but does not remove the need for local origins when multiple planets or astronomical distances are present.

## Planet topology

The implemented global surface index is the dual of a configurable-frequency refined icosahedron. Its tile count obeys the closed-form icosphere count `10f^2 + 2`: exactly twelve have five neighbors and every other tile has six. Startup validation checks tile and triangle counts, reciprocal adjacency, neighbor ranges, and the pentagon invariant before any data is uploaded.

Each tile stores its unit surface normal, an orthonormal tangent/bitangent frame, tangent half-extent, radial depth, approximate local surface-cell width, cyclic neighbor indices, and a contiguous brick-pool offset. The dense reference assigns a configurable radial brick of up to 32 layers to every surface column. Hexagonal columns contain six-sided prism/frustum cells; the twelve singular columns contain pentagonal prism/frustum cells. Their taper is required for concentric planetary shells.

The radial aspect ratio is derived from geometry rather than terrain noise. A layer's radial height is `0.5 * localSurfaceCellWidth`, and total brick depth is that height times the configured layer count. Production-frequency planets retain this ratio exactly. Coarse diagnostic topologies cap total depth at 25% of planet radius so their direction-based DDA chart cannot approach or cross the center. Relief extends inward from the unchanged outer cap; maximum-height corner bounds and camera clearance therefore remain conservative without inflation.

These are volumetric cells, not a texture or ID overlay on a sphere. Each cell is the intersection of an inner cap plane, an outer cap plane, and one inward half-space per explicit neighbor. The compute renderer performs convex ray clipping against those planes. A shared neighbor boundary is represented once mathematically: one column owns one half-space and its neighbor owns the complementary half-space. Consecutive radial cells likewise reuse a single cap plane. No inset or visualization gap is applied, so occupied cells meet exactly.

Ray depth and the selected voxel face always remain exact. For shading, the hit side records its exact neighbor and the shader computes the logical exposed cliff height from the two column layer counts. Its screen projection needs no derivatives. A side must resolve in both dimensions—projected radial height and incidence-scaled tangential width—and must also pass an explicit angular-incidence gate. That final gate matters close to the surface, where the large projected cell width can otherwise overwhelm an almost edge-on incidence and restore crawling radial chains. Cap faces are never filtered; unresolved side faces transition from one to six projected pixels toward the column's surface material, top-layer tone, and radial cap normal. Equal-height neighbors and layers buried behind a taller neighbor get zero side contrast. This is a shading/coverage LOD rather than a geometry LOD: it removes point-sampled stippling without erasing nearby face-on cliffs, changing silhouettes, or affecting DDA correctness.

Local voxel ambient occlusion is a post-hit shading term, never a traversal input. The exact hit column/layer and compact occupied heights of its ordered five- or six-column ring define a deterministic one-ring horizon and consecutive-neighbor concavity term. Surface-cell width divided by radial-layer height supplies the local horizontal run, so the cue respects the physical voxel aspect ratio without another geometry query. Side hits weight the two corner-adjacent sectors most heavily and cap their contribution, preserving enclosed step corners without creating dark stripes along exposed cliffs. The calculation reads no material page or page table and has no camera, ray direction, distance, FOV, resolution, hierarchy, temporal, or stochastic input. The separate scale-lab DDA work heatmap is diagnostic only and is not AO.

The f512 production render dispatch receives a lighting-only reinterpretation of
push-constant fields that terrain generation and editing use in their own
dispatches. This retains the portable 128-byte layout without corrupting those
systems. The lighting payload contains a world-space sun, radial-hemisphere sky
colors, material response, exposure, and debug selection. A short directional
contact horizon consults only the exact one-ring compact column heights and is
disabled on side hits. Lighting runs after DDA, micro-detail, and side-coverage
classification; background/sky rays bypass it completely.

The production miss path is an infinitely distant procedural environment. Its
world-oriented direction atlas is independent of camera position; optional
camera alignment substitutes the normalized view-local ray intentionally. Two
spherical cellular grids use deterministic integer hashes and keep each star's
anti-aliased footprint inside its owning cell, so no neighbor search, stochastic
dither, time input, or atlas-boundary pop is required. Planet angular clearance
multiplies the result by a zero-to-one silhouette exclusion outside the
conservative shell. This environment cannot submit a terrain candidate or
modify any hit status.

Bounded terrain micro-SDF detail is another post-hit shading refinement and is
compiled into the production f512 path only. Exact DDA and convex-prism
intersection first establish the parent tile, radial layer, face, depth, and
material. Top-cap hits may then evaluate a three-octave analytic field in
world-direction space and move only the shading point inward inside the same
radial layer and central owner support. Two fixed evaluations, a 17.5%-layer
hard amplitude bound, finite/radial guards, and a support fade that reaches zero
before every owner plane make escape fail closed. The activation also fades by
projected cell footprint and rejects grazing incidence, preventing temporal
speckle without introducing a coarse rendered surface. Water, side hits, and
edited columns are masked. The stored hit distance, edit/pick result, occupancy,
collision cap, streaming, and hierarchy are never modified; a terrain edit
simply suppresses the visual micro-detail for that column.

## Procedural planetary height

Height generation is a topology-wide compute dispatch, not a CPU mesh bake or a texture mapped onto a smooth sphere. Each surface-tile center is treated as a unit direction. Three independent 2D value-noise fields sample the `YZ`, `XZ`, and `XY` orthographic projections; fourth-power absolute-normal weights blend them continuously. Because the signal is a continuous function of the unit direction, it crosses the conventional anti-meridian, poles, and dominant-axis transitions without UV seams.

Low-frequency four-octave fractal noise establishes continents. A separate three-octave ridged field adds mountain structure, with ocean threshold and polar uplift applied afterward. Continental displacement is signed so marginal regions are pushed downward as interiors rise. Squared ridge peaks create taller massifs, while the inverse ridge signal explicitly carves inland basins. The resulting normalized height becomes a bottom-contiguous radial occupancy mask and occupied-layer count. Surface bands select vegetation/soil, beach sand, water, snow/ice, or exposed rock. A single shared CPU/GLSL layer-profile function assigns rock below the upper three-layer surface band. It drives startup generation, page materialization, validation, and rendering, making the material at a column/layer independent of physical page residency.

The seed and terrain parameters travel in the existing push-constant block. Startup generation writes every compact column and all initially resident pages on the GPU. Interactive regeneration does the same in the frame command stream and inserts a compute barrier before edits, streaming feedback, and rendering. This keeps topology-scale terrain work off the CPU and avoids allocating a planet-sized height texture.

The GPU receives device-local buffers for tile metadata, a spherical direction lookup, the radial-brick material pool, compact column state, a binary reference BVH, and a stackless wide-mask hierarchy. All traversal backends finish with the same exact convex intersection routine, keeping visibility independent of camera angle, silhouette position, and current terrain height.

The default backend is event-based geodesic neighbor DDA. A ray first intersects the conservative radius containing every stretched prism corner and uses the spherical atlas exactly once to seed a canonical half-open owner. From then on the current tile is carried only through its explicit five/six-neighbor topology. Side-plane crossings are computed from unnormalized bisector planes. Exactly parallel planes are classified without division; nonparallel clipping first classifies both interval endpoints and divides only when the plane actually clips the interval. Progress uses the next representable floating-point distance, so there is no world-scale crossing epsilon.

Every side crossing within four traversal ULPs of the earliest crossing belongs to one event. Face events have one entered owner; edge and vertex events enumerate every tied owner, test their expanded convex prisms, and select the post-event half-open owner deterministically by topology index. Post-event classification probes four representable distances forward without division. If every entered owner is rejected, and only then, the resolver inspects that event's exact local neighbor ring. Ordinary intervals never invoke a broad fallback or atlas reclassification. The render and edit shaders use the same transition policy. CPU differential gates compare the first DDA hit with exhaustive convex-prism intersections for the shallow basin camera patch and shared-face/triple-point rays jittered by one through four ULPs; divergence diagnostics include the ray, tile, event distance, tied owner IDs, next owner, step count, and termination. The 2,048-event bound covers long frequency-512 grazing rays.

The experimental wide backend Morton-orders surface columns, places up to four exact prism items in a leaf, and stores a 64-bit active mask in every node. Nodes are preorder-linearized with escape links, enabling stackless traversal and whole-subtree skips. Internal nodes currently use up to eight conservative spatial children; the remaining mask capacity is reserved for future surface/radial occupancy refinement. It reduces acceleration-structure storage substantially, but same-scene profiling shows that testing irregular child AABBs costs more than direct neighbor DDA.

An additional isolated adaptive-fractal-SDF lab retains that exact DDA as its
macro owner and refines only a confirmed top cap. Its virtual 2:1 hierarchy is
screen-adaptive but never rendered: it controls how many band-limited analytic
SDF octaves participate in a conservative local root interval. Neighbor-ring
balancing, continuous octave morphing, inward amplitude bounds, side-plane
support fade, edit masking, and finest local player evaluation provide the
first bounded proof before allocating persistent SDF bricks. See
[ADAPTIVE_FRACTAL_SDF_LAB.md](ADAPTIVE_FRACTAL_SDF_LAB.md) for research basis,
commands, measured limits, and the materialized-brick follow-up.

Reference acceleration is opt-in. Production builds skip binary/wide construction, allocate no corresponding Vulkan buffers, and compile those bindings and branches out of the planet shader. Tests continue to construct both hierarchies as independent ray-equivalence oracles. Profiling builds enable them with `VOXEL_ENABLE_REFERENCE_TRAVERSAL`; production avoids approximately 75 MiB at frequency 256 and 300 MiB at frequency 512.

Development edits use the same BVH and convex intersection path. A hit changes the top occupied layer in that tile's GPU-resident radial brick without CPU readback. BVH bounds cover maximum column occupancy, so layer additions and removals do not require a rebuild or refit.

## Production spherical atmosphere

The stable f512 renderer composites a render-only atmosphere after exact
geodesic DDA shading. In the default physical mode its inner density support is
below the complete radial voxel band, not above the outer cap, and its outer
sphere is configurable. Thus the integration interval reaches every exact
mountain, water, and valley hit. The previous raised inner boundary is retained
only as an opt-in artistic fade and is disabled by default. The terrain ray ends
at the exact prism hit; a sky ray ends at the outer-shell exit. Consequently the
medium cannot invent a hit, move depth, hide a coverage failure, or enter
editing/player collision.

The integrator is deterministic and noise-free. It samples the altitude field
at a closest-approach-weighted analytic quadrature point. Rayleigh, Mie, and
ozone-like absorption each contribute wavelength-dependent extinction to RGB
Beer-Lambert transmittance. Rayleigh uses its symmetric phase; Mie uses a
tunable Henyey-Greenstein anisotropy and a shorter scale height. Absorption has
no in-scattering and preferentially removes the broad green/yellow band.
Sun-path extinction uses the same coefficients with a curvature-aware bounded
air-mass approximation; a direct ray/sphere discriminant supplies planet
occultation and the atmospheric terminator. This intentionally omits the
expensive high-dimensional LUTs and multiple-scattering orders of the references
while retaining their crucial separation `L = T * scene + in-scattering` for
both terrain aerial perspective and sky/star rays:

- [Hillaire, *A Scalable and Production Ready Sky and Atmosphere Rendering Technique*, 2020](https://diglib.eg.org/items/8a3e5350-18b3-46bd-9274-3add5af88c75)
- [Bruneton and Neyret, *Precomputed Atmospheric Scattering*, 2008](https://diglib.eg.org/bitstreams/8526ee8e-15ba-4779-b7ba-2f8334772eb2/download)

The spherical core test is supplemented by a terrain-aware directional horizon
query. Starting from a half-open geodesic owner, it advances four, eight, or
sixteen explicit neighbors toward the projected sun position. Each step tests
the two closest topology candidates. A ray/radial-line and compact maximum-cap
test rejects impossible blockers first; the rare survivors use the same exact
convex side/cap clipping as visibility. The query is deliberately bounded and
returns only sun visibility—it cannot affect camera hit ownership or depth.
Compact column heights make the result resident/nonresident equivalent and
immediately reflect edits. Direct hit lighting and camera-to-hit atmosphere
share one column-stable result; only sky in-scatter performs its own bounded
query. Unknown state is never inferred from a missing detailed page.

The default single-planet optical integration is adapted from the user's own
read-only `C:\Users\Sepul\SpaceBattleSimulator` implementation, principally
`DepthAwarePlanetAtmosphere.shader`, its
`DepthAwarePlanetAtmosphereCompositor.cs` upload convention, and the
`AtmosphereOpticsModel` mirror in `ProceduralPlanetAtmosphereVolume.cs`.
Ordered Gauss-Legendre camera samples accumulate RGB Beer-Lambert extinction
front to back. A separate two/four-sample fixed sun quadrature replaces the
old air-mass estimate at medium/high quality. The reference Rayleigh, broad Mie,
and short-wavelength absorption basis is applied only in this mode; its radial
limb/outer-rim gain, warm terminator, composition-derived night spectrum, and
0.0005 transmittance early exit are retained in bounded form. Terrain and cloud
sun visibility are evaluated once at the representative optical point and
shared across the samples, avoiding nested topology/cloud marches.

Unity/HDRP depth reconstruction and Custom Pass code, multi-atmosphere arrays,
portal/ring/gas-giant branches, and 3D cloud texture slots were intentionally
not copied. The terrestrial reference has no atmosphere transmittance LUT: it
integrates analytic density directly, so the Vulkan port likewise needs no new
sampled-image binding or startup precompute. One world at the origin collapses
its array parameters into existing settings. A final-render-only phase word's
previously unused high byte carries mode, 2/3/5 view quality, 2/2/4 sun quality,
and night strength. The push block remains 128 bytes and non-render passes see
their original payload. The local reference is user-authored, used with the
author's explicit permission, and was not modified.

Three existing packed color words retain their RGB payloads and use their
unused high bits for atmosphere height/density/mode controls. Two render-only
float words are bit-packed with the existing star size/brightness plus spectral
coefficients, scale height, and anisotropy. The push block remains exactly 128
bytes, and only the final render dispatch sees this reinterpretation.

## Production volumetric clouds

The f512 production compositor contains a hollow spherical cloud medium above
the conservative terrain radius. Exact ray/sphere intervals subtract the inner
sphere from the outer interval, yielding at most two ordered segments. Those
segments are clipped at the already accepted terrain distance, so clouds are
strictly a radiance operation and cannot modify a voxel hit or create a proxy
surface. The baseline procedural source is a deterministic world-space 3D
multi-frequency field with a smooth base/top profile, coverage threshold, and
bounded detail erosion. Camera position is not an input; animation translates
the field only when both wind speed and the global animation clock are enabled.
A centralized source function makes a future authored 3D texture a descriptor
change rather than a shell/integrator rewrite. Selecting that not-yet-bound
source currently falls back safely and visibly to procedural density.

The view integrator uses Beer-Lambert segment extinction, forward
Henyey-Greenstein Mie lighting, radial sky ambient, and a bounded powder-style
multiple-scattering approximation. It runs after spectral atmosphere so exact
terrain, atmospheric in-scatter, and background stars are all behind the cloud
medium. Production low quality uses three midpoint view samples; medium/high
use six/ten. There is no temporal jitter, history buffer, or stochastic dither.

For sun visibility, the same density is integrated along the cloud-shell part
of the sun ray with two/four/six samples. Exact terrain shading multiplies this
visibility with the existing geodesic terrain-horizon result. Atmospheric
in-scatter uses that already computed hit visibility, while a sky ray performs
one representative sun-path query. The cloud compositor then reuses the same
value, avoiding a duplicate shadow march. These shadows affect direct light and
in-scatter only; terrain ownership, page residency, edit picking, and player
collision remain untouched.

Cloud state consumes two final-render-only words that were unused by the
production planet shader plus high bytes of the diagnostic phase word. Terrain
generation, streaming, and edit passes continue receiving the original values,
and the push constant block remains 128 bytes. The model follows the procedural
amplification ideas in [Goswami and Neyret 2017](https://diglib.eg.org/items/20f37d97-4db9-42a3-857a-99e5ea02de1e)
and the bounded single-scattering raymarch structure in [Tóth and Umenhoffer
2009](https://diglib.eg.org/items/07d33179-3056-49d8-b404-c13be46c8baf),
without copying assets or implementing their full simulation systems.

The combined cloud and depth-integrated atmosphere defaults measured 2.907 ms
for the stationary orbit pass, 2.939 ms while rotating/streaming, and 8.483 ms
in the close grounded player
view on the development RTX 5070. The complete production gate contains 22
tests; its 720-frame artifact replay classified 49,718,977 exact terrain hits
without a closed-shell/recoverable miss, dotted side chain, streaming overflow,
or stale request.

The isolated f1024/R2 scale lab adds a broad/narrow exact query without changing
the production target. Its broad phase is an eleven-level angular macro
hierarchy containing conservative radial min/max envelopes. The hot scalar
walker can advance only inside proven-empty radial/angular intervals and can
never produce a rendered result. The narrow phase remains the shared ordered
event-DDA and exact convex-prism intersection for shading, silhouettes, picking,
edits, and F6 collision. A 48-byte lab-only query record retains the center, all
neighbor IDs, and exact radial/surface metrics while the full 96-byte topology
record remains authoritative on the CPU and in the versioned cache. The
rejected far-field visual LOD is bypassed. Current measurements and the explicit
non-promotion status are recorded in the README scale-lab section.

Each virtual column has a compact 16-byte GPU state record containing a 32-layer occupancy mask, occupied layer count, and top material. Ray candidates read this record to obtain the top layer in constant time instead of scanning a radial brick. A second 16-byte entry maps the virtual column to a physical page, generation, and flags. Until arbitrary per-layer automata state is enabled for geodesic columns, rendering resolves the shared compact material profile for both resident and nonresident candidates; physical cells remain available for edits and future simulation. Edit passes update the material cell when resident and always update coarse column state before a compute-to-compute buffer barrier makes the result visible to rendering.

The first sparse pool caps physical detail at 8,192 column pages while retaining 655,362 virtual columns. Reusing a physical page invalidates its previous virtual owner, advances its generation, assigns the new owner, and deterministically materializes bottom-contiguous cell state.

Camera-driven feedback is connected with one-frame latency. A dedicated compute pass launches a fixed 16,384-thread low-discrepancy sampler rather than one thread per surface column. Samples cover a camera-centered spherical cap targeting 90% of physical page capacity; this headroom prevents stationary boundary churn and provides hysteresis during motion. A Cranley-Patterson phase shift changes radius and azimuth each frame so stationary sampling converges. Each direction resolves through the spherical lookup and a bounded neighbor walk.

Nonresident samples atomically stamp the reserved word of their page-table entry with the current frame generation. Duplicate directions therefore collapse on the GPU without a separate topology-sized bitset. Unique requests append tile index, expected page generation, and coarse state to the 16,384-entry feedback queue. Request-generation work is now independent of total virtual column count. The CPU consumes feedback only after the existing frame fence, rejects generation mismatches, and services at most 256 pages. Eviction candidates are ranked once per frame by camera alignment so far-hemisphere pages leave first. A maintained physical-page-to-virtual-owner table supplies the eviction owner directly, so each remap is O(1) rather than scanning all virtual columns.

Accepted pages and changed table entries are packed into a persistently mapped staging allocation. The next command buffer performs one multi-region copy into the physical cell pool and one into the page table, followed by a transfer-to-compute barrier. No queue-idle operation appears in the frame path. Coarse state carried in feedback preserves GPU height edits during materialization. Dirty detailed-page persistence and use of a dedicated transfer queue are not yet implemented.

Reference BVH leaves and wide-hierarchy bounds use AABBs constructed from the exact maximum-height dual-polygon corner vertices at the inner and outer caps, with only a numerical safety pad. Binary traversal carries each node's ray-entry distance on its stack and visits the nearer child first. These retained backends provide independent correctness oracles and controlled profiling comparisons for the default DDA path.

Two Vulkan timestamp queries bracket only the planet compute dispatch. Results are read after the next frame fence and converted using the physical device's timestamp period and valid-bit mask. Presentation, ImGui, CPU topology work, cellular simulation, and queue waiting are excluded. The bounded `--benchmark` mode renders 120 frames and prints the final measurement for repeatable local comparisons.

The dense reference scale is frequency 256 with 32 radial layers: 655,362 columns, 20,971,584 addressable voxels, and 1,310,723 binary BVH nodes. Active sparse streaming holds 8,192 pages / 262,144 detailed cells. On the development RTX 5070, bounded deduplicated streaming plus geodesic DDA sustains 482.8 FPS at 0.693 ms when stationary and converges to zero requests. The rotating workload sustains 257.5 FPS at 0.796 ms while uploading 256 pages per frame, with zero overflow and zero stale acceptance.

The production scale is frequency 512 with 32 radial layers: 2,621,442 columns and 83,886,144 addressable voxels with the same 262,144 physical detailed cells. The release RTX 5070 baseline is 1.203 ms stationary and 1.437 ms rotating; request work remains 16,384 samples and stationary feedback converges to zero.

## Planned milestones

1. Vulkan compute presentation, validation, SDL input, ImGui, and OpenAL context.
2. Camera controls plus conservative sphere/heightfield marching. **Initial slice complete.**
3. Sparse regular bricks, occupancy hierarchy, and GPU DDA. **Single-resident-volume slice complete.**
4. GPU falling-sand material buffers and deterministic movement claims. **Radial-gravity prototype complete.**
5. Geodesic tile addressing, twelve-pentagon neighbor tables, tile-local frames, and resident brick attachment. **Initial slice complete.**
6. Planet-scale residency, asynchronous transfer, LOD, persistence, and profiling. **Bounded page pool, GPU feedback, camera-centered detail selection, and staged upload slices complete.**
7. Jolt collision-proxy generation for dirty resident bricks, character physics, and GPU-to-CPU interaction queues.
