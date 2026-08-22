# Voxel Raymarch Engine

A GPU-driven planetary voxel engine prototype built with C++20, Vulkan, GLSL, SDL, Dear ImGui, OpenAL, and Jolt Physics. The target is a sparse planet-scale world whose local voxels support falling-sand cellular automata behavior.

## Current milestone

The planet now has a validated configurable-frequency icosahedral dual topology. A frequency `f` build contains `10f^2 + 2` geodesic surface columns: exactly 12 pentagonal columns and all remaining columns hexagonal. Every column has a cyclic, reciprocal neighbor table, a planet-local tangent frame, and up to 32 GPU-resident radial voxel layers. Tile metadata, occupancy/material state, brick storage, and independent traversal structures are uploaded to device-local Vulkan buffers.

The default development view renders the cells as actual convex volumes rather than coloring a smooth sphere. A GPU ray is clipped against each candidate voxel's inner cap, outer cap, and five or six side planes. Neighboring columns use complementary halves of the exact same bisector plane, while consecutive radial cells reuse the exact same cap plane. The occupied voxel complex is therefore watertight with no artificial cracks. The default visibility path enters the conservative planetary bound, locates the first surface cell, and performs exact DDA across the explicit five/six-neighbor graph. It therefore avoids searching a global BVH for every pixel.

Geodesic traversal is now an event-based DDA. The spherical atlas seeds the ray once; every later transition follows explicit tile adjacency. Shared boundaries use deterministic half-open ownership by topology index, side events within four representable traversal distances are grouped, and all tied edge/vertex crossings advance as one event. Plane clipping classifies endpoints before dividing, handles exactly parallel planes without division, and progresses with the next representable distance instead of a fixed epsilon. A division-free post-event ownership probe selects the unique interval; only a rejected tied owner expands to its exact local topology ring. Render and GPU-edit picking share this policy. The shallow-horizon frequency-512 capture completes 67,760,653 classified terrain rays with zero no-owner, far-bound, step-limit, closed-shell, or recoverable misses. The optimized normal f512 benchmark measures 277.3 FPS / 2.97 ms GPU on the development RTX 5070; visual confirmation on the user's original camera path remains a separate release check.

Exact side-face lighting is retained whenever a real voxel step is large enough to resolve on screen. The shader derives the exposed height from the logical layer-count difference across the hit side and evaluates projected wall height, incidence-scaled width, and a direct angular-incidence gate before applying a 1-to-6-pixel transition. The angular gate is necessary close to the surface, where a large cell can otherwise pass the width threshold while its radial wall is still visually edge-on and crawls as a dotted chain under camera motion. Only side-face material, depth tone, and normal contrast are filtered toward the column cap aggregate; intersection depth, silhouettes, and DDA ordering remain unchanged. Equal-layer columns therefore cannot turn microscopic frame-width differences into stippling, while nearby face-on voxel cliffs retain their full geometry and contrast.

Production lighting is evaluated only after that exact/filtered hit representation
is fixed. A world-space directional sun has adjustable azimuth, elevation, color,
and intensity. A local radial hemisphere blends user-selected horizon and zenith
sky colors, so caps, side walls, poles, and the southern hemisphere receive
stable ambient light without a camera-space bias. Water, snow, rock, sand, and
vegetation use material-aware rough specular response; a restrained sky rim,
exposure, and filmic tone mapping preserve readable highlights. Existing
camera-independent one-ring voxel AO remains the concavity term. An optional
one-ring directional contact horizon affects cap interiors only, preventing dark
radial side stripes. Noon and sunset presets plus diffuse, sky/occlusion, normal,
and AO debug views are available in the top-level `LIGHTING` panel. None of these
terms can change hit selection, depth, editing, collision, streaming, or sky
misses.

Miss rays now enter a camera-centered black space environment instead of the
old blue gradient. Two deterministic spherical cellular atlases provide fine
and brighter sparse stars with varied warm/cool color, brightness, and smoothly
filtered pixel footprints. The atlas reads only ray direction, so camera
translation has no parallax and time introduces no twinkle or temporal noise.
The default world-oriented mode reveals different stars as the camera turns;
an explicit camera-aligned option can pin the atlas to the view. A conservative
clearance band suppresses stars around the planet silhouette before compositing,
but only when the forward ray actually approaches the planet. Rays aimed into
the antipodal sky retain full star visibility instead of inheriting a duplicate
infinite-line silhouette. This prevents stars from resembling lost terrain rays
without creating a camera-distance-dependent dark disk behind the observer.
Density, brightness, size,
orientation, enable, and debug controls are in the top-level `SPACE ENVIRONMENT`
panel. Planet hits and DDA state never enter this background path.

The production f512 path now surrounds that exact voxel planet with a finite
spherical participating-medium shell. Physical mode is enabled by default and
places its base conservatively below the complete 32-layer terrain band, so
mountains, water, and the deepest valleys all receive camera-to-exact-hit
attenuation. The old raised base is available only as an explicitly disabled
artistic near-surface fade. A ray is analytically clipped to the shell, then a
deterministic closest-approach quadrature evaluates separate exponential
Rayleigh and lower-scale-height Mie densities plus an ozone-inspired absorption
layer. RGB Beer-Lambert extinction removes out-scattered and absorbed light;
turquoise Rayleigh and tunable Henyey-Greenstein forward Mie contributions are
added as in-scatter. A curvature-aware sun air-mass approximation and exact
core-sphere test provide sun extinction, terminator, and night-side shadow.
Terrain colors are composited through only the camera-to-hit segment without
changing voxel depth; miss rays integrate to the shell exit, so procedural
stars remain behind and are attenuated rather than replaced. The default
0.18-radius shell is deliberately thick and stylized for a readable small
planet. Shell extent, density, Rayleigh/Mie/absorption strengths, scale height,
Mie anisotropy, surface/fade modes, warm preset, and component debug views are
in `PLANET ATMOSPHERE`.

Terrain now participates in the atmospheric sun path rather than relying on the
spherical core terminator alone. A bounded directional horizon query starts at
the exact hit's geodesic column (or the column below a sky in-scatter sample),
walks explicit sunward adjacency, broad-rejects columns whose radial envelope
cannot meet the sun ray, and clips only the remaining candidates against their
exact occupied convex prism. The default eight-column query is deterministic
per world column, reads the compact residency-independent height profile, and
therefore remains unchanged by camera motion, page eviction, or material-page
state. GPU edits update that same profile. Direct terrain sun and atmospheric
Rayleigh/Mie in-scatter share the resulting visibility, producing
terrain-shaped ridge shadows and twilight modulation without another full
global DDA. Sky rays use their own bounded query; terrain-hit rays reuse the
already computed hit-column result. `Terrain atmosphere shadows`, quality,
distance, strength, and a sun-visibility debug output expose the tradeoff.
The controls occupy previously unused bits of the render-only reinterpretation
of the existing 128-byte push block; terrain generation, edits, collision, and
streaming still receive the original constants.

The light-transport structure follows Bruneton and Neyret's separation of
attenuated scene radiance and integrated in-scattering in
[Precomputed Atmospheric Scattering (2008)](https://diglib.eg.org/bitstreams/8526ee8e-15ba-4779-b7ba-2f8334772eb2/download),
while the dynamic artistic parameterization and explicit sky/aerial-perspective
split are informed by Hillaire's
[A Scalable and Production Ready Sky and Atmosphere Rendering Technique (2020)](https://diglib.eg.org/items/8a3e5350-18b3-46bd-9274-3add5af88c75).
This engine uses a bounded analytic spectral single-scattering approximation,
not either paper's full precomputed multiple-scattering solution.

The default atmosphere integrator now also adapts the user's own production
depth-aware atmosphere work from the read-only local
`C:\Users\Sepul\SpaceBattleSimulator` project. The primary references were:

- `Assets/SpaceBattlePrototype/Shaders/DepthAwarePlanetAtmosphere.shader`
- `Assets/SpaceBattlePrototype/Runtime/DepthAwarePlanetAtmosphereCompositor.cs`
- `Assets/SpaceBattlePrototype/Runtime/ProceduralPlanetAtmosphereVolume.cs`
  (including its `AtmosphereOpticsModel` CPU mirror)

The port retains the reference's ordered opaque-depth-clipped integration,
fixed Gauss quadrature, normalized Rayleigh/Mie/absorption spectrum, radial
limb and warm-terminator shaping, derived composition-based night scatter, and
sub-pixel transmittance early exit. It deliberately omits Unity/HDRP Custom
Pass glue, multi-body/light arrays, portals, rings, gas giants, and its cloud
texture-slot system. The reference uses direct analytic density integration for
terrestrial atmosphere rather than an atmosphere transmittance LUT, so no fake
or unnecessary LUT resource was introduced here. Our exact DDA terrain depth,
geodesic terrain-sun visibility, and procedural cloud optical visibility remain
the authoritative endpoint and occluders.

`SpaceBattle depth-integrated optics` is enabled by default. Low/medium/high
use 2/3/5 ordered camera samples and 2/2/4 sun samples; medium is the default.
`Derived night scattering` controls the same-spectrum non-solar envelope. The
legacy one-point closest-approach model remains selectable for performance A/B.
These controls occupy the formerly unused high byte of a final-render-only
phase word, leaving the Vulkan push block exactly 128 bytes. This local
reference is user-authored and was used with the author's explicit permission;
the source project remained read-only.

The same production path now adds a world-space volumetric cloud layer between
the atmosphere and the final camera. Rays are analytically clipped to a hollow
spherical cloud shell and marched only through the one or two remaining shell
segments in front of an exact terrain hit or the sky. A deterministic 3D
multi-frequency procedural density supplies coverage, body shape, and eroded
detail; it is independent of camera position and has no stochastic frame
jitter. Wind is optional and paused by default. Beer-Lambert extinction
attenuates terrain, atmospheric sky, and stars, while a forward
Henyey-Greenstein Mie term, sky ambient, and a restrained powder approximation
provide the cloud radiance. The density-source interface already reserves an
authored 3D-texture mode; until such a descriptor is bound it safely reports
and uses the procedural source rather than sampling invalid data.

Clouds also cast world-stable shadows. Each lit terrain hit evaluates a bounded
sun path through the same density field, composes that visibility with the
existing exact geodesic terrain-horizon shadow, and shares the result with the
atmospheric in-scatter pass. Sky atmosphere performs one corresponding bounded
query, which the later cloud compositing pass reuses. The low/medium/high
budgets are 3/6/10 view samples and 2/4/6 sun samples; low is the production
default. Density, shell heights, coverage, shape/detail, wind, quality, shadow
strength, source status, and density/transmittance/shadow diagnostics are in
`VOLUMETRIC CLOUDS`. This render-only medium cannot change DDA coverage, hit
depth, editing, streaming, or F6 collision. Its procedural-amplification and
bounded participating-medium design is informed by:

- [Goswami and Neyret, *Real-time Landscape-size Convective Clouds Simulation and Rendering*, 2017](https://diglib.eg.org/items/20f37d97-4db9-42a3-857a-99e5ea02de1e)
- [Tóth and Umenhoffer, *Real-time Volumetric Lighting in Participating Media*, 2009](https://diglib.eg.org/items/07d33179-3056-49d8-b404-c13be46c8baf)

No assets or source code from either work are copied.

On the development RTX 5070, the validated cloud plus depth-integrated
atmosphere defaults measured 2.907 ms stationary orbit, 2.939 ms while
rotating/streaming, and 8.483 ms in
the close grounded F6 view. The 720-frame artifact replay classified
49,718,977 terrain hits with zero closed-shell/recoverable misses, zero dotted
side chains, zero non-finite micro-detail events, and zero request overflow or
stale acceptance. All 22 production milestone tests pass.

A deterministic compute generator now maps procedural height orthographically onto the planet through three blended axis projections. Fourth-power normal weights remove cube-style projection seams without introducing a longitude seam or polar singularity. The generator writes all 2.62 million compact column states directly on the GPU and expands resident pages into radial material cells with rock interiors and water, beach, vegetation, exposed-rock, and snow surfaces. Seed, continent and mountain scales, signed continent/valley relief, mountain relief, roughness, ocean coverage, and polar uplift can be changed in Dear ImGui and applied with one GPU regeneration dispatch. The stronger default profile uses broader mountains, signed continental displacement, and explicit inland basin carving.

Radial voxels now have a physical height approximately half their local geodesic surface-cell width. At frequency 512 the full 32-layer brick therefore spans roughly twenty surface-cell widths inward, making the generated mountain-to-valley range visible at first-person scale. The outer cap remains at the original planet radius, so increased relief extends valleys inward without invalidating the conservative outer bound or surface-player clearance.

The application creates a Vulkan swapchain and a GPU-initialized 96-cubed material volume. Long-range rays conservatively march an analytic planetary bound, then switch to occupancy-assisted voxel DDA near the surface. Empty 8-cubed macrocells are skipped before fine traversal.

Granular material uses double-buffered GPU storage. A claim compute pass resolves competing moves deterministically under radial gravity, and a commit pass writes the next state while rebuilding macrocell occupancy. The automata runs on a fixed 60 Hz clock with bounded catch-up, independent of render rate. Rendering consumes committed state without CPU readback. The compute result is copied into the presentation image and Dear ImGui is drawn as the final overlay. OpenAL initializes independently so a missing audio device does not prevent graphics startup.

Jolt Physics handles rigid bodies, characters, vehicles, constraints, and triggers in a fixed-step CPU subsystem. It is intentionally separate from the GPU cellular material simulation. Jolt is pinned to version 5.6.0 and built with double-precision positions for large-world support.

The current testbed supports direct GPU voxel interaction. In the geodesic view, left click raises the nearest prism column by one radial layer and right click lowers it; neighbor-DDA picking, convex intersection, and the material-pool edit remain GPU compute work. In the legacy dense-volume view, the same buttons place sand or remove the first hit voxel. The camera panel or `F6` switches between orbital inspection and a collision-aware first-person surface player without discarding the orbital camera state. Orbit mode uses middle-mouse drag and wheel zoom and can approach to a radius-scaled 1% safety margin outside the topology's conservative outer bound.

Surface-player mode captures the mouse for first-person look, uses `WASD` to walk, `Shift` to sprint, and `Space` to jump. Escape releases the mouse so the development UI can be used; middle click re-enters control. The player's radial vector is gravity/up. Great-circle steps parallel-transport the frame, so poles, the anti-meridian, and geodesic seams cannot flip controls. At f512 the pill is derived from the real voxel scale: radius `0.001347`, height `0.005987` (exactly four `0.00149685` radial layers), eye height `0.005381`, skin `0.0000449`, and step height `0.001572` (1.05 layers) for radius-one planets. Movement is swept in substeps no longer than 45% of the capsule radius. Each substep queries the same compact occupied-layer profile and convex cap planes used by rendering; tall neighboring prisms block motion, one-layer steps are climbable, and radial gravity settles onto lower caps. A GPU feedback probe keeps the local two-ring CPU collision snapshot current even for resident pages and after edits. Pending profiles fail conservatively at maximum column height, never as empty terrain. The clearly labeled `Debug noclip / free altitude` option restores `Q`/`E` and wheel altitude controls.

Jolt remains available to the engine but is not used for planetary player collision. A Jolt integration would require mirroring millions of mutable pentagonal/hexagonal prisms into a second broadphase representation. The bounded local swept-capsule query is smaller, respects the authoritative residency-independent voxel profile directly, and avoids CPU/GPU geometry disagreement.

`F8` toggles the per-pixel ray-status overlay without moving the camera. Terrain
cap/side hits, validated empty sky, and resolver-confirmed no-owner, far-bound,
or step-limit coverage failures use separate colors. The development panel shows
the center pixel's GPU-recorded hit class, termination, ray interval, candidate
distance, tile/layer/material, closest approach, and normal. A bounded replay is
available as `voxel_engine.exe --ray-status-capture`.
Launch `voxel_engine.exe --ray-status` to enter the interactive overlay directly
when a keyboard shortcut may be intercepted by recording software.
`F9`, or `voxel_engine.exe --ray-failure-overlay`, preserves normal shading and
recolors only exact-resolver-confirmed lost terrain rays. This classifies a
suspect pixel without switching away from the view that produced it. The
lightweight interactive path audits a rolling 1/32 pixel subset; every pixel is
checked within 32 frames and confirmed failures flash brightly, while full
records remain confined to explicit status/regression modes.

`voxel_engine.exe --surface-benchmark` runs 120 deterministic moving surface-player frames, requires valid sparse-page streaming, prints timing diagnostics, and exits automatically.

The development panel can pause the automata, reset its initial material shell, or advance exactly one 60 Hz tick for inspecting movement and conflict resolution.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the intended hybrid ray-march/DDA and GPU cellular-automata design.
Algorithm references and code-provenance rules are recorded in [docs/TRAVERSAL_REFERENCES.md](docs/TRAVERSAL_REFERENCES.md).

An opt-in [spherical wormhole portal laboratory](docs/PORTAL_LAB.md) integrates
real three-dimensional view rays through an Ellis-inspired bounded optical
field, transfers throat-crossing rays into a linked endpoint frame, and then
uses the unchanged exact f512 voxel/atmosphere/cloud path. It is a separate
executable and shader set; production remains unpromoted. Its corrected RK4
path reaches an analytic throat or influence-boundary event instead of stopping
on discrete interior shells, and physical F6/orbit camera crossings now map the
camera pose and velocity to the linked endpoint with collision revalidation and
anti-ping-pong hysteresis.

A newer, still isolated [intrinsic Ellis manifold laboratory](docs/INTRINSIC_ELLIS_MANIFOLD_LAB.md)
keeps the free-fly observer and all view rays natively in signed proper depth
`l` and angular direction `n`, parallel-transports the observer tetrad through
`l=0`, and maps only asymptotic ray exits into the engine's exact Euclidean
content renderer. It has no finite influence sphere, Euclidean camera
teleport, or screen-space warp. Build it explicitly with
`VOXEL_BUILD_INTRINSIC_PORTAL_LAB=ON`; neither f512 production nor the older
comparison lab is changed.

The isolated [global static spacetime laboratory](docs/GLOBAL_STATIC_SPACETIME_LAB.md)
is the first same-universe metric-field engine stage: one engineered smooth
non-vacuum handle metric with non-compact flat-limit tails, two symmetric
physical mouth charts, a shared-exterior free-fly camera, and matching runtime
GPU geodesics feeding exact local voxel-content queries. A throat event only
remaps coordinates and the transported tetrad; no finite mouth sphere selects
rendering, media, or star policy. It is a fixed static spacetime—not an
evolving Einstein-equation solver—and builds only with
`VOXEL_BUILD_GLOBAL_METRIC_LAB=ON`.

Milestones are gated by automated topology, geometry, terrain determinism/seams, traversal-equivalence, SPIR-V, and Vulkan runtime tests. See [docs/MILESTONE_GATES.md](docs/MILESTONE_GATES.md). Run the complete gate with `cmake --build build-mingw --target milestone_gates -j 8`.

The cap-local adaptive-SDF experiment is rejected because it perturbed a surface
only after a fixed geodesic cap had already selected the hit. Its isolated code
remains for regression only. A separate true fractal-planet SDF laboratory now
derives ray hit, depth, and normal from a deterministic multiscale implicit
field. Pixel footprint selects procedural virtual leaves and smoothly adds
coherent spectral bands; conservative scalar intervals and analytic Lipschitz
bounds advance through proven-empty space before an eight-step exact local root.
No coarse square surface is rendered. F6 collision evaluates the same finest
local field, while edited macro columns override it. This finite ten-level lab
does not replace production; see
[docs/FRACTAL_PLANET_SDF_LAB.md](docs/FRACTAL_PLANET_SDF_LAB.md).

## Configure and build

Provide an ImGui source directory or allow CMake to fetch it:

```powershell
cmake -S . -B build -G "MinGW Makefiles" `
  -DVOXEL_IMGUI_SOURCE_DIR=C:/path/to/imgui
cmake --build build -j
```

The Vulkan SDK's `glslc` shader compiler must be installed. SDL2, Vulkan loader, and OpenAL development libraries must match the selected C++ compiler ABI.

Jolt Physics 5.6.0 is fetched and built by default. A graphics-only tooling build can omit it with `-DVOXEL_ENABLE_JOLT=OFF`.

The default production scale uses geodesic frequency 512 and 32 radial layers: 2,621,442 columns and 83,886,144 addressable voxels. These are configurable at generation time with `-DVOXEL_GEODESIC_FREQUENCY=...` and `-DVOXEL_TILE_RADIAL_LAYERS=...`. Only 8,192 detailed pages remain physical, and visibility plus request-generation costs are bounded rather than linear in that virtual count.

The dense reference gate uses frequency 256 and 32 radial layers: 655,362 columns, 20,971,584 addressable voxels, and 1,310,723 retained binary-BVH nodes. The binary tree is now a regression oracle rather than the default render path.

The current sparse slice preserves that entire virtual address space while capping detailed storage at 8,192 physical column pages, or 262,144 cells. A GPU page table maps virtual columns to physical pages. Nonresident columns continue to render and accept height changes from compact coarse state, so page faults do not make holes. Physical pages carry generations for safe reuse and can be deterministically evicted and materialized from coarse state. One shared CPU/GLSL layer-profile rule reconstructs rock strata and the three-layer surface band, so moving a column into or out of the resident cache cannot change its visible material.

Camera-driven streaming is active. A dedicated GPU pass takes exactly 16,384 low-discrepancy samples from a camera-centered spherical cap, independent of total topology size. Each direction uses the spherical lookup plus a short neighbor walk to resolve a virtual column. The page table's reserved word supplies a per-frame atomic deduplication stamp, so each nonresident tile reaches the CPU at most once without another topology-sized allocation. The cap targets 90% of physical capacity to leave motion hysteresis. After the frame fence, up to 256 generation-checked requests are serviced; eviction prefers pages on the hemisphere farthest from the camera. An inverse physical-page owner table makes every remap constant-time. Page data and both changed page-table entries are packed into one persistent staging block and submitted as two batched GPU copies without a queue-idle wait.

The rotating-camera regression runs 120 frames with continuous cache churn. The column request pass and O(1) remapping removed the original per-pixel atomic and linear-owner-scan bottlenecks; geodesic DDA now takes the traversal path further. Dirty-page persistence and a dedicated background transfer queue remain future work.

Column candidates use a compact GPU occupancy record to find their top layer in one read. The renderer no longer scans radial voxel cells or branches on page residency for visible material; it evaluates the same compact layer profile used to materialize detailed pages.

The production shader contains only geodesic neighbor DDA. The original near-first binary BVH and stackless Morton-ordered wide hierarchy remain available in builds configured with `-DVOXEL_ENABLE_REFERENCE_TRAVERSAL=ON`; the default build neither constructs nor uploads them. This saves roughly 75 MiB at frequency 256 and about 300 MiB at frequency 512 while also removing their runtime shader branches.

On the development RTX 5070, the original traversal bake-off measured 2.356 ms for the binary BVH, 13.156 ms for the experimental wide hierarchy, and 0.796 ms for geodesic DDA. With bounded deduplicated streaming, stationary geodesic DDA sustains 482.8 FPS at 0.693 ms and converges to zero requests. The rotating workload sustains 257.5 FPS at 0.796 ms with zero feedback overflow or stale acceptance. Use `--benchmark-bvh`, `--benchmark-wide`, or `--benchmark-dda` for same-scene comparisons.

At the promoted frequency-512 scale, the release build renders 2.62 million columns / 83.89 million virtual voxels at 1.203 ms stationary and 1.437 ms while rotating. Stationary streaming converges to zero requests; rotation maintains 256 uploads per frame with no overflow or stale acceptance.
# Isolated f1024 scale lab

> **Rejected visual experiment:** the f1024 far-field aggregate/LOD path remains
> isolated for profiling and regression work, but its visibly square distant
> terrain is not approved for production or promotion. The f512 production
> executable continues to use the original full-resolution visual path.

The production target remains `voxel_engine` at f512, radius 1, with a
512x256 direction atlas. An opt-in scalability laboratory is available with
`-DVOXEL_BUILD_SCALE_LAB=ON`. It creates a separate
`voxel_engine_scale_lab` executable and `shaders-scale-lab` shader directory;
it does not change production defaults or assets.

The lab configuration is fixed to f1024, radius 2, a 1024x512 atlas, 32 radial
layers, 8192 resident pages, and no reference BVH. It must be invoked with one
of these explicit bounded commands:

```
voxel_engine_scale_lab --scale-lab-orbit
voxel_engine_scale_lab --scale-lab-rotation
voxel_engine_scale_lab --scale-lab-surface
voxel_engine_scale_lab --scale-lab-artifact
voxel_engine_scale_lab --scale-lab-interactive
```

Append `--force-regenerate-topology` to any bounded command for a measured cold
generation. `voxel_engine_scale_lab --scale-lab-clear-cache` removes only the
exact lab cache after an explicit request; normal startup never deletes it.

Before building the topology or allocating GPU metadata, the executable checks
the exact tile/cell counts, 32-bit addressing, the portable 128-byte push
constant payload, a 1.5 GiB GPU metadata budget, and an 8 GiB conservative CPU
peak estimate. A visible SDL window title and console progress stream reports
cache validation/read/write, topology generation stages, GPU upload, terrain
initialization, and ready state. A 55-second internal watchdog leaves time for
cleanup before the strict 60-second wall-clock cap.
A failed gate terminates with a readable reason instead of falling back to a
smaller configuration.

The radius-independent topology cache is stored at
`cache/scale-lab/geodesic-f1024-l32-a1024x512-p8192-v1.bin` (about 1.25 GiB).
Its header is keyed by cache version, frequency, layers, atlas dimensions,
resident pages, reference-traversal mode, endianness, and every serialized GPU
structure size. Tile/count invariants and a payload checksum are validated
before reuse. Regeneration writes a `.partial` sibling and atomically publishes
it only after the complete file is flushed, so interruption cannot make a
partial file look valid.

Successful bounded runs print separate streaming and planet-compute timestamps,
cumulative DDA-event and atlas-refinement p50/p95/p99/max histograms, page
overflow/stale counters, startup time, process peak memory, and (for the F6
surface run) the first frame with an exact local collision profile. Passing the
lab never changes production configuration automatically.

The current lab query is a two-stage exact path. A GPU-built eleven-level
angular macro hierarchy stores conservative occupied radial minima/maxima; its
compact maximum envelope advances rays only through the minimum of a proven
radial gap and a proven angular-cell boundary distance. It never returns a hit,
material, normal, depth, or color. The narrow phase is always the watertight
leaf event-DDA and exact convex-prism intersection. Every visible result,
silhouette, edit/pick ray, and F6 collision query therefore stays exact. Any edit
invalidates broad-phase stepping until regeneration while exact traversal
remains available. The rejected square far-surface path is not callable.

The f1024 shader reads a versioned 48-byte ray-query tile record containing the
exact center, all five/six neighbor IDs, radial depth, and surface width. The
full 96-byte topology record remains the cache/CPU/edit source, so compaction
does not discard topology data or migrate the 1.25 GiB cache. Lab cap AO is also
precomputed from the same exact one-ring into unused column-state bits; side and
corner hits keep the layer-aware local calculation. Edits refresh the affected
AO ring. Production f512 retains its original 96-byte layout and AO path.

A sampled GPU differential audit reruns pure exact DDA and compares first-hit
identity and distance against the hybrid path. The distant-horizon gate also
checks grazing/side coverage, missed-shell status, non-finite bound events, and
dotted chains. Both production and the lab apply optional local geodesic voxel
AO only after an exact hit. The hit layer and the compact occupied heights of
the tile's ordered five/six-neighbor ring form a small deterministic horizon;
flat caps and convex ridges remain open while bowls and enclosed step corners
darken. The cue has no camera, distance, FOV, resolution, hierarchy, material
page, residency, or random input. It defaults to a subtle 0.12 strength and has
an intuitive grayscale debug view where dark means locally occluded. The old
event-count visual is retained in the lab only as the explicitly named `DDA
work heatmap`; it is not ambient occlusion.

Production f512 also adds optional bounded terrain micro-detail only after the
exact event-DDA has accepted a top-cap hit. A deterministic, world-direction
three-octave analytic field refines the shading point inward by 7.5% of one
radial layer by default (17.5% hard maximum), using two fixed refinement
evaluations. The effect fades smoothly between a five- and fourteen-pixel cell
footprint and to zero before the central cell-support boundary, so it cannot
cross a geodesic owner plane, change a macro silhouette, or select another
voxel. Side walls, water, grazing/silhouette rays, and edited columns remain
exact and flat. Edits therefore suppress micro-detail rather than reseeding it;
collision, picking, stored depth, occupancy, and materials remain the original
voxel result. The `Terrain micro detail` UI exposes enable, depth, and a field /
activation debug view; `--no-micro-sdf` provides a deterministic comparison
run. The f1024 lab path keeps this feature disabled and remains unpromoted.

The production artifact replay exercised 436,895 active micro-detail samples
with zero non-finite rejects, owner-boundary escapes, or radial oversteps and no
dotted chains or recoverable terrain misses. On the development RTX 5070 the
default orbit pass measured 2.868 ms versus the preceding 2.846 ms baseline;
paired enabled/disabled surface runs were 7.983/8.141 ms, so the bounded
post-hit work introduced no measurable regression beyond run-to-run variance.

This exact-only experiment is **not accepted or promoted**. On the development
RTX 5070, compact query records reduced the earlier 6.71 ms stationary result
to a best measured 4.25 ms, but it still missed the 4 ms gate (observed range
4.25-5.51 ms). Rotation measured 5.65 ms against the 5 ms gate. F6 measured
5.64 ms against its 12 ms gate with an exact collision profile on frame 2; its
DDA p99/max were 255/523, within the 512/2048 limits. The 720-frame horizon
audit compared 16,249,400 hybrid rays with pure exact DDA and found zero hit or
distance mismatches, zero non-finite bound events, zero closed-shell/recoverable
misses, and zero dotted chains. Since the full acceptance set did not pass,
interactive use requires an explicit manual evaluation request and does not
promote the lab; f512 remains the production configuration. Cold startup
completed in 18.69 seconds at a 4.49 GiB process peak; warm startup measured
1.75-2.55 seconds at 2.17 GiB. Preflight now accounts for the compact records
and both hierarchy buffers exactly: 0.797 GiB of GPU metadata against the 1.5
GiB cap. The existing 1.25 GiB topology cache is unchanged; the approximately
13.3 MiB min/max accelerator is regenerated on the GPU during the visible
startup sequence. The attempted per-ray wide frontier traversal was rejected
because register spills raised the pass to 8-14 ms; the retained scalar bound
keeps profiling out of the hot path. DDA/bound histograms are collected in the
dedicated differential run, not the clean timing runs. The unchanged f512 production benchmark
measured 2.79 ms with zero overflow/stale requests.
