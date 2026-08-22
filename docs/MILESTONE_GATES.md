# Milestone Gates

A milestone is not complete until the `milestone_gates` target passes. Visual inspection can add evidence, but it cannot replace a gate. A regression found by a user must receive an automated reproduction before the affected milestone is considered stable.

## Required command

```powershell
cmake --build build-mingw --target milestone_gates -j 8
```

This target builds the engine and optimized shaders, validates every SPIR-V module, and runs all tests labeled `milestone`.

## Current enforced gates

### Build and shader gate

- The complete C++ engine links.
- Every GLSL shader compiles for Vulkan 1.2.
- `spirv-val` accepts every generated module.
- The production planet shader compiles without binary/wide traversal bindings or branches.
- Production topology validation rejects retained reference acceleration data.

### Topology and state gate

- Frequencies 1, 2, 4, 8, 16, and 32 obey `10f^2 + 2`.
- Every topology contains exactly twelve pentagons.
- All other tiles are hexagons.
- Adjacency is reciprocal and in range.
- Brick offsets, occupancy masks, layer counts, and material cells agree.
- Every tile appears exactly once in the binary reference BVH and wide hierarchy.
- Invalid radial-mask sizes are rejected.

### Geometry and acceleration gate

- Adjacent maximum-height prism caps share exactly one two-corner edge.
- Every inner and outer prism vertex is contained by its BVH leaf.
- BVH child and leaf indices are valid.
- BVH ray coverage matches exhaustive column intersection.
- Nearest hit distances agree across multiple camera pitches, yaws, center rays, and silhouette rays.
- Stackless wide-mask traversal and geodesic neighbor DDA independently match the same exhaustive hits.
- Geodesic DDA seeds from the spherical atlas once, then carries the current owner exclusively through explicit adjacency.
- Shared faces use deterministic half-open topology/index ownership; crossings tied within four traversal ULPs advance as one face/edge/vertex event.
- Exact parallel-plane handling performs no division, interval clipping divides only after endpoint sign classification, and progression uses the next representable distance.
- The post-event owner is validated division-free; an exact local owner ring is inspected only when tied candidates are rejected.
- A synthetic low basin viewed from a grazing surface-player camera has continuous hit coverage and matches exhaustive nearest-hit distances, including paths spanning more than six cells.
- Near-horizontal rays constructed in a shared side plane, and rays through a three-column geodesic vertex, match exhaustive convex-prism depth after direction components are jittered by one through four ULPs in both directions.
- The shallow surface-player basin patch compares DDA hit/sky classification and first-hit distance against exhaustive column intersection. First divergence diagnostics record ray, step, tile, event distance, tied owners, next owner, and termination.
- Render traversal and GPU edit picking compile from the same event and half-open ownership policy.
- Wide-node masks, preorder escape links, child ranges, and leaf item ranges are valid.

### Sparse residency gate

- Virtual capacity covers every geodesic column independently of physical allocation.
- The detailed cell pool never exceeds its configured physical page count.
- Resident virtual pages own unique, in-range physical pages.
- Nonresident virtual pages cannot retain physical addresses.
- Eviction invalidates the old owner, advances the reuse generation, and preserves pool size.
- Materialized page occupancy agrees with coarse column state.
- Every materialized layer exactly matches the shared nonresident reconstruction rule.

### Visual consistency gate

- Every surface material and layer count resolves identically with and without a resident page.
- Subpixel exposed side faces enter the analytic shading filter using logical neighbor layer difference rather than total cell width.
- Equal-layer microscopic side strips receive no unstable side-face contrast.
- Tall radial walls viewed edge-on are filtered by projected width and direct angular incidence, while the same nearby cliff viewed face-on retains full contrast.
- Resolved nearby cliffs retain full geometric normal, layer tone, and material contrast.
- Terrain cap shading is never distance-filtered.

### Procedural terrain gate

- A fixed seed produces bit-identical reference height, occupancy, and material results.
- Changing the seed materially changes the sampled planet.
- Anti-meridian, polar, and orthographic dominant-axis transitions remain continuous.
- Every generated height remains inside its 32-layer radial brick.
- Every occupancy mask is bottom-contiguous and agrees with its layer count.
- Default parameters exercise water, beach, land, and at least one highland material band.
- Default normalized terrain spans more than 48% of the radial brick.
- At least 1% of reference samples form high mountains and at least 12.5% form low basins.
- Uncapped production-scale radial layers are half their local surface-cell width.
- Coarse diagnostic bricks remain within the 25%-radius center-safety cap.

### Camera-control gate

- Close zoom remains outside the conservative convex planet bound.
- The default planet can be approached to less than 2% radial clearance.
- The safety floor scales with planet radius.
- Mouse-wheel increments become finer near the surface.
- Surface-player radial and tangent-forward vectors remain normalized and orthogonal.
- Mixed great-circle movement and heading changes remain finite across poles and seams.
- First-person look pitch remains inside its pole-safe range.
- The production capsule is exactly four radial layers tall, with radius, eye, skin, step, gravity, and jump derived from the same f512 voxel H/W geometry.
- Deterministic capsule tests cover flat resting, downhill gravity, legal one-layer climbing, over-height cliff blocking, jump apex/landing, camera nonpenetration, and pole/seam frame transport.
- Collision candidates come from a bounded local topology walk and the renderer's compact occupied-layer profile; unknown GPU feedback is maximum-height solid rather than empty.
- Collision mode disables free altitude controls. The explicit debug noclip option retains Q/E and wheel altitude without being confused with the F6 player default.
- Surface-player eye radius remains above the contacted prism cap and inside the standing capsule.
- Orbital settings remain independent when the surface controller is selected.

### Vulkan runtime smoke gate

- SDL, Vulkan, OpenAL, ImGui, Jolt, topology upload, descriptors, pipelines, and synchronization initialize.
- Three complete frames render and present: one orbital frame and two moving surface-player frames.
- Vulkan validation errors or fatal runtime errors fail the process.
- The test has a 60-second timeout and exclusive GPU lock.

### Vulkan sparse-streaming gate

- A bounded 20-frame Vulkan run rotates the camera continuously to force cache churn.
- Its fixed 16,384-sample camera-cap pass must receive at least one nonresident GPU request.
- Request sampling must remain below topology size at the frequency-256 scale gate.
- Per-frame page-table generation stamps must deduplicate repeated tile samples on the GPU.
- At least one requested physical page must be materialized and uploaded.
- The camera-centered detail policy must not overflow its feedback queue.
- No stale page generation may be accepted.
- The CPU invariant gate verifies direct inverse-owner remapping and rejects stale inverse owners;
  runtime streaming therefore cannot regress to a virtual-page-table scan per eviction.
- The test shares the exclusive GPU lock and 60-second timeout.

### Vulkan surface-artifact regression gate

- A deterministic 720-frame surface-player replay has two phases: the original
  close shallow-angle heading sweep/diagonal walk and a low-clearance,
  near-horizontal distant-ridgeline sweep matching the reported horizon specks.
- Procedural stars are anchored to world-space ray directions rather than the
  screen pixel grid. The angular band immediately above the conservative
  planetary silhouette must contain zero star pixels, preventing background
  detail from masquerading as moving terrain or coverage speckles.
- GPU readback classifies cap hits, non-exposed, filtered, partial, and resolved
  side hits, invalid neighbors, and every DDA termination path.
- The interactive `F8` ray-status overlay is independent of background shading.
  Its center probe records the exact ray interval, nearest candidate, owner,
  layer, material, closest approach, and normal; only an exact-resolver-confirmed
  lost terrain hit receives a bright failure color.
- Diagnostic records retain the pixel, tile/layer/neighbor, column heights,
  material, ray, distance, side incidence, projected footprint, and local
  side-interior coverage needed to reproduce a bad sample.
- Closed-shell rays may not terminate at no-owner, far-bound, or step-limit
  exits, and the diagnostic buffer may not overflow.
- Ordinary horizon rays reported as sky are checked by a diagnostic-only exact
  shell-owner resolver; any recoverable terrain hit fails the gate.
- A spatial detector rejects separated collinear high-contrast side samples
  forming the reported radial dotted-chain pattern while allowing isolated tips
  of genuinely resolved nearby cliffs.
- Subpixel side edge/corner hits use cap-aggregate shading; wall material,
  normal, and layer contrast require a fully resolved local face interior.
- Rare grazing no-owner events resolve a bounded set of cap owners and immediate
  geodesic neighbors, retaining the exact nearest convex-prism hit.
- Each ordered DDA interval reclassifies radial cap ownership at the actual cap
  distance, and terminal horizon recovery uses a coarse terrain-height envelope
  before its bounded exact shell candidates so true sky remains inexpensive.
- Local geodesic AO is evaluated only after an exact hit from the hit layer,
  compact column height, and ordered five/six-neighbor heights. Flat caps and
  convex ridges remain open; progressively deeper bowls are monotonically
  darker; exposed top-layer cliffs do not become dark stripes.
- AO is invariant under ordered-ring rotation at seams and under camera
  distance/azimuth, FOV, resolution, hierarchy enablement, material, and page
  residency changes. Its output is bounded, and the default cannot darken a
  fully enclosed sample by more than 12 percent.
- Terrain micro-SDF refinement runs only after an exact top-cap hit and cannot
  modify the parent tile/layer, exact stored depth, material, edit/pick result,
  occupancy, or collision profile.
- Its deterministic world-space three-octave field is finite and owner-invariant
  across pentagon/hexagon seams. The two-step refinement is bounded to 17.5% of
  one radial layer and fades to zero before the central owner boundary.
- Side hits, water, edits, grazing/silhouette rays, and subpixel cells remain
  unmodified. Camera distance, azimuth, FOV, resolution, and residency cannot
  reseed the field; projected footprint only applies a smooth deterministic
  activation fade.
- The artifact replay must exercise active micro-detail samples while reporting
  zero non-finite rejects, owner-boundary escapes, radial oversteps, dotted
  chains, or recoverable/closed-shell misses.
- Run this gate directly with `voxel_engine.exe --artifact-regression`; it has a
  240-second timeout and shares the exclusive GPU lock.

### True fractal-planet SDF lab gate

- The isolated `VOXEL_BUILD_FRACTAL_PLANET_SDF_LAB` target must prove that the
  implicit field itself selects hit depth and normal. Gates cover deterministic
  nested virtual-leaf addresses, monotonically smaller physical leaves on
  camera approach, smoothly shared spectral bands, conservative unresolved-band
  ranges, closed-form Lipschitz growth, seam continuity, nonfinite fail-closed
  behavior, finest local F6 collision, and edited-column override.
- GPU orbit, rotation, surface-player, and 720-frame horizon profiles must stay
  within 4/5/12 ms, report zero nonfinite events/overflow/stale feedback, and
  produce active leaf and exact-root telemetry. Production remains unchanged;
  the lab cannot promote automatically.

### Adaptive voxelized fractal-SDF lab gate

- The isolated `VOXEL_BUILD_FRACTAL_VOXEL_SDF_LAB` target materializes sampled
  parent/child leaf grids from the deterministic true fractal field. Inside the
  bounded cache footprint, final depth and normal must come from the sampled
  cell root; the analytic field is broad/reference only and edits remain
  authoritative.
- A persistent 2:1 controller targets 1.25 projected pixels, splits above 1.60
  pixels, and merges below 0.72 pixels. Full zoom-in/out, threshold jitter, and
  one-level-per-update tests must show bounded leaf size and no thrash.
- Shared half-open corners must remain continuous at seam ULPs. Parent/child
  transitions must preserve the common field and compare against direct-field
  oracle rays without hit/sky disagreement or error beyond one selected leaf.
- Orbit, rotation, grounded-player, and 720-frame horizon profiles must report
  zero nonfinite events and zero closed-shell misses. Current bounded-prototype
  budgets are 5/5/15 ms; passing does not make the cache planet-wide and does
  not promote either lab or production.

### Rejected cap-local adaptive-SDF reference gate

- This older gate is retained only as a regression reference. It must never be
  promoted or described as the true fractal-planet implementation because its
  fixed geodesic cap selects the hit before local refinement.

- Production f512 remains unchanged. Only
  `voxel_engine_adaptive_sdf_lab` compiles the adaptive local-SDF path, and it
  requires an explicit `--adaptive-sdf-*` command.
- Exact event-DDA/prism intersection selects the macro tile, layer, cap/side,
  material, and edit owner before any adaptive evaluation. A virtual hierarchy
  cannot return shading or a hit and no coarse square surface is rendered.
- The requested virtual-cell footprint is approximately 1.25 pixels. Split and
  reverse-merge are deterministic, continuous, and use the same world field;
  topology-ring requests are balanced to a 2:1 neighbor ratio.
- Nine detail levels, every intermediate octave transition, cube-face tie,
  pentagon/hexagon owner, and seed must be deterministic. Evaluated amplitude
  plus the unresolved tail may not exceed 0.20 of one radial layer.
- The local root is bracketed by that bound and uses exactly seven iterations.
  It must stay inside the accepted layer and central owner support, remain
  finite, and preserve grazing/silhouette and half-open side ownership.
- Water, side hits, and edited columns remain exact macro geometry. Edits mask
  the procedural base in rendering and collision and persist through residency
  changes.
- F6 collision evaluates the finest local field from the fresh compact profile;
  unknown data stays conservative solid and exact freshness arrives by frame 2.
- The 720-frame close/distant replay must exercise adaptive hits and multiple
  subdivision levels with zero nonfinite/boundary failures, dotted chains,
  recoverable/closed-shell misses, overflow, or stale requests.
- Acceptance budgets are <=4 ms stationary orbit, <=5 ms rotating/streaming,
  and <=12 ms grounded F6. Passing this bounded cap-local lab does not promote
  it or claim persistent volumetric/cave SDF bricks.

### Isolated f1024 two-stage exact-query gate

- Production remains f512; only `voxel_engine_scale_lab` may use f1024/R2,
  1024x512 atlas, compact 48-byte ray-query records, and macro min/max buffers.
- The broad phase cannot emit shading/depth or replace a leaf result. Exact
  event-DDA/prism intersection remains authoritative for render, silhouettes,
  edits, picking, and F6 collision.
- A 720-frame sampled differential replay must report zero first-hit/sky,
  distance, and material mismatches, zero non-finite bound events, zero invalid
  DDA terminations, and zero dotted chains.
- Face/edge/vertex ULP, seam/pentagon, residency/material, edit, and player
  two-ring behavior remain covered by the existing CPU and Vulkan gates.
- Preflight must account for the actual compact record and accelerator bytes;
  GPU metadata stays at or below 1.5 GiB, process peak at or below 8 GiB, and
  cold visible startup below 60 seconds without publishing partial cache data.
- Acceptance requires orbit <=4 ms, rotating/streaming <=5 ms, grounded F6
  <=12 ms, DDA p99 <512/max <2048, zero overflow/stale requests, and an exact
  local collision profile within two frames. A failure keeps the lab unlaunched
  and unpromoted even if all correctness gates pass.

### Isolated intrinsic Ellis-manifold gate

- Production f512 and the older finite-mouth portal comparison remain separate.
  Only `voxel_engine_intrinsic_portal_lab` may define
  `VOXEL_INTRINSIC_PORTAL_LAB=1`.
- Camera position is native signed proper depth plus a pole-safe unit angular
  direction. Camera velocity and its full tetrad are tangent vectors and must
  be parallel transported; no Euclidean pose handoff may occur at `l=0`.
- View rays start in that same tetrad and follow the same static Ellis metric.
  Only a ray reaching an asymptotic content exit may be converted to a world
  ray; exact event-DDA remains authoritative for visible terrain.
- CPU gates require radial C1 crossing, nonradial time reversal, agreement with
  a 2048-step oracle, finite pole/ULP cases, positive frame handedness, and no
  post-exit reset.
- The 180-frame off-axis GPU replay must cross `l=0`, keep adjacent-frame
  orientation change below 0.02 rad, report finite positive-handed state, and
  produce zero closed-shell/recoverable horizon misses.
- Metric/table startup retains the 55-second watchdog. Passing this bounded
  free-fly milestone does not claim F6 collision or participating-media
  integration inside the throat.

## Gates required before later milestones

These become blocking when their systems are implemented:

- Golden-image tests for fixed cameras and topology configurations.
- GPU picking followed by raise/lower state verification through a diagnostic readback.
- Cellular-automata determinism, mass conservation, settling, and cross-tile movement.
- Dirty-page persistence, dedicated transfer-queue synchronization, and out-of-memory behavior.
- LOD seam coverage and parent/child transition tests.
- Jolt collision-proxy agreement with solid voxel occupancy.
- GPU timestamp budgets and maximum traversal-cell/leaf visit budgets.

Performance gates will use tolerances and recorded hardware profiles; they will not use fragile universal frame-rate assertions.

For a bounded local measurement outside the pass/fail gate, run `voxel_engine.exe --benchmark` from the build directory. It renders 120 frames, exits automatically, and prints the final Vulkan timestamp measurement for the planet compute dispatch.

## Dense reference baseline

The validated pre-sparse reference is geodesic frequency 256 with 32 radial layers:

- 655,362 surface columns, including exactly 12 pentagons.
- 20,971,584 addressable radial voxels.
- 1,310,723 binary BVH nodes.
- 1.32 ms isolated planet compute on the development RTX 5070.
- Approximately 529 MiB resident and 809 MiB private process memory.
- Full milestone gate completed in 8.16 seconds, including a 7.33-second Vulkan smoke test.

Active sparse streaming retains the same 20,971,584-cell virtual capacity but allocates 8,192 physical pages / 262,144 detailed cells. The 120-frame rotating-camera reference sustains 213.5 FPS at 1.46 ms combined request/render compute, with zero overflow and zero stale acceptance. The stationary reference sustains 375 FPS and finishes with zero requests or uploads. The process uses approximately 394 MiB resident / 611 MiB private memory on the same hardware. These performance numbers are descriptive rather than universal pass/fail thresholds.

The geodesic traversal bake-off keeps exact convex prism intersection common to all backends. On the development RTX 5070, the original stationary measurements were 2.356 ms for the binary BVH, 13.156 ms for the experimental irregular wide-mask BVH, and 0.796 ms for geodesic neighbor DDA. With bounded deduplicated streaming, stationary DDA sustains 482.8 FPS at 0.693 ms and converges to zero requests; rotating DDA sustains 257.5 FPS at 0.796 ms with zero feedback overflow or stale acceptance. Geodesic DDA is the default; the other paths remain selectable regression and profiling references.

The promoted production scale is frequency 512 with 32 layers: 2,621,442 columns and 83,886,144 virtual voxels. The release RTX 5070 baseline measures 1.203 ms stationary and 1.437 ms rotating with the same fixed 16,384 request samples, zero overflow, and zero stale acceptance.
