# Static Ellis GR wormhole portal laboratory

This is an isolated frequency-512 laboratory target. Configure it with
`-DVOXEL_BUILD_PORTAL_LAB=ON`; it builds `voxel_engine_portal_lab` and the
separate `shaders-portal-lab` shader set. Production compiles with
`VOXEL_PORTAL_LAB=0`, so none of this is promoted to f512 production.

## Fixed spacetime

The active spacetime is the ultrastatic Ellis wormhole

```text
ds² = -dt² + dl² + (l² + a²)(dθ² + sin²θ dφ²)
```

with fixed throat ratio `a/Rmouth = 0.35`. It is an explicit static analytic GR
metric, not an artistic inverse-square force and not an evolving
Einstein-equation solver. Spherical symmetry places each geodesic in an
equatorial plane. The precompute integrates

```text
l'' = l φ'²
φ'' = -2l/(l²+a²) l'φ'
```

and the Ellis orthonormal-frame connection. A compact quintic matching collar
joins the finite chart to ordinary game space with zero first derivative at
the cutoff; the tabulated interior is the stated Ellis metric.

## Ray and physical traversal

Opaque exact terrain is ordered before either mouth. A ray that reaches a mouth
indexes the precomputed exit map by conserved impact parameter. The table
returns same/cross-chart outcome, azimuth, affine length, and parallel-frame
transport. Intervals whose certified interpolation error exceeds `5e-4`, or
which straddle the critical `b=a` orbit, run bounded local RK4 integration of
the same equations. A critical orbit exceeding that local budget becomes a
conservative same-chart scatter, never a background/coverage hole.

The outgoing ray then enters the unchanged exact geodesic voxel DDA,
atmosphere, cloud, lighting, and star paths in the selected chart. This is
three-dimensional ray transport, not a post-process UV warp.

Camera/player state carries a continuous proper-depth coordinate through the
same tabulated radial metric and linked SO(3) frame connection. The source-side
controller basis is not progressively link-rotated: the GR ray already carries
that transport. At the throat the final oracle-validated frame is installed
once and retained. While the observer remains inside the destination half, the
GPU samples a precomputed signed-proper-depth by local-view-angle exit map and
continues every pinhole ray to the same finite chart boundary used one frame
earlier, rather than dropping directly into an unrelated Euclidean camera
origin. The radial `b=0` row is analytic; off-axis rows are integrated from the
same Ellis equations and bilinearly interpolated. This removes both components
of the recorded snap: a double link rotation and a disappearing half-chart ray
continuation.
The active Euclidean chart changes at the throat, but the rendered origin and
determinant-positive tangent frame do not jump. Destination terrain collision
is revalidated for the F6 capsule.

The dedicated F7 free-fly camera is separate from orbit and the F6 surface
player. It has a full right-handed position/forward/up/velocity state, with no
terrain collision or radial gravity, and accepts the manifold result without
projecting it onto a planet tangent plane. Its exact pose is supplied to the GPU
through the portal buffer, so it cannot bypass the shared metric.

The two endpoints are transformed charts of the same editable voxel planet,
not separately stored universes. This implementation intentionally models one
fixed analytic spacetime; it does not evolve mass-energy or solve the Einstein
field equations at runtime.

## Precompute and cache

The GPU table contains 128 metric/Christoffel samples, 256 ordered boundary
exit/frame samples, and a 96x96 signed-depth/view-angle observer exit map,
about 150 KiB total. A double-precision CPU reference uses
16,384 integration steps per radius for table construction and 32,768 for test
oracles. Midpoint interpolation is validated; failing intervals are marked for
runtime refinement.

Startup visibly reports metric/Christoffel, geodesic/frame, and GPU-upload
stages in the window title. A 55-second internal watchdog enforces the external
60-second cap. The cache is `cache/portal_gr_ellis_v4.bin`, validated by magic,
schema version, byte count, and FNV checksum. Regeneration writes `.partial`
first; invalid/partial files never masquerade as valid data.

## Controls and diagnostics

- `F6`: orbit/surface-player selection;
- `F7`: enter/leave portal free-fly;
- `Home`: reset free-fly outside portal A, aimed at its center;
- mouse: yaw/pitch; `WASD`: local flight; `Space/Ctrl`: local up/down;
- `Q/E`: roll; `Shift`: sprint; mouse wheel/UI: flight speed;
- `Escape`: release relative mouse input.

The UI exposes endpoint transforms, influence scale, fixed throat ratio,
refinement quality, free-fly speed/sensitivity, chart/proper-depth state,
the actual observer forward/up vectors and active GR half,
precompute readiness, cold/warm time, cache version/size, certified error, and
previous-frame lookup/refinement telemetry. Debug outputs distinguish affected
rays, chart crossings, refinement work, and non-finite rejection.

## Rejected predecessor

The earlier artist-curvature RK4 path produced concentric termination bands and
used a discrete throat mapping. It remains only as historical CPU test
scaffolding; the active shader calls the precomputed GR path. No artistic lens
strength or mass-bias control affects active rendering.

## Research basis

- Bugaev et al., [*Gravitational Lensing and Wormhole Shadows* (2021)](https://link.springer.com/article/10.1134/S1063772921120027).
- Tsukamoto and Harada, [*Light curves of light rays passing through a wormhole* (2017)](https://journals.aps.org/prd/abstract/10.1103/PhysRevD.95.024030).
- TomFractals' [*Project Manifold*](https://www.youtube.com/watch?v=12o-PD5WGJw) is a visual reference only.

All code/assets are original to this project.

## Gates

`spherical_wormhole_portal_invariants` covers cold/warm cache identity,
corrupt/partial rejection, positive metric coefficients, high-precision oracle
agreement for radial/ordinary/grazing/critical rays, interpolation error bounds,
the recorded F7/Home/W full-crossing sequence (bounded adjacent view angle and
origin motion), persistent post-exit orientation against the 32,768-step radial
frame oracle,
a dense finite no-gap coverage grid, C1 influence entry, intrinsic throat-depth
continuity, positive handedness, no adjacent forward/up flip, ray/free-fly
policy equivalence, six-DOF input, no-collision flight, and bounded stress.

The Vulkan smoke, portal benchmark, portal artifact replay, production artifact
replay, and complete f512 suite must pass before interactive launch.

Useful standalone commands:

```text
voxel_engine_portal_lab.exe --smoke-test
voxel_engine_portal_lab.exe --benchmark
voxel_engine_portal_lab.exe --rotation-benchmark
voxel_engine_portal_lab.exe --surface-benchmark
voxel_engine_portal_lab.exe --artifact-regression
voxel_engine_portal_lab.exe
```

The lab remains experimental until visual traversal is accepted by the user.
