# Global static spacetime lab

## Default model: same-exterior smooth handle atlas

The isolated global lab models one exterior voxel world with two localized
mouth charts joined by one continuous handle. Its window and in-app banner say
`SAME-EXTERIOR SMOOTH HANDLE ATLAS`. The native two-ended Ellis universe is
retained only behind the clearly labelled comparison checkbox.

This is a static ultrastatic spacetime,

`ds^2 = -dt^2 + g_ij dx^i dx^j`,

with an engineered, non-vacuum/exotic-matter spatial handle. It is not an
evolving Einstein-equation solver or a vacuum GR solution. An exact Ellis
handle supplies the throat; jet-matched smooth monotone curvature tails
attach it to the shared exterior.

The atlas has a shared exterior chart and one intrinsic handle chart. Mouth A
and Mouth B overlap the handle's positive and negative collar ends. The mouth
spheres are coordinate overlap surfaces only: they do not choose scene color,
terminate a ray, own a visibility family, or act as opaque geometry.

## Metric and overlap construction

The exterior optical metric is conformally Euclidean. Both mouth contributions
use a direction-independent radial profile and a C2 partition of unity. At
either overlap the other mouth contribution vanishes with three derivatives.
The complete intrinsic radial profile is exact Ellis. The exterior conformal
field is constructed from the Ellis endpoint value, first derivative, and
second derivative, then decays through a positive rational tail. Its
fourth-order far term changes no boundary jet. It
matches the pullback `R`, `R'`, and `R''` without squeezing a second lens into
either end of the short handle.

`HANDLE PROPER LENGTH` controls the actual intrinsic collar-to-collar metric
distance, not a coordinate/display scale. The accepted `Very short` default is
`2.00` throat diameters. With the default Planet-safe footprint, `a = 0.00875 R`,
so the throat diameter is `0.0175 R` and the complete collar-to-collar path is
`0.035 R`. The `0.32`-diameter option is retained and labelled
`Ultra-short (surface-like)` because it makes almost every ray that enters the
small coordinate mouth cross the waist, leaving almost no exterior-scattering
annulus. `Reference` restores the historical approximately `5.78`-diameter
path. A live change preserves normalized handle depth and rebuilds the same
CPU/GPU radial-profile parameters.

`LENS FOOTPRINT` changes physical scale rather than applying an image-space
strength or influence mask. The presets are:

| Preset | Mouth radius | Ellis throat radius `a` | Exterior tail scale |
| --- | ---: | ---: | ---: |
| Planet-safe (default) | 0.025 | 0.00875 | 0.009 |
| Compact | 0.12 | 0.042 | 0.020 |
| Moderate | 0.18 | 0.063 | 0.025 |
| Dramatic (former default) | 0.34 | 0.119 | 0.85 |

The same dimensionless Ellis core, single waist, and handle-length ratio are
retained. Planet-safe makes the physical throat and its smooth noncompact optical
tail smaller relative to the radius-one planet. It does not terminate rays,
mask pixels, or clamp mapped directions. The UI also reports the predicted
critical angular diameter for both mouth centers from the live observer pose.

The handle uses native `R(l)=sqrt(l^2+a^2)`, connection, camera transport, and
local optical Jacobian over 100% of its proper length. This resolves the
rejected “two wormholes in one hole” profile: expanding a `0.119` throat to the
former `0.423` physical collar radius inside only `0.038` proper units per side
requires an overshooting derivative and a second curvature pulse. The new
construction maps the selected coordinate mouth through a conformal factor and
exact Ellis jets, then distributes the scale match through the smooth exterior
tail. There is one areal-radius minimum at `l=0`, with no collar-local
optical waist or finite ray/content ownership surface.

This produces a positive-definite spatial metric, inverse consistency, and a
continuous connection across coordinate overlaps. It is a candid engineered
handle, not the earlier cut-and-remap portal sphere. Curvature decays smoothly
and is not compactly switched off at a finite influence radius.

## Camera and rays

The free-fly camera stores one manifold state: exterior position or intrinsic
`(u,n)` handle position, a local orthonormal tetrad, proper velocity, affine
distance, and chart ownership. Movement is proper distance per second. RK event
splitting consumes the residual frame step at a crossing. The frame is parallel
transported and corrected only for numerical drift; it is never rebuilt from a
Euclidean world axis. Both mouth centers are traversable, including End-B
orientation, with positive handedness and no teleport, cooldown, or crossfade.

Two independent whole-view flip defects were corrected. The orthonormal-frame
parallel-transport equation formerly applied radial basis expansion twice; it
did not preserve frame norm and made the view forward diverge from an identical
geodesic tangent. End B also reversed only the radial axis, giving its collar
map determinant `-1`. The transport now uses the physical orthonormal warped-
product connection, and B uses the antipodal sphere differential so radial and
both angular axes form an orientation-preserving map. Numerical cleanup is a
basis-free SO(3) projection against the transported frame, never a world-axis
rebuild or a one-axis sign repair.

Mouse yaw, pitch, and roll rotate that same authoritative tetrad in its local
orthonormal frame in both charts. In the handle, yaw is about transported local
up and pitch is about transported local right; the observer position is not
changed. The exterior `forward/up` values are only mirrors while inside. This is
important because rotating the mirror and then republishing the untouched
handle tetrad made F7 mouse look appear disabled in the original atlas build.
Movement and the same-frame GPU center ray now consume the rotated handle
tetrad directly.

Handle camera packets use a basis-free embedded tangent representation. The
older packet stored `(radial,e1,e2)` components in a pole-safe angular basis
whose least-aligned Cartesian reference axis changes on well-conditioned chart
boundaries. Parallel transport remained continuous, but using those changing
component triples directly as the GPU screen frame produced whole-view flips
inside the handle. The screen camera is now a canonical local tetrad; each
primary/AA ray is transformed by the embedded transported forward/up vectors
before the temporary angular chart is used by the intrinsic solver. Angular
basis ownership can therefore change without rotating the image.

GPU primary rays and adaptive-AA subrays use the same exterior/handle state and
half-open overlap ownership as the CPU body path. A chart transition remaps
coordinates and tangent components only. It does not reset affine distance,
ray cone, radiance, content, or ray family.

The main shader formerly tested `camera.w > 0.5`, inherited from the original
intrinsic lab where that lane was an enable bit. The same-exterior packet uses
it for signed handle depth. Near and beyond the throat, that obsolete test
skipped manifold tracing and launched DDA at the planet centre, causing
full-screen closed-shell failures and visible planet interiors. The global path
is now selected by the explicit atlas discriminator. The Vulkan crossing replay
reports zero closed-shell and zero recoverable misses.

## Content and participating media

There is one exterior terrain, material, star, atmosphere, and cloud field. No
`sign(u)` branch swaps duplicated universes. The source-side exact terrain
segment is tested before a mouth event, the vacuum handle is traversed, and the
destination-side exact query uses the transported exit state. Source and
destination intervals coexist; recording one no longer suppresses the other.

Atmosphere and clouds are integrated in physical order. An empty conservative
planet/media shell is a broad-phase interval only and cannot select the
renderer. Destination overlap also handles a mouth that emerges directly
inside that query volume. Default mouth standoff is derived from the actual
generated f512 terrain bound, so no exit is seeded inside opaque relief. Stars
use the polar-safe direction-native environment and transported ray footprint.

## Optical boundary and AA

The old fixed-radius color boundary was a renderer policy defect. It is gone.
A real handle still has a ray-family separatrix: small-impact rays traverse the
handle while larger-impact rays remain in the exterior. A critical curve is
accepted only where the geodesic Jacobian approaches zero or changes parity; it
must not coincide with a content shell, solver class, or chart threshold.

The one measure-zero critical geodesic has deterministic half-open ownership.
CPU and GPU use the same representable-scale impact band,
`max(2e-5 a, 16 epsilon_float R_entry)`; this replaces the former roughly
two-pixel snap. It is not a screen-space tolerance. Finite rays immediately
outside that band retain their own impact and are integrated normally. Mouth
chart ownership is independently tested at exact tangency and at one through
four float ULPs inward/outward.

Normal rendering uses one sample per pixel. A deterministic rotated four-sample
pattern is queued only for Jacobian distortion, parity, silhouette, or coverage
transitions. Subrays retain independent exact terrain and ordered media results;
linear-HDR radiance is coverage averaged before tone mapping. There is no
stochastic jitter, temporal history, screen warp, or synthetic depth blend.

## Controls

Executable: `build-global-metric-lab/voxel_engine_global_metric_lab.exe`

- `F7`: portal free-fly
- mouse: look
- `WASD`: local forward/strafe
- `Space` / `Ctrl`: local vertical
- `Q` / `E`: roll
- `Shift`: sprint
- `Home`: reset outside Mouth A
- `End`: reset outside Mouth B

The UI exposes same-exterior/native-reference selection; Planet-safe, Compact,
Moderate, and Dramatic physical lens-footprint presets; the `Very short`,
`Ultra-short (surface-like)`, and `Reference` handle-length presets plus a
logarithmic custom slider; and the
fixed `100% native Ellis / one waist` optical-profile status. Switching
to native Ellis while inside the handle preserves signed
proper depth, angular point, transported camera tetrad, and FOV, making the A/B
comparison meaningful instead of resetting the viewpoint. Integration quality,
End-B rotation, proper speed, exterior-tail scale, AA thresholds, chart/depth,
handedness, and explicit path/Jacobian debug views remain available.

## Hidden deterministic capture

The global-lab executable can render a bounded Vulkan run into a PNG while the
SDL window remains hidden. Automated validation must use this path; it must not
show, raise, focus, or mouse-capture the window.

```text
voxel_engine_global_metric_lab.exe \
  --headless-capture artifacts/short-handle-crossing/close-a.png \
  --capture-frames 8 \
  --capture-preset balanced \
  --capture-camera close-a \
  --capture-debug normal
```

`--capture-frames` accepts `1..600`. Capture presets are `balanced` (the
accepted Planet-safe/2.00 configuration), `no-handle`, `rejected-broad`, and
`rejected-hard-aperture`. Camera presets are `reset`, `close-a`, `close-b`, and
`oblique-a`. Debug views are `normal`, `bending`, `aa-samples`, and `jacobian`.
`--capture-tail-scale <positive-float>` overrides only the exterior-tail scale.
`--global-no-handle-reference` is also available for an identical renderer pose
without handle bending. Invalid options fail before the application is created;
after rendering, capture aborts if SDL reports that the window ever became
visible. The `no-handle` capture preset remains authoritative even if a generic
`--capture-debug normal` is also supplied.

## Validation

The CPU global-field gate covers positive metric/inverse consistency; exterior
and handle `R/R'/R''` pullback agreement; overlap limits including ULP offsets;
independent high-precision ODE comparison; Hamiltonian/null preservation; time
reversal; finite critical integration; proper camera speed; transported frame
handedness; centered/off-axis A-to-B and B-to-A body/ray equality; End-B
rotations; poles; exact tangent ownership with one-through-four-ULP inward and
outward perturbations; dense subframe crossing; and
non-owning content-shell/star/AA invariants.

The controller gate also applies nonzero yaw/pitch/roll at positive handle
depth, `u=0`, negative depth, both pole-like angular orientations, and both
mouth directions. It requires zero-input frame preservation, local-forward
movement after look, positive handedness through repeated look plus traversal,
and an immediate match between the CPU tetrad and packed GPU center ray. A
dedicated reference-axis sweep crosses the former least-aligned-axis boundary,
proves that the legacy local packet jumps, and requires the embedded packet and
off-axis reconstructed rays to remain continuous.

The short-handle gate integrates the surface-like `0.32`, intermediate `0.75`,
accepted `2.00`, and reference length presets, checks monotone
proper-length parameters, exact native Ellis `R/R'/R''` across the complete
handle, exactly one areal-radius minimum, monotonically decaying sectional
curvature away from the waist, positive finite profiles, and both C2 exterior
attachment jets, then stops immediately before and advances through the exact
collar event to reject step tunneling. A focused mouth-disk differential also
requires the accepted 2.00 path to restore a substantial source-side scattering
annulus relative to the 0.32 surface-like comparison. Manual-style A-to-B and B-to-A
crossings run at 20,
60, and 240 Hz while yaw, pitch, and roll continue. They require one event,
positive handedness, proper-speed motion, orientation-preserving A/B transition
determinants, and same-frame CPU/GPU chart packets. A genuine narrow optical
separatrix is accepted only as a localized ray/Jacobian family change; center
and majority screen rays must remain continuous, so it cannot validate a
whole-camera flip.

The lens-footprint gate sweeps both mouths at three observer distances with a
41 by 23 ray bundle and a radius-one planet placed behind/beside the mouths. It
records the screen fraction with more than five degrees of mapped-direction
deviation, the throat-crossing/critical fraction, the analytic critical-cone
area, and the fraction of direct primary-planet rays replaced by a crossing
secondary image. Compact and Dramatic use the same exact solver and differ only
in physical scale and smooth-tail parameters.

The Vulkan replay crosses both chart ends with media and AA. It requires finite
paths, zero affine exhaustion, zero owner mismatch, and zero closed-shell or
recoverable misses. Separate media-on/off sweeps exercise both mouths,
foreground and destination terrain, atmosphere, clouds, stars, and planet-near
views.

The current accepted checkpoint capture is
`artifacts/short-handle-crossing/headless-close-length200-mouth025-tail009.png`.
It uses the 2.00-diameter handle, `0.025 R` mouth, `0.00875 R` Ellis throat, and
`0.009 R` tail. The completed hidden capture set is:

- `headless-close-a-length200-mouth025-tail009.png`
- `headless-close-b-length200-mouth025-tail009.png`
- `headless-oblique-a-length200-mouth025-tail009.png`
- `headless-close-a-no-handle.png`
- `headless-close-a-rejected-broad.png`

The no-handle/corrected close-A pair uses an identical camera pose. The oblique
preset offsets the observer tangentially and looks toward the planet; it is not
the former centered camera with only a larger standoff. Still inspection shows
the planet retained in all accepted poses and confines the black critical bands
and repeated planet image to the localized lens structure. Whether those bands
read correctly during motion remains a manual acceptance question.
Automated tests and a still capture do not establish user motion acceptance;
the lab remains isolated until the user evaluates an interactive crossing and
its genuine critical curves.

## Performance and limitations

The objective footprint and planet-subject sweeps now validate the Planet-safe
configuration against the former Dramatic field and the rejected broad-tail
configuration. Exact measurements are recorded by each focused run rather than
copied from the obsolete 0.32/Compact checkpoint.

Performance acceptance is likewise based on the current command output. The
GPU gates report frame time, AA coverage/subray counts, ray ownership and affine
exhaustion, closed-shell/recoverable misses, and media coverage. Passing those
automated gates does not promote this isolated lab to f512; the current
Planet-safe/2.00 configuration still requires explicit manual-motion acceptance.

The 2026-08-22 hidden Vulkan gates on the development RTX 5070 measured:

| Gate | Frames | FPS | Planet compute | Total compute | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| ordinary benchmark | 120 | 113.99 | 8.23798 ms | 8.25283 ms | pass |
| crossing traversal | 160 | 38.8738 | 24.3035 ms | 24.3158 ms | pass |
| media-on sweep | 180 | 16.6494 | 34.0930 ms | 34.1091 ms | pass |
| no-media sweep | 180 | 16.6641 | 33.3034 ms | 33.3179 ms | pass |

The ordinary benchmark selected `0.0322222%` of pixels for adaptive AA
(`13,804` subrays). Traversal reported maximum adjacent-frame turning of
`0.336194 rad`, zero unresolved rays, affine exhaustions, owner mismatches,
closed-shell misses, and recoverable misses. Each media sweep mapped all
`16,110,000` classified rays finitely; the media-on run recorded
`7,161,205` atmosphere and `6,064,541` cloud intervals, while the no-media run
recorded zero of both as required. These measurements still miss the 18 ms
near-mouth target, so performance is not accepted.

At extreme close range, or when the observer deliberately aligns the planet
with the true Ellis critical cone, physical secondary images can still occupy a
large part of the view. Planet-safe makes that configuration much less common; it
does not suppress the genuine caustic with a nonphysical screen mask.

The next optimization target is coherent reuse of observer invariants and
fewer repeated exterior/handle solves without restoring hard endpoints or
approximation surfaces. Global F6 capsule collision and dynamic spacetime
evolution remain out of scope.

The architecture follows GPU geodesic practices from
[GRay](https://arxiv.org/abs/1303.5057) and
[Odyssey](https://arxiv.org/abs/1601.02063), with analytic Ellis geometry as
the independent throat reference. The same-exterior topology and exterior attachment are
engineered for this lab and documented as such.
