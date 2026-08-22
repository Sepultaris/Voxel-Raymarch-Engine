# Global static spacetime lab

## Accepted interactive path: native Ellis atlas

The previous shared-exterior experiment was visually rejected. Despite its
smooth conformal metric, it still applied topology through an explicit
Euclidean mouth sphere. Rays tangent to that sphere selected a different
content path, producing the huge perfectly circular star/planet boundary in
the user capture. That contour was **not** the Ellis `b=a` critical curve.

The interactive lab now uses the native signed-proper-depth Ellis atlas
everywhere:

`ds^2 = dl^2 + (l^2 + a^2) dOmega^2`.

The camera and each primary ray remain `(l,n,tangent)` states. They pass through
the regular throat at `l=0` without a Euclidean aperture event, `lastMouth`
visibility owner, or whole-scene A/B swap. Curvature decays continuously toward
both asymptotically flat ends. The same path feeds terrain, media, and the
direction-native star environment.

This is an exact static Ellis spatial metric, not an evolving Einstein solver.
The old engineered shared-exterior sphere-remap implementation remains only as
a historical automated differential and is explicitly labelled rejected in
the UI. It is not selectable as the normal interactive renderer.

## Camera, ray, and topology state

The two UI endpoints are now the `+l` and `-l` asymptotic content attachments
of one Ellis throat. They are not two localized spherical apertures visible in
one Euclidean room. `Home` starts on the positive end and `End` on the negative
end. Movement is proper distance per second in the local orthonormal tetrad;
the camera frame and velocity are parallel transported by the same integrator
used to define ray transport.

This distinction is necessary and candid: two localized mouths in a single
otherwise-Euclidean exterior require a genuine smooth three-manifold handle
atlas. The rejected cut-and-remap sphere did not provide one, and no amount of
AA can make its binary aperture ownership C0. A future same-room two-mouth
implementation must construct that topology directly rather than re-enable the
legacy sphere.

## Content ordering

Terrain, voxel materials, atmosphere, clouds, and stars remain content fields.
The continuous Ellis path advances to a possible planet-content interval,
where exact geodesic prism/event-DDA remains the terrain narrow phase. Empty
conservative shells are workload hints only and never choose a path family.

Atmosphere/cloud radiance is composed in physical order on the source and
destination content-bearing portions of the path. The inter-mouth throat is
vacuum, so no medium is sampled across an imaginary Euclidean chord. Stars use
the transported direction-native ray and one continuous footprint policy.

## Isolation and controls

This remains an isolated lab target and does not change or promote f512:

```
build-global-metric-lab/voxel_engine_global_metric_lab.exe
```

Controls are `F7`, mouse look, `WASD`, `Space/Ctrl`, `Q/E`, and `Shift`.
`Home` resets on the `+l` end and `End` on the `-l` end. UI reports signed
proper depth, frame determinant, exact metric status, cache status, and
non-finite ray telemetry. The window and banner say `NATIVE ELLIS ATLAS` so a
legacy shared-exterior build cannot be mistaken for the accepted experiment.

## Research basis and scope

The GPU geodesic architecture is informed by
[GRay](https://arxiv.org/abs/1303.5057),
[Odyssey](https://arxiv.org/abs/1601.02063), and the metric/derivative/
Christoffel separation in the
[spacetime-agnostic GR tracer](https://arxiv.org/abs/2510.15049). The analytic
Ellis model remains the reference used to validate throat behavior.

This lab does not solve the Einstein field equations, evolve stress-energy,
back-react terrain, or simulate gravitational radiation. F6 capsule collision
and gravity are not yet global-manifold physics. The free-fly controller is the
supported traversal path. Terrain bodies do not span the throat itself.
Participating media are planet-attached content fields; within a content shell
the adaptive path is handed to the exact local query as a bounded chord.
Sampling arbitrary media density at every geodesic substep is a future
extension.

## Regression gates

`intrinsic_ellis_manifold_test` checks radial and off-axis C1 throat crossing,
time reversal, parallel transport, positive handedness, proper speed, pole-safe
angular transport, and the independent signed-end exit oracle. The global
field test additionally verifies that the interactive GPU packet contains the
native signed-l state and cannot silently fall back to a mouth-owner pose. The
rejected shared-exterior path retains historical differential coverage, but is
not the interactive acceptance model.

The native Vulkan traversal performs both throat directions in one continuous
camera run. It requires finite paths, front-facing terrain coverage, ordered
media, no interior-hit fallback, and no frame with lost terrain or required
media coverage. Legacy media-on/off replays remain useful for ensuring earlier
terrain ordering fixes do not regress, but they do not establish native-atlas
continuity. Existing f512 gates remain a separate isolation check.

### Camera crossing continuity

The user capture disproved the prior owner-persistence patch. That patch made a
discrete remap less unstable but retained the wrong primitive: a ray either hit
the Euclidean mouth sphere and entered Ellis transport or did not. The finite
sphere was therefore a binary radiance-family edge, and the camera still
changed exterior embeddings at a discrete crossing.

The accepted path removes that state. The GPU packet carries signed `l`, the
angular unit vector, and the local orthonormal forward/up tetrad. `l=0` is an
ordinary integration point of the Ellis metric; there is no endpoint center,
mouth owner, event radius, or one-frame CPU/GPU chart handoff to pack. The
deterministic Vulkan replay crosses `+l -> -l`, reverses, and crosses
`-l -> +l`. Both measured throat events have zero adjacent-frame forward-angle
jump, zero unresolved ray, zero affine-budget exhaustion, and zero closed-shell
or recoverable miss. The broader flight still shows legitimate continuous
parallel transport in strong curvature and is reported separately from the
crossing event.

Visual evidence is stored in
`artifacts/global-crossing-discontinuity/native-ellis-crossing.mp4` and its
contact sheet. The old giant fixed-radius circle is absent. Strong stretching
and repeated star images can remain near the genuine Ellis separatrix; those
features follow impact parameter/Jacobian, not a fixed Euclidean sphere.

The newer bidirectional contact sheet makes the remaining circle explicit. At
the replay start `l=0.355965`, `a=0.34`, so the analytic critical cone is
`alpha=asin(a/sqrt(l^2+a^2))=0.776949 rad` (44.52 degrees). Independent ODE
samples on its two sides give `b/a=0.995721` (crossing) and `1.00428`
(scattering). The circle therefore is the exact Ellis `b=a` separatrix, not the
removed mouth sphere. Because the two asymptotic content attachments currently
carry visibly different radiance, that physical ray-family boundary remains a
large contrast edge. Four-sample coverage stabilizes its finite pixel edge but
cannot make the two boundary conditions C0 without an unphysical radiance
blend. This visual model is consequently **not user-accepted** yet. Removing
the contour requires a product/model choice: a materially smaller physical
throat, matching continuous content boundary conditions on both ends, or the
more substantial genuine same-exterior handle atlas described above.

The direction-native star environment is evaluated from the final selected
content-query origin and direction, including a mapped/lensed exit. Planet
horizon exclusion uses the closest point on the **forward ray half** only. If
the closest approach lies behind that origin, visibility is exactly one; this
prevents the former antipodal duplicate planet mask while retaining the normal
forward silhouette guard. Paired CPU toward/away checks span observer distance
and rotated mapped exits, while the media-on/off GPU replays exercise both
mouths and camera approach without changing the background ownership rule.

### Geodesic-Jacobian spatial AA

The lab now keeps the normal renderer at exactly one sample per pixel and
appends only conservative high-curvature, mouth-critical, terrain-silhouette,
or disocclusion candidates to a compact GPU queue. A separate coherent pass
traces the deterministic rotated four-sample pattern
`(-.375,-.125), (.125,-.375), (.375,.125), (-.125,.375)`. Four subrays run as
independent invocations, then a small resolve pass reconstructs the local
finite-difference geodesic Jacobian. It records determinant sign/parity,
magnification, anisotropic distortion, exit/hit family, and depth coverage.

The four samples retain independent exact DDA hits and ordered source/dest
atmosphere/cloud integration. Their linear-HDR radiance is averaged as pixel
coverage and tone mapped once. Depths and chart states are never averaged into
a synthetic ray. A parity, exit family, hit/miss, silhouette, or material-depth
transition therefore receives deterministic coverage sampling instead of a
cross-family blur. Smooth candidates rejected by the Jacobian leave the
original 1x pixel untouched. There is no frame hash, time jitter, temporal
history, or screen-space crossfade.

UI controls expose enable, distortion threshold, and magnification threshold.
The output selector adds sample-count, signed determinant/parity,
magnification, and family-rejection/coverage views. The queue and resolve use
the existing lab diagnostic allocation and indirect dispatch, so an empty AA
queue launches zero subray workgroups. `--no-global-spatial-aa` provides the
1x differential/performance reference; it does not select another geometry or
solver.

The former Euler/midpoint critical solver is replaced by an embedded
midpoint/RK4 error-controlled step with exact fractional throat/content events.
The small-step high-precision RK integrator remains an independent CPU oracle.
Both-mouth sweeps compare owner, exit origin, and direction; the certified
separatrix neighborhood is coverage sampled because its two ray families are
physically discontinuous. An unfinished affine path is still a hard failure,
never valid sky.

The throat separatrix remains a real critical curve: crossing rays see the
linked end while non-crossing rays stay on the observer end. Repeated images
and sharp magnification near that curve are physical behavior of the selected
topology, not the removed outer solver/standoff circle. Spatial AA stabilizes
that thin transition; it does not erase or blur the topology. Temporal
accumulation remains explicitly deferred.

### Spatial-AA validation and measured cost

The acceptance replay uses both mouths, media enabled and disabled, the
recorded high-magnification crossing, and subpixel camera offsets. The CPU
differential covers 257 optimized-versus-high-precision rays per mouth plus a
1025-ray near-critical sweep. Outside the explicitly certified separatrix
neighborhood, exit-origin and exit-direction error remain below `0.012` and
ray-family ownership agrees. The GPU replay reports zero unresolved affine
paths, zero whole-mouth coverage loss, and zero streaming overflow/stale
requests. The f512 production isolation suite remains 25/25.

Representative 1600x900 measurements on the development GPU are:

| Replay | Spatial AA | Planet compute | Selected pixels | Result |
| --- | ---: | ---: | ---: | --- |
| Native Ellis ordinary | off | 12.28 ms | 0% | correctness reference |
| Native Ellis ordinary | on | 32.69 ms | 1.001% | performance-unaccepted |
| Native Ellis bidirectional crossing | on | 48.32 ms | 0.411% | continuity accepted, performance-unaccepted |

The native exact path averages about 24.3 central steps in the ordinary run.
Although AA selects only about one percent of pixels, its current append/
subray/resolve implementation raises ordinary cost substantially. The requested
`12 ms` ordinary and `18 ms` near-throat targets are therefore **not accepted**
with AA enabled. Correctness was deliberately not traded back for the old
sphere policy. The standalone lab remains experimental and is not promoted to
f512.

The inspected before/after evidence is stored under
`artifacts/global-spatial-aa/`: the original stepped-banding screenshot, a
crossing recording, and a 4x4 contact sheet from the corrected renderer.

### Content-query shell is not a path endpoint

The planet-content outer radius (`planetOuterScale + 0.30`, approximately
`1.30R`) is now only a conservative broad-phase/query interval. It does not
terminate a geodesic, select a ray family, restart media, or replace the final
star direction. On crossing that shell the shader records a bounded local
terrain/media candidate interval and continues the same error-controlled
geodesic to its full affine endpoint. Exact event-DDA can select a foreground
hit inside the recorded interval; otherwise the fully transported endpoint,
direction, parity, Jacobian, and angular footprint remain authoritative.

This removes the planet-centered circular contour previously produced when
the first `1.30R` entry was incorrectly returned as the final mapped ray.
Debug output `Content query shell (cyan = crossed; never owns path)` shows the
broad-phase event explicitly without changing normal output. The corresponding
counter is also reported by the bounded GPU replays.

The CPU regression recreates the reported camera geometry and sweeps the
shell tangent at `+-ULP` and subpixel offsets for both mouths. It compares the
optimized full-affine result against an independent 8192-step integrator and
requires practical C1 endpoint/frame continuity. Media-on and media-off GPU
replays each traced 16,110,000 samples with 4,086,173 shell crossings, zero
affine-budget exhaustion, zero recovered/missed terrain hits, and continuous
coverage. Sparse Jacobian AA was then rerun on the corrected path.

The earlier `9.21/10.66 ms` measurements below this historical shell milestone
used the now-rejected shared-exterior policy and are not representative of the
native Ellis interactive path. Current native measurements are recorded in the
table above; the lab remains performance-unaccepted and isolated from f512.

The inspected corrected sweep and contact sheet are under
`artifacts/global-content-shell/`. A thin inner Einstein/terrain-limb ring can
still occur where physical ray families or an actual foreground silhouette
meet. Unlike the removed contour, it is not fixed to `1.30R`, does not select
a query policy, and moves according to the transported geodesic/Jacobian.
