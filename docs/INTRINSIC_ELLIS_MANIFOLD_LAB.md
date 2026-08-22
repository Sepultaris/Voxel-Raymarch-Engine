# Intrinsic Ellis manifold lab

This is a new isolated renderer target. It does not replace f512 production or
the earlier finite-mouth portal experiment.

## What is intrinsic

The observer lives in the static Ellis spatial manifold

```
ds^2 = dl^2 + (l^2 + a^2) dOmega^2
```

with fixed `a = 0.35`. Position is stored as signed proper depth `l` and a unit
angular direction `n`, not as a Euclidean XYZ point. The camera's right, up,
forward, and velocity are orthonormal tangent vectors. Movement integrates the
metric geodesic equations and parallel-transports that frame. Crossing `l=0`
is therefore an ordinary integration step; there is no portal-sphere event,
teleport, screen warp, or post-crossing basis reconstruction.

Angular position uses a coordinate-free unit vector plus a least-aligned-axis
tangent atlas. It remains finite at the conventional theta/phi poles.

Each view ray starts in the same intrinsic observer tetrad. Within the central
Ellis domain it follows a spatial geodesic to one of the two content exits. The cached
signed-depth/view-angle exit map handles regular rays; a bounded RK4 refinement
handles grazing or invalid interpolation cells. Only at `|l| = sqrt(1-a^2)` is
the outgoing ray converted to the selected end's Euclidean content attachment.
The existing exact f512 event-DDA, atmosphere, clouds, and star renderer then
query that content. Outside an exit, free flight continues without a proper-
depth clamp in an ordinary shared content chart. Both endpoint A and endpoint
B are visible access mouths in that chart. A enters the `+l` Ellis end and B
enters the `-l` end; rays and the free-fly camera can enter either one and
travel through the same intrinsic throat. Each mouth is a coordinate
attachment, not a rendered shell or fake surface. A hierarchy/table result never supplies final terrain
color, material, depth, or normal.

This follows the fixed analytic Ellis/Morris-Thorne model discussed by
[Tsukamoto and Harada (2017)](https://journals.aps.org/prd/abstract/10.1103/PhysRevD.95.024030)
and is consistent with the static lensing scope in
[Bugaev et al. (2021)](https://link.springer.com/article/10.1134/S1063772921120027).
It is a static analytic spacetime, not an evolving Einstein-equation solver.

Mathematically, an isolated Ellis wormhole has two asymptotically separate
ends. Showing both mouths in one playable universe requires one additional
topological choice: this lab glues both asymptotic attachments into the same
external content chart, producing a handle. The UI labels this `shared-chart
handle`; it is why both access mouths can be viewed together without replacing
the intrinsic interior with a screen-space portal effect.

## Isolation and launch

Configure with `VOXEL_BUILD_INTRINSIC_PORTAL_LAB=ON`. The executable is:

```
build-intrinsic-portal-lab/voxel_engine_intrinsic_portal_lab.exe
```

Its title and permanent overlay say `INTRINSIC ELLIS MANIFOLD Lab`. The old
comparison executable remains `build-portal-lab/voxel_engine_portal_lab.exe`;
f512 remains `build-f512/voxel_engine.exe`.

Controls in intrinsic mode:

- `F7`: toggle intrinsic free-fly / orbit
- mouse: yaw and pitch in the transported observer tetrad
- `WASD`: forward/back and lateral geodesic motion
- `Space` / `Ctrl`: third intrinsic tangent axis
- `Q` / `E`: roll
- `Shift`: sprint
- mouse wheel: free-fly speed
- `Home`: reset on the positive end, aimed through the throat
- `Escape`: release captured mouse

The UI reports signed `l`, angular `n`, frame handedness, current content/core
region, A/B mouth visibility and last entry, precompute status, and
debug views for selected asymptotic end, proper depth, refinement work, and
finite/handedness state.

## Precompute and limits

The deterministic v4 GR cache is `cache/portal_gr_ellis_v4.bin` (about 150 KiB).
It contains metric/connection samples plus signed-depth/view-angle exit data.
Cold generation has a 55-second watchdog and currently completes in roughly
1.05 seconds; warm validation/load is about 0.23 ms on the development machine.

Free-fly camera motion is unbounded in signed proper depth. The UI identifies
whether the observer is in the intrinsic core or the positive/negative
asymptotic content continuation, and reports the non-finite safety guard if it
ever rejects a step. The following
are deferred rather than simulated incorrectly:

- F6 capsule/terrain collision while physically inside the throat;
- participating media integrated *inside* the Ellis manifold (media at either
  attached content end still render normally after ray exit);
- persistent terrain/edit objects whose bodies span the throat;

Those deferments do not affect the central claim of this lab: free-fly camera
and view rays share the native Ellis metric continuously through `l=0`.

## Regression gates

`intrinsic_ellis_manifold_test` covers radial C1 throat crossing, nonradial time
reversal, positive frame handedness, frame parallel transport, pole/ULP atlas
stability, finite GPU packing, and agreement with a 2048-step CPU oracle.

`--intrinsic-traversal-regression` replays an off-axis 180-frame camera path
through `l=0` while rendering exact voxel content. It rejects nonfinite state,
handedness loss, an adjacent-frame rotation above 0.02 radians, closed-shell
misses, and recoverable horizon misses.
