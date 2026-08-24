# Periodic pose teleport — investigation log (2026-08-19)

**What this document is.** A record of a controller-agnostic driving failure
found by comparing two live logs: the car's reported position discontinuously
jumps several metres in a single 50 ms tick, roughly every ~31.7 s, on both
the NMPC and the Stanley controller. This is **not** the known `a_lat`
ceiling issue (`sim_to_real_investigation.md`) and **not** the open
centreline-curvature-spike defect (`docs/reference/`) — it is a new,
separate, currently-unexplained failure mode found while triaging a "the car
randomly steers off" report.

**Status: root cause NOT found, but substantially narrowed.** Four live
captures (see "Capture result" through "Fourth capture result" below) have
exonerated `sim_perception.py` entirely, shown the bridge's raw 250 Hz odom
output sustaining only ~22-36 Hz for nearly the whole run, shown that
**FSDS's own internal simulation clock never drops out of lockstep with wall
time, including in the exact seconds the position teleports**, and then —
the decisive result — shown that **three independent AirSim RPC calls
(`getCarState()`/odom, `getGroundSpeedSensorData()`/`clock`+`gss`, and
`getImuData()`/`imu`) all stall together**, on a precise, fixed **~34.1 s**
period. That rules out both "it's specific to `getCarState()`" and "it's a
coincidence between two calls" — this is one shared bottleneck somewhere in
the RPC/AirSim transport boundary, not anything specific to how any one
sensor's data is fetched or cached. See "Third capture result" and "Fourth
capture result" for the measurements and what they do/don't rule out.

**Why it exists.** The investigation started as "is this a controller or
planner bug" and ended by ruling out both — worth recording precisely because
the natural instinct when a controller misbehaves is to re-tune it, and doing
that here would tune around a symptom with a perception-layer cause.

---

## The bottom line, first

**The car's reported `(x, y, yaw)` genuinely teleports** — several metres and
tens of degrees in one 50 ms control tick — **roughly every 31.7 seconds**,
independent of which controller is driving, independent of whether the path
source is a precomputed CSV or the live planner, and independent of where on
the track the car physically is at the time. The controller's own solve
timing is unaffected and its subsequent steering response is a *correct*
reaction to what looks like a real, large tracking error — the bug is
upstream of both planning and control.

The likely subsystem is `fsds_ros2_bridge`'s RPC connection to FSDS, or
FSDS/Unreal Engine's own update rate on the Windows host — **not**
`sim_perception.py`, which a live capture (below) shows publishing a
perfectly steady 20 Hz throughout. See "Where this could be happening" for
four concrete candidate mechanisms, and "Capture result" for what the first
live measurement actually showed.

---

## The evidence

Two logs from the same test session, same track (`comp_test_map_3`), same
`mpc_params.py` weights:

| | `mpc_standalone_control_1787122237.csv` (NMPC) | `stanley_control_1787121945.csv` (Stanley) |
|---|---|---|
| Controller | NMPC (`use_nmpc=1`) | Stanley (no solver at all) |
| Path source | precomputed static CSV (`path_age_s` ≡ 0 the whole run) | **live planner** (`path_age_s` 0.02–0.79 s) |
| `solve_ms`/`cmd_latency_ms` at the jumps | normal (9–21 ms, well under the 50 ms budget) | n/a |
| Teleport events found | 6 | 5 |
| Teleport period | ~31.7 s | ~31.7 s (same) |

Teleport detection: frame-to-frame position delta implying an instantaneous
speed >25 m/s (`v_max` for this track is 18 m/s, so nothing legitimate should
ever exceed this).

**NMPC run** (`t0_epoch_s = 1787122239.9599`):

| run t (s) | jump size | epoch time |
|---|---|---|
| 26.7500 | 7.34 m | 1787122266.71 |
| 58.6989 | 1.07 m | 1787122298.66 |
| 90.1474 | 4.48 m | 1787122330.11 |
| 121.8498 | 9.47 m | 1787122361.81 |
| 185.2016 | 3.37 m | 1787122425.16 |
| 216.9008 | 7.03 m | 1787122456.86 |

**Stanley run** (`t0_epoch_s = 1787121949.1745`):

| run t (s) | jump size | epoch time |
|---|---|---|
| 1.8507 | 5.63 m | 1787121951.03 |
| 33.6005 | 3.51 m | 1787121982.78 |
| 80.6000 | 7.65 m | 1787122029.77 |
| 112.1006 | 3.21 m | 1787122061.28 |
| 159.1002 | 6.19 m | 1787122108.27 |

Gaps between consecutive NMPC events: 31.95 / 31.45 / 31.70 / 63.35(≈2×) /
31.70 s. Gaps between consecutive Stanley events: 31.75 / 47.0(≈1.5×) / 31.50
/ 47.0(≈1.5×) s. Both are consistent with one underlying ~31.6–31.7 s period,
with some cycles not displacing the car far enough (depending on its speed
and heading at that instant) to cross the >25 m/s detection threshold used
here — the true event rate may be higher than the "events found" counts above
suggest.

**Concrete example** (NMPC run, the t=26.75 s event) — position, heading, and
the `pose_age_s` diagnostic all break in the same single tick:

```
t=26.6986  car_x=22.4216 car_y=60.6480 car_yaw=2.32077  pose_age_s= 0.2593
t=26.7500  car_x=16.7232 car_y=65.2699 car_yaw=2.60858  pose_age_s=-0.2926
```

A ~7.3 m position jump, a ~16.6° heading jump, and `pose_age_s` flipping sign
by ~0.55 s — all in the same 50 ms step. `pose_age_s` is `now() -
header.stamp`; a negative value is physically impossible for a real
measurement in the past, so the header stamp itself is discontinuous here,
not just the position.

The run's very last teleport (t≈216.9, NMPC) is the one that ends it:
`nmpc_status` drops to 0 there, and `e_y` then *freezes* at exactly -5.6649
for the next 40 rows while `pose_age_s` climbs unbounded to 2.53 s — the pose
topic stopped delivering entirely after that point (the car was off-track by
then; nothing left to recover with).

## What this rules out

- **Controller weights/tuning** — identical `mpc_params.py` config on both
  runs; the older, cleaner run (`mpc_standalone_control_1786653189.csv`, same
  config, only 2 borderline jumps in 133 s vs. this run's 6 in 225 s) shows
  this is a *rate* difference in whatever the cause is, not something a
  weight change introduced.
- **The QP solver** — `solve_ms`/`cmd_latency_ms` are unremarkable at every
  jump; the NMPC is solving correctly, just for a state that's already wrong
  by the time it receives it.
- **The live planner / centreline quality** — the NMPC run tracks a
  precomputed static path (`path_age_s` ≡ 0 all run — the live planner isn't
  even in that run's loop) and still shows the identical symptom; the Stanley
  run *does* use the live planner and shows the same symptom at the same
  period. Varying the planner's presence entirely doesn't change the
  behaviour, so the planner is not implicated.
- **The control law itself** — Stanley (a simple proportional law, no
  solver, no delay-compensation/rollforward machinery) shows the exact same
  teleport signature as the NMPC. The one thing both controllers share is
  reading `/fsae/slam/car_position`/`/fsae/slam/car_odom` — everything
  downstream of that is provably not the cause, since it differs between the
  two runs while the symptom doesn't.
- **Slow disk I/O** — `fsae_logs/` is on native ext4 (confirmed via `mount`),
  not a Windows `/mnt/c` DrvFs mount, so periodic filesystem stalls from
  cross-OS I/O are not a factor for the logging path itself.
- **A `fsds_ros2_bridge` timeout/retry constant** — grepped for any ~30 s
  literal in `ros2/src/fsds_ros2_bridge/src/*.cpp`; the only timeout present
  is a 10 s RPC connection timeout, unrelated.

## Where this could be happening (unconfirmed)

The full pose path: FSDS.exe (Windows) → msgpack-RPC →
`fsds_ros2_bridge`'s `AirsimROSWrapper` (C++) → `/fsds/testing_only/odom`
(250 Hz) → `sim_perception.py` → `/fsae/slam/car_position`/`car_odom`
(20 Hz) → both controllers.

1. **The RPC call crosses the WSL2↔Windows boundary.**
   [`airsim_ros_wrapper.cpp:11-12`](../../../ros2/src/fsds_ros2_bridge/src/airsim_ros_wrapper.cpp)
   connects to FSDS via `airsim_client_(host_ip, RpcLibPort, timeout_sec)` —
   this environment runs ROS2 inside WSL2 while FSDS.exe runs on the Windows
   host (per `launch_all.sh`'s own "hardcoded Windows install location"
   note), so every odom sample is a network round-trip out of the VM. FSDS
   keeps simulating in real time regardless of whether that RPC call is
   answered promptly, so a stalled call followed by a resumed one would
   produce exactly this signature: not corrupted data, a **real but severely
   delayed** sample, arriving several metres of real motion later than the
   previous one the bridge processed.
2. **`sim_perception.py`'s odom subscription is `BEST_EFFORT` with
   `depth=10`** against a 250 Hz publisher
   ([`sim_perception.py:112-119`](../../../ros2/src/fsae_planning/perception/fsae_sim_perception/fsae_sim_perception/sim_perception.py)) —
   a 40 ms buffer. Best-effort means a stall anywhere upstream is silently
   dropped, not retried or surfaced as an error — consistent with there being
   no warning, no `solver_failed`, nothing else abnormal anywhere in either
   log at the moment of the jump.
3. **`pose_age_s` mixes two clock domains.** The odom's `header.stamp`
   traces back to a FSDS-internal sim-clock timestamp fetched via a
   *separate* RPC call in
   [`clock_timer_cb`, airsim_ros_wrapper.cpp:966-987](../../../ros2/src/fsds_ros2_bridge/src/airsim_ros_wrapper.cpp)
   (the code's own comment: *"I'm really sorry for this code, but
   airsim_client_ doesn't seem to expose a method to get just the time"* —
   it piggybacks on a ground-speed-sensor RPC call's timestamp instead), while
   `pose_age_s = now() - header.stamp` is computed against the controller
   node's own clock. Any drift or discontinuity between those two sources
   would explain the negative/oversized `pose_age_s` riding along with the
   jump, independent of whatever causes the position jump itself.

4. **`odom_cb`'s duplicate-suppression filter, found already flagged in
   `launch_all.sh`.** A prior, unfinished pass at this exact symptom is
   already sitting in the launch script — an unrevereted TEMPORARY
   (2026-08-09) comment enabling `RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG`
   and piping the bridge's output to `/tmp/bridge_debug.log`, with the note:
   *"checking whether AirSim's odom dedup (`equalsMessage()` in `odom_cb`)
   is starving `/fsae/slam/car_position` during the `pose_age_s` spikes seen
   in `mpc_standalone_control_*.csv`."* — i.e. someone had already noticed
   this and started investigating before this file existed. The mechanism:
   [`odom_cb`, airsim_ros_wrapper.cpp:477-509](../../../ros2/src/fsds_ros2_bridge/src/airsim_ros_wrapper.cpp)
   polls `getCarState()` every 4 ms and **skips publishing** (`return;`,
   line 495) whenever the new state `equalsMessage()`s the previous one —
   *except* when the car's velocity is exactly zero (the stationary case is
   deliberately exempted so a parked car still heartbeats). While the car is
   moving (true at every one of our jump events — v_actual 11-14 m/s), this
   filter is active: if Unreal's underlying physics tick doesn't advance
   between two 4 ms polls, that poll is silently dropped, never published,
   never counted anywhere. Checked and left unresolved: this doesn't by
   itself explain a *discontinuous jump* (a genuinely frozen physics state
   would just repeat the same position, not skip ahead of it) — it only
   explains *why* a real stall upstream (candidate 1) would be invisible
   until the moment it resolves, since nothing between the freeze and the
   resume would have been published in between for anyone to notice. The
   DEBUG logging this comment enabled does not appear to actually be
   working: checked `/tmp/bridge_debug.log` from a 2026-08-19 session and it
   contains only INFO-level connect/shutdown lines, no
   `PrintStatistics()`/`RCLCPP_DEBUG` output at all despite the env var being
   set — that channel may not be a usable diagnostic as currently wired.

None of these four is confirmed. No ~31.7 s constant exists anywhere in
`fsds_ros2_bridge`'s source, so the period most likely originates outside
code readable from a static search here: FSDS's own compiled Unreal Engine
binary (a periodic level-streaming/GC/internal tick), the OS/network layer on
the WSL2↔Windows boundary, or possibly ROS2's DDS discovery layer (commonly
defaulted to a ~30 s re-announcement cadence in some RMW implementations).

## Capture result (2026-08-19, `topic_hz_diagnostics/*_1787125892.log`)

The `ros2 topic hz` capture below was run and answers the question the
previous section posed — **`sim_perception.py` (candidate 2) is now
exonerated.**

- **`/fsae/slam/car_position` (20 Hz) is perfectly clean for the entire
  ~20 s capture** — 19.9–20.0 Hz throughout, std dev 0.0002–0.0006 s, no
  dropout, no jitter, anywhere. `sim_perception.py`'s own 20 Hz publish
  timer is not the problem.
- **`/fsds/testing_only/odom` (250 Hz, the bridge's raw RPC-polled output)
  collapses and STAYS collapsed** — clean at ~246–250 Hz for the first
  ~3 seconds, then drops to **~25–36 Hz for the remaining ~20 seconds** of
  the capture, recovering only right at the very end (102 Hz, as load
  presumably dropped during shutdown). This is a **sustained** rate
  collapse, not the ~31.7 s periodic blip the earlier evidence (position
  teleports) suggested — the teleports are a *further*, more severe stutter
  riding on top of this already-degraded baseline, not the same event as
  the baseline collapse itself.

**Revised picture:** the bottleneck is upstream of `sim_perception.py`
entirely — either FSDS/Unreal's own simulation/physics update rate, or the
RPC round-trip to fetch it, is not sustaining anywhere near the 250 Hz the
bridge polls at. Read together with candidate 4's `equalsMessage()` dedup
filter: if Unreal's own physics/game-thread rate is genuinely running at
roughly 30 fps (very plausible under render/physics/camera load, and
matching the observed ~25–36 Hz almost exactly), then out of every ~8
`getCarState()` polls at 250 Hz, roughly 7 would return a byte-identical
state and get silently dropped by the dedup filter, and only ~1 would carry
genuinely new data — which is *exactly* what "publish rate collapses from
250 Hz to ~30 Hz, sustained" looks like from outside. On that reading, the
dedup filter isn't causing the problem, it's faithfully reporting FSDS's own
degraded update rate — the 250 Hz timer period baked into
`update_odom_every_n_sec` was simply never a real rate ceiling, just a
polling interval far above what Unreal was ever answering with fresh data.

This still doesn't fully explain the larger, discrete multi-metre teleports
(a car moving at 11-14 m/s only needs ~0.5-0.9 s of gap to explain those, and
a *sustained* ~30 Hz relay should mean at most ~33 ms between genuinely new
samples, not seconds) — those look like additional, larger stutters within
an already-struggling FSDS/render pipeline, not a separate mechanism. Worth
a longer capture (the run analysed here was only ~20 s) to see whether the
same session also shows occasional multi-hundred-ms-or-worse gaps layered on
top of the ~30 Hz baseline, ideally timed against a simultaneous
`mpc_standalone`/`stanley` control log from the same run so a teleport in
`car_x`/`car_y` can be matched directly against a specific gap in
`odom_hz_*.log`.

**This "check the frame rate" lever is now superseded — see the second
capture immediately below, which measured this directly instead of
inferring it and got the opposite answer.**

## Second capture result (2026-08-19, `topic_hz_diagnostics/*_1787126824.*`, ~177 s run)

A longer (~177 s) session, this time also with `clock_drift_check.py`
running (see "Next step" below for what it does) and a matching
`mpc_standalone_control_1787126827.csv` driving log from the same session.

**The position teleport recurred — 6 times, same ~31.7 s cadence as
before**, at t = 26.65, 58.34, 65.24, 89.996, 121.70, 153.35 s (jump sizes
3.0–8.6 m). Not a one-off at the start of a run: it recurs throughout a long
session at the same characteristic period found earlier.

**The odom-rate collapse also recurred, matching the first capture almost
exactly**: ~245 Hz for the first ~3 s, then a sustained ~22–36 Hz for
essentially the entire rest of the 177 s run (347 `topic_hz` print lines,
i.e. ~347 s of 1-per-second samples covering the full run — this is not
transient). `/fsae/slam/car_position` never dropped below ~19.9 Hz the whole
time, confirming (again) that `sim_perception.py`'s 20 Hz relay never
actually starved this run either — the visible teleports must come from
something sharper than this sustained baseline degradation, layered on top
of it.

**The decisive new result: `clock_drift_check.py`'s data rules out "FSDS's
own simulation is slow."** Computed `d(sim_time)/d(wall_time)` in 1-second
windows across the whole run: it holds at 0.94–1.00 throughout (mean ratio
over the full 177 s: **0.998** — essentially perfect real-time tracking),
with **no dip at all in the specific seconds containing each of the six
teleport events** (checked individually: 0.976, 0.997, 0.986, 1.002, 0.986,
0.981 — all unremarkable). FSDS's internal simulation clock is not stalling,
slowing down, or hiccupping at any point this run, including at the exact
moments the reported car position discontinuously jumps.

This overturns the previous capture's "Unreal's own frame/physics rate is
probably running around 30 fps" reading. The simulation's own sense of time
advances correctly and continuously. **The discontinuity must therefore be
introduced somewhere between Unreal's true, continuously-advancing internal
state and what `getCarState()`'s RPC response actually delivers** — a stale
cached response, a server-side buffering/queueing behaviour independent of
the simulation clock, or something in how the RPC transport itself
serialises/delivers responses under a periodic condition this hasn't
identified yet.

**Correction (see "Third capture result" below): the claim in the previous
paragraph that this "does not touch the GSS-sourced `/clock` RPC path the
same way" is wrong.** It was based only on `/clock`'s *value* never drifting
from wall time, not on whether `/clock` itself arrives late. The very next
capture, adding a `/clock` arrival-rate measurement, shows `/clock` stalls on
the same ~33 s cadence as odom.

## Third capture result (2026-08-20, `topic_hz_diagnostics/*_1787169224.*`, ~426 s run)

The `/clock` arrival-rate capture from "Next step" (previous revision of this
doc) is now wired into `launch_all.sh` and has been run once, alongside the
existing odom/car_position/clock-drift captures, in the same session.

**`/clock`'s own arrival rate collapses in lockstep with odom, on the same
~33-34 s cadence.** Three independent measurements from this one run agree:

- `clock_drift_1787169224.csv`'s raw wall-clock timestamps (the
  `clock_drift_check.py` subscriber, sampling at ~100 Hz) show 7 gaps of
  1.4-1.7 s in `/clock` arrival, at wall-clock offsets t = 1.8, 35.8, 69.5,
  103.2, 137.4, 171.4, 205.0 s from run start — a consistent ~33.7 s period,
  matching the ~31.7 s teleport cadence from the first two captures within
  the variance already seen between runs.
- The independent `ros2 topic hz -w 5 /clock` process (`clock_hz_*.log`)
  shows its own average rate repeatedly collapsing from a ~95-107 Hz baseline
  down to ~2.6-3.5 Hz, at the same cadence.
- `odom_hz_1787169224.log` shows its familiar ~22-36 Hz sustained-degradation
  baseline (as in both prior captures) additionally punctuated by sharper
  drops at matching intervals (sample-index gaps of ~32 between drops, at
  ~1.05 s per printed sample ⇒ ~33.6 s).

**This overturns the leading hypothesis from the "Second capture result"
above.** The stall is not specific to `getCarState()`'s RPC path — it hits
`/clock`, which is sourced from a *different* RPC call (the GSS clock read;
see the comment above the `/clock` capture in `launch_all.sh`). Whatever is
periodically stalling responses, it is shared across at least two distinct
RPC calls, which points at the RPC/AirSim transport boundary itself (a
shared server-side thread, queue, or lock that both calls contend on) rather
than at anything specific to how `getCarState()` in particular is computed or
cached.

**What this does not yet tell us:** whether *every* RPC call stalls together
(a single shared bottleneck) or whether this is coincidental overlap between
two of several independently-stalling calls — testing a third, unrelated RPC
topic would distinguish these. It also does not explain *why* the stall
recurs on such a precise, fixed period (~33-34 s) rather than under load or
at random — that periodicity is itself a strong clue (something is running
on a fixed internal timer/tick — a GC pause, a periodic flush, a polling
loop with a ~33 s interval) that hasn't been chased down yet.

## Fourth capture result (2026-08-20, `topic_hz_diagnostics/*_1787170066.*`, ~137 s run)

Added a fifth capture, `ros2 topic hz -w 5 /fsds/imu` (`imu_hz_*.log`), to
test whether the stall is a shared RPC/AirSim bottleneck (predicts a third,
unrelated RPC call also stalls) or a coincidence between exactly two calls
(predicts it doesn't). `/fsds/imu` is backed by `getImuData()`
(`imu_timer_cb`, `airsim_ros_wrapper.cpp:547`) — a genuinely distinct RPC
method from both `getCarState()` (odom) and `getGroundSpeedSensorData()`
(which backs *both* `/clock` and `/fsds/gss`, so `/fsds/gss` would not have
been an independent third test).

**`/fsds/imu` also stalls, on the exact same cadence as odom and `/clock`.**
In this one run: `imu_hz`'s average rate collapses from a ~250 Hz baseline
to ~3 Hz at sample indices [8, 41, 74, 107] — a clean 33-sample period —
matching `/clock`'s stall indices exactly ([8, 41, 74, 107]) and odom's
almost exactly ([8, 40, 72, 104], off by ≤2, consistent with three
independently-polling `ros2 topic hz` processes not starting in lockstep).
The `clock_drift_check.py` wall-clock timestamps for this same run pin the
period precisely: stalls at t = 9.38, 43.48, 77.57, 111.68 s, i.e. a period
of **34.10, 34.09, 34.11 s** — tighter and more consistent than the ~31.7-33.7 s
range seen across the three earlier captures (likely just measurement
granularity in the earlier ones; this is the most precise measurement so
far).

**This answers the question left open after the third capture: it is one
shared bottleneck, not two calls coincidentally stalling together.** Three
distinct RPC calls — `getCarState()` (odom), `getGroundSpeedSensorData()`
(`/clock`/`gss`), and `getImuData()` (`imu`) — all stall in lockstep on a
fixed ~34.1 s period. Whatever this is, it sits somewhere shared by all RPC
traffic between the bridge and AirSim: the msgpack-rpc transport itself, a
lock/queue on the AirSim server side that serialises all client calls, or a
periodic stall in Unreal/AirSim's own request-handling loop that blocks
every RPC response simultaneously regardless of which method was called.

## Next step (2026-08-20 — shared-bottleneck question answered; root cause still open)

`ros2/launch_all.sh` launches, right after the bridge comes up (search for
`TEMPORARY (2026-08-19)`): five background captures — `ros2 topic hz -w 5`
on `/fsds/testing_only/odom`, `/fsae/slam/car_position`, `/clock`, and
`/fsds/imu`, plus `ros2/clock_drift_check.py` subscribing to `/clock` for
its value-drift check — all logging into `fsae_logs/topic_hz_diagnostics/`,
torn down alongside the bridge in `cleanup()`. Four sessions have now been
captured and analysed (see all four "Capture result" sections above) —
`sim_perception.py`, FSDS's own simulation clock, "specific to
`getCarState()`", and "two calls coincidentally overlapping" are all
cleared. The open question is now specifically **what** in the shared
RPC/AirSim transport stalls on a fixed ~34.1 s period.

**What's answered:** `sim_perception.py` is not the cause (candidate 2, ruled
out twice); FSDS's own simulation clock is not stalling (the "Unreal frame
rate" reading of candidate 1 is overturned); the stall is not specific to
`getCarState()` — three independent RPC calls (`getCarState()`,
`getGroundSpeedSensorData()`, `getImuData()`) all stall together on the same
fixed period (Third and Fourth capture results). The "RPC round-trip timing
inside the bridge, specific to `getCarState()`'s caching" candidate below is
now effectively ruled out too, superseded by the shared-bottleneck finding.

**What's still open**, and the most promising next things to try:

- **What has a ~34.1 s period, and where does it live — bridge, transport,
  or AirSim/Unreal server?** This is now the central question. Candidates,
  roughly in order of how directly they'd explain a period this precise and
  this widely shared:
  - A periodic stall on the **AirSim/Unreal RPC server side** that blocks
    all incoming RPC calls simultaneously (e.g. a GC pause in whatever
    runtime backs the RPC server, a periodic checkpoint/flush, a
    single-threaded request queue backing up behind some other periodic
    task). This is the most direct explanation for "every RPC call stalls
    together" and would need AirSim/Unreal-side logging or profiling during
    a run to confirm — not another ROS2-side capture.
  - A shared **client-side** bottleneck in the bridge itself — e.g. all
    RPC calls funnel through the same `car_control_mutex_` (confirmed shared
    by odom/imu/gss call sites) or the same underlying `rpc::client`
    connection, and something else holding that lock/connection for ~1.5 s
    every ~34 s would produce exactly this symptom without AirSim's own
    request handling being at fault at all. Worth checking what else in
    `airsim_ros_wrapper.cpp` acquires `car_control_mutex_` and whether any of
    it runs on a ~34 s-ish period (or accumulates to trigger something at
    that interval, e.g. a buffer/counter that wraps or flushes every N
    calls).
  - The underlying **msgpack-rpc transport** (TCP-level stall, e.g. a
    periodic keepalive/reconnect, buffer flush, or OS-level scheduling
    hiccup affecting the single socket all these calls share) — harder to
    rule in/out without packet-level capture (e.g. `tcpdump` on the bridge
    ↔ AirSim port during a run, watching for a ~34 s-periodic gap in traffic
    itself rather than in ROS2-side topic delivery).
- **Instrument `odom_cb` directly** (deferred so far because it needs a C++
  rebuild): log the wall time immediately before and after each
  `getCarState()` call. Still useful even after the shared-bottleneck
  finding — it would show whether the RPC *call itself* blocks for ~1.5 s
  (implicating the transport or server) or returns fast-but-stale
  (implicating a client-side cache/lock) during a stall window. Extending
  the same timestamp logging to `imu_timer_cb` and `gss_timer_cb`/
  `clock_timer_cb` in the same rebuild would directly show whether all three
  block simultaneously (shared lock/transport) or return staggered-but-stale
  responses (shared cache).

This block is temporary and should be removed from `launch_all.sh` (the
launch-time block, its `cleanup()` teardown, and `ros2/clock_drift_check.py`)
once the root cause is found — it is not meant to ship as a permanent part
of the launch flow.

`pose_age_s`'s clock-domain concern (candidate 3) remains an open,
independent question — worth revisiting once the position-teleport
mechanism itself is understood, since it may turn out to be a separate,
smaller effect riding on top of whatever that turns out to be.

## Candidate: host/hardware-level cause, not the simulator (2026-08-20 — raised, not investigated)

Everything ruled in/out so far (candidates above, all four captures) has
assumed the stall originates somewhere in FSDS/AirSim/Unreal or the bridge
software itself. Worth explicitly recording a different class of candidate
that hasn't been investigated at all yet: **something at the host OS or
hardware level, on this specific machine, rather than in the simulator
stack**. A stall this precise (~34.1 s, tight across repeated measurements)
and this indiscriminate (blocks every RPC call simultaneously, regardless of
which AirSim method) is at least as consistent with a periodic host-level
event as with anything in FSDS/AirSim's own code. Candidates in this class,
none yet checked:

- **WSL2 networking/vEthernet housekeeping.** This setup bridges a
  Windows-hosted `FSDS.exe` to a WSL2-hosted ROS2/bridge process across the
  WSL2 NAT/vEthernet boundary (`launch_all.sh` already does `FSDS_HOST_IP`
  gateway detection for this). WSL2's virtual switch, NAT table, or DNS
  cache has known periodic refresh/housekeeping behaviour; if any of it runs
  on a fixed cycle in the tens-of-seconds range, it could stall the
  cross-boundary socket every RPC call shares, independent of anything in
  AirSim.
- **Windows power/thermal management.** A power-plan-driven CPU
  parking/turbo-duty-cycle or thermal-throttling event on a fixed period
  could stall whatever thread(s) service RPC requests inside `FSDS.exe`,
  without any code-level cause in AirSim/Unreal itself.
- **GPU driver periodic stalls.** Unreal's main thread (which likely also
  services or gates AirSim's RPC handling) blocking on a GPU-driver-level
  periodic event — thermal throttling, a driver housekeeping tick, shader
  cache maintenance — would produce exactly the "every RPC call stalls
  together" signature seen so far, sourced from the GPU/driver rather than
  from FSDS/AirSim application code.
- **Disk I/O / antivirus interference.** A periodic flush (engine log,
  telemetry, level-streaming cache) intercepted by Windows Defender
  real-time scanning or slow disk I/O could stall the process for
  ~1.5 s on whatever period that flush runs on.
- **USB/peripheral polling.** Lower-confidence, included for completeness:
  some USB controllers/wireless peripherals have reconnect or polling
  cycles in this rough time range.

**Status: logged only, not pursued further per explicit instruction.** None
of these have been checked against this machine's actual configuration
(Windows event logs, `perfmon`, WSL2 network diagnostics, GPU driver logs,
etc.), and none should be assumed more or less likely than the
AirSim/transport-side candidates above without that evidence. If revisited,
the natural first check is whether the ~34.1 s period is stable across a
different host machine or a bare-Windows (non-WSL2) bridge setup — if the
period changes or disappears entirely on different hardware/OS
configuration, that would strongly implicate this class of cause over
anything in the FSDS/AirSim software stack.

## Related logs

- `sim_to_real_investigation.md` — a different, already-closed gap (the
  `a_lat` ceiling). Not this bug.
- `late_turn_in_investigation.md` — the NMPC's turn-in-timing work. Not this
  bug (this failure mode reproduces on Stanley too, which that file's NMPC
  work never touches).
- `docs/reference/simulator_fidelity.md`'s "Known planner defect: centreline curvature
  spikes" — a real, separate, already-documented issue. Ruled out here
  because this bug reproduces with a *precomputed, static* path where the
  planner isn't even running.
