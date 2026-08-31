#!/bin/bash
# Language: bash
# Title: Rock-Solid Auto-Launch and Cleanup Orchestrator for launch_all.sh
#
# PURPOSE
# -------
# One-command launcher for the full FSDS + fsae_planning driving stack on
# this machine: starts the Windows FSDS simulator, waits for its AirSim RPC
# server to come up, launches fsds_ros2_bridge, then launches the planning/
# control/perception stack (sim.launch.py) in the foreground. On Ctrl+C or
# SIGTERM it tears everything down (bridge process, FSDS/FSOnline/Blocks.exe,
# stray core dumps) via a single cleanup() trap, so a driving session is
# start-to-stop with no manual process hunting.
#
# This is the file to edit to change what a plain `./launch_all.sh` drives:
# which track, which controller, speed caps, and the MPC tuning shortlist.
# Do not edit the defaults inside sim.launch.py/control.launch.py for a
# one-off change — override them here instead.
#
# INDEX
# -----
#   CONFIGURATION           track selection, precomputed-map CSVs/toggles,
#                           controller choice, speed caps, MPC tuning
#                           shortlist (lines ~5-153)
#   Native vs Docker         picks a host ROS 2 install over the Docker
#                           container when available (~155-160)
#   WSL2 networking          FSDS_HOST_IP gateway detection for WSL (~162-173)
#   cleanup()                SIGINT/SIGTERM trap: kills bridge + FSDS
#                           processes, sweeps core dumps (~175-212)
#   [1/3] Launch simulator   starts FSDS.exe, waits for AirSim RPC (~233-269)
#   [2/3] Launch bridge      starts fsds_ros2_bridge in the background
#                           (~296-325; symlink-install rebuild step
#                           currently commented out just above it)
#   [3/3] Launch stack       starts sim.launch.py in the foreground with
#                           this file's config baked in as launch args
#                           (~327-368)
#
# --- CONFIGURATION ---
CONTAINER_NAME="fsds_ros2_bridge"
WINDOWS_SIM_PATH="/mnt/c/Users/marti/Downloads/fsds-v2.2.0-windows/FSDS.exe"
HOST_ROS2_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_ROS2_DIR="/root/Formula-Student-Driverless-Simulator/ros2"
HOST_REPO_ROOT="$(dirname "$HOST_ROS2_DIR")"
CONTAINER_REPO_ROOT="$(dirname "$CONTAINER_ROS2_DIR")"

# Which recorded track the car drives. Selects BOTH precomputed CSVs at once
# from fsae_planning's own ros2/src/fsae_planning/tracks/<TRACK>/ --
# speed_profile.csv (the centreline + oracle speed) and the geometry file
# (centerline.csv if the track has one, else raceline.csv -- see
# _newest_track/_track_geometry_name below).
#
# tracks/ is committed data inside fsae_planning (not gitignored, not
# generated at runtime), so a fresh clone of FSDS + fsae_planning alone can
# drive its newest track immediately -- no fsae_MPCTest checkout needed to
# READ an existing track. fsae_MPCTest is only where NEW tracks get produced
# (recording + the two exporters); its output is then copied into
# fsae_planning's tracks/<name>/ so it ships with that repo going forward.
#
# To see what's available:  ls "$(dirname "${BASH_SOURCE[0]}")/src/fsae_planning/tracks"
# To add a new one (requires fsae_MPCTest for the exporters):
#                           record a lap, run the two exporters -- full
#                           workflow in fsae_MPCTest/docs/developer_guide.md
#                           ("Recording, exporting and driving a track") --
#                           then copy the resulting tracks/<name>/ directory
#                           into ros2/src/fsae_planning/tracks/<name>/.
#
# TRACK defaults to the MOST RECENTLY RECORDED track, resolved below by
# _newest_track -- a pure-bash reimplementation of
# fsae_MPCTest/tracks/newest_track() (mtime of cone_map.json, not a
# name-embedded date, so it is correct for an undated legacy track and for a
# re-recorded one -- see that function's own docstring for why). This launch
# script must not depend on the fsae_MPCTest checkout existing (see the repo
# layout note above), so the logic is duplicated in bash rather than shelled
# out to Python; keep the two in sync if the selection rule ever changes.
#
# Set TRACK= explicitly (uncomment below) to pin a specific track instead of
# always using the newest one -- e.g. while comparing two recordings, or if
# a newer, still-being-tuned track should not yet become the default.
# TRACK=comp_test_map_3
_newest_track() {
    local tracks_dir="$1" best="" best_mtime=-1 d mtime
    [ -d "$tracks_dir" ] || return 1
    for d in "$tracks_dir"/*/; do
        d="${d%/}"
        [ -f "$d/cone_map.json" ] || continue
        mtime=$(stat -c '%Y' "$d/cone_map.json" 2>/dev/null || stat -f '%m' "$d/cone_map.json" 2>/dev/null)
        [ -n "$mtime" ] || continue
        if [ "$mtime" -gt "$best_mtime" ]; then
            best_mtime="$mtime"
            best="$(basename "$d")"
        fi
    done
    [ -n "$best" ] || return 1
    echo "$best"
}
# Which geometry file a track prefers: centerline.csv if it has one exported,
# else raceline.csv, else empty (only a cone map, nothing exported yet).
# Mirrors fsae_MPCTest/tracks/geometry_path()'s preference exactly.
_track_geometry_name() {
    local track_dir="$1"
    if [ -f "$track_dir/centerline.csv" ]; then echo "centerline.csv";
    elif [ -f "$track_dir/raceline.csv" ]; then echo "raceline.csv";
    fi
}
if [ -z "${TRACK:-}" ]; then
    TRACK="$(_newest_track "$HOST_ROS2_DIR/src/fsae_planning/tracks")" || {
        echo "ERROR: no track found under $HOST_ROS2_DIR/src/fsae_planning/tracks" \
             "(each needs at least a cone_map.json). Record one, or set TRACK=" \
             "explicitly above." >&2
        exit 1
    }
fi

# Absolute paths handed to the launch files. Absolute (not derived from the
# launch file's own location) because colcon copies those files into
# ros2/install/... at build time, which has no relationship to this file's
# location in src/. Derived from this script's location, which IS stable.
TRACK_DIR="$HOST_ROS2_DIR/src/fsae_planning/tracks/$TRACK"
# SPEED comes from speed_profile.csv (the centreline oracle), GEOMETRY from
# raceline.csv. These describe different lines, which looks wrong and was
# briefly "fixed" on 2026-08-10 by pointing both at raceline.csv -- that
# regressed the car badly (RMSE 0.33 -> 1.36 m, peak |e_y| 1.30 -> 4.80 m,
# steering saturation 5.6% -> 21.4%) and was reverted.
#
# WHY the mismatched pair is nonetheless the better one today:
#   * The precomputed-speed branch in mpc_controller_standalone.py applies NO
#     v_max clip -- v_max only reaches the live curvature_speed() branch. So
#     the CSV's own top speed IS the car's top speed, and pairing the two
#     files means the speed cap is whichever one SPEED_CSV happens to carry.
#     Check both before swapping either: as exported today speed_profile.csv
#     tops out ABOVE raceline.csv/centerline.csv, so the direction of this
#     hazard is the opposite of what it was when the 2026-08-10 revert
#     happened -- do not assume, re-read the files.
#   * speed_profile.csv is generated against CURVATURE_SPEED_A_LAT_MAX (see
#     sim/speed_profile.py), deliberately under the measured FSDS lateral
#     ceiling, so its conservatism does real work in keeping corner demand
#     inside the plant's limits -- see docs/reference/reference_path_and_speed.md's
#     "Speed-profile aggressiveness" section before raising it.
# Revisit only after the precomputed branch clips to v_max; until then this
# pairing is load-bearing, not an oversight.
SPEED_CSV="$TRACK_DIR/speed_profile.csv"
# GEOMETRY source. Defaults to the NEWEST export within $TRACK, preferring
# centerline.csv over raceline.csv when both exist (see _track_geometry_name
# above) -- centerline.csv is the DIAGNOSTIC line (raceline_optimizer --mode
# centerline): the geometric middle of the track, speed-optimised but never
# shifted laterally. Set PATH_CSV explicitly below (e.g. back to
# "$TRACK_DIR/raceline.csv") for a timed run.
#
# Why drive the centreline by default: on raceline.csv a large logged |e_y|
# is ambiguous, because the line intentionally sits near a boundary at an
# apex, so "1.8 m off the path" can be a tracking failure OR the line doing
# its job. On the centreline |e_y| is unambiguously distance from the middle
# of the track, which is what makes a "drove too close to the cones" report
# answerable from the log alone.
# Left empty (not hard-errored here) when the track has no geometry export
# yet -- e.g. a brand-new track that is about to be RECORDED, where nothing
# under TRACK_DIR exists and USE_PRECOMPUTED_PATH=false is the documented
# setting precisely so this is never read. The "fail early and loudly" guard
# below already checks PATH_CSV's existence, but only when
# USE_PRECOMPUTED_PATH=true actually needs it -- erroring unconditionally
# here would break that recording workflow before it can even start.
_TRACK_GEOMETRY_NAME="$(_track_geometry_name "$TRACK_DIR")"
PATH_CSV="$TRACK_DIR/$_TRACK_GEOMETRY_NAME"

# Precomputed-map toggles for the mpc controller (see
# fsae_planning's README.md "Precomputed-map launch args" and sim.launch.py's
# own DeclareLaunchArgument defaults for the full explanation). Set here so
# they don't need to be typed on every launch; both default to matching
# sim.launch.py's own defaults (true).
#
# Set BOTH to false (with CONTROLLER=stanley below) when recording a NEW
# track: the precomputed toggles replay the OLD map/oracle path instead of
# driving off the live planner, which defeats recording a fresh lap.
USE_PRECOMPUTED_SPEED=true
USE_PRECOMPUTED_PATH=true
# Use raceline_optimizer.py's shaped psi_target column (heading-lead
# reference, see late_turn_in_investigation.md Part 8/9/10/12) in place of
# the geometric path tangent for e_psi's reference. Only has an effect
# when USE_PRECOMPUTED_PATH=true.
#
# LIVE-TESTED 2026-08-12 at HEADING_LEAD_AUTHORITY_FRAC=0.5 (the shipped
# default) and found WORSE, not better on the first two runs: mean/peak
# |e_psi| and |e_y| both rose vs. the same-day baseline on the exact
# corner this investigation has tracked throughout, steering saturation
# roughly doubled. Likely cause (Part 12): this track has almost no true
# straights, so the lead is active nearly everywhere (not gated to the
# approach phase), fighting the corner's own geometry through its
# interior rather than helping commit to it early -- see Part 12's
# candidate #2 (gate the lead to the approach phase only) before assuming
# a different authority_frac alone will fix this.
USE_PRECOMPUTED_HEADING_PROFILE=false

# Fail early and loudly on a mistyped TRACK or a track whose exports were
# never generated. Without this the launch still starts, the controller logs
# one error and silently falls back to live curvature_speed()/the live
# planner -- i.e. it drives, just not the way the operator asked, which is
# easy to miss until the lap looks wrong.
for _f in \
    "$( [ "$USE_PRECOMPUTED_SPEED" = true ] && echo "$SPEED_CSV" )" \
    "$( [ "$USE_PRECOMPUTED_PATH"  = true ] && echo "$PATH_CSV"  )" ; do
    if [ -n "$_f" ] && [ ! -f "$_f" ]; then
        echo "ERROR: TRACK='$TRACK' is missing $_f" >&2
        echo "       Available tracks: $(ls "$HOST_ROS2_DIR/src/fsae_planning/tracks" 2>/dev/null | tr '\n' ' ')" >&2
        echo "       To add this track, export it in fsae_MPCTest (a separate" >&2
        echo "       repo -- see fsae_MPCTest/docs/developer_guide.md," >&2
        echo "       'Recording, exporting and driving a track') then copy its" >&2
        echo "       tracks/$TRACK/ directory into ros2/src/fsae_planning/tracks/:" >&2
        echo "         cd $HOST_REPO_ROOT/fsae_MPCTest" >&2
        echo "         python3 -m tuner.export_speed_profile $TRACK" >&2
        echo "         python3 -m tuner.raceline_optimizer   $TRACK" >&2
        echo "         python3 -m tuner.tools.raceline_optimizer $TRACK --mode centerline" >&2
        echo "         cp -r tracks/$TRACK \"$HOST_ROS2_DIR/src/fsae_planning/tracks/\"" >&2
        exit 1
    fi
done

# Path-tracking controller to launch (stanley | mpc — see sim.launch.py's
# own 'controller' DeclareLaunchArgument). Matches sim.launch.py's default
# (mpc) unless overridden here.
CONTROLLER=mpc

# mpc only: true (default) -> the MPC node publishes fs_msgs/ControlCommand
# directly with its own throttle/brake (fsds_bridge skipped) -- this is
# what "mpc_standalone" used to mean before mpc_controller.py and
# mpc_controller_standalone.py were merged into one node with this toggle.
# false -> steering only via the shared cmd_vel interface, fsds_bridge
# computes throttle/brake (what plain "mpc" used to mean).
STANDALONE_OUTPUT=true

# Speed caps passed to the controller node (see sim.launch.py's v_max/v_min
# DeclareLaunchArgument -- overrides fsae_params.yaml's controller.v_max/
# v_min without editing that shared file).
V_MAX=20.0
V_MIN=1.5

# ── Controller-scoped settings ───────────────────────────────────────
# [LTV-QP only] settings below have NO EFFECT when USE_NMPC=true.
# [NMPC only] settings below have NO EFFECT when USE_NMPC=false (default).
# Settings marked [shared] affect both controllers.

# ── NONLINEAR MPC (NMPC) ──────────────────────────────────────────────
# false (default) = today's LTV-QP controller (fsae_control/mpc_core.py's
# MPCController). true = the Frenet-frame NONLINEAR MPC
# (fsae_control/nmpc_core.py's NMPCController): the path's curvature kappa(s)
# is part of its prediction model, so its own rollout predicts drifting off
# line if it does not start turning -- the structural gap every mechanism in
# late_turn_in_investigation.md Parts 1-15 was working around. Offline,
# closed-loop against fsae_MPCTest's Pacejka plant on comp_test_map_3 with
# identical weights, it turned in earlier on 7/7 corners (median 25.6 m),
# cut steering saturation from 12.5% to 0.8%, cut |e_y| p90 from 1.45 m to
# 0.69 m and finished the lap 1.1 s faster -- see Part 16 §16.6/§16.7.
# LIVE-TESTED 2026-08-13 (matched same-day pair, comp_test_map_3, same
# weights): steering saturation 6.45% -> 0.58%, lap 54.72s -> 52.35s -- see
# docs/reference/control_mechanisms.md's "Nonlinear MPC"
# section for the full live A/B.
#
# Notes when true:
#   * USE_PRECOMPUTED_HEADING_PROFILE has NO EFFECT (the NMPC models the
#     curvature that profile approximates -- it logs one line saying so).
#   * The whole adaptive gain schedule (MPC_ADAPTIVE_*, anti-hunt, lookahead
#     boosts) is NOT applied: those mechanisms exist to fake the anticipation
#     this model does directly. Only the base weights below are used.
#   * USE_PRECOMPUTED_PATH / _SPEED work exactly as they do today, and are
#     the intended configuration for it.
USE_NMPC=true
# NMPC shortlist (same commented-out-by-default pattern as the MPC one below).
# NMPC_Q_E_Y/_Q_E_PSI/_Q_EPSI_DOT/_R_DELTA/_R_RATE_DELTA below now forward to
# fields that live in MPCParams itself (moved there 2026-08-13 from
# NMPCParams — see mpc_params.py's "NMPC weight overrides" section), right
# alongside every OTHER MPC weight in the shortlist further down this file.
# Kept listed here too (not just there) since they're the ones most likely
# to get tuned specifically for the NMPC.
# NMPC_HORIZON=20                     # [NMPC only] steps (x dt=0.05). 20 = 1.0 s; measured better than 35 -- see nmpc_params.py's sweep table
# Gauss-Newton iterations per tick. 1 was previously measured better AND
# cheaper than 2 *for chatter*, which is why it is the default -- but that
# was a different symptom. Being retested here against a specific one: a
# single-tick +6 deg steering step immediately followed by a retreat, seen
# inside tight corners, which lines up with an irregular reference advance
# (s0 stepping ~0.67 m on the spike tick vs ~0.33 normally) rather than with
# any cost weight. With iters=1 (real-time iteration) each solve takes ONE
# Gauss-Newton step from the warm start, so a linearisation point that moves
# further than the warm start anticipated is overshot and corrected next
# tick. A second iteration should absorb that if this reading is right; if
# the spikes survive, the disturbance is upstream in the reference/clock and
# no controller setting will remove it.
# FALSIFIED for the steering-spike symptom, do not retry without new
# evidence. Offline sweep (which shows the spikes far more strongly than live:
# 9.6% of ticks at |d|>5 deg vs 0.2-0.7% live) found the ticks actually pinned
# at the slew limit are FLAT across iteration counts -- |d|>8.9 deg is
# 2.29 / 2.22 / 2.33 % at iters 1 / 2 / 3, and max|d| sits at the 9.00 deg/tick
# ceiling in all three. Extra iterations trim only mid-range 5-9 deg activity
# while |e_y| and score both degrade, solve_ms max reaches 52.8 ms (over the
# 50 ms tick) at iters=2, and iters=3 DNFs. The spikes are therefore not an
# under-converged Gauss-Newton step.
# NMPC_SQP_ITERS=1
# Raised from 25.0 because solve_ms already peaks at ~35 ms with ONE
# iteration, so two would be truncated by the old budget and the test would
# measure the truncation instead of the second iteration. 40 ms still fits
# inside the 50 ms tick. Watch solve_ms and nmpc_iters in the log: if iters
# reads 1 on the spike ticks, the budget is still cutting it short.
# NMPC_SOLVE_BUDGET_MS=25.0
# Lateral-error weight for the NMPC only (-1 inherits the shared q_e_y,
# 6.35). Raised above the inherited value to test whether mid-corner drift is
# an error/effort-weight balance rather than a rate-cost limit: the car
# carries 0.5-1.0 m of lateral error through a corner while leaving ~10 deg
# of steering authority unused, which is what an under-weighted error term
# looks like. Set on the NMPC side only so the LTV-QP baseline is unchanged.
#
# 7.5 vs the inherited 6.35, live: score-neutral (0.4538 vs 0.4542) with a
# lower peak lateral error (1.023 vs 1.116 m), so it is kept.
#
# CAUTION on how this was nearly mis-read: raw drift-episode COUNTS across
# runs of different length made 7.5 look much worse (48 episodes vs 29). Those
# runs were 89.9 s and 55.0 s. Rate-normalised, every configuration tried on
# this track sits at ~31.5 drift episodes/min -- 31.6 / 32.0 / 31.5 across
# q6.35 and both q7.5 runs. Normalise by duration before comparing, and treat
# a single run's lap time as noisy: the same config gave 53.29 s and 47.99 s.
NMPC_Q_E_Y=7.5
# NMPC_Q_E_PSI=-1.0                   # [NMPC only] -1 = inherit q_e_psi
# NMPC_Q_EPSI_DOT=-1.0                # [NMPC only] -1 = inherit q_r. NOTE: weights HEADING-ERROR RATE (r - kappa*s_dot), not absolute yaw rate -- the one weight whose meaning changes, expect to re-sweep it
# NMPC_R_DELTA=-1.0                   # [NMPC only] -1 = inherit r_delta
# NMPC_R_RATE_DELTA=-1.0              # [NMPC only] -1 = inherit r_rate_delta
# NMPC_ALAT_CEILING_ENABLED=true      # [NMPC only] model FSDS's measured sustained a_lat ceiling inside the prediction. true is correct for FSDS; without it the NMPC oscillated and eventually spun offline (Part 16 §16.6)
# NMPC_SPLINE_REFERENCE_ENABLED=true         # [NMPC only] default true; analytic-spline kappa(s)/psi_ref(s) instead of moving-average+finite-difference. Numerical-quality fix, not a tuning knob -- see docs/reference/control_mechanisms.md's "Three MPCC-inspired additions" (2026-08-13)
# LIVE-TESTED 2026-08-13 and REJECTED: v_actual reached ~16.7 m/s against a
# v_desired of ~3.3-5 m/s for nearly 2s mid-corner, and the car went off-track
# by up to 3.6 m (mpc_standalone_control_1786585464.csv, t~58-61s) -- the
# solver pre-pays a future higher speed target the same way the three earlier
# curvature-scheduling attempts pre-paid a future turn, except the cost here
# is real off-track excursions, not just a steering wobble. See
# docs/reference/control_mechanisms.md's "Three MPCC-inspired additions" section for the
# full writeup. Do not re-enable without a fix to the underlying mechanism
# (e.g. bounding how far ahead the sampled v_ref is allowed to rise) and a
# fresh offline A/B first.
# NMPC_HORIZON_SPEED_PROFILE_ENABLED=false   # [NMPC only, EXPERIMENTAL] default false; sample the speed profile at each horizon stage's own predicted arc length instead of one frozen v_ref. REJECTED 2026-08-13 -- see note above.
# LIVE-TESTED 2026-08-13 and REJECTED: enabled with zero prior validation
# (no offline A/B), user reported it "pretty much doesn't work anymore" --
# reverted immediately without a detailed log post-mortem. Do not re-enable
# without an offline A/B first (tuner.nmpc_offline_check or equivalent) --
# this was exactly the caution given before enabling it live.
# NMPC_FRICTION_CIRCLE_ENABLED=false         # [NMPC only, EXPERIMENTAL] default false; hard per-axle tyre-force bound in the QP, additional to the existing soft alat-ceiling saturation. REJECTED 2026-08-13 -- see note above.
# NMPC_STEER_RATE_ANTI_HUNT_ENABLED=false    # [NMPC only, EXPERIMENTAL] default false; reuses the LTV-QP's steer_rate_anti_hunt penalty (extra R_rate[0,0] cost when centred/aligned/uncurving) on the NMPC too, independent of MPC_STEER_RATE_ANTI_HUNT_ENABLED above. Mutually exclusive with NMPC_CORNER_RRATE_BLEND_ENABLED below -- blend takes priority if both are set. NOT YET LIVE-TESTED -- offline A/B first (tuner.nmpc_offline_check or equivalent).
# NMPC_ANTI_HUNT_BOOST_MAX=-1.0               # [NMPC only] -1 = inherit anti_hunt_boost_max; only read when NMPC_STEER_RATE_ANTI_HUNT_ENABLED=true
# DEPRIORITIZED 2026-08-19: this changes the QP's own R_rate[steer] weight,
# which measurably made jitter WORSE (std 4.4 -> 5.5 deg) by silently
# overwriting an already-tuned NMPC_R_RATE_DELTA=4.0 with the LTV-QP's own
# unrelated, lower rrate_steer_straight/_corner endpoints (2.0/1.25) the
# moment it was enabled.
# CAUTION: enabling this OVERWRITES R_rate[steer] outright -- it does NOT scale
# r_rate_delta. With the two endpoints below left at -1 (inherit) they resolve to
# the LTV-QP's own rrate_steer_straight/_corner (2.0/1.25), which silently
# replaced a validated r_rate_delta=52.5 with ~1.8 -- a ~30x cut. Result was
# 21% steering saturation, |e_y| 1.18 m, |e_psi| 12.1 deg. If re-enabling, set
# BOTH endpoints explicitly, scaled to the current r_rate_delta (e.g. 52.5
# straight / ~25 corner), never left at -1.
#
# NOW ENABLED, to attack the late/jerky shallow-corner turn-in that the flat
# r_rate_delta=52.5 introduced (see
# fsae_MPCTest/docs/steering_turn_in_upgrade_options.md, Option 2). Goal:
# keep ~50 on straights so the chatter fix survives, but soften through
# corners so the solver stops deferring turn-in until it has to catch up at
# the actuator slew limit.
#
# WHY corner_factor_k is ALSO overridden (20.0, up from the LTV-QP's 8.0):
# corner_frac = 1 - 1/(1 + k*|kappa|) never exceeded 0.63 on this track at
# k=8, so the blend could only ever travel ~2/3 of the way to its corner
# endpoint -- even a corner endpoint of 2.0 would still leave ~32 where the
# jerks occur. Raising k makes the curve actually reach: at k=20 a 12 m-radius
# corner (the jerk-prone zone) gives corner_frac 0.63 rather than 0.40, while
# a 250 m near-straight stays at 0.07. Resulting schedule:
#   near-straight (R~250m) -> ~49     R=12m (jerk zone) -> ~25
#   R=18m                  -> ~34     R=4.7m (sharpest) -> ~16
# If chatter returns on straights, raise NMPC_RRATE_STEER_CORNER first (not
# k) -- k controls WHERE the softening applies, the endpoint controls HOW MUCH.
NMPC_CORNER_RRATE_BLEND_ENABLED=false      # [NMPC only, EXPERIMENTAL] blends R_rate[steer] between NMPC_RRATE_STEER_STRAIGHT/_CORNER by CURRENT curvature (mpc_core._corner_factor/_blend). Mutually exclusive with NMPC_STEER_RATE_ANTI_HUNT_ENABLED above -- blend takes priority if both are set.
# Saturation rate of _corner_factor = 1 - 1/(1 + k*|kappa|), shared by the
# corner blend above and NMPC_RRATE_ZONE_* below. The inherited default (8.0)
# is calibrated for the LTV-QP's soft Q-blending, where partial engagement is
# fine; the zone schedule instead NEEDS this to saturate, because its corner
# floor is only reached as corner_frac -> 1.
#
# At k=8 that never happens on this track: corner_frac needs |kappa|=1.125 to
# reach 0.9, but comp_test_map_3's tightest corner is 0.209, so corner_frac
# tops out at 0.626 and the zone multiplier bottoms at 0.84 instead of its
# 0.15 floor -- i.e. the ease/floor bands are unreachable and the "zone"
# degenerates into a mild global rate boost. Measured live: m_Rrate_zone
# ranged 0.829-1.962 with 0% of ticks in either the ease or floor band.
#
# Sizing rule: k ~= target_corner_frac/((1-target_corner_frac)*kappa_max),
# which puts |kappa|=0.209 (this track's tightest) at corner_frac 0.85 for
# k=27 and 0.93 for k=60.
#
# CAUTION: raising k beyond this does NOT fix mid-corner lateral drift, and
# k=60 was measured live to be worse (score 0.497 vs 0.454, flip% 39.0 vs
# 36.0, 34 drift episodes vs 29). It does lower the corner weight as
# intended -- effective r_rate in the drift zones went 38.7 -> 29.2 -- but
# total drift growth moved only 10.86 -> 10.45 m, with steering peaking at
# 12-15 deg of an available 25. A 25% weight cut that buys a 4% drift change
# means the steering-RATE cost is not what limits turn-in here; look at the
# error/effort weights (q_e_y vs r_delta) or the reference heading instead.
NMPC_CORNER_FACTOR_K=27.0
# NMPC_RRATE_STEER_STRAIGHT=52.5             # [NMPC only] MUST be set explicitly, never -1: at -1 it inherits the LTV-QP's 2.0 and silently discards r_rate_delta (see CAUTION above)
# NMPC_RRATE_STEER_CORNER=8.0                # [NMPC only] MUST be set explicitly, never -1 (inherits the LTV-QP's 1.25)

# [NMPC only, EXPERIMENTAL, default off] Per-stage ramp on the steering-RATE
# cost: NMPC_RRATE_STAGE_NEAR at horizon stage 0 rising to 1.0 at the last
# stage, so a first turn-in input is cheap while a sustained oscillation
# still pays close to full price. Keyed on horizon POSITION, not measured
# state -- unlike the corner blend above, which is unreachable for ~27% of
# the jerk events (no curvature/error signal 1 s beforehand).
#
# OFFLINE-REJECTED as a fix for the shallow-corner jerk: it moved
# slew-limited ticks the WRONG way (8.4% -> 12-15%) because a cheaper
# near-stage rate simply spends more of the slew budget every tick, and
# chatter rose with it. Kept because it is the ONLY change found so far that
# clears the offline nmpc_offline_check DNF (452 ticks -> full lap) and it
# improves |e_y| (0.497 -> 0.428). Enable only for that purpose, or to
# re-test live where offline has mispredicted this stack four times.
# See fsae_MPCTest/docs/steering_turn_in_upgrade_options.md (Option 1).
# NMPC_RRATE_STAGE_RAMP_ENABLED=false
# NMPC_RRATE_STAGE_NEAR=0.30                 # stage-0 multiplier; 1.0 = exact no-op

# [NMPC only, EXPERIMENTAL, default off] Continuous three-zone schedule on the
# steering-RATE cost: BOOST on a true straight, EASE on the approach to a
# corner the HORIZON can already see, FLOOR through the corner itself. Smooth
# surface (no thresholds/hysteresis) -- on a continuously-winding road `now`
# and `ahead` are both high so it just sits at the corner value.
# MULTIPLIES r_rate_delta, so it composes with the shipped 52.5 instead of
# discarding it (the trap NMPC_CORNER_RRATE_BLEND_ENABLED falls into).
# Offline at 2.0/0.35/0.15: slew-limited ticks 7.80% -> 4.95% AND chatter
# 2.825 -> 2.337 deg/tick, |e_y| roughly level. First mechanism to improve
# both at once. ENABLED for live A/B against the centerline.csv baseline
# (score 0.488) -- composes multiplicatively with NMPC_RJERK_DELTA below,
# which is already on, so a regression could be either one; disable this
# first, not the jerk term.
NMPC_RRATE_ZONE_ENABLED=true
NMPC_RRATE_ZONE_BOOST_STRAIGHT=2.0    # x r_rate on a true straight
# 0.35 DNFs in the offline sim at k=27 (off-track at the track's tightest
# corner) and so does every value below ~0.7, INCLUDING settings that make
# the zone uniformly weaker than no zone at all -- so this is not a simple
# "too much release" effect and is not yet explained. Until it is, keep this
# at a value the offline rollout completes; see docs/tuning.md's
# "Three-zone rate schedule".
NMPC_RRATE_ZONE_EASE_APPROACH=0.80    # x r_rate when a corner is AHEAD -- the turn-in release
NMPC_RRATE_ZONE_FLOOR_CORNER=0.15     # x r_rate mid-corner
#
# [NMPC only, EXPERIMENTAL, default off] Steering-JERK weight: penalises the
# SECOND difference of steering (steering ACCELERATION) instead of only the
# first. A steady ramp into a corner scores near zero and is nearly free; an
# alternating wiggle is expensive. Offline this is the strongest result of
# anything tried: at rjerk=150 with r_rate 52.5, slew 7.80% -> 2.77% and
# chatter 2.825 -> 1.686.
#
# On centerline.csv this holds 0 saturated ticks, 0 slew-limited ticks and 1
# steering reversal over 3 laps. CAUTION when judging it on a raceline
# reference instead: the same weight there shows ~4.5% saturation, which
# belongs to the reference and not to this term -- see
# docs/reference/'s "Reference line: raceline vs centreline" before
# attributing a saturation figure to this weight.
#
# Untested live: the low-r_rate variant (MPC_R_RATE_DELTA=5.0 with
# NMPC_RJERK_DELTA=250.0), which beats the flat-52.5 baseline on every
# offline metric. Set both together.
NMPC_RJERK_DELTA=150.0
# NMPC_RJERK_A=0.0

# [NMPC only, EXPERIMENTAL] Soft (slack-backed) per-stage
# speed-limit constraint: v_x_k <= v_ref_at(s_k) + NMPC_SPEED_LIMIT_MARGIN +
# slack_v_k at every horizon stage, own slack variable/weight (separate from
# the track-boundary slack, never shared). Replaces the EARLIER, REJECTED
# NMPC_HORIZON_SPEED_PROFILE_ENABLED approach, which put the per-stage speed
# target into the QP's summed COST instead of a constraint -- a plain sum of
# squared residuals lets the solver trade a bad (too-fast) residual at an
# early/in-corner stage against a good residual at a later/post-corner stage
# in the SAME solve, so it never actually had to brake in time (live-tested:
# v_actual ~16.7 m/s against v_ref ~3-5 m/s approaching a corner). A per-stage
# INEQUALITY can't be traded away that way -- it must hold at every stage
# individually. Requires a speed-profile array to actually be supplied
# (path_map_path mode); a no-op otherwise, same gating as
# NMPC_HORIZON_SPEED_PROFILE_ENABLED's own ref.v_target check.
# Offline A/B (comp_test_map_3, recorded-map rollout): |e_psi| mean 4.2->3.1
# deg, steering saturation 1.7%->0.8%, ticks >0.5 m/s over target 16.6%->0.8%,
# no DNF either way. LIVE-TESTED 2026-08-19 AND REJECTED: "didn't work" --
# reverted to false. Not yet root-caused; see docs/reference/ before
# re-attempting rather than re-enabling blind.
NMPC_SPEED_LIMIT_ENABLED=false
# NMPC_SPEED_LIMIT_MARGIN=0.5                  # m/s added on top of the profile before the bound engages
# NMPC_SPEED_LIMIT_SLACK_WEIGHT=200.0          # penalty on the speed-limit slack; much lower than the track bound's 10000 on purpose, see nmpc_params.py

# [shared (LTV-QP native, NMPC via override), EXPERIMENTAL] Soft constraint
# against steering REVERSALS (tick-to-tick sign flip), approximated by
# boosting R_rate[0,0] whenever LAST tick's steering was already close to
# zero -- see mpc_core.py's _reversal_penalty_boost docstring for why a
# reversal can't be detected directly inside a convex QP and this
# approximates it. Composes multiplicatively with steer_rate_anti_hunt/the
# corner blend, does not replace them -- keyed on a different signal
# (u_prev, not curvature/e_y/e_psi), so no double-count risk.
# Offline-validated on the LTV-QP path (see docs/reference/'s
# "Soft steering-reversal penalty" section); not yet live-tested.
REVERSAL_PENALTY_ENABLED=true
# REVERSAL_PENALTY_BOOST_MAX=4.0                # ceiling multiplier, applied when previous steering == 0
# REVERSAL_PENALTY_K=8.0                        # 1/rad; half-boost at ~7.2deg of previous steering
# NMPC-only override -- set explicitly (not -1) to enable independently of
# REVERSAL_PENALTY_ENABLED above. CAUTION: offline A/B on the NMPC path was a
# net regression (reversal count barely improved while composite score
# worsened) -- the NMPC's curvature-aware reference already suppresses most
# of what this mechanism targets, so on NMPC it mostly adds rate cost the
# solver didn't need. Do not enable without first sweeping boost_max/k
# offline; the LTV-QP-tuned default (4.0/8.0) is not known good here.
# -1 = inherit the LTV-QP value/constants above.
NMPC_REVERSAL_PENALTY_ENABLED=false
# NMPC_REVERSAL_PENALTY_BOOST_MAX=-1.0
# NMPC_REVERSAL_PENALTY_K=-1.0

# MPC tuning shortlist -- optional one-off overrides for the handful of
# MPCController weights/gains most likely to be tuned interactively, without
# editing fsae_params.yaml. The FULL set of ~56 tunables (every field in
# fsae_control/mpc_params.py's MPCParams) is always available as a launch
# arg via control.launch.py/sim.launch.py; these are just a convenience
# shortlist on top, matching V_MAX/V_MIN's pattern. Left unset (commented
# out) by default so leaving this file untouched changes nothing -- uncomment
# and set a value to override just that one field for this launch.
# MPC_Q_E_Y=6.0                         # [shared] lateral-error weight
# MPC_Q_E_PSI=1.6                       # [shared] heading-error weight
# MPC_R_DELTA=1.8                       # [shared] steering-effort weight
# MPC_R_A_ACCEL=3.0                     # [shared] acceleration-effort weight, a_cmd >= 0
# MPC_R_A_BRAKE=0.5                     # [shared] acceleration-effort weight, a_cmd < 0 (braking) -- split 2026-08-12 from a single shared r_a
# MPC_ADAPTIVE_R_RATE_DURING_FLOOR=0.625   # [LTV-QP only] R_rate softening floor, mid-corner
# MPC_ADAPTIVE_R_RATE_ENTERING_FLOOR=0.85  # [LTV-QP only] R_rate softening floor, corner approach
# MPC_CORNER_FACTOR_K=8.0                   # [LTV-QP only] corner_factor curve sharpness vs CURRENT |kappa| -- replaces the deleted lookahead gain-scheduling family (2026-08-13)
# MPC_Q_EY_CORNER=9.0                       # [LTV-QP only] Q[0,0] at full corner (corner_frac=1)
# MPC_Q_EPSI_CORNER=3.0                     # [LTV-QP only] Q[2,2] at full corner (corner_frac=1)
#
# [LTV-QP only] Disable-and-compare test (late_turn_in_investigation.md Part 3f): these two
# mechanisms have ZERO positive evidence for helping anything, and are
# directly theorised to fight EARLY TURN-IN specifically (adaptive_q_scaling
# softens Q[0,0] near centreline -- exactly the state right before a good,
# early turn-in; steer_rate_anti_hunt can multiply R_rate[0,0] up to 6x when
# the car looks centred/calm, same moment). Test ONE at a time, isolate
# before stacking. Recommended order: (1) adaptive_q_scaling_enabled=false
# alone; (2) if that alone doesn't fix it, ALSO set
# steer_rate_anti_hunt_enabled=false on top; (3) compare both against
# today's un-reverted baseline (both true) on the SAME corner.
MPC_ADAPTIVE_Q_SCALING_ENABLED=false
# MPC_STEER_RATE_ANTI_HUNT_ENABLED=false

# Real-time curvature-lookahead speed cap layered under the precomputed
# speed profile (map_path) -- see control_utils.dynamic_speed_cap()'s
# docstring. Same shortlist pattern as the MPC weights above: left unset
# (commented out) by default so leaving this file untouched changes nothing.
# Set ENABLE_DYNAMIC_SPEED_CAP=false to disable outright for an A/B run.
ENABLE_DYNAMIC_SPEED_CAP=false
# DYNAMIC_CAP_A_LAT_MAX=3.2                # m/s^2 -- lateral-accel budget, dynamic cap only
# DYNAMIC_CAP_SAFETY=0.9                   # safety margin, dynamic cap only

# Maps this file's MPC_<FIELD> shell variables onto the matching
# fsae_control.mpc_params.MPCParams field name (lowercase) and appends
# name:=value to MPC_LAUNCH_ARGS -- only for whichever of the shortlist
# above is actually uncommented/set, so an untouched shortlist appends
# nothing and every default stays exactly what fsae_params.yaml already has.
MPC_LAUNCH_ARGS=""
_append_mpc_arg() {
    local field="$1" value="$2"
    if [ -n "$value" ]; then
        MPC_LAUNCH_ARGS="$MPC_LAUNCH_ARGS $field:=$value"
    fi
}
_append_mpc_arg q_e_y "$MPC_Q_E_Y"
_append_mpc_arg q_e_psi "$MPC_Q_E_PSI"
_append_mpc_arg r_delta "$MPC_R_DELTA"
_append_mpc_arg r_a_accel "$MPC_R_A_ACCEL"
_append_mpc_arg r_a_brake "$MPC_R_A_BRAKE"
_append_mpc_arg adaptive_r_rate_during_floor "$MPC_ADAPTIVE_R_RATE_DURING_FLOOR"
_append_mpc_arg adaptive_r_rate_entering_floor "$MPC_ADAPTIVE_R_RATE_ENTERING_FLOOR"
_append_mpc_arg corner_factor_k "$MPC_CORNER_FACTOR_K"
_append_mpc_arg q_ey_corner "$MPC_Q_EY_CORNER"
_append_mpc_arg q_epsi_corner "$MPC_Q_EPSI_CORNER"
_append_mpc_arg adaptive_q_scaling_enabled "$MPC_ADAPTIVE_Q_SCALING_ENABLED"
_append_mpc_arg steer_rate_anti_hunt_enabled "$MPC_STEER_RATE_ANTI_HUNT_ENABLED"
_append_mpc_arg standalone_output "$STANDALONE_OUTPUT"
_append_mpc_arg use_nmpc "$USE_NMPC"
_append_mpc_arg nmpc_horizon "$NMPC_HORIZON"
_append_mpc_arg nmpc_sqp_iters "$NMPC_SQP_ITERS"
_append_mpc_arg nmpc_solve_budget_ms "$NMPC_SOLVE_BUDGET_MS"
_append_mpc_arg nmpc_q_e_y "$NMPC_Q_E_Y"
_append_mpc_arg nmpc_q_e_psi "$NMPC_Q_E_PSI"
_append_mpc_arg nmpc_q_epsi_dot "$NMPC_Q_EPSI_DOT"
_append_mpc_arg nmpc_r_delta "$NMPC_R_DELTA"
_append_mpc_arg nmpc_r_rate_delta "$NMPC_R_RATE_DELTA"
_append_mpc_arg nmpc_alat_ceiling_enabled "$NMPC_ALAT_CEILING_ENABLED"
_append_mpc_arg nmpc_spline_reference_enabled "$NMPC_SPLINE_REFERENCE_ENABLED"
_append_mpc_arg nmpc_horizon_speed_profile_enabled "$NMPC_HORIZON_SPEED_PROFILE_ENABLED"
_append_mpc_arg nmpc_friction_circle_enabled "$NMPC_FRICTION_CIRCLE_ENABLED"
_append_mpc_arg nmpc_steer_rate_anti_hunt_enabled "$NMPC_STEER_RATE_ANTI_HUNT_ENABLED"
_append_mpc_arg nmpc_anti_hunt_boost_max "$NMPC_ANTI_HUNT_BOOST_MAX"
_append_mpc_arg nmpc_corner_rrate_blend_enabled "$NMPC_CORNER_RRATE_BLEND_ENABLED"
_append_mpc_arg nmpc_corner_factor_k "$NMPC_CORNER_FACTOR_K"
_append_mpc_arg nmpc_rrate_steer_straight "$NMPC_RRATE_STEER_STRAIGHT"
_append_mpc_arg nmpc_rrate_steer_corner "$NMPC_RRATE_STEER_CORNER"
_append_mpc_arg nmpc_rrate_stage_ramp_enabled "$NMPC_RRATE_STAGE_RAMP_ENABLED"
_append_mpc_arg nmpc_rrate_stage_near "$NMPC_RRATE_STAGE_NEAR"
_append_mpc_arg nmpc_rrate_zone_enabled "$NMPC_RRATE_ZONE_ENABLED"
_append_mpc_arg nmpc_rrate_zone_boost_straight "$NMPC_RRATE_ZONE_BOOST_STRAIGHT"
_append_mpc_arg nmpc_rrate_zone_ease_approach "$NMPC_RRATE_ZONE_EASE_APPROACH"
_append_mpc_arg nmpc_rrate_zone_floor_corner "$NMPC_RRATE_ZONE_FLOOR_CORNER"
_append_mpc_arg nmpc_rjerk_delta "$NMPC_RJERK_DELTA"
_append_mpc_arg nmpc_rjerk_a "$NMPC_RJERK_A"
_append_mpc_arg nmpc_speed_limit_enabled "$NMPC_SPEED_LIMIT_ENABLED"
_append_mpc_arg nmpc_speed_limit_margin "$NMPC_SPEED_LIMIT_MARGIN"
_append_mpc_arg nmpc_speed_limit_slack_weight "$NMPC_SPEED_LIMIT_SLACK_WEIGHT"
_append_mpc_arg reversal_penalty_enabled "$REVERSAL_PENALTY_ENABLED"
_append_mpc_arg reversal_penalty_boost_max "$REVERSAL_PENALTY_BOOST_MAX"
_append_mpc_arg reversal_penalty_k "$REVERSAL_PENALTY_K"
_append_mpc_arg nmpc_reversal_penalty_enabled "$NMPC_REVERSAL_PENALTY_ENABLED"
_append_mpc_arg nmpc_reversal_penalty_boost_max "$NMPC_REVERSAL_PENALTY_BOOST_MAX"
_append_mpc_arg nmpc_reversal_penalty_k "$NMPC_REVERSAL_PENALTY_K"
_append_mpc_arg enable_dynamic_speed_cap "$ENABLE_DYNAMIC_SPEED_CAP"
_append_mpc_arg dynamic_cap_a_lat_max "$DYNAMIC_CAP_A_LAT_MAX"
_append_mpc_arg dynamic_cap_safety "$DYNAMIC_CAP_SAFETY"

# Use the host's native ROS 2 install when available; otherwise fall back to Docker.
if command -v ros2 >/dev/null 2>&1 && [ -f "$HOST_ROS2_DIR/install/local_setup.bash" ]; then
    USE_DOCKER=false
else
    USE_DOCKER=true
fi

# Under WSL2 (NAT networking), 127.0.0.1/localhost inside WSL does NOT reach
# the Windows host running FSDS — WSL has its own network namespace. The
# Windows host is reachable via WSL's default gateway instead. This affects
# both this script's own RPC-readiness check below AND fsds_ros2_bridge's
# connection to AirSim (fsds_ros2_bridge.launch.py already supports this via
# the FSDS_HOST_IP env var, but expects it to be set externally — it defaults
# to 'localhost' otherwise, which fails the same way). Computed once here and
# exported so both consumers agree, without requiring a manual `export` step.
# Skipped for the Docker path, where the container's own networking applies.
if [ "$USE_DOCKER" != true ] && [ -z "$FSDS_HOST_IP" ]; then
    export FSDS_HOST_IP="$(ip route show default 2>/dev/null | awk '{print $3; exit}')"
fi

cleanup() {
    echo ""
    echo "============================================="
    echo "🛑 Caught termination signal! Cleaning up..."
    echo "============================================="

    # 1. Terminate the background ROS 2 bridge process
    if [ ! -z "$BRIDGE_PID" ]; then
        echo "Stopping background ROS 2 Bridge (PID: $BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null
    fi

    # TEMPORARY (2026-08-19): tear down the topic-hz diagnostic captures —
    # see the matching TEMPORARY block in section [2/3] above.
    if [ ! -z "$HZ_ODOM_PID" ]; then
        kill "$HZ_ODOM_PID" 2>/dev/null
    fi
    if [ ! -z "$HZ_POSE_PID" ]; then
        kill "$HZ_POSE_PID" 2>/dev/null
    fi
    if [ ! -z "$HZ_CLOCK_PID" ]; then
        kill "$HZ_CLOCK_PID" 2>/dev/null
    fi
    if [ ! -z "$HZ_CLOCK_HZ_PID" ]; then
        kill "$HZ_CLOCK_HZ_PID" 2>/dev/null
    fi
    if [ ! -z "$HZ_IMU_PID" ]; then
        kill "$HZ_IMU_PID" 2>/dev/null
    fi

    # 2. Forcefully terminate the Windows visual simulator trees via taskkill
    echo "Forcefully terminating Windows FSDS window instances..."
    taskkill.exe /F /T /IM "FSDS.exe" 2>/dev/null
    taskkill.exe /F /T /IM "FSOnline.exe" 2>/dev/null
    taskkill.exe /F /T /IM "Blocks.exe" 2>/dev/null

    # 3. Clean up core dump files generated in the ROS 2 directory
    if [ "$USE_DOCKER" = true ]; then
        if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" = "true" ]; then
            echo "🧹 Sweeping up any generated core dump files inside the container..."
            docker exec "$CONTAINER_NAME" bash -c "find $CONTAINER_ROS2_DIR -maxdepth 1 -type f -name 'core.[0-9]*' -delete" 2>/dev/null
            echo "✅ Core dumps cleared."
        else
            echo "⚠️ Container wasn't running; skipped core dump purge."
        fi
    else
        echo "🧹 Sweeping up any generated core dump files..."
        find "$HOST_ROS2_DIR" -maxdepth 1 -type f -name 'core.[0-9]*' -delete 2>/dev/null
        echo "✅ Core dumps cleared."
    fi

    exit 0
}

# Catch Ctrl+C (SIGINT) and termination signals explicitly
trap cleanup SIGINT SIGTERM

echo "============================================="
echo "🏎️  Launching Formula Student Driverless Stack"
echo "============================================="

if [ "$USE_DOCKER" = true ]; then
    echo "🐳 Native ROS 2 install not found on host; using Docker container $CONTAINER_NAME."

    CONTAINER_STATUS=$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)
    if [ "$CONTAINER_STATUS" != "true" ]; then
        echo "🐳 Docker container is not running. Starting $CONTAINER_NAME..."
        docker start "$CONTAINER_NAME"
        sleep 2
    else
        echo "🐳 Docker container is already running."
    fi
else
    echo "🖥️  Native ROS 2 install found on host; running without Docker."
fi

# 1. Launch Simulator in background
AIRSIM_RPC_PORT=41451
# FSDS_HOST_IP is 'localhost' when running natively on the same machine as
# ROS 2, or the WSL default-gateway IP under WSL2 (see above) — either way
# it's the address FSDS's RPC server is actually reachable at, so the
# readiness check below must probe the same address the bridge will use.
AIRSIM_RPC_HOST="${FSDS_HOST_IP:-localhost}"
AIRSIM_READY_TIMEOUT=120   # seconds — a full competition map's Vulkan/shader/
                           # level-streaming boot can take well over the old
                           # fixed 2s sleep, which raced the bridge against a
                           # simulator that hadn't opened its RPC port yet
                           # ("Failed connecting to RPC server (airsim)").

wait_for_airsim_rpc() {
    echo "⏳ Waiting for FSDS AirSim RPC server on $AIRSIM_RPC_HOST:$AIRSIM_RPC_PORT..."
    local waited=0
    while ! (exec 3<>"/dev/tcp/$AIRSIM_RPC_HOST/$AIRSIM_RPC_PORT") 2>/dev/null; do
        exec 3>&- 2>/dev/null
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge "$AIRSIM_READY_TIMEOUT" ]; then
            echo "⚠️ Timed out after ${AIRSIM_READY_TIMEOUT}s waiting for AirSim RPC — proceeding anyway."
            return 1
        fi
    done
    exec 3>&- 2>/dev/null
    echo "✅ AirSim RPC is up after ${waited}s."
    return 0
}

if [ -d "/mnt/c/Users/marti/Downloads/fsds-v2.2.0-windows" ]; then
    echo "[1/3] Spinning up Windows Simulator within its home directory..."
    cmd.exe /c "cd /d C:\Users\marti\Downloads\fsds-v2.2.0-windows && FSDS.exe -windowed -ResX=600 -ResY=500" &
    wait_for_airsim_rpc
else
    echo "⚠️ Warning: Windows Simulator folder path not found!"
fi

# 2. Rebuild with --symlink-install so edits to src/ take effect immediately,
# without a separate `colcon build` step. Plain `colcon build` COPIES Python
# files into install/ at build time, so an edit to src/ after the last build
# is silently invisible to `ros2 launch` until rebuilt — this bit twice in
# one session (S49: a stale v_max clip; a stale Q_diag[4] weight straight
# after). --symlink-install replaces the copy with a symlink for supported
# files (this workspace's packages are all pure Python + ament_index
# resources, so every affected file qualifies), so src/ IS the running code.
# Safe to run every launch: colcon no-ops packages that are already built
# and up to date.
echo "[1.5/3] Building workspace (--symlink-install)..."
if [ "$USE_DOCKER" = true ]; then
    docker exec "$CONTAINER_NAME" bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd $CONTAINER_ROS2_DIR && \
        colcon build --symlink-install
    "
else
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        colcon build --symlink-install
    "
fi

# 2. Launch ROS 2 Bridge in background
echo "[2/3] Initializing fsds_ros2_bridge..."
# TEMPORARY (2026-08-09): RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG + output
# redirect to capture PrintStatistics()'s getCarState/odom_pub msgs/s
# (airsim_ros_wrapper.cpp statistics_timer_cb, printed every 1s via
# RCLCPP_DEBUG, normally silent) — checking whether AirSim's odom dedup
# (equalsMessage() in odom_cb) is starving /fsae/slam/car_position during the
# pose_age_s spikes seen in mpc_standalone_control_*.csv. This is an env var,
# not a CLI flag, since `ros2 launch` (unlike `ros2 run`) has no --log-level/
# --ros-args passthrough. Revert (remove the export and the tee) once
# confirmed either way.
if [ "$USE_DOCKER" = true ]; then
    docker exec "$CONTAINER_NAME" bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd $CONTAINER_ROS2_DIR && \
        source install/local_setup.bash && \
        export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG && \
        ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py 2>&1 | tee /tmp/bridge_debug.log
    " &
else
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG && \
        ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py 2>&1 | tee /tmp/bridge_debug.log
    " &
fi
BRIDGE_PID=$!
sleep 2

# TEMPORARY (2026-08-19): capturing /fsds/testing_only/odom (250 Hz, bridge
# output), /fsae/slam/car_position (20 Hz, sim_perception.py output)
# publish-rate jitter, /clock's drift against wall time, and (added
# 2026-08-20) /clock's own arrival rate, to localise the periodic (~31.7 s)
# car_x/car_y teleport documented in
# fsae_MPCTest/docs/logs/periodic_pose_teleport_investigation.md. A prior
# capture already showed /fsae/slam/car_position perfectly clean and
# /fsds/testing_only/odom sustaining only ~25-36 Hz instead of ~250 Hz for
# most of the run; the clock-drift check then showed /clock's VALUE does not
# fall behind wall time (FSDS/Unreal's own simulation is not stalling) -- the
# /clock arrival-rate capture narrows further: if it stays clean at ~100 Hz
# while odom collapses, that pins the bottleneck specifically to
# getCarState()'s RPC path, not RPC/AirSim generally. Remove this block (and
# its `kill`s in cleanup() below, and ros2/clock_drift_check.py) once that's
# answered.
HZ_LOG_DIR="$HOST_REPO_ROOT/fsae_logs/topic_hz_diagnostics"
mkdir -p "$HZ_LOG_DIR"
HZ_STAMP="$(date +%s)"
if [ "$USE_DOCKER" != true ]; then
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        ros2 topic hz -w 5 /fsds/testing_only/odom
    " > "$HZ_LOG_DIR/odom_hz_${HZ_STAMP}.log" 2>&1 &
    HZ_ODOM_PID=$!
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        ros2 topic hz -w 5 /fsae/slam/car_position
    " > "$HZ_LOG_DIR/car_position_hz_${HZ_STAMP}.log" 2>&1 &
    HZ_POSE_PID=$!
    # /clock (100 Hz, bridge output, sourced from FSDS's OWN internal sim
    # clock via a GSS RPC read -- see clock_timer_cb in
    # airsim_ros_wrapper.cpp) drifting behind wall time, rather than just
    # arriving late, would confirm FSDS/Unreal's own simulation is the
    # bottleneck (not a ROS2/network delivery issue) -- see
    # clock_drift_check.py's own docstring.
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        python3 '$HOST_ROS2_DIR/clock_drift_check.py' '$HZ_LOG_DIR/clock_drift_${HZ_STAMP}.csv'
    " > "$HZ_LOG_DIR/clock_drift_${HZ_STAMP}.log" 2>&1 &
    HZ_CLOCK_PID=$!
    # /clock arrival-rate (as opposed to its VALUE, already covered by
    # clock_drift_check.py above): if this stays clean at ~100 Hz while odom
    # collapses to ~25-36 Hz, that pins the bottleneck specifically to
    # getCarState()'s RPC path rather than RPC/AirSim generally -- see the
    # "Next step" section of periodic_pose_teleport_investigation.md.
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        ros2 topic hz -w 5 /clock
    " > "$HZ_LOG_DIR/clock_hz_${HZ_STAMP}.log" 2>&1 &
    HZ_CLOCK_HZ_PID=$!
    # /fsds/imu (getImuData() RPC, imu_timer_cb in airsim_ros_wrapper.cpp) --
    # a third, genuinely distinct RPC call from both getCarState() (odom) and
    # getGroundSpeedSensorData() (backs BOTH /clock and /fsds/gss, so gss
    # would NOT be an independent test). The third capture already found
    # /clock stalling in lockstep with odom on the same ~33-34s cadence,
    # overturning "specific to getCarState()" -- if /fsds/imu ALSO stalls on
    # that cadence, that confirms one shared RPC/AirSim bottleneck rather
    # than two calls coincidentally stalling together. See the "Next step"
    # section of periodic_pose_teleport_investigation.md.
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        ros2 topic hz -w 5 /fsds/imu
    " > "$HZ_LOG_DIR/imu_hz_${HZ_STAMP}.log" 2>&1 &
    HZ_IMU_PID=$!
    echo "      (temporary) logging topic-hz + clock-drift diagnostics to $HZ_LOG_DIR"
fi

# 3. Launch Planning Stack in the foreground
# sim.launch.py defaults to controller:=mpc, standalone_output:=true, and
# record_cones:=true, so cone recording starts automatically alongside the
# stack (no separate terminal needed). Controller is set via CONTROLLER
# above; override record_cones:=false directly on the line below if ever
# needed.
#
# The precomputed-speed/path toggles are set via USE_PRECOMPUTED_SPEED /
# USE_PRECOMPUTED_PATH above, and WHICH track's CSVs they read via TRACK.
# Full workflow: fsae_MPCTest/docs/developer_guide.md, "Recording, exporting
# and driving a track".
#
# cone_out_path sends a fresh recording into fsae_planning's own
# tracks/<TRACK>/cone_map.json, the same directory the exporters read from --
# so a re-record of the current track is picked up by
# `python3 -m tuner.export_speed_profile $TRACK` (run from fsae_MPCTest, then
# copy the result back into ros2/src/fsae_planning/tracks/$TRACK/) with no
# extra file shuffling. Recording a NEW track: point TRACK at the new name
# first, and set both precomputed toggles false so the car drives off the
# live planner instead of replaying the old line. The guard above is skipped
# in that case precisely because the CSVs don't exist yet.
#
# If TRACK_DIR does not exist yet, this IS a brand-new recording, and the
# directory actually created is dated -- "<TRACK>_<YYYYmmdd>" -- matching
# fsae_MPCTest/tracks/dated_track_name() exactly, so two recordings under the
# same TRACK= on different days never collide. A RE-record of an existing
# track (TRACK_DIR already there) is NOT dated again -- it keeps refreshing
# the same directory, which is the documented "refresh a cone map in place"
# workflow above; only first creation gets a date.
if [ ! -d "$TRACK_DIR" ]; then
    TRACK="${TRACK}_$(date +%Y%m%d)"
    TRACK_DIR="$HOST_ROS2_DIR/src/fsae_planning/tracks/$TRACK"
    # Re-derive SPEED_CSV/PATH_CSV too (computed earlier, from the
    # pre-date TRACK_DIR) so they stay consistent with the directory this
    # recording actually lands in -- in case the operator left
    # USE_PRECOMPUTED_SPEED/_PATH at true by mistake while recording a new
    # track (the documented workflow above says set both false; this is the
    # fallback for when that is not done, not the intended path).
    SPEED_CSV="$TRACK_DIR/speed_profile.csv"
    PATH_CSV="$TRACK_DIR/$(_track_geometry_name "$TRACK_DIR")"
    echo "      new track — recording into: $TRACK_DIR"
fi
echo "[3/3] Launching Autonomous Stack (Perception, Planner, Control, Cone Recorder)..."
echo "      track: $TRACK  (precomputed speed=$USE_PRECOMPUTED_SPEED path=$USE_PRECOMPUTED_PATH heading_profile=$USE_PRECOMPUTED_HEADING_PROFILE)"
echo "      controller: $CONTROLLER$( [ "$CONTROLLER" != stanley ] && echo "  [$( [ "$USE_NMPC" = true ] && echo 'NMPC (nonlinear, nmpc_core.py)' || echo 'LTV-QP (mpc_core.py)' )]" )"
mkdir -p "$TRACK_DIR"
if [ "$USE_DOCKER" = true ]; then
    # Container-side paths: the container mounts the repo at a different
    # root, so TRACK_DIR (a host path) cannot be reused verbatim here.
    CONTAINER_TRACK_DIR="$CONTAINER_ROS2_DIR/src/fsae_planning/tracks/$TRACK"
    docker exec -it "$CONTAINER_NAME" bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd $CONTAINER_ROS2_DIR && \
        source install/local_setup.bash && \
        ros2 launch fsae_bringup sim.launch.py controller:=$CONTROLLER cone_out_path:=$CONTAINER_TRACK_DIR/cone_map.json log_dir:=$CONTAINER_REPO_ROOT/fsae_logs map_path:=$CONTAINER_TRACK_DIR/$(basename "$SPEED_CSV") path_map_path:=$CONTAINER_TRACK_DIR/$(basename "$PATH_CSV") use_precomputed_speed:=$USE_PRECOMPUTED_SPEED use_precomputed_path:=$USE_PRECOMPUTED_PATH use_precomputed_heading_profile:=$USE_PRECOMPUTED_HEADING_PROFILE v_max:=$V_MAX v_min:=$V_MIN$MPC_LAUNCH_ARGS
    "
else
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        ros2 launch fsae_bringup sim.launch.py controller:=$CONTROLLER cone_out_path:='$TRACK_DIR/cone_map.json' log_dir:='$HOST_REPO_ROOT/fsae_logs' map_path:='$SPEED_CSV' path_map_path:='$PATH_CSV' use_precomputed_speed:=$USE_PRECOMPUTED_SPEED use_precomputed_path:=$USE_PRECOMPUTED_PATH use_precomputed_heading_profile:=$USE_PRECOMPUTED_HEADING_PROFILE v_max:=$V_MAX v_min:=$V_MIN$MPC_LAUNCH_ARGS
    "
fi

# Handle manual exit or fallback execution when foreground process drops out cleanly
cleanup
