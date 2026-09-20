#!/bin/bash
#
# One-command launcher for the full FSDS + fsae_planning driving stack:
# starts the Windows FSDS simulator, waits for its AirSim RPC server, launches
# fsds_ros2_bridge, then launches sim.launch.py in the foreground. Ctrl+C/
# SIGTERM tears everything down via cleanup().
#
# Edit this file to change what a plain `./launch_all.sh` drives (track,
# controller, speed caps, MPC tuning shortlist). Don't edit
# sim.launch.py/control.launch.py's own defaults for a one-off change.
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
# speed_profile.csv and the geometry file (centerline.csv if present, else
# raceline.csv -- see _newest_track/_track_geometry_name below).
#
# tracks/ is committed data inside fsae_planning, so a fresh clone of FSDS +
# fsae_planning alone can drive its newest track with no fsae_MPCTest
# checkout. fsae_MPCTest is only where NEW tracks get produced (recording +
# the two exporters); copy the output into fsae_planning's tracks/<name>/ to
# ship it.
#
# To see what's available:  ls "$(dirname "${BASH_SOURCE[0]}")/src/fsae_planning/tracks"
# To add a new one (requires fsae_MPCTest for the exporters): record a lap,
# run the two exporters -- see fsae_MPCTest/docs/developer_guide.md
# ("Recording, exporting and driving a track") -- then copy the resulting
# tracks/<name>/ directory into ros2/src/fsae_planning/tracks/<name>/.
#
# TRACK defaults to the MOST RECENTLY RECORDED track (by cone_map.json mtime,
# via _newest_track, a bash port of fsae_MPCTest/tracks/newest_track() kept
# in sync by hand since this script must not depend on fsae_MPCTest existing).
#
# Set TRACK= explicitly (uncomment below) to pin a specific track instead of
# always using the newest -- e.g. comparing two recordings, or holding back a
# still-being-tuned track from becoming the default.
#
# Existing tracks (ls ros2/src/fsae_planning/tracks/ to refresh this list):
# TRACK=comp_test_map_3
# TRACK=comp_test_map_2_20260916
# TRACK=acceleration_20260916
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
# SPEED comes from speed_profile.csv, GEOMETRY from centerline/raceline.csv --
# these describe different lines. Do not point both at the same CSV: doing so
# has regressed tracking badly before (RMSE and steering saturation both blow
# up). Do not raise SPEED_CSV's cap without checking the precomputed-speed
# branch in mpc_controller_standalone.py first: it applies no v_max clip, so
# the CSV's own top speed becomes the car's top speed directly, and
# speed_profile.csv is generated deliberately under the measured FSDS lateral
# ceiling (see docs/reference/reference_path_and_speed.md's "Speed-profile
# aggressiveness" section) -- raising it risks exceeding that ceiling.
SPEED_CSV="$TRACK_DIR/speed_profile.csv"
# Flat-speed test override. V_MAX does NOT reach the precomputed-speed branch
# (see above), so capping speed for a test means swapping this CSV, not
# setting V_MAX. A flat 3 m/s profile for comp_test_map_3 is already exported
# at speed_profile_corner_test.csv; regenerate for another speed/track with:
#   python3 -m tuner.tools.export_speed_profile <track> \
#       --corner-slowdown 0 --corner-speed <m/s>
# (threshold 0 puts every point in the "corner" branch, clamping the whole lap flat).
# SPEED_CSV="$TRACK_DIR/speed_profile_corner_test.csv"
# GEOMETRY source. Defaults to the newest export within $TRACK, preferring
# centerline.csv over raceline.csv when both exist (see _track_geometry_name
# above). Drive the centreline by default, not the raceline: on raceline.csv
# a large logged |e_y| is ambiguous (the line intentionally sits near a
# boundary at an apex, so it can mean either a tracking failure or the line
# doing its job); on the centreline |e_y| is unambiguously distance from the
# middle of the track. Set PATH_CSV explicitly (e.g. "$TRACK_DIR/raceline.csv")
# for a timed run instead.
# Left empty, not hard-errored, when the track has no geometry export yet
# (a brand-new track about to be recorded, where USE_PRECOMPUTED_PATH=false
# means this is never read); the guard below only checks PATH_CSV's
# existence when USE_PRECOMPUTED_PATH=true actually needs it.
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
USE_PRECOMPUTED_SPEED=false
USE_PRECOMPUTED_PATH=false
# Use raceline_optimizer.py's shaped psi_target column (heading-lead
# reference, see late_turn_in_investigation.md Part 8/9/10/12) in place of
# the geometric path tangent for e_psi's reference. Only has an effect
# when USE_PRECOMPUTED_PATH=true. Keep this false on tracks with few true
# straights: the lead stays active through the whole corner rather than
# just the approach, fighting the corner's own geometry instead of helping
# commit to it early. See late_turn_in_investigation.md Part 12 before
# re-enabling.
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
# late_turn_in_investigation.md Parts 1-15 was working around. Offline- and
# live-validated; see docs/reference/control_mechanisms.md's "Nonlinear MPC"
# section for the full A/B.
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
# NMPC_Q_E_Y/_Q_E_PSI/_Q_EPSI_DOT/_R_DELTA/_R_RATE_DELTA forward to fields on
# MPCParams itself (see mpc_params.py's "NMPC weight overrides" section),
# alongside every other MPC weight in the shortlist further down this file.
# Kept listed here too since they're the ones most likely to be tuned
# specifically for the NMPC.
# NMPC_HORIZON=20                     # [NMPC only] steps (x dt=0.05). Do not raise past ~20 (1.0s) without re-checking nmpc_params.py's sweep table; longer horizons measured worse
# First-order low-pass on the incoming speed target before the NMPC's cost
# function sees it. Do not jump this by a large step; large jumps have caused
# regressions before. Now the dataclass default in nmpc_params.py
# (nmpc_v_des_filter_alpha) -- no override needed here.
# Gauss-Newton iterations per tick. FALSIFIED as a fix for single-tick
# steering spikes in tight corners: ticks pinned at the slew limit are flat
# across iteration counts, extra iterations only trim mid-range activity
# while degrading |e_y| and score, and solve time can exceed the 50ms tick.
# Do not raise this to chase steering spikes; the disturbance is upstream of
# the solver iteration count.
# NMPC_SQP_ITERS=1
# NMPC_SOLVE_BUDGET_MS=25.0            # ms per solve; watch solve_ms/nmpc_iters in the log if raising -- a too-low budget silently truncates to 1 iteration
# RK4 substep counts -- see nmpc_params.py's own field docstrings for the
# full root-cause writeup. Do not lower nmpc_jac_substeps below the
# NMPC_JAC_GATE_SPEED-gated default without re-testing: a live A/B confirmed
# jac=1 above ~7 m/s causes real low-frequency steering wobble (solve time
# roughly doubles, forcing stale warm-started commands that later
# snap-correct).
# NMPC_RK_SUBSTEPS=4
# NMPC_JAC_SUBSTEPS=4
# Speed-gates nmpc_jac_substeps only (never the rollout's nmpc_rk_substeps).
# Do not lower NMPC_JAC_GATE_SPEED below ~7-8 m/s: jac=1 sensitivity is
# unstable below that. This pairing (8.0 / 2) is the default; only override
# for an A/B against the ungated behaviour (NMPC_JAC_SUBSTEPS_FAST=4).
# NMPC_JAC_GATE_SPEED=8.0
# NMPC_JAC_SUBSTEPS_FAST=2
# Same gating technique applied to the rollout's own substep count, checked
# per predicted stage. Its instability band is narrower than the Jacobian's
# (~3.75 m/s vs ~8 m/s), so this gate opens earlier. Do not use
# NMPC_RK_SUBSTEPS_FAST=2: confirmed unstable (up to ~260x perturbation
# growth) in the 2.25-3.75 m/s band; 3 is the floor. Default (4.0 / 3),
# commented out here.
# NMPC_RK_GATE_SPEED=4.0
# NMPC_RK_SUBSTEPS_FAST=3
# Standstill steering damping. At v_x=0 steering cannot move the car, but the
# SQP minimises one cost over the whole horizon, so the optimiser pre-commits
# steer toward what helps later, physically-active stages -- the car launches
# already turned. This scales stage-0's steering-effort weight only, while
# measured speed is below NMPC_STANDSTILL_SPEED. Do not raise
# NMPC_STANDSTILL_STEER_R_SCALE far past 200: values that stiff make stage 0
# a de-facto hard constraint and the solver can fight itself at the
# NMPC_STANDSTILL_SPEED crossing. Now the default (true / 0.5 / 3.0 / 200.0),
# commented out here; override only for an A/B against no damping
# (NMPC_STANDSTILL_STEER_DAMP_ENABLED=false).
# NMPC_STANDSTILL_STEER_DAMP_ENABLED=true
# NMPC_STANDSTILL_SPEED=0.5
# NMPC_STANDSTILL_STEER_R_SCALE=200.0
# Damping fades out linearly between NMPC_STANDSTILL_SPEED and this speed.
# Do not set this equal to NMPC_STANDSTILL_SPEED (a hard cutoff): releasing
# the damping in one tick moved the steering excursion later in time instead
# of removing it. 3.0 m/s is the default.
# NMPC_STANDSTILL_FADE_SPEED=3.0
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
# NMPC_ALAT_CEILING_ENABLED=true      # [NMPC only] models FSDS's measured sustained a_lat ceiling inside the prediction. Keep true for FSDS; disabling it let the NMPC oscillate and eventually spin offline
# NMPC_SPLINE_REFERENCE_ENABLED=true         # [NMPC only] default true; analytic-spline kappa(s)/psi_ref(s) instead of moving-average+finite-difference. Numerical-quality fix, not a tuning knob -- see docs/reference/control_mechanisms.md's "Three MPCC-inspired additions"
# NMPC_FRICTION_CIRCLE_ENABLED=false         # [NMPC only, EXPERIMENTAL] hard per-axle tyre-force bound, additional to the soft alat-ceiling saturation. REJECTED live with no offline A/B first ("pretty much doesn't work anymore"). Do not re-enable without an offline A/B (tuner.nmpc_offline_check) first.
# NMPC_PROGRESS_ENABLED=false                # [NMPC only, EXPERIMENTAL] NMPC picks its own speed from an arc-length progress reward; desired_speed becomes a one-sided CAP instead of a two-sided target. OFFLINE ONLY: best progress-mode score 0.892 vs the tracking baseline's 0.757 (lower is better), so it does NOT yet beat what it would replace. Needs NMPC_SLACK_LINEAR_WEIGHT>0 or it goes off-track. See fsae_MPCTest/docs/logs/nmpc_progress_term_investigation.md before enabling.
# NMPC_Q_PROGRESS=5.0                        # [NMPC only] progress-reward weight; only read when NMPC_PROGRESS_ENABLED=true. NARROW usable band at r_a_accel=1.0: below ~5 the car never breaks static friction and never launches, above ~6 it carries too much speed into corners and goes off-track
# NMPC_SLACK_LINEAR_WEIGHT=0.0               # [NMPC only] linear track-boundary slack penalty, additional to the quadratic NMPC_SLACK_WEIGHT. 0 = no-op. Effectively REQUIRED alongside NMPC_PROGRESS_ENABLED: a purely quadratic penalty has zero gradient at zero violation, and the progress reward will exploit that (measured: 1000.0 turns a q_progress=5 off-track DNF into a completed lap)
# NMPC_STEER_RATE_ANTI_HUNT_ENABLED=false    # [NMPC only, EXPERIMENTAL] reuses the LTV-QP's steer_rate_anti_hunt penalty on the NMPC, independent of MPC_STEER_RATE_ANTI_HUNT_ENABLED above. Mutually exclusive with NMPC_CORNER_RRATE_BLEND_ENABLED below -- blend takes priority if both are set. Not yet live-tested; offline A/B first.
# NMPC_ANTI_HUNT_BOOST_MAX=-1.0               # [NMPC only] -1 = inherit anti_hunt_boost_max; only read when NMPC_STEER_RATE_ANTI_HUNT_ENABLED=true
# CAUTION: enabling NMPC_CORNER_RRATE_BLEND_ENABLED below OVERWRITES
# R_rate[steer] outright, it does NOT scale r_rate_delta. If the two endpoints
# are left at -1 (inherit) they resolve to the LTV-QP's own, much lower
# rrate_steer_straight/_corner values -- silently discarding whatever
# r_rate_delta is set to and causing steering saturation. Always set BOTH
# NMPC_RRATE_STEER_STRAIGHT/_CORNER explicitly, scaled to the current
# r_rate_delta, never left at -1.
NMPC_CORNER_RRATE_BLEND_ENABLED=false      # [NMPC only, EXPERIMENTAL] blends R_rate[steer] between NMPC_RRATE_STEER_STRAIGHT/_CORNER by CURRENT curvature (mpc_core._corner_factor/_blend). Mutually exclusive with NMPC_STEER_RATE_ANTI_HUNT_ENABLED above -- blend takes priority if both are set.
# Saturation rate of _corner_factor = 1 - 1/(1 + k*|kappa|), shared by the
# corner blend above and NMPC_RRATE_ZONE_* below. The LTV-QP-inherited
# default (8.0) is too low for a schedule that needs corner_frac to actually
# reach 1 on this track's tightest corners -- the ease/floor bands become
# unreachable and the "zone" degenerates into a mild global rate boost.
# Sizing rule: k ~= target_corner_frac/((1-target_corner_frac)*kappa_max).
# Do not raise k much past 27 chasing mid-corner lateral drift: a higher k
# was measured live to be worse overall (lower corner weight, but drift
# barely improved while steering was nowhere near saturated) -- if drift
# persists, look at the error/effort weights (q_e_y vs r_delta) or the
# reference heading instead, not k.
NMPC_CORNER_FACTOR_K=27.0
# NMPC_RRATE_STEER_STRAIGHT=52.5             # [NMPC only] MUST be set explicitly, never -1: at -1 it inherits the LTV-QP's 2.0 and silently discards r_rate_delta (see CAUTION above)
# NMPC_RRATE_STEER_CORNER=8.0                # [NMPC only] MUST be set explicitly, never -1 (inherits the LTV-QP's 1.25)

# [NMPC only, EXPERIMENTAL, default off] Per-stage ramp on the steering-RATE
# cost: NMPC_RRATE_STAGE_NEAR at horizon stage 0 rising to 1.0 at the last
# stage, so a first turn-in input is cheap while a sustained oscillation
# still pays close to full price. Keyed on horizon POSITION, not measured
# state -- unlike the corner blend above, which has no curvature/error
# signal far enough ahead to catch every jerk event.
#
# Offline-rejected as a fix for shallow-corner jerk: a cheaper near-stage
# rate spends more of the slew budget every tick, moving slew-limited ticks
# the wrong way and raising chatter. Kept only because it is the one change
# found so far that clears an offline DNF; enable for that purpose, or to
# re-test live, not as a general jerk fix.
# NMPC_RRATE_STAGE_RAMP_ENABLED=false
# NMPC_RRATE_STAGE_NEAR=0.30                 # stage-0 multiplier; 1.0 = exact no-op

# [NMPC only, EXPERIMENTAL, default off] Continuous three-zone schedule on the
# steering-RATE cost: BOOST on a true straight, EASE on the approach to a
# corner the HORIZON can already see, FLOOR through the corner itself. Smooth
# surface (no thresholds/hysteresis) -- on a continuously-winding road `now`
# and `ahead` are both high so it just sits at the corner value.
# MULTIPLIES r_rate_delta, so it composes with the shipped value instead of
# discarding it (the trap NMPC_CORNER_RRATE_BLEND_ENABLED falls into).
# Composes multiplicatively with NMPC_RJERK_DELTA below, which is also on;
# if a regression shows up, disable this first, not the jerk term.
NMPC_RRATE_ZONE_ENABLED=true
NMPC_RRATE_ZONE_BOOST_STRAIGHT=2.0    # x r_rate on a true straight
# Do not set this much below ~0.7: lower values (including ones that make the
# zone uniformly weaker than no zone at all) DNF the offline sim at k=27,
# off-track at the tightest corner. Not a simple "too much release" effect,
# still unexplained -- see docs/tuning.md's "Three-zone rate schedule".
NMPC_RRATE_ZONE_EASE_APPROACH=0.80    # x r_rate when a corner is AHEAD -- the turn-in release
NMPC_RRATE_ZONE_FLOOR_CORNER=0.15     # x r_rate mid-corner
#
# [NMPC only, EXPERIMENTAL, default off] Steering-JERK weight: penalises the
# SECOND difference of steering (steering ACCELERATION) instead of only the
# first. A steady ramp into a corner scores near zero and is nearly free; an
# alternating wiggle is expensive. Strongest offline result of anything tried
# for slew/chatter reduction.
#
# CAUTION when judging this on a raceline reference instead of centerline:
# the raceline shows meaningfully higher saturation with the same weight,
# which belongs to the reference line, not this term -- see docs/reference/'s
# "Reference line: raceline vs centreline" before attributing a saturation
# figure to this weight.
NMPC_RJERK_DELTA=150.0
# NMPC_RJERK_A=0.0

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
# MPC_R_A_BRAKE=0.5                     # [shared] acceleration-effort weight, a_cmd < 0 (braking); separate from r_a_accel so braking effort can be tuned independently
# MPC_ADAPTIVE_R_RATE_DURING_FLOOR=0.625   # [LTV-QP only] R_rate softening floor, mid-corner
# MPC_ADAPTIVE_R_RATE_ENTERING_FLOOR=0.85  # [LTV-QP only] R_rate softening floor, corner approach
# MPC_CORNER_FACTOR_K=8.0                   # [LTV-QP only] corner_factor curve sharpness vs CURRENT |kappa|
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
_append_mpc_arg nmpc_rk_substeps "$NMPC_RK_SUBSTEPS"
_append_mpc_arg nmpc_jac_substeps "$NMPC_JAC_SUBSTEPS"
_append_mpc_arg nmpc_jac_gate_speed "$NMPC_JAC_GATE_SPEED"
_append_mpc_arg nmpc_jac_substeps_fast "$NMPC_JAC_SUBSTEPS_FAST"
_append_mpc_arg nmpc_rk_gate_speed "$NMPC_RK_GATE_SPEED"
_append_mpc_arg nmpc_rk_substeps_fast "$NMPC_RK_SUBSTEPS_FAST"
_append_mpc_arg nmpc_standstill_steer_damp_enabled "$NMPC_STANDSTILL_STEER_DAMP_ENABLED"
_append_mpc_arg nmpc_standstill_speed "$NMPC_STANDSTILL_SPEED"
_append_mpc_arg nmpc_standstill_steer_r_scale "$NMPC_STANDSTILL_STEER_R_SCALE"
_append_mpc_arg nmpc_standstill_fade_speed "$NMPC_STANDSTILL_FADE_SPEED"
_append_mpc_arg nmpc_q_e_y "$NMPC_Q_E_Y"
_append_mpc_arg nmpc_q_e_psi "$NMPC_Q_E_PSI"
_append_mpc_arg nmpc_q_epsi_dot "$NMPC_Q_EPSI_DOT"
_append_mpc_arg nmpc_r_delta "$NMPC_R_DELTA"
_append_mpc_arg nmpc_r_rate_delta "$NMPC_R_RATE_DELTA"
_append_mpc_arg nmpc_alat_ceiling_enabled "$NMPC_ALAT_CEILING_ENABLED"
_append_mpc_arg nmpc_v_des_filter_alpha "$NMPC_V_DES_FILTER_ALPHA"
_append_mpc_arg nmpc_spline_reference_enabled "$NMPC_SPLINE_REFERENCE_ENABLED"
_append_mpc_arg nmpc_friction_circle_enabled "$NMPC_FRICTION_CIRCLE_ENABLED"
_append_mpc_arg nmpc_progress_enabled "$NMPC_PROGRESS_ENABLED"
_append_mpc_arg nmpc_q_progress "$NMPC_Q_PROGRESS"
_append_mpc_arg nmpc_slack_linear_weight "$NMPC_SLACK_LINEAR_WEIGHT"
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
    if [ ! -z "$LIVE_VIZ_PID" ]; then
        echo "Stopping live debug visualiser (PID: $LIVE_VIZ_PID)..."
        # setsid above makes this PID its own process group leader, so the
        # negative PID kills the actual ros2 run process too, not just an
        # already-exited wrapper shell (plain `kill "$LIVE_VIZ_PID"` left the
        # matplotlib window open since the shell it targeted had exec'd away).
        kill -- "-$LIVE_VIZ_PID" 2>/dev/null
        kill "$LIVE_VIZ_PID" 2>/dev/null
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
# is silently invisible to `ros2 launch` until rebuilt -- do not assume a
# src/ edit is live without this, a stale build has cost hours of unreliable
# live-test results before. --symlink-install replaces the copy with a
# symlink for supported files (this workspace's packages are all pure
# Python + ament_index resources, so every affected file qualifies), so
# src/ IS the running code. Safe to run every launch: colcon no-ops
# packages that are already built and up to date.
# echo "[1.5/3] Building workspace (--symlink-install)..."
# if [ "$USE_DOCKER" = true ]; then
#     docker exec "$CONTAINER_NAME" bash -c "
#         source /opt/ros/jazzy/setup.bash && \
#         cd $CONTAINER_ROS2_DIR && \
#         colcon build --symlink-install
#     "
# else
#     bash -c "
#         source /opt/ros/jazzy/setup.bash && \
#         cd '$HOST_ROS2_DIR' && \
#         colcon build --symlink-install
#     "
# fi

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

# Live debug visualiser (close-up car/cones/reference path/NMPC predicted
# horizon/driven trail + control-output stats), see
# fsae_control/live_viz.py's own docstring. Sim-only debug tool: not part of
# the autonomy stack, host-only (needs a real display, no X11 forwarding set
# up for the Docker path today). Starts before the stack is fully up on
# purpose, its topics simply have no data yet and the window sits blank
# until [3/3] below starts publishing.
if [ "$USE_DOCKER" != true ]; then
    # Reap a leftover live_viz from a previous run that did not exit cleanly
    # (a crash, a killed terminal, anything that skipped cleanup() entirely):
    # setsid gives it its own process group with no controlling terminal, so
    # nothing sends it a signal when its launcher dies and it survives as an
    # orphan under /init. cleanup()'s own kill only reaches THIS run's PID,
    # so a prior orphan is otherwise invisible to it. pkill by full command
    # line, not by a saved pidfile -- simpler, and correct even if this is
    # the very first launch (no matches, no-op).
    pkill -9 -f "fsae_control/lib/fsae_control/live_viz" 2>/dev/null
    setsid bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        exec ros2 run fsae_control live_viz
    " > "$HOST_REPO_ROOT/fsae_logs/live_viz.log" 2>&1 &
    LIVE_VIZ_PID=$!
    echo "      live debug visualiser started (PID: $LIVE_VIZ_PID), log: fsae_logs/live_viz.log"
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
