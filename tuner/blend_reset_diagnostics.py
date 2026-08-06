"""
Measure how often blend_paths()'s reset_dist bypass fires, and how large the
resulting heading discontinuity is, on the offline sim.

Why this exists
----------------
docs/sim_to_real_investigation.md (S12.8/12.9) found that BOTH stacks chase a
reference heading that swings faster than the car can ever yaw, and that this
drives 78-100% of heading-error growth -- but the residual saturation gap
between live (21.1%) and sim (~5%) was NOT explained by any plant/ceiling
parameter tested (see tuner/gap_attribution_ledger.py). That rules out the
*plant*; it does not rule out the *planner*.

planning/path_utils.py's blend_paths() already exists specifically to prevent
successive from-scratch centreline rebuilds from producing a heading jump
(EMA in the map frame, alpha=0.4) -- but it has an explicit reset_dist=2.0 m
escape hatch: if the fresh rebuild's mean sample distance from the previous
published path exceeds 2 m, the blend is skipped entirely and the raw new
path is published unblended. That bypass fires exactly when the rebuild has
changed the most -- i.e. plausibly correlated with the already-documented
curvature-spike / cone-map-clutter defect (planning_control_sync.md, "Known
planner defect: centreline curvature spikes").

Nothing previously measured how often this fires or how big the resulting
jump is, on either stack. This script measures it on the offline sim (both
the recorded map and the VALIDATION_SUITE paths) via a non-invasive wrapper
around blend_paths -- it does NOT modify planning/path_utils.py or
sim/sim_track.py, so the planner's actual behaviour during the rollout is
byte-for-byte identical to an uninstrumented run.

This is a MEASUREMENT of the existing mechanism, not a fix, and not proof the
mechanism explains the live/sim saturation gap -- reset events are counted on
the SIM side only; there is no live control log in this environment to
compare against (see investigation doc for why).

Usage
-----
    python3 -m tuner.blend_reset_diagnostics
"""
import math
import os
import sys

os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

import planning.path_utils as path_utils
from settings import VALIDATION_SUITE
from tuner.offline_tuner import SYNTHETIC_PATHS
from tuner.recorded_map_rollout import DEFAULT_MAP

_orig_blend_paths = path_utils.blend_paths


class _BlendRecorder:
    """Wraps blend_paths to record reset-bypass events without changing behaviour."""

    def __init__(self):
        self.calls = 0
        self.resets = 0
        self.reset_dists = []
        self.heading_jumps_deg = []

    def __call__(self, prev, new, car_pos, alpha=0.4, ds=0.5, horizon=15.0,
                 reset_dist=2.0):
        self.calls += 1
        out = _orig_blend_paths(prev, new, car_pos, alpha=alpha, ds=ds,
                                 horizon=horizon, reset_dist=reset_dist)
        if prev is not None and len(prev) >= 2 and len(new) >= 2:
            r_new = path_utils._resample_forward(new, car_pos, ds, int(horizon / ds) + 1)
            r_prev = path_utils._resample_forward(prev, car_pos, ds, int(horizon / ds) + 1)
            if r_new is not None and r_prev is not None:
                d = float(np.mean(np.linalg.norm(r_new - r_prev, axis=1)))
                if d > reset_dist:
                    self.resets += 1
                    self.reset_dists.append(d)
                    # Heading jump: direction of the first blended segment,
                    # before vs. after this call (out vs. what a pure blend
                    # would have produced), approximated via prev/new near-field
                    # heading at the anchor.
                    h_prev = math.degrees(math.atan2(r_prev[1, 1] - r_prev[0, 1],
                                                      r_prev[1, 0] - r_prev[0, 0]))
                    h_new = math.degrees(math.atan2(r_new[1, 1] - r_new[0, 1],
                                                     r_new[1, 0] - r_new[0, 0]))
                    dh = (h_new - h_prev + 180) % 360 - 180
                    self.heading_jumps_deg.append(abs(dh))
        return out

    def summary(self):
        if self.calls == 0:
            return "  no blend_paths calls recorded"
        rate = 100.0 * self.resets / self.calls
        s = f"  calls={self.calls}  resets={self.resets} ({rate:.1f}%)"
        if self.reset_dists:
            rd = np.array(self.reset_dists)
            s += f"\n    reset trigger dist: mean {rd.mean():.2f}  max {rd.max():.2f} m"
        if self.heading_jumps_deg:
            hj = np.array(self.heading_jumps_deg)
            s += (f"\n    near-field heading jump at reset: mean {hj.mean():.1f}  "
                  f"p90 {np.percentile(hj, 90):.1f}  max {hj.max():.1f} deg")
        return s


def run_recorded_map_instrumented():
    from model.vehicle_physics import VehicleParams
    from tuner.recorded_map_rollout import run as run_recorded

    rec = _BlendRecorder()
    path_utils.blend_paths = rec
    try:
        import sim.sim_track as sim_track
        sim_track.blend_paths = rec
        run_recorded(DEFAULT_MAP, VehicleParams())
    finally:
        path_utils.blend_paths = _orig_blend_paths
        sim_track.blend_paths = _orig_blend_paths
    return rec


def run_suite_instrumented():
    from tuner.gap_attribution_ledger import run_synthetic, shipped_params
    import sim.sim_track as sim_track

    out = {}
    for name in VALIDATION_SUITE:
        rec = _BlendRecorder()
        path_utils.blend_paths = rec
        sim_track.blend_paths = rec
        try:
            run_synthetic(name, shipped_params())
        finally:
            path_utils.blend_paths = _orig_blend_paths
            sim_track.blend_paths = _orig_blend_paths
        out[name] = rec
    return out


def main():
    print("=== blend_paths() reset_dist bypass -- recorded map ===")
    rec = run_recorded_map_instrumented()
    print(rec.summary())
    print()

    print("=== blend_paths() reset_dist bypass -- VALIDATION_SUITE ===")
    for name, rec in run_suite_instrumented().items():
        print(f"{name}:")
        print(rec.summary())
    print()
    print("NOTE: this is the SIM side only. There is no live control log in")
    print("this environment to compute the matching live-side reset rate for")
    print("comparison -- see docs/sim_to_real_investigation.md for why.")


if __name__ == "__main__":
    main()
