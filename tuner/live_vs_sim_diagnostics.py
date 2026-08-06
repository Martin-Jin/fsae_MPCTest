"""
Characterise the live car's saturation symptom and compare LIKE-FOR-LIKE
against the offline sim on quantities the earlier investigation never matched.

Motivation
----------
The plant's cornering capability has been measured, modelled and (as of the
integral-law fix) validated — and it does NOT explain the residual saturation
gap: no ceiling level reproduces "completes the lap while saturating 21%".

So this looks at the other interfaces. Two comparisons in particular were
never made like-for-like:

1. SPEED-TRACKING ERROR, not mean speed.  The speed-profile question was
   closed by comparing MEAN achieved speed (8.20 sim vs 8.03 live, 2%). A mean
   is exactly the controller-influenced aggregate that hides a lag: a car that
   is late decelerating is too fast at corner entry and too slow on exit with
   an identical mean. `fsds_bridge` DISCARDS the MPC's `a_cmd` and re-derives
   throttle with its own P-controller, so live has a proportional lag stage the
   sim does not model at all.

2. SATURATION EPISODE STRUCTURE.  21% of ticks at the stop can mean two very
   different things: long sustained pulls (the controller genuinely cannot
   turn enough) or fast chatter against the limit (a jittery reference). These
   demand different fixes, and the aggregate percentage cannot tell them apart.

Usage
-----
    python3 -m tuner.live_vs_sim_diagnostics
    python3 -m tuner.live_vs_sim_diagnostics --no-sim   # live logs only
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

LOG_DIR = os.path.join(os.path.dirname(os.path.dirname(_HERE)), "fsae_logs")
MAX_STEER_DEG = 25.0
SAT_FRAC = 0.95


def _load(path):
    with open(path) as f:
        rows = [ln for ln in f if not ln.startswith("#")]
    return list(csv.DictReader(rows))


def _col(rows, name, default=np.nan):
    return np.array([float(r[name]) if r.get(name) not in (None, "") else default
                     for r in rows])


def episodes(mask, dt):
    """Contiguous-run lengths (s) of a boolean mask."""
    out, run = [], 0
    for m in mask:
        if m:
            run += 1
        elif run:
            out.append(run * dt)
            run = 0
    if run:
        out.append(run * dt)
    return np.array(out)


def describe_sat(label, sat, dt):
    ep = episodes(sat, dt)
    pct = 100.0 * sat.mean()
    if len(ep) == 0:
        print(f"  {label:<12} {pct:5.2f}%   no episodes")
        return
    print(f"  {label:<12} {pct:5.2f}%   episodes n={len(ep):4d}  "
          f"median {np.median(ep):5.3f}s  p90 {np.percentile(ep, 90):5.3f}s  "
          f"max {ep.max():5.3f}s  rate {len(ep) / (len(sat) * dt):5.2f}/s")


def describe_speed(label, v, v_tgt):
    e = v - v_tgt
    over = e > 0
    print(f"  {label:<12} err mean {e.mean():+6.2f}  MAE {np.abs(e).mean():5.2f}  "
          f"p10 {np.percentile(e, 10):+6.2f}  p90 {np.percentile(e, 90):+6.2f}  "
          f"over-speed {100.0 * over.mean():5.1f}%")


def conditional(label, sat, series):
    """Compare each signal inside vs outside saturation episodes.

    This is the causal link test: if saturation is driven by arriving at
    corners over-speed, the speed error should be markedly higher inside
    saturation than outside it.
    """
    print(f"  {label} — inside vs outside saturation "
          f"({100.0 * sat.mean():.1f}% inside)")
    print(f"    {'signal':<14}{'inside':>9}{'outside':>9}{'delta':>9}")
    for name, x in series:
        x = np.asarray(x, float)
        ok = np.isfinite(x)
        a = x[sat & ok]
        b = x[(~sat) & ok]
        if len(a) == 0 or len(b) == 0:
            continue
        print(f"    {name:<14}{a.mean():9.2f}{b.mean():9.2f}"
              f"{a.mean() - b.mean():+9.2f}")


def reference_quality(label, t, car_yaw, e_psi_rad, yaw_rate, v):
    """Decompose heading-error growth into car lag vs reference motion.

    e_psi = wrap(car_yaw - ref_psi)  (mpc_core.compute_tracking_errors)
      =>  d(e_psi)/dt = yaw_rate - d(ref_psi)/dt

    So a rising heading error has exactly two possible sources: the car is not
    yawing fast enough, or the REFERENCE heading is swinging away from it. A
    physical track reference should move at v*kappa -- smooth and bounded. If
    d(ref_psi)/dt carries large spikes, the controller is chasing a reference
    that jumps, and no plant change can fix that.

    This is a different statistic from the curvature comparison that
    eliminated "the live centreline is worse" (docs, hypothesis 3): a
    centreline can have acceptable curvature statistics and still deliver a
    jumping HEADING, e.g. if it is rebuilt from scratch each republish.
    """
    ref_psi = np.unwrap(np.asarray(car_yaw, float) - np.asarray(e_psi_rad, float))
    dref = np.gradient(ref_psi, t)
    dref_deg = np.degrees(dref)
    yr_deg = np.degrees(np.asarray(yaw_rate, float))

    # what the reference heading rate SHOULD be, bounded by the car's speed and
    # the tightest curvature the track can physically hold
    print(f"  {label} reference-heading motion")
    print(f"    |d(ref_psi)/dt|  mean {np.abs(dref_deg).mean():7.1f}  "
          f"p90 {np.percentile(np.abs(dref_deg), 90):7.1f}  "
          f"p99 {np.percentile(np.abs(dref_deg), 99):7.1f}  "
          f"max {np.abs(dref_deg).max():7.1f}  deg/s")
    print(f"    |yaw_rate|       mean {np.abs(yr_deg).mean():7.1f}  "
          f"p90 {np.percentile(np.abs(yr_deg), 90):7.1f}  "
          f"p99 {np.percentile(np.abs(yr_deg), 99):7.1f}  "
          f"max {np.abs(yr_deg).max():7.1f}  deg/s")
    ratio = np.abs(dref_deg).mean() / max(np.abs(yr_deg).mean(), 1e-9)
    print(f"    ref/car heading-rate ratio {ratio:5.2f}  "
          f"(>1 means the reference swings faster than the car ever yaws)")

    # attribute growth of |e_psi| to whichever term dominates at that tick
    de = np.gradient(np.asarray(e_psi_rad, float), t)
    growing = np.abs(e_psi_rad) > np.radians(10.0)
    growing &= np.sign(de) == np.sign(e_psi_rad)   # error getting worse
    if growing.sum() > 10:
        ref_driven = np.abs(dref[growing]) > np.abs(np.asarray(yaw_rate)[growing])
        print(f"    while |e_psi|>10 deg and worsening (n={growing.sum()}): "
              f"reference-driven {100.0 * ref_driven.mean():5.1f}%  "
              f"car-lag-driven {100.0 * (~ref_driven).mean():5.1f}%")


def live_report():
    paths = sorted(glob.glob(os.path.join(LOG_DIR, "mpc_standalone_control_*.csv")))
    print(f"=== LIVE logs ({len(paths)}) ===\n")
    picked = None
    for p in paths:
        rows = _load(p)
        if len(rows) < 200:
            print(f"{os.path.basename(p)}: {len(rows)} rows — too short, skipped")
            continue
        t = _col(rows, "t")
        dt = float(np.median(np.diff(t)))
        steer = _col(rows, "steer_deg")
        sat = np.abs(steer) > SAT_FRAC * MAX_STEER_DEG
        v, vt = _col(rows, "v_actual"), _col(rows, "v_desired")
        e_psi = np.abs(_col(rows, "e_psi_deg"))
        r = _col(rows, "yaw_rate")
        alat = np.abs(v * r)

        print(f"{os.path.basename(p)}  n={len(rows)}  dt={dt:.3f}s  "
              f"dur={t[-1] - t[0]:.1f}s")
        print(f"  speed        mean {v.mean():5.2f}  max {v.max():5.2f}   "
              f"target mean {vt.mean():5.2f}")
        print(f"  |e_psi|      mean {e_psi.mean():5.2f}  "
              f"p90 {np.percentile(e_psi, 90):5.2f}")
        print(f"  a_lat        max {alat.max():5.2f}  "
              f">7.5 {100.0 * (alat > 7.5).mean():5.2f}%")
        describe_sat("steer sat", sat, dt)
        describe_speed("speed trk", v, vt)

        # a_cmd is discarded by fsds_bridge -- how far is achieved from asked?
        a_cmd = _col(rows, "a_cmd")
        a_ach = np.gradient(v, t)
        good = np.isfinite(a_cmd) & np.isfinite(a_ach)
        if good.sum() > 50:
            c = np.corrcoef(a_cmd[good], a_ach[good])[0, 1]
            sl = np.polyfit(a_cmd[good], a_ach[good], 1)[0]
            print(f"  a_cmd->a_ach slope {sl:5.2f}  corr {c:5.2f}  "
                  f"(1.0/1.0 would mean the bridge honoured a_cmd)")
        reference_quality("live", t, _col(rows, "car_yaw"),
                          np.radians(_col(rows, "e_psi_deg")), r, v)
        conditional("live", sat, [("speed", v), ("v_target", vt),
                                  ("speed err", v - vt), ("|e_psi| deg", e_psi),
                                  ("a_lat", alat), ("|e_y| m", np.abs(_col(rows, "e_y")))])
        print()
        if picked is None or len(rows) > len(picked[1]):
            picked = (p, rows)
    return picked


def sim_report(mode=None, gain=None):
    from model.vehicle_physics import VehicleParams
    from tuner.recorded_map_rollout import DEFAULT_MAP, run

    p = VehicleParams()
    if mode:
        p.alat_ceiling_mode = mode
    if gain:
        p.alat_ceiling_gain = gain

    print(f"=== SIM (recorded map, mode={p.alat_ceiling_mode} "
          f"gain={p.alat_ceiling_gain}) ===\n")
    h = run(DEFAULT_MAP, p)["history"]
    dt = 0.05
    v = np.asarray(h["v"], float)
    vt = np.asarray(h["v_target"], float)
    steer = np.asarray(h["u_steer"], float)
    sat = np.abs(steer) > SAT_FRAC * p.max_steer
    e_psi = np.degrees(np.abs(np.asarray(h["e_psi"], float)))
    r = np.asarray(h["r"], float)
    alat = np.abs(v * r)

    print(f"  n={len(v)}  dur={len(v) * dt:.1f}s")
    print(f"  speed        mean {v.mean():5.2f}  max {v.max():5.2f}   "
          f"target mean {vt.mean():5.2f}")
    print(f"  |e_psi|      mean {e_psi.mean():5.2f}  "
          f"p90 {np.percentile(e_psi, 90):5.2f}")
    print(f"  a_lat        max {alat.max():5.2f}  "
          f">7.5 {100.0 * (alat > 7.5).mean():5.2f}%")
    describe_sat("steer sat", sat, dt)
    describe_speed("speed trk", v, vt)
    reference_quality("sim", np.arange(len(v)) * dt, np.asarray(h["psi"], float),
                      np.asarray(h["e_psi"], float), r, v)
    conditional("sim", sat, [("speed", v), ("v_target", vt),
                             ("speed err", v - vt), ("|e_psi| deg", e_psi),
                             ("a_lat", alat),
                             ("|e_y| m", np.abs(np.asarray(h["e_y"], float)))])
    print()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--no-sim", action="store_true")
    ap.add_argument("--mode", choices=("p", "pi"))
    ap.add_argument("--gain", type=float)
    args = ap.parse_args()

    live_report()
    if not args.no_sim:
        sim_report(args.mode, args.gain)


if __name__ == "__main__":
    main()
