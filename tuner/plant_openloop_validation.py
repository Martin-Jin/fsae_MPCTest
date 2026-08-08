"""
Validate the OFFLINE PLANT against the open-loop measurements taken on FSDS.

Why this exists
---------------
`steering_sysid_analysis.py` and `steering_step_analysis.py` answer "what does
FSDS do?". Neither answers "does our plant model now reproduce it?" — and that
question went unasked when `alat_ceiling_gain` was refitted 3000 -> 700, which
is how a 13% surplus in SUSTAINED cornering survived in the model.

This script closes that loop. It replays the two measured open-loop
experiments through `model/vehicle_physics.py` at matched (speed, steering) and
reports the residuals.

The two experiments probe different regimes, and that matters:

  step  (steering_step_*.csv)   3 s hold  -> TRANSIENT peak + short settle
  sweep (steering_sysid_*.csv)  long orbit -> SUSTAINED cornering

Sustained cornering is what builds heading error through a long corner, so the
sweep is the more relevant target for the closed-loop saturation gap, and it is
held-out data: nothing in the ceiling model was ever fitted to it.

Usage
-----
    python3 -m tuner.plant_openloop_validation              # both, current params
    python3 -m tuner.plant_openloop_validation --ab         # A/B the ceiling laws
    python3 -m tuner.plant_openloop_validation --robustness # rig confound checks

Interpreting the output
-----------------------
A positive residual means THE SIM CORNERS HARDER THAN FSDS. That direction
matters: a sim that out-corners the car gets the yaw it asks for, never builds
heading error, and never saturates its steering — which is exactly the residual
sim-to-real gap this file was written to chase.
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from model.vehicle_physics import (  # noqa: E402
    IDX_R, IDX_VX, VehicleParams, init_plant_state, step_nonlinear_plant,
)
from tuner.csv_log import medfilt as _medfilt  # noqa: E402
from tuner.csv_log import read_data_lines  # noqa: E402

# The harnesses (ros2/run_steering_*.sh) always write to $HOME/fsae_logs, never
# to a repo-relative path. A one-off manual copy previously left stale
# duplicates in <repo_root>/fsae_logs; point at the real source so this always
# picks up the newest run.
LOG_DIR = os.path.join(os.path.expanduser("~"), "fsae_logs")
MAX_STEER_RAD = np.radians(25.0)
NOMINAL_STEERS = (0.5, 0.65, 0.8, 1.0)
# Below this speed the measurements show the ceiling never engages, so these
# points test the tyre/geometry model instead of the ceiling.
CAPPED_SPEED = 7.0


# ─────────────────────────────────────────────────────────────────────────────
# helpers
# ─────────────────────────────────────────────────────────────────────────────
def _newest(pattern):
    hits = sorted(glob.glob(os.path.join(LOG_DIR, pattern)))
    if not hits:
        raise FileNotFoundError(f"no log matching {pattern} in {LOG_DIR}")
    return hits[-1]


def _load(path):
    return list(csv.DictReader(read_data_lines(path)))


def _summarise(t, yaw, v, settle_frac=0.4):
    """Peak (median-filtered, per the step analyser) and settled (tail mean)."""
    yaw = np.abs(np.asarray(yaw, float))
    v = np.asarray(v, float)
    alat = v * yaw
    tail = np.asarray(t) >= np.max(t) * (1.0 - settle_frac)
    return dict(yaw_peak=float(_medfilt(yaw).max()),
                yaw_settled=float(yaw[tail].mean()),
                alat_peak=float(_medfilt(alat).max()),
                alat_settled=float(alat[tail].mean()),
                v_settled=float(v[tail].mean()))


# ─────────────────────────────────────────────────────────────────────────────
# offline plant, driven open-loop exactly as the FSDS node drove the car
# ─────────────────────────────────────────────────────────────────────────────
def run_plant(target_v, delta_cmd, hold_s, dt=0.02, settle_s=3.0,
              ceiling=True, mode=None, gain=None, tau=None,
              kp=1.5, ki=0.5):
    """Settle straight at target_v, then step steering and hold.

    Speed is held by a PI on `a_cmd`, mirroring the PI+launch-floor the
    measurement node used. `--robustness` checks the result is not an artefact
    of that choice (longitudinal force steals lateral grip via the friction
    ellipse, so an over-aggressive gain would depress cornering).
    """
    p = VehicleParams()
    p.alat_ceiling_enabled = ceiling
    if mode is not None:
        p.alat_ceiling_mode = mode
    if gain is not None:
        p.alat_ceiling_gain = gain
    if tau is not None:
        p.alat_ceiling_tau = tau

    st = init_plant_state(0.0, 0.0, 0.0, vx0=target_v)
    ei = 0.0

    def a_cmd(vx):
        nonlocal ei
        e = target_v - vx
        ei = float(np.clip(ei + e * dt, -20.0, 20.0))
        return float(np.clip(kp * e + ki * ei, -8.0, 8.0))

    for _ in range(int(settle_s / dt)):
        st = step_nonlinear_plant(st, [0.0, a_cmd(st[IDX_VX])], dt, p)

    t, yaw, v = [], [], []
    for i in range(int(hold_s / dt)):
        st = step_nonlinear_plant(st, [delta_cmd, a_cmd(st[IDX_VX])], dt, p)
        t.append((i + 1) * dt)
        yaw.append(st[IDX_R])
        v.append(st[IDX_VX])
    return _summarise(np.array(t), yaw, v)


# ─────────────────────────────────────────────────────────────────────────────
# measured ground truth
# ─────────────────────────────────────────────────────────────────────────────
def measured_step():
    rows = _load(_newest("steering_step_*.csv"))
    trials = {}
    for r in rows:
        if r["phase"] != "step" or not r["t_step"]:
            continue
        trials.setdefault(int(r["trial"]), []).append(r)

    out = []
    for k in sorted(trials):
        rs = trials[k]
        t = np.array([float(r["t_step"]) for r in rs])
        s = _summarise(t, [float(r["yaw_rate"]) for r in rs],
                       [float(r["v_actual"]) for r in rs])
        s.update(trial=k, target_v=float(rs[0]["target_v"]),
                 steer_norm=abs(float(rs[0]["steer_norm"])),
                 delta_cmd=abs(float(rs[0]["delta_cmd_rad"])),
                 hold_s=float(t.max()), n=len(rs))
        out.append(s)
    return out


def measured_sweep():
    """Sustained cornering per (speed, steering) from the `record` phase."""
    rows = _load(_newest("steering_sysid_*.csv"))
    pts = {}
    for r in rows:
        if r["phase"] != "record":
            continue
        sn = round(abs(float(r["steer_norm"])), 2)
        if sn not in NOMINAL_STEERS:
            continue
        va, yr = float(r["v_actual"]), abs(float(r["yaw_rate"]))
        if va < 0.5:
            continue
        pts.setdefault((float(r["target_v"]), sn), []).append((va, yr, va * yr))

    out = {}
    for k, arr in pts.items():
        a = np.array(arr)
        out[k] = dict(v=float(a[:, 0].mean()), yaw=float(a[:, 1].mean()),
                      alat=float(a[:, 2].mean()), n=len(arr))
    return out


# ─────────────────────────────────────────────────────────────────────────────
# reports
# ─────────────────────────────────────────────────────────────────────────────
def report_step(cfg=None):
    cfg = cfg or {}
    meas = measured_step()
    print(f"\n=== STEP test replay ({len(meas)} trials) "
          f"— transient peak + short settle ===")
    hdr = (f"{'v':>5} {'steer':>6} | {'a_lat settled':>21} | "
           f"{'a_lat peak':>21} | {'yaw settled':>19}")
    print(hdr)
    print(f"{'':>5} {'':>6} | {'meas':>7}{'sim':>7}{'err':>7} | "
          f"{'meas':>7}{'sim':>7}{'err':>7} | {'meas':>6}{'sim':>6}{'err':>7}")
    print("-" * len(hdr))

    rows = []
    for m in meas:
        s = run_plant(m["target_v"], m["delta_cmd"], m["hold_s"], **cfg)
        rows.append((m, s))
        print(f"{m['target_v']:5.1f} {m['steer_norm']:6.2f} | "
              f"{m['alat_settled']:7.2f}{s['alat_settled']:7.2f}"
              f"{s['alat_settled'] - m['alat_settled']:+7.2f} | "
              f"{m['alat_peak']:7.2f}{s['alat_peak']:7.2f}"
              f"{s['alat_peak'] - m['alat_peak']:+7.2f} | "
              f"{m['yaw_settled']:6.3f}{s['yaw_settled']:6.3f}"
              f"{s['yaw_settled'] - m['yaw_settled']:+7.3f}")

    for label, sel in (("CAPPED (v >= 7)", lambda m: m["target_v"] >= CAPPED_SPEED),
                       ("UNCAPPED (v < 7)", lambda m: m["target_v"] < CAPPED_SPEED)):
        sub = [(m, s) for m, s in rows if sel(m)]
        if not sub:
            continue
        ms = np.mean([m["alat_settled"] for m, _ in sub])
        ss = np.mean([s["alat_settled"] for _, s in sub])
        mp = np.mean([m["alat_peak"] for m, _ in sub])
        sp = np.mean([s["alat_peak"] for _, s in sub])
        print(f"  {label:17} n={len(sub)}  "
              f"settled meas {ms:5.2f} sim {ss:5.2f} (x{ss / ms:4.2f})   "
              f"peak meas {mp:5.2f} sim {sp:5.2f} (x{sp / mp:4.2f})")
    return rows


def report_sweep(cfg=None, hold_s=6.0):
    cfg = cfg or {}
    meas = measured_sweep()
    n = sum(m["n"] for m in meas.values())
    print(f"\n=== SWEEP replay ({len(meas)} points, {n} samples) "
          f"— SUSTAINED cornering (held-out) ===")
    hdr = (f"{'v':>5} {'steer':>6} {'n':>4} | {'a_lat':>21} | {'yaw rate':>21}")
    print(hdr)
    print(f"{'':>5} {'':>6} {'':>4} | {'meas':>7}{'sim':>7}{'err':>7} | "
          f"{'meas':>7}{'sim':>7}{'err':>7}")
    print("-" * len(hdr))

    errs, capped_errs = [], []
    for key in sorted(meas):
        v, sn = key
        m = meas[key]
        s = run_plant(v, sn * MAX_STEER_RAD, hold_s, **cfg)
        e = s["alat_settled"] - m["alat"]
        errs.append(e)
        if v >= CAPPED_SPEED:
            capped_errs.append(e)
        print(f"{v:5.1f} {sn:6.2f} {m['n']:4d} | "
              f"{m['alat']:7.2f}{s['alat_settled']:7.2f}{e:+7.2f} | "
              f"{m['yaw']:7.3f}{s['yaw_settled']:7.3f}"
              f"{s['yaw_settled'] - m['yaw']:+7.3f}")

    e = np.array(errs)
    print(f"  ALL      n={len(e):2d}  mean err {e.mean():+5.2f}  "
          f"MAE {np.abs(e).mean():4.2f}  max |err| {np.abs(e).max():4.2f}")
    if capped_errs:
        c = np.array(capped_errs)
        print(f"  CAPPED   n={len(c):2d}  mean err {c.mean():+5.2f}  "
              f"MAE {np.abs(c).mean():4.2f}  max |err| {np.abs(c).max():4.2f}")
    return errs


def report_ab():
    """Show that the proportional law cannot fit settled and peak together."""
    meas = [m for m in measured_step() if m["target_v"] >= CAPPED_SPEED]
    m_set = np.mean([m["alat_settled"] for m in meas])
    m_pk = np.mean([m["alat_peak"] for m in meas])
    print("\n=== A/B: ceiling law vs the two step-test targets ===")
    print(f"measured (capped trials, n={len(meas)}): "
          f"settled {m_set:.2f}   peak {m_pk:.2f}\n")

    for mode, gains, note in (
        ('p', [300, 700, 1500, 3000, 6000],
         "PROPORTIONAL — equilibrium must sit ABOVE the setpoint"),
        ('pi', [150, 300, 450, 700, 1200],
         "LEAKY INTEGRAL — settled pinned at the ceiling by structure"),
    ):
        print(f"  {note}")
        print(f"  {'gain':>7} {'settled':>8} {'err':>7} {'peak':>8} {'err':>7}")
        for g in gains:
            sett, peak = [], []
            for m in meas:
                s = run_plant(m["target_v"], m["delta_cmd"], m["hold_s"],
                              mode=mode, gain=g)
                sett.append(s["alat_settled"])
                peak.append(s["alat_peak"])
            sm, pm = np.mean(sett), np.mean(peak)
            print(f"  {g:7.0f} {sm:8.2f} {sm - m_set:+7.2f} "
                  f"{pm:8.2f} {pm - m_pk:+7.2f}")
        print()


def report_robustness():
    """Guard against the rig itself producing the residual (trap #4)."""
    print("\n=== Rig robustness ===")
    print("Speed-hold gains: longitudinal force steals lateral grip via the")
    print("friction ellipse, so an aggressive PI would depress cornering.")
    print(f"  {'kp':>5} {'ki':>5} | {'8 m/s full lock':>16} "
          f"{'4 m/s full lock':>16}")
    for kp, ki in ((0.5, 0.1), (1.5, 0.5), (3.0, 1.0)):
        a = run_plant(8.0, MAX_STEER_RAD, 3.0, kp=kp, ki=ki)
        b = run_plant(4.0, MAX_STEER_RAD, 3.0, kp=kp, ki=ki)
        print(f"  {kp:5.1f} {ki:5.1f} | {a['alat_settled']:16.2f} "
              f"{b['alat_settled']:16.2f}")

    print("\nTimestep (plant runs dt=0.05 on a rollout, measured at 50 Hz):")
    for dt in (0.05, 0.02, 0.01):
        a = run_plant(8.0, MAX_STEER_RAD, 3.0, dt=dt)
        print(f"  dt={dt:4.2f} | settled {a['alat_settled']:5.2f}  "
              f"peak {a['alat_peak']:5.2f}")

    print("\nCeiling must be INACTIVE below ~6 m/s (documented check):")
    for v in (3.0, 4.0, 5.0):
        on = run_plant(v, MAX_STEER_RAD, 3.0, ceiling=True)
        off = run_plant(v, MAX_STEER_RAD, 3.0, ceiling=False)
        flag = "OK" if abs(on["alat_settled"] - off["alat_settled"]) < 0.02 else "ACTIVE"
        print(f"  {v:4.1f} m/s | on {on['alat_settled']:5.2f}  "
              f"off {off['alat_settled']:5.2f}  {flag}")

    print("\nUnconstrained plant capability (ceiling off, for reference):")
    for v in (8.0, 12.0):
        off = run_plant(v, MAX_STEER_RAD, 3.0, ceiling=False)
        print(f"  {v:4.1f} m/s | settled {off['alat_settled']:5.2f}  "
              f"peak {off['alat_peak']:5.2f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ab", action="store_true",
                    help="A/B the proportional vs integral ceiling law")
    ap.add_argument("--robustness", action="store_true",
                    help="rig confound checks")
    ap.add_argument("--mode", choices=("p", "pi"),
                    help="override alat_ceiling_mode")
    ap.add_argument("--gain", type=float, help="override alat_ceiling_gain")
    ap.add_argument("--tau", type=float, help="override alat_ceiling_tau")
    args = ap.parse_args()

    cfg = {k: v for k, v in
           (("mode", args.mode), ("gain", args.gain), ("tau", args.tau))
           if v is not None}

    p = VehicleParams()
    print(f"plant ceiling: mode={cfg.get('mode', p.alat_ceiling_mode)} "
          f"ceiling={p.alat_ceiling} "
          f"gain={cfg.get('gain', p.alat_ceiling_gain)} "
          f"tau={cfg.get('tau', p.alat_ceiling_tau)}")

    if args.robustness:
        report_robustness()
        return
    if args.ab:
        report_ab()
        return
    report_step(cfg)
    report_sweep(cfg)


if __name__ == "__main__":
    main()
