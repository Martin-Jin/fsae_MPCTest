"""Identify WHICH mechanism caps FSDS's yaw rate, from step-input transients.

Companion to the `steering_step` ROS node.  The sweep
(`steering_sysid_analysis.py`) established THAT yaw is capped; this decides
which of three mechanisms does it, using the shape of the transient:

  A. HARD YAW-RATE LIMIT    yaw rises and CLIPS at r_max. No overshoot, and the
                            approach flattens abruptly rather than smoothly.
  B. SPEED-SCALED AUTHORITY yaw rises smoothly to a lower plateau -- a clean
                            first-order response, as if a smaller angle had
                            been commanded. No overshoot.
  C. ACTIVE DAMPING         yaw OVERSHOOTS the settled value then decays as the
                            damping term builds. Overshoot is the fingerprint;
                            neither A nor B can produce it.

Discriminators, in order of strength:

  1. OVERSHOOT (peak/final). >10% is the signature of C and rules out A and B.
  2. PEAK vs the steady-state cap. If the peak exceeds the sweep's fitted
     ceiling, the cap is not a hard clip -- the car demonstrably goes faster
     than it is later allowed to hold.
  3. RISE SHAPE. A first-order response reaches 63% of final in one time
     constant; a clipped one reaches its plateau early then goes flat.
  4. SETTLING. C decays back over a characteristic time; A and B do not decay
     at all once they arrive.

Usage:
    python -m tuner.checks.steering_step_analysis <steering_step_log.csv>
"""
import sys

import numpy as np

from tuner.csv_log import load_columns, medfilt

# Steady-state ceiling fitted by the sweep (m/s2 equivalent lateral).
SWEEP_CEILING_ALAT = 7.0
# Single-sample yaw spikes appear in FSDS telemetry (one 4.5 rad/s sample was
# seen amid ~1.0 rad/s neighbours). Median-filter before peak-finding, or the
# overshoot statistic just measures the glitch.
MEDFILT = 5


def load(path):
    return load_columns(path, string_columns=('phase',))


def analyse_trial(t_step, yaw_rate, v, steer):
    """Return transient descriptors for one step, or None if unusable."""
    ok = np.isfinite(t_step) & np.isfinite(yaw_rate)
    if ok.sum() < 20:
        return None
    ts, r = t_step[ok], np.abs(medfilt(yaw_rate[ok], k=MEDFILT))
    order = np.argsort(ts)
    ts, r = ts[order], r[order]
    if ts[-1] < 1.0:
        return None

    final = float(np.mean(r[ts > ts[-1] - 0.5]))      # last 0.5 s
    if final < 0.05:
        return None
    peak = float(r.max())
    t_peak = float(ts[int(np.argmax(r))])
    overshoot = 100.0 * (peak - final) / final

    # Time to 63% of final: the first-order time constant.
    tgt = 0.63 * final
    above = np.flatnonzero(r >= tgt)
    tau = float(ts[above[0]]) if len(above) else np.nan

    return {
        'v': float(np.nanmean(v[ok])),
        'steer': float(np.nanmean(np.abs(steer[ok]))),
        'peak': peak, 'final': final, 'overshoot': overshoot,
        't_peak': t_peak, 'tau': tau,
        'peak_alat': peak * float(np.nanmean(v[ok])),
        'final_alat': final * float(np.nanmean(v[ok])),
    }


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    d = load(argv[1])
    print(f"=== {argv[1].split('/')[-1]} ===")

    trials = []
    for tr in sorted(set(d['trial'][np.isfinite(d['trial'])])):
        m = (d['trial'] == tr) & (d['phase'] == 'step')
        if m.sum() < 20:
            continue
        res = analyse_trial(d['t_step'][m], d['yaw_rate'][m],
                            d['v_actual'][m], d['steer_norm'][m])
        if res:
            trials.append(res)

    if len(trials) < 2:
        print("!! Too few usable step trials. Did the run complete?")
        return 1

    print(f"{len(trials)} step trials\n")
    print(f"  {'v':>5} {'steer':>6} {'peak_r':>7} {'final_r':>8} "
          f"{'over%':>7} {'t_peak':>7} {'tau':>6} {'peak_aLat':>10}")
    for x in sorted(trials, key=lambda q: (q['v'], q['steer'])):
        print(f"  {x['v']:5.1f} {x['steer']:6.2f} {x['peak']:7.3f} "
              f"{x['final']:8.3f} {x['overshoot']:6.1f}% {x['t_peak']:7.2f} "
              f"{x['tau']:6.2f} {x['peak_alat']:10.2f}")

    # Above the ~6 m/s threshold is where the cap engages; below it the sweep
    # found s ~ 1.0, so those trials are the control group.
    hi = [x for x in trials if x['v'] > 6.0]
    lo = [x for x in trials if x['v'] <= 6.0]
    print()
    if lo:
        print(f"  below 6 m/s (cap not engaged): mean overshoot "
              f"{np.mean([x['overshoot'] for x in lo]):.1f}%")
    if not hi:
        print("!! No trials above 6 m/s — cannot test the cap. Re-run with "
              "higher speeds.")
        return 1

    ov = np.array([x['overshoot'] for x in hi])
    pk = np.array([x['peak_alat'] for x in hi])
    fi = np.array([x['final_alat'] for x in hi])
    print(f"  above 6 m/s (cap engaged):     mean overshoot {ov.mean():.1f}% "
          f"(min {ov.min():.1f}, max {ov.max():.1f})")
    print(f"    peak  lateral accel: mean {pk.mean():.2f} max {pk.max():.2f} m/s2")
    print(f"    final lateral accel: mean {fi.mean():.2f} m/s2 "
          f"(sweep fitted ceiling {SWEEP_CEILING_ALAT:.1f})")

    print("\n=== VERDICT ===")
    # "Peak exceeds the cap" must compare like with like: the transient PEAK
    # against that same trial's own SETTLED value, not against a fixed lateral
    # figure. A hard yaw clip at r_max still produces a large a_lat at high
    # speed (0.75 rad/s x 12 m/s = 9 m/s2), so an absolute threshold flags
    # every mechanism and discriminates nothing.
    exceeds = float(np.mean(pk / np.maximum(fi, 1e-6))) > 1.15

    # A cap pins DIFFERENT steering angles to the SAME settled response; a
    # scaled authority keeps them proportional. This is what separates a
    # ceiling from a gain reduction.
    hi_by_speed = {}
    for x in hi:
        hi_by_speed.setdefault(round(x['v']), []).append(x)
    clip_evidence = []
    for vv, xs in hi_by_speed.items():
        if len(xs) < 2:
            continue
        finals = np.array([x['final'] for x in xs])
        steers = np.array([x['steer'] for x in xs])
        if steers.max() / max(steers.min(), 1e-6) < 1.3:
            continue        # angles too close to tell apart
        clip_evidence.append(finals.max() / max(finals.min(), 1e-6))
    # Median, not mean: one weak trial (FSDS occasionally under-delivers a
    # step) inflates a single speed's spread and would mask a real cap.
    pinned = (len(clip_evidence) > 0
              and float(np.median(clip_evidence)) < 1.30)

    # WHAT is being held constant? A yaw-rate ceiling holds `final` equal
    # across speeds; a lateral-acceleration ceiling holds `final * v` equal.
    # These are the same thing at one speed and diverge across a sweep, so
    # this needs at least two capped speeds to answer.
    speeds_hi = sorted(hi_by_speed)
    ceiling_kind = None
    if len(speeds_hi) >= 2:
        r_by_v = np.array([np.mean([x['final'] for x in hi_by_speed[s]])
                           for s in speeds_hi])
        a_by_v = np.array([np.mean([x['final_alat'] for x in hi_by_speed[s]])
                           for s in speeds_hi])
        r_spread = r_by_v.max() / max(r_by_v.min(), 1e-6)
        a_spread = a_by_v.max() / max(a_by_v.min(), 1e-6)
        ceiling_kind = ('lateral acceleration' if a_spread < r_spread
                        else 'yaw rate')
        print(f"\n  settled response across capped speeds:")
        for s, rr, aa in zip(speeds_hi, r_by_v, a_by_v):
            print(f"    v={s:5.1f}  r={rr:.3f} rad/s   a_lat={aa:.2f} m/s2")
        print(f"    yaw-rate spread {r_spread:.2f}x vs "
              f"lateral-accel spread {a_spread:.2f}x")
        print(f"    -> the cap holds {ceiling_kind.upper()} constant")

    if pinned and ov.mean() > 10.0:
        # Both signatures at once: the response is capped (different angles
        # give the same settled value) AND it overshoots on the way in.
        # A static clip cannot overshoot, so the cap is enforced by a
        # DYNAMIC term that takes time to build.
        kind = ceiling_kind or 'lateral acceleration'
        lvl = float(np.mean(fi)) if kind.startswith('lateral') else float(np.mean([x['final'] for x in hi]))
        unit = 'm/s2' if kind.startswith('lateral') else 'rad/s'
        print("\n=== VERDICT ===")
        print(f"  DYNAMICALLY-ENFORCED {kind.upper()} CEILING (~{lvl:.1f} {unit}).")
        print(f"  Two signatures together: different steering angles settle to")
        print(f"  the same response (a cap), but yaw first OVERSHOOTS it by")
        print(f"  {ov.mean():.0f}% and then decays (not a static clip).")
        print()
        print("  So it is neither a pure hard limit nor pure damping: a")
        print("  restoring term builds over ~the observed decay time and pulls")
        print("  the car back to the ceiling once yaw exceeds it.")
        print()
        print("  TO MODEL: a first-order lag toward the ceiling, not a clip.")
        print("  A clip would match steady state but remove the turn-in")
        print("  transient the MPC actually reacts to.")
        return 0

    if ov.mean() > 10.0:
        print(f"  ACTIVE DAMPING (mechanism C). Yaw overshoots by "
              f"{ov.mean():.0f}% on average, then decays.")
        print("  Neither a hard yaw limit nor a speed-scaled steering map can")
        print("  overshoot: both would rise to their plateau and stay there.")
        if exceeds:
            print(f"  Confirmed: peak lateral reaches {pk.max():.1f} m/s2, above")
            print(f"  the {SWEEP_CEILING_ALAT:.1f} m/s2 steady-state cap — the car CAN")
            print("  exceed the cap briefly, it just is not allowed to hold it.")
        print()
        print("  TO MODEL: add a yaw-damping torque to the plant, not a clip.")
        print("  Fit the decay time constant from t_peak and the settling")
        print("  shape; a clip would reproduce steady state but give the wrong")
        print("  turn-in behaviour, which is what the MPC actually reacts to.")
    elif pinned:
        rmax = float(np.mean([x['final'] for x in hi]))
        print(f"  HARD YAW-RATE LIMIT (mechanism A) at ~{rmax:.2f} rad/s.")
        print(f"  No overshoot ({ov.mean():.1f}%), and different steering angles")
        print("  settle to the SAME yaw rate — the response is being clipped,")
        print("  not scaled. A scaled authority would keep the angles")
        print("  proportional to each other.")
        print()
        print("  TO MODEL: clip yaw rate in the plant at this value.")
    else:
        tau_hi = np.nanmean([x['tau'] for x in hi])
        tau_lo = np.nanmean([x['tau'] for x in lo]) if lo else np.nan
        print(f"  SPEED-SCALED AUTHORITY (mechanism B). No overshoot "
              f"({ov.mean():.1f}%),")
        print("  and yaw stays proportional to steering angle — just reduced.")
        print(f"    rise time constant above 6 m/s: {tau_hi:.2f}s "
              f"(below: {tau_lo:.2f}s)")
        if abs(tau_hi - tau_lo) < 0.1:
            print("    Same time constant either side: the dynamics are")
            print("    unchanged, only the gain — consistent with B.")
        print()
        print("  TO MODEL: scale effective steering angle by f(v) in the")
        print("  plant. Fit f(v) from the sweep's s-vs-speed table.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
