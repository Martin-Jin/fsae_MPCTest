"""
Export a recorded cone map's oracle path + speed profile to a lightweight CSV
the LIVE ROS node can load directly.

Why this exists
----------------
sim/track_io.load_recorded_track() turns a cone_recorder JSON into a dense
(path_X, path_Y, path_Psi, path_v, blue, yellow) tuple, but that
reconstruction depends on scipy's CubicSpline and ~250 lines of centreline-
walking logic (planning/boundary.build_path_walls(), marched over the whole
lap) that fsae_planning's live ROS package does not otherwise use anywhere.
Porting all of that into the car's control package just to read one array
would add a new scipy dependency and a second copy of that reconstruction
logic to keep in sync forever.

Instead: do the heavy reconstruction HERE, offline, once per map, and write
out just the four arrays the live node actually needs (x, y, psi, v_target).
The live loader (control_utils.load_speed_profile_csv() /
load_path_profile_csv(), fsae_planning repo) is a ~15-line CSV reader with no
scipy and no boundary.py port. psi is exported alongside x/y so a live
consumer that wants the path itself (not just the speed lookup) doesn't have
to re-derive heading from consecutive points if it would rather use the
spline-fit heading directly — see USE_PRECOMPUTED_PATH in settings.py.

Re-run this whenever the map changes (a new cone_recorder capture, or a
speed_profile.py change that would alter the oracle profile).

Usage
-----
    python3 -m tuner.tools.export_speed_profile                    # the default track
    python3 -m tuner.tools.export_speed_profile comp_test_map_3    # a track by name
    python3 -m tuner.tools.export_speed_profile --list             # what tracks exist
    python3 -m tuner.tools.export_speed_profile /path/to/cone_map.json out.csv
    python3 -m tuner.tools.export_speed_profile comp_test_map_3 --open-loop  # stop at finish

    # Low-speed CORNER testing: drive normally, slow to 3 m/s only where the
    # path curvature exceeds a threshold, instead of a flat low v_max the
    # whole lap (which makes a test run crawl to the corner it is meant to
    # test). Writes speed_profile_corner_test.csv, never speed_profile.csv.
    python3 -m tuner.tools.export_speed_profile --corner-slowdown 0.10
    python3 -m tuner.tools.export_speed_profile --corner-slowdown 0.10 --corner-speed 2.0
    # No --corner-slowdown value: prints the track's curvature distribution
    # and per-threshold corner-zone count, to help pick one.
    python3 -m tuner.tools.export_speed_profile --corner-slowdown

By default the profile is computed closed-loop (a continuous lap: point 0
gets a braking obligation from the fast point before it at the end of the
array, and vice versa) — see speed_profile.compute_speed_profile()'s
closed_loop docstring. Pass --open-loop only for a genuinely point-to-point
recording that is not meant to be lapped.

With a track name (or no argument), the output goes to
`tracks/<name>/speed_profile.csv`, which is where `launch_all.sh`'s `TRACK=`
variable points the car — so the common case needs no output path. See
`tracks/__init__.py` for the directory layout.
"""
import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(_HERE)))

from sim.speed_profile import (  # noqa: E402
    compute_corner_slowdown_profile, compute_path_curvature,
)
from sim.track_io import load_recorded_track  # noqa: E402
from tracks import (  # noqa: E402
    SPEED_PROFILE_NAME, default_out_for, list_tracks, newest_track,
    resolve_map_arg,
)

CORNER_TEST_NAME = "speed_profile_corner_test.csv"


def export(map_path: str, out_path: str, closed_loop: bool = True,
           corner_slowdown_kappa: float = None, corner_speed: float = 3.0) -> None:
    path_X, path_Y, path_Psi, path_v, _blue, _yellow = load_recorded_track(
        map_path, closed_loop=closed_loop
    )
    kind = "oracle path + speed profile"
    if corner_slowdown_kappa is not None:
        # path_v above is the NORMAL profile; discard it and recompute with
        # the corner clamp -- path_X/path_Y/path_Psi (the geometry) are
        # unaffected either way, only the speed target changes.
        path_v = compute_corner_slowdown_profile(
            path_X, path_Y, kappa_threshold=corner_slowdown_kappa,
            corner_speed=corner_speed, closed_loop=closed_loop,
        )
        kind = (f"oracle path + CORNER-SLOWDOWN speed profile "
                f"(kappa>{corner_slowdown_kappa} -> {corner_speed} m/s)")
    with open(out_path, "w") as f:
        f.write(f"# x,y,psi,v_target -- {kind} exported by "
                "tuner.tools.export_speed_profile from a cone_recorder map.\n")
        f.write(f"# source_map={os.path.abspath(map_path)}\n")
        if corner_slowdown_kappa is not None:
            f.write(f"# corner_slowdown_kappa_threshold={corner_slowdown_kappa}\n")
            f.write(f"# corner_slowdown_corner_speed={corner_speed}\n")
        f.write("x,y,psi,v_target\n")
        for x, y, psi, v in zip(path_X, path_Y, path_Psi, path_v):
            f.write(f"{x:.4f},{y:.4f},{psi:.5f},{v:.4f}\n")
    print(f"Wrote {len(path_X)} points -> {out_path}")
    print(f"v_target range: {path_v.min():.2f} - {path_v.max():.2f} m/s")


def _print_curvature_guide(path_X, path_Y):
    """
    Printed when --corner-slowdown is given with no value, to help pick a
    kappa_threshold: the track's curvature distribution, and how many
    distinct corner zones (contiguous runs of |kappa| above threshold, small
    gaps merged) each candidate threshold would flag. Too low a threshold
    catches every gentle bend (the whole-lap-slow problem this feature exists
    to avoid); too high catches nothing.
    """
    kappa = compute_path_curvature(path_X, path_Y)
    ak = np.abs(kappa)
    print("Curvature |kappa| distribution on this track (1/m):")
    for p in (50, 75, 90, 95, 99, 100):
        print(f"  p{p:<3d} {np.percentile(ak, p):.4f}")
    print("\nCorner zones by threshold (contiguous |kappa|>threshold runs, "
          "small gaps merged):")
    n = len(ak)
    for thr in (0.06, 0.08, 0.10, 0.12, 0.15):
        mask = ak > thr
        idx = np.where(mask)[0]
        if len(idx) == 0:
            print(f"  kappa>{thr:.2f}: 0 points, 0 zones")
            continue
        groups = 1
        prev = idx[0]
        for i in idx[1:]:
            if i - prev > 3:
                groups += 1
            prev = i
        print(f"  kappa>{thr:.2f}: {mask.sum()}/{n} points, {groups} corner zones")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "map", nargs="?", default=None,
        help="Track name under tracks/ (default: the most recently recorded "
             f"track, currently {newest_track()!r}), or an explicit path to "
             "a cone_map.json.")
    ap.add_argument(
        "out_path", nargs="?", default=None,
        help="Output CSV (default: speed_profile.csv beside the source map).")
    ap.add_argument("--list", action="store_true",
                    help="List available tracks and exit.")
    ap.add_argument(
        "--open-loop", dest="closed_loop", action="store_false", default=True,
        help="Treat the path as point-to-point instead of a lap: the speed "
             "profile brakes to a stop at the last point instead of staying "
             "connected across the start/finish seam. Default is closed-loop "
             "(continuous lap), which is correct for a recorded track.")
    ap.add_argument(
        "--no-overwrite", dest="allow_overwrite", action="store_false", default=True,
        help="Refuse to replace an existing output file; error instead of "
             "silently overwriting a previous export of the same track.")
    ap.add_argument(
        "--corner-slowdown", type=float, nargs="?", const=-1.0, default=None,
        metavar="KAPPA",
        help="Low-speed CORNER testing: normal speed everywhere except "
             "|kappa| > KAPPA, clamped to --corner-speed there. Writes "
             f"{CORNER_TEST_NAME}, never {SPEED_PROFILE_NAME}. Given with no "
             "value, prints the track's curvature distribution and a "
             "per-threshold corner-zone count instead of exporting, to help "
             "pick KAPPA.")
    ap.add_argument(
        "--corner-speed", type=float, default=3.0, metavar="M/S",
        help="Speed to clamp to where --corner-slowdown's threshold is "
             "exceeded (default: %(default)s m/s). Ignored without "
             "--corner-slowdown.")
    args = ap.parse_args()

    if args.list:
        for name in list_tracks():
            print(name)
        return

    try:
        map_path = resolve_map_arg(args.map)
    except ValueError as e:
        # A mistyped track name is operator error, not a bug -- a one-line
        # message naming the available tracks is more useful than a traceback.
        ap.error(str(e))

    if args.corner_slowdown == -1.0:  # --corner-slowdown given with no value
        path_X, path_Y, _psi, _v, _blue, _yellow = load_recorded_track(
            map_path, closed_loop=args.closed_loop)
        _print_curvature_guide(path_X, path_Y)
        return

    if args.corner_slowdown is not None:
        default_name = CORNER_TEST_NAME
    else:
        default_name = SPEED_PROFILE_NAME
    try:
        out_path = args.out_path or default_out_for(
            map_path, default_name, allow_overwrite=args.allow_overwrite)
    except FileExistsError as e:
        ap.error(str(e))
    export(map_path, out_path, closed_loop=args.closed_loop,
           corner_slowdown_kappa=args.corner_slowdown, corner_speed=args.corner_speed)


if __name__ == "__main__":
    main()
