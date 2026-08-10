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
    python3 -m tuner.export_speed_profile                    # the default track
    python3 -m tuner.export_speed_profile comp_test_map_3    # a track by name
    python3 -m tuner.export_speed_profile --list             # what tracks exist
    python3 -m tuner.export_speed_profile /path/to/cone_map.json out.csv

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
sys.path.insert(0, os.path.dirname(_HERE))

from sim.track_io import load_recorded_track  # noqa: E402
from tracks import (  # noqa: E402
    DEFAULT_TRACK, SPEED_PROFILE_NAME, default_out_for, list_tracks,
    resolve_map_arg,
)


def export(map_path: str, out_path: str) -> None:
    path_X, path_Y, path_Psi, path_v, _blue, _yellow = load_recorded_track(map_path)
    with open(out_path, "w") as f:
        f.write("# x,y,psi,v_target -- oracle path + speed profile exported by "
                "tuner.export_speed_profile from a cone_recorder map.\n")
        f.write(f"# source_map={os.path.abspath(map_path)}\n")
        f.write("x,y,psi,v_target\n")
        for x, y, psi, v in zip(path_X, path_Y, path_Psi, path_v):
            f.write(f"{x:.4f},{y:.4f},{psi:.5f},{v:.4f}\n")
    print(f"Wrote {len(path_X)} points -> {out_path}")
    print(f"v_target range: {path_v.min():.2f} - {path_v.max():.2f} m/s")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "map", nargs="?", default=None,
        help=f"Track name under tracks/ (default: {DEFAULT_TRACK}), or an "
             "explicit path to a cone_map.json.")
    ap.add_argument(
        "out_path", nargs="?", default=None,
        help="Output CSV (default: speed_profile.csv beside the source map).")
    ap.add_argument("--list", action="store_true",
                    help="List available tracks and exit.")
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
    out_path = args.out_path or default_out_for(map_path, SPEED_PROFILE_NAME)
    export(map_path, out_path)


if __name__ == "__main__":
    main()
