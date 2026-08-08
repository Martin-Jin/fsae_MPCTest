"""
Export a recorded cone map's oracle speed profile to a lightweight CSV the
LIVE ROS node can load directly.

Why this exists
----------------
sim/track_io.load_recorded_track() turns a cone_recorder JSON into a dense
(path_X, path_Y, path_Psi, path_v, blue, yellow) tuple, but that
reconstruction depends on scipy's CubicSpline and ~250 lines of centreline-
walking logic (planning/boundary.build_path_walls(), marched over the whole
lap) that fsae_planning's live ROS package does not otherwise use anywhere.
Porting all of that into the car's control package just to read one array
would add a new scipy dependency and a second copy of that reconstruction
logic to keep in sync forever (see docs/sim_to_real_investigation.md S48).

Instead: do the heavy reconstruction HERE, offline, once per map, and write
out just the three arrays the live node actually needs (x, y, v_target). The
live loader (control_utils.load_speed_profile_csv(), fsae_planning repo) is
a ~15-line CSV reader with no scipy and no boundary.py port.

Re-run this whenever the map changes (a new cone_recorder capture, or a
speed_profile.py change that would alter the oracle profile).

Usage
-----
    python3 -m tuner.export_speed_profile /path/to/cone_map.json out.csv
    python3 -m tuner.export_speed_profile  # defaults: repo-root cone_map.json -> speed_profile_export.csv
"""
import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from sim.track_io import load_recorded_track  # noqa: E402

DEFAULT_MAP = os.path.join(
    os.path.dirname(os.path.dirname(_HERE)), "cone_map.json")
DEFAULT_OUT = os.path.join(_HERE, "speed_profile_export.csv")


def export(map_path: str, out_path: str) -> None:
    path_X, path_Y, path_Psi, path_v, _blue, _yellow = load_recorded_track(map_path)
    with open(out_path, "w") as f:
        f.write("# x,y,v_target -- oracle speed profile exported by "
                "tuner.export_speed_profile from a cone_recorder map.\n")
        f.write(f"# source_map={os.path.abspath(map_path)}\n")
        f.write("x,y,v_target\n")
        for x, y, v in zip(path_X, path_Y, path_v):
            f.write(f"{x:.4f},{y:.4f},{v:.4f}\n")
    print(f"Wrote {len(path_X)} points -> {out_path}")
    print(f"v_target range: {path_v.min():.2f} - {path_v.max():.2f} m/s")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("map_path", nargs="?", default=DEFAULT_MAP)
    ap.add_argument("out_path", nargs="?", default=DEFAULT_OUT)
    args = ap.parse_args()
    export(args.map_path, args.out_path)


if __name__ == "__main__":
    main()
