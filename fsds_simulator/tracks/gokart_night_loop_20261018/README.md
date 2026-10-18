# gokart_night_loop_20261018

A hand-laid cone loop driven by the go-kart on 2026-10-18 at night (headlamp only), under
Stanley steering at a fixed 11 % throttle. **Not an FSDS track**: rebuilt from a real
rosbag, so it is excluded from `tracks.newest_track()` and never becomes an offline
script's default map. Load it by name.

| File | What it is |
|---|---|
| `cone_map.json` | 17 blue, 22 yellow, 1 orange cone (`orange` is ignored by the simulator loaders). The final SLAM map of run 2 (`2026-10-18_gocart_run2_tierA_stanley11pct_night`), replayed through `fsae_autonomous`'s `cone_landmark_mapper` with `tuner/tools/rosbag_slam_replay.sh` (fused cones up to 7 m) and converted with `python3 -m tuner.tools.rosbag_cone_map <replay> --slam` |
| `preview_2026-10-18_gocart_run2_slam.png` | written by `rosbag_cone_map` with the map: the cones, the SLAM-corrected path, every row edge checked against the 3 m laid spacing, and a results box (spacing; same-frame accuracy of this map and of the odometry-only map; run-3 agreement added with `--note`) |
| `centerline.csv` | `tuner.tools.raceline_optimizer --mode centerline`. The geometry is the reconstructed centreline; the `v_target` column is the simulated car's speed profile and means nothing for the kart |

## How far to trust it

Terms: *median* = half the values at or below it; *p90* = 90 % at or below it; *same-frame* = two cones seen in one camera frame, their measured gap against the map's gap (no odometry involved). More in `fsae_MPCTest/docs/reference/glossary.md`.

- **Local shape: about 1 cm.** Distances between two cones seen in the same camera frame
  match the map to 0.011 m median (0.03 m p90). The odometry-only map of the same run
  scored 0.059 m (0.22 m p90), so it was replaced.
- **Repeatability: about 1 cm.** SLAM on run 3 (a separate stack start), rigidly aligned,
  matches all 39 cones to 0.012 m median.
- **Cones may have moved between runs.** They were adjusted during the session (and the
  orange start gate was removed before run 4), so only within one lap are positions known
  to be fixed. Run 4's SLAM map sits 0.15 m median from this one; that mixes moved cones
  with map error.
- **Frame:** the ZED odometry frame of run 2 (origin where the stack started).
- **Orange cones:** the gate had 3; SLAM recorded 1, at (6.3, 0.7). It joins the yellow row in the preview's spacing check.
- **Cones left out:** the odometry-only map had 25 yellow and 4 orange; SLAM keeps a cone
  only once seen 3 times and associated consistently. Which cones those were is not
  pinned down; the odometry map's extras sit on the bottom row, where the start gate was.
- **Cone spacing** about 3 m along each row (estimated when laid out, not measured).
- **The reconstructed centreline** stops 5 m short of its start by design
  (`sim.track_io._MARCH_TAIL_GAP`).
