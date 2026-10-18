# gokart_night_loop_20261018

A hand-laid cone loop driven by the go-kart on 2026-10-18 at night (headlamp only), under
Stanley steering at a fixed 11 % throttle. **Not an FSDS track**: rebuilt from a real
rosbag, so it is excluded from `tracks.newest_track()` and never becomes an offline
script's default map. Load it by name.

| File | What it is |
|---|---|
| `cone_map.json` | 17 blue, 25 yellow, 4 orange cones (`orange` is ignored by the simulator loaders). Built by `python3 -m tuner.tools.rosbag_cone_map` from `2026-10-18_gocart_run2_tierA_stanley11pct_night` (`/cone_detection_fused`, 1-7 m range, 0.75 m merge radius, at least 5 sightings) |
| `preview.png` | the cones and the kart's odometry path for that run |
| `centerline.csv` | `tuner.tools.raceline_optimizer --mode centerline`. The geometry is the reconstructed centreline; the `v_target` column is the simulated car's speed profile and means nothing for the kart |

## How far to trust it

- **Frame:** the ZED odometry frame of that run (origin where the stack started). Another
  run's map is in a different frame.
- **Repeatability:** the same tool on run 3 (a separate stack start), rigidly aligned
  onto this map, matches 41 of 43 cones within 0.5 m, median 0.11 m.
- **Spread:** each cone's sightings sit a median 0.16-0.18 m from its position.
- **Drift:** runs 1 and 4 show the odometry drifting about 1 m per lap, which duplicates
  cones, so they were not used. Runs 2 and 3 are clean.
- **Cone spacing** about 3 m along each row (estimated when laid out, not measured).
- **The start gate:** the orange cones and one yellow cone sit inside the loop near
  (6, 1-3). This is where the kart veered left on runs 1-3.
- **The reconstructed centreline** stops 5 m short of its start by design
  (`sim.track_io._MARCH_TAIL_GAP`).
