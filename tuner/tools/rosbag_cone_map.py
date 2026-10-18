"""
Rebuild a track's cone map from a real-vehicle rosbag.

The simulator's cone_recorder writes `cone_map.json` from FSDS ground truth. A real
vehicle has no ground truth, but every `fsae_interfaces/ConeDetection` message carries
the cones in the vehicle frame together with the pose they were seen from
(`car_pose`, yaw in `orientation.w`, the repo convention). Projecting every detection
into the odometry frame and clustering the repeats gives a cone map in the same JSON
format, so the offline tools (`sim.track_io`, `export_speed_profile`,
`raceline_optimizer`, `gui/simulation.py`) can load a track the go-kart actually drove.

How the map is built
--------------------
1. Read `/cone_detection_fused` (LiDAR-refined; `--topic` to change) straight from the
   bag's sqlite3 files. No ROS install is needed: the message is decoded by hand
   (`_decode_cone_detection`), so this runs anywhere numpy does.
2. Keep detections between `--min-range` and `--max-range` of the camera. Range error
   grows with the square of distance, and the kart's camera only sees about two cones
   ahead confidently, so far detections are dropped rather than averaged in.
3. Rotate and translate each detection into the odometry frame with its own `car_pose`.
4. Cluster per colour: a detection joins the nearest cluster within `--merge-radius`
   (below half the cone spacing, so neighbouring cones never merge), else starts one.
   A cluster's position is the median of its detections, which ignores the odd bad
   range reading that a mean would follow.
5. Drop clusters seen fewer than `--min-obs` times (one-off false positives), and where
   a blue and a yellow cluster sit on the same spot keep only the better-supported one
   (a cone the classifier sometimes calls the other colour).

What it cannot fix
------------------
- Odometry drift. The pose is visual-inertial odometry, never corrected by the cones, so
  cones re-seen on a later lap land slightly shifted. Small drift widens a cluster
  (reported as `spread`); drift past `--merge-radius` duplicates cones. Check the
  printed per-lap drift and the plot before trusting a map.
- One bag, one frame. Each stack start puts the odometry origin where the vehicle was,
  so maps from different runs are in different frames. Build one map per bag.
- Orange cones are kept under an `orange` key that the simulator loaders ignore.

Usage
-----
    python3 -m tuner.tools.rosbag_cone_map ~/fsae_data/bags/<bag_dir> --name gokart_night_loop
    python3 -m tuner.tools.rosbag_cone_map <bag_dir> --name <name> --plot preview.png
    python3 -m tuner.tools.rosbag_cone_map <bag_dir> --out /tmp/cone_map.json   # no track dir

`--name` writes `<TRACKS_DIR>/<name>_<YYYYmmdd of the bag>/cone_map.json` (see
`tracks/__init__.py`); `--out` writes anywhere.
"""
import argparse
import datetime
import glob
import json
import math
import os
import sqlite3
import struct

import numpy as np

import tracks

DEFAULT_TOPIC = '/cone_detection_fused'
COLOURS = ('yellow', 'blue', 'small_orange', 'big_orange')   # message field order


# ── Reading the bag ──────────────────────────────────────────────────────────

class _Cdr:
    """Minimal little-endian CDR reader. Alignment is relative to the payload start,
    just after the 4-byte encapsulation header."""

    def __init__(self, data: bytes):
        if data[1] != 0x01:
            raise ValueError('only little-endian CDR is supported')
        self.buf = data[4:]
        self.pos = 0

    def _align(self, n):
        self.pos += (-self.pos) % n

    def u32(self):
        self._align(4)
        (v,) = struct.unpack_from('<I', self.buf, self.pos)
        self.pos += 4
        return v

    def i32(self):
        self._align(4)
        (v,) = struct.unpack_from('<i', self.buf, self.pos)
        self.pos += 4
        return v

    def f64s(self, n):
        self._align(8)
        v = struct.unpack_from(f'<{n}d', self.buf, self.pos)
        self.pos += 8 * n
        return v

    def string(self):
        n = self.u32()
        s = self.buf[self.pos:self.pos + n - 1].decode()
        self.pos += n
        return s


def _decode_cone_detection(data: bytes):
    """fsae_interfaces/ConeDetection: header, car_pose, then yellow, blue,
    small_orange, big_orange as sequences of geometry_msgs/Point."""
    r = _Cdr(data)
    sec, nsec = r.i32(), r.u32()
    r.string()                                   # frame_id
    px, py, _pz = r.f64s(3)
    _qx, _qy, _qz, qw = r.f64s(4)                # yaw is stored in orientation.w
    cones = {}
    for colour in COLOURS:
        n = r.u32()
        cones[colour] = np.array(r.f64s(3 * n)).reshape(n, 3)[:, :2] if n else np.empty((0, 2))
    return sec + nsec * 1e-9, (px, py, qw), cones


def read_detections(bag_dir: str, topic: str = DEFAULT_TOPIC):
    """Yield (t, (x, y, yaw), {colour: (N, 2) local}) for every message on `topic`,
    in time order across all of the bag's split files."""
    files = sorted(glob.glob(os.path.join(bag_dir, '*.db3')),
                   key=lambda f: int(os.path.splitext(f)[0].rsplit('_', 1)[-1]))
    if not files:
        raise FileNotFoundError(f'no .db3 files in {bag_dir} (only sqlite3 bags are supported)')
    for f in files:
        con = sqlite3.connect(f'file:{f}?mode=ro', uri=True)
        row = con.execute('SELECT id FROM topics WHERE name = ?', (topic,)).fetchone()
        if row is None:
            con.close()
            continue
        for (data,) in con.execute(
                'SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp', (row[0],)):
            yield _decode_cone_detection(bytes(data))
        con.close()


# ── Building the map ─────────────────────────────────────────────────────────

class _Clusters:
    def __init__(self, radius):
        self.radius = radius
        self.centre = []        # running mean, used only for assignment
        self.points = []        # every member, for the median and the spread

    def add(self, p):
        if self.centre:
            c = np.array(self.centre)
            d = np.hypot(c[:, 0] - p[0], c[:, 1] - p[1])
            i = int(np.argmin(d))
            if d[i] < self.radius:
                self.points[i].append(p)
                n = len(self.points[i])
                self.centre[i] = c[i] + (p - c[i]) / n
                return
        self.centre.append(p.copy())
        self.points.append([p])

    def summary(self, min_obs):
        """[(median_xy, count, spread_m)] for clusters with at least min_obs members."""
        out = []
        for pts in self.points:
            if len(pts) < min_obs:
                continue
            a = np.array(pts)
            med = np.median(a, axis=0)
            spread = float(np.median(np.hypot(*(a - med).T)))
            out.append((med, len(pts), spread))
        return out


def build_cone_map(bag_dir, topic=DEFAULT_TOPIC, min_range=1.0, max_range=7.0,
                   merge_radius=0.75, min_obs=5):
    """Returns (cones, stats). cones: {colour: [(xy, count, spread)]} with colour in
    blue / yellow / orange."""
    groups = {'blue': ('blue',), 'yellow': ('yellow',), 'orange': ('small_orange', 'big_orange')}
    cl = {g: _Clusters(merge_radius) for g in groups}
    n_msgs = n_kept = 0
    poses = []
    for _t, (x, y, yaw), cones in read_detections(bag_dir, topic):
        n_msgs += 1
        poses.append((x, y))
        c, s = math.cos(yaw), math.sin(yaw)
        for g, fields in groups.items():
            for f in fields:
                for lx, ly in cones[f]:
                    r = math.hypot(lx, ly)
                    if not (min_range <= r <= max_range):
                        continue
                    cl[g].add(np.array([x + c * lx - s * ly, y + s * lx + c * ly]))
                    n_kept += 1
    if n_msgs == 0:
        raise ValueError(f'no messages on {topic} in {bag_dir}')
    out = {g: cl[g].summary(min_obs) for g in groups}
    out['blue'], out['yellow'], n_conflicts = _resolve_colour_conflicts(
        out['blue'], out['yellow'], merge_radius)
    stats = {'messages': n_msgs, 'detections_used': n_kept, 'colour_conflicts': n_conflicts,
             'path': np.array(poses)}
    return out, stats


def _resolve_colour_conflicts(blue, yellow, radius):
    """Where a blue and a yellow cone sit within `radius`, keep the one seen more."""
    drop_b, drop_y = set(), set()
    for i, (pb, nb, _) in enumerate(blue):
        for j, (py_, ny, _) in enumerate(yellow):
            if np.hypot(*(pb - py_)) < radius:
                (drop_y if nb >= ny else drop_b).add(j if nb >= ny else i)
    keep_b = [c for i, c in enumerate(blue) if i not in drop_b]
    keep_y = [c for j, c in enumerate(yellow) if j not in drop_y]
    return keep_b, keep_y, len(drop_b) + len(drop_y)


def bag_date(bag_dir):
    """Local date of the bag's first message, from metadata.yaml's start time."""
    meta = os.path.join(bag_dir, 'metadata.yaml')
    with open(meta) as f:
        for line in f:
            if 'nanoseconds_since_epoch' in line:
                ns = int(line.split(':')[1])
                return datetime.datetime.fromtimestamp(ns * 1e-9).date()
    return datetime.date.today()


def write_cone_map(path, cones, source, lap_closed=True):
    payload = {
        'blue': [[float(v) for v in c[0]] for c in cones['blue']],
        'yellow': [[float(v) for v in c[0]] for c in cones['yellow']],
        'orange': [[float(v) for v in c[0]] for c in cones['orange']],
        'source': source,
        'lap_closed': lap_closed,
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(payload, f, indent=2)


def plot(path_png, cones, stats, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 8))
    p = stats['path']
    ax.plot(p[:, 0], p[:, 1], '-', color='0.7', lw=0.8, label='vehicle path (odometry)')
    for g, col in (('blue', 'tab:blue'), ('yellow', 'gold'), ('orange', 'tab:orange')):
        if cones[g]:
            xy = np.array([c[0] for c in cones[g]])
            ax.scatter(xy[:, 0], xy[:, 1], s=40, c=col, edgecolors='k', label=f'{g} ({len(xy)})')
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    ax.set_title(title, fontsize=9)
    ax.set_xlabel('x (m, odometry frame)')
    ax.set_ylabel('y (m)')
    fig.tight_layout()
    fig.savefig(path_png, dpi=120)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('bag_dir')
    dest = ap.add_mutually_exclusive_group(required=True)
    dest.add_argument('--name', help='track base name; written to TRACKS_DIR/<name>_<bag date>/')
    dest.add_argument('--out', help='explicit cone_map.json path')
    ap.add_argument('--topic', default=DEFAULT_TOPIC)
    ap.add_argument('--min-range', type=float, default=1.0)
    ap.add_argument('--max-range', type=float, default=7.0)
    ap.add_argument('--merge-radius', type=float, default=0.75)
    ap.add_argument('--min-obs', type=int, default=5)
    ap.add_argument('--open', action='store_true', help='mark the track as not a closed loop')
    ap.add_argument('--plot', help='write a PNG preview here')
    a = ap.parse_args(argv)

    bag_dir = os.path.abspath(os.path.expanduser(a.bag_dir))
    cones, stats = build_cone_map(bag_dir, a.topic, a.min_range, a.max_range,
                                  a.merge_radius, a.min_obs)
    if a.out:
        out = a.out
    else:
        name = tracks.dated_track_name(a.name, bag_date(bag_dir))
        out = os.path.join(tracks.TRACKS_DIR, name, tracks.CONE_MAP_NAME)
    source = f'rosbag:{os.path.basename(bag_dir)} topic:{a.topic}'
    write_cone_map(out, cones, source, lap_closed=not a.open)

    print(f'{stats["messages"]} messages, {stats["detections_used"]} detections used, '
          f'{stats["colour_conflicts"]} blue/yellow conflicts resolved')
    for g in ('blue', 'yellow', 'orange'):
        if cones[g]:
            n = np.array([c[1] for c in cones[g]])
            sp = np.array([c[2] for c in cones[g]])
            print(f'  {g:6s} {len(n):3d} cones, sightings median {np.median(n):.0f}, '
                  f'spread median {np.median(sp):.2f} m, max {sp.max():.2f} m')
    print(f'wrote {out}')
    if a.plot:
        plot(a.plot, cones, stats, source)
        print(f'wrote {a.plot}')


if __name__ == '__main__':
    main()
