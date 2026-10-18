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

SLAM mode (`--slam`, preferred)
-------------------------------
Steps 1-5 trust the odometry completely. With `--slam` the input is instead a bag made by
`rosbag_slam_replay.sh`, which replays the recording through fsae_autonomous's
`cone_landmark_mapper`. That node corrects the pose every time a known cone is re-seen,
and its final `/fsae/slam/cone_map` message is taken as the map. On the 2026-10-18 kart
bags its cone-to-cone distances match single-frame measurements to about 1 cm, against
4-8 cm for steps 1-5 (docs/guides/offline_guide.md, "Results on the 2026-10-18 kart bags").
It still fails after an odometry jump: the node has no relocalisation.

What steps 1-5 cannot fix
-------------------------
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
    python3 -m tuner.tools.rosbag_cone_map <bag_dir> --name <name> --plot preview.png \
        --spacing-reference 3.0 --note "any extra result line"     # edges + results box
    python3 -m tuner.tools.rosbag_cone_map <bag_dir> --out /tmp/cone_map.json   # no track dir
    python3 -m tuner.tools.rosbag_cone_map <slam_replay_bag> --slam --name <name>

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
SLAM_TOPIC = '/fsae/slam/cone_map'
SLAM_PATH_TOPIC = '/fsae/slam/path'
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

    def u8(self):
        v = self.buf[self.pos]
        self.pos += 1
        return v

    def f32(self):
        self._align(4)
        (v,) = struct.unpack_from('<f', self.buf, self.pos)
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


def _decode_cone_map_stamped(data: bytes):
    """fsae_interfaces/ConeMapStamped: header, then Cone[] (id, confidence, colour,
    PoseWithCovariance, radius, height). Returns [(x, y, colour, confidence, var_x, var_y)]."""
    r = _Cdr(data)
    r.i32(), r.u32()
    r.string()
    out = []
    for _ in range(r.u32()):
        r.u32()                                  # id
        conf = r.f32()                           # cone_landmark_mapper: observation count
        colour = r.u8()                          # 0 blue, 1 orange, 2 yellow, 3 other
        x, y, _z = r.f64s(3)
        r.f64s(4)                                # orientation
        cov = r.f64s(36)
        r.f32(), r.f32()                         # radius, height
        out.append((x, y, colour, conf, cov[0], cov[7]))
    return out


def _decode_path(data: bytes):
    """nav_msgs/Path: header, then PoseStamped[]. Returns an (N, 2) array of x, y."""
    r = _Cdr(data)
    r.i32(), r.u32()
    r.string()
    xy = []
    for _ in range(r.u32()):
        r.i32(), r.u32()
        r.string()
        x, y, _z = r.f64s(3)
        r.f64s(4)
        xy.append((x, y))
    return np.array(xy).reshape(-1, 2)


def _messages(bag_dir: str, topic: str):
    """Raw CDR payloads on `topic`, in time order across all of the bag's split files."""
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
            yield bytes(data)
        con.close()


def read_detections(bag_dir: str, topic: str = DEFAULT_TOPIC):
    """Yield (t, (x, y, yaw), {colour: (N, 2) local}) for every message on `topic`."""
    for data in _messages(bag_dir, topic):
        yield _decode_cone_detection(data)


def read_slam_map(bag_dir: str, topic: str = SLAM_TOPIC, min_obs=3):
    """The LAST cone map on `topic` (the SLAM node's final estimate), as the same
    {colour: [(xy, count, spread_m)]} structure build_cone_map returns. spread is the
    landmark's marginal standard deviation, sqrt of the larger axis variance."""
    last = None
    n_msgs = 0
    for data in _messages(bag_dir, topic):
        last = data
        n_msgs += 1
    if last is None:
        raise ValueError(f'no messages on {topic} in {bag_dir}')
    names = {0: 'blue', 1: 'orange', 2: 'yellow'}
    cones = {'blue': [], 'yellow': [], 'orange': []}
    for x, y, colour, conf, vx, vy in _decode_cone_map_stamped(last):
        if colour in names and conf >= min_obs:
            cones[names[colour]].append((np.array([x, y]), int(conf), math.sqrt(max(vx, vy, 0.0))))
    path = np.empty((0, 2))
    for data in _messages(bag_dir, SLAM_PATH_TOPIC):
        path = data
    if not isinstance(path, np.ndarray):
        path = _decode_path(path)
    return cones, {'messages': n_msgs, 'detections_used': 0, 'colour_conflicts': 0,
                   'path': path}


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


def plot(path_png, cones, stats, title, spacing_reference=None, notes=()):
    """Cones over the vehicle path. With spacing_reference, also each row's edges from
    cone_spacing_check (coloured by result, labelled with their length) and a box with the
    spacing summary; `notes` adds lines to that box (e.g. accuracy checks run elsewhere)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 9))
    p = stats['path']
    ax.plot(p[:, 0], p[:, 1], '-', color='0.7', lw=0.8, label=stats.get('path_label', 'vehicle path'))
    box = []
    if spacing_reference is not None:
        from tuner.tools import cone_spacing_check as csc
        colour_of = {'ok': 'tab:green', 'short': 'tab:orange', 'long': 'tab:orange',
                     'duplicate?': 'tab:red', 'missing cone?': 'tab:purple'}
        box.append(f'Row spacing vs {spacing_reference:.2f} m laid '
                   f'(on spacing = within {csc.TOLERANCE_M:.1f} m):')
        for g in ('yellow', 'blue'):
            if len(cones[g]) < 2:
                continue
            pts = np.array([c[0] for c in cones[g]])
            es = csc.row_edges(pts, csc.chain_row(pts, csc.MAX_EDGE_FACTOR * spacing_reference))
            for i, j, length in es:
                lab = csc.classify(length, spacing_reference, csc.TOLERANCE_M)
                ax.plot(*np.array([pts[i], pts[j]]).T, '-', color=colour_of[lab], lw=1.8, zorder=1)
                ax.annotate(f'{length:.2f}', tuple((pts[i] + pts[j]) / 2), fontsize=6,
                            ha='center', va='center',
                            bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.7))
            sm = csc.summarise([e[2] for e in es], spacing_reference, csc.TOLERANCE_M)
            box.append(f'  {g}: median {sm["median"]:.2f} m, p10-p90 {sm["p10"]:.2f}-{sm["p90"]:.2f}, '
                       f'on spacing {sm["ok"]}/{sm["n"]}, duplicates {sm["duplicate"]}, '
                       f'missing {sm["missing"]}')
        for lab, col in (('edge on spacing', 'tab:green'), ('edge outside tolerance', 'tab:orange'),
                         ('probable duplicate', 'tab:red'), ('probable missing cone', 'tab:purple')):
            ax.plot([], [], '-', color=col, lw=1.8, label=lab)
    box.extend(notes)
    if box:
        # Below the axes, so it never hides cones.
        ax.text(0.0, -0.08, '\n'.join(box), transform=ax.transAxes, fontsize=8, va='top',
                family='monospace', bbox=dict(boxstyle='round', fc='white', ec='0.6'))
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
    fig.savefig(path_png, dpi=120, bbox_inches='tight')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('bag_dir')
    dest = ap.add_mutually_exclusive_group(required=True)
    dest.add_argument('--name', help='track base name; written to TRACKS_DIR/<name>_<bag date>/')
    dest.add_argument('--out', help='explicit cone_map.json path')
    ap.add_argument('--topic', default=DEFAULT_TOPIC)
    ap.add_argument('--slam', action='store_true',
                    help=f'take the final SLAM map ({SLAM_TOPIC}) instead of clustering detections')
    ap.add_argument('--min-range', type=float, default=1.0)
    ap.add_argument('--max-range', type=float, default=7.0)
    ap.add_argument('--merge-radius', type=float, default=0.75)
    ap.add_argument('--min-obs', type=int, default=None,
                    help='minimum sightings per cone (default 5; 3 with --slam, its own promote_after)')
    ap.add_argument('--open', action='store_true', help='mark the track as not a closed loop')
    ap.add_argument('--plot', help='write a PNG preview here')
    ap.add_argument('--spacing-reference', type=float,
                    help='with --plot: draw row edges and their check against this laid spacing, m')
    ap.add_argument('--note', action='append', default=[],
                    help='with --plot: a line for the results box (repeatable)')
    a = ap.parse_args(argv)

    bag_dir = os.path.abspath(os.path.expanduser(a.bag_dir))
    if a.slam:
        cones, stats = read_slam_map(bag_dir, SLAM_TOPIC, 3 if a.min_obs is None else a.min_obs)
        stats['path_label'] = 'vehicle path (SLAM)'
        topic = SLAM_TOPIC
    else:
        cones, stats = build_cone_map(bag_dir, a.topic, a.min_range, a.max_range,
                                      a.merge_radius, 5 if a.min_obs is None else a.min_obs)
        stats['path_label'] = 'vehicle path (odometry)'
        topic = a.topic
    if a.out:
        out = a.out
    else:
        name = tracks.dated_track_name(a.name, bag_date(bag_dir))
        out = os.path.join(tracks.TRACKS_DIR, name, tracks.CONE_MAP_NAME)
    source = f'rosbag:{os.path.basename(bag_dir)} topic:{topic}'
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
        plot(a.plot, cones, stats, source, a.spacing_reference, a.note)
        print(f'wrote {a.plot}')


if __name__ == '__main__':
    main()
