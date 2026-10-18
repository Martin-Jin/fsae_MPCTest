"""
Check a cone map's spacing against the distance the cones were laid at.

Joins each row of cones (blue, yellow) into a chain of edges, neighbour to neighbour, and
compares every edge length with REFERENCE_SPACING_M. Use it on a map rebuilt from a rosbag
(`rosbag_cone_map`) to judge the map, or on any track to check how it was laid out.

How to read the result
----------------------
Terms: an *edge* is the line between two neighbouring cones of one row, and its length
is their gap. *Median* is the middle edge length once sorted (half are shorter). *p10* and
*p90* are the lengths 10 % and 90 % of edges are at or below, so "p10-p90" is the range the
middle 80 % fall in. *Median error* is the median length minus the reference.

- **On spacing (within TOLERANCE_M):** the map agrees with the layout there.
- **Short edge, under DUPLICATE_M:** almost always one cone mapped twice (a drift or
  association failure), not two real cones.
- **Long edge, over MISSING_FACTOR x the reference:** a cone missing from the map, or a
  real gap in the layout (a start gate, a corner laid wider).
- **Spread inside tolerance** is usually how the cones were placed by hand. To tell
  placement from map error, compare two maps of the same layout: the same edge with the
  same length in both is placement; a different length is map error.
- **The reference is only as good as the layout.** A spacing estimated by eye checks
  consistency, not absolute accuracy. Tape a few neighbouring cones for a real reference.

How the rows are chained
------------------------
Greedy nearest neighbour: start at the cone nearest the map origin (where the recording
started), repeatedly step to the nearest unvisited cone of the same colour within
MAX_EDGE_FACTOR x the reference, then close the loop back to the start if it is that close.
A row that cannot be walked in one chain (a cone too far from the rest, or an open track)
is reported as several chains. Hairpins tighter than the spacing can make the chain jump
across a corner; the plot shows that at a glance.

Usage
-----
    python3 -m tuner.tools.cone_spacing_check gokart_night_loop_20261018
    python3 -m tuner.tools.cone_spacing_check <track or cone_map.json> --reference 3.0 --tolerance 0.5
    python3 -m tuner.tools.cone_spacing_check <track> --colours yellow --plot spacing.png --csv edges.csv
    python3 -m tuner.tools.cone_spacing_check <track> --width-reference 3.5   # also check track width
"""
import argparse
import csv
import json
import math

import numpy as np

import tracks

# The spacing the cones were laid at, along each row (m). The 2026-10-18 go-kart loop was
# laid at about 3 m, estimated by eye.
REFERENCE_SPACING_M = 3.0
# An edge within this of the reference counts as on spacing (m).
TOLERANCE_M = 0.5
# Shorter than this is flagged as a probable duplicate cone (m).
DUPLICATE_M = 1.0
# Longer than this x the reference is flagged as a probable missing cone.
MISSING_FACTOR = 1.5
# The chain never joins two cones farther apart than this x the reference.
MAX_EDGE_FACTOR = 2.5

ROW_COLOURS = ('blue', 'yellow')


def load_rows(map_arg):
    path = tracks.resolve_map_arg(map_arg)
    with open(path) as f:
        payload = json.load(f)
    rows = {c: np.array(payload.get(c, []), dtype=float).reshape(-1, 2) for c in ROW_COLOURS}
    return path, rows


def chain_row(points, max_edge):
    """Greedy nearest-neighbour chains through one row. Returns a list of chains, each a
    list of point indices; a chain ending with its own first index is a closed loop."""
    remaining = set(range(len(points)))
    chains = []
    while remaining:
        idx = sorted(remaining)
        start = min(idx, key=lambda i: np.hypot(*points[i]))
        chain = [start]
        remaining.discard(start)
        while remaining:
            cur = points[chain[-1]]
            cand = sorted(remaining)
            d = np.hypot(*(points[cand] - cur).T)
            k = int(np.argmin(d))
            if d[k] > max_edge:
                break
            chain.append(cand[k])
            remaining.discard(cand[k])
        if len(chain) > 2 and np.hypot(*(points[chain[-1]] - points[chain[0]])) <= max_edge:
            chain.append(chain[0])
        chains.append(chain)
    return chains


def row_edges(points, chains):
    """[(i, j, length)] for consecutive cones in every chain."""
    out = []
    for ch in chains:
        for i, j in zip(ch[:-1], ch[1:]):
            out.append((i, j, float(np.hypot(*(points[j] - points[i])))))
    return out


def classify(length, reference, tolerance):
    if length < DUPLICATE_M:
        return 'duplicate?'
    if length > MISSING_FACTOR * reference:
        return 'missing cone?'
    if abs(length - reference) <= tolerance:
        return 'ok'
    return 'short' if length < reference else 'long'


def width_edges(rows):
    """Each yellow cone to its nearest blue cone: a lower bound on the track width there
    (the nearest blue cone is often diagonally across)."""
    b, y = rows['blue'], rows['yellow']
    if not len(b) or not len(y):
        return []
    out = []
    for i, p in enumerate(y):
        d = np.hypot(*(b - p).T)
        j = int(np.argmin(d))
        out.append((i, j, float(d[j])))
    return out


def summarise(lengths, reference, tolerance):
    a = np.array(lengths)
    labels = [classify(v, reference, tolerance) for v in a]
    return {
        'n': len(a),
        'median': float(np.median(a)), 'mean': float(a.mean()),
        'p10': float(np.percentile(a, 10)), 'p90': float(np.percentile(a, 90)),
        'min': float(a.min()), 'max': float(a.max()),
        'median_error': float(np.median(a - reference)),
        'ok': labels.count('ok'),
        'duplicate': labels.count('duplicate?'), 'missing': labels.count('missing cone?'),
    }


def plot(path_png, rows, edges, reference, tolerance, title, widths=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colour_of = {'ok': 'tab:green', 'short': 'tab:orange', 'long': 'tab:orange',
                 'duplicate?': 'tab:red', 'missing cone?': 'tab:purple'}
    fig, ax = plt.subplots(figsize=(9, 9))
    if widths:
        for i, j, _ in widths:
            p, q = rows['yellow'][i], rows['blue'][j]
            ax.plot([p[0], q[0]], [p[1], q[1]], ':', color='0.75', lw=0.8)
    for c, es in edges.items():
        pts = rows[c]
        for i, j, length in es:
            lab = classify(length, reference, tolerance)
            p, q = pts[i], pts[j]
            ax.plot([p[0], q[0]], [p[1], q[1]], '-', color=colour_of[lab], lw=2)
            ax.annotate(f'{length:.2f}', ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2),
                        fontsize=7, ha='center', va='center',
                        bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.7))
    for c, col in (('blue', 'tab:blue'), ('yellow', 'gold')):
        if len(rows[c]):
            ax.scatter(rows[c][:, 0], rows[c][:, 1], s=45, c=col, edgecolors='k', zorder=3)
    for lab, col in (('ok', 'tab:green'), ('outside tolerance', 'tab:orange'),
                     (f'< {DUPLICATE_M} m (duplicate?)', 'tab:red'),
                     (f'> {MISSING_FACTOR}x ref (missing cone?)', 'tab:purple')):
        ax.plot([], [], '-', color=col, lw=2, label=lab)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    ax.set_title(f'{title}\nreference {reference:.2f} m, tolerance {tolerance:.2f} m', fontsize=9)
    fig.tight_layout()
    fig.savefig(path_png, dpi=120)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('map', nargs='?', default=None,
                    help='track name or cone_map.json path (default: newest FSDS track)')
    ap.add_argument('--reference', type=float, default=REFERENCE_SPACING_M,
                    help=f'laid spacing along a row, m (default {REFERENCE_SPACING_M})')
    ap.add_argument('--tolerance', type=float, default=TOLERANCE_M,
                    help=f'an edge within this of the reference is on spacing, m (default {TOLERANCE_M})')
    ap.add_argument('--colours', nargs='+', choices=ROW_COLOURS, default=list(ROW_COLOURS))
    ap.add_argument('--width-reference', type=float,
                    help='also check yellow-to-nearest-blue distance against this track width, m')
    ap.add_argument('--plot', help='write a PNG of the edges, coloured by result')
    ap.add_argument('--csv', help='write every edge to a CSV')
    a = ap.parse_args(argv)

    path, rows = load_rows(a.map)
    max_edge = MAX_EDGE_FACTOR * a.reference
    print(f'{path}\nreference {a.reference:.2f} m, tolerance {a.tolerance:.2f} m')
    print('(median: half the edges at or below; p10-p90: range of the middle 80 % of edges; '
          'error: median minus reference)')
    edges = {}
    records = []
    for c in a.colours:
        pts = rows[c]
        if len(pts) < 2:
            print(f'  {c:6s} fewer than 2 cones, skipped')
            continue
        chains = chain_row(pts, max_edge)
        es = row_edges(pts, chains)
        edges[c] = es
        s = summarise([e[2] for e in es], a.reference, a.tolerance)
        closed = sum(1 for ch in chains if len(ch) > 2 and ch[0] == ch[-1])
        print(f'  {c:6s} {len(pts):3d} cones, {s["n"]} edges in {len(chains)} chain(s) '
              f'({closed} closed): median {s["median"]:.2f} m (error {s["median_error"]:+.2f}), '
              f'p10-p90 {s["p10"]:.2f}-{s["p90"]:.2f}, min {s["min"]:.2f}, max {s["max"]:.2f}')
        print(f'         on spacing {s["ok"]}/{s["n"]}, probable duplicates {s["duplicate"]}, '
              f'probable missing cones {s["missing"]}')
        for i, j, length in es:
            lab = classify(length, a.reference, a.tolerance)
            records.append((c, i, j, *pts[i], *pts[j], length, length - a.reference, lab))
            if lab in ('duplicate?', 'missing cone?'):
                print(f'         {lab:13s} {length:5.2f} m between ({pts[i][0]:.1f}, {pts[i][1]:.1f}) '
                      f'and ({pts[j][0]:.1f}, {pts[j][1]:.1f})')
    widths = None
    if a.width_reference is not None:
        widths = width_edges(rows)
        if widths:
            s = summarise([w[2] for w in widths], a.width_reference, a.tolerance)
            print(f'  width  yellow to nearest blue (a lower bound): median {s["median"]:.2f} m '
                  f'vs {a.width_reference:.2f}, p10-p90 {s["p10"]:.2f}-{s["p90"]:.2f}')
            for i, j, length in widths:
                p, q = rows['yellow'][i], rows['blue'][j]
                records.append(('width', i, j, *p, *q, length, length - a.width_reference,
                                classify(length, a.width_reference, a.tolerance)))
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['row', 'i', 'j', 'x_i', 'y_i', 'x_j', 'y_j', 'length_m', 'error_m', 'result'])
            w.writerows(records)
        print(f'wrote {a.csv}')
    if a.plot:
        plot(a.plot, rows, edges, a.reference, a.tolerance, path, widths)
        print(f'wrote {a.plot}')


if __name__ == '__main__':
    main()
