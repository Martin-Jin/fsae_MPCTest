"""
Interactive time-scrubbing playback of one control telemetry log.

Reads the (control_csv, path_csv) pair `fsae_control/telemetry_logger.py`
writes (see that module's docstring for the schema) and shows, side by side:

  - left:  the scored signals (e_y, e_psi_deg, kappa, steer_deg, v) stacked
           on a shared time axis, with a vertical cursor marking "now"
  - top right:  the full driven trajectory (and, if available, the planner's
           most-recent path snapshot at "now") with a triangle marking the
           car's current position/heading
  - bottom right: the same scene zoomed tightly to the car's current
           section of track, so heading and lateral error are easy to read
           at a glance

A slider under the metrics panel scrubs "now" through the run; dragging it
updates the cursor, the triangle, and the zoom window together.

This is presentation/analysis tooling only -- it never touches a live
controller, just the CSVs it wrote after the fact.

Usage
-----
    python -m tuner.plot_playback <control_csv>

    # no CSV given -> auto-load the newest log in RECORDED_RUNS_DIR
    # (fsds_simulator/recorded_runs/ -- see plot_control_log.py's constant)
    python -m tuner.plot_playback

    # explicit signal set for the left-hand metrics panel
    python -m tuner.plot_playback run.csv --signals e_y,e_psi_deg,yaw_rate

The sibling `<tag>_path_<stamp>.csv` next to the control CSV is loaded
automatically if present, to draw the planner's path as it looked at each
moment in time. If it's missing (e.g. a Stanley log, or an older run from
before log_path() existed), the map/zoom views fall back to showing just the
car's own driven trajectory (car_x/car_y) with no live path overlay.
"""
import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

from tuner import csv_log
from tuner.plot_control_log import (
    DEFAULT_SIGNALS, DERIVED_PAIRS, RECORDED_RUNS_DIR, _label_for,
    find_latest_log, load_log,
)

CAR_TRIANGLE = np.array([
    [1.0, 0.0],
    [-0.6, 0.5],
    [-0.6, -0.5],
])
CAR_LENGTH_M = 3.0   # visual size only, not the real wheelbase
ZOOM_HALF_WIDTH_M = 12.0   # how far ahead/behind/either-side the zoom view shows


def _path_csv_for(control_csv_path):
    """Return the sibling `<tag>_path_<stamp>.csv` path, or None if absent.

    telemetry_logger.py always names the two files `{tag}_control_{stamp}.csv`
    / `{tag}_path_{stamp}.csv` in the same directory (see its `paths`
    property) -- this just re-derives the second name from the first.
    """
    d = os.path.dirname(control_csv_path)
    base = os.path.basename(control_csv_path)
    if '_control_' not in base:
        return None
    candidate = os.path.join(d, base.replace('_control_', '_path_', 1))
    return candidate if os.path.exists(candidate) else None


def _load_path_snapshots(path_csv):
    """Return (snapshot_ts, list_of_(x_array, y_array)) sorted by time.

    Each row in the path CSV is one point of one snapshot; rows sharing the
    same `t` are one polyline as the planner saw it at that instant (see
    telemetry_logger.py's log_path()). Grouped here so playback can pick
    "the most recent snapshot at or before now" in O(log n).
    """
    cols = csv_log.load_columns(path_csv)
    order = np.argsort(cols['t'], kind='stable')
    t, x, y = cols['t'][order], cols['x'][order], cols['y'][order]

    snapshot_ts = []
    snapshot_xy = []
    start = 0
    for i in range(1, len(t) + 1):
        if i == len(t) or t[i] != t[start]:
            snapshot_ts.append(t[start])
            snapshot_xy.append((x[start:i], y[start:i]))
            start = i
    return np.array(snapshot_ts), snapshot_xy


def _snapshot_at_or_before(snapshot_ts, snapshot_xy, t_now):
    """Return the (x, y) polyline of the latest snapshot with t <= t_now.

    Falls back to the earliest snapshot if `t_now` precedes every one (can
    happen for the first fraction of a second before the first path log).
    Returns (None, None) if there are no snapshots at all.
    """
    if len(snapshot_ts) == 0:
        return None, None
    idx = np.searchsorted(snapshot_ts, t_now, side='right') - 1
    idx = max(idx, 0)
    return snapshot_xy[idx]


def _rotate(points, yaw_rad):
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T


class Playback:
    """Owns the figure and all the little bits of mutable plot state.

    Kept as a class (rather than a pile of closures) purely so the widgets
    (Slider) have somewhere to hold their callbacks without falling out of
    scope -- matplotlib widgets stop working the moment their last Python
    reference is garbage collected.
    """

    def __init__(self, control_csv, signals=DEFAULT_SIGNALS):
        self.cols, self.meta = load_log(control_csv)
        if 't' not in self.cols:
            sys.exit(f'{control_csv}: no `t` column -- not a control telemetry CSV.')
        if 'car_x' not in self.cols or 'car_y' not in self.cols:
            sys.exit(f'{control_csv}: no car_x/car_y columns -- cannot show a map view.')

        self.label = _label_for(control_csv, self.meta)
        self.signals = signals
        self.t = self.cols['t']
        self.n = len(self.t)

        path_csv = _path_csv_for(control_csv)
        if path_csv is not None:
            self.snapshot_ts, self.snapshot_xy = _load_path_snapshots(path_csv)
            print(f'Loaded {len(self.snapshot_ts)} path snapshot(s) from {path_csv}')
        else:
            self.snapshot_ts, self.snapshot_xy = np.array([]), []
            print('No sibling path CSV found -- map/zoom views will show the '
                  'driven trajectory only, with no live planner-path overlay.')

        self._build_figure()
        self._draw_static()
        self._update(0)

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------
    def _build_figure(self):
        self.fig = plt.figure(figsize=(15, 9))
        self.fig.suptitle(f'Playback — {self.label}', fontsize=11)

        # Left column: one row per signal, plus a slider row underneath.
        # Right column: map (top) over a square zoomed view (bottom).
        # A 2-column outer grid keeps the left column's width independent of
        # how tall the right column's two stacked panels are.
        outer = self.fig.add_gridspec(
            1, 2, width_ratios=[1.15, 1.0], wspace=0.28,
            left=0.09, right=0.98, top=0.93, bottom=0.08,
        )
        n_rows = len(self.signals)
        left = outer[0].subgridspec(
            n_rows + 1, 1, height_ratios=[*([1] * n_rows), 0.35], hspace=0.12,
        )
        self.signal_axes = [self.fig.add_subplot(left[i, 0]) for i in range(n_rows)]
        for ax in self.signal_axes[:-1]:
            ax.tick_params(labelbottom=False)
        self.slider_ax = self.fig.add_subplot(left[n_rows, 0])

        right = outer[1].subgridspec(2, 1, height_ratios=[1, 1], hspace=0.22)
        self.map_ax = self.fig.add_subplot(right[0, 0])
        self.zoom_ax = self.fig.add_subplot(right[1, 0])

    # ------------------------------------------------------------------
    # One-time drawing: everything that doesn't change as the slider moves
    # ------------------------------------------------------------------
    def _draw_static(self):
        for ax, sig in zip(self.signal_axes, self.signals):
            self._plot_signal(ax, sig)
        self.signal_axes[-1].set_xlabel('t (s)', fontsize=9)

        # Fixed-overflow y-axis labels: rotate to horizontal and give the
        # figure enough left margin (set in _build_figure) rather than
        # letting matplotlib's default vertical ylabel run into the tick
        # labels of the row above/below it.
        for ax, sig in zip(self.signal_axes, self.signals):
            ax.set_ylabel(sig, fontsize=8, rotation=0, ha='right', va='center',
                          labelpad=8)
            ax.tick_params(axis='both', labelsize=7)
            ax.grid(True, alpha=0.25)

        self.cursors = [
            ax.axvline(self.t[0], color='crimson', linewidth=1.0, alpha=0.8)
            for ax in self.signal_axes
        ]

        # Full driven trajectory, drawn once -- the map view otherwise only
        # ever adds/moves the "now" marker and the path-snapshot overlay.
        self.map_ax.plot(self.cols['car_x'], self.cols['car_y'], '-',
                         color='0.75', linewidth=1.0, zorder=1,
                         label='driven trajectory')
        self.map_ax.set_aspect('equal', adjustable='datalim')
        self.map_ax.set_title('Map', fontsize=9)
        self.map_ax.tick_params(labelsize=7)
        self.map_ax.grid(True, alpha=0.2)

        self.zoom_ax.set_aspect('equal', adjustable='box')
        self.zoom_ax.set_title('Current section (zoomed)', fontsize=9)
        self.zoom_ax.tick_params(labelsize=7)
        self.zoom_ax.grid(True, alpha=0.2)

        (self.map_path_line,) = self.map_ax.plot(
            [], [], '-', color='tab:blue', linewidth=1.4, alpha=0.85,
            zorder=2, label='planned path (now)',
        )
        (self.zoom_traj_line,) = self.zoom_ax.plot(
            [], [], '-', color='0.75', linewidth=1.2, zorder=1,
        )
        (self.zoom_path_line,) = self.zoom_ax.plot(
            [], [], '-', color='tab:blue', linewidth=1.8, alpha=0.9, zorder=2,
        )

        self.map_marker = self.map_ax.fill(
            [], [], color='crimson', zorder=3, label='car',
        )[0]
        self.zoom_marker = self.zoom_ax.fill(
            [], [], color='crimson', zorder=3,
        )[0]
        self.map_ax.legend(fontsize=6, loc='upper right')

        self.slider = Slider(
            self.slider_ax, 't', float(self.t[0]), float(self.t[-1]),
            valinit=float(self.t[0]), valstep=self.t,
        )
        self.slider.label.set_size(8)
        self.slider.on_changed(self._on_slide)

    def _plot_signal(self, ax, sig):
        if sig in DERIVED_PAIRS:
            col_a, col_b, (name_a, name_b) = DERIVED_PAIRS[sig]
            pairs = [(col_a, name_a, '-'), (col_b, name_b, '--')]
        else:
            pairs = [(sig, sig, '-')]
        plotted_any = False
        for col_name, disp_name, style in pairs:
            if col_name not in self.cols or not np.any(np.isfinite(self.cols[col_name])):
                continue
            ax.plot(self.t, self.cols[col_name], style, linewidth=1.1,
                    alpha=0.85 if style == '-' else 0.6, label=disp_name)
            plotted_any = True
        if not plotted_any:
            ax.text(0.5, 0.5, f'"{sig}" not present', ha='center', va='center',
                    transform=ax.transAxes, color='gray', fontsize=8)
            print(f'warning: signal "{sig}" missing/empty — skipped', file=sys.stderr)
        elif sig in DERIVED_PAIRS:
            ax.legend(fontsize=6, loc='upper right')

    # ------------------------------------------------------------------
    # Per-frame update: everything that depends on "now"
    # ------------------------------------------------------------------
    def _index_for_time(self, t_now):
        return int(np.clip(np.searchsorted(self.t, t_now), 0, self.n - 1))

    def _on_slide(self, t_now):
        self._update(self._index_for_time(t_now))
        self.fig.canvas.draw_idle()

    def _update(self, i):
        t_now = self.t[i]
        for cursor in self.cursors:
            cursor.set_xdata([t_now, t_now])

        car_x, car_y = self.cols['car_x'][i], self.cols['car_y'][i]
        car_yaw = self.cols['car_yaw'][i] if 'car_yaw' in self.cols else 0.0

        path_x, path_y = _snapshot_at_or_before(self.snapshot_ts, self.snapshot_xy, t_now)
        if path_x is not None:
            self.map_path_line.set_data(path_x, path_y)
            self.zoom_path_line.set_data(path_x, path_y)

        tri = _rotate(CAR_TRIANGLE * CAR_LENGTH_M, car_yaw) + [car_x, car_y]
        self.map_marker.set_xy(tri)
        self.zoom_marker.set_xy(tri)

        self.zoom_traj_line.set_data(
            self.cols['car_x'][:i + 1], self.cols['car_y'][:i + 1],
        )
        self.zoom_ax.set_xlim(car_x - ZOOM_HALF_WIDTH_M, car_x + ZOOM_HALF_WIDTH_M)
        self.zoom_ax.set_ylim(car_y - ZOOM_HALF_WIDTH_M, car_y + ZOOM_HALF_WIDTH_M)

        e_y = self.cols['e_y'][i] if 'e_y' in self.cols else float('nan')
        e_psi = self.cols['e_psi_deg'][i] if 'e_psi_deg' in self.cols else float('nan')
        self.zoom_ax.set_title(
            f'Current section (zoomed) — e_y={e_y:.2f} m, e_psi={e_psi:.1f}°',
            fontsize=9,
        )

    def show(self):
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('log', nargs='?', default=None,
                     help='control telemetry CSV. Omit to auto-load the newest '
                          f'log in {RECORDED_RUNS_DIR}')
    ap.add_argument('--signals', default=None,
                     help=f'comma-separated column names for the left panel '
                          f'(default: {",".join(DEFAULT_SIGNALS)})')
    args = ap.parse_args()

    log = args.log
    if log is None:
        latest = find_latest_log(RECORDED_RUNS_DIR)
        if latest is None:
            sys.exit(f'No *_control_*.csv found in {RECORDED_RUNS_DIR} and no CSV '
                      'path was given. Copy a log there, or pass one explicitly:\n'
                      '  python -m tuner.plot_playback <control_csv>')
        print(f'No CSV given -- auto-loaded newest log: {latest}')
        log = latest

    signals = args.signals.split(',') if args.signals else DEFAULT_SIGNALS
    Playback(log, signals).show()


if __name__ == '__main__':
    main()
