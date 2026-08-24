"""
Interactive time-scrubbing playback of one or more control telemetry logs.

Reads the (control_csv, path_csv) pair(s) `fsae_control/telemetry_logger.py`
writes (see that module's docstring for the schema) and shows, side by side:

  - left:  the scored signals (e_y, e_psi_deg, kappa, steer_deg, v) stacked
           on a shared time axis, with a vertical cursor marking "now" --
           one line per run per signal when multiple logs are given
  - top right:  each run's full driven trajectory (and, if available, the
           planner's most-recent path snapshot at "now") with a triangle
           marking that run's car position/heading
  - bottom right: the same scene zoomed tightly to the car's current
           section of track, so heading and lateral error are easy to read
           at a glance

A slider under the metrics panel scrubs a shared "now" time through the run;
dragging it updates every run's cursor, triangle, and path overlay together.
Each run gets its own colour (signal lines, car marker, driven trajectory,
and path overlay all match) and its own checkbox to show/hide it everywhere
at once -- built for eyeballing a single run or comparing two controllers
head-to-head (e.g. an MPC log against a Stanley log recorded on the same
track) without writing a one-off script each time.

This is presentation/analysis tooling only -- it never touches a live
controller, just the CSVs it wrote after the fact.

Usage
-----
    python -m tuner.tools.plot_playback <control_csv> [<control_csv> ...]

    # no CSV given -> auto-load the newest run from each subfolder of
    # RECORDED_RUNS_DIR (fsds_simulator/recorded_runs/ -- see that constant
    # below), e.g. one from LMPC/, one from NMPC/, one from Stanley/
    python -m tuner.tools.plot_playback

    # no CSV given, but want every run in every subfolder overlaid
    python -m tuner.tools.plot_playback --all

    # no CSV given, but only want the single newest run overall
    python -m tuner.tools.plot_playback --latest-only

    # overlay two explicit runs -- each gets its own colour, signal lines,
    # marker, trajectory, and path overlay, plus a checkbox to hide/show it
    python -m tuner.tools.plot_playback \\
        ~/fsae_logs/mpc_standalone_control_<stamp>.csv \\
        ~/fsae_logs/stanley_control_<stamp>.csv

    # explicit signal set for the left-hand metrics panel
    python -m tuner.tools.plot_playback run.csv --signals e_y,e_psi_deg,yaw_rate

Each run's sibling `<tag>_path_<stamp>.csv` next to its control CSV is
loaded automatically if present, to draw the planner's path as it looked at
each moment in time. If it's missing (e.g. a Stanley log, or an older run
from before log_path() existed), that run's map/zoom views fall back to
showing just its own driven trajectory (car_x/car_y) with no live path
overlay. Runs may have different `t` sampling/length -- the slider drives a
single shared time value, and each run independently looks up its own
nearest sample.
"""
import argparse
import datetime
import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import CheckButtons, RadioButtons, Slider

from tuner import csv_log

# One row per signal by default -- covers the "lateral error / heading error /
# corner curvature / steering / speed" ask directly; --signals overrides this.
# 'corner_frac' (0=straight -> 1=full corner, see ADAPTIVE_COLUMNS in
# telemetry_logger.py) stands in for curvature -- there is no raw `kappa`
# column in the control CSV schema (live node or offline tuner), so a bare
# 'kappa' here would always warn-and-skip.
DEFAULT_SIGNALS = ['e_y', 'e_psi_deg', 'corner_frac', 'steer_deg', 'v']

# Auto-search target when no CSV path is given on the command line. Not
# populated automatically by any live-node launch path (that's controlled by
# ros2/launch_all.sh's log_dir, outside this repo) -- copy/move CSVs out of
# ~/fsae_logs (or wherever log_dir pointed) into here yourself after a run.
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_HERE))
RECORDED_RUNS_DIR = os.path.join(_REPO_ROOT, 'fsds_simulator', 'recorded_runs')

# Curated drop zone: if this holds any `*_control_*.csv` (directly, not in a
# further subfolder), auto-load uses ONLY this folder instead of scanning
# every controller subfolder of RECORDED_RUNS_DIR. Lets you pick "the run(s)
# to look at" by moving files here rather than passing a CLI path each time.
GRAPH_DIR = os.path.join(RECORDED_RUNS_DIR, 'graph')


def _find_control_csvs(log_dir):
    """Return every `*_control_<stamp>.csv` directly in or under `log_dir`.

    Recurses one or more levels so per-controller subfolders (e.g.
    `recorded_runs/LMPC/`, `recorded_runs/NMPC/`, `recorded_runs/Stanley/`)
    are picked up alongside anything left flat in `log_dir` itself.
    """
    return glob.glob(os.path.join(log_dir, '**', '*_control_*.csv'), recursive=True)


def _stamp(path):
    """Sort key for a control CSV: seconds since the epoch, or -1 if unparseable.

    Handles BOTH filename stamp formats, so a directory holding runs recorded
    either side of the change sorts correctly as one set:

      `<tag>_control_20260824-134232.csv`  local `%Y%m%d-%H%M%S` (current)
      `<tag>_control_1787532892.csv`       epoch seconds (older runs)

    The datetime form is parsed with `mktime` rather than compared as a string
    because the two forms have to be ordered against EACH OTHER; a lexicographic
    key would sort every 10-digit epoch name before every '2026…' name
    regardless of when the runs actually happened.

    Interpreted as LOCAL time, matching how `ControlLogger` writes it. A run
    recorded in a different timezone therefore sorts by its wall-clock reading
    rather than its true instant — harmless for ordering runs from one machine,
    which is all this is used for.
    """
    name = os.path.splitext(os.path.basename(path))[0]
    try:
        suffix = name.rsplit('_control_', 1)[1]
    except IndexError:
        return -1
    try:
        return int(datetime.datetime.strptime(suffix, '%Y%m%d-%H%M%S').timestamp())
    except ValueError:
        pass
    try:
        return int(suffix)
    except ValueError:
        return -1


def find_latest_log(log_dir=RECORDED_RUNS_DIR):
    """Return the most recently recorded `*_control_<stamp>.csv` under `log_dir`.

    Sorts on the filename stamp decoded by `_stamp` (which accepts both the
    current `%Y%m%d-%H%M%S` form and the older epoch-seconds form), not
    lexicographic filename order — so it stays correct both when `tag` differs
    in length between runs ('mpc_standalone' vs 'stanley') and when a directory
    holds runs recorded either side of the stamp-format change.
    """
    candidates = _find_control_csvs(log_dir)
    if not candidates:
        return None
    return max(candidates, key=_stamp)


def find_latest_per_folder(log_dir=RECORDED_RUNS_DIR):
    """Return the newest `*_control_<stamp>.csv` from each immediate
    subfolder of `log_dir` (e.g. one from `LMPC/`, one from `NMPC/`, one
    from `Stanley/`), oldest-newest first. Runs left flat directly in
    `log_dir` (not in any subfolder) are grouped together as one "folder".

    This is what `--latest-only` uses, since with per-controller subfolders
    the useful "latest" comparison is one representative run per
    controller, not just the single most recent file across all of them
    (which could leave every other controller unrepresented).
    """
    candidates = _find_control_csvs(log_dir)
    by_folder = {}
    for path in candidates:
        folder = os.path.dirname(os.path.abspath(path))
        if _stamp(path) > _stamp(by_folder.get(folder, '')):
            by_folder[folder] = path
    return sorted(by_folder.values(), key=_stamp)


def find_all_logs(log_dir=RECORDED_RUNS_DIR):
    """Return every `*_control_<epoch>.csv` under `log_dir`, oldest first.

    Same stamp-based ordering as `find_latest_log`, so multiple runs overlay
    in the order they were recorded rather than filename order.
    """
    return sorted(_find_control_csvs(log_dir), key=_stamp)

# Signals not literally a CSV column, built from two columns that are.
# {name: (col_a, col_b, label)} -> plotted as two lines sharing one row.
DERIVED_PAIRS = {
    'v': ('v_actual', 'v_desired', ('v_actual', 'v_desired')),
}

COLORS = plt.rcParams['axes.prop_cycle'].by_key()['color']


def _read_header_metadata(path):
    """Parse the `# key=value` comment block telemetry_logger.py prepends.

    Returns a dict of whatever key=value pairs are present (composite_score,
    lap_time_s, steering_sat_ratio, ...) -- see ControlLogger.close(). Not
    every log has every key (score_is_partial=1 runs skip time_bonus, older
    logs predate some columns), so callers must .get() with a default.
    """
    meta = {}
    with open(path) as f:
        for line in f:
            if not line.startswith('#'):
                break
            line = line[1:].strip()
            if '=' not in line:
                continue
            key, _, rest = line.partition('=')
            value = rest.split()[0] if rest.split() else rest
            meta[key.strip()] = value.strip()
    return meta


def load_log(path):
    """Return (columns_dict, metadata_dict) for one control CSV."""
    cols = csv_log.load_columns(path)
    meta = _read_header_metadata(path)
    return cols, meta


def _label_for(path, meta):
    """Short label for a run: its `recorded_runs/<folder>/` name if it's in
    one (e.g. 'LMPC', 'NMPC', 'Stanley' -- the real signal for which
    controller produced it, since `tag` is often shared between controllers,
    e.g. both LMPC and NMPC logs use 'mpc_standalone'), else the filename
    tag. Kept short deliberately (no score/lap suffix) since it's reused
    verbatim as a checkbox/radio-button/legend label.
    """
    parent = os.path.basename(os.path.dirname(os.path.abspath(path)))
    if parent and parent != os.path.basename(os.path.abspath(RECORDED_RUNS_DIR)):
        return parent
    return os.path.basename(path).split('_control_')[0]


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


class _Run:
    """One loaded control log plus everything derived from it.

    Each run keeps its own `t`/columns/path snapshots -- lengths and
    sampling can differ between runs (e.g. an MPC log vs. a Stanley log),
    so "now" is looked up per-run from a shared time value rather than
    assuming a shared index.
    """

    def __init__(self, control_csv, color):
        self.path = control_csv
        self.color = color
        self.cols, self.meta = load_log(control_csv)
        if 't' not in self.cols:
            sys.exit(f'{control_csv}: no `t` column -- not a control telemetry CSV.')
        if 'car_x' not in self.cols or 'car_y' not in self.cols:
            sys.exit(f'{control_csv}: no car_x/car_y columns -- cannot show a map view.')

        self.label = _label_for(control_csv, self.meta)
        self.t = self.cols['t']
        self.n = len(self.t)

        path_csv = _path_csv_for(control_csv)
        if path_csv is not None:
            self.snapshot_ts, self.snapshot_xy = _load_path_snapshots(path_csv)
            print(f'Loaded {len(self.snapshot_ts)} path snapshot(s) from {path_csv}')
        else:
            self.snapshot_ts, self.snapshot_xy = np.array([]), []
            print(f'{self.label}: no sibling path CSV found -- map/zoom views will '
                  'show the driven trajectory only, with no live planner-path overlay.')

        # Populated by Playback._draw_static().
        self.lines = []          # every Line2D belonging to this run (signals)
        self.map_traj_line = None
        self.zoom_traj_line = None
        self.map_path_line = None
        self.zoom_path_line = None
        self.map_marker = None
        self.zoom_marker = None
        self.cursors = []
        self.visible = True

    def index_for_time(self, t_now):
        return int(np.clip(np.searchsorted(self.t, t_now), 0, self.n - 1))

    def set_visible(self, visible):
        self.visible = visible
        for line in self.lines:
            line.set_visible(visible)
        for artist in (self.map_traj_line, self.zoom_traj_line,
                       self.map_path_line, self.zoom_path_line,
                       self.map_marker, self.zoom_marker):
            if artist is not None:
                artist.set_visible(visible)
        for cursor in self.cursors:
            cursor.set_visible(visible)


class Playback:
    """Owns the figure and all the little bits of mutable plot state.

    Kept as a class (rather than a pile of closures) purely so the widgets
    (Slider, CheckButtons) have somewhere to hold their callbacks without
    falling out of scope -- matplotlib widgets stop working the moment
    their last Python reference is garbage collected.
    """

    def __init__(self, control_csvs, signals=DEFAULT_SIGNALS):
        self.signals = signals
        self.runs = [
            _Run(path, COLORS[i % len(COLORS)])
            for i, path in enumerate(control_csvs)
        ]
        self._dedupe_labels()
        # Shared "now" ranges over the union of every run's time span, so
        # the slider can always reach the start/end of the longest run.
        self.t_min = min(run.t[0] for run in self.runs)
        self.t_max = max(run.t[-1] for run in self.runs)

        self.focus_idx = 0  # index into self.runs the zoom view tracks

        self._build_figure()
        self._draw_static()
        self._update(self.t_min)

    def _dedupe_labels(self):
        """Suffix ` #2`, ` #3`, ... onto labels shared by multiple runs.

        Folder-based labels (e.g. two 'LMPC' runs recorded at different
        times) collide by design -- checkboxes/radio buttons/legends all key
        off `run.label`, so duplicates must be told apart for those lookups
        to resolve the right run.
        """
        seen = {}
        for run in self.runs:
            seen[run.label] = seen.get(run.label, 0) + 1
            if seen[run.label] > 1:
                run.label = f'{run.label} #{seen[run.label]}'

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------
    def _build_figure(self):
        self.fig = plt.figure(figsize=(15, 9))
        title = ' vs '.join(run.label for run in self.runs)
        self.fig.suptitle(f'Playback — {title}', fontsize=11)

        # Left column: one row per signal, plus a slider row underneath.
        # Right column: map (top) over a square zoomed view (bottom).
        # A 2-column outer grid keeps the left column's width independent of
        # how tall the right column's two stacked panels are.
        outer = self.fig.add_gridspec(
            1, 2, width_ratios=[1.15, 1.0], wspace=0.28,
            left=0.13, right=0.98, top=0.93, bottom=0.08,
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

        for run in self.runs:
            run.cursors = [
                ax.axvline(run.t[0], color=run.color, linewidth=1.0, alpha=0.8)
                for ax in self.signal_axes
            ]

        self.map_ax.set_aspect('equal', adjustable='datalim')
        self.map_ax.set_title('Map', fontsize=9)
        self.map_ax.tick_params(labelsize=7)
        self.map_ax.grid(True, alpha=0.2)

        self.zoom_ax.set_aspect('equal', adjustable='box')
        self.zoom_ax.set_title('Current section (zoomed)', fontsize=9)
        self.zoom_ax.tick_params(labelsize=7)
        self.zoom_ax.grid(True, alpha=0.2)

        for run in self.runs:
            # Full driven trajectory, drawn once -- the map view otherwise
            # only ever adds/moves the "now" marker and the path overlay.
            (run.map_traj_line,) = self.map_ax.plot(
                run.cols['car_x'], run.cols['car_y'], '-',
                color=run.color, linewidth=1.0, alpha=0.5, zorder=1,
                label=f'{run.label} — driven trajectory',
            )
            (run.map_path_line,) = self.map_ax.plot(
                [], [], '-', color=run.color, linewidth=1.4, alpha=0.9,
                zorder=2,
            )
            (run.zoom_traj_line,) = self.zoom_ax.plot(
                [], [], '-', color=run.color, linewidth=1.2, alpha=0.5, zorder=1,
            )
            (run.zoom_path_line,) = self.zoom_ax.plot(
                [], [], '-', color=run.color, linewidth=1.8, alpha=0.9, zorder=2,
            )
            run.map_marker = self.map_ax.fill(
                [], [], color=run.color, zorder=3, label=f'{run.label} — car',
            )[0]
            run.zoom_marker = self.zoom_ax.fill(
                [], [], color=run.color, zorder=3,
            )[0]
        self.map_ax.legend(fontsize=6, loc='upper right')

        self.slider = Slider(
            self.slider_ax, 't', float(self.t_min), float(self.t_max),
            valinit=float(self.t_min),
        )
        self.slider.label.set_size(8)
        self.slider.on_changed(self._on_slide)

        self._build_checkboxes()
        self._build_focus_selector()

    def _build_focus_selector(self):
        """Radio buttons choosing which run the zoom view (bottom right)
        tracks. Only useful with 2+ runs -- with one run there's nothing to
        switch between.
        """
        if len(self.runs) < 2:
            return
        height = 0.1 * len(self.runs)
        radio_ax = self.fig.add_axes([0.005, 0.06, 0.08, height])
        radio_ax.set_xticks([])
        radio_ax.set_yticks([])
        radio_ax.patch.set_alpha(0)
        for spine in radio_ax.spines.values():
            spine.set_visible(False)
        radio_ax.set_title('Zoom focus', fontsize=7)
        labels = [run.label for run in self.runs]
        self.focus_radio = RadioButtons(radio_ax, labels, active=self.focus_idx)
        colors = [run.color for run in self.runs]
        for label, color in zip(self.focus_radio.labels, colors):
            label.set_color(color)

        def _on_focus(label):
            self.focus_idx = next(i for i, r in enumerate(self.runs) if r.label == label)
            self._update(self.slider.val)
            self.fig.canvas.draw_idle()

        self.focus_radio.on_clicked(_on_focus)

    def _build_checkboxes(self):
        if len(self.runs) < 2:
            return  # nothing to disambiguate with a single run
        radio_height = 0.1 * len(self.runs)
        check_height = 0.12 * len(self.runs)
        check_ax = self.fig.add_axes(
            [0.005, 0.06 + radio_height + 0.06, 0.08, check_height])
        check_ax.set_xticks([])
        check_ax.set_yticks([])
        check_ax.patch.set_alpha(0)
        for spine in check_ax.spines.values():
            spine.set_visible(False)
        check_ax.set_title('Show/hide', fontsize=7)
        labels = [run.label for run in self.runs]
        self.checks = CheckButtons(check_ax, labels, [True] * len(labels))
        colors = [run.color for run in self.runs]
        self.checks.set_check_props(dict(facecolor=colors))
        self.checks.set_label_props(dict(color=colors))

        def _on_click(label):
            run = next(r for r in self.runs if r.label == label)
            run.set_visible(not run.visible)
            self.fig.canvas.draw_idle()

        self.checks.on_clicked(_on_click)

    def _plot_signal(self, ax, sig):
        plotted_any = False
        for run in self.runs:
            if sig in DERIVED_PAIRS:
                col_a, col_b, (name_a, name_b) = DERIVED_PAIRS[sig]
                pairs = [(col_a, name_a, '-'), (col_b, name_b, '--')]
            else:
                pairs = [(sig, sig, '-')]
            for col_name, disp_name, style in pairs:
                if col_name not in run.cols or not np.any(np.isfinite(run.cols[col_name])):
                    continue
                label = f'{disp_name} [{run.label}]' if len(self.runs) > 1 else disp_name
                line, = ax.plot(
                    run.t, run.cols[col_name], style, color=run.color,
                    linewidth=1.1, alpha=0.85 if style == '-' else 0.6,
                    label=label,
                )
                run.lines.append(line)
                plotted_any = True
        if not plotted_any:
            ax.text(0.5, 0.5, f'"{sig}" not present', ha='center', va='center',
                    transform=ax.transAxes, color='gray', fontsize=8)
            print(f'warning: signal "{sig}" missing/empty in every log — skipped', file=sys.stderr)
        elif sig in DERIVED_PAIRS or len(self.runs) > 1:
            ax.legend(fontsize=6, loc='upper right')

    # ------------------------------------------------------------------
    # Per-frame update: everything that depends on "now"
    # ------------------------------------------------------------------
    def _on_slide(self, t_now):
        self._update(t_now)
        self.fig.canvas.draw_idle()

    def _update(self, t_now):
        for run in self.runs:
            for cursor in run.cursors:
                cursor.set_xdata([t_now, t_now])

            i = run.index_for_time(t_now)
            car_x, car_y = run.cols['car_x'][i], run.cols['car_y'][i]
            car_yaw = run.cols['car_yaw'][i] if 'car_yaw' in run.cols else 0.0

            path_x, path_y = _snapshot_at_or_before(run.snapshot_ts, run.snapshot_xy, t_now)
            if path_x is not None:
                run.map_path_line.set_data(path_x, path_y)
                run.zoom_path_line.set_data(path_x, path_y)

            tri = _rotate(CAR_TRIANGLE * CAR_LENGTH_M, car_yaw) + [car_x, car_y]
            run.map_marker.set_xy(tri)
            run.zoom_marker.set_xy(tri)

            run.zoom_traj_line.set_data(
                run.cols['car_x'][:i + 1], run.cols['car_y'][:i + 1],
            )

        # Zoom on the focused run's car position (default: first run; pick
        # another via the radio buttons) -- keeps the zoomed section
        # anchored to one consistent car rather than jumping between them.
        primary = self.runs[self.focus_idx]
        i0 = primary.index_for_time(t_now)
        car_x, car_y = primary.cols['car_x'][i0], primary.cols['car_y'][i0]
        self.zoom_ax.set_xlim(car_x - ZOOM_HALF_WIDTH_M, car_x + ZOOM_HALF_WIDTH_M)
        self.zoom_ax.set_ylim(car_y - ZOOM_HALF_WIDTH_M, car_y + ZOOM_HALF_WIDTH_M)

        e_y = primary.cols['e_y'][i0] if 'e_y' in primary.cols else float('nan')
        e_psi = primary.cols['e_psi_deg'][i0] if 'e_psi_deg' in primary.cols else float('nan')
        focus_tag = f' [{primary.label}]' if len(self.runs) > 1 else ''
        self.zoom_ax.set_title(
            f'Current section (zoomed){focus_tag} — e_y={e_y:.2f} m, e_psi={e_psi:.1f}°',
            fontsize=9,
        )

    def show(self):
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='*',
                     help='control telemetry CSV(s). Omit to auto-load the newest run '
                          f'from each subfolder of {RECORDED_RUNS_DIR} (e.g. one from '
                          'LMPC/, one from NMPC/, one from Stanley/ -- see --all/'
                          '--latest-only for other auto-load modes), or -- if '
                          f'{GRAPH_DIR} has any runs in it -- only from there')
    ap.add_argument('--signals', default=None,
                     help=f'comma-separated column names for the left panel '
                          f'(default: {",".join(DEFAULT_SIGNALS)})')
    ap.add_argument('--recorded-runs', default=RECORDED_RUNS_DIR,
                     help='directory to auto-search when no CSV is given '
                          f'(default: {RECORDED_RUNS_DIR})')
    ap.add_argument('--all', action='store_true',
                     help='when no CSV is given, load every run in --recorded-runs '
                          'instead of just the newest per subfolder')
    ap.add_argument('--latest-only', action='store_true',
                     help='when no CSV is given, load only the single newest run '
                          'across all of --recorded-runs (overrides --all)')
    args = ap.parse_args()

    logs = args.logs
    if not logs:
        search_dir = args.recorded_runs
        # Curated graph/ folder takes over the auto-search entirely when it
        # has runs in it -- that's the point of moving files there instead of
        # leaving them in LMPC/NMPC/Stanley.
        if search_dir == RECORDED_RUNS_DIR and _find_control_csvs(GRAPH_DIR):
            search_dir = GRAPH_DIR
        if args.latest_only:
            latest = find_latest_log(search_dir)
            logs = [latest] if latest is not None else []
        elif args.all:
            logs = find_all_logs(search_dir)
        else:
            logs = find_latest_per_folder(search_dir)
        if not logs:
            sys.exit(f'No *_control_*.csv found in {search_dir} and no CSV '
                      'path was given. Copy a log there, or pass one explicitly:\n'
                      '  python -m tuner.tools.plot_playback <control_csv>')
        noun = 'run' if len(logs) == 1 else f'{len(logs)} runs'
        print(f'No CSV given -- auto-loaded {noun} from {search_dir}:')
        for log in logs:
            print(f'  {log}')

    signals = args.signals.split(',') if args.signals else DEFAULT_SIGNALS
    Playback(logs, signals).show()


if __name__ == '__main__':
    main()
