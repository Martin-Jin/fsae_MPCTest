"""
Interactive multi-signal plot of one or more control telemetry CSVs.

Reads the CSVs `fsae_control/telemetry_logger.py` writes (t, e_y, e_psi_deg,
kappa, steer_deg, v_actual/v_desired, ... -- see ADAPTIVE_COLUMNS for the
full set) and plots a chosen set of signals against time on stacked axes,
with a checkbox panel to toggle each line on/off. Built for eyeballing one
run or comparing two runs (e.g. an MPC log against a Stanley log on the same
track) without writing a one-off script each time.

This is presentation/analysis tooling only -- it never touches a live
controller, just the CSVs it wrote after the fact.

Usage
-----
    python3 -m tuner.plot_control_log <control_csv> [<control_csv> ...]

    # no CSV given -> auto-load the newest log in RECORDED_RUNS_DIR
    # (fsds_simulator/recorded_runs/ -- see that constant below)
    python3 -m tuner.plot_control_log

    # explicit signal set (default: e_y, e_psi_deg, kappa, steer_deg, v)
    python3 -m tuner.plot_control_log run.csv --signals e_y,e_psi_deg,yaw_rate

    # list every signal a log contains, then exit
    python3 -m tuner.plot_control_log run.csv --list-signals

Each file gets its own colour; each signal gets its own row (shared time
axis). A checkbox panel on the left toggles a signal's lines (all files) on
or off. Missing/all-NaN columns (e.g. the m_Q_*/m_R_* adaptive columns on a
Stanley log) are skipped with a warning rather than plotting an empty axis.
"""
import argparse
import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import CheckButtons

from tuner import csv_log

# One row per signal by default -- covers the "lateral error / heading error /
# corner curvature / steering / speed" ask directly; --signals overrides this.
DEFAULT_SIGNALS = ['e_y', 'e_psi_deg', 'kappa', 'steer_deg', 'v']

# Auto-search target when no CSV path is given on the command line. Not
# populated automatically by any live-node launch path (that's controlled by
# ros2/launch_all.sh's log_dir, outside this repo) -- copy/move CSVs out of
# ~/fsae_logs (or wherever log_dir pointed) into here yourself after a run.
_HERE = os.path.dirname(os.path.abspath(__file__))
RECORDED_RUNS_DIR = os.path.join(
    os.path.dirname(_HERE), 'fsds_simulator', 'recorded_runs',
)


def find_latest_log(log_dir=RECORDED_RUNS_DIR):
    """Return the most recently recorded `*_control_<epoch>.csv` in `log_dir`.

    Sorts on the numeric epoch-seconds suffix ControlLogger stamps each
    filename with (see telemetry_logger.py's `stamp = int(time.time())`),
    not lexicographic filename order, so it's correct even if `tag` differs
    in length between runs (e.g. 'mpc_standalone' vs 'stanley').
    """
    candidates = glob.glob(os.path.join(log_dir, '*_control_*.csv'))
    if not candidates:
        return None

    def _stamp(path):
        name = os.path.splitext(os.path.basename(path))[0]
        try:
            return int(name.rsplit('_control_', 1)[1])
        except (IndexError, ValueError):
            return -1

    return max(candidates, key=_stamp)

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
    tag = os.path.basename(path).split('_control_')[0]
    score = meta.get('composite_score')
    lap = meta.get('lap_time_s')
    extra = []
    if score is not None:
        extra.append(f'score={float(score):.3f}')
    if lap is not None:
        extra.append(f'lap={float(lap):.1f}s')
    return f"{tag} ({', '.join(extra)})" if extra else tag


def plot_logs(paths, signals=DEFAULT_SIGNALS):
    logs = []
    for path in paths:
        cols, meta = load_log(path)
        if 't' not in cols:
            sys.exit(f'{path}: no `t` column -- not a control telemetry CSV.')
        logs.append((path, cols, meta))

    n_rows = len(signals)
    fig, axes = plt.subplots(
        n_rows, 1, sharex=True, figsize=(11, 1.8 * n_rows + 1.2), squeeze=False,
    )
    axes = axes[:, 0]
    fig.subplots_adjust(left=0.22, right=0.97, top=0.94, bottom=0.08, hspace=0.08)

    lines_by_label = {}   # checkbox label -> list of Line2D across all axes
    visible = {}

    for row, sig in enumerate(signals):
        ax = axes[row]
        plotted_any = False
        for i, (path, cols, meta) in enumerate(logs):
            file_label = _label_for(path, meta)
            color = COLORS[i % len(COLORS)]
            if sig in DERIVED_PAIRS:
                col_a, col_b, (name_a, name_b) = DERIVED_PAIRS[sig]
                pairs = [(col_a, name_a, '-'), (col_b, name_b, '--')]
            else:
                pairs = [(sig, sig, '-')]
            for col_name, disp_name, style in pairs:
                if col_name not in cols or not np.any(np.isfinite(cols[col_name])):
                    continue
                label = f'{disp_name} [{file_label}]'
                line, = ax.plot(
                    cols['t'], cols[col_name], style, color=color,
                    linewidth=1.3, alpha=0.7 if style == '--' else 0.9,
                    label=label,
                )
                lines_by_label[label] = lines_by_label.get(label, []) + [line]
                visible[label] = True
                plotted_any = True
        if not plotted_any:
            ax.text(0.5, 0.5, f'"{sig}" not present in any log', ha='center',
                     va='center', transform=ax.transAxes, color='gray')
            print(f'warning: signal "{sig}" missing/empty in every log — skipped', file=sys.stderr)
        ax.set_ylabel(sig, fontsize=9)
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=7, loc='upper right')
    axes[-1].set_xlabel('t (s)')
    fig.suptitle('Control telemetry — ' + ' vs '.join(
        _label_for(p, m) for p, _, m in logs
    ), fontsize=10)

    # Checkbox panel: one row per plotted line, independent of which axis
    # it's on, so e.g. "e_y [mpc]" and "e_y [stanley]" toggle separately.
    labels = list(lines_by_label.keys())
    if labels:
        check_ax = fig.add_axes([0.01, 0.08, 0.18, 0.86])
        check_ax.set_axis_off()
        checks = CheckButtons(check_ax, labels, [True] * len(labels))

        def _on_click(label):
            visible[label] = not visible[label]
            for line in lines_by_label[label]:
                line.set_visible(visible[label])
            fig.canvas.draw_idle()

        checks.on_clicked(_on_click)
        fig._plot_control_log_checks = checks  # keep a reference alive

    plt.show()


def _list_signals(paths):
    for path in paths:
        cols, _meta = load_log(path)
        present = [k for k, v in cols.items() if np.any(np.isfinite(v)) if v.dtype.kind == 'f']
        print(f'{path}:')
        for name in present:
            print(f'  {name}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='*',
                     help='control telemetry CSV(s). Omit to auto-load the newest '
                          f'log in {RECORDED_RUNS_DIR}')
    ap.add_argument('--signals', default=None,
                     help=f'comma-separated column names to plot (default: {",".join(DEFAULT_SIGNALS)})')
    ap.add_argument('--list-signals', action='store_true',
                     help='print every numeric column present in the given log(s) and exit')
    ap.add_argument('--recorded-runs', default=RECORDED_RUNS_DIR,
                     help='directory to auto-search when no CSV is given '
                          f'(default: {RECORDED_RUNS_DIR})')
    args = ap.parse_args()

    logs = args.logs
    if not logs:
        latest = find_latest_log(args.recorded_runs)
        if latest is None:
            sys.exit(f'No *_control_*.csv found in {args.recorded_runs} and no CSV '
                      'path was given. Copy a log there, or pass one explicitly:\n'
                      '  python3 -m tuner.plot_control_log <control_csv>')
        print(f'No CSV given -- auto-loaded newest log: {latest}')
        logs = [latest]

    if args.list_signals:
        _list_signals(logs)
        return

    signals = args.signals.split(',') if args.signals else DEFAULT_SIGNALS
    plot_logs(logs, signals)


if __name__ == '__main__':
    main()
