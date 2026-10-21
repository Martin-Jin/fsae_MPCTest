"""
fsae_control/live_viz/app.py — the live debug windows and `main`

Builds the matplotlib map window and the two cost-breakdown windows once, then
updates them in place from a LiveVizNode on one Tk timer. The Tk backend must
be selected before pyplot is imported, so that call stays above the pyplot
import here.

The cost-breakdown windows are plain Tk canvases (_TkDebugWindow), not
matplotlib figures: their ~24 text labels cost about 100 ms per frame in
matplotlib on the developer's WSL machine, which held them to about 5 Hz,
while Tk canvas text updates are near-free.

Why the map's artists are updated and blitted, not redrawn: the old loop did
ax.clear() and rebuilt every scatter, line, legend, text box and bar each
frame and moved the map's axes limits every frame (so every tick label was
re-rendered). That capped the refresh at about 25 Hz and cost about 0.4 s of
rendering per frame on the developer's WSL machine. Here:

- each artist is created once and only its data changes (set_data,
  set_offsets, set_width, set_text);
- the map's axes are fixed and the data is drawn relative to a point just
  ahead of the car, so the axes, ticks, labels and legend never change;
- everything that never changes is rendered once into a cached background,
  and each frame only restores it and redraws the moving artists
  (blitting, see _Blitter);
- a window is redrawn only when a new message arrived.

A full redraw still happens on a resize, a legend change or a controller
switch, and re-captures the background.
"""

import os
import signal
import time
import tkinter as tk
from tkinter import font as tkfont

import matplotlib
import numpy as np

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt  # noqa: E402 (backend must be selected first)
from matplotlib.patches import Polygon  # noqa: E402
from matplotlib.text import Text  # noqa: E402

import rclpy  # noqa: E402

from fsae_control.live_viz.node import LiveVizNode, get_car_triangle  # noqa: E402
from fsae_control.live_viz.panels import (  # noqa: E402
    DEBUG_BAR_GROUPS, DEBUG_HORIZON_TERMS, DEBUG_REDRAW_HZ, REDRAW_HZ, STANLEY_ERROR_TERMS,
    STATS_REDRAW_HZ,
    STANLEY_LAW_TERMS, VIEW_AHEAD, VIEW_BEHIND, VIEW_HALF_WIDTH,
)

_EMPTY = np.empty((0, 2))


_BAR_BLUE, _BAR_RED, _BAR_EMPTY = '#1f77b4', '#d62728', '#d9d9d9'
_FONT = ('DejaVu Sans', 10)
_FONT_SMALL = ('DejaVu Sans', 8)
_FONT_VALUE = ('DejaVu Sans Mono', 8)
_TICKS = (0, 20, 40, 60, 80, 100)


class _TkBarPanel:
    """
    One horizontal 0-100% bar per name on a plain Tk canvas, in a FIXED order.

    Every name keeps its own row whether or not it has data this tick; a
    missing one is drawn as an empty grey row. Omitting it used to shift every
    row below it by one slot whenever that term's presence changed, which made
    the panel look like it was jumping although no value had. Rows are in the
    declared order with the first name at the bottom (as matplotlib's barh
    drew them), never sorted by value, so a reader tracking one term does not
    have to re-find its row.

    Drawn with Tk canvas items rather than matplotlib: a matplotlib text label
    costs about 4 ms to render on the developer's machine, which capped the
    old debug window at about 5 Hz, while updating a Tk canvas item is
    near-free. Items are created once; place() lays them out (on a resize)
    and update() only moves bars and changes text.
    """

    def __init__(self, canvas, names, xlabel, red_threshold, title=''):
        c = self.c = canvas
        self.names = list(names)
        self.red_threshold = red_threshold
        self._values = {}
        self._rows_y = []
        self._x0 = self._x1 = 0
        self.title_id = c.create_text(0, 0, text=title, anchor='s', font=_FONT_SMALL)
        self.xlabel_id = c.create_text(0, 0, text=xlabel, anchor='n', font=_FONT_SMALL)
        self.frame_id = c.create_rectangle(0, 0, 0, 0, outline='black')
        self.ticks = [(c.create_line(0, 0, 0, 0), c.create_text(0, 0, text=str(v), anchor='n',
                                                                font=_FONT_SMALL))
                      for v in _TICKS]
        self.rows = [(c.create_text(0, 0, text=n, anchor='e', font=_FONT),
                      c.create_rectangle(0, 0, 0, 0, fill=_BAR_EMPTY, width=0),
                      c.create_text(0, 0, text='', anchor='w', font=_FONT_VALUE))
                     for n in self.names]

    def label_width(self):
        f = tkfont.Font(font=_FONT)
        return max(f.measure(n) for n in self.names)

    def place(self, x0, y0, x1, y1):
        """Lay the plot box out at (x0, y0)-(x1, y1), canvas pixels."""
        c = self.c
        self._x0, self._x1 = x0, x1
        c.coords(self.frame_id, x0, y0, x1, y1)
        c.coords(self.title_id, (x0 + x1) / 2, y0 - 3)
        c.coords(self.xlabel_id, (x0 + x1) / 2, y1 + 18)
        for (line, text), v in zip(self.ticks, _TICKS):
            x = x0 + (x1 - x0) * v / 100.0
            c.coords(line, x, y1, x, y1 + 4)
            c.coords(text, x, y1 + 5)
        h = (y1 - y0) / max(len(self.names), 1)
        self._rows_y = [(y1 - h * (i + 0.5), h * 0.4) for i in range(len(self.names))]
        for (label, _bar, _val), (yc, _hh) in zip(self.rows, self._rows_y):
            c.coords(label, x0 - 6, yc)
        self._draw_bars()

    def update(self, values: dict) -> None:
        """`values` maps a name with data this tick to (pct, label text)."""
        self._values = values
        self._draw_bars()

    def _draw_bars(self):
        if not self._rows_y:
            return
        c, x0, x1 = self.c, self._x0, self._x1
        for name, (_label, bar, val), (yc, hh) in zip(self.names, self.rows, self._rows_y):
            if name in self._values:
                pct, text = self._values[name]
                xe = x0 + (x1 - x0) * max(0.0, min(pct, 100.0)) / 100.0
                c.coords(bar, x0 + 1, yc - hh, xe, yc + hh)
                c.itemconfigure(bar, fill=_BAR_RED if pct >= self.red_threshold else _BAR_BLUE)
                c.coords(val, xe + 5, yc)
                c.itemconfigure(val, text=text)
            else:
                c.coords(bar, x0 + 1, yc - hh, x0 + 1, yc + hh)
                c.itemconfigure(val, text='')


class _TkDebugWindow:
    """
    A top-level Tk window of _TkBarPanels in columns, with a title and one
    header line. Each column is a list of (panel, weight) stacked top to
    bottom with heights in proportion to weight. Re-laid out on resize.
    Closing the window only hides it, so the next controller switch can show
    it again.
    """

    _RIGHT_MARGIN = 230   # room for a near-100% bar's value text
    _TOP = 52             # title and header line

    def __init__(self, master, title, size):
        self.win = tk.Toplevel(master)
        self.win.title(title)
        self.win.geometry(size)
        self.win.withdraw()
        self.win.protocol('WM_DELETE_WINDOW', self.win.withdraw)
        self.canvas = tk.Canvas(self.win, background='white', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.title_id = self.canvas.create_text(0, 8, text=title, anchor='n',
                                                font=('DejaVu Sans', 12))
        self.header_id = self.canvas.create_text(0, 30, text='', anchor='n', font=_FONT)
        self.columns = []
        self.canvas.bind('<Configure>', lambda _e: self.layout())

    def add_column(self, panels_and_weights):
        self.columns.append(list(panels_and_weights))

    def set_header(self, text):
        self.canvas.itemconfigure(self.header_id, text=text)

    def set_visible(self, visible):
        (self.win.deiconify if visible else self.win.withdraw)()

    def layout(self):
        w = self.canvas.winfo_width()
        h = self.canvas.winfo_height()
        if w < 50 or h < 50:
            return
        self.canvas.coords(self.title_id, w / 2, 8)
        self.canvas.coords(self.header_id, w / 2, 30)
        col_w = w / max(len(self.columns), 1)
        for ci, column in enumerate(self.columns):
            left = ci * col_w + 12 + max(p.label_width() for p, _w in column)
            right = (ci + 1) * col_w - self._RIGHT_MARGIN
            total = sum(wt for _p, wt in column)
            gap, y = 56, self._TOP + 18
            usable = h - self._TOP - 18 - gap * len(column)
            for panel, wt in column:
                ph = usable * wt / total
                panel.place(left, y, max(right, left + 50), y + ph)
                y += ph + gap


class _Blitter:
    """
    Redraw only a figure's moving artists over a cached background.

    The background is captured after every full draw (startup, resize, or an
    explicit request), with the moving artists marked animated so the full
    draw leaves them out. update() then restores it, draws only those artists
    and blits the result, which skips the axes, ticks and labels entirely.

    Text is the expensive part (about 4 ms per label on the developer's
    machine, against about 5 ms for every cone, line and the car together), so
    a text artist whose string and position have not changed is restored from
    the pixels cached when it was last drawn instead of being re-rendered.
    That cache holds whatever was underneath the text at the time, so text
    drawn over moving data should have an opaque box (see the map's stats).
    """

    def __init__(self, fig, artists):
        self.fig = fig
        self.artists = list(artists)
        self._bg = None
        self._text_cache = {}
        for a in self.artists:
            a.set_animated(True)
        fig.canvas.mpl_connect('draw_event', self._on_draw)

    def add(self, artist):
        artist.set_animated(True)
        self.artists.append(artist)

    def _on_draw(self, _event):
        canvas = self.fig.canvas
        self._bg = canvas.copy_from_bbox(self.fig.bbox)
        self._text_cache.clear()
        self._draw_artists()

    def _draw_artists(self):
        canvas = self.fig.canvas
        for a in self.artists:
            if not a.get_visible():
                continue
            if not isinstance(a, Text):
                self.fig.draw_artist(a)
                continue
            key = (a.get_text(), a.get_position())
            cached = self._text_cache.get(id(a))
            if cached is not None and cached[0] == key:
                canvas.restore_region(cached[1])
                continue
            self.fig.draw_artist(a)
            if not key[0]:
                self._text_cache.pop(id(a), None)
                continue
            renderer = canvas.get_renderer()
            box = a.get_bbox_patch()
            extent = (box.get_window_extent(renderer) if box is not None
                      else a.get_window_extent(renderer)).expanded(1.0, 1.0).padded(2)
            self._text_cache[id(a)] = (key, canvas.copy_from_bbox(extent))

    def full_redraw(self):
        """Re-render the background (it changed), e.g. a new legend."""
        self._bg = None
        self.fig.canvas.draw_idle()

    def update(self):
        canvas = self.fig.canvas
        if self._bg is None:
            canvas.draw_idle()
            return
        canvas.restore_region(self._bg)
        self._draw_artists()
        canvas.blit(self.fig.bbox)


class LiveViz:
    """The three windows and one update step. See the module docstring."""

    def __init__(self, node):
        self.node = node
        self._last_count = -1
        self._last_present = None
        self._show_stanley = None

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self._build_map()

        # Separate windows for the cost breakdowns, drawn with plain Tk (see
        # _TkBarPanel), on the map window's Tk root. MPC window: left column,
        # the 3 step-0 unit-family panels (own 100% scale each), heights in
        # proportion to their row count so the 9-row tracking panel is not
        # squeezed; right column, every term's horizon-summed share of
        # total_cost on one shared scale (slack is left out, see
        # DEBUG_HORIZON_TERMS, so those bars fall a little short of summing to
        # total_cost).
        master = self.fig.canvas.get_tk_widget()
        self.dbg_win = _TkDebugWindow(master, 'MPC weighted-cost breakdown (debug)', '1300x900')
        c = self.dbg_win.canvas
        self.mpc_panels = [_TkBarPanel(c, names, label, red_threshold=50.0)
                           for _group, label, names in DEBUG_BAR_GROUPS]
        self.dbg_win.add_column([(p, len(p.names) + 1) for p in self.mpc_panels])
        self.horizon_panel = _TkBarPanel(
            c, DEBUG_HORIZON_TERMS, 'Horizon-summed cost (% of true total solver cost)',
            red_threshold=30.0, title='Every term, full predicted horizon')
        self.dbg_win.add_column([(self.horizon_panel, 1)])

        # Stanley's own window, shown instead of the MPC one when Stanley is
        # the active controller.
        self.stanley_win = _TkDebugWindow(master, 'Stanley control-law breakdown (debug)',
                                          '800x600')
        self._stanley_err_labels = {'e_y': 'lateral error (e_y)',
                                    'e_psi': 'heading error (e_psi)'}
        self._stanley_law_labels = {
            'heading_error': 'heading error term',
            'atan_cross_track': 'atan2 cross-track term',
            'yaw_rate_damping': 'yaw-rate damping term',
        }
        sc = self.stanley_win.canvas
        self.stanley_error_panel = _TkBarPanel(
            sc, [self._stanley_err_labels[n] for n in STANLEY_ERROR_TERMS],
            'Tracking error (% of |e_y| + |e_psi|)', red_threshold=60.0,
            title='Heading vs. lateral error')
        self.stanley_law_panel = _TkBarPanel(
            sc, [self._stanley_law_labels[n] for n in STANLEY_LAW_TERMS],
            'Share of total steering magnitude (|heading| + |atan2| + |damping|)',
            red_threshold=60.0, title='Control-law term breakdown')
        self.stanley_win.add_column([(self.stanley_error_panel, 1), (self.stanley_law_panel, 1)])

        self.fig.tight_layout()
        self.blit_map = _Blitter(self.fig, (
            self.sc_left, self.sc_right, self.ln_ref, self.ln_pred, self.ln_trail,
            self.car, self.txt_stats, self.txt_laps))
        self._title = None
        self._last_debug = 0.0
        self._last_stats = 0.0

    # ------------------------------------------------------------------
    # Map window
    # ------------------------------------------------------------------
    def _build_map(self):
        ax = self.ax
        ax.set_aspect('equal')
        # Fixed window around a point VIEW_AHEAD/VIEW_BEHIND-weighted ahead of
        # the car (see _view_origin). Fixed limits are what make blitting
        # possible; the axes therefore read metres relative to that point,
        # not track coordinates.
        r = 0.5 * (VIEW_AHEAD + VIEW_BEHIND) + VIEW_HALF_WIDTH
        ax.set_xlim(-r, r)
        ax.set_ylim(-r, r)
        ax.set_xlabel('x relative to view centre (m)', fontsize=8)
        ax.set_ylabel('y relative to view centre (m)', fontsize=8)
        self.sc_left = ax.scatter([], [], c='tab:blue', marker='^', s=25, label='left (blue)')
        self.sc_right = ax.scatter([], [], c='gold', marker='^', s=25, label='right (yellow)')
        (self.ln_ref,) = ax.plot([], [], c='tab:gray', lw=1.5, ls='--',
                                 label='reference/planner path')
        (self.ln_pred,) = ax.plot([], [], c='tab:red', lw=2.0, marker='o', ms=3,
                                  label='NMPC predicted horizon')
        (self.ln_trail,) = ax.plot([], [], c='tab:green', lw=1.2, alpha=0.7,
                                   label='driven trail')
        self.car = Polygon(np.zeros((3, 2)), closed=True, fc='black', label='car',
                           visible=False)
        ax.add_patch(self.car)
        # Opaque boxes: these are restored from cached pixels between text
        # changes (see _Blitter), so anything showing through would be stale.
        self.txt_stats = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top',
                                 ha='left', fontsize=9, family='monospace',
                                 bbox=dict(boxstyle='round', fc='white'))
        self.txt_laps = ax.text(0.02, 0.02, '', transform=ax.transAxes, va='bottom',
                                ha='left', fontsize=8, family='monospace',
                                bbox=dict(boxstyle='round', fc='lightyellow'))
        self.legend = None

    def _view_origin(self):
        """World point the fixed map window is centred on: midway along the
        VIEW_BEHIND..VIEW_AHEAD span in the car's heading direction."""
        node = self.node
        if not node.have_pose:
            return np.zeros(2)
        fwd = np.array([np.cos(node.car_yaw), np.sin(node.car_yaw)])
        return np.array([node.car_x, node.car_y]) + fwd * 0.5 * (VIEW_AHEAD - VIEW_BEHIND)

    def _update_map(self):
        node = self.node
        o = self._view_origin()
        self.sc_left.set_offsets(node.left_cones - o if node.left_cones.size else _EMPTY)
        self.sc_right.set_offsets(node.right_cones - o if node.right_cones.size else _EMPTY)

        # Precomputed mode: the planner never runs, so ref_path stays empty
        # and static_ref_path carries the reference; draw whichever has data.
        if node.static_ref_path.size:
            ref, ref_label = node.static_ref_path, 'precomputed reference path'
        elif node.ref_path.size:
            ref, ref_label = node.ref_path, 'reference/planner path'
        else:
            ref, ref_label = _EMPTY, self.ln_ref.get_label()
        self.ln_ref.set_data(ref[:, 0] - o[0], ref[:, 1] - o[1])
        self.ln_ref.set_label(ref_label)

        pred = node.nmpc_pred_path if node.nmpc_pred_path.size else _EMPTY
        self.ln_pred.set_data(pred[:, 0] - o[0], pred[:, 1] - o[1])
        trail = np.array(node.trail) if len(node.trail) >= 2 else _EMPTY
        self.ln_trail.set_data(trail[:, 0] - o[0], trail[:, 1] - o[1])

        if node.have_pose:
            tx, ty = get_car_triangle(node.car_x, node.car_y, node.car_yaw)
            self.car.set_xy(np.column_stack([np.asarray(tx) - o[0], np.asarray(ty) - o[1]]))
            self.car.set_visible(True)

        # The legend lists only artists that have data, as before, but is
        # rebuilt only when that set (or the reference label) changes.
        present = (bool(node.left_cones.size), bool(node.right_cones.size), bool(ref.size),
                   ref_label, bool(pred.size), bool(trail.size), node.have_pose)
        if present != self._last_present:
            self._last_present = present
            handles = [h for h, on in zip(
                (self.sc_left, self.sc_right, self.ln_ref, self.ln_pred, self.ln_trail, self.car),
                present[:3] + present[4:]) if on]
            if self.legend is not None:
                self.legend.remove()
            self.legend = self.ax.legend(handles=handles, loc='lower right', fontsize=8)
            self.blit_map.full_redraw()

        controller = node.active_controller()
        label = {'mpc': 'MPC', 'stanley': 'Stanley', 'unknown': '(unknown)'}[controller]
        if label != self._title:
            self._title = label
            self.ax.set_title(f'Live {label} debug view')
            self.blit_map.full_redraw()
        # The numbers change every message, but re-rendering the stats box
        # every frame costs most of a map frame, so its text refreshes at
        # STATS_REDRAW_HZ and is restored from cache in between.
        now = time.monotonic()
        if now - self._last_stats >= 1.0 / STATS_REDRAW_HZ:
            self._last_stats = now
            self.txt_stats.set_text(self._stats_text(controller, label))
        self.txt_laps.set_text(self._laps_text())

    def _stats_text(self, controller, label):
        node = self.node
        stats = (
            f"controller = {label}\n"
            f"v = {node.car_speed:.2f} m/s\n"
            f"steer = {node.steering:+.3f}\n"
            f"throttle = {node.throttle:.2f}  brake = {node.brake:.2f}\n"
        )
        if node.cmd_speed_target is not None:
            stats += f"cmd v_target = {node.cmd_speed_target:.2f} m/s\n"
        stats += f"control topic: {node.control_topic or '(none yet)'}\n"
        # Stanley never predicts a horizon, so the line is MPC-only.
        if controller != 'stanley':
            stats += f"NMPC horizon: {'yes' if node.nmpc_pred_path.size else 'no'}"
        # Progress-term diagnostics (nmpc_progress_enabled only). cap_over
        # separates "the cap is holding the car back" from "the car chose to
        # go slower"; s_gap growing tick over tick means the solve is stuck.
        prog = (node.debug_weights or {}).get('progress')
        if prog:
            v_cap = prog.get('v_cap')
            over = prog.get('speed_cap_over')
            gap = prog.get('s_target_gap_end')
            stats += "\nprogress term:"
            if v_cap is not None:
                stats += f"\n  v_cap = {v_cap:.2f} m/s"
            if over is not None:
                state = 'CAP BINDING' if over > 0.05 else 'under cap'
                stats += f"\n  cap_over = {over:+.2f} m/s ({state})"
            if gap is not None:
                stats += f"\n  s_gap_end = {gap:.2f} m"
        return stats.rstrip('\n')

    def _laps_text(self):
        # One line per completed lap, most recent marked. "none completed"
        # explains an empty panel on a live-planner run, which has no
        # precomputed speed profile and so no lap tracker.
        laps = self.node.lap_summaries
        if not laps:
            return "Laps: none completed (needs precomputed path)"
        lines = []
        for i, lap in enumerate(laps):
            marker = '>' if i == len(laps) - 1 else ' '
            score = lap.get('composite_score')
            lap_time = lap.get('lap_time_s')
            acc = lap.get('pred_acc_pct')
            acc_s = f"{acc:.1f}%" if acc is not None else "n/a"
            time_s = f"{lap_time:.1f}s" if lap_time is not None else "?"
            lines.append(f"{marker} Lap {lap.get('lap_idx', '?')}  {time_s}  "
                         f"score {score:.3f}  horizon {acc_s}")
        return "Laps:\n" + "\n".join(lines)

    # ------------------------------------------------------------------
    # Debug windows
    # ------------------------------------------------------------------
    def _update_mpc_debug(self):
        # Step-0 weighted cost per unit-family group, each as a share of its
        # own group's sum (see mpc_controller.py's _publish_debug_weights());
        # none sums to total_cost, the full-horizon objective in the title.
        dw = self.node.debug_weights
        terms = dw.get('terms', {}) if dw is not None else {}
        for panel in self.mpc_panels:
            values = {}
            for name in panel.names:
                t = terms.get(name)
                if t is None:
                    continue
                # 'steering'/'accel' are synthetic combined bars (effort +
                # rate) with no single error or weight of their own.
                if t['error'] is None:
                    text = f"{t['pct']:.1f}%  (effort + rate combined)"
                else:
                    text = f"{t['pct']:.1f}%  (v={t['error']:+.4f}, w={t['weight']:.2f})"
                values[name] = (t['pct'], text)
            panel.update(values)

        horizon = dw.get('horizon_terms', {}) if dw is not None else {}
        self.horizon_panel.update({
            n: (horizon[n]['pct'], f"{horizon[n]['pct']:.1f}%  (cost={horizon[n]['cost']:.3f})")
            for n in DEBUG_HORIZON_TERMS if n in horizon})

        header = []
        if dw is not None:
            if dw.get('total_cost') is not None:
                header.append(f"true total solver cost = {dw['total_cost']:.4f}")
            if dw.get('solve_ms') is not None:
                header.append(f"solve time = {dw['solve_ms']:.2f} ms")
        self.dbg_win.set_header('   |   '.join(header))

    def _update_stanley_debug(self):
        ds = self.node.debug_stanley
        errors = {'e_y': ds.get('e_y') if ds else None, 'e_psi': ds.get('e_psi') if ds else None}
        total = sum(abs(v) for v in errors.values() if v is not None)
        values = {}
        for n, v in errors.items():
            if v is not None:
                pct = 100.0 * abs(v) / total if total > 0.0 else 0.0
                values[self._stanley_err_labels[n]] = (pct, f"{pct:.1f}%  (v={v:+.4f} rad or m)")
        self.stanley_error_panel.update(values)

        law = ds.get('terms', {}) if ds is not None else {}
        self.stanley_law_panel.update({
            self._stanley_law_labels[n]: (law[n]['pct'],
                                          f"{law[n]['pct']:.1f}%  (v={law[n]['value']:+.4f} rad)")
            for n in STANLEY_LAW_TERMS if n in law})

        header = []
        if ds is not None and ds.get('steering') is not None:
            header.append(f"steering command = {ds['steering']:+.4f} rad")
        self.stanley_win.set_header('   |   '.join(header))

    def _sync_debug_visibility(self, show_stanley):
        """
        Show whichever debug window matches the active controller, and only
        on a transition, or a window the user moved would be fought back into
        place every frame. Both start withdrawn.
        """
        if show_stanley == self._show_stanley:
            return
        self._show_stanley = show_stanley
        self.dbg_win.set_visible(not show_stanley)
        self.stanley_win.set_visible(show_stanley)

    # ------------------------------------------------------------------
    # One frame
    # ------------------------------------------------------------------
    def tick(self, force=False) -> bool:
        """
        Drain queued messages, update every artist and request a redraw.
        Returns False (and draws nothing) when no message arrived since the
        last frame, so an idle stack costs almost no CPU.

        All queued callbacks are processed, not one: the controller publishes
        at 20 Hz and the other topics faster, so one spin_once per frame would
        fall behind and show a stale state. Each callback is a cheap attribute
        write, so the bounded loop empties quickly and stops as soon as
        nothing new arrived.
        """
        node = self.node
        for _ in range(50):
            before = node._callback_count
            rclpy.spin_once(node, timeout_sec=0.0)
            if node._callback_count == before:
                break
        if not force and node._callback_count == self._last_count:
            return False
        self._last_count = node._callback_count

        self._update_map()
        self.blit_map.update()
        show_stanley = node.active_controller() == 'stanley'
        self._sync_debug_visibility(show_stanley)
        # 0.8 of the period, so timer jitter does not make the debug window
        # skip every other tick when the two rates are close.
        now = time.monotonic()
        if not force and now - self._last_debug < 0.8 / DEBUG_REDRAW_HZ:
            return True
        self._last_debug = now
        if show_stanley:
            self._update_stanley_debug()
        else:
            self._update_mpc_debug()
        return True


def main():
    rclpy.init()
    node = LiveVizNode()

    # Tk's mainloop (entered below via plt.show()) is a blocking C event
    # loop and CPython only runs a signal handler between bytecode
    # instructions, so SIGTERM/SIGINT while blocked in Tk was left to chance.
    # launch_all.sh's cleanup() sends SIGTERM (and Ctrl+C sends SIGINT)
    # expecting a prompt exit; os._exit(0) rather than sys.exit()/plt.close()
    # so the process does not wait for Tk to unwind its own loop, which is
    # the step that used to hang.
    def _handle_shutdown_signal(_signum, _frame):
        os._exit(0)
    signal.signal(signal.SIGTERM, _handle_shutdown_signal)
    signal.signal(signal.SIGINT, _handle_shutdown_signal)

    viz = LiveViz(node)
    # One timer drives all three windows: Tk has one event loop, so a
    # draw_idle() on any figure's canvas is serviced from it.
    timer = viz.fig.canvas.new_timer(interval=int(1000.0 / REDRAW_HZ))
    timer.add_callback(viz.tick)
    timer.start()
    # First frame from inside the event loop, so the debug windows are shown
    # or kept hidden after plt.show() has mapped the map window.
    first = viz.fig.canvas.new_timer(interval=50)
    first.single_shot = True
    first.add_callback(lambda: viz.tick(force=True))
    first.start()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
