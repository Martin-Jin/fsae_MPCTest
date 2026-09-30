"""
fsae_control/live_viz/app.py — matplotlib window and `main`

Builds the figure (map, path, horizon, driven trail, stats and bar panels) and
redraws it from a LiveVizNode on a timer. The Tk backend must be selected
before pyplot is imported, so that call stays above the pyplot import here.
"""

import os
import signal

import matplotlib
import numpy as np

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt  # noqa: E402 (backend must be selected first)
from matplotlib.animation import FuncAnimation  # noqa: E402

import rclpy  # noqa: E402

from fsae_control.live_viz.node import LiveVizNode, get_car_triangle  # noqa: E402
from fsae_control.live_viz.panels import (  # noqa: E402
    DEBUG_BAR_GROUPS, DEBUG_HORIZON_TERMS, REDRAW_HZ, STANLEY_ERROR_TERMS,
    STANLEY_LAW_TERMS, VIEW_AHEAD, VIEW_BEHIND, VIEW_HALF_WIDTH,
)


def main():
    rclpy.init()
    node = LiveVizNode()

    # Tk's mainloop (entered below via plt.show()) is a blocking C event
    # loop: CPython only runs a signal handler between bytecode
    # instructions, so with no handler registered here, SIGTERM/SIGINT
    # delivery while blocked in Tk was left to chance -- it only got through
    # incidentally, whenever FuncAnimation's own timer happened to pump the
    # loop back into Python. launch_all.sh's cleanup() sends SIGTERM (and
    # Ctrl+C sends SIGINT) expecting a prompt exit; os._exit(0) rather than
    # sys.exit()/plt.close() so the process dies immediately instead of
    # waiting for Tk to unwind its own C loop cleanly (which is exactly the
    # step that was hanging).
    def _handle_shutdown_signal(_signum, _frame):
        os._exit(0)
    signal.signal(signal.SIGTERM, _handle_shutdown_signal)
    signal.signal(signal.SIGINT, _handle_shutdown_signal)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')

    # Separate window (per the "too much for one window" call): the weighted
    # cost breakdown has nothing spatial about it and doesn't need to share
    # a canvas with the map view. Left column: the 3 step-0 unit-family
    # panels (own 100% scale each). Right column: one tall panel with every
    # term's horizon-summed share of total_cost, on one shared scale, since
    # those percentages are genuinely comparable to each other (slack is
    # deliberately excluded here, unused -- see DEBUG_HORIZON_TERMS, so
    # these bars fall a little short of summing to total_cost).
    #
    # layout='constrained' (NOT the one-shot tight_layout() this used to
    # call once before plt.show()) on both this figure and fig_stanley
    # below: tight_layout() computes fixed axes-position fractions ONCE at
    # startup and never again, so a window later resized smaller than its
    # requested figsize (dragged by the user, or placed smaller by the
    # window manager) has no way to re-reserve room for the y-axis category
    # labels, and they get clipped by the figure's own left edge -- measured
    # directly against a screenshot showing exactly that. constrained_layout
    # re-solves the layout on every draw, including a resize, so the labels
    # always get the margin they actually need at the CURRENT window size.
    # hspace/wspace widened from the gridspec default: the per-panel y-axis
    # labels (particularly the horizon panel's, one shared tall column) were
    # visually overlapping the panel above/beside them at the default
    # spacing once labels started reserving real margin instead of being
    # clipped away.
    fig_dbg = plt.figure(figsize=(13, 9), layout='constrained')
    gs = fig_dbg.add_gridspec(len(DEBUG_BAR_GROUPS), 2, width_ratios=[1.0, 1.0],
                              hspace=0.6, wspace=0.35)
    ax_bars = [fig_dbg.add_subplot(gs[i, 0]) for i in range(len(DEBUG_BAR_GROUPS))]
    ax_horizon = fig_dbg.add_subplot(gs[:, 1])
    mpc_axes = ax_bars + [ax_horizon]

    # Stanley's own debug figure -- separate from fig_dbg (built once,
    # same as fig_dbg, then shown/hidden as a whole depending on which
    # controller is actually active, see redraw_debug()). Two panels: the
    # user's own two asks, one shared window each.
    fig_stanley = plt.figure(figsize=(8, 6), layout='constrained')
    gs_stanley = fig_stanley.add_gridspec(2, 1, hspace=0.6)
    ax_stanley_error = fig_stanley.add_subplot(gs_stanley[0, 0])
    ax_stanley_law = fig_stanley.add_subplot(gs_stanley[1, 0])
    stanley_axes = [ax_stanley_error, ax_stanley_law]

    fig_dbg.suptitle('MPC weighted-cost breakdown (debug)')
    fig_stanley.suptitle('Stanley control-law breakdown (debug)')

    def redraw(_frame):
        # Process every callback queued since the last frame, not just one:
        # at REDRAW_HZ < 50 Hz (the control loop's own rate), a single
        # spin_once per frame falls behind and each redraw would show a
        # stale, queued-up state rather than the latest tick. rclpy has no
        # built-in "drain everything ready right now" call, so spin_once
        # (non-blocking, timeout_sec=0) is called in a bounded loop instead;
        # each of this node's callbacks is a cheap attribute write, so a
        # 50 Hz backlog empties in well under a millisecond, and the loop
        # exits itself (via the callback-count check) once nothing is left,
        # rather than always running to the cap.
        for _ in range(20):
            before = node._callback_count
            rclpy.spin_once(node, timeout_sec=0.0)
            if node._callback_count == before:
                break
        ax.clear()
        ax.set_aspect('equal')

        if node.left_cones.size:
            ax.scatter(node.left_cones[:, 0], node.left_cones[:, 1],
                       c='tab:blue', marker='^', s=25, label='left (blue)')
        if node.right_cones.size:
            ax.scatter(node.right_cones[:, 0], node.right_cones[:, 1],
                       c='gold', marker='^', s=25, label='right (yellow)')

        # Precomputed mode: the planner never runs (sim.launch.py gates it
        # off), so ref_path stays empty and static_ref_path carries the real
        # reference instead -- draw whichever one actually has data, not
        # both (they're never populated at the same time in practice).
        if node.static_ref_path.size:
            ax.plot(node.static_ref_path[:, 0], node.static_ref_path[:, 1],
                    c='tab:gray', lw=1.5, ls='--', label='precomputed reference path')
        elif node.ref_path.size:
            ax.plot(node.ref_path[:, 0], node.ref_path[:, 1],
                    c='tab:gray', lw=1.5, ls='--', label='reference/planner path')

        if node.nmpc_pred_path.size:
            ax.plot(node.nmpc_pred_path[:, 0], node.nmpc_pred_path[:, 1],
                    c='tab:red', lw=2.0, marker='o', ms=3, label='NMPC predicted horizon')

        if len(node.trail) >= 2:
            trail = np.array(node.trail)
            ax.plot(trail[:, 0], trail[:, 1], c='tab:green', lw=1.2, alpha=0.7,
                    label='driven trail')

        if node.have_pose:
            tx, ty = get_car_triangle(node.car_x, node.car_y, node.car_yaw)
            ax.fill(tx, ty, c='black', label='car')

            fwd = np.array([np.cos(node.car_yaw), np.sin(node.car_yaw)])
            right = np.array([np.sin(node.car_yaw), -np.cos(node.car_yaw)])
            center = np.array([node.car_x, node.car_y])
            forward_span = center + fwd * VIEW_AHEAD
            backward_span = center - fwd * VIEW_BEHIND
            ax.set_xlim(min(forward_span[0], backward_span[0]) - VIEW_HALF_WIDTH,
                        max(forward_span[0], backward_span[0]) + VIEW_HALF_WIDTH)
            ax.set_ylim(min(forward_span[1], backward_span[1]) - VIEW_HALF_WIDTH,
                        max(forward_span[1], backward_span[1]) + VIEW_HALF_WIDTH)
            del right  # reserved for a future car-relative (rotated) view

        controller = node.active_controller()
        controller_label = {'mpc': 'MPC', 'stanley': 'Stanley', 'unknown': '(unknown)'}[controller]

        stats = (
            f"controller = {controller_label}\n"
            f"v = {node.car_speed:.2f} m/s\n"
            f"steer = {node.steering:+.3f}\n"
            f"throttle = {node.throttle:.2f}  brake = {node.brake:.2f}\n"
        )
        if node.cmd_speed_target is not None:
            stats += f"cmd v_target = {node.cmd_speed_target:.2f} m/s\n"
        stats += f"control topic: {node.control_topic or '(none yet)'}\n"
        # NMPC horizon line is MPC-specific (Stanley never predicts a
        # horizon at all) -- only shown when MPC is the one actually active,
        # rather than printing a permanently-"no" line for a Stanley run.
        if controller != 'stanley':
            stats += f"NMPC horizon: {'yes' if node.nmpc_pred_path.size else 'no'}"
        # Progress-term diagnostics (nmpc_progress_enabled only; the key is
        # absent on every other run, so nothing is printed then). cap_over
        # separates "the cap is holding the car back" from "the car chose to
        # go slower than it was allowed", which is the question the progress
        # term exists to change the answer to. s_gap GROWING tick-over-tick
        # means the solve is stuck or regressing, not converging.
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
        ax.text(0.02, 0.98, stats.rstrip('\n'), transform=ax.transAxes, va='top', ha='left',
                fontsize=9, family='monospace',
                bbox=dict(boxstyle='round', fc='white', alpha=0.85))

        # Lap panel: one line per completed lap (score + horizon accuracy,
        # see ControlLogger.finish_lap()), most recent highlighted. Shown
        # bottom-left so it doesn't compete with the top-left stats box.
        # "none completed" explains an otherwise-silent panel for a
        # live-planner run (no precomputed speed profile -> no
        # LapProgressTracker -> lap_summary never publishes, see
        # mpc_controller.py's own _lap_tracker construction).
        if node.lap_summaries:
            lines = []
            for i, lap in enumerate(node.lap_summaries):
                marker = '>' if i == len(node.lap_summaries) - 1 else ' '
                score = lap.get('composite_score')
                lap_time = lap.get('lap_time_s')
                acc = lap.get('pred_acc_pct')
                acc_s = f"{acc:.1f}%" if acc is not None else "n/a"
                time_s = f"{lap_time:.1f}s" if lap_time is not None else "?"
                lines.append(
                    f"{marker} Lap {lap.get('lap_idx', '?')}  {time_s}  "
                    f"score {score:.3f}  horizon {acc_s}")
            lap_text = "Laps:\n" + "\n".join(lines)
        else:
            lap_text = "Laps: none completed (needs precomputed path)"
        ax.text(0.02, 0.02, lap_text, transform=ax.transAxes, va='bottom', ha='left',
                fontsize=8, family='monospace',
                bbox=dict(boxstyle='round', fc='lightyellow', alpha=0.85))

        ax.legend(loc='lower right', fontsize=8)
        ax.set_title(f'Live {controller_label} debug view')

    def _draw_pct_bars(ax, all_names, present, pcts, details, red_threshold, xlabel):
        """Shared bar-graph renderer for every debug panel below: a
        horizontal 0-100% bar per name in ALL_NAMES, in that FIXED order,
        every single frame -- red past red_threshold else blue, with
        details[i] appended to each present bar's label.

        Takes the full fixed name list, not just the ones with data this
        tick, and always draws one row per name in ALL_NAMES: a name
        missing from `present` (this frame's actual mode has no data for
        it, e.g. 'progress' when nmpc_progress_enabled is off) still gets
        an empty grey row at its own fixed position instead of being
        omitted. Omitting it used to collapse every row below it upward by
        one slot the instant that term's presence changed tick to tick,
        which is what made the whole panel appear to jump around even
        though no single term's own value did anything unusual -- ax.clear()
        every frame plus barh() placing bars in LIST order (not a fixed
        category axis) means a shorter list is a visually different
        layout, not just fewer bars. Centralised so every panel wraps its
        label text the same way (see the wrap step below, added because
        long detail strings were being clipped past the figure's right
        edge)."""
        ax.clear()
        present_set = set(present)
        pct_by_name = dict(zip(present, pcts))
        detail_by_name = dict(zip(present, details))
        pcts_fixed = [pct_by_name.get(n, 0.0) for n in all_names]
        colors = ['tab:red' if p >= red_threshold else 'tab:blue'
                  if n in present_set else 'lightgrey'
                  for n, p in zip(all_names, pcts_fixed)]
        # ALL_NAMES passed straight through, not reordered: barh's own
        # bottom-to-top placement of a fixed list is exactly what the group
        # panels already rendered before this fix (first declared name at
        # the bottom), so this keeps their look unchanged and gives the
        # horizon panel that same fixed, stable order instead of its old
        # per-frame value sort.
        bars = ax.barh(all_names, pcts_fixed, color=colors)
        for name, bar in zip(all_names, bars):
            if name not in present_set:
                continue
            detail = detail_by_name[name]
            # Bar labels are drawn in DATA coordinates (x in [0, 100],
            # not axes-fraction), so a wide label on a near-100% bar
            # can extend past the axes' right edge and get clipped by
            # the figure boundary -- clip_on=False lets it draw into
            # the figure margin instead (tight_layout/subplots_adjust
            # below reserves that margin), and a fixed-width right
            # margin is reserved on every panel for exactly this.
            ax.text(bar.get_width() + 1.5, bar.get_y() + bar.get_height() / 2,
                    detail, va='center', ha='left', fontsize=7,
                    family='monospace', clip_on=False)
        ax.set_xlim(0, 100)
        ax.set_xlabel(xlabel, fontsize=8)

    def _set_window_visible(fig, visible: bool) -> None:
        """Figure.set_visible() only controls whether the figure's ARTISTS
        draw onto its own canvas -- it does not touch the OS-level window
        the TkAgg backend opened for it, so the inactive controller's debug
        figure (fig_stanley under MPC/NMPC, or vice versa) was left on
        screen showing nothing, a blank numbered "Figure" window with no
        way to tell why it was empty. Tk's own window object, reached via
        the canvas manager, is what actually needs withdraw()/deiconify()."""
        window = fig.canvas.manager.window
        if visible:
            window.deiconify()
        else:
            window.withdraw()

    _debug_visibility_state = {'show_stanley': None}

    def _sync_debug_visibility():
        """Which of the two debug figures is actually meaningful right now
        -- called from both figures' own animations (each figure needs its
        own FuncAnimation to redraw its own canvas; a single animation
        tied to one figure does not repaint a different figure's canvas)."""
        show_stanley = node.active_controller() == 'stanley'
        fig_dbg.set_visible(not show_stanley)
        fig_stanley.set_visible(show_stanley)
        for ax_ in mpc_axes:
            ax_.set_visible(not show_stanley)
        for ax_ in stanley_axes:
            ax_.set_visible(show_stanley)
        # withdraw()/deiconify() are window-manager calls, not cheap artist
        # toggles -- only issue them on an actual transition (this function
        # runs at REDRAW_HZ from TWO animations, ~50 calls/sec combined),
        # or a manually moved/resized debug window gets fought back into
        # place every frame even while the controller never changes.
        if _debug_visibility_state['show_stanley'] != show_stanley:
            _debug_visibility_state['show_stanley'] = show_stanley
            _set_window_visible(fig_dbg, not show_stanley)
            _set_window_visible(fig_stanley, show_stanley)
        return show_stanley

    def redraw_debug(_frame):
        if not _sync_debug_visibility():
            _redraw_mpc_debug()

    def redraw_debug_stanley(_frame):
        if _sync_debug_visibility():
            _redraw_stanley_debug()

    def _redraw_mpc_debug():
        # Weighted-cost breakdown, one bar-graph panel per unit-family group
        # (see DEBUG_BAR_GROUPS) -- which term is costing the solver the
        # most right now, as a share of ITS OWN GROUP's sum. See
        # mpc_controller.py's _publish_debug_weights() for what "weighted
        # cost" means here (error^2 * effective weight, step-0 only) and why
        # no group's bars sum to total_cost (shown in the figure suptitle):
        # total_cost is the true full-horizon solved objective, including
        # terms (full-horizon summation, jerk, slack) this bar graph does
        # not attempt to decompose per-tick.
        dw = node.debug_weights
        terms = dw.get('terms', {}) if dw is not None else {}
        for ax_bar, (_group, label, names) in zip(ax_bars, DEBUG_BAR_GROUPS):
            present = [n for n in names if n in terms]
            pcts = [terms[n]['pct'] for n in present]
            details = []
            for name in present:
                t = terms[name]
                # 'steering'/'accel' are synthetic combined bars (effort
                # + rate folded together, see _publish_debug_weights())
                # with no single error/weight of their own to show.
                if t['error'] is None:
                    detail = f"{t['pct']:.1f}%  (effort + rate combined)"
                else:
                    detail = f"{t['pct']:.1f}%  (v={t['error']:+.4f}, w={t['weight']:.2f})"
                details.append(detail)
            _draw_pct_bars(ax_bar, names, present, pcts, details, red_threshold=50.0, xlabel=label)

        # Right-side panel: every term's horizon-summed cost as a share of
        # total_cost, one shared scale (see DEBUG_HORIZON_TERMS' comment).
        # Fixed row order (DEBUG_HORIZON_TERMS' own declared order), NOT
        # sorted by current value -- sorting by value every frame was
        # exactly what made a term's row visibly jump as its cost share
        # crossed another term's, even though each term's OWN value was
        # moving smoothly. A reader tracking "is e_y still climbing"
        # should not have to re-find e_y's row after every redraw.
        horizon_terms = dw.get('horizon_terms', {}) if dw is not None else {}
        present_h = [n for n in DEBUG_HORIZON_TERMS if n in horizon_terms]
        pcts_h = [horizon_terms[n]['pct'] for n in present_h]
        details_h = [f"{horizon_terms[n]['pct']:.1f}%  (cost={horizon_terms[n]['cost']:.3f})"
                     for n in present_h]
        _draw_pct_bars(ax_horizon, DEBUG_HORIZON_TERMS, present_h, pcts_h, details_h,
                       red_threshold=30.0,
                       xlabel='Horizon-summed cost (% of true total solver cost)')
        ax_horizon.set_title('Every term, full predicted horizon', fontsize=9)

        header = []
        if dw is not None:
            total_cost = dw.get('total_cost')
            if total_cost is not None:
                header.append(f"true total solver cost = {total_cost:.4f}")
            solve_ms = dw.get('solve_ms')
            if solve_ms is not None:
                header.append(f"solve time = {solve_ms:.2f} ms")
        fig_dbg.suptitle('MPC weighted-cost breakdown (debug)'
                          + ('\n' + '   |   '.join(header) if header else ''))

    def _redraw_stanley_debug():
        # Panel 1: Stanley's own two tracking errors (heading, cross-track/
        # lateral), one shared 0-100% scale -- see STANLEY_ERROR_TERMS'
        # comment for why "percentage" here is share of |e_y|+|e_psi|.
        ds = node.debug_stanley
        e_y = ds.get('e_y') if ds is not None else None
        e_psi = ds.get('e_psi') if ds is not None else None
        error_values = {'e_y': e_y, 'e_psi': e_psi}
        total_abs = sum(abs(v) for v in error_values.values() if v is not None)
        present_e = [n for n in STANLEY_ERROR_TERMS if error_values.get(n) is not None]
        pcts_e = [(100.0 * abs(error_values[n]) / total_abs) if total_abs > 0.0 else 0.0
                  for n in present_e]
        labels_e = {'e_y': 'lateral error (e_y)', 'e_psi': 'heading error (e_psi)'}
        details_e = [f"{p:.1f}%  (v={error_values[n]:+.4f} rad or m)"
                     for n, p in zip(present_e, pcts_e)]
        # Fixed row per STANLEY_ERROR_TERMS entry (translated to its own
        # label), not just the ones with data this tick -- same fix as the
        # MPC panels above, so a term temporarily missing (e.g. e_psi not
        # yet published) gets an empty row at its own position instead of
        # collapsing the other row up to fill the gap.
        _draw_pct_bars(ax_stanley_error, [labels_e[n] for n in STANLEY_ERROR_TERMS],
                       [labels_e[n] for n in present_e], pcts_e, details_e,
                       red_threshold=60.0, xlabel='Tracking error (% of |e_y| + |e_psi|)')
        ax_stanley_error.set_title('Heading vs. lateral error', fontsize=9)

        # Panel 2: Stanley's own three additive control-law terms, as a
        # share of the total steering magnitude they combine to produce --
        # see stanley_controller.py's _publish_debug_stanley() and
        # STANLEY_LAW_TERMS' own comment.
        law_terms = ds.get('terms', {}) if ds is not None else {}
        present_l = [n for n in STANLEY_LAW_TERMS if n in law_terms]
        pcts_l = [law_terms[n]['pct'] for n in present_l]
        labels_l = {
            'heading_error': 'heading error term',
            'atan_cross_track': 'atan2 cross-track term',
            'yaw_rate_damping': 'yaw-rate damping term',
        }
        details_l = [f"{law_terms[n]['pct']:.1f}%  (v={law_terms[n]['value']:+.4f} rad)"
                     for n in present_l]
        _draw_pct_bars(ax_stanley_law, [labels_l[n] for n in STANLEY_LAW_TERMS],
                       [labels_l[n] for n in present_l], pcts_l, details_l,
                       red_threshold=60.0,
                       xlabel='Share of total steering magnitude (|heading| + |atan2| + |damping|)')
        ax_stanley_law.set_title('Control-law term breakdown', fontsize=9)

        header = []
        if ds is not None and ds.get('steering') is not None:
            header.append(f"steering command = {ds['steering']:+.4f} rad")
        fig_stanley.suptitle('Stanley control-law breakdown (debug)'
                              + ('\n' + '   |   '.join(header) if header else ''))

    fig.tight_layout()
    # fig_dbg/fig_stanley do NOT call tight_layout() here -- both were
    # created with layout='constrained' above, which re-solves margins on
    # every draw instead of once at startup, and calling tight_layout() on
    # top of that either raises or silently fights it depending on
    # matplotlib version, neither of which is wanted.
    ani = FuncAnimation(fig, redraw, interval=1000.0 / REDRAW_HZ, cache_frame_data=False)
    ani_dbg = FuncAnimation(fig_dbg, redraw_debug, interval=1000.0 / REDRAW_HZ,
                             cache_frame_data=False)
    ani_stanley = FuncAnimation(fig_stanley, redraw_debug_stanley, interval=1000.0 / REDRAW_HZ,
                                 cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
