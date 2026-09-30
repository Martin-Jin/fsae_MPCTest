"""
fsae_control/mpc/debug_publish.py — `_DebugPublishMixin`: lap/horizon bookkeeping and debug topics

`_process_lap_and_horizon` scores completed laps and matures horizon-accuracy
predictions; `_publish_debug_weights` publishes the weighted cost breakdown for
live_viz.py. Split from mpc_controller.py only to keep files readable.
"""

import json
import math

import numpy as np
from std_msgs.msg import String


class _DebugPublishMixin:
    """Lap/horizon bookkeeping and debug-weight publishing for MPCControllerNode."""

    # ------------------------------------------------------------------
    # Debug telemetry (live_viz.py's weighted-error breakdown panel)
    # ------------------------------------------------------------------

    def _process_lap_and_horizon(self, t: float, tel: dict) -> tuple[float | None, float | None, dict | None]:
        """
        One call per tick, shared by both output modes (standalone/cmd_vel)
        below: feeds this tick's pose + (if NMPC) predicted horizon into
        self._horizon_acc, advances self._lap_tracker, and — on the tick a
        lap completes — finalises that lap's score via
        self._telemetry.finish_lap().

        Must be called AFTER self._mpc.compute() (needs tel/last_telemetry)
        and BEFORE self._telemetry.log_control() (its return values feed
        straight into that call's pred_err_m/pred_acc_pct/lap_summary
        kwargs, so the lap's own completing tick logs its score on the same
        row — see log_control()'s docstring for why that ordering matters).

        Returns (pred_err_m, pred_acc_pct, lap_summary): the first two are
        this tick's own OWN matured prediction comparison (None most ticks —
        maturity is ~1 s after the prediction was made, not immediate), the
        third is finish_lap()'s return value on the tick a lap completes,
        else None.
        """
        pred_err_m = pred_acc_pct = None
        if self._horizon_acc is not None:
            fa = self._car_pos + self._mpc.lf * np.array(
                [math.cos(self._car_yaw), math.sin(self._car_yaw)])
            self._horizon_acc.add_pose(t, fa[0], fa[1])
            pred_xy = tel.get('nmpc_pred_xy')
            if pred_xy is not None:
                pose_age_s = tel.get('pose_age_s') or 0.0
                n_delay = tel.get('n_delay') or 0
                n_latency = tel.get('n_latency') or 0
                t_stage0 = t - pose_age_s + (n_delay + n_latency) * self._mpc.dt
                pred_x, pred_y = pred_xy
                self._horizon_acc.add_prediction(
                    t_stage0, self._mpc.dt, pred_x, pred_y, self._car_speed)
            matured = self._horizon_acc.update(t)
            if matured is not None:
                pred_err_m, pred_acc_pct = matured

        lap_summary = None
        if self._lap_tracker is not None:
            lap = self._lap_tracker.update(self._car_pos, t, self._car_speed)
            if lap is not None:
                lap_err = lap_acc = None
                if self._horizon_acc is not None:
                    lap_mean = self._horizon_acc.pop_lap_mean()
                    if lap_mean is not None:
                        lap_err, lap_acc = lap_mean
                lap_summary = self._telemetry.finish_lap(
                    lap, pred_acc_pct=lap_acc, pred_err_m=lap_err)
                lap_summary['lap_idx'] = lap['lap_idx']
                msg = String()
                msg.data = json.dumps(lap_summary)
                self.pub_lap_summary.publish(msg)

        return pred_err_m, pred_acc_pct, lap_summary

    def _publish_debug_weights(self, tel: dict) -> None:
        """
        Weighted-cost breakdown of every term the MPC actually solved this
        tick (error^2 * effective weight, as a share of its own GROUP's sum,
        see `group` below), plus the true solved objective and solve_ms.
        Debug-only, for live_viz.py's bar-graph panel(s).

        Grouped rather than one shared 0-100% scale: tracking errors
        (metres/radians), input effort (the command itself) and input rate
        (change per tick) are wildly different magnitudes squared against
        their own weights, so e.g. a steering-rate term of a few
        milliradians/tick will always round to ~0% next to a 0.3 m lateral
        error even when the rate cost is the one actually dominant within
        its own group -- comparing within a group is the only comparison
        that's meaningful. live_viz.py draws one 100% bar-graph per group.

        The `terms` breakdown is step-0-only (the current tick's
        instantaneous errors/rates/command, not summed over the horizon) for
        both solvers, so it stays an "at a glance, which term is biggest
        right now" signal, not a horizon-summed one. The tracking group also
        carries two synthetic combined entries, 'steering' and 'accel',
        each that input's effort + rate-of-change step-0 cost folded into
        one bar (their constituent steer_effort/accel_effort/delta_u_steer/
        delta_u_accel entries are kept too, for the effort/rate panels).

        `horizon_terms`, published separately, IS the true full-horizon
        per-term cost (see mpc_core.py's _compute_cost_breakdown /
        nmpc_core.py's _cost_breakdown), each shown as a share of
        total_cost -- the exact full-horizon scalar each solver minimised
        (cp.Problem.value for the LTV-QP, the SQP's own converged objective
        for NMPC). These percentages are directly comparable to each other
        (unlike the step-0 `terms` breakdown above), but do NOT sum to
        exactly 100%/total_cost: the soft track-boundary slack cost is
        deliberately excluded from horizon_terms (unused in this debug
        display) even though it's still part of total_cost.

        Falls back to the static MPCParams weight when no corner-blended
        "_eff" value is in last_telemetry (always true for NMPC, which has
        no blending step for Q/R).
        """
        params = self._mpc.params
        a_cmd = tel.get('a_cmd', 0.0)
        # accel/brake effort is one QP term split by sign (see mpc_core.py's
        # _build_qp cp.pos(u)/cp.neg(u) split) -- mirror that split here so
        # exactly one of the two is ever nonzero for a given tick, matching
        # what the solver actually charged rather than double-counting.
        r_a_eff = tel.get('R_a_accel_eff', params.r_a_accel) if a_cmd >= 0.0 \
            else tel.get('R_a_brake_eff', params.r_a_brake)
        terms = {
            'e_y':      ('tracking', tel.get('e_y', 0.0),      tel.get('Q_ey_eff', params.q_e_y)),
            'e_yd':     ('tracking', tel.get('e_yd', 0.0),      params.q_e_yd),
            'e_psi':    ('tracking', tel.get('e_psi', 0.0),    tel.get('Q_epsi_eff', params.q_e_psi)),
            'yaw_rate': ('tracking', tel.get('yaw_rate', 0.0), tel.get('Q_r_eff', params.q_r)),
            # Row 4's NAME follows the mode, because its MEANING does. Under
            # tracking it is the two-sided speed error e_v. Under the NMPC
            # progress term it is a one-sided speed-CAP hinge that reads
            # exactly 0.0 whenever the car is under the cap, i.e. most of a
            # lap -- reporting that as "e_v" makes a working controller look
            # like it has zero speed error, which is the opposite of what a
            # flat bar there means. Keyed off the telemetry the controller
            # actually published, not the parameter, so the label cannot
            # disagree with the running controller.
            ('v_cap_hinge' if 'nmpc_v_cap' in tel else 'e_v'):
                        ('tracking', tel.get('e_v', 0.0),      params.q_e_v),
            'steer_effort': ('effort', tel.get('delta_cmd', 0.0),
                              tel.get('R_steer_eff', params.r_delta)),
            'accel_effort': ('effort', a_cmd, r_a_eff),
            'delta_u_steer': ('rate', tel.get('delta_u_steer', 0.0),
                              tel.get('Rrate_steer_eff',
                                      tel.get('Rrate_steer_corner_blend', params.r_rate_delta))),
            'delta_u_accel': ('rate', tel.get('delta_u_accel', 0.0), params.r_rate_a),
        }
        # NMPC progress term (nmpc_progress_enabled only). Keyed off the
        # telemetry the controller actually published rather than the
        # parameter, so this stays correct if the flag and the running
        # controller ever disagree. The residual is the horizon-END gap
        # (see nmpc_core.py's _outputs: h_prog is zero at every other
        # stage), which is why it is read from nmpc_s_target_gap_end rather
        # than a step-0 quantity like every other row here.
        if 'nmpc_s_target_gap_end' in tel:
            terms['progress'] = ('tracking', tel['nmpc_s_target_gap_end'],
                                 params.nmpc_q_progress)
        costs = {name: weight * error ** 2 for name, (group, error, weight) in terms.items()}

        # Two combined bars for the top (tracking) panel: 'steering' and
        # 'accel' each fold that input's effort + rate-of-change step-0 cost
        # into one bar, so the tracking panel shows how much the two inputs
        # are costing overall alongside the 5 tracking-error terms. The
        # underlying steer_effort/accel_effort/delta_u_steer/delta_u_accel
        # entries stay as their own bars too (still needed by the effort/
        # rate panels) -- these are additional entries, not replacements.
        costs['steering'] = costs['steer_effort'] + costs['delta_u_steer']
        costs['accel'] = costs['accel_effort'] + costs['delta_u_accel']
        terms['steering'] = ('tracking', None, None)
        terms['accel'] = ('tracking', None, None)

        group_totals: dict[str, float] = {}
        for name, (group, _error, _weight) in terms.items():
            group_totals[group] = group_totals.get(group, 0.0) + costs[name]
        breakdown = {
            name: {
                'group': group,
                'error': error,
                'weight': weight,
                'cost': costs[name],
                'pct': (100.0 * costs[name] / group_totals[group])
                       if group_totals[group] > 0.0 else 0.0,
            }
            for name, (group, error, weight) in terms.items()
        }

        # Every term's horizon-summed cost (mpc_core.py's
        # _compute_cost_breakdown / nmpc_core.py's _cost_breakdown, both
        # under last_telemetry['cost_breakdown']['horizon_terms']), each as
        # a share of total_cost -- one shared scale, unlike the step-0
        # groups above, since these are all genuinely comparable: exactly
        # the terms the solver actually summed to reach total_cost.
        total_cost = tel.get('total_cost')
        horizon_terms_raw = (tel.get('cost_breakdown') or {}).get('horizon_terms', {})
        horizon_terms = {
            name: {
                'cost': cost,
                'pct': (100.0 * cost / total_cost) if total_cost else 0.0,
            }
            for name, cost in horizon_terms_raw.items()
        }

        msg = String()
        payload = {
            'terms': breakdown,
            'horizon_terms': horizon_terms,
            'solve_ms': tel.get('solve_ms'),
            'total_cost': total_cost,
        }
        # NMPC progress term (nmpc_progress_enabled only). Sent as their own
        # keys rather than folded into 'terms' because these are raw
        # diagnostics, not weighted cost shares: v_cap/speed_cap_over answer
        # "is the cap binding or did the car choose to go slower", and
        # s_target_gap_end GROWING tick-over-tick is the signature of a
        # stuck solve. Absent on every other run, so live_viz skips the line.
        if 'nmpc_v_cap' in tel:
            payload['progress'] = {
                'v_cap': tel.get('nmpc_v_cap'),
                'speed_cap_over': tel.get('nmpc_speed_cap_over'),
                's_target_gap_end': tel.get('nmpc_s_target_gap_end'),
            }
        msg.data = json.dumps(payload)
        self.pub_debug_weights.publish(msg)
