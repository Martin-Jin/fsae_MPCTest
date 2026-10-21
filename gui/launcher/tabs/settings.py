"""
gui/launcher/tabs/settings.py — Tab 4: Settings. Edits the handful of
commonly-retuned settings/*.py constants (Q/R weights, NMPC overrides,
adaptive feature flags) in place, syncing to the live mpc_params.py/
nmpc_params.py/fsae_params.yaml when present.
"""

from __future__ import annotations

import queue
import re
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from tuner.tools.param_files import (
    _backup_once, _launch_arg_vars, _read_dataclass_field, _read_dataclass_field_desc,
    _LIST_FIELD_MAP, _read_shortlist_var, _read_var, _rewrite_dataclass_field, _rewrite_var,
    _rewrite_yaml_field,
)
from gui.launcher.paths import RepoPaths
from gui.launcher.theme import _field_label, _make_scrollable, center_column, numeric_validator

# ---------------------------------------------------------------------------
# Tab 4: Settings (the settings package)
# ---------------------------------------------------------------------------

# (label, settings name, mpc_params.py field name, kind) -- kind is
# 'float' or 'bool'. mpc_params.py name is None for a field with no live
# equivalent (skipped when syncing to the live file). Order matches
# the settings package's own section grouping.
_SCALAR_FIELDS: list[tuple[str, str, str | None, str]] = [
    ("R_A_ACCEL", "R_A_ACCEL", "r_a_accel", "float"),
    ("R_A_BRAKE", "R_A_BRAKE", "r_a_brake", "float"),
    ("SPEED_TARGET_DEFICIT_MAX", "SPEED_TARGET_DEFICIT_MAX",
     "speed_target_deficit_max", "float"),
    # NMPC only, active on the car right now (launch_all.sh ships these
    # uncommented, non-default) but previously had no Settings-tab widget
    # at all -- only editable by hand-editing launch_all.sh/nmpc_params.py
    # directly, bypassing this tab's settings/mirror/YAML sync.
    ("NMPC_RJERK_DELTA", "NMPC_RJERK_DELTA", "nmpc_rjerk_delta", "float"),
    ("NMPC_RJERK_A", "NMPC_RJERK_A", "nmpc_rjerk_a", "float"),
    ("NMPC_RRATE_ZONE_BOOST_STRAIGHT", "NMPC_RRATE_ZONE_BOOST_STRAIGHT",
     "nmpc_rrate_zone_boost_straight", "float"),
    ("NMPC_RRATE_ZONE_EASE_APPROACH", "NMPC_RRATE_ZONE_EASE_APPROACH",
     "nmpc_rrate_zone_ease_approach", "float"),
    ("NMPC_RRATE_ZONE_FLOOR_CORNER", "NMPC_RRATE_ZONE_FLOOR_CORNER",
     "nmpc_rrate_zone_floor_corner", "float"),
]


# NMPC weight overrides: -1.0 means "inherit the base weight", any other
# value diverges just that one weight for the NMPC -- see the settings package's own
# "NMPC weight overrides" section comment, and mpc_params.py's matching
# nmpc_* fields (same sentinel convention on both sides). Each gets an
# "override" checkbox (checked = use the number field's value, unchecked =
# write -1.0). (label, settings name, mpc_params.py name)
_NMPC_OVERRIDE_FIELDS: list[tuple[str, str, str]] = [
    ("q_e_y", "NMPC_Q_E_Y", "nmpc_q_e_y"),
    ("q_e_yd", "NMPC_Q_E_YD", "nmpc_q_e_yd"),
    ("q_e_psi", "NMPC_Q_E_PSI", "nmpc_q_e_psi"),
    ("q_epsi_dot", "NMPC_Q_EPSI_DOT", "nmpc_q_epsi_dot"),
    ("q_e_v", "NMPC_Q_E_V", "nmpc_q_e_v"),
    ("r_delta", "NMPC_R_DELTA", "nmpc_r_delta"),
    ("r_a_accel", "NMPC_R_A_ACCEL", "nmpc_r_a_accel"),
    ("r_a_brake", "NMPC_R_A_BRAKE", "nmpc_r_a_brake"),
    ("r_rate_delta", "NMPC_R_RATE_DELTA", "nmpc_r_rate_delta"),
    ("r_rate_a", "NMPC_R_RATE_A", "nmpc_r_rate_a"),
    ("terminal_scale", "NMPC_TERMINAL_SCALE", "nmpc_terminal_scale"),
    ("corner_factor_k", "NMPC_CORNER_FACTOR_K", "nmpc_corner_factor_k"),
]


# (label, settings name, length, mpc_params.py field name per list index).
# The index map is tuner/tools/param_files.py's _LIST_FIELD_MAP, shared with
# param_report; None = an index with no live field, skipped when syncing.
_LIST_FIELDS: list[tuple[str, str, int, list[str | None]]] = [
    (name, name, len(fields_), fields_) for name, fields_ in _LIST_FIELD_MAP.items()
]


# Per-index DISPLAY name for each _LIST_FIELDS entry, shown as the row label
# in the Settings tab -- separate from _LIST_FIELDS' own mpc_fields column,
# whose None entries are load-bearing (the save loop uses `mpc_field is
# None` to skip syncing that index to mpc_params.py, since it has no live
# dataclass field). An index with no LIVE field can still be a real,
# always-0 STATE, so it gets the state/input name instead of a generic
# "unused" repeated with no way to tell which state each one is -- and
# R_diag[1] (a_cmd) is not unused at all, just nominal-only (superseded by
# R_A_ACCEL/R_A_BRAKE), so labelling it "unused" was actively wrong, not
# just uninformative. Names from lmpc/controller.py's own state/input vector
# docstring: x = [e_y, e_yd, e_psi, r, e_v, e_a, delta_act, a_act],
# u = [delta_cmd, a_cmd].
_LIST_FIELD_INDEX_NAMES: dict[str, list[str]] = {
    "Q_diag": ["e_y", "e_yd", "e_psi", "r", "e_v", "e_a (always 0)",
               "delta_act (always 0)", "a_act (always 0)"],
    "R_diag": ["delta_cmd", "a_cmd (nominal only, see R_A_ACCEL/R_A_BRAKE)"],
    "R_rate_diag": ["delta_cmd rate", "a_cmd rate"],
}


# Per-index unit/meaning, shown as a muted sub-line under each row's name
# label -- the group-level _LIST_FIELD_DESC below states these once in
# prose above the whole card, but a bare row label like "e_y" with no unit
# or meaning next to IT specifically was easy to lose track of while
# scrolling past 8+ rows. Same content as _LIST_FIELD_DESC, split per row.
_LIST_FIELD_INDEX_DESC: dict[str, list[str]] = {
    "Q_diag": [
        "Lateral deviation from path centreline [1/m^2]",
        "Rate of change of lateral deviation [1/(m/s)^2]",
        "Heading error relative to path tangent [1/rad^2]",
        "Yaw rate [1/(rad/s)^2]",
        "Speed error: car_speed - desired_speed [1/(m/s)^2]",
        "Unused, always weighted 0 -- not a live tunable",
        "Unused, always weighted 0 -- not a live tunable",
        "Unused, always weighted 0 -- not a live tunable",
    ],
    "R_diag": [
        "Steering command effort [1/rad^2]",
        "Acceleration command effort -- NOMINAL ONLY, superseded by "
        "R_A_ACCEL/R_A_BRAKE below, which weight accel/brake independently",
    ],
    "R_rate_diag": [
        "Steering rate of change, i.e. tick-to-tick jerk [1/(rad/s)^2]",
        "Acceleration rate of change, i.e. tick-to-tick jerk [1/(m/s^3)^2]",
    ],
}


# Per-index breakdown for the 3 weight vectors above -- unlike the scalar/
# override fields, a list field has no single mpc_params.py field to read a
# desc/unit from, so these are written out by hand (values taken from
# mpc_params.py's own Q_diag/R_diag/R_rate_diag index-mapping comment).
_LIST_FIELD_DESC: dict[str, str] = {
    "Q_diag": ("State-tracking cost weights [1/unit^2], one per state: "
               "e_y (lateral dev., 1/m^2) · e_yd (lateral dev. rate, 1/(m/s)^2) · "
               "e_psi (heading error, 1/rad^2) · r (yaw rate, 1/(rad/s)^2) · "
               "e_v (speed error, 1/(m/s)^2) · last 3 unused, always 0."),
    "R_diag": ("Input-effort cost weights [1/unit^2]: delta_cmd (steering, 1/rad^2). "
               "The 2nd entry (a_cmd) is nominal-only now -- see R_A_ACCEL/R_A_BRAKE below "
               "for the actual accel/brake effort weights."),
    "R_rate_diag": ("Input rate-of-change cost weights [1/unit^2] (tick-to-tick change, not "
                    "the input itself): delta_cmd rate (1/(rad/s)^2), a_cmd rate (1/(m/s^3)^2)."),
}


# Feature flags, grouped by which controller(s) they affect -- mirrors
# mpc_params.py's own per-field "controller": "both"/"ltv_qp_only"/
# "nmpc_only" metadata tag exactly (that tag is the source of truth for
# this grouping). (label, settings name or None, mpc_params.py name).
# settings name is None when the flag is live-only (delay compensation
# has no offline toggle -- the offline rollout always has it on).
_FEATURE_GROUPS: list[tuple[str, list[tuple[str, str | None, str]]]] = [
    ("Both controllers", [
        ("Delay compensation enabled", None, "delay_compensation_enabled"),
    ]),
    ("LTV-QP only", [
        ("Adaptive Q scaling enabled", "ADAPTIVE_Q_SCALING_ENABLED", "adaptive_q_scaling_enabled"),
        ("Steer-rate anti-hunt enabled", "STEER_RATE_ANTI_HUNT_ENABLED",
         "steer_rate_anti_hunt_enabled"),
        ("Reference-heading rate limit enabled", "REF_HEADING_RATE_LIMIT_ENABLED",
         "ref_heading_rate_limit_enabled"),
        ("Reversal penalty enabled (experimental)", "REVERSAL_PENALTY_ENABLED",
         "reversal_penalty_enabled"),
    ]),
    ("NMPC only", [
        ("Steer-rate anti-hunt enabled (experimental)", "NMPC_STEER_RATE_ANTI_HUNT_ENABLED",
         "nmpc_steer_rate_anti_hunt_enabled"),
        ("Reversal penalty enabled (experimental)", "NMPC_REVERSAL_PENALTY_ENABLED",
         "nmpc_reversal_penalty_enabled"),
        ("Rate-cost stage ramp enabled (experimental)", "NMPC_RRATE_STAGE_RAMP_ENABLED",
         "nmpc_rrate_stage_ramp_enabled"),
        ("Rate-cost 3-zone schedule enabled", "NMPC_RRATE_ZONE_ENABLED",
         "nmpc_rrate_zone_enabled"),
        ("Corner rate-blend enabled (experimental)", "NMPC_CORNER_RRATE_BLEND_ENABLED",
         "nmpc_corner_rrate_blend_enabled"),
        # NMPC_PROGRESS_ENABLED itself is NOT here: it is a launch_all.sh
        # CLI arg, which overrides this tab's dataclass-default/YAML writes
        # at ROS param declaration time, so a checkbox here could be
        # toggled with zero effect on the next real launch. The Launch
        # tab's own checkbox is the only widget that touches the
        # launch_all.sh shortlist line and is the sole place to enable/
        # disable this. The numeric fields below (q_progress etc.) stay
        # here since they have no such precedence conflict.
    ]),
]


# NMPC progress-term numeric settings (see the settings package's own block and
# docs/logs/nmpc_progress_term_investigation.md). Plain scalars, NOT the
# -1.0-inherit override convention _NMPC_OVERRIDE_FIELDS uses: none of
# these has a base weight to inherit from, the rows they weight do not
# exist at all unless NMPC_PROGRESS_ENABLED is on. Only read when it is.
# (label, settings name, help text)
_NMPC_PROGRESS_FIELDS: list[tuple[str, str, str]] = [
    ("q_progress", "NMPC_Q_PROGRESS",
     "Progress-reward weight [1/m^2]. NARROW usable band at r_a_accel=1.0: "
     "below ~5 the car never breaks static friction and never launches, "
     "above ~6 it carries too much speed into corners and goes off-track."),
    ("progress_reach", "NMPC_PROGRESS_REACH",
     "How far out of reach the progress target sits [-]. The kinematic "
     "floor term is what lets the car launch at all (the speed cap is "
     "deliberately small at a standing start); below ~1.8 it stalls."),
    ("progress_v_min", "NMPC_PROGRESS_V_MIN",
     "Low-speed floor [m/s] sharing the speed-cap row and weight. Guards "
     "the standstill trivial solution."),
    ("slack_linear_weight", "NMPC_SLACK_LINEAR_WEIGHT",
     "LINEAR track-boundary slack penalty [1/m], on top of the quadratic "
     "one. 0 = off. Effectively REQUIRED with the progress term: a purely "
     "quadratic penalty has zero gradient at zero violation and the "
     "progress reward exploits that. 1000 turned an off-track DNF into a "
     "completed lap."),
    ("track_halfwidth", "NMPC_TRACK_HALFWIDTH",
     "Soft |e_y| bound [m] where BOTH the quadratic and linear slack above "
     "start penalising. Narrowed from the LTV-QP's 3.5 m to 3.0 m for the "
     "progress-term experiment: progress has an analytic incentive to hug "
     "the boundary, so the slack needs headroom to catch a mistake before "
     "the car reaches the true track edge, not right at it."),
]



# Applicability tag for each _FEATURE_GROUPS group (see SettingsTab._gate).
_FEATURE_GROUP_TAGS = {"Both controllers": "mpc", "LTV-QP only": "ltv", "NMPC only": "nmpc"}

# _NMPC_PROGRESS_FIELDS entries that are read only when the progress term is
# on. The slack and track-halfwidth rows apply to every NMPC run.
_PROGRESS_ONLY_FIELDS = {"NMPC_Q_PROGRESS", "NMPC_PROGRESS_REACH", "NMPC_PROGRESS_V_MIN"}


def _launch_context_from_file(launch_all_sh: Path) -> dict:
    """Controller ('stanley'/'ltv'/'nmpc') and progress-term state as
    launch_all.sh currently sets them; the starting point before the Launch
    tab is bound."""
    if not launch_all_sh.is_file():
        return {"controller": "nmpc", "progress": False}
    controller = (_read_var(launch_all_sh, "CONTROLLER") or "mpc").strip()
    use_nmpc = (_read_var(launch_all_sh, "USE_NMPC") or "false").strip().lower() == "true"
    name = "stanley" if controller == "stanley" else ("nmpc" if use_nmpc else "ltv")
    state = _read_shortlist_var(launch_all_sh, "NMPC_PROGRESS_ENABLED")
    progress = bool(state and state[0] and state[1].strip().lower() == "true")
    return {"controller": name, "progress": name == "nmpc" and progress}


def _profile_field_names() -> list[str]:
    """Every settings NAME the Settings tab reads/writes, derived from
    the same field-group tables the tab itself builds its widgets from
    (rather than a separately hand-maintained list, which would silently
    drift the moment a new field is added to one table but not the other).
    A profile snapshot is exactly this set of NAME=value assignments, so
    saving/loading one is guaranteed to cover the tab's whole surface."""
    names: list[str] = []
    for _label, name, _length, _mpc_fields in _LIST_FIELDS:
        names.append(name)
    for _label, name, _mpc_field, _kind in _SCALAR_FIELDS:
        names.append(name)
    for _label, name, _mpc_field in _NMPC_OVERRIDE_FIELDS:
        names.append(name)
    for _label, name, _desc in _NMPC_PROGRESS_FIELDS:
        names.append(name)
    for _group_title, entries in _FEATURE_GROUPS:
        for _label, settings_name, mpc_field in entries:
            names.append(settings_name if settings_name is not None else mpc_field)
    return names



# How long a Settings-tab status message (a save confirmation, or the muted
# post-flash state) stays on screen before self-clearing. Long enough to
# read a short sentence without feeling rushed, short enough that it can't
# be mistaken for describing the CURRENT state after the user keeps editing.
_STATUS_AUTO_CLEAR_MS = 4000



class SettingsTab(ttk.Frame):
    def __init__(self, parent: ttk.Notebook, paths: RepoPaths) -> None:
        super().__init__(parent, padding=0)
        self._paths = paths
        self._backed_up: set[Path] = set()
        self._launch_vars = (_launch_arg_vars(paths.launch_all_sh)
                             if paths.launch_all_sh.is_file() else {})
        # Widgets greyed out when their setting does not apply to the current
        # selection (see _refresh_applicability). The controller and progress
        # state come from the Launch tab once bind_launch_context() is called;
        # until then they are read from launch_all.sh.
        self._gated: list[tuple[ttk.Widget, tuple]] = []
        self._context = _launch_context_from_file(paths.launch_all_sh)

        header = ttk.Frame(self, padding=(24, 24, 24, 0))
        header.pack(fill="x")
        center_column(header, top=24, bottom=0)
        ttk.Label(header, text="Settings", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(header, text="the settings package — saved changes apply the next time it's imported.",
                  style="Muted.TLabel").pack(anchor="w", pady=(2, 0))

        body = _make_scrollable(self, body_padding=(24, 16, 24, 24))

        def section(title: str, desc: str = "") -> ttk.Frame:
            ttk.Label(body, text=title, style="SectionHeading.TLabel").pack(anchor="w", pady=(20, 2))
            if desc:
                ttk.Label(body, text=desc, style="Muted.TLabel", wraplength=560).pack(
                    anchor="w", pady=(0, 8))
            else:
                ttk.Frame(body, height=6).pack()
            card = ttk.Frame(body, style="Card.TFrame", padding=16)
            card.pack(fill="x")
            return card

        weights_card = section(
            "Core weights",
            "The QP cost's Q/R/R_rate diagonals -- how expensive each tracking error or "
            "control action is. Shared by LTV-QP and NMPC (NMPC overrides below can diverge "
            "from these per-weight).")
        self._list_vars: dict[str, list[tk.StringVar]] = {}
        r = 0
        for label, name, length, mpc_fields in _LIST_FIELDS:
            desc = _LIST_FIELD_DESC.get(name, "")
            _field_label(weights_card, r, label, desc, label_style="Card.TLabel",
                         desc_style="CardMuted.TLabel", wraplength=520)
            r += 2 if desc else 1
            raw = _read_var(paths.settings_dir, name) or "[]"
            current = _parse_float_list(raw, length)
            for i, mpc_field in enumerate(mpc_fields):
                pinned = _to_float(self._pinned_launch_value(mpc_field))
                if pinned is not None:
                    current[i] = pinned
            # One labelled row per index, not one wide horizontal row -- the
            # old layout packed up to 8 entries side by side in a fixed-width
            # card, which ran off the visible window with no way to see or
            # reach the tail entries. Row label is the actual state/input
            # name (_LIST_FIELD_INDEX_NAMES), NOT mpc_fields[i] -- mpc_fields'
            # own None entries mean "no live dataclass field to sync to",
            # which is a different thing from "this index is unused": R_diag
            # index 1 (a_cmd) is None there but is a real, nominal-only
            # weight, not one that's always 0.
            index_names = _LIST_FIELD_INDEX_NAMES.get(name, mpc_fields)
            index_descs = _LIST_FIELD_INDEX_DESC.get(name, [])
            vars_for_field = []
            for i, v in enumerate(current):
                index_label = index_names[i] if i < len(index_names) else f"index {i}"
                index_desc = index_descs[i] if i < len(index_descs) else ""
                # Same label+description two-row convention every other
                # field on this tab uses (_field_label), so a bare row name
                # like "e_y" always has its unit/meaning directly under it
                # instead of relying on the one group-level description
                # above the whole card, which is easy to lose track of
                # while scrolling past 8+ rows.
                ttk.Label(weights_card, text=f"  {index_label}", style="CardMuted.TLabel").grid(
                    row=r, column=0, sticky="nw", padx=(16, 0), pady=(6, 0 if index_desc else 6))
                var = tk.StringVar(value=str(v))
                entry = ttk.Entry(weights_card, textvariable=var, width=10)
                entry.grid(row=r, column=1, sticky="w", pady=(6, 0 if index_desc else 6))
                self._gate(entry, "mpc")
                if index_desc:
                    desc_label = ttk.Label(weights_card, text=f"  {index_desc}",
                                           style="CardMuted.TLabel", wraplength=440)
                    desc_label.grid(row=r + 1, column=0, sticky="nw", padx=(16, 0), pady=(0, 6))
                    if name == "Q_diag" and i == 4:
                        self._e_v_desc = (desc_label, index_desc)
                vars_for_field.append(var)
                r += 2 if index_desc else 1
            self._list_vars[name] = vars_for_field

        ttk.Separator(weights_card).grid(row=r, column=0, columnspan=2, sticky="ew", pady=12)
        r += 1

        self._scalar_vars: dict[str, tk.Variable] = {}
        for label, name, _mpc_field, kind in _SCALAR_FIELDS:
            desc = _read_dataclass_field_desc(paths.mpc_params_py, _mpc_field) if _mpc_field else ""
            if not desc and _mpc_field:
                # Structural NMPC-only scalars (e.g. nmpc_rjerk_delta, the
                # rrate-zone endpoints) live in nmpc_params.py, not
                # mpc_params.py -- same fallback the feature-flag loop
                # below already needs for the same reason.
                desc = _read_dataclass_field_desc(paths.nmpc_params_py, _mpc_field)
            _field_label(weights_card, r, label, desc, label_style="Card.TLabel",
                         desc_style="CardMuted.TLabel", wraplength=420)
            raw = self._pinned_launch_value(_mpc_field) or _read_var(paths.settings_dir, name)
            if kind == "bool":
                var = tk.BooleanVar(value=(raw or "False").strip().lower() == "true")
                widget = ttk.Checkbutton(weights_card, style="Card.TCheckbutton", variable=var)
            else:
                var = tk.StringVar(value=raw or "0.0")
                widget = ttk.Entry(weights_card, textvariable=var, width=10)
            widget.grid(row=r, column=1, sticky="w")
            if name.startswith("NMPC_RRATE_ZONE_"):
                self._gate(widget, "nmpc", "zone")
            elif name.startswith("NMPC_"):
                self._gate(widget, "nmpc")
            else:
                self._gate(widget, "mpc")
            self._scalar_vars[name] = var
            r += 2 if desc else 1

        overrides_card = section(
            "NMPC weight overrides",
            "Unchecked = inherit the matching core weight above (-1.0 sentinel). Check "
            "\"override\" and set a value to diverge just that one weight for the NMPC only.")
        self._override_vars: dict[str, tuple[tk.BooleanVar, tk.StringVar]] = {}
        r = 0
        for label, name, mpc_field in _NMPC_OVERRIDE_FIELDS:
            desc = _read_dataclass_field_desc(paths.mpc_params_py, mpc_field)
            _field_label(overrides_card, r, label, desc, label_style="Card.TLabel",
                         desc_style="CardMuted.TLabel", columnspan=3, wraplength=520)
            raw = (self._pinned_launch_value(mpc_field)
                   or _read_var(paths.settings_dir, name) or "-1.0")
            is_override = _to_float(raw) is not None and _to_float(raw) >= 0.0
            enabled_var = tk.BooleanVar(value=is_override)
            value_var = tk.StringVar(value=raw if is_override else "")
            check = ttk.Checkbutton(overrides_card, text="override", style="Card.TCheckbutton",
                                    variable=enabled_var)
            check.grid(row=r, column=1, sticky="w", padx=(0, 12))
            entry = ttk.Entry(overrides_card, textvariable=value_var, width=10)
            entry.grid(row=r, column=2, sticky="w")
            self._gate(check, "nmpc")
            self._gate(entry, "nmpc", ("override", name))
            self._override_vars[name] = (enabled_var, value_var)
            r += 2 if desc else 1

        progress_card = section(
            "NMPC progress term (experimental) and track-boundary slack",
            "The first three rows are only read when the NMPC progress term is enabled in "
            "the feature flags below; OFFLINE-ONLY so far, it completes a lap but does not "
            "yet beat the tracking controller it would replace. track_halfwidth is always "
            "read by the NMPC (progress mode or not) -- it is the soft track boundary both "
            "slack terms above are measured against. See "
            "docs/logs/nmpc_progress_term_investigation.md.")
        self._progress_vars: dict[str, tk.StringVar] = {}
        r = 0
        for label, name, desc in _NMPC_PROGRESS_FIELDS:
            _field_label(progress_card, r, label, desc, label_style="Card.TLabel",
                         desc_style="CardMuted.TLabel", wraplength=520)
            var = tk.StringVar(value=self._pinned_launch_value(name.lower())
                               or _read_var(paths.settings_dir, name) or "0.0")
            entry = ttk.Entry(progress_card, textvariable=var, width=10)
            entry.grid(row=r, column=1, sticky="w")
            if name in _PROGRESS_ONLY_FIELDS:
                self._gate(entry, "nmpc", "progress")
            else:
                self._gate(entry, "nmpc")
            self._progress_vars[name] = var
            r += 2 if desc else 1

        # Feature flags, grouped by which controller(s) they affect -- see
        # _FEATURE_GROUPS' own comment for why this grouping is trustworthy
        # (it mirrors mpc_params.py's own per-field "controller" metadata).
        self._feature_vars: dict[str, tk.BooleanVar] = {}
        for group_title, entries in _FEATURE_GROUPS:
            group_card = section(f"Feature flags · {group_title}")
            r = 0
            for label, settings_name, mpc_field in entries:
                desc = _read_dataclass_field_desc(paths.mpc_params_py, mpc_field)
                if not desc:
                    # Structural NMPC flags live in nmpc_params.py, not
                    # mpc_params.py -- fall back to it so those rows still
                    # get their help text instead of rendering bare.
                    desc = _read_dataclass_field_desc(paths.nmpc_params_py, mpc_field)
                if settings_name is None:
                    desc = (desc + " " if desc else "") + "(live-only, no settings equivalent)"
                _field_label(group_card, r, label, desc, label_style="Card.TLabel",
                             desc_style="CardMuted.TLabel", wraplength=520)
                raw = self._pinned_launch_value(mpc_field)
                if raw is None and settings_name:
                    raw = _read_var(paths.settings_dir, settings_name)
                if raw is None:
                    raw = _read_dataclass_field(paths.mpc_params_py, mpc_field)
                var = tk.BooleanVar(value=(raw or "False").strip() in ("True", "true"))
                check = ttk.Checkbutton(group_card, style="Card.TCheckbutton", variable=var)
                check.grid(row=r, column=1, sticky="w")
                tags = [_FEATURE_GROUP_TAGS[group_title]]
                if mpc_field == "nmpc_steer_rate_anti_hunt_enabled":
                    tags.append("no_blend")
                self._gate(check, *tags)
                # Keyed by mpc_field (always present) since settings_name can be None.
                self._feature_vars[mpc_field] = var
                r += 2 if desc else 1

        footer = ttk.Frame(body)
        footer.pack(fill="x", pady=(24, 0))
        ttk.Button(footer, text="Save", style="Accent.TButton",
                   command=self._on_save).pack(side="left")
        # Separate button, separate destinations: Save (above) writes
        # settings.py + the live dataclasses/YAML + the fsds_simulator
        # mirror, all files THIS GUI itself edits. "Overwrite All Params"
        # instead pushes the live checkout's CURRENT on-disk param files
        # (tuner/tools/sync_mpc_params.py's own source) into fsae_autonomous
        # and the fsds_simulator mirror -- it does not read anything this
        # tab's widgets hold, and in particular can run with unsaved
        # Settings-tab edits still pending (those are a separate, later
        # sync once THIS session's edits are themselves saved and pushed).
        self.overwrite_all_button = ttk.Button(
            footer, text="Overwrite All Params...", command=self._on_overwrite_all_params)
        self.overwrite_all_button.pack(side="left", padx=(10, 0))
        self.status_var = tk.StringVar(value="")
        self.status_label = ttk.Label(footer, textvariable=self.status_var, style="Muted.TLabel")
        self.status_label.pack(side="left", padx=(14, 0))
        # Cancels a pending auto-clear/flash-revert self.after() job when a
        # newer status message (or another dirty edit) supersedes it before
        # it fires -- without this, an old timer firing late could blank a
        # brand new message or drop the flash color back to muted too soon.
        self._status_clear_job: str | None = None

        # Dirty-state tracking: wired up now that every widget dict above is
        # fully populated, rather than one trace_add() call per creation
        # site (7 of them, easy to add an 8th field type later and forget
        # the trace). is_dirty toggles true the moment ANY tracked Variable
        # changes; _on_save()'s own success path is the only thing that
        # clears it, so "dirty" tracks "differs from the last save", not
        # "differs from the last settings import".
        self._is_dirty = False
        self._dirty_trace_ids: list[tuple[tk.Variable, str]] = []
        self._wire_dirty_tracking()

        # Every Entry on this tab is a number; refuse letters as they are typed.
        for widget, _tags in self._gated:
            if isinstance(widget, ttk.Entry):
                widget.configure(validate="key", validatecommand=numeric_validator(widget))

        # Unsaved-edit highlight: each field is compared with the value it had
        # at the last save (or when the tab was built) and shown red while it
        # differs, so after the "unsaved changes" prompt the tab shows which
        # fields were not saved. Typing a value back to the saved one clears it.
        self._widget_for_var = {}
        for widget, _tags in self._gated:
            opt = "textvariable" if isinstance(widget, ttk.Entry) else "variable"
            self._widget_for_var[str(widget.cget(opt))] = widget
        self._saved_values: dict[str, str] = {}
        self._snapshot_saved()
        for var, _tid in self._dirty_trace_ids:
            var.trace_add("write", lambda name, *_a: self._update_unsaved_highlight(name))

        # Settings that switch other settings on or off.
        for var in (self._feature_vars["nmpc_rrate_zone_enabled"],
                    self._feature_vars["nmpc_corner_rrate_blend_enabled"],
                    *(enabled for enabled, _value in self._override_vars.values())):
            var.trace_add("write", lambda *_a: self._refresh_applicability())
        self._refresh_applicability()

    # ------------------------------------------------------------------
    # Applicability: grey out what the current selection does not use
    # ------------------------------------------------------------------
    def _gate(self, widget: ttk.Widget, *tags) -> None:
        """Register `widget` as usable only when every tag holds (see
        _tag_holds)."""
        self._gated.append((widget, tags))

    def _tag_holds(self, tag) -> bool:
        controller = self._context["controller"]
        if tag == "mpc":
            return controller in ("ltv", "nmpc")
        if tag == "ltv":
            return controller == "ltv"
        if tag == "nmpc":
            return controller == "nmpc"
        if tag == "progress":
            return self._context["progress"]
        if tag == "zone":
            return self._feature_vars["nmpc_rrate_zone_enabled"].get()
        if tag == "no_blend":
            # The corner rate-blend takes priority over anti-hunt when both
            # are on (nmpc/solver.py), so anti-hunt would be ignored.
            return not self._feature_vars["nmpc_corner_rrate_blend_enabled"].get()
        if isinstance(tag, tuple) and tag[0] == "override":
            return self._override_vars[tag[1]][0].get()
        return True

    def _refresh_applicability(self) -> None:
        """Disable every registered widget whose setting has no effect with
        the current controller, progress-term and feature-flag selection.
        A disabled field keeps its value and is still saved, so switching
        back restores it unchanged."""
        for widget, tags in self._gated:
            usable = all(self._tag_holds(t) for t in tags)
            widget.state(["!disabled"] if usable else ["disabled"])
        # e_v stays editable with the progress term on: the NMPC still uses
        # this weight (or its NMPC override), on the one-sided speed-cap
        # hinge instead of the two-sided speed error. Say so on the row.
        label, normal = self._e_v_desc
        if self._context["controller"] == "nmpc" and self._context["progress"]:
            label.configure(text="  Progress term on: weights the one-sided speed-CAP hinge "
                                 "(only speed above the cap is penalised), not a two-sided "
                                 "speed error. Usually needs its own value [1/(m/s)^2]")
        else:
            label.configure(text=f"  {normal}")

    def bind_launch_context(self, launch_tab) -> None:
        """Follow the Launch tab's controller and progress-term selection
        live, so this tab greys out what the next launch will not use."""
        def update(*_args):
            controller = launch_tab.controller_var.get()
            self._context = {
                "controller": controller,
                "progress": controller == "nmpc" and launch_tab.progress_var.get(),
            }
            self._refresh_applicability()
        launch_tab.controller_var.trace_add("write", update)
        launch_tab.progress_var.trace_add("write", update)
        update()

    def _sync_live_field(self, mpc_field: str, literal: str, errors: list[str]) -> None:
        """Writes mpc_field's value to every place a launched node could
        actually read it from, not just the dataclass default the rest of
        this tab edits. Under control.launch.py the dataclass default shadows
        fsae_params.yaml, but a node started without that launch file reads
        the YAML, so both are kept equal. Tries the live dataclass fields first (one
        of mpc_params.py/nmpc_params.py will have it, never both), then the
        matching fsds_simulator/ mirror copies (the change-ledger for
        fsae_planning, see CLAUDE.md's "Third copy" section), then both
        fsae_params.yaml copies. Missing files (e.g. a layout this tool
        doesn't recognise) are skipped rather than reported as errors --
        only a field that SHOULD exist somewhere but was found nowhere at
        all is an error, mirroring the existing per-call-site behaviour.

        `literal` is Python syntax (True/False/repr(float)), matching what
        the dataclass writers need. YAML's own boolean spelling is lowercase
        (true/false); this file's other booleans are already lowercase (one
        pre-existing `steer_rate_anti_hunt_enabled: True` is drift, not the
        convention to match), so the YAML writes below use a translated
        `yaml_literal` instead of `literal` directly."""
        p = self._paths
        wrote_dataclass = (
            _rewrite_dataclass_field(p.mpc_params_py, mpc_field, literal)
            or _rewrite_dataclass_field(p.nmpc_params_py, mpc_field, literal))
        if not wrote_dataclass:
            errors.append(f"{mpc_field}: field not found in mpc_params.py or nmpc_params.py")
        if p.mirror_mpc_params_py.is_file() or p.mirror_nmpc_params_py.is_file():
            wrote_mirror = (
                _rewrite_dataclass_field(p.mirror_mpc_params_py, mpc_field, literal)
                or _rewrite_dataclass_field(p.mirror_nmpc_params_py, mpc_field, literal))
            if not wrote_mirror:
                errors.append(f"{mpc_field}: field not found in the fsds_simulator/ mirror")
        yaml_literal = {"True": "true", "False": "false"}.get(literal, literal)
        if p.fsae_params_yaml.is_file():
            if not _rewrite_yaml_field(p.fsae_params_yaml, mpc_field, yaml_literal):
                errors.append(f"{mpc_field}: key not found in fsae_params.yaml")
        if p.mirror_fsae_params_yaml.is_file():
            if not _rewrite_yaml_field(p.mirror_fsae_params_yaml, mpc_field, yaml_literal):
                errors.append(f"{mpc_field}: key not found in the mirror fsae_params.yaml")
        # An uncommented launch_all.sh shortlist line is passed as a launch
        # arg, which beats both the YAML and the dataclass default. Without
        # this write, a save (or a loaded profile) changed every file above
        # while FSDS kept running the shortlist's old value, and the offline
        # settings silently disagreed with the car. Commented-out lines are
        # left commented: they pass nothing, so the dataclass default written
        # above already wins (control.launch.py forwards it after the YAML).
        var = self._launch_vars.get(mpc_field)
        if var and self._pinned_launch_value(mpc_field) is not None:
            if not _rewrite_var(p.launch_all_sh, var, yaml_literal):
                errors.append(f"{mpc_field}: launch_all.sh line {var} could not be rewritten")

    def _pinned_launch_value(self, mpc_field: str | None) -> str | None:
        """The value an UNCOMMENTED launch_all.sh shortlist line passes for
        mpc_field, or None when the line is commented out or absent. Read
        fresh each call so a hand edit made while the GUI is open is seen.
        Widgets start from this value when present, because it is what FSDS
        actually runs."""
        var = self._launch_vars.get(mpc_field) if mpc_field else None
        if not var or not self._paths.launch_all_sh.is_file():
            return None
        return _read_var(self._paths.launch_all_sh, var)

    def _on_save(self) -> None:
        settings_path = self._paths.settings_dir
        mpc_params_path = self._paths.mpc_params_py
        nmpc_params_path = self._paths.nmpc_params_py
        sync_live = mpc_params_path.is_file()
        try:
            errors: list[str] = []
            _backup_once(settings_path, self._backed_up)
            if sync_live:
                for path in (mpc_params_path, nmpc_params_path,
                             self._paths.mirror_mpc_params_py, self._paths.mirror_nmpc_params_py,
                             self._paths.fsae_params_yaml, self._paths.mirror_fsae_params_yaml,
                             self._paths.launch_all_sh):
                    if path.is_file():
                        _backup_once(path, self._backed_up)
            elif not getattr(self, "_warned_no_live_file", False):
                self._warned_no_live_file = True
                messagebox.showwarning(
                    "Settings tab",
                    f"Live file not found at:\n{mpc_params_path}\n\n"
                    "Saving the settings package only -- the live simulator's mpc_params.py will "
                    "NOT be updated. This can happen if the repo layout differs from what "
                    "this tool assumes (fsae_MPCTest and ros2/ as siblings).")

            for label, name, length, mpc_fields in _LIST_FIELDS:
                vars_for_field = self._list_vars[name]
                try:
                    values = [float(v.get()) for v in vars_for_field]
                except ValueError:
                    errors.append(f"{label}: not all entries are numbers")
                    continue
                literal = "[" + ", ".join(repr(v) for v in values) + "]"
                if not _rewrite_var(settings_path, name, literal):
                    errors.append(f"{name}: assignment not found in any settings/*.py file")
                if sync_live:
                    for value, mpc_field in zip(values, mpc_fields):
                        if mpc_field is None:
                            continue
                        self._sync_live_field(mpc_field, repr(value), errors)

            for _label, name, mpc_field, kind in _SCALAR_FIELDS:
                var = self._scalar_vars[name]
                if kind == "bool":
                    literal = "True" if var.get() else "False"
                else:
                    try:
                        literal = repr(float(var.get()))
                    except ValueError:
                        errors.append(f"{name}: not a number")
                        continue
                if not _rewrite_var(settings_path, name, literal):
                    errors.append(f"{name}: assignment not found in any settings/*.py file")
                if sync_live and mpc_field is not None:
                    self._sync_live_field(mpc_field, literal, errors)

            for name, (enabled_var, value_var) in self._override_vars.items():
                mpc_field = next(f for _l, n, f in _NMPC_OVERRIDE_FIELDS if n == name)
                if enabled_var.get():
                    try:
                        literal = repr(float(value_var.get()))
                    except ValueError:
                        errors.append(f"{name}: not a number")
                        continue
                else:
                    literal = "-1.0"
                if not _rewrite_var(settings_path, name, literal):
                    errors.append(f"{name}: assignment not found in any settings/*.py file")
                if sync_live:
                    self._sync_live_field(mpc_field, literal, errors)

            for mpc_field, var in self._feature_vars.items():
                settings_name = next(
                    (s for _grp, entries in _FEATURE_GROUPS for _l, s, f in entries
                     if f == mpc_field), None)
                literal = "True" if var.get() else "False"
                if settings_name is not None:
                    if not _rewrite_var(settings_path, settings_name, literal):
                        errors.append(f"{settings_name}: assignment not found in any settings/*.py file")
                if sync_live:
                    self._sync_live_field(mpc_field, literal, errors)

            # Progress-term scalars. q_progress is a weight (mpc_params.py);
            # the other three are structural (nmpc_params.py). Each is tried
            # against both files for the same reason as the flags above.
            for _label, name, _desc in _NMPC_PROGRESS_FIELDS:
                var = self._progress_vars[name]
                try:
                    literal = repr(float(var.get()))
                except ValueError:
                    errors.append(f"{name}: not a number")
                    continue
                if not _rewrite_var(settings_path, name, literal):
                    errors.append(f"{name}: assignment not found in any settings/*.py file")
                if sync_live:
                    self._sync_live_field(name.lower(), literal, errors)

            if errors:
                messagebox.showerror("Settings tab", "\n".join(errors))
                self._set_status("Save had errors, see dialog.", style="Warning.TLabel")
                # Deliberately NOT cleared: an error means at least one field
                # did not actually reach every file it needed to, so the
                # in-memory widgets and what's on disk can still disagree.
            else:
                if sync_live:
                    self._set_status(
                        "Saved to the settings package, the live mpc_params.py/nmpc_params.py, "
                        "fsae_params.yaml, their fsds_simulator/ mirrors, and any active "
                        "launch_all.sh shortlist line. Restart the sim to pick up the live change.",
                        style="Success.TLabel", flash=True, auto_clear=True)
                else:
                    self._set_status(
                        "Saved. Takes effect next time the settings package is imported.",
                        style="Success.TLabel", flash=True, auto_clear=True)
                self._is_dirty = False
                self._snapshot_saved()
        except OSError as exc:
            messagebox.showerror("Settings tab", f"Failed to save: {exc!r}")

    def _wire_dirty_tracking(self) -> None:
        """Attaches a write-trace to every Variable this tab owns, across
        all five widget dicts, so _is_dirty flips true the instant any
        field changes -- typing in an Entry counts too, not just committing
        a value, since StringVar's 'write' trace fires on every keystroke
        (a false positive here, dirty when nothing SUBSTANTIVE changed yet,
        is far cheaper than a false negative that fails to warn)."""
        all_vars: list[tk.Variable] = []
        for var_list in self._list_vars.values():
            all_vars.extend(var_list)
        all_vars.extend(self._scalar_vars.values())
        for enabled_var, value_var in self._override_vars.values():
            all_vars.append(enabled_var)
            all_vars.append(value_var)
        all_vars.extend(self._progress_vars.values())
        all_vars.extend(self._feature_vars.values())
        for var in all_vars:
            trace_id = var.trace_add("write", self._mark_dirty)
            self._dirty_trace_ids.append((var, trace_id))

    def _mark_dirty(self, *_args) -> None:
        self._is_dirty = True

    def _snapshot_saved(self) -> None:
        """Record every field's current value as its saved value and clear
        all highlights. Called once the tab is built and after each save."""
        self._saved_values = {str(var): str(var.get()) for var, _tid in self._dirty_trace_ids}
        for name in self._saved_values:
            self._update_unsaved_highlight(name)

    def _unsaved_names(self) -> list[str]:
        return [str(var) for var, _tid in self._dirty_trace_ids
                if str(var.get()) != self._saved_values.get(str(var))]

    def _update_unsaved_highlight(self, var_name: str) -> None:
        widget = self._widget_for_var.get(var_name)
        if widget is None:
            return
        var = next(v for v, _t in self._dirty_trace_ids if str(v) == var_name)
        unsaved = str(var.get()) != self._saved_values.get(var_name)
        if isinstance(widget, ttk.Entry):
            widget.configure(style="Unsaved.TEntry" if unsaved else "TEntry")
        else:
            widget.configure(style="Unsaved.Card.TCheckbutton" if unsaved
                             else "Card.TCheckbutton")

    def _set_status(self, text: str, style: str = "Muted.TLabel",
                     flash: bool = False, auto_clear: bool = False) -> None:
        """Central status-line writer for this tab. `flash=True` briefly
        shows `style` (Success.TLabel on a successful save) before settling
        back to Muted.TLabel, so a save that lands while the user is looking
        elsewhere on the card still catches the eye, not just changes some
        text that was already there. `auto_clear=True` blanks the line a
        few seconds later so a stale "Saved" message doesn't linger and get
        mistaken for describing the CURRENT state after further edits."""
        if self._status_clear_job is not None:
            self.after_cancel(self._status_clear_job)
            self._status_clear_job = None
        self.status_var.set(text)
        self.status_label.configure(style=style)
        if flash:
            self._status_clear_job = self.after(1400, lambda: self._revert_status_style(text))
        elif auto_clear:
            self._status_clear_job = self.after(_STATUS_AUTO_CLEAR_MS, self._clear_status)

    def _revert_status_style(self, text_when_scheduled: str) -> None:
        self._status_clear_job = None
        # Only revert the FLASH color, not the text -- a newer _set_status()
        # call already replaced both if one happened in the meantime, and
        # this job's own auto_clear (if any) is scheduled separately below.
        if self.status_var.get() == text_when_scheduled:
            self.status_label.configure(style="Muted.TLabel")
            self._status_clear_job = self.after(_STATUS_AUTO_CLEAR_MS, self._clear_status)

    def _clear_status(self) -> None:
        self._status_clear_job = None
        self.status_var.set("")
        self.status_label.configure(style="Muted.TLabel")

    def has_unsaved_changes(self) -> bool:
        # Value-based, so an edit typed back to the saved value is not
        # reported as unsaved.
        return self._is_dirty and bool(self._unsaved_names())

    def _on_overwrite_all_params(self) -> None:
        """Runs tuner.tools.sync_mpc_params --apply: overwrites
        mpc_params.py/nmpc_params.py/fsae_params.yaml in fsae_autonomous
        and the fsds_simulator mirror with the LIVE checkout's current
        copies. See that script's own module docstring for the full
        rationale; this handler only wires it into the GUI and adds the
        confirmation step. A plain subprocess call (matching this file's
        own "every button shells out, no tuner-package imports" design),
        not a library call into sync_mpc_params's own functions.

        Two-step: dry run first (background thread, since it's touching 6
        files, however briefly), then a CONFIRM dialog stating exactly
        what the apply run will overwrite, built from the dry run's own
        output so the destinations named are the ones that will actually
        be touched (e.g. if fsae_autonomous is not found at any known
        path, that's reflected here too, not just discovered after the
        fact)."""
        self.overwrite_all_button.configure(state="disabled")
        self._set_status("Checking what would change...", style="Muted.TLabel")
        self._run_sync_script(apply=False, on_done=self._on_overwrite_dry_run_done)

    def _run_sync_script(self, apply: bool, on_done) -> None:
        result_queue: "queue.Queue" = queue.Queue()

        def run() -> None:
            cmd = [sys.executable, "-m", "tuner.tools.sync_mpc_params"]
            if apply:
                cmd.append("--apply")
            try:
                result = subprocess.run(cmd, cwd=str(self._paths.fsae_mpctest),
                                         capture_output=True, text=True)
                result_queue.put(result)
            except OSError as exc:
                result_queue.put(exc)

        threading.Thread(target=run, daemon=True).start()
        self._poll_sync_queue(result_queue, on_done)

    def _poll_sync_queue(self, result_queue: "queue.Queue", on_done) -> None:
        try:
            outcome = result_queue.get_nowait()
        except queue.Empty:
            self.after(100, self._poll_sync_queue, result_queue, on_done)
            return
        on_done(outcome)

    def _on_overwrite_dry_run_done(self, outcome) -> None:
        if isinstance(outcome, OSError):
            self.overwrite_all_button.configure(state="normal")
            messagebox.showerror("Overwrite All Params", f"Failed to run: {outcome!r}")
            self._set_status("Overwrite failed, see dialog.", style="Warning.TLabel")
            return
        output = (outcome.stdout or "") + (outcome.stderr or "")
        if outcome.returncode != 0:
            self.overwrite_all_button.configure(state="normal")
            messagebox.showerror(
                "Overwrite All Params",
                "sync_mpc_params could not run cleanly:\n\n" + output.strip())
            self._set_status("Overwrite failed, see dialog.", style="Warning.TLabel")
            return
        if "Already in sync" in output:
            self.overwrite_all_button.configure(state="normal")
            messagebox.showinfo(
                "Overwrite All Params",
                "fsae_autonomous and the fsds_simulator mirror already match "
                "the live checkout's current mpc_params.py/nmpc_params.py/"
                "fsae_params.yaml. Nothing to overwrite.")
            self._set_status("Already in sync, nothing to overwrite.", style="Muted.TLabel")
            return
        # Destination labels come from sync_mpc_params's own machine-
        # readable "SYNC-TARGET: <label> = changed/unchanged/missing" lines
        # (one per destination, always printed, dry run or not), not by
        # parsing the diff output's file paths -- a path-parsing regex
        # cannot reliably recover "fsae_autonomous" from an arbitrary
        # discovered checkout path, and this way the warning always names
        # exactly what THIS run found (e.g. omits fsae_autonomous entirely
        # if that checkout was not found at any known path).
        changed = re.findall(r'^SYNC-TARGET: (.+) = changed$', output, re.MULTILINE)
        dest_text = ", ".join(changed) if changed else "the destinations listed above"
        proceed = messagebox.askyesno(
            "Overwrite All Params -- confirm",
            "This will OVERWRITE mpc_params.py, nmpc_params.py, and "
            "fsae_params.yaml in:\n\n"
            f"  {dest_text}\n\n"
            "with the CURRENT files from the live checkout "
            "(ros2/src/fsae_planning/), one-way. Any local edits in those "
            "destinations to those 3 files will be LOST, except for a "
            "one-time .bak backup saved alongside each overwritten file.\n\n"
            "This does NOT touch the settings package, this Settings tab's own "
            "unsaved edits, or any file other than those 3 per destination.\n\n"
            "fsae_autonomous is never committed or pushed by this tool -- "
            "only its local working tree is overwritten. Review the change "
            "there yourself before committing.\n\n"
            "Proceed?")
        if not proceed:
            self.overwrite_all_button.configure(state="normal")
            self._set_status("Overwrite cancelled.", style="Muted.TLabel")
            return
        self._set_status("Overwriting...", style="Muted.TLabel")
        self._run_sync_script(apply=True, on_done=self._on_overwrite_apply_done)

    def _on_overwrite_apply_done(self, outcome) -> None:
        self.overwrite_all_button.configure(state="normal")
        if isinstance(outcome, OSError):
            messagebox.showerror("Overwrite All Params", f"Failed to run: {outcome!r}")
            self._set_status("Overwrite failed, see dialog.", style="Warning.TLabel")
            return
        output = (outcome.stdout or "") + (outcome.stderr or "")
        if outcome.returncode != 0:
            messagebox.showerror(
                "Overwrite All Params",
                "sync_mpc_params failed partway through:\n\n" + output.strip())
            self._set_status("Overwrite failed partway, see dialog.", style="Warning.TLabel")
            return
        self._set_status(
            "Params overwritten in fsae_autonomous and the fsds_simulator mirror.",
            style="Success.TLabel", flash=True, auto_clear=True)

    def capture_profile_values(self) -> dict[str, str]:
        """Current value of every settings NAME this tab manages, as
        {name: raw-literal-text}, read straight from each field's own
        widget (not from the settings package) so an unsaved in-progress edit is
        captured too. Used by the Profiles tab's "Save current as profile".
        Keys match _profile_field_names() exactly."""
        values: dict[str, str] = {}
        for _label, name, _length, _mpc_fields in _LIST_FIELDS:
            floats = [_to_float(v.get()) for v in self._list_vars[name]]
            values[name] = "[" + ", ".join(repr(f if f is not None else 0.0)
                                            for f in floats) + "]"
        for _label, name, _mpc_field, kind in _SCALAR_FIELDS:
            var = self._scalar_vars[name]
            values[name] = "True" if (kind == "bool" and var.get()) else (
                "False" if kind == "bool" else repr(_to_float(var.get()) or 0.0))
        for _label, name, _mpc_field in _NMPC_OVERRIDE_FIELDS:
            enabled_var, value_var = self._override_vars[name]
            values[name] = repr(_to_float(value_var.get()) or 0.0) if enabled_var.get() else "-1.0"
        for _label, name, _desc in _NMPC_PROGRESS_FIELDS:
            values[name] = repr(_to_float(self._progress_vars[name].get()) or 0.0)
        for _group_title, entries in _FEATURE_GROUPS:
            for _label, settings_name, mpc_field in entries:
                key = settings_name if settings_name is not None else mpc_field
                values[key] = "True" if self._feature_vars[mpc_field].get() else "False"
        return values

    def apply_profile_values(self, values: dict[str, str]) -> None:
        """Inverse of capture_profile_values(): pushes a {name: raw-literal}
        dict (as loaded from a profile JSON file) into every matching
        widget, then calls _on_save() unchanged so the write path (the settings package,
        live dataclasses, both YAMLs, both mirrors) is EXACTLY the one normal
        editing already uses -- no separate load-time file-writing logic to
        keep in sync with _on_save's own. A name present in the profile but
        not recognised by any field table (e.g. a profile saved by an older
        GUI version before a field was added or removed) is silently
        skipped, not an error: profiles are a convenience snapshot, not a
        strict schema."""
        for _label, name, length, _mpc_fields in _LIST_FIELDS:
            if name in values:
                floats = _parse_float_list(values[name], length)
                for var, f in zip(self._list_vars[name], floats):
                    var.set(str(f))
        for _label, name, _mpc_field, kind in _SCALAR_FIELDS:
            if name not in values:
                continue
            var = self._scalar_vars[name]
            if kind == "bool":
                var.set(values[name].strip() == "True")
            else:
                var.set(str(_to_float(values[name]) or 0.0))
        for _label, name, _mpc_field in _NMPC_OVERRIDE_FIELDS:
            if name not in values:
                continue
            enabled_var, value_var = self._override_vars[name]
            f = _to_float(values[name])
            is_override = f is not None and f >= 0.0
            enabled_var.set(is_override)
            value_var.set(values[name] if is_override else "")
        for _label, name, _desc in _NMPC_PROGRESS_FIELDS:
            if name in values:
                self._progress_vars[name].set(str(_to_float(values[name]) or 0.0))
        for _group_title, entries in _FEATURE_GROUPS:
            for _label, settings_name, mpc_field in entries:
                key = settings_name if settings_name is not None else mpc_field
                if key in values:
                    self._feature_vars[mpc_field].set(values[key].strip() == "True")
        self._on_save()



def _to_float(text: str) -> float | None:
    try:
        return float(text)
    except (TypeError, ValueError):
        return None



def _parse_float_list(literal: str, expected_length: int) -> list[float]:
    """Best-effort parse of a `[1.0, 2, 3.5]`-style literal already
    extracted from a settings/ line. Falls back to zeros of the expected
    length on anything unparseable, rather than raising into the GUI."""
    inner = literal.strip()
    if inner.startswith("[") and inner.endswith("]"):
        inner = inner[1:-1]
    parts = [p.strip() for p in inner.split(",") if p.strip()]
    values: list[float] = []
    for p in parts:
        v = _to_float(p)
        if v is None:
            return [0.0] * expected_length
        values.append(v)
    if len(values) != expected_length:
        values = (values + [0.0] * expected_length)[:expected_length]
    return values

