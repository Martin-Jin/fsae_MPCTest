"""
gui/launcher/tabs/launch.py — Tab 1: Launch Sim. Rewrites the commonly-
changed variables in ros2/launch_all.sh, then runs it.
"""

from __future__ import annotations

import queue
import shutil
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk

from gui.launcher.file_edit import (
    _backup_once, _read_shortlist_var, _read_var, _rewrite_shortlist_var, _rewrite_var,
)
from gui.launcher.paths import RepoPaths
from gui.launcher.process_utils import (
    _insert_run_label, _run_detached, _sibling_path_csv, _stop_process,
)
from gui.launcher.theme import _field_label, _make_scrollable

# How long (seconds) the Launch tab's Stop button polls fsae_logs/ for a
# freshly-written CSV before giving up silently. ControlLogger.close()
# reliably finishes in well under a second in the normal single-Ctrl+C
# case (file close + optional header rewrite), but runs from the node's
# own signal handler asynchronously with this button's click, not
# synchronously with it -- a single immediate check routinely found
# nothing. A few seconds' margin comfortably covers the normal case
# without leaving the user waiting on a genuinely stuck/crashed node.
_STOP_LOG_POLL_TIMEOUT_S = 5.0



# ---------------------------------------------------------------------------
# Tab 1: Launch Sim
# ---------------------------------------------------------------------------

class LaunchTab(ttk.Frame):
    def __init__(self, parent: ttk.Notebook, paths: RepoPaths) -> None:
        super().__init__(parent, padding=0)
        self._paths = paths
        self._backed_up: set[Path] = set()
        self._proc: subprocess.Popen | None = None
        self._launch_started_at: float | None = None
        # Prior toggle values, saved when "Record new track" is checked so
        # they can be restored when unchecked -- recording forces
        # USE_PRECOMPUTED_SPEED/PATH off and CONTROLLER to stanley (see
        # launch_all.sh's own "Set BOTH to false (with CONTROLLER=stanley
        # below) when recording a NEW track" comment), which would
        # otherwise silently clobber whatever the user had set for a normal
        # drive.
        self._pre_record_state: dict[str, object] | None = None

        body = _make_scrollable(self)
        self._body = body

        ttk.Label(body, text="Launch Sim", style="Heading.TLabel").grid(
            row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(body, text="ros2/launch_all.sh — rewritten in place, then run.",
                  style="Muted.TLabel").grid(row=1, column=0, columnspan=2, sticky="w",
                                              pady=(2, 20))

        row = 2
        self.record_var = tk.BooleanVar(value=False)
        _field_label(
            body, row, "Record new track",
            "Drives live (no precomputed path/speed) with Stanley, the recommended setup for "
            "recording a fresh cone map -- see launch_all.sh's own note on this.")
        ttk.Checkbutton(body, variable=self.record_var,
                         command=self._on_record_toggle).grid(row=row, column=1, sticky="w")

        row += 2
        self.new_track_name_var = tk.StringVar(value="")
        self.new_track_row = row
        _field_label(body, row, "New track name",
                     "Folder name the recording (cone_map.json) will be written under.")
        self.new_track_entry = ttk.Entry(body, textvariable=self.new_track_name_var, width=32)
        self.new_track_entry.grid(row=row, column=1, sticky="w")
        self._set_record_row_visible(False)

        row += 2
        _field_label(body, row, "Track", "Which recorded track's cone map/speed profile to drive.")
        tracks = sorted(p.name for p in paths.tracks_dir.iterdir() if p.is_dir()) \
            if paths.tracks_dir.is_dir() else []
        self.track_var = tk.StringVar(value=_read_var(paths.launch_all_sh, "TRACK") or "")
        ttk.Combobox(body, textvariable=self.track_var, values=tracks, width=32,
                     state="readonly" if tracks else "normal").grid(row=row, column=1, sticky="w")

        row += 2
        _field_label(body, row, "Controller",
                     "Path-tracking controller: Stanley, or MPC (linear time-varying QP, or "
                     "the nonlinear NMPC).")
        controller = _read_var(paths.launch_all_sh, "CONTROLLER") or "mpc"
        use_nmpc = (_read_var(paths.launch_all_sh, "USE_NMPC") or "false").strip().lower()
        if controller.strip() == "stanley":
            initial = "stanley"
        elif use_nmpc == "true":
            initial = "nmpc"
        else:
            initial = "ltv"
        self.controller_var = tk.StringVar(value=initial)
        controller_frame = ttk.Frame(body)
        controller_frame.grid(row=row, column=1, sticky="w")
        for text, value in (("Stanley", "stanley"), ("MPC · LTV-QP", "ltv"), ("MPC · NMPC", "nmpc")):
            ttk.Radiobutton(controller_frame, text=text, value=value,
                            variable=self.controller_var,
                            command=self._on_controller_changed).pack(side="left", padx=(0, 14))

        # NMPC progress term: OFF by default and hidden unless NMPC is
        # selected, see NMPC_PROGRESS_ENABLED's own launch_all.sh comment
        # for why -- measured 2026-09-21 to go off-track at ~10% of a lap
        # at every weight tried, so this is an experiment to opt INTO, not
        # a normal driving mode. launch_all.sh ships this shortlist entry
        # commented out, so read/write goes through the
        # _read_shortlist_var/_rewrite_shortlist_var pair (plain
        # _read_var/_rewrite_var only handle an already-uncommented line).
        # Label/desc/checkbox are captured (not built via _field_label) so
        # the whole row can be grid_remove()'d as a unit when hidden,
        # rather than leaving an orphaned label above an empty row.
        row += 2
        self.progress_row = row
        self.progress_label = ttk.Label(body, text="Progress term (NMPC only, experimental)")
        self.progress_label.grid(row=row, column=0, sticky="nw", pady=(8, 0))
        self.progress_desc = ttk.Label(
            body, style="Muted.TLabel", wraplength=420,
            text="Replaces the two-sided speed-error cost with a one-sided speed cap plus a "
                 "progress reward, letting the NMPC pick its own speed below the cap instead "
                 "of tracking a target. NOT validated: goes off-track at ~10% of a lap at "
                 "every weight tried so far. Set the linear track-boundary slack weight "
                 "(Settings tab) yourself before using this -- measured NECESSARY, not just "
                 "helpful, once this is on, but this checkbox no longer sets it for you.")
        self.progress_desc.grid(row=row + 1, column=0, sticky="nw", pady=(0, 8))
        progress_state = _read_shortlist_var(paths.launch_all_sh, "NMPC_PROGRESS_ENABLED")
        self.progress_var = tk.BooleanVar(
            value=bool(progress_state and progress_state[0] and progress_state[1] == "true"))
        self.progress_check = ttk.Checkbutton(body, variable=self.progress_var)
        self.progress_check.grid(row=row, column=1, sticky="w")
        self._set_progress_row_visible(initial == "nmpc")

        row += 2
        ttk.Separator(body).grid(row=row, column=0, columnspan=2, sticky="ew", pady=16)

        row += 1
        self.standalone_var = self._bool_row(
            row, "Standalone output (MPC only)",
            "On: the MPC node publishes steering+throttle+brake directly. Off: steering only, "
            "fsds_bridge computes throttle/brake.",
            "STANDALONE_OUTPUT")
        row += 2
        self.precomp_speed_var = self._bool_row(
            row, "Use precomputed speed profile",
            "On: follow the track's saved speed_profile.csv. Off: compute speed live from "
            "path curvature.",
            "USE_PRECOMPUTED_SPEED")
        row += 2
        self.precomp_path_var = self._bool_row(
            row, "Use precomputed path",
            "On: drive the track's saved centreline/raceline. Off: drive the live planner's "
            "output (needed when recording a new track).",
            "USE_PRECOMPUTED_PATH")

        row += 2
        ttk.Separator(body).grid(row=row, column=0, columnspan=2, sticky="ew", pady=16)

        row += 1
        _field_label(body, row, "V_MAX", "Upper speed cap (m/s) handed to the controller node.")
        self.v_max_var = tk.StringVar(value=_read_var(paths.launch_all_sh, "V_MAX") or "20.0")
        ttk.Entry(body, textvariable=self.v_max_var, width=10).grid(row=row, column=1, sticky="w")

        row += 2
        _field_label(body, row, "V_MIN", "Lower speed floor (m/s) handed to the controller node "
                     "-- the car never targets slower than this even mid-corner.")
        self.v_min_var = tk.StringVar(value=_read_var(paths.launch_all_sh, "V_MIN") or "1.5")
        ttk.Entry(body, textvariable=self.v_min_var, width=10).grid(row=row, column=1, sticky="w")

        row += 2
        self.status_var = tk.StringVar(value="")
        ttk.Label(body, textvariable=self.status_var, style="Muted.TLabel").grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(18, 6))

        row += 1
        button_row = ttk.Frame(body)
        button_row.grid(row=row, column=0, columnspan=2, sticky="w")
        self.launch_button = ttk.Button(button_row, text="Launch", style="Accent.TButton",
                                         command=self._on_launch)
        self.launch_button.pack(side="left")
        self.stop_button = ttk.Button(button_row, text="Stop", command=self._on_stop,
                                       state="disabled")
        self.stop_button.pack(side="left", padx=(10, 0))
        self.export_button = ttk.Button(button_row, text="Export & Save Track",
                                         command=self._on_export, state="disabled")
        self.export_button.pack(side="left", padx=(10, 0))
        # TEMPORARY (2026-09-20): runs run_brake_sysid.sh instead of the
        # normal stack, via launch_all.sh's own RUN_BRAKE_SYSID toggle --
        # see that variable's comment in launch_all.sh for why this can't
        # just be another checkbox alongside the normal launch options (it
        # must run INSTEAD OF sim.launch.py, not alongside it). Remove this
        # button once the brake system-ID investigation is closed out.
        self.brake_sysid_button = ttk.Button(
            button_row, text="Run Brake Sysid", command=self._on_run_brake_sysid)
        self.brake_sysid_button.pack(side="left", padx=(10, 0))

    def _set_record_row_visible(self, visible: bool) -> None:
        if visible:
            self.new_track_entry.grid()
        else:
            self.new_track_entry.grid_remove()

    def _set_progress_row_visible(self, visible: bool) -> None:
        widgets = (self.progress_label, self.progress_desc, self.progress_check)
        if visible:
            for w in widgets:
                w.grid()
        else:
            for w in widgets:
                w.grid_remove()

    def _on_controller_changed(self) -> None:
        self._set_progress_row_visible(self.controller_var.get() == "nmpc")

    def _on_record_toggle(self) -> None:
        self._set_record_row_visible(self.record_var.get())
        if self.record_var.get():
            self._pre_record_state = {
                "controller": self.controller_var.get(),
                "precomp_speed": self.precomp_speed_var.get(),
                "precomp_path": self.precomp_path_var.get(),
            }
            self.controller_var.set("stanley")
            self.precomp_speed_var.set(False)
            self.precomp_path_var.set(False)
        elif self._pre_record_state is not None:
            self.controller_var.set(self._pre_record_state["controller"])
            self.precomp_speed_var.set(self._pre_record_state["precomp_speed"])
            self.precomp_path_var.set(self._pre_record_state["precomp_path"])
            self._pre_record_state = None
        # tk.StringVar.set() does not fire a Radiobutton's own `command`,
        # so the progress-row visibility needs an explicit refresh here too.
        self._on_controller_changed()

    def refresh_from_disk(self) -> None:
        """Re-reads every field this tab seeded from launch_all.sh at
        __init__ time. Without this, the tab's widgets only ever reflect
        the file's contents at the moment the GUI window was opened --
        stale the instant launch_all.sh changes on disk afterward (a hand
        edit, another concurrent GUI/session per CLAUDE.md's "Concurrent
        session collisions" note, or this same tab's own previous Launch
        click). The next Launch click writes _pending_values() straight
        from these widgets, so a stale value here silently reverts
        whatever the more recent on-disk edit set -- this was exactly the
        "Launch tab doesn't seem to overwrite" symptom for
        NMPC_PROGRESS_ENABLED, since that field can ALSO be true/false on
        disk independent of what this tab last wrote. Skipped while
        "Record new track" is checked, since that mode intentionally
        holds a forced-override state this tab hasn't written yet."""
        if self.record_var.get():
            return
        paths = self._paths
        self.track_var.set(_read_var(paths.launch_all_sh, "TRACK") or self.track_var.get())
        controller = _read_var(paths.launch_all_sh, "CONTROLLER") or "mpc"
        use_nmpc = (_read_var(paths.launch_all_sh, "USE_NMPC") or "false").strip().lower()
        if controller.strip() == "stanley":
            self.controller_var.set("stanley")
        elif use_nmpc == "true":
            self.controller_var.set("nmpc")
        else:
            self.controller_var.set("ltv")
        progress_state = _read_shortlist_var(paths.launch_all_sh, "NMPC_PROGRESS_ENABLED")
        self.progress_var.set(bool(progress_state and progress_state[0] and progress_state[1] == "true"))
        self.standalone_var.set(
            (_read_var(paths.launch_all_sh, "STANDALONE_OUTPUT") or "true").strip().lower() == "true")
        self.precomp_speed_var.set(
            (_read_var(paths.launch_all_sh, "USE_PRECOMPUTED_SPEED") or "true").strip().lower() == "true")
        self.precomp_path_var.set(
            (_read_var(paths.launch_all_sh, "USE_PRECOMPUTED_PATH") or "true").strip().lower() == "true")
        self.v_max_var.set(_read_var(paths.launch_all_sh, "V_MAX") or self.v_max_var.get())
        self.v_min_var.set(_read_var(paths.launch_all_sh, "V_MIN") or self.v_min_var.get())
        self._on_controller_changed()

    def capture_profile_values(self) -> dict[str, str]:
        """This tab's own fields, in the same {launch_all.sh NAME: raw
        value} shape _pending_values() already builds for a real Launch
        click, plus NMPC_PROGRESS_ENABLED (not in _pending_values() since
        it needs the comment-toggling shortlist write, not a plain one --
        see _on_launch's own comment on this). A saved profile previously
        captured ONLY the Settings tab's fields (Q/R weights, NMPC
        overrides, feature flags): loading it left the Launch tab's track/
        controller/precomputed-path/etc. choice untouched, so 'load a
        profile' silently did not reproduce the run it was saved from
        unless those also happened to already match. Record new track
        mode is deliberately NOT captured, a profile is a driving
        configuration, not a recording-in-progress snapshot."""
        values = self._pending_values()
        values["NMPC_PROGRESS_ENABLED"] = (
            "true" if (self.controller_var.get() == "nmpc" and self.progress_var.get()) else "false")
        return values

    def apply_profile_values(self, values: dict[str, str]) -> None:
        """Inverse of capture_profile_values(): pushes a saved profile's
        launch_all.sh-shaped values into this tab's own widgets. Mirrors
        SettingsTab.apply_profile_values()'s own silently-skip-unknown-keys
        behaviour, a profile is a convenience snapshot, not a strict
        schema. Does NOT write launch_all.sh itself or start anything --
        same "widgets now, disk on next explicit action" split
        SettingsTab.apply_profile_values() uses (there: _on_save(); here:
        the user's own next Launch click, or Save on the Settings tab if
        that ran in the same load)."""
        if "TRACK" in values and not self.record_var.get():
            self.track_var.set(values["TRACK"])
        if "CONTROLLER" in values and "USE_NMPC" in values:
            if values["CONTROLLER"].strip() == "stanley":
                self.controller_var.set("stanley")
            elif values["USE_NMPC"].strip().lower() == "true":
                self.controller_var.set("nmpc")
            else:
                self.controller_var.set("ltv")
            self._on_controller_changed()
        if "NMPC_PROGRESS_ENABLED" in values:
            self.progress_var.set(values["NMPC_PROGRESS_ENABLED"].strip().lower() == "true")
        if "STANDALONE_OUTPUT" in values:
            self.standalone_var.set(values["STANDALONE_OUTPUT"].strip().lower() == "true")
        if "USE_PRECOMPUTED_SPEED" in values:
            self.precomp_speed_var.set(values["USE_PRECOMPUTED_SPEED"].strip().lower() == "true")
        if "USE_PRECOMPUTED_PATH" in values:
            self.precomp_path_var.set(values["USE_PRECOMPUTED_PATH"].strip().lower() == "true")
        if "V_MAX" in values:
            self.v_max_var.set(values["V_MAX"])
        if "V_MIN" in values:
            self.v_min_var.set(values["V_MIN"])

    def _bool_row(self, row: int, label: str, desc: str, var_name: str) -> tk.BooleanVar:
        _field_label(self._body, row, label, desc)
        current = (_read_var(self._paths.launch_all_sh, var_name) or "true").strip().lower()
        var = tk.BooleanVar(value=(current == "true"))
        ttk.Checkbutton(self._body, variable=var).grid(row=row, column=1, sticky="w")
        return var

    def _pending_values(self) -> dict[str, str]:
        controller = self.controller_var.get()
        track = self.new_track_name_var.get().strip() if self.record_var.get() \
            else self.track_var.get()
        values = {
            "TRACK": track,
            "CONTROLLER": "stanley" if controller == "stanley" else "mpc",
            "USE_NMPC": "true" if controller == "nmpc" else "false",
            "STANDALONE_OUTPUT": "true" if self.standalone_var.get() else "false",
            "USE_PRECOMPUTED_SPEED": "true" if self.precomp_speed_var.get() else "false",
            "USE_PRECOMPUTED_PATH": "true" if self.precomp_path_var.get() else "false",
            "V_MAX": self.v_max_var.get().strip(),
            "V_MIN": self.v_min_var.get().strip(),
        }
        return values

    def _on_launch(self) -> None:
        if self.record_var.get() and not self.new_track_name_var.get().strip():
            messagebox.showerror("Launch tab", "Enter a name for the new track first.")
            return
        values = self._pending_values()
        preview_lines = [f"  {k}={v}" for k, v in values.items()]
        if self.controller_var.get() == "nmpc" and self.progress_var.get():
            preview_lines.append("  NMPC_PROGRESS_ENABLED=true (EXPERIMENTAL, not validated)")
            preview_lines.append(
                "  NMPC_SLACK_LINEAR_WEIGHT left as-is -- set it from the Settings tab; "
                "measured NECESSARY (not just helpful) with the progress term on, or the "
                "car cuts corners and goes off-track")
        preview = "\n".join(preview_lines)
        if not messagebox.askyesno(
                "Confirm launch",
                f"About to rewrite ros2/launch_all.sh with:\n\n{preview}\n\n"
                "and start it. Continue?"):
            return
        try:
            _backup_once(self._paths.launch_all_sh, self._backed_up)
            missing = [name for name, value in values.items()
                       if not _rewrite_var(self._paths.launch_all_sh, name, value)]
            # NMPC_PROGRESS_ENABLED ships as a commented-out shortlist entry
            # (off by default, see the checkbox's own on-screen warning), so
            # it needs the comment-toggling rewrite, not the plain one above
            # which only handles an already-uncommented line.
            #
            # Does NOT touch NMPC_SLACK_LINEAR_WEIGHT (previously force-set
            # to 1000.0 here) -- the user asked to tune that one manually
            # from the Settings tab instead of having this checkbox override
            # it. Still measured NECESSARY (not just helpful) with the
            # progress term on, or the car cuts corners and goes off-track;
            # that warning now lives only in the confirmation dialog, not
            # enforced here.
            progress_on = self.controller_var.get() == "nmpc" and self.progress_var.get()
            shortlist_ok = _rewrite_shortlist_var(
                self._paths.launch_all_sh, "NMPC_PROGRESS_ENABLED",
                progress_on, "true" if progress_on else "false")
            if not shortlist_ok:
                missing.append("NMPC_PROGRESS_ENABLED")
            if missing:
                messagebox.showerror(
                    "Launch tab",
                    f"Could not find these variables in launch_all.sh: {', '.join(missing)}\n"
                    "The script may have changed shape; edit it directly for now.")
                return
            self._proc = _run_detached(["bash", "launch_all.sh"],
                                        cwd=self._paths.fsds_root / "ros2")
            self._launch_started_at = time.time()
            self._launch_controller_label = {
                "stanley": "Stanley", "ltv": "LMPC", "nmpc": "NMPC",
            }[self.controller_var.get()]
            self.status_var.set("Launched. Check the terminal/log window launch_all.sh opened.")
            self.launch_button.configure(state="disabled")
            self.stop_button.configure(state="normal")
            self.export_button.configure(
                state="normal" if self.record_var.get() else "disabled")
        except OSError as exc:
            messagebox.showerror("Launch tab", f"Failed to launch: {exc!r}")

    def _on_run_brake_sysid(self) -> None:
        if not messagebox.askyesno(
                "Confirm brake system-ID",
                "This drives FSDS directly with fixed throttle/brake "
                "(accelerate, coast, hard-brake, repeat) to measure real "
                "achieved deceleration. It bypasses the controller entirely "
                "and does NOT steer or avoid cones -- run it in an open area.\n\n"
                "Continue?"):
            return
        try:
            _backup_once(self._paths.launch_all_sh, self._backed_up)
            # Flip the flag on just long enough to launch, then immediately
            # write it back to false -- launch_all.sh has already read its
            # own source by the time the child process starts, so the file
            # at rest never claims sysid mode is the current default. Mirrors
            # how _on_launch never leaves a one-off override sitting live.
            if not _rewrite_var(self._paths.launch_all_sh, "RUN_BRAKE_SYSID", "true"):
                messagebox.showerror(
                    "Launch tab",
                    "Could not find RUN_BRAKE_SYSID in launch_all.sh -- "
                    "the script may have changed shape; edit it directly for now.")
                return
            self._proc = _run_detached(["bash", "launch_all.sh"],
                                        cwd=self._paths.fsds_root / "ros2")
            _rewrite_var(self._paths.launch_all_sh, "RUN_BRAKE_SYSID", "false")
            self._launch_started_at = time.time()
            self._launch_controller_label = "Brake Sysid"
            self.status_var.set(
                "Brake sysid running. Check the terminal/log window it opened; "
                "the log lands in fsae_logs/brake_sysid_*.csv.")
            self.launch_button.configure(state="disabled")
            self.stop_button.configure(state="normal")
        except OSError as exc:
            messagebox.showerror("Launch tab", f"Failed to launch: {exc!r}")

    def stop_running_sim(self) -> bool:
        """Signals a running launch and resets the buttons, without any of
        _on_stop's log-offer follow-up. Separate from _on_stop so window
        teardown can reuse it: the launched process group survives this
        GUI (start_new_session=True), so closing the window without this
        would strand a running sim with no Stop button left to press.
        Returns True if a live process was actually signalled."""
        was_running = self._proc is not None and self._proc.poll() is None
        if self._proc is not None:
            _stop_process(self._proc)
        self.launch_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        return was_running

    def _on_stop(self) -> None:
        self.stop_running_sim()
        self.status_var.set("Stop signal sent (same as Ctrl+C). "
                             "The sim/bridge windows will close themselves.")
        # The node's telemetry file isn't necessarily flushed/closed the
        # instant the signal is sent -- ControlLogger.close() runs from the
        # node's own SIGINT handler, not synchronously with this click, so
        # checking fsae_logs/ exactly once here routinely found nothing.
        # Poll for up to _STOP_LOG_POLL_TIMEOUT_S instead of a single
        # synchronous check.
        self._poll_for_stopped_run(deadline=time.time() + _STOP_LOG_POLL_TIMEOUT_S)

    def _poll_for_stopped_run(self, deadline: float) -> None:
        control_csv = self._find_new_control_csv()
        if control_csv is not None:
            self._offer_move_log_to_recorded_runs(control_csv)
            return
        if time.time() >= deadline:
            return  # gave it a few seconds; no new log appeared, say nothing
        self.after(300, self._poll_for_stopped_run, deadline)

    def _find_new_control_csv(self) -> Path | None:
        """Newest `*_control_*.csv` in fsae_logs/ written since this launch
        started, or None if none yet -- only considers logs from THIS
        launch (by mtime), so an unrelated older file sitting in
        fsae_logs/ is never swept up by mistake."""
        if self._launch_started_at is None:
            return None
        logs_dir = self._paths.fsae_logs_dir
        if not logs_dir.is_dir():
            return None
        # 1 s tolerance: filesystem mtime resolution can be coarser than
        # time.time()'s float precision, so a file written a few hundred ms
        # after this launch started can still report an mtime a hair
        # *before* self._launch_started_at on some filesystems/platforms.
        cutoff = self._launch_started_at - 1.0
        new_controls = [
            p for p in logs_dir.glob("*_control_*.csv")
            if p.stat().st_mtime >= cutoff
        ]
        if not new_controls:
            return None
        return max(new_controls, key=lambda p: p.stat().st_mtime)

    def _offer_move_log_to_recorded_runs(self, control_csv: Path) -> None:
        """After Stop finds a freshly-written CSV, offers to move it (and
        its sibling path CSV) into fsds_simulator/recorded_runs/<Controller>/,
        the same manual step debugging_tools.md's "Telemetry playback"
        section already documents doing by hand, and offers to rename it
        first."""
        path_csv = _sibling_path_csv(control_csv)
        label = getattr(self, "_launch_controller_label", "Stanley")
        if not messagebox.askyesno(
                "Save recorded run?",
                f"Move this run's log into fsds_simulator/recorded_runs/{label}/?\n\n"
                f"  {control_csv.name}"
                + (f"\n  {path_csv.name}" if path_csv else "")):
            return

        custom_label = simpledialog.askstring(
            "Name this run",
            "Optional label for this run (leave blank to keep the default name).\n"
            "Inserted between the tag and timestamp, e.g. "
            "mpc_standalone_<label>_control_<stamp>.csv — matches this project's "
            "own convention for a hand-labelled run (see debugging_tools.md).",
            parent=self,
        )
        dest_control_name = _insert_run_label(control_csv.name, custom_label)
        dest_path_name = _insert_run_label(path_csv.name, custom_label) if path_csv else None

        dest_dir = self._paths.recorded_runs_dir / label
        dest_dir.mkdir(parents=True, exist_ok=True)
        try:
            shutil.move(str(control_csv), str(dest_dir / dest_control_name))
            if path_csv is not None:
                shutil.move(str(path_csv), str(dest_dir / dest_path_name))
            self.status_var.set(
                f"Run saved as fsds_simulator/recorded_runs/{label}/{dest_control_name}.")
        except OSError as exc:
            messagebox.showerror("Launch tab", f"Failed to move log: {exc!r}")

    def _on_export(self) -> None:
        name = self.new_track_name_var.get().strip()
        if not name:
            messagebox.showerror("Launch tab", "No new track name set.")
            return
        dest = self._paths.tracks_dir / name
        if dest.is_dir() and any(dest.iterdir()):
            if not messagebox.askyesno(
                    "Overwrite existing track?",
                    f"'{name}' already has files under ros2/src/fsae_planning/tracks/. "
                    "Exporting will overwrite its speed_profile.csv/raceline.csv/"
                    "centerline.csv. Continue?"):
                return
        self.export_button.configure(state="disabled")
        self.status_var.set(f"Exporting '{name}'…")

        # tkinter's .after() is not safe to call from a worker thread even
        # under a running mainloop (observed hanging in practice) -- the
        # worker only ever pushes onto this thread-safe queue; a poller
        # registered from the MAIN thread (_poll_export_queue, scheduled
        # below) is what actually calls back into tkinter.
        result_queue: queue.Queue = queue.Queue()

        def run_export() -> None:
            try:
                for cmd in (
                    [sys.executable, "-m", "tuner.tools.export_speed_profile", name],
                    [sys.executable, "-m", "tuner.tools.raceline_optimizer", name],
                    [sys.executable, "-m", "tuner.tools.raceline_optimizer", name,
                     "--mode", "centerline"],
                ):
                    result = subprocess.run(cmd, cwd=str(self._paths.fsae_mpctest),
                                             capture_output=True, text=True)
                    if result.returncode != 0:
                        result_queue.put(("failed", cmd, result))
                        return
                result_queue.put(("done", name))
            except OSError as exc:
                result_queue.put(("error", exc))

        threading.Thread(target=run_export, daemon=True).start()
        self._poll_export_queue(result_queue)

    def _poll_export_queue(self, result_queue: "queue.Queue") -> None:
        try:
            outcome = result_queue.get_nowait()
        except queue.Empty:
            self.after(100, self._poll_export_queue, result_queue)
            return
        kind = outcome[0]
        if kind == "failed":
            self._on_export_failed(outcome[1], outcome[2])
        elif kind == "done":
            self._on_export_done(outcome[1])
        else:
            messagebox.showerror("Export failed", f"Failed to run exporter: {outcome[1]!r}")
            self.export_button.configure(state="normal")

    def _on_export_failed(self, cmd: list[str], result: subprocess.CompletedProcess) -> None:
        self.export_button.configure(state="normal")
        self.status_var.set("Export failed, see dialog.")
        messagebox.showerror(
            "Export failed",
            f"{' '.join(cmd)}\n\nexit code {result.returncode}\n\n{result.stderr[-2000:]}")

    def _on_export_done(self, name: str) -> None:
        self.export_button.configure(state="normal")
        src = self._paths.tracks_dir / name
        mirror_root = self._paths.fsae_mpctest / "fsds_simulator" / "tracks"
        try:
            mirror_root.mkdir(parents=True, exist_ok=True)
            shutil.copytree(src, mirror_root / name, dirs_exist_ok=True)
            self.status_var.set(
                f"Exported '{name}' to ros2/src/fsae_planning/tracks/ and mirrored to "
                "fsds_simulator/tracks/.")
        except OSError as exc:
            messagebox.showerror(
                "Export tab",
                f"Exported to ros2/src/fsae_planning/tracks/{name}/ OK, but failed to "
                f"mirror into fsds_simulator/tracks/: {exc!r}")

