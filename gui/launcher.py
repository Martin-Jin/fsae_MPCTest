"""
gui/launcher.py — Centralized launcher/debug GUI

PURPOSE
-------
One tkinter app, tabbed by tool, wrapping the separate entry points this
project otherwise requires editing a script/CLI for by hand:

  1. Launch Sim     — rewrites the commonly-changed variables in
                       ros2/launch_all.sh, then runs it.
  2. Debug a Log     — a file browser over fsae_logs/ and
                       fsds_simulator/recorded_runs/, then runs
                       tuner.tools.plot_playback on the selection.
  3. Run Offline Sim — launches gui/simulation.py (the 2D matplotlib tool).
  4. Settings         — edits the handful of commonly-retuned settings.py
                       constants (Q/R weights, NMPC overrides, adaptive
                       feature flags) in place.

This file contains NO simulation/plotting/tuning logic of its own: every
button shells out to an existing, already-working tool via subprocess.
Rewriting ros2/launch_all.sh / settings.py in place is the only state this
file mutates; both are plain `NAME = value` / `NAME=value` line-oriented
files with no multi-line assignments among the fields this GUI touches,
so a single regex substitution per field is enough (_rewrite_var below).

USED BY
-------
  Standalone: run with `python -m gui.launcher` from fsae_MPCTest/.

ASSUMED LAYOUT
--------------
Per this project's documented repo layout: this file's own location
resolves fsae_MPCTest/'s root two levels up (gui/launcher.py -> gui/ ->
fsae_MPCTest/), and the outer FSDS sim repo root is fsae_MPCTest/'s own
parent, with ros2/launch_all.sh and ros2/src/fsae_planning/tracks/ under
that same root (`_repo_paths()` below).
"""

from __future__ import annotations

import os
import queue
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import font as tkfont
from tkinter import messagebox, simpledialog, ttk


# ---------------------------------------------------------------------------
# Theme: a flat, muted palette on top of ttk's stdlib "clam" theme (the only
# built-in theme that's actually restylable -- "default"/"alt"/"classic" are
# the dated Motif-style look this is deliberately moving away from). No new
# dependency: clam ships with tkinter itself.
# ---------------------------------------------------------------------------

class Palette:
    bg = "#1e1f22"          # app background
    surface = "#26272b"     # panels/cards
    surface_alt = "#2d2f34"  # inputs, list rows
    border = "#3a3c42"
    text = "#e7e8ea"
    text_muted = "#9a9da5"
    accent = "#5b8def"      # primary action
    accent_hover = "#6f9bf2"
    accent_text = "#0d1117"
    danger = "#e5626b"
    font_family = "Segoe UI"
    font_family_fallback = "Helvetica"


def _resolve_font_family() -> str:
    available = set(tkfont.families())
    for candidate in (Palette.font_family, Palette.font_family_fallback,
                      "DejaVu Sans", "Arial"):
        if candidate in available:
            return candidate
    return "TkDefaultFont"


def apply_theme(root: tk.Tk) -> None:
    """Configures ttk's 'clam' theme with a flat, dark, minimal palette and
    consistent spacing. Called once, on the root window, before any widgets
    are built -- ttk styles are process-global, not per-widget."""
    family = _resolve_font_family()
    base_font = (family, 10)
    heading_font = (family, 11, "bold")
    small_font = (family, 9)

    root.option_add("*Font", base_font)
    root.configure(bg=Palette.bg)

    style = ttk.Style(root)
    style.theme_use("clam")

    style.configure(".", background=Palette.bg, foreground=Palette.text,
                     font=base_font, borderwidth=0)
    style.configure("TFrame", background=Palette.bg)
    style.configure("Card.TFrame", background=Palette.surface)
    style.configure("TLabel", background=Palette.bg, foreground=Palette.text)
    style.configure("Card.TLabel", background=Palette.surface, foreground=Palette.text)
    style.configure("Muted.TLabel", background=Palette.bg, foreground=Palette.text_muted,
                     font=small_font)
    style.configure("CardMuted.TLabel", background=Palette.surface, foreground=Palette.text_muted,
                     font=small_font)
    style.configure("Heading.TLabel", background=Palette.bg, foreground=Palette.text,
                     font=heading_font)
    style.configure("SectionHeading.TLabel", background=Palette.bg,
                     foreground=Palette.text_muted, font=heading_font)

    style.configure("TNotebook", background=Palette.bg, borderwidth=0, tabmargins=(0, 6, 0, 0))
    style.configure("TNotebook.Tab", background=Palette.surface, foreground=Palette.text_muted,
                     padding=(16, 8), borderwidth=0, font=base_font)
    style.map("TNotebook.Tab",
              background=[("selected", Palette.bg)],
              foreground=[("selected", Palette.text)])

    style.configure("TButton", background=Palette.surface_alt, foreground=Palette.text,
                     padding=(12, 7), borderwidth=0, focusthickness=0, relief="flat",
                     font=base_font)
    style.map("TButton",
              background=[("active", Palette.border), ("pressed", Palette.border)])

    style.configure("Accent.TButton", background=Palette.accent, foreground=Palette.accent_text,
                     padding=(14, 8), borderwidth=0, relief="flat", font=(family, 10, "bold"))
    style.map("Accent.TButton",
              background=[("active", Palette.accent_hover), ("pressed", Palette.accent_hover)])

    style.configure("TEntry", fieldbackground=Palette.surface_alt, foreground=Palette.text,
                     insertcolor=Palette.text, bordercolor=Palette.border,
                     lightcolor=Palette.surface_alt, darkcolor=Palette.surface_alt,
                     borderwidth=1, padding=6)
    style.map("TEntry", bordercolor=[("focus", Palette.accent)])

    style.configure("TCombobox", fieldbackground=Palette.surface_alt, foreground=Palette.text,
                     background=Palette.surface_alt, bordercolor=Palette.border,
                     arrowcolor=Palette.text_muted, borderwidth=1, padding=6)
    style.map("TCombobox",
              fieldbackground=[("readonly", Palette.surface_alt)],
              foreground=[("readonly", Palette.text)])
    root.option_add("*TCombobox*Listbox.background", Palette.surface_alt)
    root.option_add("*TCombobox*Listbox.foreground", Palette.text)
    root.option_add("*TCombobox*Listbox.selectBackground", Palette.accent)

    style.configure("TCheckbutton", background=Palette.bg, foreground=Palette.text,
                     font=base_font, focuscolor=Palette.bg)
    style.map("TCheckbutton", background=[("active", Palette.bg)])
    style.configure("Card.TCheckbutton", background=Palette.surface, foreground=Palette.text,
                     font=base_font, focuscolor=Palette.surface)
    style.map("Card.TCheckbutton", background=[("active", Palette.surface)])

    style.configure("TRadiobutton", background=Palette.bg, foreground=Palette.text,
                     font=base_font, focuscolor=Palette.bg)
    style.map("TRadiobutton", background=[("active", Palette.bg)])

    style.configure("Vertical.TScrollbar", background=Palette.surface_alt,
                     troughcolor=Palette.bg, bordercolor=Palette.bg,
                     arrowcolor=Palette.text_muted, borderwidth=0)
    style.map("Vertical.TScrollbar", background=[("active", Palette.border)])

    style.configure("TSeparator", background=Palette.border)


def _field_label(parent: tk.Widget, row: int, label: str, desc: str = "",
                  label_style: str = "TLabel", desc_style: str = "Muted.TLabel",
                  columnspan: int = 1, wraplength: int = 420) -> None:
    """Places LABEL in column 0 of `row`, and, if given, a small muted DESC
    line spanning the same columns in the row immediately below it -- so
    every field across the app carries a short description of what it does
    and what unit it's in, not just a bare name. The control itself
    (entry/combobox/checkbutton) is added separately by the caller at
    (row, column=1+); DESC's row is left otherwise empty so it never
    collides with the control."""
    ttk.Label(parent, text=label, style=label_style).grid(
        row=row, column=0, sticky="nw", pady=(8, 0 if desc else 8))
    if desc:
        ttk.Label(parent, text=desc, style=desc_style, wraplength=wraplength).grid(
            row=row + 1, column=0, columnspan=columnspan, sticky="nw", pady=(0, 8))


def _make_scrollable(parent: tk.Widget, body_padding=(24, 24, 24, 24)) -> ttk.Frame:
    """Wraps `parent` in a vertically-scrollable canvas and returns the
    inner content frame to build a tab's widgets into. Every tab uses this
    (not just Settings, which had it originally) since a tab's content can
    exceed the window's fixed height depending on which optional rows are
    showing (e.g. the Launch tab's "Record new track" fields, or a export/
    stop button row) -- without it, the fixed-size window just clips the
    bottom of the tab with no way to reach it, which is exactly what
    happened before this existed. Mouse-wheel scrolling is bound only while
    the pointer is over this canvas, so it doesn't hijack the notebook's
    own scroll-if-any or a sibling tab's canvas."""
    canvas = tk.Canvas(parent, borderwidth=0, highlightthickness=0, background=Palette.bg)
    scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
    body = ttk.Frame(canvas, padding=body_padding)
    body_window = canvas.create_window((0, 0), window=body, anchor="nw")
    body.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
    canvas.bind("<Configure>", lambda e: canvas.itemconfigure(body_window, width=e.width))
    canvas.configure(yscrollcommand=scrollbar.set)
    canvas.pack(side="left", fill="both", expand=True)
    scrollbar.pack(side="right", fill="y")

    def _on_wheel(event):
        # Linux (X11) delivers Button-4/5 with no `.delta`; Windows/macOS
        # deliver <MouseWheel> with a signed `.delta` (multiples of 120 on
        # Windows). Handle both rather than assuming one platform.
        if getattr(event, "num", None) == 4:
            canvas.yview_scroll(-1, "units")
        elif getattr(event, "num", None) == 5:
            canvas.yview_scroll(1, "units")
        else:
            canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

    def _bind_wheel(_e=None):
        canvas.bind_all("<MouseWheel>", _on_wheel)
        canvas.bind_all("<Button-4>", _on_wheel)
        canvas.bind_all("<Button-5>", _on_wheel)

    def _unbind_wheel(_e=None):
        canvas.unbind_all("<MouseWheel>")
        canvas.unbind_all("<Button-4>")
        canvas.unbind_all("<Button-5>")

    canvas.bind("<Enter>", _bind_wheel)
    canvas.bind("<Leave>", _unbind_wheel)
    return body


@dataclass(frozen=True)
class RepoPaths:
    fsae_mpctest: Path
    fsds_root: Path
    launch_all_sh: Path
    tracks_dir: Path
    recorded_runs_dir: Path
    fsae_logs_dir: Path
    settings_py: Path
    mpc_params_py: Path


def _repo_paths() -> RepoPaths:
    fsae_mpctest = Path(__file__).resolve().parent.parent
    fsds_root = fsae_mpctest.parent
    return RepoPaths(
        fsae_mpctest=fsae_mpctest,
        fsds_root=fsds_root,
        launch_all_sh=fsds_root / "ros2" / "launch_all.sh",
        tracks_dir=fsds_root / "ros2" / "src" / "fsae_planning" / "tracks",
        recorded_runs_dir=fsae_mpctest / "fsds_simulator" / "recorded_runs",
        # Matches launch_all.sh's own `log_dir:='$HOST_REPO_ROOT/fsae_logs'`
        # exactly (HOST_REPO_ROOT is that script's own outer-FSDS-repo-root
        # variable) -- NOT the user's home directory, which only coincides
        # with the repo root by accident. ControlLogger (telemetry_logger.py)
        # opens its CSV here at node startup, not lazily at shutdown, so
        # getting this directory right is what makes the Stop button's
        # "save this run?" prompt able to find anything at all.
        fsae_logs_dir=fsds_root / "fsae_logs",
        settings_py=fsae_mpctest / "settings.py",
        mpc_params_py=(fsds_root / "ros2" / "src" / "fsae_planning" / "control"
                       / "fsae_control" / "fsae_control" / "mpc" / "mpc_params.py"),
    )


# ---------------------------------------------------------------------------
# Shared file-rewrite helpers (Launch tab -> launch_all.sh, Settings tab ->
# settings.py). Both files use the same flat "NAME = value" / "NAME=value"
# style with no multi-line assignments for any field this GUI touches.
# ---------------------------------------------------------------------------

def _var_pattern(name: str) -> re.Pattern:
    # \b after the name prevents NMPC_Q_E_Y's pattern from matching
    # NMPC_Q_E_YD's line (both start with the same prefix) -- \b sits
    # between the last name character and whitespace/'=', but NOT between
    # 'Y' and 'D' (both word characters), so the YD line never matches.
    return re.compile(rf"^(\s*{re.escape(name)}\b\s*=\s*)([^#\n]*?)(\s*(?:#.*)?)$",
                       re.MULTILINE)


def _read_var(path: Path, name: str) -> str | None:
    """Current raw right-hand-side text of NAME's assignment, or None if
    NAME's assignment line isn't found in path."""
    text = path.read_text()
    m = _var_pattern(name).search(text)
    return m.group(2).strip() if m else None


def _rewrite_var(path: Path, name: str, new_value: str) -> bool:
    """Rewrite NAME's assignment to new_value, preserving the line's
    leading whitespace/alignment and any trailing inline comment. Returns
    False (no-op, file untouched) if NAME's assignment line isn't found."""
    text = path.read_text()
    pattern = _var_pattern(name)
    if not pattern.search(text):
        return False
    new_text = pattern.sub(lambda m: f"{m.group(1)}{new_value}{m.group(3)}", text, count=1)
    path.write_text(new_text)
    return True


def _dataclass_field_pattern(name: str) -> re.Pattern:
    """Matches a mpc_params.py-style `name: <type> = field(default=VALUE, ...)`
    field, capturing only the default's value up to its next comma --
    NOT the whole line, since metadata dicts routinely span multiple lines
    (e.g. nmpc_q_epsi_dot's) and must be left completely untouched. Anchors
    on `name\\s*:` (a field name is always immediately followed by its type
    annotation's colon), so q_e_y's pattern cannot match q_e_yd's line the
    way a bare \\b boundary alone might for a same-prefix field name pair."""
    return re.compile(
        rf"^(\s*{re.escape(name)}\s*:\s*\w+\s*=\s*field\(\s*default\s*=\s*)([^,\)]*)",
        re.MULTILINE)


def _read_dataclass_field(path: Path, name: str) -> str | None:
    """Current raw default-value text of NAME's field(default=...), or
    None if NAME's field definition isn't found in path."""
    text = path.read_text()
    m = _dataclass_field_pattern(name).search(text)
    return m.group(2).strip() if m else None


def _rewrite_dataclass_field(path: Path, name: str, new_value: str) -> bool:
    """Rewrite NAME's field(default=...) value, leaving its metadata dict
    (unit/desc/controller, possibly spanning several lines) untouched.
    Returns False (no-op, file untouched) if NAME's field isn't found."""
    text = path.read_text()
    pattern = _dataclass_field_pattern(name)
    if not pattern.search(text):
        return False
    new_text = pattern.sub(lambda m: f"{m.group(1)}{new_value}", text, count=1)
    path.write_text(new_text)
    return True


# Matches one double-quoted Python string literal, capturing its INNER
# text (group 1) separately from the surrounding quotes.
_STRING_LITERAL_RE = r'"((?:[^"\\]|\\.)*)"'


def _read_dataclass_field_desc(path: Path, name: str) -> str:
    """Extracts field(default=..., metadata={"unit": ..., "desc": ...})'s
    unit/desc text straight out of mpc_params.py's own source, so the GUI's
    tooltip text can never drift from what the live file actually says --
    no separate copy of these strings is maintained in this GUI. Handles
    Python's implicit adjacent-string-literal concatenation (used by
    nmpc_q_epsi_dot's multi-line desc) and metadata dicts that span several
    lines. Returns '' if the field, or a desc within it, isn't found (never
    raises into the GUI over a docstring-formatting quirk elsewhere)."""
    if not path.is_file():
        return ""
    text = path.read_text()
    m = re.search(rf"^\s*{re.escape(name)}\s*:\s*\w+\s*=\s*field\(", text, re.MULTILINE)
    if not m:
        return ""
    # Bracket-depth scan from the opening '(' to its matching ')', so the
    # whole field(...) call is captured regardless of how many lines its
    # metadata dict spans.
    start = m.end() - 1
    depth = 0
    i = start
    while i < len(text):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                break
        i += 1
    block = text[m.start():i + 1]

    desc_match = re.search(rf'"desc"\s*:\s*((?:{_STRING_LITERAL_RE}\s*)+)', block)
    desc = "".join(m.group(1) for m in re.finditer(_STRING_LITERAL_RE, desc_match.group(1))) \
        if desc_match else ""

    unit_match = re.search(rf'"unit"\s*:\s*{_STRING_LITERAL_RE}', block)
    unit = unit_match.group(1) if unit_match else ""

    if desc and unit and unit not in ("unitless", "bool"):
        return f"{desc}  [{unit}]"
    return desc


def _backup_once(path: Path, done: set[Path]) -> None:
    """Back up path to path.bak the first time this session touches it, so
    a bad rewrite has a one-command recovery (`mv file.bak file`) without
    needing git -- launch_all.sh is a local, never-pushed file whose
    working-tree state matters session-to-session (see CLAUDE.md)."""
    if path in done:
        return
    shutil.copy(path, path.with_suffix(path.suffix + ".bak"))
    done.add(path)


def _run_detached(cmd: list[str], cwd: Path) -> subprocess.Popen:
    """Launch cmd as its own background process, own process group, no
    supervision from this GUI -- matches running it from a terminal.
    launch_all.sh/plot_playback.py/gui.simulation.py all already handle
    their own lifecycle (signal handling, window teardown). Returns the
    Popen handle so a caller that needs to stop it later (the Launch tab's
    Stop button) can -- callers that don't care are free to ignore it."""
    return subprocess.Popen(cmd, cwd=str(cwd), start_new_session=True)


def _sibling_path_csv(control_csv: Path) -> Path | None:
    """Return the sibling `<tag>_path_<stamp>.csv` next to control_csv, or
    None if absent -- same file-naming convention plot_playback.py's own
    _path_csv_for() re-derives (telemetry_logger.py always writes the two
    with matching tag/stamp, see its `paths` property), reimplemented here
    rather than imported so this GUI has no dependency on the tuner
    package's internals."""
    if "_control_" not in control_csv.name:
        return None
    candidate = control_csv.with_name(control_csv.name.replace("_control_", "_path_", 1))
    return candidate if candidate.exists() else None


# How long (seconds) the Launch tab's Stop button polls fsae_logs/ for a
# freshly-written CSV before giving up silently. ControlLogger.close()
# reliably finishes in well under a second in the normal single-Ctrl+C
# case (file close + optional header rewrite), but runs from the node's
# own signal handler asynchronously with this button's click, not
# synchronously with it -- a single immediate check routinely found
# nothing. A few seconds' margin comfortably covers the normal case
# without leaving the user waiting on a genuinely stuck/crashed node.
_STOP_LOG_POLL_TIMEOUT_S = 5.0

# How long (milliseconds) window teardown waits after signalling a running
# launch before destroying the window. Only a courtesy pause so the signal
# lands before this process exits -- launch_all.sh's cleanup runs in its
# own process group and finishes regardless of how long this GUI lives.
_CLOSE_STOP_GRACE_MS = 500


def _insert_run_label(filename: str, label: str | None) -> str:
    """Splices LABEL between the tag and `_control_`/`_path_` in filename,
    e.g. `mpc_standalone_control_123.csv` + "best" ->
    `mpc_standalone_best_control_123.csv` -- the same hand-labelled-run
    convention this project's own recorded_runs/ folder already uses (see
    debugging_tools.md's "descriptive topic segment... added by hand"
    note), which plot_playback.py's discovery already tolerates since it
    only looks for `_control_`/`_path_` plus the trailing stamp. Returns
    filename unchanged if label is empty/None or the expected marker isn't
    present."""
    if not label:
        return filename
    label = re.sub(r"\s+", "_", label.strip())
    for marker in ("_control_", "_path_"):
        if marker in filename:
            return filename.replace(marker, f"_{label}{marker}", 1)
    return filename


def _stop_process(proc: subprocess.Popen) -> None:
    """Sends SIGINT to proc's whole process group (it was started with
    start_new_session=True, i.e. it IS its own group leader), the same
    signal a terminal Ctrl+C delivers -- launch_all.sh's own `trap cleanup
    SIGINT SIGTERM` already handles this cleanly, so this does not need to
    do any of that teardown itself."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        pass


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
                            variable=self.controller_var).pack(side="left", padx=(0, 14))

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
        preview = "\n".join(f"  {k}={v}" for k, v in values.items())
        if not messagebox.askyesno(
                "Confirm launch",
                f"About to rewrite ros2/launch_all.sh with:\n\n{preview}\n\n"
                "and start it. Continue?"):
            return
        try:
            _backup_once(self._paths.launch_all_sh, self._backed_up)
            missing = [name for name, value in values.items()
                       if not _rewrite_var(self._paths.launch_all_sh, name, value)]
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


# ---------------------------------------------------------------------------
# Tab 2: Debug a Log
# ---------------------------------------------------------------------------

class LogDebugTab(ttk.Frame):
    def __init__(self, parent: ttk.Notebook, paths: RepoPaths) -> None:
        super().__init__(parent, padding=24)
        self._paths = paths

        ttk.Label(self, text="Debug a Log", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(self,
                  text="fsae_logs/ and fsds_simulator/recorded_runs/ — opens tuner.tools.plot_playback.",
                  style="Muted.TLabel").pack(anchor="w", pady=(2, 16))

        toolbar = ttk.Frame(self)
        toolbar.pack(fill="x", pady=(0, 12))
        ttk.Button(toolbar, text="Refresh", command=self._refresh).pack(side="left")
        ttk.Button(toolbar, text="Debug Latest (auto)",
                   command=self._debug_latest).pack(side="left", padx=(8, 0))
        ttk.Button(toolbar, text="Debug Selected", style="Accent.TButton",
                   command=self._debug_selected).pack(side="right")

        list_frame = ttk.Frame(self, style="Card.TFrame")
        list_frame.pack(fill="both", expand=True)
        list_frame.rowconfigure(0, weight=1)
        list_frame.columnconfigure(0, weight=1)

        self.listbox = tk.Listbox(
            list_frame, selectmode="extended", height=20,
            background=Palette.surface_alt, foreground=Palette.text,
            selectbackground=Palette.accent, selectforeground=Palette.accent_text,
            activestyle="none", borderwidth=0, highlightthickness=0,
            relief="flat",
        )
        self.listbox.grid(row=0, column=0, sticky="nsew", padx=1, pady=1)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.listbox.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.listbox.configure(yscrollcommand=scrollbar.set)

        self._entries: list[Path] = []
        self._refresh()

    def _find_control_csvs(self) -> list[Path]:
        found: list[Path] = []
        if self._paths.fsae_logs_dir.is_dir():
            found += sorted(self._paths.fsae_logs_dir.glob("*_control_*.csv"))
        root = self._paths.recorded_runs_dir
        if root.is_dir():
            found += sorted(root.glob("*_control_*.csv"))
            for sub in sorted(p for p in root.iterdir() if p.is_dir()):
                found += sorted(sub.glob("*_control_*.csv"))
        return found

    def _refresh(self) -> None:
        self._entries = self._find_control_csvs()
        self.listbox.delete(0, tk.END)
        if not self._entries:
            self.listbox.insert(tk.END, "  No recorded runs found yet.")
            self.listbox.configure(state="disabled")
            return
        self.listbox.configure(state="normal")
        for path in self._entries:
            try:
                label = str(path.relative_to(self._paths.fsae_mpctest))
            except ValueError:
                label = str(path)
            self.listbox.insert(tk.END, f"  {label}")

    def _debug_selected(self) -> None:
        selection = [self._entries[i] for i in self.listbox.curselection()]
        if not selection:
            messagebox.showinfo("Debug a Log", "Select one or more logs first.")
            return
        cmd = [sys.executable, "-m", "tuner.tools.plot_playback"] + [str(p) for p in selection]
        _run_detached(cmd, cwd=self._paths.fsae_mpctest)

    def _debug_latest(self) -> None:
        cmd = [sys.executable, "-m", "tuner.tools.plot_playback"]
        _run_detached(cmd, cwd=self._paths.fsae_mpctest)


# ---------------------------------------------------------------------------
# Tab 3: Run Offline Sim
# ---------------------------------------------------------------------------

class OfflineSimTab(ttk.Frame):
    def __init__(self, parent: ttk.Notebook, paths: RepoPaths) -> None:
        super().__init__(parent, padding=24)
        self._paths = paths

        ttk.Label(self, text="Run Offline Sim", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(self, text="2D matplotlib closed-loop tester — gui/simulation.py.",
                  style="Muted.TLabel").pack(anchor="w", pady=(2, 20))

        card = ttk.Frame(self, style="Card.TFrame", padding=16)
        card.pack(fill="x")
        ttk.Label(
            card, style="Card.TLabel", justify="left", wraplength=480,
            text=(
                "Rough signal only — its dynamics do not match FSDS. Cross-check "
                "anything that matters against\n"
                "python -m tuner.recorded_map_rollout or a real FSDS session "
                "before trusting it (see CLAUDE.md).\n\n"
                "Track/synthetic-path selection and initial-condition sliders are "
                "configured inside the tool itself.\n"
                "Q/R weights and NMPC overrides come from settings.py — use the "
                "Settings tab to change those first."
            ),
        ).pack(anchor="w")

        ttk.Button(self, text="Launch Offline Sim", style="Accent.TButton",
                   command=self._launch).pack(anchor="w", pady=(20, 0))

    def _launch(self) -> None:
        _run_detached([sys.executable, "-m", "gui.simulation"], cwd=self._paths.fsae_mpctest)


# ---------------------------------------------------------------------------
# Tab 4: Settings (settings.py)
# ---------------------------------------------------------------------------

# (label, settings.py name, mpc_params.py field name, kind) -- kind is
# 'float' or 'bool'. mpc_params.py name is None for a field with no live
# equivalent (skipped when syncing to the live file). Order matches
# settings.py's own section grouping.
_SCALAR_FIELDS: list[tuple[str, str, str | None, str]] = [
    ("R_A_ACCEL", "R_A_ACCEL", "r_a_accel", "float"),
    ("R_A_BRAKE", "R_A_BRAKE", "r_a_brake", "float"),
]

# NMPC weight overrides: -1.0 means "inherit the base weight", any other
# value diverges just that one weight for the NMPC -- see settings.py's own
# "NMPC weight overrides" section comment, and mpc_params.py's matching
# nmpc_* fields (same sentinel convention on both sides). Each gets an
# "override" checkbox (checked = use the number field's value, unchecked =
# write -1.0). (label, settings.py name, mpc_params.py name)
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
]

# (label, settings.py name, mpc_params.py field names for each list index).
# Only indices with a real live-side weight are mapped -- Q_diag[5:8]
# (e_a/delta_act/a_act) are always 0.0, not tunables, and R_diag[1] is
# nominal-only now (superseded by R_A_ACCEL/R_A_BRAKE, see settings.py's own
# comment on it), so neither gets a mpc_params.py entry (None = skip).
_LIST_FIELDS: list[tuple[str, str, int, list[str | None]]] = [
    ("Q_diag", "Q_diag", 8,
     ["q_e_y", "q_e_yd", "q_e_psi", "q_r", "q_e_v", None, None, None]),
    ("R_diag", "R_diag", 2, ["r_delta", None]),
    ("R_rate_diag", "R_rate_diag", 2, ["r_rate_delta", "r_rate_a"]),
]

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
# this grouping). (label, settings.py name or None, mpc_params.py name).
# settings.py name is None when the flag is live-only (delay compensation
# has no offline toggle -- the offline rollout always has it on).
_FEATURE_GROUPS: list[tuple[str, list[tuple[str, str | None, str]]]] = [
    ("Both controllers", [
        ("Delay compensation enabled", None, "delay_compensation_enabled"),
    ]),
    ("LTV-QP only", [
        ("Adaptive Q scaling enabled", "ADAPTIVE_Q_SCALING_ENABLED", "adaptive_q_scaling_enabled"),
        ("Steer-rate anti-hunt enabled", "STEER_RATE_ANTI_HUNT_ENABLED",
         "steer_rate_anti_hunt_enabled"),
        ("Adaptive R-rate enabled in corners", "ADAPTIVE_R_RATE_ENABLE_IN_CORNERS",
         "adaptive_r_rate_enable_in_corners"),
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
        ("Rate-cost 3-zone schedule enabled (experimental)", "NMPC_RRATE_ZONE_ENABLED",
         "nmpc_rrate_zone_enabled"),
        ("Corner rate-blend enabled (experimental)", "NMPC_CORNER_RRATE_BLEND_ENABLED",
         "nmpc_corner_rrate_blend_enabled"),
    ]),
]


class SettingsTab(ttk.Frame):
    def __init__(self, parent: ttk.Notebook, paths: RepoPaths) -> None:
        super().__init__(parent, padding=0)
        self._paths = paths
        self._backed_up: set[Path] = set()

        header = ttk.Frame(self, padding=(24, 24, 24, 0))
        header.pack(fill="x")
        ttk.Label(header, text="Settings", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(header, text="settings.py — saved changes apply the next time it's imported.",
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
        for label, name, length, _mpc_fields in _LIST_FIELDS:
            desc = _LIST_FIELD_DESC.get(name, "")
            _field_label(weights_card, r, label, desc, label_style="Card.TLabel",
                         desc_style="CardMuted.TLabel", wraplength=520)
            raw = _read_var(paths.settings_py, name) or "[]"
            current = _parse_float_list(raw, length)
            entry_frame = ttk.Frame(weights_card, style="Card.TFrame")
            entry_frame.grid(row=r, column=1, sticky="w")
            vars_for_field = []
            for i, v in enumerate(current):
                var = tk.StringVar(value=str(v))
                ttk.Entry(entry_frame, textvariable=var, width=7).grid(row=0, column=i, padx=3)
                vars_for_field.append(var)
            self._list_vars[name] = vars_for_field
            r += 2 if desc else 1

        ttk.Separator(weights_card).grid(row=r, column=0, columnspan=2, sticky="ew", pady=12)
        r += 1

        self._scalar_vars: dict[str, tk.Variable] = {}
        for label, name, _mpc_field, kind in _SCALAR_FIELDS:
            desc = _read_dataclass_field_desc(paths.mpc_params_py, _mpc_field) if _mpc_field else ""
            _field_label(weights_card, r, label, desc, label_style="Card.TLabel",
                         desc_style="CardMuted.TLabel", wraplength=420)
            raw = _read_var(paths.settings_py, name)
            if kind == "bool":
                var = tk.BooleanVar(value=(raw or "False").strip() == "True")
                ttk.Checkbutton(weights_card, style="Card.TCheckbutton", variable=var).grid(
                    row=r, column=1, sticky="w")
            else:
                var = tk.StringVar(value=raw or "0.0")
                ttk.Entry(weights_card, textvariable=var, width=10).grid(
                    row=r, column=1, sticky="w")
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
            raw = _read_var(paths.settings_py, name) or "-1.0"
            is_override = _to_float(raw) is not None and _to_float(raw) >= 0.0
            enabled_var = tk.BooleanVar(value=is_override)
            value_var = tk.StringVar(value=raw if is_override else "")
            ttk.Checkbutton(overrides_card, text="override", style="Card.TCheckbutton",
                             variable=enabled_var).grid(row=r, column=1, sticky="w", padx=(0, 12))
            ttk.Entry(overrides_card, textvariable=value_var, width=10).grid(
                row=r, column=2, sticky="w")
            self._override_vars[name] = (enabled_var, value_var)
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
                if settings_name is None:
                    desc = (desc + " " if desc else "") + "(live-only, no settings.py equivalent)"
                _field_label(group_card, r, label, desc, label_style="Card.TLabel",
                             desc_style="CardMuted.TLabel", wraplength=520)
                raw = _read_var(paths.settings_py, settings_name) if settings_name else None
                if raw is None:
                    raw = _read_dataclass_field(paths.mpc_params_py, mpc_field)
                var = tk.BooleanVar(value=(raw or "False").strip() in ("True", "true"))
                ttk.Checkbutton(group_card, style="Card.TCheckbutton", variable=var).grid(
                    row=r, column=1, sticky="w")
                # Keyed by mpc_field (always present) since settings_name can be None.
                self._feature_vars[mpc_field] = var
                r += 2 if desc else 1

        footer = ttk.Frame(body)
        footer.pack(fill="x", pady=(24, 0))
        ttk.Button(footer, text="Save", style="Accent.TButton",
                   command=self._on_save).pack(side="left")
        self.status_var = tk.StringVar(value="")
        ttk.Label(footer, textvariable=self.status_var, style="Muted.TLabel").pack(
            side="left", padx=(14, 0))

    def _on_save(self) -> None:
        settings_path = self._paths.settings_py
        mpc_params_path = self._paths.mpc_params_py
        sync_live = mpc_params_path.is_file()
        try:
            errors: list[str] = []
            _backup_once(settings_path, self._backed_up)
            if sync_live:
                _backup_once(mpc_params_path, self._backed_up)
            elif not getattr(self, "_warned_no_live_file", False):
                self._warned_no_live_file = True
                messagebox.showwarning(
                    "Settings tab",
                    f"Live file not found at:\n{mpc_params_path}\n\n"
                    "Saving settings.py only -- the live simulator's mpc_params.py will "
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
                    errors.append(f"{name}: assignment not found in settings.py")
                if sync_live:
                    for value, mpc_field in zip(values, mpc_fields):
                        if mpc_field is None:
                            continue
                        if not _rewrite_dataclass_field(mpc_params_path, mpc_field, repr(value)):
                            errors.append(f"{mpc_field}: field not found in mpc_params.py")

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
                    errors.append(f"{name}: assignment not found in settings.py")
                if sync_live and mpc_field is not None:
                    if not _rewrite_dataclass_field(mpc_params_path, mpc_field, literal):
                        errors.append(f"{mpc_field}: field not found in mpc_params.py")

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
                    errors.append(f"{name}: assignment not found in settings.py")
                if sync_live:
                    if not _rewrite_dataclass_field(mpc_params_path, mpc_field, literal):
                        errors.append(f"{mpc_field}: field not found in mpc_params.py")

            for mpc_field, var in self._feature_vars.items():
                settings_name = next(
                    (s for _grp, entries in _FEATURE_GROUPS for _l, s, f in entries
                     if f == mpc_field), None)
                literal = "True" if var.get() else "False"
                if settings_name is not None:
                    if not _rewrite_var(settings_path, settings_name, literal):
                        errors.append(f"{settings_name}: assignment not found in settings.py")
                if sync_live:
                    if not _rewrite_dataclass_field(mpc_params_path, mpc_field, literal):
                        errors.append(f"{mpc_field}: field not found in mpc_params.py")

            if errors:
                messagebox.showerror("Settings tab", "\n".join(errors))
                self.status_var.set("Save had errors, see dialog.")
            elif sync_live:
                self.status_var.set(
                    "Saved to settings.py and the live mpc_params.py. "
                    "Restart the sim to pick up the live change.")
            else:
                self.status_var.set("Saved. Takes effect next time settings.py is imported.")
        except OSError as exc:
            messagebox.showerror("Settings tab", f"Failed to save: {exc!r}")


def _to_float(text: str) -> float | None:
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def _parse_float_list(literal: str, expected_length: int) -> list[float]:
    """Best-effort parse of a `[1.0, 2, 3.5]`-style literal already
    extracted from a settings.py line. Falls back to zeros of the expected
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


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

class LauncherApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("FSDS / fsae_MPCTest launcher")
        self.geometry("780x680")
        self.minsize(620, 480)

        apply_theme(self)
        self.configure(bg=Palette.bg)

        paths = _repo_paths()
        notebook = ttk.Notebook(self, padding=(0, 0))
        notebook.pack(fill="both", expand=True, padx=10, pady=10)

        self._launch_tab = LaunchTab(notebook, paths)
        notebook.add(self._launch_tab, text="Launch Sim")
        notebook.add(LogDebugTab(notebook, paths), text="Debug a Log")
        notebook.add(OfflineSimTab(notebook, paths), text="Run Offline Sim")
        notebook.add(SettingsTab(notebook, paths), text="Settings")

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self) -> None:
        """Stops a sim still running under the Launch tab before tearing
        down the window. launch_all.sh runs in its own process group and
        is not supervised by this GUI, so without this it would keep
        running (nodes, bridge, diagnostic captures) with the only Stop
        button gone."""
        if self._launch_tab.stop_running_sim():
            # Give launch_all.sh's own `trap cleanup` a moment to act on
            # the SIGINT before the interpreter exits. Not a guarantee of
            # completion, just avoids racing teardown against process
            # exit; cleanup continues independently either way.
            self.after(_CLOSE_STOP_GRACE_MS, self.destroy)
            return
        self.destroy()


def main() -> None:
    LauncherApp().mainloop()


if __name__ == "__main__":
    main()
