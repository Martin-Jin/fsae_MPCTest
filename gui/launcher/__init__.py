"""
gui/launcher/ — Centralized launcher/debug GUI

PURPOSE
-------
One tkinter app, tabbed by tool, wrapping the separate entry points this
project otherwise requires editing a script/CLI for by hand:

  1. Launch Sim      — rewrites the commonly-changed variables in
                        ros2/launch_all.sh, then runs it.
  2. Debug a Log      — a file browser over fsae_logs/ and
                        fsds_simulator/recorded_runs/, then runs
                        tuner.tools.plot_playback on the selection.
  3. Run Offline Sim  — launches gui/simulation.py (the 2D matplotlib tool).
  4. Settings         — edits the handful of commonly-retuned settings/*.py
                        constants (Q/R weights, NMPC overrides, adaptive
                        feature flags) in place.
  5. Profiles         — named snapshots of the Settings + Launch tabs'
                        combined state.

This package contains NO simulation/plotting/tuning logic of its own:
every button shells out to an existing, already-working tool via
subprocess. Rewriting ros2/launch_all.sh / settings/*.py in place is the
only state this package mutates; both use plain `NAME = value` /
`NAME=value` line-oriented fields with no multi-line assignments among the
fields this GUI touches, so a single regex substitution per field is
enough (see file_edit.py).

LAYOUT
------
  theme.py           the ttk "clam"-based palette and shared widget helpers
  paths.py           RepoPaths, every file/directory this GUI touches
  file_edit.py        regex read/write helpers shared by every tab
  process_utils.py    subprocess lifecycle + recorded-run filename helpers
  tabs/launch.py       Tab 1
  tabs/log_debug.py    Tab 2
  tabs/offline_sim.py  Tab 3
  tabs/settings.py     Tab 4
  tabs/profiles.py     Tab 5
  app.py              LauncherApp (ties every tab together) + main()

USED BY
-------
  Standalone: run with `python -m gui.launcher` from fsae_MPCTest/
  (see __main__.py -- needed because this is now a package, not a single
  module, so `python -m` can't find main() without it).

ASSUMED LAYOUT
--------------
Per this project's documented repo layout: paths.py's own location
resolves fsae_MPCTest/'s root three levels up (gui/launcher/paths.py ->
launcher/ -> gui/ -> fsae_MPCTest/), and the outer FSDS sim repo root is
fsae_MPCTest/'s own parent, with ros2/launch_all.sh and
ros2/src/fsae_planning/tracks/ under that same root (see paths._repo_paths).
"""
from __future__ import annotations

from gui.launcher.app import LauncherApp, main

__all__ = ["LauncherApp", "main"]
