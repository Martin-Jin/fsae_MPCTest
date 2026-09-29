"""
gui/launcher/process_utils.py — subprocess lifecycle and recorded-run
filename helpers shared by LaunchTab and OfflineSimTab.
"""

from __future__ import annotations

import os
import re
import signal
import subprocess
from pathlib import Path

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

