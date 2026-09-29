"""
gui/launcher/app.py — LauncherApp: the five-tab Tk window that ties every
other module in this package together, plus main() for `python -m
gui.launcher`.
"""

from __future__ import annotations

import signal
import tkinter as tk
from tkinter import messagebox, ttk

from gui.launcher.paths import _repo_paths
from gui.launcher.tabs.launch import LaunchTab
from gui.launcher.tabs.log_debug import LogDebugTab
from gui.launcher.tabs.offline_sim import OfflineSimTab
from gui.launcher.tabs.profiles import ProfilesTab
from gui.launcher.tabs.settings import SettingsTab
from gui.launcher.theme import Palette, apply_theme

# How long (milliseconds) window teardown waits after signalling a running
# launch before destroying the window. Only a courtesy pause so the signal
# lands before this process exits -- launch_all.sh's cleanup runs in its
# own process group and finishes regardless of how long this GUI lives.
_CLOSE_STOP_GRACE_MS = 500



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
        self._settings_tab = SettingsTab(notebook, paths)
        notebook.add(self._settings_tab, text="Settings")
        notebook.add(ProfilesTab(notebook, paths, self._settings_tab, self._launch_tab), text="Profiles")

        # Warn on leaving the Settings tab with unsaved edits, not just on
        # window close -- switching to Launch Sim/Profiles/etc. and back is
        # the more common way an edit gets silently forgotten, since nothing
        # else about the app suggests leaving a tab discards anything.
        self._notebook = notebook
        self._last_tab_was_settings = False
        self._reentering_tab_guard = False
        notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Ctrl+C in the terminal that started this GUI used to just kill
        # this process outright (Tk's mainloop() blocks in C and never
        # calls _on_close), leaving launch_all.sh's whole process group
        # (nodes, bridge, diagnostic captures) running with no Stop button
        # left to press -- the exact strand _on_close's own docstring
        # describes for the window-close path, but via SIGINT instead.
        # signal.signal() only queues the Python-level handler; it will not
        # actually run until the interpreter next checks for one, which a
        # blocked C mainloop never does on its own. The periodic no-op
        # self.after() below is what forces that check often enough for
        # Ctrl+C to feel immediate.
        signal.signal(signal.SIGINT, self._on_sigint)
        self._pump_for_signals()

    def _pump_for_signals(self) -> None:
        self.after(200, self._pump_for_signals)

    def _on_sigint(self, signum, frame) -> None:
        self._on_close()

    def _on_tab_changed(self, _event=None) -> None:
        """Fires on EVERY tab switch, both away from and onto Settings, so
        this only acts the moment the PREVIOUS tab was Settings and it had
        unsaved edits (checked before _last_tab_was_settings is updated
        below) -- warning when arriving at Settings, or on every switch
        regardless of which tab, would fire constantly for no reason.

        ttk.Notebook.select() re-triggers this same virtual event, but NOT
        synchronously (measured, not assumed): the "No, stay on Settings"
        branch below calls select() to force the reselect, and Tk only
        actually DELIVERS that event later, after select() has already
        returned -- a `finally:`-style guard reset right after the call
        clears itself before the queued event arrives and does nothing.
        _reentering_tab_guard is instead cleared via after_idle(), which
        runs after the current event queue (including the reselect's own
        queued <<NotebookTabChanged>>) has been drained."""
        if self._reentering_tab_guard:
            return
        if self._last_tab_was_settings and self._settings_tab.has_unsaved_changes():
            settings_index = self._notebook.index(self._settings_tab)
            if not messagebox.askyesno(
                    "Unsaved changes",
                    "The Settings tab has unsaved changes. Switch away anyway?\n\n"
                    "Nothing is lost from this tab's own widgets, but settings.py/the "
                    "live files still hold the OLD values until you press Save."):
                self._reentering_tab_guard = True
                self._notebook.select(settings_index)
                self.after_idle(self._clear_reentering_tab_guard)
                return
        # Re-sync the Launch tab's widgets from launch_all.sh on arrival,
        # not just at GUI startup -- see refresh_from_disk()'s own
        # docstring for why a stale widget here silently reverts a more
        # recent on-disk edit the next time Launch is pressed.
        if self._notebook.select() == str(self._launch_tab):
            self._launch_tab.refresh_from_disk()
        self._last_tab_was_settings = (
            self._notebook.select() == str(self._settings_tab))

    def _clear_reentering_tab_guard(self) -> None:
        self._reentering_tab_guard = False

    def _on_close(self) -> None:
        """Stops a sim still running under the Launch tab before tearing
        down the window. launch_all.sh runs in its own process group and
        is not supervised by this GUI, so without this it would keep
        running (nodes, bridge, diagnostic captures) with the only Stop
        button gone."""
        if self._settings_tab.has_unsaved_changes() and not messagebox.askyesno(
                "Unsaved changes",
                "The Settings tab has unsaved changes that will be lost "
                "(the widgets, not settings.py, since nothing has written them "
                "yet). Quit anyway?"):
            return
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

