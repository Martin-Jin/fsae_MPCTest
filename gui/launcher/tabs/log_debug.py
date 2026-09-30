"""
gui/launcher/tabs/log_debug.py — Tab 2: Debug a Log. A file browser over
fsae_logs/ and fsds_simulator/recorded_runs/, then runs
tuner.tools.plot_playback on the selection.
"""

from __future__ import annotations

import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from gui.launcher.paths import RepoPaths
from gui.launcher.process_utils import _run_detached
from gui.launcher.theme import Palette

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

