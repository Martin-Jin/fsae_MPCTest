"""
gui/launcher/tabs/offline_sim.py — Tab 3: Run Offline Sim. Launches
gui/simulation.py (the 2D matplotlib tool).
"""

from __future__ import annotations

import sys
from tkinter import ttk

from gui.launcher.paths import RepoPaths
from gui.launcher.process_utils import _run_detached

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
                "python -m tuner.validation.recorded_map_rollout or a real FSDS session "
                "before trusting it (see CLAUDE.md).\n\n"
                "Track/synthetic-path selection and initial-condition sliders are "
                "configured inside the tool itself.\n"
                "Q/R weights and NMPC overrides come from the settings package — use the "
                "Settings tab to change those first."
            ),
        ).pack(anchor="w")

        ttk.Button(self, text="Launch Offline Sim", style="Accent.TButton",
                   command=self._launch).pack(anchor="w", pady=(20, 0))

    def _launch(self) -> None:
        _run_detached([sys.executable, "-m", "gui.simulation"], cwd=self._paths.fsae_mpctest)

