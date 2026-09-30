"""
gui/launcher/tabs/profiles.py — Tab 5: Profiles. Named snapshots of every
field the Settings tab manages plus the Launch tab's own driving
configuration, stored as one JSON file per profile.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

from gui.launcher.paths import RepoPaths
from gui.launcher.tabs.launch import LaunchTab
from gui.launcher.tabs.settings import SettingsTab, _profile_field_names
from gui.launcher.theme import Palette

# ---------------------------------------------------------------------------
# Tab 5: Profiles
# ---------------------------------------------------------------------------

class ProfilesTab(ttk.Frame):
    """Named snapshots of every field the Settings tab manages (see
    _profile_field_names()) PLUS the Launch tab's own driving configuration
    (track, controller, precomputed speed/path, V_MAX/V_MIN,
    NMPC_PROGRESS_ENABLED), stored as one JSON file per profile under
    RepoPaths.profiles_dir. A profile used to capture only the Settings
    tab's weights/flags -- loading one left whatever track/controller/
    precomputed-path choice was already selected untouched, so it did not
    actually reproduce the run it was saved from unless those separately
    already matched. Save/Load go through SettingsTab's/LaunchTab's own
    capture_profile_values()/apply_profile_values() so a loaded profile is
    written the same way a normal edit on either tab would be, via the
    exact same code paths, not a third one. The two tabs' key sets are
    disjoint (Settings tab uses settings.py NAMEs like Q_diag/NMPC_Q_E_Y;
    Launch tab uses launch_all.sh NAMEs like TRACK/CONTROLLER), so they
    merge into one flat dict with no collision to resolve."""

    def __init__(self, parent: ttk.Notebook, paths: RepoPaths, settings_tab: "SettingsTab",
                 launch_tab: "LaunchTab") -> None:
        super().__init__(parent, padding=24)
        self._paths = paths
        self._settings_tab = settings_tab
        self._launch_tab = launch_tab

        ttk.Label(self, text="Profiles", style="Heading.TLabel").pack(anchor="w")
        ttk.Label(
            self,
            text=f"settings_profiles/ — full snapshots of every Settings-tab field plus the "
                 f"Launch tab's driving configuration "
                 f"({len(_profile_field_names())}+ values).",
            style="Muted.TLabel").pack(anchor="w", pady=(2, 16))

        toolbar = ttk.Frame(self)
        toolbar.pack(fill="x", pady=(0, 12))
        ttk.Button(toolbar, text="Refresh", command=self._refresh).pack(side="left")
        ttk.Button(toolbar, text="Delete Selected",
                   command=self._delete_selected).pack(side="left", padx=(8, 0))
        ttk.Button(toolbar, text="Load Selected", style="Accent.TButton",
                   command=self._load_selected).pack(side="right")
        ttk.Button(toolbar, text="Save Current As Profile...",
                   command=self._save_current).pack(side="right", padx=(0, 8))

        list_frame = ttk.Frame(self, style="Card.TFrame")
        list_frame.pack(fill="both", expand=True)
        list_frame.rowconfigure(0, weight=1)
        list_frame.columnconfigure(0, weight=1)

        self.listbox = tk.Listbox(
            list_frame, selectmode="browse", height=20,
            background=Palette.surface_alt, foreground=Palette.text,
            selectbackground=Palette.accent, selectforeground=Palette.accent_text,
            activestyle="none", borderwidth=0, highlightthickness=0,
            relief="flat",
        )
        self.listbox.grid(row=0, column=0, sticky="nsew", padx=1, pady=1)
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.listbox.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.listbox.configure(yscrollcommand=scrollbar.set)

        self.status_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.status_var, style="Muted.TLabel").pack(
            anchor="w", pady=(10, 0))

        self._entries: list[Path] = []
        self._refresh()

    def _refresh(self) -> None:
        self.listbox.delete(0, tk.END)
        self._entries = sorted(
            self._paths.profiles_dir.glob("*.json")) if self._paths.profiles_dir.is_dir() else []
        for path in self._entries:
            self.listbox.insert(tk.END, f"  {path.stem}")
        self.status_var.set(f"{len(self._entries)} profile(s) in {self._paths.profiles_dir}")

    def _selected_path(self) -> Path | None:
        selection = self.listbox.curselection()
        if not selection:
            messagebox.showinfo("Profiles", "Select a profile first.")
            return None
        return self._entries[selection[0]]

    def _save_current(self) -> None:
        name = simpledialog.askstring(
            "Save profile", "Profile name:", parent=self)
        if not name:
            return
        name = name.strip()
        if not name:
            return
        # Keep the on-disk filename obviously tied to the name typed in,
        # while still being a safe filename on every OS this GUI runs on
        # (Windows forbids \\/:*?"<>| in a filename; POSIX only really
        # cares about / and NUL, but the stricter set costs nothing here).
        safe_name = re.sub(r'[\\/:*?"<>|]', "_", name).strip()
        if not safe_name:
            messagebox.showerror("Profiles", "That name has no valid characters left.")
            return
        target = self._paths.profiles_dir / f"{safe_name}.json"
        if target.exists() and not messagebox.askyesno(
                "Save profile", f"'{safe_name}' already exists. Overwrite it?"):
            return
        try:
            self._paths.profiles_dir.mkdir(parents=True, exist_ok=True)
            values = self._settings_tab.capture_profile_values()
            values.update(self._launch_tab.capture_profile_values())
            payload = {"name": name, "values": values}
            target.write_text(json.dumps(payload, indent=2, sort_keys=True))
        except OSError as exc:
            messagebox.showerror("Profiles", f"Failed to save profile: {exc!r}")
            return
        self._refresh()
        self.status_var.set(f"Saved '{safe_name}' ({len(values)} values).")

    def _load_selected(self) -> None:
        path = self._selected_path()
        if path is None:
            return
        try:
            payload = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            messagebox.showerror("Profiles", f"Failed to read profile: {exc!r}")
            return
        values = payload.get("values", {})
        if not messagebox.askyesno(
                "Load profile",
                f"Overwrite EVERY current Settings-tab AND Launch-tab value with "
                f"'{path.stem}' ({len(values)} values)? The Settings-tab values write "
                "settings.py, the live dataclasses, both fsae_params.yaml copies, and "
                "both fsds_simulator/ mirrors immediately, the same as pressing Save on "
                "the Settings tab. The Launch-tab values (track/controller/precomputed "
                "speed & path/V_MAX/V_MIN/progress term) only update that tab's widgets "
                "-- launch_all.sh itself is not touched until the next Launch click."):
            return
        self._settings_tab.apply_profile_values(values)
        self._launch_tab.apply_profile_values(values)
        self.status_var.set(f"Loaded '{path.stem}'. Restart the sim to pick up the live change.")

    def _delete_selected(self) -> None:
        path = self._selected_path()
        if path is None:
            return
        if not messagebox.askyesno("Delete profile", f"Delete profile '{path.stem}'? "
                                    "This cannot be undone."):
            return
        try:
            path.unlink()
        except OSError as exc:
            messagebox.showerror("Profiles", f"Failed to delete profile: {exc!r}")
            return
        self._refresh()
        self.status_var.set(f"Deleted '{path.stem}'.")

