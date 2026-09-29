"""
gui/launcher/paths.py — RepoPaths: every file/directory this GUI reads or
writes, resolved once from this package's own location on disk (see
_repo_paths' ASSUMED LAYOUT note in the docstring below).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

@dataclass(frozen=True)
class RepoPaths:
    fsae_mpctest: Path
    fsds_root: Path
    launch_all_sh: Path
    tracks_dir: Path
    recorded_runs_dir: Path
    fsae_logs_dir: Path
    # Directory, not a single file: settings.py became the settings/
    # package (settings/general.py, settings/nmpc.py, ...). _read_var/
    # _rewrite_var search every settings/*.py file for NAME's assignment,
    # since a field's submodule isn't tracked here -- see those functions'
    # own docstrings.
    settings_dir: Path
    mpc_params_py: Path
    # NMPC structural/solver fields live in their own dataclass, separate
    # from mpc_params.py's weights -- see nmpc_params.py's own docstring.
    # Only the fields this GUI actually writes need to be found here.
    nmpc_params_py: Path
    # fsae_params.yaml is what ROS actually loads a field's RUNTIME value
    # from -- it overrides the dataclass default the GUI edits above, so a
    # value the GUI "saved" could silently keep running at the OLD number
    # until this file was found and fixed by hand (r_a_accel 2.25 vs 1.0,
    # nmpc_track_halfwidth 3.0 vs 3.5 both did this). The mirror copies
    # under fsds_simulator/ are the change-ledger for fsae_planning (see
    # CLAUDE.md's "Third copy" section) and must track the live ones.
    fsae_params_yaml: Path
    mirror_mpc_params_py: Path
    mirror_nmpc_params_py: Path
    mirror_fsae_params_yaml: Path
    # Named snapshots of the Settings tab's full field set (see
    # _PROFILE_FIELD_NAMES), one JSON file per profile. Lives in
    # fsae_MPCTest (offline repo), not ros2/ or its mirror -- a profile is
    # a GUI/tuning convenience, not part of the change ledger for
    # fsae_planning, so it does not belong under fsds_simulator/.
    profiles_dir: Path


def _repo_paths() -> RepoPaths:
    # This file now lives at gui/launcher/paths.py (one level deeper than
    # the original gui/launcher.py), so resolving fsae_MPCTest/ needs one
    # more .parent than before: paths.py -> launcher/ -> gui/ -> fsae_MPCTest/.
    fsae_mpctest = Path(__file__).resolve().parent.parent.parent
    fsds_root = fsae_mpctest.parent
    fsae_planning_mpc = (fsds_root / "ros2" / "src" / "fsae_planning" / "control"
                         / "fsae_control" / "fsae_control" / "mpc")
    mirror_root = fsae_mpctest / "fsds_simulator"
    mirror_mpc = mirror_root / "control" / "fsae_control" / "fsae_control" / "mpc"
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
        settings_dir=fsae_mpctest / "settings",
        mpc_params_py=fsae_planning_mpc / "mpc_params.py",
        nmpc_params_py=fsae_planning_mpc / "nmpc_params.py",
        fsae_params_yaml=(fsds_root / "ros2" / "src" / "fsae_planning" / "common"
                          / "fsae_bringup" / "config" / "fsae_params.yaml"),
        mirror_mpc_params_py=mirror_mpc / "mpc_params.py",
        mirror_nmpc_params_py=mirror_mpc / "nmpc_params.py",
        mirror_fsae_params_yaml=(mirror_root / "common" / "fsae_bringup"
                                 / "config" / "fsae_params.yaml"),
        profiles_dir=fsae_mpctest / "settings_profiles",
    )

