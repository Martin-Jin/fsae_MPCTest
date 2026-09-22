"""
tuner/tools/sync_mpc_params.py — push live-tested MPC params to
fsae_autonomous and fsae_MPCTest's fsds_simulator mirror.

Run from fsae_MPCTest/:

    python -m tuner.tools.sync_mpc_params            # dry run, shows diffs only
    python -m tuner.tools.sync_mpc_params --apply     # actually overwrite

WHY THIS EXISTS
---------------
The car's actual runtime weights live in THREE files per side
(mpc_params.py, nmpc_params.py, fsae_params.yaml, see CLAUDE.md's "Single
source of truth for MPC tuning"), and fsae_params.yaml specifically
OVERRIDES the dataclass default= at ROS param declare time -- syncing the
.py files without it leaves the old number running with no visible sign
anything is wrong (this happened at least twice: r_a_accel and
nmpc_track_halfwidth both went stale this way before the fix that added
fsae_params.yaml to the GUI's own save path). This script exists so a
live-tested change only has to be copied by hand ONCE (into
fsae_planning, the live checkout) and then pushed everywhere else with
one command, rather than three manual file edits repeated across two
more repos.

ONE-WAY, FROM LIVE ONLY. fsae_planning (ros2/src/fsae_planning/) is
always the source; fsae_autonomous and fsae_MPCTest/fsds_simulator/ are
always the destinations. This matches the stated workflow: params get
retuned and validated live first, and only THEN pushed out to the other
two checkouts, never the other direction. This script does not read
fsae_autonomous's or fsae_MPCTest's own current values as a source for
anything.

WHAT IT SYNCS
-------------
Exactly the 3 files CLAUDE.md's "Single source of truth for MPC tuning"
section names as the live side of the parity boundary:
    mpc/mpc_params.py
    mpc/nmpc_params.py
    common/fsae_bringup/config/fsae_params.yaml
Nothing else. In particular this does NOT touch mpc_core.py/nmpc_core.py/
mpc_controller.py/live_viz.py or any other source file -- those still
need the ordinary manual "propagate this code change" step per
CLAUDE.md's "Third copy" section, this script is scoped to tunable
PARAMS only, per the request that created it.

SAFETY
------
Dry run by default: prints a unified diff per file per destination and
touches nothing. --apply is required to actually overwrite, and each
destination still gets a one-time .bak backup (skipped if a .bak from
this run already exists) before being overwritten, same convention as
gui/launcher.py's own file-safety mechanism.

fsae_autonomous is a production repo this project's own CLAUDE.md never
lets an agent commit or push -- this script only ever writes into the
LOCAL working tree there. Committing/pushing fsae_autonomous (and
mirroring to fsae_MPCRos, see CLAUDE.md's own section on that) stays a
separate, deliberate, human-initiated step.
"""
from __future__ import annotations

import argparse
import difflib
import sys
from pathlib import Path

# Paths are resolved relative to this file, not the CWD, so the script
# works whether invoked from fsae_MPCTest/ (the documented way) or
# anywhere else -- same reasoning as gui/launcher.py's _repo_paths().
_FSAE_MPCTEST = Path(__file__).resolve().parent.parent.parent
_FSDS_ROOT = _FSAE_MPCTEST.parent

# Relative path (from a repo's fsae_control package root) of each synced
# file, used to build every side's full path below. Order matches
# CLAUDE.md's own file list.
_RELATIVE_PATHS = {
    "mpc_params.py": Path("control/fsae_control/fsae_control/mpc/mpc_params.py"),
    "nmpc_params.py": Path("control/fsae_control/fsae_control/mpc/nmpc_params.py"),
    "fsae_params.yaml": Path("common/fsae_bringup/config/fsae_params.yaml"),
}

_LIVE_ROOT = _FSDS_ROOT / "ros2" / "src" / "fsae_planning"
_MIRROR_ROOT = _FSAE_MPCTEST / "fsds_simulator"
# fsae_autonomous is a SIBLING checkout of the outer FSDS repo per
# CLAUDE.md's documented layout. If that layout is stale (it was found to
# be, 2026-09-23: the real checkout is at ros2_autonomous/src/
# fsae_autonomous/), _find_autonomous_root() below searches a short list
# of known-observed locations instead of hardcoding one, and reports
# clearly if neither is found rather than silently doing nothing.
_AUTONOMOUS_CANDIDATES = [
    _FSDS_ROOT / "fsae_autonomous",
    _FSDS_ROOT / "ros2_autonomous" / "src" / "fsae_autonomous",
]


def _find_autonomous_root() -> Path | None:
    for candidate in _AUTONOMOUS_CANDIDATES:
        if (candidate / ".git").is_dir():
            return candidate
    return None


def _destinations() -> list[tuple[str, Path]]:
    dests = [("fsds_simulator mirror", _MIRROR_ROOT)]
    autonomous_root = _find_autonomous_root()
    if autonomous_root is not None:
        dests.append(("fsae_autonomous", autonomous_root))
    else:
        print(
            "WARNING: fsae_autonomous checkout not found at any known "
            f"location ({', '.join(str(c) for c in _AUTONOMOUS_CANDIDATES)}). "
            "Skipping it -- only the fsds_simulator mirror will be synced. "
            "If it has moved again, add its new path to _AUTONOMOUS_CANDIDATES.",
            file=sys.stderr,
        )
    return dests


def _backup_once(path: Path) -> None:
    backup = path.with_suffix(path.suffix + ".bak")
    if backup.exists():
        return
    backup.write_bytes(path.read_bytes())


def _show_diff(label: str, src_text: str, dst_text: str, dst_path: Path) -> bool:
    """Prints a unified diff if src and dst differ. Returns True if they
    differ (i.e. this file needs syncing)."""
    if src_text == dst_text:
        return False
    diff = difflib.unified_diff(
        dst_text.splitlines(keepends=True),
        src_text.splitlines(keepends=True),
        fromfile=f"{label} (current)",
        tofile=f"{label} (would become)",
    )
    print(f"\n=== {dst_path} ===")
    sys.stdout.writelines(diff)
    return True


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apply", action="store_true",
                     help="Actually overwrite destination files. Without this, "
                          "only a diff preview is printed and nothing is touched.")
    args = ap.parse_args()

    if not _LIVE_ROOT.is_dir():
        sys.exit(f"Live source checkout not found at {_LIVE_ROOT}")

    destinations = _destinations()
    any_diff = False
    any_error = False
    changed_labels: set[str] = set()
    missing_labels: set[str] = set()

    for name, rel_path in _RELATIVE_PATHS.items():
        src_path = _LIVE_ROOT / rel_path
        if not src_path.is_file():
            print(f"ERROR: live source file missing: {src_path}", file=sys.stderr)
            any_error = True
            continue
        src_text = src_path.read_text()

        for dest_label, dest_root in destinations:
            dst_path = dest_root / rel_path
            if not dst_path.is_file():
                print(f"ERROR: {dest_label} destination file missing: {dst_path}",
                      file=sys.stderr)
                any_error = True
                missing_labels.add(dest_label)
                continue
            dst_text = dst_path.read_text()
            differs = _show_diff(f"{name} -> {dest_label}", src_text, dst_text, dst_path)
            any_diff = any_diff or differs
            if differs:
                changed_labels.add(dest_label)
            if differs and args.apply:
                _backup_once(dst_path)
                dst_path.write_text(src_text)
                print(f"  written ({dst_path})")

    # Machine-readable summary line, for any caller (the GUI's "Overwrite
    # All Params" button in particular) that wants to know WHICH named
    # destinations actually differ without parsing the diff output above --
    # printed regardless of --apply, since a dry run needs this just as
    # much as an apply run does, to build an accurate confirmation prompt.
    for label, _root in destinations:
        state = "changed" if label in changed_labels else (
            "missing" if label in missing_labels else "unchanged")
        print(f"SYNC-TARGET: {label} = {state}")

    if any_error:
        sys.exit(1)
    if not any_diff:
        print("Already in sync: every destination matches the live source.")
        return
    if not args.apply:
        print("\nDry run only -- nothing written. Re-run with --apply to sync.")
    else:
        print("\nSync complete.")
        print("Remember: fsae_autonomous is never committed/pushed by an agent, "
              "and fsae_MPCTest's own git status will show the mirror change "
              "for you to commit/push separately, same as any other edit.")


if __name__ == "__main__":
    main()
