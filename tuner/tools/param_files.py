"""
tuner/tools/param_files.py — where the MPC/NMPC parameter files live in
each checkout, and the helpers that read and rewrite them. No CLI.

Shared by tuner/tools/sync_mpc_params.py, tuner/tools/param_report.py and
the launcher GUI (gui/launcher/), so the repo layout, the settings/ <->
dataclass index map and the launch_all.sh parsing exist once. Kept free of
tkinter so the command-line tools can import it headless.

All of the settings package's files, launch_all.sh, fsae_params.yaml, and
mpc_params.py/nmpc_params.py use the same flat "NAME = value" /
"NAME=value" style with no multi-line assignments for any field these
tools touch, so a single regex substitution per field is enough.
"""

from __future__ import annotations

import re
import shutil
from pathlib import Path

# ---------------------------------------------------------------------------
# Repo layout. Resolved from this file, not the CWD, so every caller works
# whether invoked from fsae_MPCTest/ or anywhere else.
# ---------------------------------------------------------------------------

_FSAE_MPCTEST = Path(__file__).resolve().parent.parent.parent
_FSDS_ROOT = _FSAE_MPCTEST.parent
_LAUNCH_ALL = _FSDS_ROOT / "ros2" / "launch_all.sh"
_SETTINGS_DIR = _FSAE_MPCTEST / "settings"

# Relative path (from a ROS tree's root) of each live parameter file. The
# same relative paths hold in fsae_planning, the fsds_simulator/ mirror and
# fsae_autonomous.
_RELATIVE_PATHS = {
    "mpc_params.py": Path("control/fsae_control/fsae_control/mpc/mpc_params.py"),
    "nmpc_params.py": Path("control/fsae_control/fsae_control/mpc/nmpc_params.py"),
    "fsae_params.yaml": Path("common/fsae_bringup/config/fsae_params.yaml"),
}

_LIVE_ROOT = _FSDS_ROOT / "ros2" / "src" / "fsae_planning"
_MIRROR_ROOT = _FSAE_MPCTEST / "fsds_simulator"
# fsae_autonomous has moved before, so a short list of observed locations
# is searched instead of hardcoding one. Add the new path here if it moves
# again (CLAUDE.md "Git layout").
_AUTONOMOUS_CANDIDATES = [
    _FSDS_ROOT / "fsae_autonomous",
    _FSDS_ROOT / "ros2_autonomous" / "src" / "fsae_autonomous",
]


def _find_autonomous_root() -> Path | None:
    for candidate in _AUTONOMOUS_CANDIDATES:
        if (candidate / ".git").is_dir():
            return candidate
    return None


# settings/ stores the core weights as index vectors rather than one
# constant per field. Each list gives the live dataclass field for every
# index; None marks an index with no live field (Q_diag[5:8] are always-0
# states, R_diag[1] is nominal only, superseded by R_A_ACCEL/R_A_BRAKE).
# Every other field maps to UPPER(field) in settings/.
_LIST_FIELD_MAP: dict[str, list[str | None]] = {
    "Q_diag": ["q_e_y", "q_e_yd", "q_e_psi", "q_r", "q_e_v", None, None, None],
    "R_diag": ["r_delta", None],
    "R_rate_diag": ["r_rate_delta", "r_rate_a"],
}

# ---------------------------------------------------------------------------
# Read/rewrite helpers (launch_all.sh, settings/*.py, fsae_params.yaml,
# mpc_params.py/nmpc_params.py).
# ---------------------------------------------------------------------------

def _var_pattern(name: str) -> re.Pattern:
    # \b after the name prevents NMPC_Q_E_Y's pattern from matching
    # NMPC_Q_E_YD's line (both start with the same prefix) -- \b sits
    # between the last name character and whitespace/'=', but NOT between
    # 'Y' and 'D' (both word characters), so the YD line never matches.
    return re.compile(rf"^(\s*{re.escape(name)}\b\s*=\s*)([^#\n]*?)(\s*(?:#.*)?)$",
                       re.MULTILINE)


def _settings_file_for(settings_dir: Path, name: str) -> Path | None:
    """Which settings/*.py file holds NAME's assignment, or None if no
    submodule has it. Searched fresh every call (not cached) since a hand
    edit or a concurrent session (see CLAUDE.md's "Concurrent sessions"
    note) can move a field between submodules between calls."""
    pattern = _var_pattern(name)
    for f in sorted(settings_dir.glob("*.py")):
        if pattern.search(f.read_text()):
            return f
    return None


def _read_var(path: Path, name: str) -> str | None:
    """Current raw right-hand-side text of NAME's assignment, or None if
    NAME's assignment line isn't found. `path` may be a single file
    (launch_all.sh) or a directory (settings/), in which case every
    settings/*.py file is searched -- a field's submodule isn't tracked
    anywhere else in this GUI, so this is the one place that resolves it."""
    if path.is_dir():
        f = _settings_file_for(path, name)
        if f is None:
            return None
        path = f
    text = path.read_text()
    m = _var_pattern(name).search(text)
    return m.group(2).strip() if m else None


def _rewrite_var(path: Path, name: str, new_value: str) -> bool:
    """Rewrite NAME's assignment to new_value, preserving the line's
    leading whitespace/alignment and any trailing inline comment. Returns
    False (no-op, file untouched) if NAME's assignment line isn't found.
    `path` may be a single file or a directory (settings/), see _read_var."""
    if path.is_dir():
        f = _settings_file_for(path, name)
        if f is None:
            return False
        path = f
    text = path.read_text()
    pattern = _var_pattern(name)
    if not pattern.search(text):
        return False
    new_text = pattern.sub(lambda m: f"{m.group(1)}{new_value}{m.group(3)}", text, count=1)
    path.write_text(new_text)
    return True


def _launch_arg_vars(launch_all_sh: Path) -> dict[str, str]:
    """{MPCParams/NMPCParams field: launch_all.sh shell variable}, parsed
    from that file's own `_append_mpc_arg <field> "$<VAR>"` lines, so the
    mapping cannot drift from the script that applies it."""
    text = launch_all_sh.read_text()
    return dict(re.findall(r'^_append_mpc_arg (\w+) "\$(\w+)"', text, re.MULTILINE))


def _shortlist_var_pattern(name: str) -> re.Pattern:
    """Like _var_pattern, but also matches a shortlist entry that is
    currently COMMENTED OUT (`# NAME=value  # ...`), which is how most of
    launch_all.sh's NMPC-only shortlist ships by default (see that file's
    own "commented-out overrides" convention, CLAUDE.md's "Single source
    of truth for MPC tuning"). Group 1 captures the optional leading
    `# ` so callers can toggle it on/off independently of the value."""
    return re.compile(
        rf"^(\s*# ?)?({re.escape(name)}\b\s*=\s*)([^#\n]*?)(\s*(?:#.*)?)$",
        re.MULTILINE)


def _read_shortlist_var(path: Path, name: str) -> tuple[bool, str] | None:
    """(enabled, value) for a shortlist var that may be commented out, or
    None if NAME's assignment line isn't found at all. enabled=False means
    the line is currently `# NAME=value`."""
    text = path.read_text()
    m = _shortlist_var_pattern(name).search(text)
    if m is None:
        return None
    return (m.group(1) is None, m.group(3).strip())


def _rewrite_shortlist_var(path: Path, name: str, enabled: bool, new_value: str) -> bool:
    """Set NAME's shortlist line to `NAME=new_value` (enabled) or
    `# NAME=new_value` (disabled), preserving alignment/trailing comment.
    Returns False (no-op) if NAME's line isn't found at all."""
    text = path.read_text()
    pattern = _shortlist_var_pattern(name)
    if not pattern.search(text):
        return False
    prefix = "" if enabled else "# "
    new_text = pattern.sub(lambda m: f"{prefix}{m.group(2)}{new_value}{m.group(4)}",
                            text, count=1)
    path.write_text(new_text)
    return True


def _yaml_field_pattern(name: str) -> re.Pattern:
    """Matches a `controller:` block field in fsae_params.yaml: 4-space
    indented `name: value    # comment`. \\b after the name prevents
    r_a_accel's pattern matching a field with the same prefix (there is no
    such collision today, but nmpc_q_e_y/nmpc_q_e_yd already burned this
    once for the shell/dataclass patterns above, so the same guard is
    applied here on the same reasoning). The indent is fixed at 4 spaces
    (this file's own convention, verified against every field under
    `controller:`), not \\s*, so this cannot match a same-named key that
    might exist under a different top-level node's ros__parameters block."""
    return re.compile(rf"^(    {re.escape(name)}\b:\s*)([^#\n]*?)(\s*(?:#.*)?)$",
                       re.MULTILINE)


def _read_yaml_field(path: Path, name: str) -> str | None:
    """Current raw value text of NAME's `controller:`-block YAML field, or
    None if NAME's line isn't found in path."""
    text = path.read_text()
    m = _yaml_field_pattern(name).search(text)
    return m.group(2).strip() if m else None


def _rewrite_yaml_field(path: Path, name: str, new_value: str) -> bool:
    """Rewrite NAME's `controller:`-block YAML field to new_value, preserving
    indentation and any trailing inline comment. Returns False (no-op) if
    NAME's line isn't found in path."""
    text = path.read_text()
    pattern = _yaml_field_pattern(name)
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
    working-tree state matters session-to-session (see CLAUDE.md).

    `path` may be the settings/ package directory instead of a single file
    (any of its *.py submodules could be the one a save actually writes
    to); each submodule gets its own file.py.bak, same recovery command
    per file as before."""
    if path in done:
        return
    if path.is_dir():
        for f in sorted(path.glob("*.py")):
            _backup_once(f, done)
        done.add(path)
        return
    shutil.copy(path, path.with_suffix(path.suffix + ".bak"))
    done.add(path)

