"""
tuner/tools/param_report.py — which MPC/NMPC value actually runs, per repo,
and where the repos disagree. Read-only: never writes any file.

Run from fsae_MPCTest/:

    python3 -m tuner.tools.param_report                  # flagged rows only
    python3 -m tuner.tools.param_report --all            # every field
    python3 -m tuner.tools.param_report --field rrate    # name filter
    python3 -m tuner.tools.param_report --prod-launch nmpc_gokart
    python3 -m tuner.tools.param_report --check          # exit 1 on a parity break

WHY THIS EXISTS
---------------
One MPCParams/NMPCParams field can be set in up to three layers per ROS
tree (a launch argument, fsae_params.yaml, the dataclass default), plus a
hand-kept offline constant in settings/. Reading any one file can give the
wrong answer. This resolves each tree's own precedence and prints the
value that would run, with the layer it came from.

PRECEDENCE, AS THE LAUNCH FILES ACTUALLY APPLY IT
--------------------------------------------------
launch_ros applies a Node's `parameters=[...]` list in order, later entries
winning. fsae_planning's control.launch.py (and fsae_autonomous's) pass
`[config, {**every_field: LaunchConfiguration(field)}]`, and each of those
launch arguments defaults to the dataclass default. So for a node started
through such a launch file:
    launch argument (launch_all.sh)  >  dataclass default  >  YAML
i.e. the YAML value for a dataclass field never reaches the node. A launch
file that passes only a literal dict (nmpc_gokart, nmpc_shadow) does not
shadow the YAML, and there the order is literal > YAML > dataclass. This is
detected per launch file by parsing it (see _node_params), not assumed.

COLUMNS / FLAGS
---------------
  live     fsae_planning, as launched by ros2/launch_all.sh
  mirror   fsae_MPCTest/fsds_simulator/ files (no launch_all.sh of its own)
  offline  fsae_MPCTest/settings/ constant
  prod     fsae_autonomous, as launched by --prod-launch (default control)

  OFFLINE  offline constant != live dataclass default, or no offline
           constant exists (parity break, --check fails). Fields that are
           live-only by design (_NO_OFFLINE) are never flagged.
  MIRROR   mirror dataclass/YAML != live files  (--check fails)
  LAUNCH   launch_all.sh overrides the field to a value other than the
           dataclass default, so a live run differs from what the offline
           constant (which tracks the dataclass) tunes against.
  SHADOW   the YAML sets a value other than the dataclass default but the
           launch file overrides it (dead YAML entry, see PRECEDENCE)
  PROD     production differs from live, or the field is missing there.
           Informational: production is maintained separately and may
           differ on purpose. `UNV` marks a field whose production source
           or YAML carries the UNVERIFIED token.

Only MPCParams/NMPCParams fields are covered. Stanley gains and node-level
parameters (v_max, map_path, ...) are not dataclass fields and are out of
scope.
"""
from __future__ import annotations

import argparse
import ast
import importlib.util
import math
import re
import sys
from pathlib import Path

import yaml

from tuner.tools.param_files import (
    _FSAE_MPCTEST, _LAUNCH_ALL, _LIST_FIELD_MAP, _LIVE_ROOT, _MIRROR_ROOT,
    _RELATIVE_PATHS, _find_autonomous_root, _launch_arg_vars, _read_var,
)

_LAUNCH_DIR = Path("common/fsae_bringup/launch")

# {field: (settings vector, index)}, inverted from the shared map. Every
# other field maps to UPPER(field) or NMPC_<UPPER(field)>.
_LIST_FIELDS = {
    field: (name, i)
    for name, fields_ in _LIST_FIELD_MAP.items()
    for i, field in enumerate(fields_) if field is not None
}
# Offline values that live outside settings/ or under another name.
_OFFLINE_ALIASES = {
    "ref_heading_rise_rate_deg_s": ("settings", "REF_HEADING_RISE_RATE"),
    "predict_epsi_clip": ("sim.rollout.delay", "_PREDICT_EPSI_CLIP"),
}
# Fields with no offline counterpart by design, per
# docs/reference/offline_live_parity.md "Fields with no matched offline
# constant". Shown as "live-only" and never flagged OFFLINE.
_NO_OFFLINE = {
    "anti_hunt_boost_max", "nmpc_anti_hunt_boost_max",
    "delay_compensation_enabled", "max_delay_compensation_steps",
    "min_delay_compensation_steps",
    "pose_age_lp_alpha", "n_delay_hysteresis",
}

_GROUP_TITLES = {
    "both": "Shared by LTV-QP and NMPC",
    "ltv_qp_only": "LTV-QP (lmpc) only",
    "nmpc_only": "NMPC only",
    None: "Other / untagged",
}


# --- value helpers --------------------------------------------------------

def _norm(v):
    """Bring a value from any source (Python, YAML, shell text) to one
    comparable type: bool, float, or str."""
    if isinstance(v, bool) or v is None:
        return v
    if isinstance(v, (int, float)):
        return float(v)
    s = str(v).strip().strip("'\"")
    if s.lower() in ("true", "false"):
        return s.lower() == "true"
    try:
        return float(s)
    except ValueError:
        return s


def _same(a, b) -> bool:
    a, b = _norm(a), _norm(b)
    if isinstance(a, float) and isinstance(b, float) and not isinstance(a, bool) \
            and not isinstance(b, bool):
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12)
    return a == b


def _fmt(v) -> str:
    v = _norm(v)
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return f"{v:.6g}"
    return str(v)


# --- sources --------------------------------------------------------------

def _load_dataclass_fields(path: Path, attr: str) -> dict:
    """{field: (default, metadata)} from a standalone *_params.py file. These
    files import only dataclasses/math, so they load by path without the
    ROS package on sys.path."""
    if not path.is_file():
        return {}
    name = f"_param_report_{abs(hash(str(path)))}"
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod  # dataclass() looks the module up while building
    spec.loader.exec_module(mod)
    return {n: (d, m) for n, d, m in getattr(mod, attr)}


def _load_yaml_block(path: Path, node: str) -> dict:
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    return (data.get(node) or {}).get("ros__parameters") or {}


def _launch_all_overrides(path: Path) -> dict:
    """{field: value} for every `_append_mpc_arg <field> "$VAR"` whose VAR has
    a non-empty, uncommented assignment. Commented shortlist lines pass
    nothing, as in launch_all.sh. Same helpers the GUI's Settings tab uses
    to find a pinned value."""
    if not path.is_file():
        return {}
    out = {}
    for field, var in _launch_arg_vars(path).items():
        value = _read_var(path, var)
        if value and value.strip("'\""):
            out[field] = value
    return out


def _node_params(launch_file: Path, node_name: str) -> tuple[bool, dict]:
    """(passes_every_field, {field: literal value}) for the Node(name=node_name)
    in launch_file. passes_every_field is True when the parameters dict
    contains a `**something` unpack, which in these launch files is always
    the generated {field: LaunchConfiguration(field)} map, and means the
    dataclass default (the launch argument's default) beats the YAML.
    Literal values given via float_arg('x')/LaunchConfiguration('x') resolve
    to that launch argument's declared default_value."""
    if not launch_file.is_file():
        return False, {}
    tree = ast.parse(launch_file.read_text())
    declared = {}
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        fname = getattr(call.func, "id", getattr(call.func, "attr", ""))
        if fname == "DeclareLaunchArgument" and call.args \
                and isinstance(call.args[0], ast.Constant):
            for kw in call.keywords:
                if kw.arg == "default_value" and isinstance(kw.value, ast.Constant):
                    declared[call.args[0].value] = kw.value.value

    def resolve(node):
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.Call) and node.args and isinstance(node.args[0], ast.Constant):
            fname = getattr(node.func, "id", getattr(node.func, "attr", ""))
            if fname in ("float_arg", "bool_arg", "LaunchConfiguration"):
                return declared.get(node.args[0].value)
        return None

    passes_all, literals = False, {}
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        if getattr(call.func, "id", "") != "Node":
            continue
        kws = {kw.arg: kw.value for kw in call.keywords}
        name = kws.get("name")
        if not (isinstance(name, ast.Constant) and name.value == node_name):
            continue
        params = kws.get("parameters")
        for elt in getattr(params, "elts", []):
            if not isinstance(elt, ast.Dict):
                continue
            for k, v in zip(elt.keys, elt.values):
                if k is None:
                    passes_all = True
                elif isinstance(k, ast.Constant):
                    val = resolve(v)
                    if val is not None:
                        literals[k.value] = val
    return passes_all, literals


def _unverified_fields(params_paths: list[Path], yaml_path: Path) -> set[str]:
    """Fields whose dataclass definition (with the comment lines directly
    above it) or YAML line (with its comment block) contains UNVERIFIED."""
    hits = set()
    for path in params_paths:
        if not path.is_file():
            continue
        lines = path.read_text().splitlines()
        for i, line in enumerate(lines):
            m = re.match(r"\s*(\w+)\s*:\s*\w+\s*=\s*field\(", line)
            if not m:
                continue
            j = i
            while j > 0 and lines[j - 1].strip().startswith("#"):
                j -= 1
            k, depth = i, 0
            while k < len(lines):
                depth += lines[k].count("(") - lines[k].count(")")
                if depth <= 0:
                    break
                k += 1
            if "UNVERIFIED" in "\n".join(lines[j:k + 1]):
                hits.add(m.group(1))
    if yaml_path.is_file():
        lines = yaml_path.read_text().splitlines()
        for i, line in enumerate(lines):
            m = re.match(r"\s+(\w+):", line)
            if not m:
                continue
            j = i
            while j > 0 and lines[j - 1].strip().startswith("#"):
                j -= 1
            if "UNVERIFIED" in "\n".join(lines[j:i + 1]):
                hits.add(m.group(1))
    return hits


def _offline_value(settings, field: str):
    if field in _OFFLINE_ALIASES:
        module, attr = _OFFLINE_ALIASES[field]
        return attr, getattr(importlib.import_module(module), attr)
    if field in _LIST_FIELDS:
        name, idx = _LIST_FIELDS[field]
        return f"{name}[{idx}]", getattr(settings, name)[idx]
    for name in (field.upper(), "NMPC_" + field.upper()):
        if hasattr(settings, name):
            return name, getattr(settings, name)
    return None, None


# --- resolution -----------------------------------------------------------

class _Tree:
    """One ROS tree's dataclasses + YAML + chosen launch file."""

    def __init__(self, root: Path | None, yaml_node: str, launch: str, node: str,
                 cli_overrides: dict | None = None):
        self.ok = root is not None and root.is_dir()
        self.dc, self.yaml, self.passes_all, self.literals = {}, {}, False, {}
        self.cli = cli_overrides or {}
        if not self.ok:
            return
        for attr, key in (("MPC_PARAM_FIELDS", "mpc_params.py"),
                          ("NMPC_PARAM_FIELDS", "nmpc_params.py")):
            self.dc.update(_load_dataclass_fields(root / _RELATIVE_PATHS[key], attr))
        self.yaml = _load_yaml_block(root / _RELATIVE_PATHS["fsae_params.yaml"], yaml_node)
        self.passes_all, self.literals = _node_params(
            root / _LAUNCH_DIR / f"{launch}.launch.py", node)

    def effective(self, field: str):
        """(value, layer) as the node would receive it, or (None, 'missing')."""
        if field not in self.dc:
            return None, "missing"
        if field in self.cli:
            return self.cli[field], "launch_all"
        if field in self.literals:
            return self.literals[field], "launch"
        if self.passes_all:
            return self.dc[field][0], "dataclass"
        if field in self.yaml:
            return self.yaml[field], "yaml"
        return self.dc[field][0], "dataclass"

    def shadowed_yaml(self, field: str) -> bool:
        """True when the YAML sets a value that differs from the dataclass
        default but the launch file's generated launch argument (whose
        default IS the dataclass default) overrides it, so the YAML edit has
        no effect on the node."""
        if field not in self.yaml or field not in self.dc or not self.passes_all:
            return False
        return not _same(self.yaml[field], self.dc[field][0])


def build_rows(prod_launch: str) -> list[dict]:
    sys.path.insert(0, str(_FSAE_MPCTEST))
    import settings  # noqa: E402  (needs fsae_MPCTest on sys.path)

    live = _Tree(_LIVE_ROOT, "controller", "control", "controller",
                 _launch_all_overrides(_LAUNCH_ALL))
    mirror = _Tree(_MIRROR_ROOT, "controller", "control", "controller")
    prod_root = _find_autonomous_root()
    prod = _Tree(prod_root, "nmpc_controller", prod_launch, "nmpc_controller")
    unv = set()
    if prod.ok:
        unv = _unverified_fields(
            [prod_root / _RELATIVE_PATHS[k] for k in ("mpc_params.py", "nmpc_params.py")],
            prod_root / _RELATIVE_PATHS["fsae_params.yaml"])

    rows = []
    for field in sorted(set(live.dc) | set(prod.dc), key=lambda f: (
            f not in live.dc, list(live.dc).index(f) if f in live.dc else 0, f)):
        meta = (live.dc.get(field) or prod.dc.get(field))[1]
        lv, ll = live.effective(field)
        mv = mirror.dc.get(field, (None,))[0]
        oname, ov = _offline_value(settings, field)
        pv, pl = prod.effective(field)
        flags = []
        if field in live.dc and field not in _NO_OFFLINE \
                and (oname is None or not _same(ov, live.dc[field][0])):
            flags.append("OFFLINE")
        if ll == "launch_all" and not _same(lv, live.dc[field][0]):
            flags.append("LAUNCH")
        if field in live.dc and (field not in mirror.dc
                                 or not _same(mv, live.dc[field][0])
                                 or not _same(mirror.yaml.get(field), live.yaml.get(field))):
            flags.append("MIRROR")
        if live.shadowed_yaml(field) or prod.shadowed_yaml(field):
            flags.append("SHADOW")
        if prod.ok and (pl == "missing" or field not in live.dc or not _same(pv, lv)):
            flags.append("PROD")
        rows.append(dict(
            field=field, group=meta.get("controller"), flags=flags,
            live=(lv, ll), mirror=mv, offline=(oname, ov), prod=(pv, pl),
            unverified=field in unv,
            yaml_live=live.yaml.get(field), yaml_prod=prod.yaml.get(field),
        ))
    return rows, live, prod


def _print(rows, live, prod, prod_launch, show_all):
    print(f"live: launch_all.sh + control.launch.py "
          f"(YAML {'shadowed by dataclass' if live.passes_all else 'honoured'}); "
          f"prod: {prod_launch}.launch.py "
          f"(YAML {'shadowed by dataclass' if prod.passes_all else 'honoured'})"
          + ("" if prod.ok else "  [fsae_autonomous not found]"))
    hdr = f"{'field':34} {'live (layer)':24} {'mirror':10} {'offline':22} {'prod (layer)':26} flags"
    by_group: dict = {}
    for r in rows:
        by_group.setdefault(r["group"], []).append(r)
    for group in ("both", "ltv_qp_only", "nmpc_only", None):
        shown = [r for r in by_group.get(group, []) if show_all or r["flags"]]
        if not shown:
            continue
        print(f"\n== {_GROUP_TITLES.get(group, group)} ==\n{hdr}")
        for r in shown:
            lv, ll = r["live"]
            oname, ov = r["offline"]
            pv, pl = r["prod"]
            live_s = f"{_fmt(lv)} ({ll})" if ll != "missing" else "missing"
            prod_s = f"{_fmt(pv)} ({pl})" if pl != "missing" else "missing"
            if r["unverified"]:
                prod_s += " UNV"
            if r["field"] in _NO_OFFLINE:
                off_s = "live-only"
            elif oname:
                off_s = _fmt(ov) + (f" {oname}" if oname.endswith("]") else "")
            else:
                off_s = "no constant"
            print(f"{r['field']:34} {live_s:24} {_fmt(r['mirror']):10} {off_s:22} "
                  f"{prod_s:26} {','.join(r['flags'])}")
            for side, yv, tree in (("live", r["yaml_live"], live), ("prod", r["yaml_prod"], prod)):
                if tree.shadowed_yaml(r["field"]):
                    print(f"{'':34}   {side} YAML says {_fmt(yv)}, never reaches the node")

    counts = {k: sum(k in r["flags"] for r in rows) for k in ("OFFLINE", "MIRROR", "LAUNCH", "SHADOW", "PROD")}
    print("\nSummary: " + ", ".join(f"{k} {v}" for k, v in counts.items())
          + f"  ({len(rows)} fields)")
    return counts


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--all", action="store_true", help="show unflagged fields too")
    ap.add_argument("--field", help="only fields whose name contains this text")
    ap.add_argument("--prod-launch", default="control",
                    help="fsae_autonomous launch file (without .launch.py) whose "
                         "nmpc_controller node to resolve; default: control")
    ap.add_argument("--check", action="store_true",
                    help="exit 1 if any OFFLINE or MIRROR flag is raised")
    args = ap.parse_args()

    rows, live, prod = build_rows(args.prod_launch)
    if args.field:
        rows = [r for r in rows if args.field in r["field"]]
    counts = _print(rows, live, prod, args.prod_launch, args.all or bool(args.field))
    if args.check and (counts["OFFLINE"] or counts["MIRROR"]):
        sys.exit(1)


if __name__ == "__main__":
    main()
