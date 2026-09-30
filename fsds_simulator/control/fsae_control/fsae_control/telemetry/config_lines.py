"""
fsae_control/telemetry/config_lines.py — launch-configuration header

`build_config_lines` renders the entire launch-time configuration of a run as
`#` comment lines so one CSV reproduces the run it came from.
"""

import dataclasses


def build_config_lines(
    controller: str,
    launch_flags: dict | None = None,
    mpc_params=None,
    nmpc_params=None,
    nmpc_effective: dict | None = None,
) -> list[str]:
    """
    Build the `#`-commented run-configuration dump ControlLogger.
    set_config_lines() stores for `_write_score_header()` to write. Every
    line is returned ALREADY prefixed with `# ` (ControlLogger.
    set_config_lines() does not add its own prefix).

    Parameters
    ----------
    controller : str
        'stanley' | 'mpc' | 'mpc_standalone' -- whichever node built this.
    launch_flags : dict, optional
        Arbitrary key -> value pairs the CALLER already has from its own
        get_parameter() calls (map_path, path_map_path,
        use_precomputed_heading_profile, enable_dynamic_speed_cap,
        dynamic_cap_a_lat_max, dynamic_cap_safety, v_max, v_min, ...) --
        this function doesn't know or care what a node declares, it just
        formats whatever dict it's handed. Empty/None values are logged as
        such (not skipped), since "map_path was empty" (-> live planner
        mode) is exactly the kind of thing worth being able to read back.
    mpc_params : mpc_params.MPCParams, optional
        Dumped via dataclasses.asdict() -- every field, whatever they
        currently are. This is DELIBERATE: hand-listing field names here
        would silently go stale every time a weight is added, renamed or
        removed (as already happened once this session, see the
        corner_factor rewrite) -- asdict() cannot go stale, it reflects
        whatever MPCParams actually is at import time.
    nmpc_params : nmpc_params.NMPCParams, optional
        Same treatment, only meaningful when controller used_nmpc=True.
    nmpc_effective : dict, optional
        The NMPC's RESOLVED weights after its own `-1.0`-inherits-from-
        MPCParams logic (e.g. {'w_out': ctrl.w_out.tolist(), 'r_delta':
        ctrl.r_delta, ...} read straight off the constructed
        NMPCController) -- logged separately from nmpc_params's raw
        (possibly -1.0) override fields so a reader doesn't have to
        mentally re-run the inheritance to know what was ACTUALLY used.

    What this does NOT capture (read this before assuming the dump is
    complete): the adaptive-gain SCHEME itself -- e.g. today's
    `_corner_factor`/`_blend` continuous blend in mpc_core.py -- is CODE,
    not a parameter, and isn't reproducible from a config dump the way a
    numeric weight is. If the scheme changes (as it already has once this
    session), a config header from an OLDER run describes weights for a
    scheme that no longer exists. `mpc_core.py`'s own module/function
    docstrings are the authoritative description of the CURRENT scheme;
    this dump only ever tells you the NUMBERS that scheme was using.
    """
    lines: list[str] = [f'# controller={controller}']
    lines.append(f'# use_nmpc={int(bool(nmpc_effective))}')

    if launch_flags:
        for key, val in launch_flags.items():
            lines.append(f'# launch.{key}={val!r}')

    if mpc_params is not None:
        for key, val in dataclasses.asdict(mpc_params).items():
            lines.append(f'# mpc_params.{key}={val!r}')

    if nmpc_params is not None:
        for key, val in dataclasses.asdict(nmpc_params).items():
            lines.append(f'# nmpc_params.{key}={val!r}')

    if nmpc_effective:
        for key, val in nmpc_effective.items():
            lines.append(f'# nmpc_effective.{key}={val!r}')

    return lines
