"""
tracks/ — one directory per recorded track, holding that track's cone map and
both of its exported CSVs.

LAYOUT
------
    tracks/<track_name>/
        cone_map.json      the cone_recorder capture (the source of truth)
        speed_profile.csv  tuner.export_speed_profile output (centreline + oracle speed)
        raceline.csv       tuner.raceline_optimizer output (minimum-time line)

WHY THIS EXISTS
---------------
Before this, a recording landed at a single repo-root `cone_map.json` that the
next recording overwrote, and both exports were loose files in `tuner/`
(`speed_profile_export.csv`, `raceline_export.csv`) sitting among 30+ Python
modules. Nothing in a filename said which map or which tuning run produced it
— the source map was recorded only in a `# source_map=` comment inside the
file — and switching the car to a different track meant passing two absolute
paths on the command line or editing a hardcoded default in `sim.launch.py`.

Grouping by track makes the set self-describing: one directory is everything
needed to drive one track, and `launch_all.sh`'s `TRACK=` variable selects it
by name.

Every path helper here is filesystem-only (no scipy, no ROS, no numpy) so the
live ROS package could adopt it later without pulling in this repo's
dependency stack.
"""
import os

TRACKS_DIR = os.path.dirname(os.path.abspath(__file__))

CONE_MAP_NAME = "cone_map.json"
SPEED_PROFILE_NAME = "speed_profile.csv"
RACELINE_NAME = "raceline.csv"

#: Track used when a tool is run with no track argument. Matches the
#: `TRACK=` default in `ros2/launch_all.sh` — keep the two in sync so the
#: offline tools and the car operate on the same track by default.
DEFAULT_TRACK = "comp_test_map_3"


def track_dir(name=DEFAULT_TRACK):
    """Absolute path to one track's directory (not required to exist yet)."""
    return os.path.join(TRACKS_DIR, name)


def cone_map_path(name=DEFAULT_TRACK):
    return os.path.join(track_dir(name), CONE_MAP_NAME)


def speed_profile_path(name=DEFAULT_TRACK):
    return os.path.join(track_dir(name), SPEED_PROFILE_NAME)


def raceline_path(name=DEFAULT_TRACK):
    return os.path.join(track_dir(name), RACELINE_NAME)


def list_tracks():
    """
    Names of every track directory that actually holds a cone map, sorted.

    A directory without a `cone_map.json` is skipped rather than reported:
    it can't be re-exported from, so offering it as a choice would only
    produce a confusing failure one step later.
    """
    if not os.path.isdir(TRACKS_DIR):
        return []
    return sorted(
        n for n in os.listdir(TRACKS_DIR)
        if os.path.isfile(os.path.join(TRACKS_DIR, n, CONE_MAP_NAME))
    )


def resolve_map_arg(arg, default_track=DEFAULT_TRACK):
    """
    Accept either a track NAME or an explicit path to a cone map, and return
    the cone-map path to load.

    Both forms are supported because the tools predate `tracks/` and were
    documented (and scripted) as taking a path. Disambiguation is by
    filesystem check, not by string shape, so a track that happens to be named
    like a path still resolves correctly:

      - None                      -> the default track's cone map
      - an existing file          -> that file, used as-is
      - a name in tracks/         -> that track's cone_map.json
      - an existing directory     -> cone_map.json inside it
      - anything else             -> ValueError naming the available tracks

    Raising (rather than falling back to the default) on an unknown name is
    deliberate: silently exporting the wrong track is far more expensive to
    notice than a failed command, because the mistake only shows up as odd
    behaviour on the car several steps later.
    """
    if arg is None:
        return cone_map_path(default_track)
    if os.path.isfile(arg):
        return arg
    candidate = cone_map_path(arg)
    if os.path.isfile(candidate):
        return candidate
    if os.path.isdir(arg):
        inner = os.path.join(arg, CONE_MAP_NAME)
        if os.path.isfile(inner):
            return inner
    available = ", ".join(list_tracks()) or "(none)"
    raise ValueError(
        f"No cone map for {arg!r}: not an existing file, and not a track in "
        f"{TRACKS_DIR}. Available tracks: {available}"
    )


def default_out_for(map_path, filename):
    """
    Where an export should land, given the cone map it was produced from.

    A map inside `tracks/<name>/` exports alongside itself, so the trio stays
    together with no output path to type. A map from anywhere else (a one-off
    capture, an explicit path) exports next to that file instead of being
    dragged into `tracks/` — an export belongs with its source, and quietly
    materialising a half-populated track directory would make `list_tracks()`
    advertise something that isn't a real track.
    """
    return os.path.join(os.path.dirname(os.path.abspath(map_path)), filename)
