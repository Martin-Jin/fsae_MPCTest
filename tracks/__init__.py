"""
tracks/ — one directory per recorded track, holding that track's cone map and
both of its exported CSVs.

LAYOUT
------
    tracks/<track_name>/
        cone_map.json      the cone_recorder capture (the source of truth)
        speed_profile.csv  tuner.tools.export_speed_profile output (centreline + oracle speed)
        raceline.csv       tuner.tools.raceline_optimizer output (minimum-time line)
        centerline.csv     tuner.tools.raceline_optimizer --mode centerline output

A brand-new track name gets a date suffix automatically (`<name>_<YYYYmmdd>`,
see `dated_track_name()`), so two recordings on different days never collide
and nothing has to be renamed by hand. Re-recording an EXISTING track name
(refreshing its cone map in place — see `ros2/launch_all.sh`'s own comment on
this) keeps writing into the same directory rather than dating it again; only
first creation is dated.

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

DEFAULT TRACK IS THE NEWEST ONE, NOT A FIXED NAME
--------------------------------------------------
`DEFAULT_TRACK` used to be a hardcoded string (`"comp_test_map_3"`) that had
to be kept in sync by hand with `ros2/launch_all.sh`'s own `TRACK=` default —
an easy thing to forget after recording a new track. It is now `None`, and
every function that takes a track name treats `None` as "resolve
`newest_track()` right now" rather than baking in a fixed name at import
time. `newest_track()` picks the track whose `cone_map.json` has the newest
mtime — not the newest-LOOKING name — so it is correct even for a track
recorded before the dating scheme existed, or re-recorded (which refreshes
the mtime without changing the directory name).

WHERE THE DATA ACTUALLY LIVES
------------------------------
`TRACKS_DIR` resolves to `ros2/src/fsae_planning/tracks/` — committed data
INSIDE the `fsae_planning` repo, not inside this repo. This is deliberate:
`fsae_planning` + FSDS must be drivable with no `fsae_MPCTest` checkout at
all, which means the track data itself (not just the code that reads it) has
to ship with `fsae_planning`. `control.launch.py`/`sim.launch.py`'s
`map_path`/`path_map_path` defaults and `ros2/launch_all.sh`'s `TRACK=`
variable all point at this same directory.

This repo (`fsae_MPCTest`) is where NEW tracks get produced — record a lap,
then run `export_speed_profile`/`raceline_optimizer` — but the tools here
write into `fsae_planning`'s `tracks/` directly (see `TRACKS_DIR` below), so
there is no separate copy step for a checkout that has both repos side by
side. `fsae_planning` is a separate git repo with its own remote (see this
project's CLAUDE.md) — changes under `TRACKS_DIR` are local edits to that
checkout; committing/pushing them is a decision for that repo, not this one.

Every path helper here is filesystem-only (no scipy, no ROS, no numpy) so the
live ROS package could adopt it later without pulling in this repo's
dependency stack.
"""
import datetime
import os

TRACKS_DIR = os.path.abspath(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "ros2", "src", "fsae_planning", "tracks",
    )
)

CONE_MAP_NAME = "cone_map.json"
SPEED_PROFILE_NAME = "speed_profile.csv"
RACELINE_NAME = "raceline.csv"
#: Centreline variant of RACELINE_NAME, same 5-column format. A path that
#: stays on the geometric centre of the track instead of cutting corners --
#: slower, but its lateral error is directly readable as "how far off the
#: middle of the track am I", which a cornering-optimised raceline's is not.
CENTERLINE_NAME = "centerline.csv"

#: A sentinel, not a name: every function below that takes a track `name`
#: treats `None` as "call `newest_track()` now". Kept as a module constant
#: (rather than every function defaulting to `name=None` inline) so the
#: intent reads the same everywhere this is referenced.
DEFAULT_TRACK = None


def _resolve(name):
    """`None` -> the newest track; anything else -> unchanged."""
    if name is not None:
        return name
    newest = newest_track()
    if newest is None:
        raise ValueError(
            f"No tracks found under {TRACKS_DIR} (each needs a {CONE_MAP_NAME}"
            " at minimum). Record one, or pass an explicit track name/path."
        )
    return newest


def newest_track():
    """
    Name of the track whose `cone_map.json` has the newest mtime, or `None` if
    `TRACKS_DIR` holds no track at all.

    Deliberately mtime-based, not name-based: a track recorded before
    `dated_track_name()` existed has no date in its name and must still be
    selectable as "newest" the moment it is the most recently recorded one,
    and re-recording an existing track (refreshing its cone map in place)
    must make it newest again without renaming its directory.
    """
    names = list_tracks()
    if not names:
        return None
    return max(names, key=lambda n: os.path.getmtime(cone_map_path(n)))


def dated_track_name(base_name, when=None):
    """
    `<base_name>_<YYYYmmdd>`, the name a brand-new track directory is given.

    `when` is an explicit `datetime.date`/`datetime.datetime`, injectable for
    a caller that wants a deterministic name (tests, or a caller that already
    has its own timestamp); omitted, it reads the current local date. Only
    used for a track that does not exist yet — see the module docstring's
    "re-recording keeps the existing directory" rule; a caller re-recording
    an existing track passes that track's own (already-decided) name straight
    through instead of calling this again.
    """
    if when is None:
        when = datetime.date.today()
    return f"{base_name}_{when:%Y%m%d}"


def track_dir(name=DEFAULT_TRACK):
    """Absolute path to one track's directory (not required to exist yet)."""
    return os.path.join(TRACKS_DIR, _resolve(name))


def cone_map_path(name=DEFAULT_TRACK):
    return os.path.join(track_dir(name), CONE_MAP_NAME)


def speed_profile_path(name=DEFAULT_TRACK):
    return os.path.join(track_dir(name), SPEED_PROFILE_NAME)


def raceline_path(name=DEFAULT_TRACK):
    return os.path.join(track_dir(name), RACELINE_NAME)


def centerline_path(name=DEFAULT_TRACK):
    return os.path.join(track_dir(name), CENTERLINE_NAME)


def geometry_path(name=DEFAULT_TRACK):
    """
    The path-geometry file to drive/tune against: `centerline.csv` if the
    track has one, else `raceline.csv`, else `None` if the track has neither
    exported yet (only a cone map).

    This is the "newest export, preferring the centreline one" that
    `ros2/launch_all.sh`'s `PATH_CSV` default now resolves through — see that
    script's own comment before overriding it to force the raceline.
    """
    name = _resolve(name)
    c = centerline_path(name)
    if os.path.isfile(c):
        return c
    r = raceline_path(name)
    if os.path.isfile(r):
        return r
    return None


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

      - None                      -> the NEWEST track's cone map
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


def default_out_for(map_path, filename, allow_overwrite=True):
    """
    Where an export should land, given the cone map it was produced from.

    A map inside `tracks/<name>/` exports alongside itself, so the trio stays
    together with no output path to type. A map from anywhere else (a one-off
    capture, an explicit path) exports next to that file instead of being
    dragged into `tracks/` — an export belongs with its source, and quietly
    materialising a half-populated track directory would make `list_tracks()`
    advertise something that isn't a real track.

    `allow_overwrite=False` raises `FileExistsError` instead of returning a
    path that already has a file on it — this is the check a caller makes
    BEFORE writing (this function never touches the filesystem itself beyond
    the `os.path.isfile` check), so a re-export of the same track/kind either
    aborts with a clear message or has to be requested explicitly, instead of
    silently replacing a file another session might still be using. Default
    is `True` (the historical behaviour: always overwrite) because re-running
    an exporter after retuning the SAME track is the common case, not the
    exception -- opt in to the guard with `--no-overwrite` where a tool
    exposes it, don't flip the default.
    """
    out = os.path.join(os.path.dirname(os.path.abspath(map_path)), filename)
    if not allow_overwrite and os.path.isfile(out):
        raise FileExistsError(
            f"{out} already exists and --no-overwrite was given. Pass a "
            "different output path, or drop --no-overwrite to replace it."
        )
    return out
