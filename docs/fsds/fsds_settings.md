# FSDS/Live Settings

**This doc covers the FSDS/live settings surface only**: how MPC/NMPC
tuning values and feature flags are configured on the ROS 2 side. For the
offline (`fsae_MPCTest`) equivalent, `settings.py`, see
[docs/architecture.md](../architecture.md)'s "Configuring the Project" and
"Configuring the Vehicle" sections, and [docs/tuning.md](../tuning.md) for
what each weight/flag does. For what "FSDS" vs "offline" mean, see
[docs/reference/simulator_glossary.md](../reference/simulator_glossary.md).

The two sides are kept numerically identical by hand, not by a shared
import (`fsae_MPCTest` cannot import from `fsae_planning`, or vice versa).
See `CLAUDE.md`'s "Single source of truth for MPC tuning, per side" section
for the standing rule, and
[offline_live_parity.md](../reference/offline_live_parity.md) for the
field-by-field mapping table. This doc only covers the live side's own
mechanics: where its settings live and how they reach the running
controller.

## The three places a live setting can be set, and how they relate

Every MPC/NMPC weight, gain, and feature flag (~56 `MPCParams` fields +
~34 `NMPCParams` fields) is declared **once**, as dataclass fields with
metadata (`unit`, `desc`, which controller it applies to), and everything
else is generated from that, so the value can't drift out of sync with its
own launch arg or YAML default:

1. **`MPCParams`** (`ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc/mpc_params.py`)
   and **`NMPCParams`** (`.../mpc/nmpc_params.py`, a separate sibling
   dataclass, not a subclass) are the actual source of truth. Each field
   carries a default value and metadata directly on it, e.g.:

   ```python
   q_e_y: float = field(default=6.4, metadata={
       "unit": "1/m^2",
       "desc": "lateral deviation from path centreline",
       "controller": "both",
   })
   ```

   `declare_mpc_params(node)` / `declare_nmpc_params(node)` declare every
   field as a ROS 2 parameter on the controller node (defaulting to
   `DEFAULT_MPC_PARAMS`/`DEFAULT_NMPC_PARAMS`), and
   `mpc_params_from_node(node)` / `nmpc_params_from_node(node)` read them
   back into a fresh `MPCParams`/`NMPCParams` instance at node-construction
   time, which is what `MPCController`/`NMPCController` actually receive.

2. **`common/fsae_bringup/config/fsae_params.yaml`**'s `controller:` block
   lists every field with the *same* default as the dataclass, under the
   comment "single source of truth for tunables." This is the YAML a
   deployment edits to change a default without touching code.

3. **`control.launch.py`/`sim.launch.py`** generate one `DeclareLaunchArgument`
   per field directly from `MPC_PARAM_FIELDS`/`NMPC_PARAM_FIELDS` (the
   dataclasses' own field-metadata tuples), via a small helper applied in a
   list comprehension. **Launch args are not hand-written**: adding a field
   to `MPCParams`/`NMPCParams` is enough for it to show up as a launch arg
   automatically, so the dataclass, the YAML defaults, and the launch args
   can't independently drift out of field-name sync (their *values* can
   still differ, since YAML/launch args are meant to override the
   dataclass default; that's the point of having them).

**Effective value at runtime, in override order**: dataclass default →
`fsae_params.yaml` → a launch arg (either passed directly to `ros2 launch`,
or forwarded by `ros2/launch_all.sh`'s shortlist below). The dataclass
default alone is not necessarily what's running; check the launch args
actually in effect for a given run before assuming a field's dataclass
default is live.

## `ros2/launch_all.sh`'s shortlist

`ros2/launch_all.sh` is the day-to-day launch script (see
[fsds_integration_guide.md](fsds_integration_guide.md)), and it carries a
small, commented-out-by-default shortlist of the fields most commonly
retuned interactively, forwarded via a `_append_mpc_arg field value` helper
that only adds `field:=value` to the launch command when the shell
variable is actually set. An untouched shortlist changes nothing; the
dataclass/YAML defaults still apply.

Two shortlist blocks exist:

- **Top-of-file basics**: `CONTROLLER`, `STANDALONE_OUTPUT`, `V_MAX`,
  `V_MIN`, and **`USE_NMPC`**, active (uncommented) by default rather than
  left at the dataclass's own default. This is the file to check before
  assuming which controller (LTV-QP vs. NMPC) or speed cap is actually
  running on a given launch, the dataclass default alone does not tell
  you.
- **"MPC tuning shortlist"**, further down: a longer commented-out list
  covering the weights/gains most likely to be tuned interactively
  (`MPC_Q_E_Y`, `MPC_Q_E_PSI`, `MPC_R_DELTA`, `MPC_R_A_ACCEL`/`_BRAKE`,
  `MPC_SPEED_TARGET_DEFICIT_MAX`, adaptive-gain and corner-factor fields,
  dynamic speed cap), plus a large NMPC-specific shortlist (horizon, SQP
  iteration count, corner-factor/rrate-zone/rjerk fields, progress term).
  A handful of these are also left active rather than commented out; **read
  the script directly for the current state** rather than trusting a
  cached description, this shortlist changes as tuning continues.

## Perception feeding the live planner, at a glance

The live stack's `sim_perception` node (an FOV/box/radius filter over
FSDS's cone ground truth) publishes `left_track`/`right_track`,
`cone_detection`, and `car_position` on separate timers; one of
`centerline_planner`/`skidpad_planner` (selected via the `planner` launch
arg) turns those into a published centreline the controller tracks. This
is a live ROS 2 node graph, not a Python function call, so its behaviour
(publish rates, FOV limits) is fixed by the running nodes' own parameters
in `fsae_params.yaml`, not by anything in this doc.

The offline side has its own reimplementation of this same idea
(`SimPerception`/`SimPlanner` in `sim/sim_track.py`, gated by
`USE_PLANNER`) built specifically to mirror this live behaviour closely
enough that a bug reproduced offline is a real perception/planning bug,
not a simulator artifact. See
[architecture.md's "Simulated Perception and Planning"](../architecture.md#simulated-perception-and-planning-use_planner)
section for how that offline reimplementation works; it is not covered
further here since this doc is FSDS/live-only.
