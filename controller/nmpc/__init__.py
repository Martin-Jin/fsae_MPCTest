"""
controller/nmpc/ — Nonlinear MPC (NMPC), offline counterpart of
the live ROS 2 side's `fsae_control.nmpc_core.NMPCController`.

PURPOSE
-------
`controller/lmpc/solve.py`'s `solve_mpc()` is a linear time-varying MPC: its
prediction model (`model/bicycle_model.py`) is the bicycle model in ERROR
coordinates with the reference path's own rotation entirely absent from it
(`e_psi_dot` = yaw rate only, never `r - kappa(s)*s_dot`). With the car
exactly on-line and on-heading approaching a corner, that model's own N-step
rollout predicts staying at zero forever — no cost weighting can produce
turn-in before real tracking error exists. See `model/bicycle_model.py`'s own
docstring and `docs/junior_project_mpc_docs.md` §4.2 for the plain-language
version, and `late_turn_in_investigation.md` (live repo) Parts 1-15 for the
full investigation this is downstream of.

This package is a SECOND controller that closes that gap: a Frenet-frame
(curvilinear-coordinate) nonlinear model, where arc length `s` is a STATE and
the path's curvature `kappa(s)` is looked up from it directly, so a bend
ahead is part of the dynamics rather than reweighted cost. Solved by
Gauss-Newton SQP (repeated re-linearisation + a condensed dense QP per
iteration, via OSQP), not one convex QP.

RELATIONSHIP TO THE LIVE SIDE
------------------------------
This is a faithful, independent PORT of `nmpc_core.py`'s `NMPCController` —
same model, same SQP/condensing/OSQP scheme, same variable/function names
where they carry over — NOT an import (`fsae_MPCTest` cannot import from the
live `fsae_planning` checkout and vice versa; CLAUDE.md's standing
"no settings.py-on-the-car" rule, from the other direction here). Kept
numerically identical BY HAND, the same discipline as every other
live/offline pair in this project (Q_diag/R_diag/R_rate_diag, etc.) — see
`settings.py`'s NMPC_* constants and `docs/tuning.md`'s NMPC section for the
field-by-field mapping this needs to be kept in sync with if either side's
model or solver changes.

Two differences from the live module, both because this is the offline side:
  - Vehicle constants (lf, lr, m, Iz, Cf, Cr, tau_delta, tau_a) are read
    directly from the `VehicleParams` instance already passed around this
    repo (the SAME source `model/bicycle_model.py`'s linear model and the
    24-state nonlinear plant both use), rather than a hardcoded copy — this
    repo has no "no cross-import" constraint against its own `model/` package.
  - Cost weights are NOT read from a dataclass. `sim/rollout/core.py`'s
    `run_core_rollout()` already receives the CURRENT weight set (whether
    from `settings.py` or a CMA-ES tuning candidate) as `Q`/`R`/`R_rate`
    arrays; `NMPCController` here is constructed with those same arrays (plus
    the NMPC-only override scalars from `settings.py`) so a tuner sweep
    reaches the NMPC's weights exactly the way it reaches the LTV-QP's.

USED BY
-------
  sim/rollout/core.py — run_core_rollout(), when settings.USE_NMPC is True.
    Constructed once per rollout (outside the step loop, so its warm start
    persists across ticks exactly like the LTV path's u_prev/command_queue).

DOES NOT USE
------------
  controller/lmpc/solve.py, model/bicycle_model.py (this package's own model
  replaces both when active), gui/simulation.py (imported from
  rollout_core.py only, same reasoning as controller/lmpc/solve.py's own
  "DOES NOT USE" note).
MODULE MAP (live `nmpc_core.py` is one file; this split is offline-only)
----------
  layout.py           IDX_*/NX/NU/NH_* layout, FD step sizes, _DENOM_FLOOR, _wrap
  reference.py        PathReference
  dynamics.py         _Plant, _tyre_forces, _f, _f_scalar, _step_scalar, _step
  outputs.py          _outputs (cost residual rows)
  weight_schedule.py  _rrate_zone_scale, _rrate_stage_ramp
  solver.py           _csc_pattern, NMPCController
To diff against the live module, compare by function name; the order above
is the order the live file defines them in.
"""

from controller.nmpc.layout import (  # noqa: F401
    IDX_S, IDX_EY, IDX_EPSI, IDX_VX, IDX_VY, IDX_R, IDX_DELTA, IDX_A,
    NX, NU, NH_TRACKING, NH_PROGRESS, NH_FRICTION, _wrap,
)
from controller.nmpc.reference import PathReference  # noqa: F401
from controller.nmpc.dynamics import (  # noqa: F401
    _Plant, _tyre_forces, _f, _f_scalar, _step_scalar, _step,
)
from controller.nmpc.outputs import _outputs  # noqa: F401
from controller.nmpc.weight_schedule import (  # noqa: F401
    _rrate_zone_scale, _rrate_stage_ramp,
)
from controller.nmpc.solver import NMPCController  # noqa: F401
