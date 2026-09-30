"""
fsae_control/lmpc/predict.py — delay compensation rollforward

`predict_ahead` rolls the measured error state through the commands already
issued but not yet reflected in the pose. The comment blocks below record why
the rollforward depth is capped and why n_delay is filtered.
"""

import numpy as np

# ── Reference-heading rate limit ─────────────────────────────────────────────
# Mirrors fsae_MPCTest/sim/rollout/core.py's REF_HEADING_RATE_LIMIT_ENABLED /
# REF_HEADING_RISE_RATE / _rate_limit_ref_psi — keep all three in sync.
# Caps how fast the tracked reference heading (path_yaw in _error_state) may
# change per tick, symmetric in both directions — unlike the speed-target
# rise limiter, there is no "always safe" direction for a heading reference.
# Disabled: holding the reference back during turn-in leaves a larger
# heading deficit to claw back later, which makes steering saturation worse.
# Re-test against a synthetic slalom path offline before re-enabling.
# Moved to MPCParams.ref_heading_rate_limit_enabled /
# .ref_heading_rise_rate_deg_s (mpc_params.py) — see those fields for the
# current values.

# ── Delay compensation ──────────────────────────────────────────────────────
# Real delay (perception + planning + control + actuation latency) is
# unknown and time-varying, unlike fsae_MPCTest's simulator-only fixed
# DELAY_STEPS. compute() is instead told how OLD the pose it's solving
# against is (pose_age_s, measured from the pose message's own timestamp —
# see mpc_controller.py/_pose_cb) and converts that into a step count itself.
# See predict_ahead() below for the same small-angle-clip rollforward
# validated in fsae_MPCTest/sim/rollout/core.py.
# The rollforward depth (n_delay) is capped at a small value because
# predict_ahead() iterates the linear model n_delay times with NO
# ground-truth correction, so pose noise compounds through every extra
# matrix multiply and the QP faithfully tracks the resulting jitter into the
# steering command. This is the same mechanism the DELAY_COMPENSATION_ENABLED
# note below records (elevated reversal rate in high-n_delay windows): a
# deep rollforward can turn measurement noise into steering thrash that
# consumes the whole corner-approach phase (steer swinging +-5-10 deg per
# tick at e_y/e_psi ~0), causing late turn-in and running wide -- a failure
# no Q/R weight change can fix, because the command is already saturated by
# noise before the corner starts.
# Disabling compensation outright was tried and was WORSE (see
# DELAY_COMPENSATION_ENABLED's RESULT note), so this caps rollforward DEPTH
# instead: still compensates typical latency, but bounds the noise
# compounding to a level measured to clearly improve peak/rms lateral error
# and steering reversal rate over deeper caps, across a wide range of
# measured pose_age_s.
#
# CAVEAT: this validates the n_delay -> reversal-rate mechanism, NOT the
# separate (and unsupported) claim that pose latency explains a
# sudden-corner regression -- a same-settings control run showed peak |e_y|
# essentially unchanged across a wide pose_age swing. The real fix is still
# stabilising pose_age_s upstream, not living on a truncated rollforward.
# Moved to MPCParams.max_delay_compensation_steps (the cap: a bigger measured
# age is clamped, not trusted blindly) and MPCParams.predict_epsi_clip (rad,
# the small-angle bound used in predict_ahead) — see mpc_params.py for the
# current values.

# ── n_delay stabilisation ───────────────────────────────────────────────────
# pose_age_s is noisy: control-loop jitter makes a raw round(pose_age_s / dt)
# flip between adjacent step counts tick to tick. Each flip changes how many
# commands predict_ahead() rolls x0 through, so x0 jumps discontinuously
# between rollforward depths on consecutive solves — injecting step changes
# into the state the QP sees at the control rate. Two guards, applied in
# compute():
#   1. Low-pass pose_age_s so a single late message can't move the step count.
#   2. Hysteresis on the resulting integer: only change n_delay when the
#      filtered estimate is clearly past the midpoint of the current bin, so
#      an age hovering near a boundary doesn't dither.
# Moved to MPCParams.pose_age_lp_alpha (per-tick low-pass on pose_age_s,
# ~0.3 s settle at 20 Hz) and MPCParams.n_delay_hysteresis (steps of deadband
# either side of a bin boundary) — see mpc_params.py for the current values.


def predict_ahead(
    x0: np.ndarray,
    Ad: np.ndarray,
    Bd: np.ndarray,
    pending_cmds,
    epsi_clip: float = 0.5,
) -> np.ndarray:
    """
    Roll the linear error-state model forward through commands already
    issued but not yet reflected in the measured pose, so the MPC solves
    against the state it will actually face instead of a stale x0.

    pending_cmds must be ordered oldest-first (the order they were issued).
    Mirrors fsae_MPCTest/sim/rollout/core.py's predict_ahead() exactly,
    including the e_psi clip — see that function's docstring for why the
    clip is needed (the e_psi -> e_y_dot coupling in Ad is only valid for
    small angles, and this rollforward has no per-step ground-truth
    correction the way the closed-loop MPC horizon does).

    epsi_clip (rad, ~28.6 deg at the default) is that small-angle bound; the
    default matches MPCParams.predict_epsi_clip, which MPCController passes
    explicitly.
    """
    x_p = x0.copy()
    for u in pending_cmds:
        x_p[2] = np.clip(x_p[2], -epsi_clip, epsi_clip)
        x_p = Ad @ x_p + Bd @ u
    return x_p
