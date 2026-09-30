"""
fsae_control/telemetry/columns.py — adaptive-feature trace columns

ADAPTIVE_COLUMNS defines the trailing CSV columns written from the
controllers' `last_telemetry`, in order. One cell per entry; controllers with
no adaptive features leave them empty so the column set is identical.
"""



# ── Adaptive-feature trace columns ───────────────────────────────────────
# Written from mpc_core's last_telemetry (see its "Adaptive-feature trace"
# comment). Each m_* column is the multiplier that ONE feature applied to ONE
# weight on that tick, so 1.0 means "this feature did nothing here" and the
# product of a weight's m_* columns times its base value is its *_eff column.
# That decomposition is the point: it tells you which feature moved a weight,
# not merely that the weight moved.
#
# Order here defines CSV column order; log_control writes one cell per
# entry, in this order, by looking each key up in the `adaptive` mapping.
# Controllers with no adaptive features (Stanley) pass nothing and every cell
# is written empty, so the column set stays identical across controllers.
ADAPTIVE_COLUMNS = (
    # Corner-factor scheduler (mpc_core._corner_factor/_low_speed_corner_boost):
    # a single CURRENT-curvature-driven fraction, and the low-speed boost
    # that adds to it, both gated multiplicatively so the boost cannot fire
    # on low speed alone.
    'corner_factor',           # 0 (straight) -> 1 (full corner), from current-position kappa only
    'low_speed_corner_boost',  # extra push toward "full corner", active only when corner_factor > 0 AND speed is low
    'corner_frac',             # corner_factor + low_speed_corner_boost, clipped to [0,1] -- the shared blend driver below
    # Q[0,0]/Q[2,2]/Q[3,3] after the straight/corner blend, before adaptive_Q_scaling's centred-softening.
    'Q_ey_base', 'Q_epsi_base', 'Q_r_base',
    # R[0,0]/R_rate[0,0] after their own straight/corner blend.
    'R_steer_corner_blend', 'Rrate_steer_corner_blend',
    # Per-feature multipliers still in use.
    'm_Q_ey_soften',      # adaptive_Q_scaling's centred-softening multiplier
    'm_R_speed',          # adaptive_R_scaling's speed-based multiplier
    'm_Rrate_antihunt',   # steer_rate_anti_hunt's straight/centred/aligned boost multiplier
    'm_Rrate_zone',       # nmpc_rrate_zone_enabled's three-zone (straight/approach/corner) multiplier; 1.0 = off
    'm_Rrate_reversal',   # reversal_penalty_boost's near-zero-previous-steer boost multiplier
    # Absolute weights handed to the QP after all of the above.
    'Q_ey_eff', 'Q_epsi_eff', 'Q_r_eff', 'R_steer_eff', 'Rrate_steer_eff',
    'R_a_accel_eff', 'R_a_brake_eff',  # a_cmd effort weight, split by sign; R_a_*_eff already includes the heading-error-driven asymmetry (epsi_ra_*)
    # ── NMPC-only columns (nmpc_core.NMPCController; empty for every LTV-QP
    # run, exactly as the m_* columns are empty for Stanley). Appended at the
    # END so the existing column order — and every offline script that parses
    # these CSVs by name — is unaffected.
    #
    # These are the NMPC's equivalent of the m_* decomposition: they say what
    # the solver did (how many Gauss-Newton iterations, whether the QP
    # subproblem actually solved, the achieved cost) and what its own
    # prediction expected (terminal e_y/e_psi, peak predicted |e_y|,
    # curvature at the far end of the horizon). A run where nmpc_status
    # spends time at 0, or where nmpc_pred_ey_end disagrees badly with the
    # e_y actually reached ~1 s later, is a model/solver problem rather than
    # a weighting problem — which is the distinction the LTV-QP's telemetry
    # could never make.
    'nmpc_iters',              # Gauss-Newton SQP iterations actually taken this tick
    'nmpc_status',             # 1.0 = QP subproblem solved, 0.0 = not (rejected/failed/budget)
    'nmpc_cost',               # achieved nonlinear cost of the shipped trajectory
    'nmpc_s0',                 # arc length of the car's Frenet projection (m)
    'nmpc_kappa_horizon_end',  # kappa(s) at the far end of the PREDICTED horizon (1/m)
    'nmpc_pred_ey_end',        # predicted e_y at the end of the horizon (m)
    'nmpc_pred_epsi_end',      # predicted e_psi at the end of the horizon (rad)
    'nmpc_pred_ey_max_abs',    # peak predicted |e_y| anywhere in the horizon (m)
    'n_latency',               # nmpc_latency_compensation_enabled's rollforward depth (0 when off)
    # nmpc_friction_circle_enabled only (empty otherwise, same convention as
    # every other column above); see `docs/reference/README.md`'s "Three
    # MPCC-inspired additions" section.
    'nmpc_fyf_max_abs',        # peak |front-axle lateral tyre force| anywhere in the horizon (N)
    'nmpc_fyr_max_abs',        # peak |rear-axle lateral tyre force| anywhere in the horizon (N)
    # nmpc_progress_enabled only (empty otherwise, same convention as every
    # other column above). Declared here BEFORE the feature is used live on
    # purpose: nmpc_friction_circle_enabled shipped without its two columns
    # and its own diagnostics silently never reached a CSV, which is how a
    # conflicting F_max went undiagnosed. See
    # fsae_MPCTest/docs/logs/nmpc_progress_term_investigation.md.
    'nmpc_v_cap',              # the speed CAP the cost used this tick (m/s)
    'nmpc_speed_cap_over',     # max(0, v_x - v_cap): 0 = car chose to go slower than the cap, >0 = cap binding (m/s)
    'nmpc_s_target_gap_end',   # s_target_N - s at the horizon end. GROWING tick-over-tick means a stuck/regressing solve (m)
)
