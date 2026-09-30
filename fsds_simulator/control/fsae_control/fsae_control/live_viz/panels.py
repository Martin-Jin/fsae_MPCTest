"""
fsae_control/live_viz/panels.py — view geometry and debug-panel term tables

Window size/refresh constants and the ordered term lists behind each bar-graph
panel. Pure data: no ROS or matplotlib imports.
"""

# Close-up window: how far ahead/behind/either side of the car the axes span
# (metres). Small enough to actually see steering/tracking detail up close,
# per the "close up" ask -- this is a debug tool, not a full-track overview.
VIEW_HALF_WIDTH = 15.0
VIEW_AHEAD = 25.0
VIEW_BEHIND = 8.0

TRAIL_MAXLEN = 2000        # ~40s at 50 Hz control rate, plenty for a debug view
REDRAW_HZ = 25.0           # window refresh rate; independent of the 50 Hz control loop.
# Not pushed higher than this: each frame does a full ax.clear() + re-plot
# (scatter/lines/legend/text), not a blit-based partial update, so redraw
# cost scales with cone/path point counts: past ~25-30 Hz the redraw itself
# starts taking longer than the interval on a typical track-sized cone map,
# and frames just queue up behind rclpy.spin_once() instead of arriving
# sooner. Move to blitting (redrawing only changed artists) if a higher rate
# is ever needed.


# Which terms (mpc_controller.py's _publish_debug_weights() dict keys)
# belong on which bar-graph panel, and each panel's own axis label. Grouped
# by unit family, NOT all in one chart -- a steering-rate term of a few
# milliradians/tick and a 0.3 m lateral error are both real costs but
# squared-and-weighted they sit at wildly different absolute magnitudes, so
# one shared 0-100% scale makes the smaller group always look like ~0% even
# when it is the dominant term within its own family. Matches the 'group'
# field _publish_debug_weights() tags each term with; kept as an explicit
# order/label table here (not derived from the message) so panel order is
# stable regardless of dict iteration order.
DEBUG_BAR_GROUPS = (
    # 'progress' and 'v_cap_hinge' are NMPC-only and only present when
    # nmpc_progress_enabled; absent terms are skipped, so listing them here
    # is inert in every other configuration. 'v_cap_hinge' REPLACES 'e_v'
    # in that mode rather than reusing its name (see mpc_controller.py's
    # _publish_debug_weights), so a bar labelled e_v is always a real
    # two-sided speed error and never a cap hinge sitting at zero.
    ('tracking', 'Tracking error cost (% of tracking total)',
     ('e_y', 'e_yd', 'e_psi', 'yaw_rate', 'e_v', 'v_cap_hinge', 'progress',
      'steering', 'accel')),
    ('effort', 'Input effort cost (% of effort total)',
     ('steer_effort', 'accel_effort')),
    ('rate', 'Input rate-of-change cost (% of rate total)',
     ('delta_u_steer', 'delta_u_accel')),
)

# Order the horizon-summed panel draws its bars in. Unlike DEBUG_BAR_GROUPS
# above (step-0-only, one 100% scale per unit-family group), every one of
# these IS on one shared scale together: each is a share of total_cost, the
# solver's true full-horizon objective, so they are genuinely comparable
# (see mpc_controller.py's _publish_debug_weights()'s horizon_terms).
DEBUG_HORIZON_TERMS = (
    'e_y', 'e_yd', 'e_psi', 'yaw_rate', 'e_v', 'v_cap_hinge', 'progress',
    'steer_effort', 'accel_effort', 'delta_u_steer', 'delta_u_accel',
)

# Stanley's debug_stanley terms shown on its own error panel (NOT its
# control-law-term panel below): just its two tracking errors, one shared
# 100% scale -- heading error and cross-track (lateral) error are both
# radians/metres-squared-weighted-free raw values here (Stanley has no QP
# cost weights), so "percentage" is share of |e_y|+|e_psi|, a simple
# at-a-glance "which error dominates" signal, analogous to MPC's tracking
# panel but with only the two terms Stanley actually has.
STANLEY_ERROR_TERMS = ('e_y', 'e_psi')

# Stanley's three additive control-law terms (see
# stanley_controller.py's _publish_debug_stanley()) -- percentage of the
# sum of their absolute values, i.e. "how much of this tick's total
# steering effort came from which term", matching the user's own framing
# ("atan term vs heading error term... as a percentage of total control
# input to steering").
STANLEY_LAW_TERMS = ('heading_error', 'atan_cross_track', 'yaw_rate_damping')
