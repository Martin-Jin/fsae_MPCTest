# Superseded and Rejected Mechanisms

Mechanisms that were built and then removed, superseded or rejected.

Kept so a future session does not re-invent something already tried and
measured. Each entry records what it did and why it went away.

A larger set of removals — the whole lookahead gain-scheduling family — has its
own document: `docs/removed_mechanisms.md`.

## Exit-heading boost: superseded by the corner-factor scheduler

The old `_lookahead_exit_boost`/`_update_lookahead_peak`/`dist_since_peak`
mechanism (which boosted `Q[2,2]` for a decaying window after a corner's
peak curvature, to help the car straighten out on exit) no longer exists on
either side. It was replaced by the corner-factor scheduler — see
"Corner-factor scheduler — what replaced the lookahead gain-scheduling
family" above for the current mechanism, and
[`removed_mechanisms.md`](logs/removed_mechanisms.md) for what was removed.

For the history of the exit-boost mechanism itself — the timing bug where
its decay clock was keyed on lookahead-window peak curvature instead of the
car's own physical apex (causing it to decay to a no-op before the car
reached the corner exit), the follow-up fix making the decay window
speed-scaled instead of a fixed 5 m, and a rejected `R[1,1]`/`Q[4,4]`
heading-misalignment accel gate — see `docs/logs/late_turn_in_investigation.md`,
"Addendum (2026-08-11): exit-heading boost was firing at the wrong time".

## Accel effort weight (superseded by accel/brake split)

The MPC's acceleration-effort weight (`R_diag[1]` / `MPCParams.r_a`) is no longer a single scalar applied symmetrically to accel and brake — it was replaced by independent accel/brake weights (see "Accel/brake effort weight split" below). Do not reintroduce a single shared `r_a` scalar without accounting for why it was split: a shared weight that is loose enough to accelerate well on straights is also too loose on braking, and vice versa.

Historical tuning path and full measurements: `docs/logs/sim_to_real_investigation.md` § 59 ("MPC underaccelerating on clean straights: `r_a` swept 0.85 → 0.77").

## Low-speed steering-rate boost (removed)

A mechanism that scaled `R_rate[0,0]` up at low speed (`_low_speed_steer_rate_boost`, `boost_max=2.5, k=0.35`) was tried, live-tested, and disabled the same day for regressing turn-in; it no longer exists in either codebase at all, having been removed along with the rest of the lookahead gain-scheduling family when the corner-factor scheduler replaced it. See "Corner-factor scheduler" above for what replaced it and the current mechanism, and `docs/logs/late_turn_in_investigation.md`'s "Appendix — Low-speed steering-rate boost: full incident" for the full incident history.

## Curvature-forcing term: a rejected approach to blind path-bending prediction

The QP's dynamics model (`Ad`/`Bd`) has no path-curvature term, so with
`e_y ≈ e_psi ≈ 0` on a straight approach its own predicted rollout stays
near zero regardless of how sharply the real path bends ahead — no
reweighting of an *existing* tracking error (`adaptive_Q_lookahead`,
`lookahead_steer_effort_relax`, etc.) can compensate, since there is no
predicted error yet for a cheaper weight to act on.

A forcing term (`curvature_forcing_enabled`/`curvature_forcing_gain`) was
built to inject predicted curvature directly into the dynamics constraint
(`w[2,k] = -v_x·κ(s_k)·dt·gain`) so the QP's own rollout would anticipate
the bend. It is **structurally unsound and disabled**: because the term
perturbs the same recursion the QP minimizes cost over, the solver is free
to choose *how* to spend the disturbance across the horizon, and at any
gain large enough to matter it commits to a transient steer *away* from the
corner before correcting — reproduced in a clean, noise-free synthetic QP
test across a full gain sweep, not a live-noise artifact.

**Do not re-enable `curvature_forcing_enabled` by flipping the flag alone.**
A future redesign should shift the *reference*/error definition (curve the
heading `e_psi` is measured against) rather than perturb the QP's own
dynamics recursion — this is the direction the later NMPC formulation takes,
where curvature enters as a function of a state the solver actively chooses
rather than external data it can defer absorbing.

See `docs/logs/late_turn_in_investigation.md`'s "Part 6b" for the full
derivation, the synthetic verification, the anti-hunt interaction
(`anti_hunt_k_lookahead`, settled at `15.0`), and the gain-sweep evidence
behind the structural-unsoundness finding.

## Precomputed corner segmentation (removed)

*(Historical: a precomputed per-waypoint `CornerMap` once replaced the live corner-anticipation scan with an exact index lookup for static paths. It was part of the ~15-mechanism lookahead gain-scheduling family removed wholesale by the corner-factor rewrite — see [`Corner-factor scheduler`](#corner-factor-scheduler-what-replaced-the-lookahead-gain-scheduling-family) above for what replaced it, and [`removed_mechanisms.md`](removed_mechanisms.md)'s "7. Precomputed corner segmentation (`CornerMap`)" for the mechanism-level summary. Full implementation history in `docs/logs/late_turn_in_investigation.md` Parts 3-6.)*
