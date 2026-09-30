"""
model/vehicle_physics/tyres.py — Pacejka MF94 tyre force curves

PURPOSE
-------
Pure functions mapping slip angle (lateral) or slip ratio (longitudinal) and
normal load to tyre force, using the MF94 magic formula. Holds no state.

USED BY
-------
  plant_step.py, once per wheel per sub-step.
"""
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# PACEJKA MF94 TYRE MODEL — LATERAL
# ─────────────────────────────────────────────────────────────────────────────
def _mf94(x_in, B, C, D, E):
    """
    Core Pacejka MF94 (Magic Formula 1994) shape function.

    Computes the normalised tyre force curve used for both lateral (Fy) and
    longitudinal (Fx) tyre force. Does NOT apply friction scaling (mu*Fz)
    or horizontal/vertical offsets — those are applied in the callers.

    The MF94 formula:
        Bx = B * x_in
        y = D * sin(C * arctan(Bx − E * (Bx − arctan(Bx))))

    Behaviour:
      - B controls initial slope: dFy/d(alpha) at alpha=0 ≈ B*C*D (stiffness)
      - C controls shape: C<2 → pronounced peak, C=2 → flat peak (asymptotes)
      - D is the peak value (force normalised; actual peak = mu*Fz*D)
      - E < 0 sharpens the peak and increases the shape near zero slip;
        E → 1 gives a rounder response (typical for bias-ply; slicks: E ≈ −1.5)

    Parameters
    ----------
    x_in : float
        Input slip (slip angle in rad for lateral; slip ratio for longitudinal).
    B, C, D, E : float
        Pacejka shape coefficients.

    Returns
    -------
    float : Normalised force (dimensionless; multiply by mu*Fz to get Newtons).

    Called by: pacejka_lateral_mf94(), pacejka_longitudinal_mf94()
    """
    Bx = B * x_in
    return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))


def pacejka_lateral_mf94(alpha, Fz, mu, B, C, D, E, Sv, Sh,
                          gamma=0.0, camber_stiff=0.0):
    """
    Full MF94 lateral tyre force including offsets and camber thrust (N).

    Formula:
        Fy = mu * Fz * MF94(alpha + Sh) + Sv + camber_stiff * Fz * gamma

    The ply-steer offset Sh shifts the zero-force slip angle slightly
    (non-zero Fy at zero slip angle is common in real tyres due to
    construction asymmetry). Sv is a constant vertical offset (conicity).
    Camber thrust adds lateral force proportional to camber angle gamma
    and normal load Fz.

    Parameters
    ----------
    alpha : float
        Tyre slip angle (rad). Positive = tyre pointing left of travel direction.
    Fz : float
        Normal load on the tyre (N). Scales peak lateral force.
    mu : float
        Effective peak friction coefficient (dimensionless). Already includes
        load sensitivity degradation — computed externally.
    B, C, D, E : float
        Pacejka shape coefficients (see _mf94 docstring).
    Sv : float
        Vertical offset (N) — ply-steer / conicity constant force.
    Sh : float
        Horizontal offset (rad) — shifts zero-crossing of the Fy curve.
    gamma : float
        Camber angle (rad). Positive = top of tyre leans outward from car centre.
    camber_stiff : float
        Camber stiffness coefficient: fraction of Fz added as Fy per rad of camber.

    Returns
    -------
    float : Lateral tyre force Fy (N). Positive = leftward in tyre frame.

    Called by: step_nonlinear_plant() — once per wheel per substep.
    """
    Fy0 = mu * Fz * _mf94(alpha + Sh, B, C, D, E) + Sv
    return Fy0 + camber_stiff * Fz * gamma


# ─────────────────────────────────────────────────────────────────────────────
# PACEJKA MF94 TYRE MODEL — LONGITUDINAL
# ─────────────────────────────────────────────────────────────────────────────
def pacejka_longitudinal_mf94(kappa, Fz, mu, Bx, Cx, Dx, Ex):
    """
    MF94 longitudinal tyre force (N).

    Formula:
        Fx = mu * Fz * MF94(kappa)

    No offsets (Sv, Sh) are needed for the longitudinal direction; the
    symmetry of driving vs. braking is captured by the kappa sign alone.

    Longitudinal slip ratio kappa:
        kappa = (r_eff * omega − vx) / max(r_eff * omega, |vx|)
    Positive kappa = wheel spinning faster than ground speed (drive slip / wheelspin).
    Negative kappa = wheel slower than ground speed (brake slip / lockup).
    Peak Fx typically occurs at |kappa| ≈ 0.10-0.15 for a racing slick.

    Parameters
    ----------
    kappa : float
        Longitudinal slip ratio (dimensionless).
    Fz : float
        Normal load (N).
    mu : float
        Effective peak friction (dimensionless; load-sensitivity applied externally).
    Bx, Cx, Dx, Ex : float
        Pacejka longitudinal shape coefficients.

    Returns
    -------
    float : Longitudinal tyre force Fx (N). Positive = driving/forward force.

    Called by: step_nonlinear_plant() — once per wheel per substep.
    """
    return mu * Fz * _mf94(kappa, Bx, Cx, Dx, Ex)
