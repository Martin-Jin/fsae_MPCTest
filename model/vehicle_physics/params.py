"""
model/vehicle_physics/params.py — VehicleParams

PURPOSE
-------
The single source of truth for vehicle physics: mass, geometry, tyres,
suspension, aero and actuator limits, with the rationale for each value
inline. Instantiated once per simulation run and passed by reference.

USED BY
-------
  plant_step.py, state.py, gui/simulation.py, tuner/offline_tuner.py and the
  tuner/tools/ scripts that need the car's geometry or ceiling.
"""
import numpy as np


# ─────────────────────────────────────────────────────────────
# VEHICLE PARAMETERS
# ─────────────────────────────────────────────────────────────
class VehicleParams:
    """
    Physical parameters for a Formula Student electric vehicle (~255 kg wet).

    All values are based on typical FS EV specifications with rationale
    provided inline. This class is instantiated once per simulation run and
    passed by reference to avoid re-allocation overhead during inner loops.

    Used by: step_nonlinear_plant(), init_plant_state(), plant_to_tracking_error()
    Instantiated in: gui/simulation.py, tuner/offline_tuner.py (init_worker, run_headless_rollout)
    """

    def __init__(self):
        # ── Tuning Constants (Modify these to change car behavior) ────
        GRIP_SCALE    = 1.1  # Scales tyre stiffness and Pacejka slope
        INERTIA_SCALE = 0.8  # Scales yaw inertia and wheel rotational mass
        COASTING_SCALE = 3.0 # < 1.0 = Rolls further, > 1.0 = Stops faster

        # ── Geometry ────────────────────────────────────────────────────────
        # lf/lr/Iz must match the live lmpc/controller.py exactly (see this repo's
        # plant/model parity rule). Neither set of values is measured (the
        # true lf/lr/Iz aren't in the FSDS repo -- they live in git-LFS
        # .uasset binaries), so these are a deliberate choice, not a guess:
        # lf=0.85 > lr=0.70 would make the bicycle model OVERSTEER
        # (understeer gradient K_us < 0, v_crit ~35 m/s) -- a needless
        # stability risk on a car whose real balance can't be measured.
        # lf < lr instead makes it UNDERSTEER (stable at every speed).
        # Iz ~= m*lf*lr (~151.7) is the standard yaw-inertia estimate; a
        # smaller value under-estimates it, making the model expect a
        # twitchier car than reality. Keep these two files in sync manually.
        self.lf    = 0.70     # Distance from CoM to front axle (m)
        self.lr    = 0.85     # Distance from CoM to rear  axle (m)
        self.m     = 255.0    # Total vehicle mass including driver (kg)
        self.Iz    = 150.0    # Yaw moment of inertia about CoM (kg·m²)
        self.tf    = 1.25     # Front track width between tyre contact patches (m)
        self.tr    = 1.20     # Rear  track width between tyre contact patches (m)
        # FSDS's own docs state 25 cm CoG height (stationary) for its vehicle
        # model; matched here rather than an independently-guessed value.
        # (FSDS's collision bounding box dimensions are explicitly documented
        # as unrelated to the real vehicle geometry, so tf/tr/lf/lr above are
        # NOT derived from it — only this CoG height figure is used.)
        self.h_cg  = 0.25     # Centre-of-gravity height (m); drives load transfer
        self.g     = 9.81     # Gravitational acceleration (m/s²); set early —
                               # needed by static_fz_per_corner() for the Cf/Cr
                               # slope-matching calc further below.
        # Actuator limits: enforced as hard bounds in controller/lmpc/solve.py's QP constraints.
        # 25deg matches the live stack's physical steering limit (see
        # ros2/src/fsae_planning/control/fsae_control/fsae_control/lmpc/constants.py's
        # MAX_STEER_RAD and fsae_control.control_utils/fsds_bridge).
        self.max_steer       = np.radians(25.0)  # Max rack-limited steering angle (rad)
        # Max steering SLEW rate (rad/s) — how fast the rack can move, as
        # opposed to how far.  Feeds the MPC's hard du_max constraint (see
        # lmpc/build.py's init_parameterized_mpc and the live lmpc/controller.py, which
        # must agree).  Measured from live FSDS telemetry by inverting the
        # logged yaw rate through the kinematic bicycle (delta = atan(L*r/v)):
        # achieved roadwheel rate reached p99 ~138 deg/s and max ~218 deg/s,
        # so the actuator is at least ~200 deg/s.  180 deg/s sits just under
        # that measured floor.  Refine via system-ID on the running sim.
        self.max_steer_rate  = np.radians(180.0)
        # FS EV peak acceleration ~12 m/s² (0→17 m/s in ~2 s); braking ~9 m/s² (~0.9g).
        self.max_accel       = 12.0              # Max longitudinal acceleration (m/s²)
        # Matches lmpc/constants.py's MAX_BRAKE. Not an FSDS-measured value; the MPC
        # never commands braking anywhere near this limit regardless of
        # weighting, so it acts as a backstop rather than an active bound.
        # Keep numerically identical to lmpc/constants.py.
        self.max_accel_brake = -7.0             # Max longitudinal braking (m/s²)
        # This project's real car tops out at ~60 km/h (16.7 m/s) — a slower
        # autonomous test platform, not FSDS's own ~27 m/s simulator ceiling.
        self.max_v = 16.7 # Maximum possible speed the vehicle can go

        # ── FSDS lateral-acceleration ceiling ────────────────────────────────
        # Models a simulator quirk, not real tyre grip: FSDS caps sustained
        # lateral acceleration at ~7.5 m/s² (well below the ~12.3 this car
        # reaches on a lap), enforced as a lagged restoring yaw moment rather
        # than a hard clip — it overshoots ~30% before settling, which a clip
        # cannot reproduce. See docs/reference/vehicle_physics.md "The FSDS
        # lateral-acceleration ceiling" for the plain-English explanation and
        # `docs/reference/simulator_fidelity.md`'s ceiling section for the full
        # measurement/derivation this is fitted to — not repeated here.
        # Set alat_ceiling_enabled=False to recover the unconstrained plant
        # (e.g. for real-vehicle work; this models FSDS, not the physical car).
        self.alat_ceiling_enabled = True
        self.alat_ceiling = 7.5    # Low-speed floor (m/s²) — see ceiling_value() below
        # Speed-dependent ceiling(v) = intercept + slope*v, engaged only where
        # it exceeds the flat floor above (see ceiling_value()). Sweep-fit
        # values; the step-test fit disagrees on level but not shape — doc.
        self.alat_ceiling_slope = 0.47       # m/s² per m/s
        self.alat_ceiling_intercept = 2.46   # m/s²
        # 'pi' = validated leaky-integral law (settles at the ceiling by
        # structure, for any gain). 'p' = rejected proportional law (its
        # equilibrium must sit above the setpoint by construction — no gain
        # fits both settled level and transient peak); kept only so
        # tuner/validation/plant_openloop_validation.py --ab can reproduce the
        # measurement that rejected it.
        self.alat_ceiling_mode = 'pi'
        # Restoring-moment gain (N·m per m/s² of excess), fitted to the
        # measured PEAK only — the settled value is not independently fitted,
        # it falls out of the integral structure at the ceiling.
        self.alat_ceiling_gain = 450.0
        # Lag (s) on the restoring moment — produces the measured overshoot;
        # under the integral law affects ONLY the transient, not the settled
        # value. Measured 2026-08-07 via a long step-hold; re-measure with
        # ros2/run_steering_step.sh --no-sim -p 'speeds:=[5.0,8.0,12.0]'
        # -p 'step_s:=8.0' -p 'repeats:=2' then
        # tuner.validation.plant_openloop_validation if corner entry time changes.
        self.alat_ceiling_tau = 0.40

        # ── Unsprung Mass ────────────────────────────────────────────────────
        self.m_us  = 7.5      # Unsprung mass per corner: wheel + upright + hub (kg)

        # ── Tyre Geometry ────────────────────────────────────────────────────
        self.r_eff = 0.2286   # Effective rolling radius for a 13" wheel+tyre (m)

        # ── Rotational Inertia ───────────────────────────────────────────────
        # Each driven wheel's effective inertia includes the motor/gearbox
        # referred through the reduction ratio.
        self.I_wheel      = 0.9 * INERTIA_SCALE  # Per-wheel rotational inertia (kg·m²)
        self.I_drivetrain = 0.05 * INERTIA_SCALE # Motor+gearbox inertia referred to wheel (kg·m²)
        self.I_w_eff_r    = self.I_wheel + self.I_drivetrain  # Rear driven wheels
        self.I_w_eff_f    = self.I_wheel                       # Front free-rolling wheels

        # ── Suspension Spring / Damper / ARB ────────────────────────────────
        # Wheel-rate = spring-rate × motion_ratio²; motion_ratio ≈ 0.65-0.70
        # for a pushrod FS suspension gives ~25 N/mm front, ~30 N/mm rear.
        self.k_susp_f = 25000.0   # Front wheel-rate (N/m)
        self.k_susp_r = 30000.0   # Rear  wheel-rate (N/m)
        # Damping at ~30% of critical damping plus unsprung contribution.
        # c_crit ≈ 2*sqrt(k * m_corner); 30% of that + unsprung ≈ 1500 front.
        self.c_damp_f = 1500.0    # Front corner damper rate (N·s/m)
        self.c_damp_r = 1800.0    # Rear  corner damper rate (N·s/m)
        # Anti-roll bars add an effective wheel-rate that couples left and right
        # suspension: the ARB force = k_arb * (z_left - z_right).
        self.k_arb_f = 8000.0     # Front ARB equivalent wheel-rate (N/m)
        self.k_arb_r = 6000.0     # Rear  ARB equivalent wheel-rate (N/m)
        # Hard bump/droop stops at ±40 mm from equilibrium.
        self.z_max =  0.040       # Maximum compression from equilibrium (m)
        self.z_min = -0.040       # Maximum droop from equilibrium (m)

        # ── Kinematic Camber Gain ────────────────────────────────────────────
        # In a double-wishbone suspension, jounce (compression) induces negative
        # camber on the outer wheel, increasing lateral grip.
        # Gain = 0.5 deg/mm = 0.5 * π/180 / 0.001 ≈ 8.73 rad/m
        self.camber_gain    = 8.73  # Camber change per unit suspension travel (rad/m)
        self.camber_stiff_f = 0.15  # Front camber stiffness: fraction of Fz added as Fy per rad
        self.camber_stiff_r = 0.12  # Rear  camber stiffness (slightly lower, typical slick)

        # ── Pacejka MF94 Lateral Coefficients ───────────────────────────────
        # The MF94 formula: Fy = mu*Fz * sin(C * arctan(B*alpha - E*(B*alpha - arctan(B*alpha))))
        # B: stiffness factor (controls initial slope of the Fy-vs-alpha curve)
        # C: shape factor    (controls the sharpness of the peak)
        # D: peak factor     (scales peak force; combined with mu*Fz gives peak Fy)
        # E: curvature factor (negative = sharper peak, typical for racing slick)
        # Sv, Sh: vertical/horizontal offsets from ply-steer and conicity
        self.B_f  = 15.0 * GRIP_SCALE;  self.C_f  = 1.45;  self.D_f  = 1.0 * GRIP_SCALE
        self.E_f  = -1.5;  self.Sv_f = 0.0;   self.Sh_f = 0.002

        self.B_r  = 12.0 * GRIP_SCALE;  self.C_r  = 1.45;  self.D_r  = 1.0 * GRIP_SCALE
        self.E_r  = -1.8;  self.Sv_r = 0.0;   self.Sh_r = 0.001

        # ── Pacejka MF94 Longitudinal Coefficients ───────────────────────────
        # Same shape function applied to longitudinal slip ratio kappa.
        # Peak at slip ratio ≈ 0.10-0.15 for a slick tyre.
        self.Bx_r = 12.0;  self.Cx_r = 1.65
        self.Dx_r = 1.00;  self.Ex_r = -0.5

        # ── Tyre Relaxation Lengths ───────────────────────────────────────────
        # A tyre does not respond instantaneously to a slip angle change.
        # The lateral force builds up over a "relaxation length" σ (m):
        #   dFy/dt = (vx / σ) * (Fy_steady_state − Fy_actual)
        # At vx=10 m/s with σ=0.45 m: time constant ≈ 40 ms — significant at 20 Hz.
        self.sigma_y_f = 0.45      # Front lateral relaxation length (m)
        self.sigma_y_r = 0.40      # Rear  lateral relaxation length (m)

        # ── Friction and Load Sensitivity ────────────────────────────────────
        # Peak friction coefficient for a dry racing slick. FSDS deliberately
        # doesn't publish its own tyre friction model (to stop teams
        # reverse-engineering/overfitting to the sim), so this is sanity-
        # checked against published FSAE 13" slick data instead: typical peak
        # mu ~1.4-1.8 on a good surface. Base value picked near the middle of
        # that range so GRIP_SCALE's default 1.1x still lands inside it
        # rather than compounding past it (mu * GRIP_SCALE = 1.76).
        self.mu     = 1.6 * GRIP_SCALE # Peak friction coefficient (dimensionless)
        # Reduced load sensitivity: slicks show less degradation than road tyres.
        # At nominal Fz~600 N: mu_eff = 1.76*(1 - 0.00012*600) ≈ 1.63 — still strong.
        self.k_sens = 0.00012      # Load sensitivity (1/N)

        # ── Linear cornering stiffness (MPC internal model only) ─────────────
        # model/bicycle_model.py's linear MPC model uses Cf/Cr, not these Pacejka
        # curves directly — but they MUST match the Pacejka curve's initial
        # slope (C_eff ≈ mu_eff * Fz_nominal * B * C * D) or the MPC's internal
        # prediction silently diverges from the plant it's actually
        # controlling (see README "If you import new tyre data"). Computed
        # here from the coefficients above — rather than hardcoded — so this
        # can never silently drift out of sync with them again.
        Fz_f_nom, Fz_r_nom = self.static_fz_per_corner()  # Static per-corner load (N)
        mu_f_eff = self.mu * (1.0 - self.k_sens * Fz_f_nom)
        mu_r_eff = self.mu * (1.0 - self.k_sens * Fz_r_nom)
        self.Cf = mu_f_eff * Fz_f_nom * self.B_f * self.C_f * self.D_f  # Front cornering stiffness (N/rad)
        self.Cr = mu_r_eff * Fz_r_nom * self.B_r * self.C_r * self.D_r  # Rear  cornering stiffness (N/rad)

        # ── Aerodynamics ─────────────────────────────────────────────────────
        # Total downforce coefficient Cl_A split 43/57 front/rear for
        # neutral balance: more rear downforce than front is typical FS tuning.
        self.rho       = 1.225     # Air density at sea level (kg/m³)
        # Drag area (Cd * frontal area). FSDS documents Cd=0.3 for its vehicle
        # model but doesn't publish a frontal area; ~1.1 m² is a reasonable
        # estimate for a small open-wheel FS car, giving Cd_A ≈ 0.33 m².
        # Deliberately NOT scaled by COASTING_SCALE (see Crr below) — aero
        # drag is a well-defined physical quantity that shouldn't be inflated
        # just to tune coast-down feel. At max_v=16.7 m/s this contributes
        # only ~56 N of drag, appropriately small for this speed range.
        self.Cd_A      = 0.33      # Drag area (m²)
        self.Cl_A_f    = 0.645     # Front wing downforce area (m²)
        self.Cl_A_r    = 0.855     # Rear  wing downforce area (m²)
        # Under braking the nose dips (pitch forward), increasing front downforce
        # and reducing rear. Cl_pitch_sens = 0.03 means a 1g deceleration shifts
        # front Cl_f up by 3% and rear Cl_r down by 3%.
        self.Cl_pitch_sens = 0.03  # Fractional Cl change per unit (a/g)

        # ── Rolling Resistance and Stiction ──────────────────────────────────
        # Crr is a lumped, constant-magnitude force standing in for both true
        # rolling resistance and the drivetrain/motor cogging drag that isn't
        # modelled anywhere else in this plant (there's no explicit motor-
        # braking/regen term). Unlike Cd_A above, this IS the deliberate
        # non-physical tuning knob for "how hard does the car coast to a stop
        # with no throttle/brake input" — COASTING_SCALE=3.0 gives ~61.5 N
        # total, the same order of magnitude as a real rolling-resistance-only
        # estimate (Crr_coeff*m*g ≈ 0.015*255*9.81 ≈ 38 N) with headroom for
        # the missing cogging/regen effect. Reduce COASTING_SCALE if coast-down
        # feels too aggressive; Cd_A stays physically fixed regardless.
        self.Crr        = 20.5  * COASTING_SCALE # Constant rolling drag force at speed (N)
        self.F_stiction = 600.0 # Static breakaway force (N); (From the four wheels in total, so / 4 for a single wheel)

        # ── Actuator Lag ─────────────────────────────────────────────────────
        # First-order lag: d(delta_act)/dt = (delta_cmd - delta_act) / tau_delta
        # EV motors respond almost instantly; FS rack-and-pinion steering has
        # a small but non-negligible lag (~80 ms) compared to a hydraulic system.
        self.tau_delta = 0.08      # Steering actuator time constant (s)
        self.tau_a     = 0.02      # Acceleration (torque) time constant (s)

    @property
    def L(self):
        """Total wheelbase: lf + lr (m). Used frequently in weight distribution formulae."""
        return self.lf + self.lr

    def alat_ceiling_at(self, vx):
        """
        Speed-dependent sustained lateral-accel ceiling (m/s²). See the
        alat_ceiling*/alat_ceiling_slope/alat_ceiling_intercept comments
        above for the derivation.

        max(flat, line) so this never lowers the ceiling below the
        already-validated flat value -- it only raises it at higher speed,
        where the sweep fit rises above 7.5 (v >~ 10.7 m/s).
        """
        return max(self.alat_ceiling,
                    self.alat_ceiling_intercept + self.alat_ceiling_slope * vx)

    def static_fz_per_corner(self):
        """
        Compute the static normal load at each front and rear corner at rest
        (no aerodynamics, no longitudinal acceleration).

        Physics: weight distribution by moment balance about the rear axle:
          Fz_front_total = m * g * (lr / L)
          Fz_rear_total  = m * g * (lf / L)
        Divided by 2 for left/right symmetry.

        Returns
        -------
        (Fz_f, Fz_r) : (float, float)
            Normal load per front corner (N) and per rear corner (N).

        Used by: static_z_equilibrium(), and indirectly by step_nonlinear_plant()
                 to set baseline Fz before load transfer is applied.
        """
        Fz_f = self.m * self.g * (self.lr / self.L) / 2.0
        Fz_r = self.m * self.g * (self.lf / self.L) / 2.0
        return Fz_f, Fz_r

    def static_z_equilibrium(self, v_nominal=7.0):
        """
        Compute the suspension equilibrium deflection z_eq (m) at a nominal
        cruise speed, including aerodynamic downforce.

        Why include aero in the equilibrium?
        If the equilibrium is computed at rest (no aero), then when the vehicle
        reaches speed the aero load pushes the suspension down by z_eq_aero, which
        would appear as a large initial transient. Using a speed-inclusive equilibrium
        centres the suspension in its travel range at typical running speeds.

        Physics: At equilibrium, spring force = total static load:
          k * z_eq = Fz_static + 0.5 * (0.5 * rho * Cl_A * v²)
        Solving: z_eq = (Fz_static + F_aero_per_corner) / k

        Parameters
        ----------
        v_nominal : float
            Representative cruise speed for aero calculation (m/s).
            Defaults to 7.0 m/s (the previous fixed v_ref).

        Returns
        -------
        (z_eq_f, z_eq_r) : (float, float)
            Front and rear equilibrium suspension deflection (m).

        Used by: step_nonlinear_plant() — called once per call to compute the
                 spring force baseline.
        """
        Fz_f_static, Fz_r_static = self.static_fz_per_corner()
        # Aerodynamic downforce per axle at nominal speed, split per corner (÷2)
        F_down_f = 0.5 * self.rho * self.Cl_A_f * v_nominal**2
        F_down_r = 0.5 * self.rho * self.Cl_A_r * v_nominal**2
        Fz_f_total = Fz_f_static + 0.5 * F_down_f   # per corner
        Fz_r_total = Fz_r_static + 0.5 * F_down_r   # per corner
        # Deflection = load / spring-rate
        return Fz_f_total / self.k_susp_f, Fz_r_total / self.k_susp_r
