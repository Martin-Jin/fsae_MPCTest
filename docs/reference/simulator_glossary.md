# Simulator Glossary: FSDS, Offline Rollout, 2D GUI

Three different things in this project get called "the simulator." This doc defines each one once and states, in plain terms, how trustworthy each is as a stand-in for the real car. Other docs link here instead of re-explaining the distinction.

## The three things

| Name | What it is | Where it lives | What it's for |
|---|---|---|---|
| **FSDS** | The AirSim/UE4 3D simulator: a real physics engine driving a simulated car. | Outer repo root (`Formula-Student-Driverless-Simulator`). | What the live ROS 2 stack (`fsae_planning`) actually drives in. Used mainly to test and validate new features before they're trusted on the real car. |
| **Offline rollout** | A headless, non-graphical closed-loop simulation: `sim/rollout_core.py` driving the 24-state nonlinear plant in `model/vehicle_physics.py`. | `fsae_MPCTest` repo. | What the CMA-ES auto-tuner (`tuner/offline_tuner.py`) and `python -m tuner.recorded_map_rollout` actually run against. This is the "offline sim" the rest of this project's docs mean by that phrase unless they say otherwise. |
| **2D GUI** | An interactive matplotlib tool (`gui/simulation.py`) wrapping the same offline rollout, for drawing/loading a path and watching one run frame by frame. | `fsae_MPCTest` repo. | Visualization and manual prototyping only. It is not a separate simulation engine, it calls the same `rollout_core.run_core_rollout()` the offline rollout does, so its physics fidelity is identical to the offline rollout's, not lower. It is not itself validated against FSDS or the real car (nothing offline is, see below). |

If a doc in this project says "the simulator" without qualifying it, check which of the offline rollout or FSDS the surrounding sentence actually means; this project's docs are usually locally unambiguous but rarely spell out the three-way distinction explicitly, which is what this page is for.

## How trustworthy is each one

This is the most important thing to take from this page.

**The offline rollout (and the 2D GUI, which shares its physics) is rough validation only.** It has never been matched against reality and carries no measured accuracy figure. Its job is to check that the control math and logic behave sensibly, and to get a tuned weight set into the right ballpark, not to predict how the car will actually perform. A result that looks good offline is a starting point for further tuning, not a confirmed outcome.

**FSDS is mainly used to test and validate new features**, and is a materially closer approximation to the real car than the offline rollout, since it's a real physics engine rather than an offline Python plant model. But FSDS itself is **also not confirmed accurate against the real car**. Read this as a relative ordering, FSDS is closer than the offline rollout, not as "FSDS is validated": concrete measured gaps between FSDS and the real car are documented in `CLAUDE.md`'s "The offline sim does not yet fully predict the car" section (steering saturation 4.8% in the offline rollout vs. 21.1% on the real car, on the same map and gains) and in [simulator_fidelity.md](simulator_fidelity.md).

Concretely, from least to most representative of the real car, though none is confirmed accurate:

```
2D GUI  ≈  offline rollout   <   FSDS   <   real car
(same physics as offline)      (real physics engine,
                                 still not validated)
```

Always validate a planning/control change live, at minimum in FSDS, ideally on the real car, before trusting an offline-only result. See `CLAUDE.md`'s "Testing" section for the project's actual correctness bar.

## Which doc do I want

| Question | Offline-side doc | FSDS/live-side doc |
|---|---|---|
| How do I run it? | [docs/offline_guide.md](../offline_guide.md) | [docs/fsds/fsds_integration_guide.md](../fsds/fsds_integration_guide.md) |
| How do I configure/tune it? | [docs/architecture.md](../architecture.md)'s "Configuring the Project"/"Configuring the Vehicle", [docs/tuning.md](../tuning.md) | [docs/fsds/fsds_settings.md](../fsds/fsds_settings.md) |
| How does FSDS/ROS 2 actually connect? | n/a | [docs/fsds/fsds_ros_integration.md](../fsds/fsds_ros_integration.md) |
| Where do the two diverge, and why? | [simulator_fidelity.md](simulator_fidelity.md) | [simulator_fidelity.md](simulator_fidelity.md) (same doc, covers both) |
| What must stay numerically identical between them? | [offline_live_parity.md](offline_live_parity.md) | [offline_live_parity.md](offline_live_parity.md) (same doc, covers both) |
| How does the controller/MPC math work? | [lmpc.md](../lmpc.md), [nmpc.md](../nmpc.md), [error_state_reference.md](../error_state_reference.md) — universal, describe both sides | (same) |

This page only points; it doesn't restate what those docs already cover.
