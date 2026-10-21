# External Reference: CMR Driverless Controls

**This is an external reference about another team's stack.** It summarises the public controls documentation of CMR Driverless (Carnegie Mellon Racing), what this project took from it, and what was held off. It is not part of this codebase and describes none of this project's code, except in the last section, which links to the project's own docs.

Sources:

- CMR Driverless docs landing page: <https://cmr.red/driverless/driverless-docs/build/html/index.html>
- CMR controls docs: <https://cmr.red/driverless/dv-controls-docs/_build/index.html>

**Nothing here is measured.** CMR's docs publish no lap times, tracking-error figures or A/B results for any of their design choices. They also say it is an open question whether their actuator-delay modelling helped at all. Every claim about how well something works is therefore CMR's description, UNVERIFIED by any number, unless it is marked as a measurement from this project.

## Contents

- [MPPI is a sampling-based MPC that searches over commands](#mppi-is-a-sampling-based-mpc-that-searches-over-commands)
- [MPPI perturbs the commands because they are what it decides](#mppi-perturbs-the-commands-because-they-are-what-it-decides)
- [CMR chose MPPI because its model and cost cannot be differentiated](#cmr-chose-mppi-because-its-model-and-cost-cannot-be-differentiated)
- [MPPI trades precision for flexibility](#mppi-trades-precision-for-flexibility)
- [CMR replays commands through real time gaps to compensate for delay](#cmr-replays-commands-through-real-time-gaps-to-compensate-for-delay)
- [This project took two ideas and held off the rest](#this-project-took-two-ideas-and-held-off-the-rest)

## MPPI is a sampling-based MPC that searches over commands

MPPI (Model Predictive Path Integral control) is a form of MPC (model predictive control: plan a short command sequence by predicting the car with a model, apply the first command, replan next tick). Instead of solving for the best sequence with gradients, it tries many random sequences and averages them, weighted by how well each one scores. CMR bases theirs on Williams et al., "Model Predictive Path Integral Control using Covariance Variable Importance Sampling".

**The loop, once per control tick:**

1. Take the current state estimate. CMR's state is `x`, `y`, yaw and speed.
2. Take the previous tick's best command sequence (steering angle and wheel torque per step) and add zero-mean random noise to it, many times over, to make `num_samples` candidate sequences.
3. Roll out every candidate: apply the dynamics model step by step to get a predicted trajectory.
4. Score every trajectory with the cost function, giving a cost-to-go `J` (the total cost of a state and every state after it).
5. Average the candidates step by step, each weighted by `exp(-J / lambda - D)`. `lambda` is a temperature (low: trust only the best samples; high: average broadly). `D` is the log of the sampled noise's probability density. CMR's docs state that this gives unlikely perturbations more weight, because they explore more.
6. Send the first command of the averaged sequence. Keep the rest, shifted one step, as the next tick's starting sequence (a [warm start](../reference/glossary.md)).

**The noise is a search, not a robustness measure.** It does not model sensor noise or a disturbance on the car. It is how the method explores the space of possible command sequences around its current best guess. With enough samples the weighted average moves toward lower cost, in the way a gradient step would for a smooth problem.

## MPPI perturbs the commands because they are what it decides

- **Commands are the decision variable.** The controller's output is a command sequence, so the search has to vary commands.
- **The pose is a measured starting point.** Perturbing it would score trajectories that start from somewhere the car is not. Averaging those would not produce a better command for the car that exists.
- **Why it warm-starts:** consecutive ticks face almost the same problem, so the previous plan shifted by one step is already close to the new answer. Sampling around it spends the samples where the answer probably is. Starting from scratch each tick would need far more samples for the same quality.
- **When there is no previous plan:** on the first tick, or after a reset, there is nothing to shift. The general practice is to start from a default sequence (for example zero steering and zero torque) and let the samples move it. CMR's docs do not say which default they use.

## CMR chose MPPI because its model and cost cannot be differentiated

CMR's stated reasons are that their dynamics model is not differentiable, their objective is non-convex, MPPI is fast and simple, and each sample can run in parallel on their onboard GPU. The model and the cost explain the first two.

**The model is a kinematic bicycle with no tyre slip.** It assumes no lateral or longitudinal slip, uniform circular motion between steps, and negligible wheel inertia. Understeer enters only through a tuned understeer gradient `kappa_us` (measured by skidpad runs at several speeds):

```
delta_hat = delta / (1 + kappa_us * v)        # effective steering angle
r_rear    = (L_front + L_rear) / tan(delta_hat)
alpha     = atan(L_rear / r_rear)              # body slip angle
v_next    = | v + F_net / m * dt |             # speed, never negative
```

The car then moves along the arc of radius `r_rear` for one step.

- **The absolute value has a kink at zero speed.** Its derivative jumps there, so a gradient-based solver cannot use it near a standstill.
- **The arc radius is infinite at zero steering.** `tan(delta_hat) = 0` makes `r_rear` undefined, so straight-line motion needs a separate branch. That is a second non-smooth point. The formula implies this branch, but their page does not show it.

**The cost is a lookup table with no useful gradient.** It is the larger reason. CMR turns the planner's path into a GPU texture once per path:

- **What each pixel holds:** an RGBA value (four channels per pixel). R is distance along the path, G is distance from the path, B is yaw relative to the path (unused), and A is 1 inside the track and -1 outside.
- **How it is drawn:** "fake boundary points" are placed either side of the path at a deliberately oversized track width, and the graphics hardware interpolates between them.
- **How overlaps are resolved:** each vertex gets a depth equal to its distance from the path. The depth test (the graphics pipeline's hidden-surface check) then keeps the value closest to the path wherever regions overlap.
- **How it is used:** every rollout state reads its along-path and across-path position straight from the texture.

That table is piecewise and sampled, so its gradients are not meaningful. Together with an in-bounds flag that jumps from 1 to -1, the cost is non-convex and non-smooth, which a gradient method such as an SQP (sequential quadratic programming, see [Gauss-Newton SQP](../reference/glossary.md)) cannot handle well. MPPI only needs to evaluate the cost, never differentiate it.

## MPPI trades precision for flexibility

| | MPPI as CMR uses it | Gradient-based NMPC (an SQP on a smooth problem) |
|---|---|---|
| Cost per state | One texture lookup on the GPU | Projection onto the path plus Jacobians |
| Noisy path | Rollouts run in `x`/`y`, so curvature noise never enters the prediction | A [Frenet-frame](../reference/glossary.md) prediction integrates `kappa`, so noise in it becomes predicted error |
| Off-track states | Still scored, because the oversized fake bounds keep the table defined there | Needs a soft constraint or slack |
| Cost shape | Anything that can be evaluated | Must be smooth enough to linearise |
| Implementation | Simple: sample, roll out, average | Condensing, QP set-up, trust regions, line search |
| Local minima | Less prone, because samples spread across the space | Can settle in the nearest minimum |
| Output smoothness | Noisier, from finite sampling | Smooth |
| Hardware | Needs a GPU for enough samples in real time | Runs on a CPU |
| Convergence | Approximate, limited by sample count | Precise on a smooth problem |

The comparison is qualitative. CMR publishes no measurements for either column, and this project has not run MPPI.

## CMR replays commands through real time gaps to compensate for delay

**Why it exists:** CMR's track tests showed delay on both sides of the controller. Upstream is LIDAR capture to the path reaching the controller. Downstream is the command being sent to the actuators acting. At speed the car travels a significant distance in that time, so planning from the last measured state plans from the past.

**What it does:** it predicts the car's state at the current time from the stale measurement, using the same model as the controller.

**How it works:**

1. Each new path defines a new frame with the car at its origin.
2. A buffer sorted by timestamp records three kinds of entry: measured speed, each command sent, and the pose (zero in each new path's frame).
3. Each command is stamped at its send time plus an `approx_propagation_delay`, so it enters the record when it is expected to act, not when it was sent.
4. Records older than the latest pose are discarded, except the most recent speed and command.
5. To get the current state, the model is stepped from the latest pose to each following record in time order, through the real time gap between them. At each record the current speed or command is updated, until the current time is reached.

CMR also measured the steering rack's slew rate and the torque response lag, and project the steering angle and torque forward with them. They report that it is an open question whether this helped.

**How this differs from this project's rollforward:** this project measures the pose's age, converts it to a whole number of control ticks (`n_delay`, filtered and capped at 3), and replays that many of its last commands, each for one full tick. CMR replays each command for the time it was active and separates actuation delay from sensing delay. This project counts only how old the pose is. The mechanism is in [lmpc.md](../controllers/lmpc.md) ("Delay compensation"), and the failure it contributed to is in [periodic_pose_teleport_investigation.md](../logs/periodic_pose_teleport_investigation.md).

## This project took two ideas and held off the rest

**Taken:**

- **The pose-age clock fix.** Reviewing how CMR timestamps its records led to checking how this project measures [pose age](../reference/glossary.md). The age was read across two clocks, and their drift pinned `n_delay` at its cap on fresh poses. It is now timed on the controller's own clock. See the "Delay-compensation fix" section of [periodic_pose_teleport_investigation.md](../logs/periodic_pose_teleport_investigation.md). Not yet merged to `main`: it is on the feat/cmr-controls-learnings branch.
- **A world-frame NMPC model, now the default.** It applies CMR's "roll out in `x`/`y`, measure path error only in the cost" idea to this project's NMPC. See [nmpc.md](../controllers/nmpc.md), "An experimental world-frame model keeps curvature out of the prediction". Not yet merged to `main`: it is on the feat/nmpc-cartesian-model branch, which also carries the clock fix.

**Held off:**

| Idea | Why it is held off |
|---|---|
| Rollforward by exact elapsed time instead of whole ticks | With the clock fix, `n_delay` should sit at 0 or 1, so the gain is small. Needs live data first |
| A separate actuation delay | FSDS's command-to-wheel delay has never been measured |
| Stamping poses when `sim_perception` receives them | Would also count the 30 to 45 ms the sample waits in `sim_perception`, but changes the stamp's meaning for every node that reads it |
| An SVM midline (a support vector machine fit between the blue and yellow cones, smooth by construction) | A planner change. The known centreline-curvature defect and the controller workarounds tuned around it must be re-checked first |
| A heartbeat watchdog that restarts dead nodes before driving | A production and kart robustness concern more than a simulation one |

**Why not MPPI itself:** this project's model is differentiable, its SQP NMPC already works, and the stack runs in Python on a CPU, where MPPI could not afford enough samples per tick.
