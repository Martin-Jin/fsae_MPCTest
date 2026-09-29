"""
controller/lmpc/ — the linear time-varying MPC ("LMPC" in the docs, as
opposed to controller/nmpc/'s Frenet-frame nonlinear MPC).

  build.py   init_parameterized_mpc() — builds the parameterized CVXPY QP
  solve.py   solve_mpc() — the per-tick solve, plus the compiled-problem cache

Both existed as one file, controller/optimiser.py, until split for
readability; no behaviour change. See build.py's module docstring for the
full MPC design (cost function, warm start, parameterized formulation).
"""

from controller.lmpc.build import init_parameterized_mpc  # noqa: F401
from controller.lmpc.solve import solve_mpc  # noqa: F401
