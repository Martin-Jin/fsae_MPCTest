"""
Lightweight CSV telemetry logger for the control nodes (debug/tuning aid).

Both controllers (Stanley, MPC) can write two compact CSVs so we can separate
planner problems from controller problems offline:

  <tag>_control_<stamp>.csv  @ control rate — one row per control step:
      t, car_x, car_y, car_yaw, v_actual, v_desired, steer_deg, e_y, e_psi_deg, yaw_rate
  <tag>_path_<stamp>.csv     @ ~1 Hz — snapshots of the planned path (long form):
      t, idx, x, y

Sign convention (SHARED by both controllers so the A/B plots overlay):
  e_y   > 0  → car is to the LEFT of the path (metres)
  e_psi > 0  → car heading is rotated CCW (left) relative to the path tangent

Why CSV and not rosbag: the tracking errors e_y / e_psi are computed inside the
controller and never published to a topic, so rosbag can't see them — yet they
are exactly what distinguishes "path is wiggly" (small e_y, wiggly path) from
"controller oscillates" (steer + e_y swing on a smooth path).  CSV is also far
smaller (~100 KB/min) and directly plottable.
"""
import csv
import math
import os
import time


class ControlLogger:
    def __init__(self, tag: str, log_dir: str = '', path_period: float = 1.0):
        log_dir = os.path.expanduser(log_dir) if log_dir else os.path.expanduser('~/fsae_logs')
        os.makedirs(log_dir, exist_ok=True)
        stamp = int(time.time())
        self._ctrl_path = os.path.join(log_dir, f'{tag}_control_{stamp}.csv')
        self._path_path = os.path.join(log_dir, f'{tag}_path_{stamp}.csv')

        self._ctrl_f = open(self._ctrl_path, 'w', newline='')
        self._path_f = open(self._path_path, 'w', newline='')
        self._ctrl_w = csv.writer(self._ctrl_f)
        self._path_w = csv.writer(self._path_f)
        self._ctrl_w.writerow(
            ['t', 'car_x', 'car_y', 'car_yaw', 'v_actual', 'v_desired',
             'steer_deg', 'e_y', 'e_psi_deg', 'yaw_rate'])
        self._path_w.writerow(['t', 'idx', 'x', 'y'])

        self._path_period = path_period
        self._last_path_t: float | None = None
        self._n = 0

    @property
    def paths(self) -> tuple[str, str]:
        return self._ctrl_path, self._path_path

    def log_control(self, t, car_x, car_y, car_yaw, v_actual, v_desired,
                    steer_rad, e_y, e_psi_rad, yaw_rate) -> None:
        self._ctrl_w.writerow([
            f'{t:.4f}', f'{car_x:.4f}', f'{car_y:.4f}', f'{car_yaw:.5f}',
            f'{v_actual:.3f}', f'{v_desired:.3f}', f'{math.degrees(steer_rad):.3f}',
            f'{e_y:.4f}', f'{math.degrees(e_psi_rad):.3f}', f'{yaw_rate:.4f}',
        ])
        self._n += 1
        if self._n % 20 == 0:          # flush ~1 s so a Ctrl-C leaves valid data
            self._ctrl_f.flush()

    def log_path(self, t, path) -> None:
        if self._last_path_t is not None and (t - self._last_path_t) < self._path_period:
            return
        self._last_path_t = t
        for i, pt in enumerate(path):
            self._path_w.writerow([f'{t:.4f}', i, f'{float(pt[0]):.4f}', f'{float(pt[1]):.4f}'])
        self._path_f.flush()

    def close(self) -> None:
        for f in (self._ctrl_f, self._path_f):
            try:
                f.close()
            except Exception:
                pass
