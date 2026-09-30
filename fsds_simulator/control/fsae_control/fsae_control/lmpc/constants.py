"""
fsae_control/lmpc/constants.py — physical limits shared by the LTV-QP and NMPC controllers

The MPC math elsewhere in lmpc/ is a byte-for-byte port of the offline stack so
tuned weights transfer. The one intentional difference is MAX_STEER_RAD, 25 deg
here (matching fsae_control.control_utils and fsds_bridge) against 35 deg
upstream, so the MPC plans steering the car can deliver. Re-apply that change
if re-syncing from upstream.
"""

import math

# Maximum physical steering deflection.  25deg matches this stack's limit
# (fsae_control.control_utils / fsds_bridge); upstream used 35deg.
MAX_STEER_RAD: float = math.radians(25.0)
MAX_ACCEL: float = 12.0
MAX_BRAKE: float = 7.0
