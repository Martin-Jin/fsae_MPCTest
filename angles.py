"""
angles.py — the one angle-wrap helper, shared by controller/ and sim/.

Top-level, not nested in either package: controller/ never imports from
sim/ (sim/ imports from controller/, not the other way round), so a
shared helper needs a home neither package's existing dependency
direction rules out. Was duplicated as controller/nmpc/layout.py::_wrap
and sim/rollout/reference.py::_normalize_angle, byte-identical
implementations under two different names.
"""
import numpy as np


def wrap_angle(angle):
    """Wrap an angle to (−π, π] using atan2."""
    return np.arctan2(np.sin(angle), np.cos(angle))
