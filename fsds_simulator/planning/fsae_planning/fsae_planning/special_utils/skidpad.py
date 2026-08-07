"""
Skidpad (figure-8) track geometry.

The FS skidpad is two circles driven as a figure-8.  This module reconstructs
that geometry from the accumulated cone map and builds a single closed,
tangent-continuous centreline the car can lap repeatedly:

build_figure8
    Cluster the cones into the two circles (2-means), fit a circle to each
    (algebraic Kåsa fit), and stitch them into one figure-8 loop that crosses
    cleanly at the point between the two circle centres.

path_deviation
    Shortest distance from the car to the centreline — used to decide when the
    car has slid off the lane ("spun off course").

The figure-8 loop is returned in track order (last point ≈ first point) so it
drops straight into path_utils.roll_loop_to_car for a forward planning window.
"""
import numpy as np

# Default lane half-width (centreline → cone line) used only if the cone radial
# spread is degenerate.  Real skidpad lane width is ~3 m → ~1.5 m half-width.
_DEFAULT_HALF_WIDTH = 1.5
_MIN_CONES          = 8     # need at least this many cones to attempt a fit


class Figure8Track:
    """A reconstructed figure-8 centreline plus the geometry it was built from."""

    __slots__ = ('loop', 'centres', 'lane_radius', 'half_width')

    def __init__(self, loop, centres, lane_radius, half_width):
        self.loop        = loop          # (N, 2) closed centreline, track order
        self.centres     = centres       # (2, 2) the two circle centres
        self.lane_radius = lane_radius   # mean centreline radius (m)
        self.half_width  = half_width    # centreline → cone line distance (m)


def fit_circle(points: np.ndarray) -> tuple[np.ndarray, float]:
    """
    Algebraic (Kåsa) least-squares circle fit.

    Solves x² + y² = 2·a·x + 2·b·y + c for (a, b, c); the centre is (a, b) and
    the radius is sqrt(c + a² + b²).  Returns (centre[2], radius).
    """
    x = points[:, 0]
    y = points[:, 1]
    A = np.column_stack([2.0 * x, 2.0 * y, np.ones(len(points))])
    rhs = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
    centre = sol[:2]
    radius = float(np.sqrt(max(0.0, sol[2] + centre[0] ** 2 + centre[1] ** 2)))
    return centre, radius


def _fit_circle_pair(cones: np.ndarray, iters: int = 40):
    """
    Fit the two skidpad circles to the pooled cones.

    Clustering the cones by nearest centre does not work: each circle's *outer*
    ring reaches past the midpoint between the two centres, so a nearest-centre
    split steals each circle's near cones for its neighbour and drags the fitted
    centres apart.  What is stable is which *ring* a cone lies on, so this is an
    EM over ring membership: each circle is described by (centre, r_inner,
    r_outer) and every cone is assigned to the circle whose nearer ring it sits
    closest to.  A cone where one circle's outer ring meets the other's inner
    ring is genuinely shared by both rings, so assigning it either way leaves it
    exactly on the ring it lands in and does not bias that fit.

    Each circle's centre is recovered by fitting its inner and outer rings
    separately and averaging: a single algebraic fit over both concentric rings
    at once is biased (it trades centre position against a radius that matches
    neither ring).

    Seeded by splitting the cones across their principal axis (the line through
    the two centres).  Returns [(centre, r_inner, r_outer), ...] ordered left→
    right by centre x, or None if it cannot form two circles.
    """
    if len(cones) < _MIN_CONES:
        return None

    m = cones.mean(axis=0)
    _, _, vt = np.linalg.svd(cones - m)
    axis = vt[0]                                   # line through the two centres
    labels = (((cones - m) @ axis) > 0.0).astype(int)

    params = None
    for _ in range(iters):
        params = []
        for k in (0, 1):
            g = cones[labels == k]
            if len(g) < 3:
                return None
            c, _ = fit_circle(g)
            d = np.linalg.norm(g - c, axis=1)
            mid = 0.5 * (d.min() + d.max())
            inner, outer = g[d < mid], g[d >= mid]
            if len(inner) >= 3 and len(outer) >= 3:
                ci, _ = fit_circle(inner)
                co, _ = fit_circle(outer)
                c = 0.5 * (ci + co)
            d = np.linalg.norm(g - c, axis=1)
            params.append((c, float(d.min()), float(d.max())))

        residual = np.column_stack([
            np.minimum(np.abs(np.linalg.norm(cones - c, axis=1) - ri),
                       np.abs(np.linalg.norm(cones - c, axis=1) - ro))
            for c, ri, ro in params
        ])
        new_labels = residual.argmin(axis=1)
        if np.array_equal(new_labels, labels):
            break
        labels = new_labels

    if params is None:
        return None
    return sorted(params, key=lambda p: p[0][0])   # left → right by centre x


def _arc(centre: np.ndarray, radius: float,
         start_ang: float, sweep: float, n: int) -> np.ndarray:
    """Sample n points along a circular arc from start_ang sweeping `sweep` rad."""
    ang = start_ang + np.linspace(0.0, sweep, n, endpoint=False)
    return centre + radius * np.column_stack([np.cos(ang), np.sin(ang)])


def build_figure8(
    blue: np.ndarray,
    yellow: np.ndarray,
    n_per_circle: int = 180,
) -> Figure8Track | None:
    """
    Reconstruct the figure-8 centreline from the cone map.

    Cone colour is irrelevant to the geometry (both circles use both colours as
    inner/outer boundaries), so blue and yellow are pooled.  The two circles are
    fitted together by ring membership (see _fit_circle_pair) and stitched into a
    single figure-8 that crosses at the midpoint between the centres.

    The two centreline circles of an FS skidpad are tangent (centres 2·R apart),
    so the centreline radius is pinned to half the centre separation.  This makes
    the circles kiss exactly at the midpoint — the arcs share that crossing point
    with no gap — and keeps each circle's path concentric with its cone rings so
    it stays centred between the inner and outer cones the whole way round.

    The crossing is built tangent-continuous: circle 0 is traversed CCW starting
    from the point facing circle 1, and circle 1 is traversed CW starting from
    the point facing circle 0, so travel direction through the crossing matches
    on both passes (the defining property of a figure-8).

    Returns a Figure8Track, or None if there are too few cones to fit.
    """
    cones = [c for c in (blue, yellow) if len(c) > 0]
    if not cones:
        return None
    pts = np.vstack(cones)

    circles = _fit_circle_pair(pts)
    if circles is None:
        return None
    (c0, rin0, rout0), (c1, rin1, rout1) = circles

    u = c1 - c0
    u_norm = float(np.linalg.norm(u))
    if u_norm < 1e-6:
        return None
    u = u / u_norm

    # Tangent circles: centreline radius is half the centre separation, so both
    # circles pass through the midpoint and the figure-8 closes with no gap.
    radius = 0.5 * u_norm
    half_width = float(np.mean([(rout0 - rin0) * 0.5, (rout1 - rin1) * 0.5]))

    ang0 = float(np.arctan2(u[1], u[0]))          # c0 → point facing c1 (= midpoint)
    ang1 = ang0 + np.pi                            # c1 → point facing c0 (= midpoint)

    # Circle 0 CCW (+sweep), circle 1 CW (-sweep): travel through the crossing
    # is the same direction on both, giving a true figure-8 rather than an O.
    arc0 = _arc(c0, radius, ang0, 2.0 * np.pi,  n_per_circle)
    arc1 = _arc(c1, radius, ang1, -2.0 * np.pi, n_per_circle)
    loop = np.vstack([arc0, arc1, arc0[:1]])       # close back to the start

    if not np.isfinite(half_width) or half_width < 1e-3:
        half_width = _DEFAULT_HALF_WIDTH

    return Figure8Track(
        loop=loop,
        centres=np.array([c0, c1]),
        lane_radius=radius,
        half_width=half_width,
    )


def path_deviation(loop: np.ndarray, car_pos: np.ndarray) -> float:
    """Shortest distance from car_pos to the centreline polyline (metres)."""
    if loop is None or len(loop) < 2:
        return float('inf')

    a = loop[:-1]
    b = loop[1:]
    ab = b - a
    ab2 = np.einsum('ij,ij->i', ab, ab)
    ab2 = np.where(ab2 < 1e-12, 1.0, ab2)
    t = np.einsum('ij,ij->i', car_pos - a, ab) / ab2
    t = np.clip(t, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return float(np.min(np.linalg.norm(car_pos - proj, axis=1)))
