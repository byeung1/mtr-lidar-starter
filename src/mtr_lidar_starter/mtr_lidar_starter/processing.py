"""Per-scan obstacle costmap in the incoming LiDAR frame (NumPy only)."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Costmap:
    # Rows increase along +Y; columns increase along +X.
    costs: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    stamp_seconds: float
    frame_id: str
    obstacle_points: int


class Processor:
    def __init__(self, *, resolution=0.1, size_m=20.0, min_height=-1.0,
                 max_height=2.0, min_range=0.5, inflation_radius=0.6):
        values = (resolution, size_m, min_height, max_height, min_range,
                  inflation_radius)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Costmap settings must be finite')
        if resolution <= 0 or size_m <= 0 or min_height > max_height:
            raise ValueError('Invalid resolution, size, or height interval')
        if min_range < 0 or inflation_radius < 0:
            raise ValueError('Range and inflation radius must be nonnegative')
        self.resolution = resolution
        self.cells = math.ceil(size_m / resolution)
        self.origin = -self.cells * resolution / 2
        self.min_height, self.max_height = min_height, max_height
        self.min_range = min_range
        self.inflation_radius = inflation_radius
        # Precompute a circular inflation kernel with costs that fall with distance.
        radius_cells = math.ceil(inflation_radius / resolution)
        self.kernel = []
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                distance = math.hypot(dx, dy) * resolution
                if 0 < distance <= inflation_radius:
                    cost = max(1, min(99, math.ceil(99 * (1 - distance / inflation_radius))))
                    self.kernel.append((dy, dx, cost))

    def reset(self):
        """No accumulated map state: every scan constructs a fresh grid."""

    def process(self, xyz, *, stamp_seconds, frame_id):
        """Return costs: -1 unknown, 1..99 inflated, 100 obstacle.

        No free-space inference or pose accumulation is performed. Height is
        relative to the sensor, not a measured ground/water surface.
        """
        xyz = np.asarray(xyz, dtype=np.float32)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError('xyz must have shape (N, 3)')
        upper = self.origin + self.cells * self.resolution
        keep = (np.isfinite(xyz).all(axis=1)
                & (xyz[:, 2] >= self.min_height) & (xyz[:, 2] <= self.max_height)
                & (np.hypot(xyz[:, 0], xyz[:, 1]) >= self.min_range)
                & (xyz[:, 0] >= self.origin) & (xyz[:, 0] < upper)
                & (xyz[:, 1] >= self.origin) & (xyz[:, 1] < upper))
        points = xyz[keep]
        indices = np.floor((points[:, :2] - self.origin) / self.resolution).astype(int)
        # Clip roundoff at the upper boundary after metric bounds checking.
        indices = np.clip(indices, 0, self.cells - 1)
        costs = np.full((self.cells, self.cells), -1, dtype=np.int16)
        if len(indices):
            occupied = np.unique(indices, axis=0)
            cols, rows = occupied[:, 0], occupied[:, 1]
            costs[rows, cols] = 100
            for dy, dx, cost in self.kernel:
                rr, cc = rows + dy, cols + dx
                valid = (rr >= 0) & (rr < self.cells) & (cc >= 0) & (cc < self.cells)
                np.maximum.at(costs, (rr[valid], cc[valid]), cost)
        return Costmap(costs, self.resolution, self.origin, self.origin,
                       stamp_seconds, frame_id, len(points))
