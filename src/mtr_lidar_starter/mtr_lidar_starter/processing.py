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
    def __init__(self, *, size_m=20.0, min_height=-1.0,
                 max_height=2.0, min_range=0.5, density_scale=10.0):
        values = (size_m, min_height, max_height, min_range, density_scale)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Costmap settings must be finite')
        if size_m <= 0 or min_height > max_height or density_scale <= 0:
            raise ValueError('Invalid size, height interval, or density scale')
        if min_range < 0:
            raise ValueError('Minimum range must be nonnegative')
        self.cells = 100
        self.resolution = size_m / self.cells
        self.origin = -size_m / 2
        self.min_height, self.max_height = min_height, max_height
        self.min_range = min_range
        self.density_scale = density_scale
        self.max_distance = math.sqrt(2) * size_m / 2

    def reset(self):
        """No accumulated map state: every scan constructs a fresh grid."""

    def process(self, xyz, *, stamp_seconds, frame_id):
        """Return costs: -1 unknown, 1..100 concentration/proximity score.

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
            flat = indices[:, 1] * self.cells + indices[:, 0]
            counts = np.bincount(flat, minlength=self.cells ** 2)
            distance_sums = np.bincount(
                flat, weights=np.hypot(points[:, 0], points[:, 1]),
                minlength=self.cells ** 2)
            occupied = counts > 0
            mean_distance = distance_sums[occupied] / counts[occupied]
            # Equal-area tiles: point count measures per-scan concentration.
            # Saturation avoids one unusually dense tile rescaling the whole map.
            density = counts[occupied] / (counts[occupied] + self.density_scale)
            proximity = 1 - np.clip(mean_distance / self.max_distance, 0, 1)
            # More points and closer returns both increase cost. Distance halves
            # the score at the farthest corner instead of erasing distant hits.
            scores = 100 * density * (0.5 + 0.5 * proximity)
            costs.ravel()[occupied] = np.clip(np.rint(scores), 1, 100).astype(np.int16)
        return Costmap(costs, self.resolution, self.origin, self.origin,
                       stamp_seconds, frame_id, len(points))
