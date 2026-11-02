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
    def __init__(self, *, size_m=20.0, min_height=-0.2,
                 max_height=0.2, min_range=0.5):
        values = (size_m, min_height, max_height, min_range)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Costmap settings must be finite')
        if size_m <= 0 or min_height > max_height:
            raise ValueError('Invalid size or height interval')
        if min_range < 0:
            raise ValueError('Minimum range must be nonnegative')
        self.cells = 100
        self.resolution = size_m / self.cells
        self.origin = -size_m / 2
        self.min_height, self.max_height = min_height, max_height
        self.min_range = min_range

    def reset(self):
        """No accumulated map state: every scan constructs a fresh grid."""

    def process(self, xyz, *, stamp_seconds, frame_id):
        """Return -1 unknown, 0 ray-observed space, 100 near-height return.

        Heights are sensor-relative. Only returns in the configured height band
        provide obstacle or clearing evidence; unobserved directions stay unknown.
        """
        xyz = np.asarray(xyz, dtype=np.float32)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError('xyz must have shape (N, 3)')
        upper = self.origin + self.cells * self.resolution
        keep = (np.isfinite(xyz).all(axis=1)
                & (xyz[:, 2] >= self.min_height) & (xyz[:, 2] <= self.max_height)
                & (np.hypot(xyz[:, 0], xyz[:, 1]) >= self.min_range))
        points = xyz[keep]
        in_grid = ((points[:, 0] >= self.origin) & (points[:, 0] < upper)
                   & (points[:, 1] >= self.origin) & (points[:, 1] < upper))
        obstacle_points = points[in_grid]
        indices = np.floor((obstacle_points[:, :2] - self.origin) / self.resolution).astype(int)
        indices = np.clip(indices, 0, self.cells - 1)
        costs = np.full((self.cells, self.cells), -1, dtype=np.int16)
        if len(indices):
            costs[indices[:, 1], indices[:, 0]] = 100
        if len(points):
            # Clip out-of-grid rays to the boundary without inventing edge obstacles.
            xy = points[:, :2].astype(np.float64)
            scale = np.minimum(1.0, (upper - self.resolution * 1e-6)
                               / np.maximum(np.max(np.abs(xy), axis=1), 1e-12))
            endpoints = np.floor((xy * scale[:, None] - self.origin)
                                 / self.resolution).astype(int)
            endpoints = np.unique(np.clip(endpoints, 0, self.cells - 1), axis=0)
            center = int(math.floor(-self.origin / self.resolution))
            for end_x, end_y in endpoints:
                # Bresenham walks each observed ray; stop at any obstacle tile.
                x, y = center, center
                dx, dy = abs(end_x - x), -abs(end_y - y)
                sx, sy = (1 if x < end_x else -1), (1 if y < end_y else -1)
                error = dx + dy
                while True:
                    if costs[y, x] == 100:
                        break
                    cell_x = self.origin + (x + 0.5) * self.resolution
                    cell_y = self.origin + (y + 0.5) * self.resolution
                    if math.hypot(cell_x, cell_y) >= self.min_range:
                        costs[y, x] = 0
                    if x == end_x and y == end_y:
                        break
                    twice_error = 2 * error
                    if twice_error >= dy:
                        error += dy
                        x += sx
                    if twice_error <= dx:
                        error += dx
                        y += sy
        return Costmap(costs, self.resolution, self.origin, self.origin,
                       stamp_seconds, frame_id, len(obstacle_points))
