"""Start here. Ordinary Python/NumPy: no ROS-specific algorithm required.

Replace Processor.process(), add modules, or reorganize the package as needed.
The return value is currently ignored: choose your own output format and viewer.
"""
class Processor:
    def reset(self):
        """Clear tracking/map state here when a recording loops backwards."""
        pass

    def process(self, xyz, *, stamp_seconds, frame_id):
        """xyz: finite float32 NumPy array (N, 3), metres, in frame_id.

        X forward, Y left, Z up in lidar_link. No height/range/obstacle filters
        are imposed. Timestamp is copied from the input cloud, not wall time.
        Add filtering, clustering, grids, tracking, or your own visualization.
        Return a result if desired, then handle it in lidar_node.py.
        """
        return None
