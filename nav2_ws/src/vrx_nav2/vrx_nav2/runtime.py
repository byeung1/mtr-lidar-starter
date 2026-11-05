import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException


def spin(factory):
    rclpy.init()
    node = factory()
    # Cloud callbacks do not wait for TF; they use a bounded pending queue.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # A signal can invalidate the rclpy context while take_message is active.
        # Preserve real runtime failures; suppress only an already-shutdown context.
        if rclpy.ok():
            raise
    finally:
        if hasattr(node, 'neutral') and rclpy.ok():
            node.neutral()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
