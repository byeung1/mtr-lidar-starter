"""Pygame costmap viewer; ROS callbacks and drawing share the main thread."""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from .lidar_node import LidarNode


def cost_colors(costs):
    """Gray unknown; green ray-observed space; red near-height returns."""
    colors = np.full((*costs.shape, 3), (60, 210, 55), dtype=np.uint8)
    colors[costs < 0] = (100, 105, 110)
    colors[costs > 0] = (245, 55, 55)
    return colors


def main():
    import pygame
    rclpy.init()
    pygame.init()
    node = None
    try:
        latest = [None]
        node = LidarNode(result_callback=lambda result: latest.__setitem__(0, result))
        screen = pygame.display.set_mode((800, 870))
        pygame.display.set_caption('LiDAR local costmap')
        font = pygame.font.Font(None, 24)
        clock = pygame.time.Clock()
        running = True
        rendered = None
        surface = None
        while running and rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    running = False
            if not running:
                break
            rclpy.spin_once(node, timeout_sec=0.0)
            screen.fill((24, 28, 34))
            result = latest[0]
            if result is not None:
                if result is not rendered:
                    costs = result.costs
                    colors = cost_colors(costs)
                    # Screen right is -Y; screen up is +X (forward).
                    surface = pygame.surfarray.make_surface(colors[::-1, ::-1, :])
                    surface = pygame.transform.scale(surface, (800, 800))
                    # Each of the 100 x 100 tiles is eight screen pixels wide.
                    for pixel in range(0, 801, 8):
                        pygame.draw.line(surface, (30, 34, 40), (pixel, 0), (pixel, 800))
                        pygame.draw.line(surface, (30, 34, 40), (0, pixel), (800, pixel))
                    rendered = result
                screen.blit(surface, (0, 0))
                width = result.costs.shape[1] * result.resolution
                sx = round((result.origin_y + width) / width * 800)
                sy = round((result.origin_x + width) / width * 800)
                pygame.draw.circle(screen, (75, 210, 255), (sx, sy), 6)
                pygame.draw.line(screen, (75, 210, 255), (sx, sy), (sx, sy - 22), 3)
                label = f'{result.frame_id} | 100 x 100 | {result.resolution:g} m/cell | {result.obstacle_points:,} points'
            else:
                label = 'Waiting for /lidar/points...'
            screen.blit(font.render(label, True, (235, 235, 235)), (12, 810))
            screen.blit(font.render('Up: +X  Left: +Y  Green: observed  Red: at LiDAR height  Gray: unknown', True, (235, 235, 235)), (12, 840))
            pygame.display.flip()
            clock.tick(30)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
