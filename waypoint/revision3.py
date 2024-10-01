import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        # Subscribe to the global or local costmap topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',  # or '/local_costmap/costmap'
            self.costmap_callback,
            10)
        self.subscription  # prevent unused variable warning

    def costmap_callback(self, msg):
        # Extract the costmap data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        # Call function to find frontiers
        frontier_grid = self.find_frontiers(data, width, height)

        # Print the frontier grid
        self.print_frontier_grid(frontier_grid, width, height)

    def find_frontiers(self, data, width, height):
        """
        Find frontiers in the costmap.
        Frontiers are areas where free space (value 0) is adjacent to unknown space (value -1).
        """
        frontier_grid = [['  ' for _ in range(width)] for _ in range(height)]  # Create an empty grid
        
        for y in range(height):
            for x in range(width):
                idx = x + y * width
                # Check if the current cell is free space (value 0)
                if data[idx] == 0:
                    # Check if it's adjacent to unknown space (value -1)
                    if self.is_frontier(x, y, data, width, height):
                        frontier_grid[x][y] = ' +'  # Mark as a frontier

        return frontier_grid

    def is_frontier(self, x, y, data, width, height):
        """
        Check if a given free space cell is adjacent to unknown space.
        """
        # Define neighbor offsets for 4-connected grid (up, down, left, right)
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        for dx, dy in neighbors:
            nx = x + dx
            ny = y + dy
            
            if 0 <= nx < width and 0 <= ny < height:
                idx = nx + ny * width
                if data[idx] == -1:  # Unknown space
                    return True
        return False

    def print_frontier_grid(self, frontier_grid, width, height):
        """
        Print the frontier grid to the terminal, with ' +' for frontiers and ' ' for non-frontiers.
        """
        print("\nFrontier Grid:\n")
        for y in range(height):
            row = ''.join(frontier_grid[y])
            print(row)


def main(args=None):
    rclpy.init(args=args)
    frontier_detector = FrontierDetector()
    rclpy.spin(frontier_detector)
    frontier_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
