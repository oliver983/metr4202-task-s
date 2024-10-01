import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import math

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        # Subscribe to global costmap to detect frontiers
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',  # Adjust topic if necessary
            self.costmap_callback,

            10)
        
        # Publish waypoint
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)  # Publisher for the waypoint
       
        # Placeholder for robot's current position (update with actual position if needed)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Prevent unused variable warning
        self.subscription

    def costmap_callback(self, msg):
        # Extract the costmap data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        # Call function to find frontiers
        frontier_grid, frontiers = self.find_frontiers(data, width, height)

        # Print the frontier grid
        #self.print_frontier_grid(frontier_grid, width, height)

        # Find the closest frontier and publish it as a waypoint
        if frontiers:
            print("Frontier detected")
            closest_frontier = self.find_closest_frontier(frontiers, origin_x, origin_y, resolution)
            self.publish_waypoint(closest_frontier, origin_x, origin_y, resolution)

    def find_frontiers(self, data, width, height):
        """
        Find frontiers in the costmap.
        Frontiers are areas where free space (value 0) is adjacent to unknown space (value -1).
        """
        frontier_grid = [[' ' for _ in range(width)] for _ in range(height)]  # Create an empty grid
        frontiers = []  # List to store coordinates of frontier cells
        
        for y in range(height):
            for x in range(width):
                idx = x + y * width
                if data[idx] == 0:  # Free space
                    if self.is_frontier(x, y, data, width, height):
                        frontier_grid[y][x] = '+'
                        frontiers.append((x, y))  # Store the frontier coordinates

        return frontier_grid, frontiers

    def is_frontier(self, x, y, data, width, height):
        """
        Check if a given free space cell is adjacent to unknown space (-1).
        """
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
        Print the frontier grid to the terminal, with '+' for frontiers and ' ' for non-frontiers.
        """
        print("\nFrontier Grid:\n")
        for y in range(height):
            row = ''.join(frontier_grid[y])
            print(row)

    def find_closest_frontier(self, frontiers, origin_x, origin_y, resolution):
        """
        Find the closest frontier to the robot's current position.
        """
        min_distance = float('inf')
        closest_frontier = None

        for x, y in frontiers:
            # Convert grid coordinates to world coordinates
            frontier_x = origin_x + x * resolution
            frontier_y = origin_y + y * resolution

            # Calculate the Euclidean distance between the robot and the frontier
            distance = math.sqrt((frontier_x - self.robot_x) ** 2 + (frontier_y - self.robot_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                closest_frontier = (frontier_x, frontier_y)

        return closest_frontier

    def publish_waypoint(self, frontier, origin_x, origin_y, resolution):
        """
        Publish the closest frontier as a waypoint using a PoseStamped message.
        """
        if frontier:
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.header.stamp = self.get_clock().now().to_msg()
            
            
            waypoint.pose.position.x = frontier[0]
            waypoint.pose.position.y = frontier[1]
            waypoint.pose.position.z = 0.0
            waypoint.pose.orientation.w = 1.0  # Neutral orientation

            # Log and publish the waypoint
            self.get_logger().info(f"Publishing waypoint to frontier at ({frontier[0]}, {frontier[1]})")
            self.publisher.publish(waypoint)


def main(args=None):
    rclpy.init(args=args)
    frontier_detector = FrontierDetector()
    rclpy.spin(frontier_detector)
    frontier_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
