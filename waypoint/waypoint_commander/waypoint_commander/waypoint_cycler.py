import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.msg import BehaviorTreeLog
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import time

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')

        # Subscribe to global costmap to detect frontiers
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',  # Adjust topic if necessary
            self.costmap_callback,
            10)
        
        # Subscribe to the state of the robot
        self.subscription = self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.bt_log_callback, 10)

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)


        # Publish waypoint
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)  # Publisher for the waypoint
       
        # Placeholder for robot's current position (update with actual position if needed)
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.callback_first = 1

        # State of robot
        self.is_idle = True

        self.idle_first = False

        self.visited_points = []

        # Prevent unused variable warning
        self.subscription

    def odom_callback(self, msg):
        """
        Callback for the odometry subscriber.
        Updates the robot's current position (x, y) and orientation.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

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

        #First, go to the farthest point
        #Then, go follow the closest frontier

        # Find the closest frontier and publish it as a waypoint
        if frontiers and self.is_idle and self.callback_first:
            self.is_idle = False
            print("Frontier detected")
            farthest_frontier = self.find_farthest_frontier(frontiers, origin_x, origin_y, resolution)
            self.publish_waypoint(farthest_frontier, origin_x, origin_y, resolution)
            self.current_waypoint = farthest_frontier
            self.callback_first = 0

        elif frontiers and not self.callback_first and self.idle_first:
            closest_frontier = self.find_closest_frontier(frontiers, origin_x, origin_y, resolution)
            self.publish_waypoint(closest_frontier, origin_x, origin_y, resolution)

        

    def bt_log_callback(self, msg):
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
                self.is_idle = True
                self.idle_first = True
            # Checks for failure on reaching a waypoint
            if event.node_name == 'NavigateRecovery':
                self.get_logger().info(f"Status: {event.current_status}")
        
 

    def find_frontiers(self, data, width, height):
        """
        Find frontiers in the costmap.
        Frontiers are areas where free space (value 0) is adjacent to unknown space (value -1).
        """
        frontier_grid = [['  ' for _ in range(width)] for _ in range(height)]  # Create an empty grid
        frontiers = []  # List to store coordinates of frontier cells
        
        for y in range(height):
            for x in range(width):
                idx = x + y * width
                if data[idx] >= 0 and data[idx] < 40:  #The costmap value. Might need to change for frontiers at narrow places.
                    if self.is_frontier(x, y, data, width, height):
                        frontier_grid[y][x] = ' +'
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
        min_distance = math.inf
        closest_frontier = None

        for x, y in frontiers:
            # Convert grid coordinates to world coordinates
            frontier_x = origin_x + x * resolution
            frontier_y = origin_y + y * resolution

            # Calculate the Euclidean distance between the robot and the frontier
            distance = math.sqrt((frontier_x - self.robot_x) ** 2 + (frontier_y - self.robot_y) ** 2)

            if distance < min_distance and (frontier_x, frontier_y) not in self.visited_points:
                min_distance = distance
                closest_frontier = (frontier_x, frontier_y)

        return closest_frontier
    
    def find_farthest_frontier(self, frontiers, origin_x, origin_y, resolution):
        """
        Find the farthest frontier to the robot's current position.
        """
        max_distance = 0
        farthest_frontier = None

        for x, y in frontiers:
            # Convert grid coordinates to world coordinates
            frontier_x = origin_x + x * resolution
            frontier_y = origin_y + y * resolution

            # Calculate the Euclidean distance between the robot and the frontier
            distance = math.sqrt((frontier_x - self.robot_x) ** 2 + (frontier_y - self.robot_y) ** 2)

            if distance > max_distance:
                max_distance = distance
                farthest_frontier = (frontier_x, frontier_y)

        return farthest_frontier

    def publish_waypoint(self, frontier, origin_x, origin_y, resolution):
        """
        Publish the closest frontier as a waypoint using a PoseStamped message.
        """

        if frontier in self.visited_points:
            self.get_logger().info(f"Point {frontier} already visited, skipping.")
            return
        
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

            self.visited_points.append(frontier)



def main(args=None):
    rclpy.init(args=args)
    frontier_detector = FrontierDetector()
    rclpy.spin(frontier_detector)
    frontier_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
