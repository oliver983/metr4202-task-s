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
       
        # Placeholder for robot's current position and orientation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0  # Robot's current yaw (orientation)

        # State of robot
        self.is_idle = True
        self.visited_points = []
        self.radius = 150  # Starting radius

        # Prevent unused variable warning
        self.subscription

    def odom_callback(self, msg):
        """
        Callback for the odometry subscriber.
        Updates the robot's current position (x, y) and orientation (yaw).
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw (heading) from the quaternion orientation
        orientation = msg.pose.pose.orientation
        _, _, self.robot_yaw = self.euler_from_quaternion(orientation)

    def euler_from_quaternion(self, orientation):
        """
        Convert quaternion (x, y, z, w) to Euler angles (roll, pitch, yaw).
        """
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # Quaternion to Euler (yaw only, since we are interested in 2D)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return (0.0, 0.0, yaw)  # We only care about yaw for 2D navigation

    def costmap_callback(self, msg):
        """
        Callback for the costmap data, triggers frontier detection and goal sending.
        Waits until robot is idle before sending a new goal.
        """
        # Extract the costmap data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        # Call function to find frontiers
        frontier_grid, frontiers = self.find_frontiers(data, width, height)

        # Wait until the robot is idle before sending a waypoint
        if frontiers and self.is_idle:
            self.get_logger().info("Robot is idle, finding farthest frontier within the radius")
            farthest_frontier = self.find_farthest_frontier_with_angle_preference(
                frontiers, origin_x, origin_y, resolution, self.radius
            )
            self.radius -= 25
            if farthest_frontier:
                self.publish_waypoint(farthest_frontier, origin_x, origin_y, resolution)
                self.is_idle = False  # Set to false after sending a new waypoint
            else:
                self.get_logger().info("No frontier found within the radius, increasing.")
                self.radius += 50
                farthest_frontier = self.find_farthest_frontier_with_angle_preference(
                    frontiers, origin_x, origin_y, resolution, self.radius)

    def bt_log_callback(self, msg):
        """
        Callback for the behavior tree log, sets the robot's idle state.
        When the robot reaches the goal or fails, it becomes idle.
        """
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and event.current_status == 'IDLE':
                self.is_idle = True  # Robot is now idle, ready for the next waypoint
                self.get_logger().info("Robot is now idle, ready for the next frontier.")
            elif event.node_name == 'NavigateRecovery' and event.current_status != 'IDLE':
                self.is_idle = False  # Robot is busy or in motion

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
                if data[idx] >= 0 and data[idx] < 60:  # Costmap value (might need adjustment)
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

    def find_farthest_frontier_with_angle_preference(self, frontiers, origin_x, origin_y, resolution, radius):
        """
        Find the farthest frontier within a certain radius (in grid tiles),
        with a preference for frontiers aligned with the robot's current orientation.
        """
        max_distance = 0
        farthest_frontier = None
        best_angle_diff = math.pi  # Start with the maximum possible angle difference (180 degrees)

        for x, y in frontiers:
            # Convert grid coordinates to world coordinates
            frontier_x = origin_x + x * resolution
            frontier_y = origin_y + y * resolution

            # Calculate the distance to the frontier in grid tiles
            distance_in_tiles = math.sqrt((x - self.robot_x / resolution) ** 2 + (y - self.robot_y / resolution) ** 2)

            # Check if the distance is within the specified radius
            if distance_in_tiles <= radius and (frontier_x, frontier_y) not in self.visited_points:
                distance = math.sqrt((frontier_x - self.robot_x) ** 2 + (frontier_y - self.robot_y) ** 2)
                
                # Calculate the angle to the frontier
                angle_to_frontier = math.atan2(frontier_y - self.robot_y, frontier_x - self.robot_x)
                
                # Compute the difference between the robot's current angle and the target angle
                angle_diff = abs(self.normalize_angle(angle_to_frontier - self.robot_yaw))

                # Prioritize based on both distance and alignment with the current direction
                if distance > max_distance and angle_diff < best_angle_diff:
                    max_distance = distance
                    best_angle_diff = angle_diff
                    farthest_frontier = (frontier_x, frontier_y)

        return farthest_frontier

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_waypoint(self, frontier, origin_x, origin_y, resolution):
        """
        Publish the farthest frontier as a waypoint using a PoseStamped message.
        """
        
        if frontier in self.visited_points:
            self.get_logger().info(f"Point {frontier} already visited, skipping.")
            return
        
        if frontier:
            print(self.radius)
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
