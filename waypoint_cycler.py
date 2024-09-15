import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import threading
import math


class navigationControl(Node):
    def __init__(self):
        super().__init__('Explorationcsc')
        # Subscriptions for map, odometry, and laser scan topics
        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)  # Nav2 goal topic

        self.get_logger().info(" active by but")

        # Initialize variables to store map, odometry, and scan data
        self.map_data = None
        self.resolution = None
        self.originX = None
        self.originY = None
        self.width = None
        self.height = None
        self.data = None
        self.grid = None

    def scan_callback(self, msg):
        # Store laser scan data
        self.scan_data = msg
        self.scan = msg.ranges

    def map_callback(self, msg):
        # Store map data
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = msg.data
        self.get_logger().info("check faf")

        # Format the map into a 2D grid
        self.map_formatting()

        # Process the map for frontier detection
        if self.map_data is not None:
            frontiers = self.get_frontiers(self.map_data)
            if frontiers:
                chosen_frontier = self.choose_frontier(frontiers)
                if chosen_frontier:
                    point = self.decide_point(chosen_frontier)
                    #self.get_logger().info(f'Navigating to point: {point}')
                    #elf.navigate_to_point(point)

    def odom_callback(self, msg):
        # Store odometry data
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

    def map_formatting(self):
        # Convert flat map data to a 2D grid
        map_data = self.data
        gridcounter = 0
        xl = self.width
        yl = self.height
        self.grid = [[0 for _ in range(yl)] for _ in range(xl)]
        for x in range(xl):
            for y in range(yl):
                self.grid[x][y] = map_data[gridcounter]
                gridcounter += 1

    def get_frontiers(self, map_data):
        # Converts map data to a NumPy array
        map_array = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))

        # Function to identify frontiers
        return self.find_frontiers(map_array)

    def find_frontiers(self, map_array):
        frontiers = []
        height, width = map_array.shape

        for y in range(height):
            for x in range(width):
                if map_array[y, x] == 0:  # Free space
                    neighbors = self.get_neighbors(x, y, width, height)
                    if any(map_array[ny, nx] == -1 for nx, ny in neighbors):  # Adjacent to unknown
                        frontier = (x, y)
                        if frontier not in frontiers:
                            frontiers.append(frontier)
        return self.group_frontiers(frontiers)

    def group_frontiers(self, frontiers):
        grouped_frontiers = []
        # Implement logic to group frontiers if necessary
        return grouped_frontiers

    def choose_frontier(self, frontiers):
        # Select the largest frontier based on length
        if frontiers:
            chosen_frontier = max(frontiers, key=lambda f: self.calculate_length(f))
            return chosen_frontier
        return None

    def calculate_length(self, frontier):
        # Calculate length of the frontier
        return np.linalg.norm(np.array(frontier[-1]) - np.array(frontier[0]))

    def decide_point(self, frontier):
        # Pick a point in the frontier (e.g., the midpoint or based on other criteria)
        return frontier[len(frontier) // 2]

    def get_neighbors(self, x, y, width, height):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if (dx != 0 or dy != 0) and 0 <= x + dx < width and 0 <= y + dy < height:
                    neighbors.append((x + dx, y + dy))
        return neighbors

    def navigate_to_point(self, point):
        self.get_logger().info(" cheack 1")
        # Convert grid coordinates to world coordinates using map resolution and origin
        goal_x = self.originX + point[0] * self.resolution
        goal_y = self.originY + point[1] * self.resolution

        # Create a PoseStamped message for the goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"  # Ensure the goal is in the "map" frame
        goal_msg.header.stamp = self.get_clock().now().to_msg()  # Get the current time
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation.w = 1.0  # Facing forward, assuming no rotation for simplicity

        # Publish the goal to Nav2
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal: x = {goal_x}, y = {goal_y}")

    def euler_from_quaternion(self, x, y, z, w):
        """Convert a quaternion into euler angles (roll, pitch, yaw)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians


def main(args=None):
    
    #print(" active 1")
    # Initialize ROS 2 communication
    rclpy.init(args=args)
    
    # Create an instance of your navigationControl node
    navigation_control = navigationControl()
    #get_logger().info(" active 2")
    # Keep the node alive and listen to incoming topics
    try:
        rclpy.spin(navigation_control)
        #get_logger().info(" active 3")
    except KeyboardInterrupt:
        pass

    # Shutdown the node gracefully when finished
    navigation_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
