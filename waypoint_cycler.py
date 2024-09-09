import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading
import math

class navigationControl(Node):
    def __init__(self):
        super().__init__('Exploration')
        # Subscriptions for map, odometry, and laser scan topics
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        print("[INFO] Exploration mode active")

        # Store the map information
        self.map_data = None
        self.resolution = None
        self.originX = None
        self.originY = None
        self.width = None
        self.height = None
        self.data = None

    def target_callback(self):
        # Exploration logic here (replace with your actual exploration function)
        pass

    def scan_callback(self, msg):
        self.scan_data = msg
        self.scan = msg.ranges

    def map_callback(self, msg):
        # Store map data
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

        # Call the function to print the map info
        #self.print_map_info()
        self.map_formatting()
        #print(self.data[1])

    def odom_callback(self, msg):
        # Store odometry data
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

    def map_formatting(self):
        map_data = self.data
        gridcounter = 0
        
        xl = self.width
        yl = self.height
        
        #print("this is the lenght of the map ")
        #print(len(map_data))
        #print('this is the width and hight ')
        #print(f"Width: {self.width} cells")
        #print(f"Height: {self.height} cells")
        self.grid = [[0 for a in range(xl)] for b in range(yl)]
        for y in range(yl):
            for x in range(xl):
                self.grid[y][x]= map_data[gridcounter]
                gridcounter = gridcounter + 1

    def print_map_info(self):
        # Function to print the map information
        print("Map Information:")
        print(f"Resolution: {self.resolution} m/cell")
        print(f"Origin: ({self.originX}, {self.originY})")
        print(f"Width: {self.width} cells")
        print(f"Height: {self.height} cells")
        print(f"Map Data: {self.data}...")  # Print the first 10 map data values for brevity

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
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
    # Initialize ROS 2 communication
    rclpy.init(args=args)

    # Create an instance of your navigationControl node
    navigation_control = navigationControl()

    # Keep the node alive and listen to incoming topics
    try:
        rclpy.spin(navigation_control)
    except KeyboardInterrupt:
        pass

    # Shutdown the node gracefully when finished
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
