import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/oliver/turtlebot3_ws/src/waypoint_commander/install/waypoint_commander'
