METR4202 Team Assignment
 
A brief description of what the project does: 
An exploration strategy for a TurtleBot3 robot to search an unknown environment and locate 
targets within a map using RViz, Gazebo and TurtleBot3.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Contributing](#contributing)
- [License](#license)

## Installation Process
1. Clone the repository:
   ```bash
   git clone https://github.com/username/project-name.git



## Build Process
1. Colcon build in workspace directory using:
   ```bash
   cd ~/<your workspace>
   colcon build
   ```
3. Source the workspace:
   ```bash
   source ~/<your workspace>/install/setup.bash
   ```

## How to run (in simulation):

1. Run installation types
2. In a terminal, launch the waffle_pi in the turtlebot3_world:
   ```bash
   export TURTLEBOT3_MODEL=waffle_pi
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```
3. Open a seperate window and run nav2 and slam
   ```bash
   export TURTLEBOT3_MODEL=waffle_pi
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
   ```
4. Open a seperate window and run waypoint commander code
   ```bash
   ros2 run waypoint_commander waypoint_cycler
   ```


## Running aruco marker detector:
1. Have gazebo, ros2 and slam running.

2. Build in the workspace
   ```bash
   cd ~/<your workspace>
   colcon build
   ```
   
3. Source the setup.bash
   ```bash
   source install/setup.bash
   ```
4. Open a new terminal and navigate to your workspace

5. Run command within the workspace:
   ```bash
   ros2 run aruco_detector aruco_detector
   ```
   A window of the camera view should appear, indicating that it is running. The detected marker id and location is printed in the terminal opened. 
