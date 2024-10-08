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
   cd ~/<your workspace> (goto your workspace)
   colcon build
   ```
3. Source the workspace:
   ```bash
   source ~/<your workspace>/install/setup.bash
   ```
## Problems:
1. Frontier detection only works for light gray and dark grey. This makes the robot unable to detect narrow paths. The fix needs to prioritise light grey frontier, but when it has run out of light grey within itself (certain radius) then it will try to visit the coloured frontiers.

2. Sometimes there are single grey frontier glitch, which causes the robot to get stuck. (Revision 5)


ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True

## How to run:

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


## Running aruco marker detector (optional):
1. have gazebo open with a map loaded (similar to prac4)

2. colcon build

3. source the setup.bash

4. open a new terminal

5. run command:
ros2 run aruco_detector aruco_detector
