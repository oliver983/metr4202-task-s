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

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/username/project-name.git

## Problems (revision4 & revision5):
1. Frontier detection only works for light gray and dark grey. This makes the robot unable to detect narrow paths. The fix needs to prioritise light grey frontier, but when it has run out of light grey within itself (certain radius) then it will try to visit the coloured frontiers.

2. Sometimes there are single grey frontier glitch, which causes the robot to get stuck.


ahahahahahah
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True slam:=True
