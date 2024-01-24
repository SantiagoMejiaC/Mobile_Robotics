# UV-LARM Jarvis

## Students

- Santiago MEJIA
- Yven Lucas DIENELT

## Overview
Follow the steps and code provided to make the Kobuki turtleBot3 robot move around on its own, steer clear of obstacles, and even create a map of its surroundings. It can also spot green NukaCola bottles along the way.

## Installation

The package is based on Ubuntu 22.04.

### Create Workspace and clone packages
```
cd ~/ws/
```
Clone IMT packages tbot and tsim
```
git clone https://bitbucket.org/imt-mobisyst/pkg-tbot.git
```
```
git clone https://bitbucket.org/imt-mobisyst/pkg-tsim.git
```
Clone the Jarvis package
```
git clone https://github.com/SantiagoMejiaC/uvlarm-Jarvis.git
```
### Build and source
```
cd ~/ros2_ws/
colcon build
source ./install/setup.bash
```

## Challenges
All functionalities of challenge 1 are required and used to be able to solve challenge 2

### Launch files
The launch files are used to start several nodes at once. As required by the challenge, the functions have been split in three launch files. All files can be launched with the ros2 launch command below.
```
ros2 launch grp_jarvis "filename".yaml 
```
*replace the filename with the launch file you want to execute
#### - sim_launch.yaml
This file launches all required files to show autonomous movement in the gazebo simulation and includes the challenge1.yaml to open all necessary nodes. It can be used to see results without a physical robot.
#### - tbot_launch.yaml
This file is meant to be used on the PC that is on the robot. It launches autonomous movement on the physical bot. It includes the minimal_launch.yaml in order to connect to the bot.
It also launches the bottle detection node which publishes a message as soon as a bottle is detected in the topic /NodeDetector.
#### - visualize.yaml 
This file is meant to be run on the operator PC. It launches rviz and the slam toolbox to map the area. It also launches a teleop node to control the robot manually and the Navigation which alows to manually set a target destination in the map.
## Remarks
We were not able to correctly publish the distance of the detected bottle. As a consequence we cannot map the bottles on the map. However we wrote a program which can publish fixed markers on the map Markers.py.
Further developement would be needed to fix the distance. It should be quick to implement the published distance in the Markers.py as we already wrote the part which would transform the coordinates from the local frame to the global frame. Some debugging would be necessary.








