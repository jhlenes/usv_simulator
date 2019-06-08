# usv_simulator
A USV simulator for ROS Melodic and Gazebo 9.

Forked from https://bitbucket.org/osrf/vrx.

## Installation
Navigate to the ```src/``` folder in your catkin workspace, e.g. ```cd ~/catkin_ws/src```. Then run the following (the command ```sudo rosdep init``` will print an error if you have already executed it since installing ROS. This error can be ignored.)
```
git clone https://github.com/jhlenes/usv_simulator.git
cd ..
sudo apt update
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
catkin_make
source devel/setup.bash
```

## Simulate the Otter USV
```
roslaunch otter_gazebo otter.launch 
```
Faster than real-time simulation is also available, if your computer is fast enough. Without a dedicated graphics card this is only useful for simulation without gui, i.e. with argument gui:=false. 
```
roslaunch otter_gazebo otter_fast.launch 
```
Check out the launch files for available arguments.

### Control the Otter USV with the keyboard
```
roslaunch otter_gazebo keydrive.launch 
```
