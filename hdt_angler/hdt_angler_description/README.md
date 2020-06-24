# hdt_angler

The `hdt_angler` package provides a ROS interface to HDT's ANGLER arm. 

### Initial setup

0. Install [ROS Kinetic](http://wiki.ros.org/rosdep) on [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)

Follow directions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)

1. Create catkin workspace

```console
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/angler_ws/src
$ cd ~/angler_ws/src
$ catkin_init_workspace .
$ cd ..
$ catkin_make
```

2. Copy required repositories into `src` folder

 ~/angler_ws/src/hdt_angler/hdt_angler_description/


3. Install required packages and build

```console
$ cd ~/angler_ws
$ source devel/setup.bash
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make
```

### Running the system

Use the following command to start the system in simulation:

```console
$ cd ~/angler_ws
$ source devel/setup.bash
$ roslaunch hdt_angler_description hdt_angler_a_rviz.launch 
```

This will start the `hdt_angler_rviz` interface, bring up an `RViz` visualization of the system, and allow users to control the arm simulation in joint-by-joint control using sliders. To launch the other models simply change the 'a' in 'hdt_angler_a_rviz' to 'b','c', or 'd'.

# 6dof_demo
