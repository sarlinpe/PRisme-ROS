# PRisme Simulation
This projects aims at developing a comprehensive realistic simulation environment for Robopoly’s [PRisme robotics platform](http://robopoly.epfl.ch/prisme) and [printed base](https://github.com/Robopoly/Printed-base). It is based on the [Robot Operating System (ROS)](http://www.ros.org), a hassle-free robotics framework, and [Gazebo](http://gazebosim.org), a powerful simulation software.

One might use this as a base for the simulation of a more advanced robot by modifying its physical description or by adding new sensors. It can also be considered as an educational tool for the study of robotics software and algorithms in applications such as localisation or path planing without the need to access the physical robot.

## Scope
Software development is particularly difficult in robotics, as the execution of the code (the application) highly depends on the proper functioning of the hardware (mechanics and electronics).
Robotics simulations aim at overcoming this problem by making robot software development less dependent on the actual physical hardware. This allows to split tasks among the development team, increasing time-efficiency. It is really easy to move to the physical robot once it is ready, as the software does not need to be adapted.


The ROS framework brings this further, by providing a way to develop software portions as computational units (*nodes*) that communicate with standard packets (*messages*) and can be run and tested independently from each other (see [this](http://wiki.ros.org/ROS/Introduction) for a comprehensive introduction).

## Features
Five ROS packages are the core of this simulation environment:
* `prisme_description`: the physical description of the robot, including 3D models, instantiations of sensors and the Rviz launcher
* `prisme_gazebo`: maps and launchers for gazebo scenarios
* `prisme_control`: the controller for the differential drive
* `prisme_command`: high level application nodes

Currently implemented sensors are:
* IR range sensors (4 in the front)
* IR light sensors (2 under, `light_sensor_gazebo` package)
* Linear camera
* IMU (`imu_gazebo` package)
* Encoders

Simulation realism is improved by using real mass and inertia values for the 3D model and by adding a realistic noise to all sensors.

Simulation performance is improved by simultaneously using two different types of 3D models: a visual mesh corresponding to the physical robot's appearance, and a collision (simplified) model used by Gazebo physics engine.

## Installation
First of all, ensure that [ROS Jade](http://wiki.ros.org/jade/Installation) as well as the corresponding `gazebo-5` and `ros_control` packages are installed.

Then clone this repo in your catkin source directory and build it:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ git clone https://github.com/Skydes/PRisme-Sim.git
$ cd ~/catkin_ws
$ catkin make
```

It is recommended to create in your  workspace a bash script containing the following lines:
```
$ source ~/catkin_sw/devel/setup.bash
$ source /opt/ros/jade/setup.bash
```
This should be executed in every new terminal window as:
```
$ . <script_name>.sh
```

## Usage
The simulation components are run as separate ROS nodes that rely on each other. Each of them is started by running the `.launch` file of the corresponding package in a new terminal window:
```
$ roslaunch prisme_gazebo prisme_line.launch
$ roslaunch prisme_control prisme_control.launch
$ roslaunch prisme_description prisme_rviz.launch
$ rosrun prisme_command line_follower.py
```

One can alternatively create a [global launcher](http://wiki.ros.org/roslaunch/XML) that will take care of the individual nodes and that can be run this way:
```
$ roslaunch prisme_command line.launch
```

## Application
A new high-level application node can be written (in Python for instance) and added to the `prisme_command/src` directory. It can read sensor values by subscribing to topics that regularly publish messages and whose list can be obtained from `$ rostopic list`.

This project currently contains two application examples:
* Line following using IR (light) sensors and a P controller
* Obstacle avoidance using IR (range) sensors

## Customisation
It is easy to modify the 3D models or to add new sensors by [editing the `.urdf` and `.gazebo` files](http://gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros) in the `gazebo_description/urdf` directory.

New simulation cases can be created by adding `.world` files to the `prisme_gazebo/worlds` directory.

## What comes next
* Tests on the real sensors to determine the actual Gaussian noise parameters.
* A plugin publishing a discrete-value odometry topic for quadrature encoders.
* Application nodes for sensor fusion, localisation, SLAM (with Rviz interface).
* Microcontroller ROS firmware for the PRismino (with the Bluetooth module).
* Examples of multi-robot simulations.
* Examples of neural network training using multiple simulation instances.

## Credits
This project was entirely developed by Paul-Edouard Sarlin.
Special thanks are addressed to:
* Karl Kangur, who created the PRisme kit, had first the idea of such a project, and provided with a good [starting point](https://github.com/Nurgak/Virtual-Robot-Challenge).
* Alessandro Settimi, for its realistic [Gazebo IMU sensor plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/363).
* Ricardo Téllez, for its draft of the [Gazebo light sensor plugin](http://www.theconstructsim.com/create-a-ros-sensor-plugin-for-gazebo/).
