# Robotics and Intelligent Systems

## Lab-1 Fall-2022
### Instructor: Prof. Dr. Francesco Maurelli f.maurelli@jacobs-university.de

- implementation of [uuv_simulator][1] and [husky][2]
- [phobos][3] # blender add on to generate URDF

## Quick Start

```$ cd ros_project/src```

```$ git clone https://github.com/uuvsimulator/uuv_simulator.git```

```$ git clone https://github.com/husky/husky.git```

```$ git clone https://github.com/ros-teleop/teleop_twist_keyboard.git #optional```

```$ catkin build # and source devel/setup.bash```

```$ roslaunch uuv_gazebo_worlds empty_underwater_world.launch```

```$ roslaunch uuv_descriptions upload_rexrov.launch mode:=default x:=0 y:=0 z:=-20 namespace:=rexrov```

```$ roslaunch uuv_control_cascaded_pid joy_velocity.launch uuv_name:=rexrov model_name:=rexrov joy_id:=0```

all stuff related to rostopic params etc is saved in /dump folder


## Creating new robot

```$ rosrun uuv_assistants create_new_robot_model --robot_name foronz```

## Config thruster manager

```$ rosrun uuv_assistants create_new_robot_model --robot_name foronz```

or

```$ rosrun uuv_assistants create_thruster_manager_configuration --robot_name foronz --output_dir <CATKIN_PKG>```

```$ cd ~/catkin_ws```

```$ catkin build```

```$ roslaunch uuv_gazebo_worlds empty_underwater_world.launch```

```$ roslaunch foronz_description upload.launch```

```$ roslaunch foronz_control start_thruster_manager.launch reset_tam:=true```

To command the vehicle, you have to start the thruster manager and publish a ROS Wrench message to the input topic

```/<ROBOT_NAME or ROBOT_NAMESPACE>/thruster_manager/input```

## To run the tasks as per lab questions
```$ roslaunch my_project <task>```

eg

```$ roslaunch my_project task2.launch # task3.launch or task4d.launch```

## Project Guide
### Task1
Default rexrov launches gazebo GUI and URDF and some other nodes used to control the AUV, like velocity_teleop wrench_controller or which subscribe to Twist/Wrench messages or services. 
Please find the output for the nodes, topics, services and messages including screenshots of rqt or the terminal output in ```ros_project/dump``` folder.
- ```$ rosnode list``` # debug information about ROS Nodes, including publications, subscriptions and connections.
- ```$ rostopic list``` # publishers, subscribers, publishing rate, and ROS Messages.
- ```$ rossrv list``` # prints out service descriptions, packages that contain .srv files
- ```$ rosservice list``` # listing and querying ROS Services.
- ```$ rosmsg list``` # displaying information about ROS Message types.
- ```$ rqt_graph``` # GUI plugin for visualizing the ROS computation graph.

## Task2
- ```$ roslaunch my_project task2.launch```
- ```$ cd bagfiles```
- ```$ rosbag record rexrov/cmd_vel```
- ```$ rqt_plot rexrov/cmd_vel/angular:linear```
- ```$ rosbag play <bagfile>```
- The screenshots are save in /dump folder

## Task3
- node can be found at ```ros_project/src/my_project/scripts/wrench_controller.py```
- ```$ roslaunch my_project task3.launch```

## Task4
- Blender files are saved in /foronz_xD
- robot descriptions are ```ros_project/src/foronz_simulator/foronz_description```
- Using default thruster placement cuz why not. 
- part c node ```src/my_project/scripts/thruster_allocator.py```
- finallyy ```$ roslaunch my_project task4d.launch```

# Project Contribution
## Group Members
- Faraz Ahmad (fa.ahmad@jacobs-university.de)
- Muaaz Rajput (murajput@jacobs-universty.de)
- Ainna Zafar (a.zafar@jacobs-university.de)

Each task of the project was done by the three group members together, so there is really no way to distribute the contributions. Consider it 33.333333333333% contributed by each member. Vielen Dank!

[1]: https://github.com/my-name-is-D/uuv_simulator.git "uuv_simulator"

[2]: https://github.com/husky/husky "husky"

[3]: https://github.com/dfki-ric/phobos "phobos"
