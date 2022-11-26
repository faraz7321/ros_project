# Robotics and Intelligent Systems

## Lab-1 Fall-2022

- implementation of uuv_simulation and husky gazebo
- https://github.com/my-name-is-D/uuv_simulator.git
- https://github.com/husky/husky
- https://github.com/dfki-ric/phobos # blender add on to generate URDF

## Quick Start

```$ cd ros_project/src```

```$ git clone https://github.com/uuvsimulator/uuv_simulator.git```

```$ git clone https://github.com/husky/husky.git```

```$ git clone https://github.com/ros-teleop/teleop_twist_keyboard.git #optional```

```$ catkin build # and source devel/setup.bash```

```$ roslaunch uuv_gazebo_worlds empty_underwater_world.launch```

```$ roslaunch uuv_descriptions upload_rexrov.launch mode:=default x:=0 y:=0 z:=-20 namespace:=rexrov```

```$ roslaunch uuv_control_cascaded_pid joy_velocity.launch uuv_name:=rexrov model_name:=rexrov joy_id:=0```


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
