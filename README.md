# 🤖 Robotic Warehouse - Technical Project

## :package: About

This repository contains the Technical Project code for the **Robotics Lab 2025** course.
This system simulates an autonomous multi-robot system performing an automated task in a warehouse environment.

The Robotic setup is : 

1. **Differential Drive Mobile Robot Fra2Mo**
   Mobile robot used for autonomous navigation, mapping and transportation.

2. **KUKA iiwa**
   Industrial manipulator used for perform the pick and place task.
   
## :hammer: Build

Clone this repository in the `src` folder of your ROS 2 Workspace, built it, and source the environment:

```shell
cd ~/ros2_ws/src
git clone https://github.com/Moreno-Zaccara/Technical-Project---Robotic-Warehouse.git
```
Build and source the setup file: 

```shell
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## :white_check_mark: Usage

### 🔍 Exploration & Mapping

To start autonomous exploration of the unknown environment, launch this command: 

```shell
ros2 launch ros2_fra2mo fra2mo_explore.launch.py
```

When the exploration is complete, we have to save the map in the correct path: 

```shell
cd ~/ros2_ws/src/ros2_fra2mo/maps/
ros2 run nav2_map_server map_saver_cli -f map
```
### 🎥 Video
📺 [Robotic Warehouse - Exploration](https://youtu.be/WmOycmKGOek)

### ▶️ Simulation

Launch the Robotic Warehouse custom world in gazebo, spawn the robots and the controllers :

```shell
ros2 launch final_project warehouse.launch.py
```

Launch the Navigation layer with navigation stack, perception and manupulation logic :

```shell
ros2 launch final_project pick_and_place.launch.py 
```

Run the mission execution node to start the complete simulation of the pick and place and transport of the package

```shell
cd ~/ros2_ws/src/final_project/src
python3 fra2mo_mission.py
```
### 🎥 Video
📺 [Robotic Warehouse - Simulation](https://youtu.be/wKI7aBO_ejM)

