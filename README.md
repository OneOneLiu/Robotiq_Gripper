# Robotiq_Gripper

This ROS package facilitates the simulation and control of a Robotiq gripper, allowing for visualization in RViz, simulation in Gazebo, and operation of real hardware.

## 1. Features

- **RViz Visualization**: Launch with `vis_gripper.launch` to see the gripper in RViz.
- **Gazebo Simulation**: Use `gazebo_load_gripper.launch` to load the gripper into Gazebo.
- **Gazebo Interactive Control**: Use `gazebo_load_control_gripper.launch` to control the gripper in Gazebo with a GUI slider.
- **Real Gripper Control**: Use `real_gripper_control.launch` to operate a real Robotiq gripper, interfacing via commands and feedback with the hardware.

## 2. Dependencies

The control of real gripper is implemented from this [repository](https://github.com/TAMS-Group/robotiq.git). 

## 3. Quick Start

Connect the robotiq gripper to the host machine through USB. And then start docker container

```bash {.line-numbers}
# Visualize in RViz:
roslaunch robotiq_gripper vis_gripper.launch

# Simulate in Gazebo:
roslaunch robotiq_gripper gazebo_load_gripper.launch

# Control in Gazebo with GUI:
roslaunch robotiq_gripper gazebo_load_control_gripper.launch

# Control the real gripper (connect the gripper to the host before starting the container):
roslaunch robotiq_gripper real_gripper_control.launch
```

# 4. Note
- The `real_gripper_control.py` script in the `src` directory handles command transmission to the gripper and state feedback, integrating with `RViz` for visualization. This is managed by the `joint_state_publisher` node configured to listen to `/gripper/joint_states`.
- Connect the usb cable of the gripper to the host machine before starting the container.