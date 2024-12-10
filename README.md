# KUKA youBot Simulation



This repository contains the code and documentation for a mechatronics project focused on simulating and controlling the KUKA youBot platform. Developed during the Summer of 2020 at the University of Tehran, this project encompasses various aspects of robotics, including kinematics, control algorithms, and practical applications.

## Project Overview

The primary objective of this project was to simulate the KUKA youBot—a versatile mobile manipulator combining an omnidirectional mobile base with a 5-degree-of-freedom robotic arm. The simulation aimed to explore the robot's capabilities in tasks such as object manipulation, path planning, and executing complex movements.

## Repository Structure

- **ubot_world/**: Contains simulation environments and configurations related to the KUKA youBot platform.

- **webots_ros/**: Integrates the Webots simulator with the Robot Operating System (ROS) to facilitate robot simulation and control.

## Key Features

- **Kinematic Analysis**: Detailed study and implementation of both forward and inverse kinematics for the youBot's manipulator, utilizing the Denavit-Hartenberg (D-H) convention.

- **Control Algorithms**: Development of control strategies for precise movement and manipulation, including feedback loop control systems and trajectory planning.

- **Simulation Environment**: Creation of realistic simulation scenarios using Webots and ROS, enabling testing and validation of control algorithms and robot behaviors.

- **Application Scenarios**: Implementation of tasks such as the Tower of Hanoi and color-based object sorting to demonstrate the robot's capabilities in handling complex operations.

## Getting Started

To set up the simulation environment and run the provided code, follow these steps:

1. **Install ROS**: Ensure that ROS is installed on your system. For installation instructions, visit the [ROS Installation Guide](http://wiki.ros.org/ROS/Installation).

2. **Install Webots**: Download and install the Webots simulator from the [official website](https://cyberbotics.com/).

3. **Clone the Repository**:

   ```bash
   git clone https://github.com/shaaker-ma/Mechatronics.git

## Dependencies

- ROS: Robot Operating System (tested with ROS Melodic).

- Webots: Open-source robot simulator.

- Python: For scripting and control algorithms.

- C++: Core programming language for robot control.
