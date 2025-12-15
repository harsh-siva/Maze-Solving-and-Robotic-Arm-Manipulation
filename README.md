# Maze Solving and Robotic Arm Manipulation for MyCobot Pro 600

This repository contains the documentation, source code, and design files for a project where we integrate the MyCobot Pro 600 collaborative robotic arm with its digital twin to solve a 4×4 maze. The system combines computer vision, robot kinematics, and simulation tools to autonomously plan and execute accurate navigation paths from image to real-world motion.

## Overview

This project showcases how a collaborative robotic arm can be paired with a digital twin to perform a real-world automation task. Using the MyCobot Pro 600, we solve a 4×4 maze by combining a vision pipeline, kinematic modeling, and simulation in order to:

- Plan and execute precise end-effector motion.
- Validate paths in a MATLAB-based digital twin before deployment.
- Reproduce validated trajectories on the physical robot via TCP communication.

## Features

- **Digital Twin Integration**: Use MATLAB to simulate, visualize, and verify robot trajectories before sending them to hardware.  
- **Maze Navigation**: Solve a 4×4 maze in Python with OpenCV and convert the resulting path into robot-frame coordinates.  
- **Inverse Kinematics**: Compute joint configurations in MATLAB for smooth and continuous movement along the maze path.  
- **Real-Time Vision**: Use the AI Kit camera to capture the maze and define key waypoints for navigation.

## Project Workflow

1. **Maze Solution**
   - The maze image is acquired using the AI Kit camera.  
   - OpenCV-based image processing extracts maze boundaries, grid structure, and path waypoints.  
   - These waypoints are then mapped and calibrated to real-world coordinates.

2. **Digital Twin Simulation**
   - A digital twin of the MyCobot Pro 600 is created in MATLAB using URDF files exported from Fusion 360.  
   - Simulated motion is used to verify reachability, continuity, and correctness of the path.

3. **Path Execution**
   - Calibrated waypoints are passed through an inverse kinematics model to obtain joint angles.  
   - The computed angles are transmitted over TCP to the MyCobot Pro 600, enabling real-world path execution.

4. **Validation**
   - Motion in the digital twin is compared against the behavior of the physical robot.  
   - Discrepancies are analyzed and reduced to improve accuracy and reliability.

## Technology Stack

- **Hardware**: MyCobot Pro 600 robotic arm, AI Kit camera  
- **Software**: Python (OpenCV, socket programming), MATLAB (inverse kinematics, digital twin), Fusion 360 (robot modeling and URDF export)  
- **Algorithms**: OpenCV-based maze solving and linear interpolation for coordinate mapping and trajectory refinement  

## Future Scope

- **Scalability**: Extend the framework to handle larger and more complex maze layouts.  
- **Automation**: Incorporate additional sensors to enable dynamic obstacle avoidance and more autonomous decision-making.  
- **Real-Time Feedback**: Close the loop using live sensor feedback to adjust robot motion on the fly and increase robustness.

## Demonstration

A demonstration video illustrates the complete pipeline from maze solving to robotic execution:<br>  
[Demonstraion_Video](https://github.com/harsh-siva/Maze-Solving-and-Robotic-Arm-Manipulation/blob/main/Digital_Twin_Video_Final_Project.mp4)<br>  
[Real Time Video](https://github.com/harsh-siva/Maze-Solving-and-Robotic-Arm-Manipulation/blob/main/Demonstration_Video.mp4)<br>  
<br>
![RAS_Final_Project](https://github.com/user-attachments/assets/4fbe5d59-d03d-4115-9aec-51ab28eb82fe)
