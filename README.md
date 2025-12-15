# Integration of MyCobot Pro 600 and Digital Twin for Maze Navigation

This repository includes the documentation, code, and design resources for a project that integrates the MyCobot Pro 600 collaborative robotic arm with its digital twin to solve a 4x4 maze. The system leverages computer vision, robotic kinematics, and simulation technologies to autonomously plan and execute precise navigation paths.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Project Workflow](#project-workflow)
- [Technology Stack](#technology-stack)
- [Future Scope](#future-scope)
- [Demonstration](#demonstration)
- [References](#references)

## Overview
This project demonstrates the integration of a collaborative robotic arm with its digital twin for real-world automation tasks. The MyCobot Pro 600 robotic arm was used to solve a 4x4 maze, combining vision systems, kinematic modeling, and simulation for:
- Precise motion planning and execution.
- Path validation using a MATLAB-based digital twin.
- Real-world trajectory implementation using TCP communication.

## Features
- **Digital Twin Integration**: Simulate and verify robotic motion paths using MATLAB.
- **Maze Navigation**: Solve a 4x4 maze using OpenCV and convert solution paths to robot coordinates.
- **Inverse Kinematics**: MATLAB-based computation of joint angles for smooth path execution.
- **Real-Time Vision**: AI kit camera detects and define key waypoints in the maze.

## Project Workflow
1. **Maze Solution**:
   - Maze is captured using the AI kit camera.
   - Image processing detects boundaries and generates waypoints using OpenCV.
   - Waypoints calibrated to real-world coordinates.
2. **Digital Twin Simulation**:
   - Digital twin created in MATLAB using URDF files exported from Fusion 360.
   - Simulated robot motion validated for accuracy.
3. **Path Execution**:
   - Waypoints processed through an inverse kinematics model to compute joint angles.
   - Angles transmitted via TCP communication to the robotic arm for real-world execution.
4. **Validation**:
   - Compared digital twin results with physical robot motion to ensure precision and resolve discrepancies.

## Technology Stack
- **Hardware**: MyCobot Pro 600 robotic arm, AI kit camera.
- **Software**: Python (OpenCV, socket programming), MATLAB (inverse kinematics, digital twin), Fusion 360 (robot modeling).
- **Algorithms**: OpenCV for maze solving, linear interpolation for coordinate mapping.

## Future Scope
- **Scalability**: Extend the system to navigate larger and more complex mazes.
- **Automation**: Enhance autonomous navigation by integrating sensors for dynamic obstacle avoidance.
- **Real-Time Feedback**: Implement live adjustments to robot motion paths based on sensory input.

## Demonstration
A demonstration video showcasing the maze-solving process and robotic arm execution.<br>
[Demonstraion_Video](https://github.com/ChinmayAmrutkar/Integration-of-MyCobot-Pro-600-and-Digital-Twin-for-Maze-Navigation/blob/main/Digital_Twin_Video_Final_Project.mp4)<br>
[Real Time Video](https://github.com/ChinmayAmrutkar/Integration-of-MyCobot-Pro-600-and-Digital-Twin-for-Maze-Navigation/blob/main/Demonstration_Video.mp4) <br> 
<br>
![RAS_Final_Project](https://github.com/user-attachments/assets/4fbe5d59-d03d-4115-9aec-51ab28eb82fe)


## References
For detailed methodologies and data, refer to the full [documentation](https://github.com/ChinmayAmrutkar/Integration-of-MyCobot-Pro-600-and-Digital-Twin-for-Maze-Navigation/blob/main/Documentation.pdf).
