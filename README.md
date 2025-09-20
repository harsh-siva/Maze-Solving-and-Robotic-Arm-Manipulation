# Maze Solving and Robotic Arm Manipulation with MyCobot Pro 600

## Project Aim
Developed a complete pipeline for solving a **4×4 maze** and executing the solution path on a **MyCobot Pro 600 robotic arm**.  
Integrated a **digital twin, vision-based calibration, and socket programming** to plan, validate, and execute robotic movements.

---

## Motivation
This project demonstrates the use of **digital twin technology** and **algorithmic path planning** in robotics. It emphasizes:
- Robotic path planning and kinematic verification  
- Real-world execution through socket programming  
- Bridging simulation with physical robotic manipulation  

---

## Methodology

### Digital Twin Creation
- Modeled **MyCobot Pro 600** in **MATLAB Simulink & Fusion 360**  
- Verified kinematic equivalence using manufacturer parameters  

### Maze Solving
- Solved the **4×4 maze** using a **Breadth-First Search (BFS)** algorithm in Python  
- Extracted waypoints from solution paths  
- Converted pixel coordinates → real-world coordinates → joint angles via **inverse kinematics (IK)**  

### Socket Programming
- Established **TCP/IP communication** between PC and robot  
- Sent joint angle commands from Python to robot via TCP/IP  

### AI Kit Camera + ArUco Marker Calibration
- Used **AI Kit camera** for workspace calibration  
- Placed **ArUco markers** for position mapping  
- Transformed pixel coordinates into world coordinates for accurate control  

### Path Planning & Execution
- Verified planned paths in **digital twin**  
- Executed on **physical robot** using computed joint angles  
- Planned straight-line paths between **marker-defined points**  

---

## Highlights
- ✅ Maze successfully solved and executed on the **MyCobot Pro 600**  
- ✅ BFS algorithm provided efficient path generation  
- ✅ Accurate mapping between image coordinates and real-world workspace  
- ✅ Reliable communication via **socket programming** ensured smooth motion execution  
- ✅ Demonstrated effectiveness of **digital twins** for pre-verification  

---

## Limitations
- Dependent on accurate **camera calibration** for precision  
- BFS limited to **grid-based maze solving** (not generalized for continuous spaces)  
- Occasional lack of **IK solutions** for certain waypoints  

---

## Future Directions
- Integrate advanced motion planning algorithms (**A*** or **RRT***)  
- Improve **camera calibration automation**  
- Incorporate **SLAM or vision-based feedback** for dynamic corrections  
- Extend methodology to **more complex robotic arms and multi-robot systems**  

---

## Takeaway
This project showcases the integration of **AI-based path planning, digital twins, and real-world robotic execution**, providing a **scalable workflow for autonomous robotic manipulation tasks**.
