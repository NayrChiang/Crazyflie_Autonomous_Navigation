# 🚁 Crazyflie: Advanced Real-Time Autonomous Quadcopter Navigation

This project presents a complete autonomy pipeline for a quadrotor aerial robot, developed as part of the MEAM 6200 (Advanced Robotics) course at the University of Pennsylvania. The system integrates **3D environment mapping**, **graph search-based path planning**, **smooth trajectory generation**, and a **geometric nonlinear SE(3) controller** to achieve stable and collision-free quadrotor flight.

The pipeline was tested both in simulation and on a **Crazyflie 2.0** quadrotor in a motion capture environment. Throughout this project, we progressed from modeling the quadrotor's dynamics, to planning collision-free paths, generating feasible trajectories, and finally executing those plans in the real world.

---

## 🧠 System Overview

The quadrotor autonomy pipeline consists of the following key components:

1. **Environment Representation**  
   A 3D map of the world is parsed from a JSON file and converted into a voxel-based occupancy grid, which defines free and occupied space. Margins are added to account for the quadrotor’s physical size.

2. **Path Planning**  
   Using an A* graph search algorithm, a collision-free path is computed from the start to the goal position. The planner operates in discrete voxel space, guided by a Chebyshev heuristic.

3. **Trajectory Generation**  
   The planned path is sparsified using an adaptive waypoint selection method and then smoothed into a continuous, dynamically feasible trajectory using minimum snap optimization.

4. **Geometric Control**  
   A nonlinear SE(3) controller computes thrust and torques needed to track the desired trajectory. The controller converts position and orientation errors into control commands and outputs rotor speeds.

5. **Simulation and Real-World Testing**  
   The entire stack is validated in simulation using a Crazyflie dynamics model, and later deployed in a real lab environment using a Crazyflie 2.0 quadrotor with Vicon motion capture feedback.

---

## 📂 Project Pipeline

### 1. SE(3) Control
- `se3_control.py` implements a geometric PD controller in SE(3).
- Tracks position and orientation using tuned gain matrices:

```python
Kp = np.diag([6., 6., 40])
Kd = np.diag([5, 5, 10])
Kr = np.diag([200, 200, 40])
Kw = np.diag([22, 22, 10])
```

### 2. Graph Search Planning
- `graph_search.py` implements A* over a 26-connected grid.
- Uses Chebyshev distance as heuristic and returns a dense collision-free path.

### 3. Trajectory Generation
- `world_traj.py`:
  - Uses an adaptive waypoint selection strategy.
  - Generates minimum snap trajectories for smooth flight execution.

### 4. Occupancy Mapping
- `occupancy_map.py` builds a 3D voxel grid with inflated obstacle boundaries.
- Handles coordinate conversions between metric and voxel index space.

### 5. Simulation & Testing
- `sandbox.py` integrates the full stack:
  - Loads test map
  - Plans & generates trajectory
  - Simulates flight
  - Reports: goal reached, collisions, timing, distance

---

## 📊 Results

### A* Path, Waypoints, and Trajectory
![A* Path, Waypoints, and Trajectory](https://github.com/NayrChiang/Crazyflie_Autonomous_Navigation/blob/0837767ba6fd430e6a5c341038682e25f143774e/docs/Images/A_Path%2C_Waypoints%2C_and_Trajectory.png)

### 3D Path Visualization
![3D Path](https://github.com/NayrChiang/Crazyflie_Autonomous_Navigation/blob/0837767ba6fd430e6a5c341038682e25f143774e/docs/Images/3D_Path.png)

### Position vs. Time
![Position vs. Time](https://github.com/NayrChiang/Crazyflie_Autonomous_Navigation/blob/3fad9e506301fc10ba3470cceef8a08ae1e87d2d/docs/Images/Position_vs_Time.png)

### Orientation vs. Time
![Orientation vs. Time](https://github.com/NayrChiang/Crazyflie_Autonomous_Navigation/blob/3fad9e506301fc10ba3470cceef8a08ae1e87d2d/docs/Images/Orientation_vs_Time.png)

### Commands vs. Time
![Commands vs. Time](https://github.com/NayrChiang/Crazyflie_Autonomous_Navigation/blob/0837767ba6fd430e6a5c341038682e25f143774e/docs/Images/Commands_vs_Time.png)

### Flight Demonstration Video
[Flight Animation](https://github.com/NayrChiang/Crazyflie_Autonomous_Navigation/blob/3fad9e506301fc10ba3470cceef8a08ae1e87d2d/docs/Images/Flight_Animation.gif)

---

## 🔹 Repository Structure

```
📂 crazyflie-autonomous-navigation/
│── 📂 docs/               # Contains project report and results
│── 📂 code/               # All of the project source code
│── README.md              # Project overview, discussion, and results
│── LICENSE                # Open-source license (MIT recommended)
│── .gitignore             # Ignore unnecessary files (build files, logs)
```


## 📅 Final Report

See [`docs/report_project1.4_team6.pdf`](https://github.com/NayrChiang/Crazyflie_Autonomous_Navigation/blob/3fad9e506301fc10ba3470cceef8a08ae1e87d2d/docs/MEAM_6200_1_4_Team_6_Report.pdf) for details of the real-world testing results with Crazyflie.

---

## 📏 Acknowledgments

- University of Pennsylvania | MEAM 6200: Advanced Robotics  
- Crazyflie Labs & Vicon Motion Capture Lab
- `flightsim` Python library for simulation framework

---

## ❌ License

This repository and its contents are shared for educational showcase purposes only. **Reproduction, reuse, or redistribution of any part of this project is not permitted without explicit permission from the authors.**

All rights reserved.
