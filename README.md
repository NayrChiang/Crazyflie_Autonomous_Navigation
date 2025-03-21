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
   The planned path is sparsified using the Ramer–Douglas–Peucker (RDP) algorithm, then smoothed into a continuous, dynamically feasible trajectory using minimum jerk optimization. A trapezoidal velocity profile ensures smooth transitions between waypoints.

4. **Geometric Control**  
   A nonlinear SE(3) controller computes thrust and torques needed to track the desired trajectory. The controller converts position and orientation errors into control commands and outputs rotor speeds.

5. **Simulation and Real-World Testing**  
   The entire stack is validated in simulation using a Crazyflie dynamics model, and later deployed in a real lab environment using a Crazyflie 2.0 quadrotor with Vicon motion capture feedback.

---

## 📂 Project Pipeline

### 1. Modeling & Dynamics
- Simulated the quadrotor using the Newton-Euler formulation.
- Used a 6-DOF state representation with full rotational dynamics.

### 2. Occupancy Mapping
- `occupancy_map.py` builds a 3D voxel grid with inflated obstacle boundaries.
- Handles coordinate conversions between metric and voxel index space.

### 3. Graph Search Planning
- `graph_search.py` implements A* over a 26-connected grid.
- Uses Chebyshev distance as heuristic and returns a dense collision-free path.

### 4. Trajectory Generation
- `world_traj.py`:
  - Uses RDP to prune waypoints.
  - Allocates time using trapezoidal velocity profiles.
  - Generates minimum jerk trajectories via QP optimization.

### 5. SE(3) Control
- `se3_control.py` implements a geometric PD controller in SE(3).
- Tracks position and orientation using tuned gain matrices:

```python
Kp = diag([6., 6., 40])
Kd = diag([5, 5, 10])
Kr = diag([200, 200, 40])
Kw = diag([22, 22, 10])
```

### 6. Simulation & Testing
- `sandbox.py` integrates the full stack:
  - Loads test map
  - Plans & generates trajectory
  - Simulates flight
  - Reports: goal reached, collisions, timing, distance

---

## 📊 Results

| Metric            | Result        |
|------------------|---------------|
| Goal Reached     | ✅ Yes         |
| Collision-Free   | ✅ Yes         |
| Max Speed        | ~1.0 m/s      |
| Avg Error        | ~6 cm         |
| Planning Time    | ~1.2 sec      |

Demo plots, response graphs, and maze trajectories are included in the `docs/` folder.

---

## 🔹 Repository Structure

```
crazyflie-autonomous-navigation/
├── control/
│   └── se3_control.py
├── planning/
│   ├── occupancy_map.py
│   └── graph_search.py
├── trajectory/
│   └── world_traj.py
├── simulation/
│   └── sandbox.py
├── docs/
│   └── report_project1.4_team6.pdf
└── README.md
```

---

## 🚀 How to Run

1. Install dependencies:
```bash
pip install numpy scipy matplotlib cvxopt
```

2. Run simulation:
```bash
python simulation/sandbox.py
```

3. Optional: Visualize occupancy map or trajectory results in 3D plots.

---

## 📅 Final Report

See [`docs/report_project1.4_team6.pdf`](docs/report_project1.4_team6.pdf) for details of the real-world testing results with Crazyflie.

---

## 📧 Contact

For questions or collaboration:
- LinkedIn: [Your LinkedIn](https://linkedin.com/in/yourprofile)
- Email: [your.email@example.com](mailto:your.email@example.com)

---

## 📏 Acknowledgments

- University of Pennsylvania | MEAM 6200: Advanced Robotics  
- Crazyflie Labs & Vicon Motion Capture Lab
- `flightsim` Python library for simulation framework

---

## ❌ License

This repository and its contents are shared for educational showcase purposes only. **Reproduction, reuse, or redistribution of any part of this project is not permitted without explicit permission from the authors.**

All rights reserved.
