# rescue-robot-simulation
# 🤖 Autonomous Rescue Robot – Virtual Simulation

A virtual intelligent rescue robot operating in a 2D grid environment using **A\*** path planning and **Finite State Machine (FSM)** behavior.

---

## 📌 Project Overview

This project simulates an autonomous rescue robot navigating a grid-based disaster area.  
The robot avoids obstacles, locates injured victims, and transports them safely to a designated safe zone using algorithmic decision-making and shortest-path planning.

The focus is on **programming and algorithmic thinking** in Python (no hardware required).

---

## 🎯 Objectives

The robot must:

- Navigate a **2D grid environment**
- Avoid static obstacles
- Identify and select a victim to rescue
- Compute a path using **A\*** algorithm
- Move step-by-step to the victim
- Carry the victim to the **Safe Zone**
- Repeat until all victims are rescued

---

## 🧠 Core Concepts

### 1) Grid-Based Environment
The world is represented as a 2D grid containing:

- `.` Empty cell  
- `#` Obstacle  
- `P` Victim  
- `S` Safe Zone  
- `R` Robot  

The robot moves in **4 directions**: up, down, left, right.

---

### 2) A* Path Planning (A-star)
The robot computes the shortest path to the target using the A* evaluation function:

\[
f(n) = g(n) + h(n)
\]

Where:

- **g(n)**: cost from start to the current node (number of steps)
- **h(n)**: heuristic estimate to the goal (Manhattan distance)

Manhattan Distance:

\[
|x_1 - x_2| + |y_1 - y_2|
\]

This heuristic fits the grid because movement is limited to four directions.

---

### 3) Finite State Machine (FSM)
Robot behavior is organized into clear states:

- `SEARCHING` → choose the next victim  
- `PLANNING_TO_VICTIM` → compute path to victim using A*  
- `MOVING_TO_VICTIM` → follow the planned path step-by-step  
- `PLANNING_TO_SAFE` → compute path to safe zone  
- `CARRYING_TO_SAFE` → carry victim to safe zone step-by-step  
- `FINISHED` → all victims rescued  

FSM makes the decision logic clean, structured, and easy to debug.

---

### 4) Simulation Loop
The simulation runs step-by-step:

1. Display the environment
2. Robot decides the next action based on its current state
3. Robot moves one step (if applicable)
4. Update state and repeat

---

## 🏗️ Project Structure

- `Environment`  
  - Holds the grid, obstacles, victims, and safe zone  
  - Provides valid moves via 4-neighbor expansion (`neighbors4`)  

- `AStarPlanner`  
  - Implements A* search with a priority queue (`heapq`)  
  - Reconstructs the final path once goal is reached  

- `Robot`  
  - Maintains position, target, state, path, carrying status  
  - FSM logic implemented in `decide()`  

- `Simulation`  
  - Runs the main loop (display → decide → step → repeat)

---

## ⚙️ Features

- ✅ Multiple victims rescue mission  
- ✅ Intelligent victim selection (nearest victim strategy)  
- ✅ A* shortest path planning with Manhattan heuristic  
- ✅ Obstacle avoidance (no collisions)  
- ✅ FSM-based robot behavior (clean transitions)  
- ✅ Step-by-step simulation output (state, rescued count, carrying)

---

## ▶️ How to Run

### ✅ Option 1: VS Code / Terminal
1. Make sure Python is installed.
2. Run:

```bash
python rescue_robot.py
