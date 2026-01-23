# Autonomous Foraging Robot (Pololu 3Pi+)

> **Autonomous magnetic-object foraging robot with waypoint navigation, PID control, line-boundary detection, and timed mission execution**

## Author

**YuTong Li**  
MSc Robotics  
University of Bristol


## Project Overview

This project implements an **autonomous foraging robot** on the **Pololu 3Pi+ platform**, capable of:

- Systematically **searching predefined waypoints**
- **Detecting magnetic objects** using a magnetometer
- **Pushing objects across a boundary** using calibrated line sensors
- **Returning autonomously to the start position**

The system integrates **odometry-based navigation**, **PID motion control**, **sensor fusion**, and a **finite-state machine (FSM)** to achieve robust autonomous behaviour.



##  Hardware Platform

- **Robot:** Pololu 3Pi+  
- **Sensors:**
  - Wheel encoders (odometry)
  - Line sensors (black boundary detection)
  - Magnetometer (Z-axis magnetic field detection)
- **Actuators:** Differential drive DC motors
- **Display:** Onboard LCD (mission countdown timer)



## System Architecture

The control software is structured into **modular components**:

```
├── Encoders        → Wheel odometry
├── Kinematics      → Pose estimation (x, y, θ)
├── Motors          → Low-level motor control
├── PID             → Heading & distance controllers
├── LineSensors     → Boundary detection during pushing
├── Magnetometer    → Magnetic object detection
├── LCD             → 4-minute mission countdown
```

---

## Behaviour Logic (FSM)

The robot operates using a **finite-state machine**.

### Foraging States

| State  | Description |
|------|------------|
| SEARCH | Navigate through predefined waypoints |
| PUSH | Push detected magnetic object forward |
| RETURN | Navigate back to the origin `(0,0)` |
| WAIT | Idle (reserved state) |

### Navigation Sub-States

| State | Description |
|------|------------|
| STAGE_TURNING | Rotate to face target |
| STAGE_DRIVING | Drive towards target |
| STAGE_DONE | Target reached |



## Navigation & Control

### Odometry-Based Navigation
- Robot pose `(x, y, θ)` updated every **20 ms**
- Encoder-based dead reckoning

### PID Motion Control
- **Heading PID** → orientation alignment
- **Distance PID** → forward motion
- Angular correction gain scaled by remaining distance

### Waypoint Search Strategy

The robot follows a fixed waypoint list to ensure **arena coverage**:

```
(270, -50) → (230, -255) → (310, -420)
→ (167, -420) → (47, -400) → (80, -160)
```



## Magnetic Object Detection

- Magnetometer **Z-axis** sampled every **100 ms**
- Detection threshold:

```
|Z| > 10500
```

When a magnetic object is detected:
- Waypoint search is interrupted
- Robot switches to **PUSH** mode
- Line sensors are enabled



##  Line Boundary Detection (Push Phase)

- Line sensors are **only active during PUSH**
- Sensors are calibrated via **360° in-place rotation**
- Black line detected when:

```
Calibrated sensor value > 0.6
```

### Extra Push Strategy

To ensure successful boundary crossing:
- Robot continues pushing for **200 ms after first line detection**
- Prevents premature stopping at the boundary

---

## ⏱ Mission Timer (LCD)

- Total mission duration: **4 minutes**
- LCD displays **remaining time in real-time**
- Robot **automatically stops** when time expires



## Software Features Summary

- Modular and readable architecture
- FSM-based autonomous behaviour
- Robust PID-controlled navigation
- Selective sensor activation
- Safety stop on mission timeout



## Demonstration

The experimental behaviour of the robot is demonstrated in the following videos:

- **Bilibili:** https://www.bilibili.com/video/BV16ssYzrEZi/?vd_source=28781ed4cdf06a485cee040b20395637  
- **YouTube:** https://www.youtube.com/watch?v=GABk7jKQryw


## How to Run

1. Flash the program to the **Pololu 3Pi+**
2. Place the robot at the origin `(0,0)`
3. Power on and start the mission
4. Observe autonomous search, push, and return behaviour
5. **It is recommended to prepare a printed or hand-drawn arena map** to help verify whether the robot follows the expected waypoints and trajectories during testing.




## Key Learning Outcomes

- Practical PID tuning on real robots
- Odometry-based navigation limitations
- FSM design for autonomous systems
- Sensor calibration
- Time-constrained robotic missions




