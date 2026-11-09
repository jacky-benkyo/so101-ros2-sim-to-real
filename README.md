# so101-ros2-sim-to-real
To implement real-time bidirectional synchronization, safety watchdog mechanisms, motion control between a MuJoCo physics simulation and a physical 6-axis SO-101 robotic arm.

# SO-101 Humanoid Robotic Arm: ROS 2 & MuJoCo Sim-to-Real Integration

A personal robotics project implementing real-time bidirectional synchronization, safety watchdog mechanisms, and software-level motion smoothing between a MuJoCo physics simulation and a physical 6-axis robotic arm.

## Key Features
- **ROS 2 Jazzy Integration:** Utilizes `sensor_msgs/msg/JointState` topics to stream joint states between the Python bridge and simulation nodes on Ubuntu 24.04.
- **Sim-to-Real Synchronization:** Maps MuJoCo internal radian outputs (`data.qpos`) to physical Feetech servo encoder counts (`0–4095`) with custom joint direction multipliers.
- **Software Slew-Rate Limiter:** Implements a velocity-limiting step filter (`MAX_STEP`) to ensure smooth, jerk-free trajectory tracking and protect physical hardware.
- **Safety Watchdog Mechanism:** Continuously monitors live position feedback from the hardware bus and triggers an immediate emergency torque disablement (`disable_torque()`) if tracking errors exceed safety thresholds.

## Tech Stack & Hardware
- **OS:** Ubuntu 24.04 LTS
- **Middleware:** ROS 2 Jazzy
- **Simulation:** MuJoCo Physics Engine (`mujoco.viewer.launch_passive`)
- **Hardware:** 6-Axis SO-101 Robotic Arm powered by Feetech STS3215 serial servos (`SerialMotorsBus`)

MuJoCo is chosen because of its high computational efficiency and fast simulation speed, enabling smooth execution even on systems without high-end GPUs.

# Project Structure
- `so101_bridge.py`: Hardware bridge node handling serial communication, safety checks, and motion smoothing.
- `mujoco_sim_node.py`: Simulation publisher node managing physics steps and ROS 2 joint state publishing.

## Demo
pending
