# Bridge V4 - Robot Deployment Framework

A modular C++17 deployment framework for RL-based robot control using ONNX Runtime for inference.

## Architecture

```
bridge_v4/
├── bridge_core/     # Core ament package (shared functionality)
├── mujoco_ros/      # MuJoCo simulator wrapper package
├── bridge_t1/       # T1 robot-specific deployment package
└── t1_description/  # T1 robot description (MJCF, meshes)
```

## Packages

### bridge_core
Core functionality including:
- **types.hpp**: Common data structures (RobotState, RobotCommand, Config)
- **transforms.hpp**: Quaternion and math utilities
- **config_manager**: YAML configuration loading
- **state_machine**: Robot control state management
- **rl_controller**: Main RL control loop
- **algorithm_base**: ONNX Runtime-based algorithm interface
- **Mod algorithm**: GRU-based policy implementation

### mujoco_ros
Standalone MuJoCo simulation with ROS2 interface:
- Publishes: `/mujoco/joint_states`, `/mujoco/imu`, `/mujoco/odom`
- Subscribes: `/mujoco/joint_pos_cmd`, `/mujoco/kp_cmd`, `/mujoco/kd_cmd`
- Services: `/mujoco/reset`, `/mujoco/pause`

### bridge_t1
T1 humanoid robot deployment:
- Connects bridge_core with mujoco_ros
- Includes T1-specific configuration
- Launch files for simulation

### t1_description
T1 robot description:
- MuJoCo MJCF model with torque actuators
- STL meshes

## Dependencies

- ROS2 Humble (or later)
- ONNX Runtime
- MuJoCo 3.x
- yaml-cpp

## Building

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Link or copy packages
ln -s /home/harry/Music/v4/v4/* .

# Copy T1 meshes (required before building)
./setup_meshes.sh

# Build
cd ~/ros2_ws
colcon build --symlink-install
```

## Running T1 in MuJoCo

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Launch T1 simulation
ros2 launch bridge_t1 t1_mujoco.launch.py
```

### Joystick Controls
- **L1 (button 4)**: Stand up / Stop RL
- **R1 + A (buttons 5 + 0)**: Start RL
- **B (button 1)**: Sit down
- **Left stick**: X/Y velocity commands
- **Right stick**: Yaw command
- **Select (button 6)**: Reset simulation

## Adding a New Robot

1. Create a new robot description package (e.g., `my_robot_description`)
2. Create a new robot deployment package (e.g., `bridge_my_robot`)
3. Implement `RobotInterface` if needed for your robot's SDK
4. Create configuration YAML with robot-specific parameters
5. Create launch file

## Configuration

Configuration files are YAML format with three sections:

```yaml
robot:
  type: "RobotName"
  num_dof: 23
  joint_names: [...]
  dof_activated: [...]  # Indices of RL-controlled joints

algorithm:
  name: "Mod"
  model_name: "model.onnx"
  dt: 0.002
  decimation: 10
  # ... scaling and limits

control:
  rl_kp: [...]
  rl_kd: [...]
  default_dof_pos: [...]
  # ... other control parameters
```

## License

MIT

