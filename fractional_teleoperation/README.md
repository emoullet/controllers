# Fractional Teleoperation

This package provides:
- a ROS 2 controller plugin: `fractional_teleoperation/FractionalTeleoperationController`
- a standalone ROS 2 node executable: `fractional_teleoperation_node`

Both implementations now share the same computation layer in:
- `include/fractional_teleoperation/fractional_teleoperation_core.hpp`
- `src/fractional_teleoperation_core.cpp`

## Overview

The teleoperation law is based on a fractional model:

```
D^alpha x_d(t) = K u(t)
```

with:
- `u(t)`: normalized joystick command (`extender_msgs/msg/TeleopCommand`)
- `alpha`: fractional order
- `K`: gain

The implementation uses Grünwald-Letnikov coefficients and a discrete recursive update.

## Shared Architecture

The shared core contains the common math and control post-processing used by both node and controller:

- Grünwald-Letnikov coefficients generation
- Dynamic `alpha` computation and coefficient refresh
- Fractional integration update
- Desired-position to velocity conversion
- Input-frame transform (`base` / `ee`), mode filtering, velocity scaling
- Adaptive gain computation (`dt` or `perceptual` mode)

This keeps behavior consistent between the node and controller paths.

## Fractional Update Used

With precomputed coefficients `c_k`, the implemented recursive update is:

```
x_k = dt^alpha * K * u_k - sum_{j=1..L} c_j * x_{k-j}
```

where:
- `L` is bounded by `memory_length`
- `c_0 = 1`
- `c_k = c_{k-1} * (k - 1 - alpha) / k`

## Dynamic Alpha

When `use_dynamic_alpha: true`:

```
lambda = ||u_linear||

if lambda < l_0:      alpha = 0
elif lambda > l_max:  alpha = alpha_max
else:                 alpha = alpha_max * (lambda - l_0) / (l_max - l_0)
```

When `alpha` changes beyond a small threshold, coefficients are recomputed.

## Adaptive Gain

For the node, `adapt_gain_to_alpha` enables gain adaptation:

- `adaptive_gain_mode: dt`
  - `K = v_max * dt^(1 - alpha)`
- `adaptive_gain_mode: perceptual`
  - `K = v_max * Gamma(alpha) / t_ref^(alpha - 1)`

(`Gamma(alpha)` uses a safe lower bound for very small `alpha`.)

## Main Parameters

Shared behavior parameters (node + controller):
- `alpha`
- `gain_K`
- `memory_length`
- `dt`
- `velocity_scale`
- `input_frame` (`base` or `ee`)
- `use_dynamic_alpha`, `alpha_max`, `l_0`, `l_max`

Node-specific parameters:
- `adapt_gain_to_alpha`, `adaptive_gain_mode`, `v_max`, `t_ref`
- `vel_cmd_topic`
- marker topics/scales including desired position marker

Controller-specific parameters:
- `robot_type`, `base_frame`, `tool_frame`, `command_names`

See configuration examples in:
- `config/franka_params.yaml`
- `config/explorer_params.yaml`
- `config/node_params.yaml`

## Usage

Controller mode:
1. Load controller config in `controller_manager`
2. Spawn `fractional_teleoperation_controller`
3. Publish teleop commands on `/teleop_cmd`

Node mode:
1. Launch `fractional_teleoperation_node` with `config/node_params.yaml`
2. Publish teleop commands on `/teleop_cmd`
3. Read velocity command on configured `vel_cmd_topic`

## Launch Commands

From the workspace root:

```bash
colcon build --packages-select fractional_teleoperation
source install/setup.bash
```

Core controller launch:

```bash
ros2 launch fractional_teleoperation fractional_teleoperation.launch.py
```

Explorer + qontrol integration test launch:

```bash
ros2 launch fractional_teleoperation test_explorer_fractional_qontrol.launch.py
```

Standalone node test launch:

```bash
ros2 launch fractional_teleoperation test_fractional_teleoperation_node.launch.py
```

2D video + hand joystick + fractional controller (test robot):

```bash
ros2 launch fractional_teleoperation test_video_2d_fractional_tele_with_controller_launch.py \
  folder_path:=/absolute/path/to/videos
```

2D video + hand joystick + fractional controller (explorer description):

```bash
ros2 launch fractional_teleoperation test_video_2d_hand_joystick_fractional_tele_launch.py \
  folder_path:=/absolute/path/to/videos
```

2D video + hand joystick only (no controller):

```bash
ros2 launch fractional_teleoperation test_video_2d_hand_joystick_simple_launch.py \
  folder_path:=/absolute/path/to/videos
```

3D/depth video + hand joystick + fractional controller:

```bash
ros2 launch fractional_teleoperation test_video_fractional_teleoperation_launch.py \
  folder_path:=/absolute/path/to/videos
```

Useful optional launch arguments (depending on file):

```bash
use_sim_time:=true
use_simulation:=true
gui:=true
fps:=30
controller_config:=/absolute/path/to/config.yaml
hand_joystick_config:=/absolute/path/to/joystick_config.yaml
```

## Dependencies

- ROS 2
- `controller_interface`
- `pluginlib`
- `robot_interfaces`
- `extender_msgs`
- `Eigen3`

## References

- Podlubny, I. (1999). *Fractional Differential Equations*. Academic Press.
- Oustaloup, A., et al. (2000). Frequency-band complex noninteger differentiator.
