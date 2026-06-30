# Fractional Teleoperation

This package provides:
- a ROS 2 controller plugin: `fractional_teleoperation/FractionalTeleoperationController`
- a standalone ROS 2 node executable: `fractional_teleoperation_node`

Both implementations now share the same computation layer in:
- `include/fractional_teleoperation/fractional_teleoperation_core.hpp`
- `src/fractional_teleoperation_core.cpp`

## Overview

The teleoperation law operates on a fractional **offset** Δx between the desired position and a slowly drifting reference position x_ref. The standalone node uses one fractional order, while the controller can tune linear and angular channels independently:

```
^C_0 D_t^alpha Δx(t) = K u(t)
^C_0 D_t^alpha_linear Δx_linear(t) = K_linear u_linear(t)
^C_0 D_t^alpha_angular Δx_angular(t) = K_angular u_angular(t)
```

with:
- `u(t)`: normalized joystick command (`extender_msgs/msg/TeleopCommand`)
- `Δx(t) = x_des(t) − x_ref(t)`: offset between desired and reference position
- `alpha` / `linear_alpha` / `angular_alpha`: fractional order, controlling memory depth and response smoothing
- `K`: gain

The desired position is reconstructed as `x_des = x_ref + Δx`, and the commanded velocity is the time-derivative of Δx.

The reference position `x_ref` slowly drifts toward `x_des`, so when the user holds the end-effector at a given position, the joystick can gradually be returned to zero without any further motion. The drift rate and update law are configurable (see [Reference Drift Parameters](#reference-drift-parameters)).

The implementation uses Grünwald-Letnikov coefficients and a discrete recursive update.

## Shared Architecture

The shared core contains the common math and control post-processing used by both node and controller:

- Grünwald-Letnikov coefficients generation
- Dynamic `alpha` computation and coefficient refresh
- Fractional integration update
- Desired-position to velocity conversion
- Input-frame transform (`base` / `ee`), mode filtering, velocity scaling
- Adaptive gain computation (`dt`, `perceptual`, or `geometric_transition` mode)

The controller and node share the math primitives, though their public parameters differ where noted below.

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

For the standalone node, when `use_dynamic_alpha: true`:

```
lambda = ||u_linear||

if lambda < l_0:      alpha = alpha_min
elif lambda > l_max:  alpha = alpha_max
else:                 alpha = alpha_min + (alpha_max - alpha_min) * (lambda - l_0) / (l_max - l_0)
```

When `alpha` changes beyond a small threshold, coefficients are recomputed.

For the controller, dynamic alpha is split by channel:

```
lambda_linear = ||u_linear||
lambda_angular = ||u_angular||
```

`use_dynamic_alpha_linear` drives `linear_alpha` from `linear_alpha_min` to `linear_alpha_max` using `linear_l_0` and `linear_l_max`. `use_dynamic_alpha_angular` does the same for `angular_alpha` using the angular range parameters.

## Adaptive Gain

For the standalone node, `normalize_gain_for_alpha` enables gain adaptation:

- `alpha_gain_normalization_mode: dt`
  - `K = v_max * dt^(1 - alpha)`
- `alpha_gain_normalization_mode: perceptual`
  - `K = v_max * Gamma(alpha) / t_ref^(alpha - 1)`
- `alpha_gain_normalization_mode: geometric_transition`
  - `K = k_0^(1 - alpha) * k_1^alpha`
  - `k_0` is the position-amplification endpoint and `k_1` is the velocity-gain endpoint

(`Gamma(alpha)` uses a safe lower bound for very small `alpha`.)
(`k_0` and `k_1` must stay strictly positive.)

In the controller, gain adaptation is configured independently with `linear_normalize_gain_for_alpha` and `angular_normalize_gain_for_alpha`. Each channel has its own mode and normalization constants, producing `K_linear` and `K_angular`.

## Node Parameter Reference

All parameters can be set in `config/node_params.yaml` or passed at launch with `--ros-args -p name:=value`.

---

### Core Fractional Offset Dynamics

These parameters define the main control law `^C_0 D_t^alpha Δx(t) = K u(t)`.

| Parameter | Type | Default | Range | Description |
|---|---|---|---|---|
| `alpha` | float | 0.8 | (0, 2) | Fractional order. Values near 0 produce a heavy integrator effect (very persistent motion). Values near 1 approximate a standard velocity integrator. Values above 1 introduce an inertial/predictive character. |
| `fractional_offset_gain` | float | 1.0 | > 0 | Gain on the fractional law. Directly scales the amplitude of Δx and the commanded velocity. Overridden at runtime when `normalize_gain_for_alpha: true`. |
| `memory_length` | int | 50 | ≥ 1 | Number of past Δx samples kept for the Grünwald-Letnikov sum. Longer memory improves fractional approximation accuracy at the cost of CPU. Values of 30–100 are typical. |
| `dt` | float | 0.01 | > 0 | Control loop time step (s). **Must match the actual timer period.** A mismatch distorts gain scaling and the physical meaning of `alpha`. 0.01 = 100 Hz. |
| `output_velocity_scale` | float | 1.0 | ≥ 0 | Global multiplier applied to all output velocities after the fractional computation. Use this for coarse workspace-to-robot scaling without re-tuning gain or alpha. |
| `global_linear_velocity_saturation` | float | 0.0 | ≥ 0 | Hard cap on the norm of the linear velocity vector after frame transform, mode filtering, drift compensation, and output scaling. `0` disables saturation. |
| `global_angular_velocity_saturation` | float | 0.0 | ≥ 0 | Hard cap on the norm of the angular velocity vector after frame transform, mode filtering, drift compensation, and output scaling. `0` disables saturation. |
| `input_frame` | string | `"base"` | `"base"` / `"ee"` | Frame in which joystick axes are interpreted. `"base"`: joystick x/y/z map directly to base-frame Cartesian axes. `"ee"`: joystick vector is rotated by the current end-effector orientation, so motion is always relative to the tool. |

**Tuning guidance:**
- Start with `alpha = 1.0` (standard integrator) to validate the setup, then lower toward 0.5–0.8 for smoother, more persistent motion.
- If motion feels unresponsive at first contact, increase `fractional_offset_gain` (or `v_max` when adaptive gain is enabled) before adjusting `alpha`.
- Keep `memory_length` ≥ 2× the expected settling time in samples. Short histories cause GL truncation artifacts, especially at low `alpha`.
- Use the saturation parameters as hard safety caps and `output_velocity_scale` as the coarse tuning knob.

---

### Adaptive Gain

When enabled, `fractional_offset_gain` is automatically recomputed from `v_max` so that the peak commanded velocity stays calibrated as `alpha` changes.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `normalize_gain_for_alpha` | bool | `false` | Enable automatic gain recomputation. When `true`, the YAML value of `fractional_offset_gain` is ignored at runtime. |
| `alpha_gain_normalization_mode` | string | `"perceptual"` | Formula used to compute K. `"dt"`: `K = v_max × dt^(1−alpha)`. `"perceptual"`: `K = v_max × Γ(alpha) / t_ref^(alpha−1)`. `"geometric_transition"`: `K = k_0^(1−alpha) × k_1^alpha`. The geometric mode transitions from a position-amplification endpoint at low alpha toward a velocity-gain endpoint at high alpha. |
| `v_max` | float | 1.0 | Maximum commanded velocity (m/s) when the joystick is fully saturated. The primary speed-range knob when adaptive gain is active. |
| `t_ref` | float | 0.2 | Reference time (s) for the perceptual gain formula. Represents the time horizon over which the velocity response is normalised. Only used when `alpha_gain_normalization_mode: "perceptual"`. |
| `k_0` | float | 1.0 | Positive position-amplification endpoint used only when `alpha_gain_normalization_mode: "geometric_transition"`. For `alpha = 0`, the adaptive gain tends to `k_0`. |
| `k_1` | float | 1.0 | Positive velocity-gain endpoint used only when `alpha_gain_normalization_mode: "geometric_transition"`. For `alpha = 1`, the adaptive gain tends to `k_1`. |

**Tuning guidance:**
- Enable adaptive gain when you vary `alpha` (statically or via dynamic alpha) and want velocity amplitude to stay consistent across configurations.
- Set `v_max` to a safe low value first and increase gradually. This is the most direct speed safety limit.
- `t_ref` rarely needs adjustment; values in 0.1–0.5 s cover most setups.
- Use `geometric_transition` when you want dynamic alpha to interpolate continuously between two hand-tuned gains; at `alpha = 0.5`, the gain becomes `sqrt(k_0 * k_1)`.

---

### Dynamic Alpha

Automatically scales `alpha` with the joystick input norm, providing finer control at low deflections and faster response at high deflections.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_dynamic_alpha` | bool | `false` | Enable joystick-norm-driven alpha scaling. |
| `alpha_min` | float | 0.0 | Alpha value used when `‖u‖ < l_0`. Set above zero to keep a minimum fractional memory effect even for small joystick inputs. |
| `alpha_max` | float | 1.0 | Alpha value reached when `‖u‖ ≥ l_max`. Best combined with `normalize_gain_for_alpha` to keep the velocity range consistent. |
| `l_0` | float | 0.1 | Joystick norm below which `alpha = alpha_min`. |
| `l_max` | float | 1.0 | Joystick norm above which `alpha = alpha_max`. Alpha scales linearly between `l_0` and `l_max`. |

**Tuning guidance:**
- Set `l_0` just above the joystick noise floor (typically 0.05–0.15).
- Set `l_max` to the joystick saturation norm (1.0 for a fully normalised interface).
- Start with `alpha_min = 0.0` if you want low-input dead-zone behavior, or raise it to `0.1`–`0.3` for smoother low-speed response.
- Always pair with `normalize_gain_for_alpha: true` so the velocity range stays consistent as alpha varies.

---

### Reference Drift Parameters

`x_ref` is a reference position that slowly drifts toward the current desired position `x_des = x_ref + Δx`. Because the fractional offset law acts on Δx, when the user holds the end-effector still, Δx → 0 and the joystick can be returned to center.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_reference_drift` | bool | `true` | Enable the drifting reference. When `false`, `x_ref` stays at zero and the law acts on absolute desired position (legacy behavior). |
| `reference_drift_joystick_threshold` | float | 0.0 | Drift is applied only when `‖u_linear‖` is above this threshold. Use this to prevent reference drift while the joystick is near neutral/noise. |
| `reference_update_mode` | string | `"first_order"` | Law used to update `x_ref`. `"first_order"`: exponential blend toward `x_des`. `"fractional"`: fractional-order integration of the reference error with its own alpha. |

**First-order mode** (`reference_update_mode: "first_order"`):

```
x_ref[k+1] = x_ref[k] + reference_first_order_rate × dt × (x_des[k] − x_ref[k])
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `reference_first_order_rate` | float | 0.15 | Drift speed (1/s) for first-order mode. Higher values make `x_ref` chase `x_des` faster, requiring the user to return the joystick to zero more quickly. Typical range: 0.05–0.5. |

**Fractional mode** (`reference_update_mode: "fractional"`):

Applies a fractional integral of the reference error, producing a smoother, memory-laden drift that inherits the same character as the main control law:

```
^C_0 D_t^reference_fractional_alpha (x_des − x_ref) → x_ref update step
```

| Parameter | Type | Default | Description |
|---|---|---|---|
| `reference_fractional_alpha` | float | 0.8 | Fractional order for the reference drift. Lower values produce slower, more persistent drift; values near 1 approximate first-order behaviour. Range (0, 2). |
| `reference_fractional_gain` | float | 0.15 | Gain of the fractional reference update law. It scales a memory-bearing fractional operator, so it is not the same kind of parameter as `reference_first_order_rate`. |

**Tuning guidance:**
- Start with `reference_update_mode: "first_order"` and `reference_first_order_rate: 0.1`–`0.3`.
- Too fast (`> 0.5`): the user must actively hold the joystick deflected to maintain offset — feels like the joystick springs back.
- Too slow (`< 0.05`): the end-effector keeps drifting long after the user releases the joystick.
- Use `"fractional"` mode if you want the drift to have a smooth, graduated memory character. A good starting point is `reference_fractional_alpha` equal to the main `alpha`, then tune `reference_fractional_gain` separately from the first-order rate because it acts through the fractional operator and its history.

---

### Fractional Offset Scaling

The fractional offset Δx can be scaled by time-dependent factors that ramp up smoothly when the joystick becomes active. This prevents sudden jumps in commanded position and velocity when the user first moves the joystick. The ramp profile is **fully configurable**.

The scale for each component follows a selected ramp profile from 0 to its maximum value over a configurable ramp time. The ramp time is measured from when the joystick norm first exceeds `reference_drift_joystick_threshold_`; when the joystick drops below the threshold, the ramp resets to zero on the next tick.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `linear_offset_scale_max` | float | 1.0 | Maximum scale factor applied to the linear fractional offset Δx_lin. Set to < 1 to dampen linear motion. |
| `angular_offset_scale_max` | float | 1.0 | Maximum scale factor applied to the angular fractional offset Δx_ang. |
| `offset_ramp_time` | float | 2.5 | Ramp duration in seconds. Time for scale to rise from ~0 to ~1. Set to 0 for instantaneous (no ramp). |
| `offset_ramp_profile` | string | `"sigmoid"` | Ramp profile shape. Options: `"linear"`, `"quadratic"`, `"smoothstep"`, `"sigmoid"`, `"exponential"`, `"inverse_exponential"`, `"tanh"`, `"cubic_hermite"`, `"step"`. |

**Available ramp profiles:**

| Profile | Formula | Character | Use case |
|---|---|---|---|
| `linear` | `scale(t) = t_ratio` | Constant velocity rise; corners at start/end | Simple, predictable; best for testing |
| `quadratic` | `scale(t) = t_ratio²` | Smooth start, constant-acceleration ramp | Good balance; slightly cheaper than sigmoid |
| `smoothstep` | `scale(t) = t_ratio²(3 - 2·t_ratio)` | Smooth position, velocity, and acceleration; cube Hermite | **Recommended for most robotics tasks** |
| `sigmoid` | `scale(t) = 1/(1 + e^(-10(t_ratio - 0.5)))` | Smooth symmetric S-curve; gentle at boundaries | Very smooth, symmetric feel; default |
| `exponential` | `scale(t) = 1 - e^(-5·t_ratio)` | Fast onset with asymptotic approach | Responsive; good for fast teleoperation |
| `inverse_exponential` | `scale(t) = 1 - (1 - t_ratio)³` | Gentle slow start, fast tail | Delicate, surgical feel; safer onset |
| `tanh` | `scale(t) = (tanh(6(t_ratio - 0.5)) + 1)/2` | Sigmoid alternative; slightly cheaper | Same feel as sigmoid with marginal CPU savings |
| `cubic_hermite` | `scale(t) = 3t_ratio² - 2t_ratio³` | Zero velocity at both start and end; smooth settle | Elegant; zero-velocity boundaries |
| `step` | `scale(t) = 1` if `t_ratio ≥ 1` else `0` | Instantaneous rise (ignored if `ramp_time > 0`) | For comparison; not recommended |

**Tuning guidance:**

- **For general use:** Use default `"sigmoid"` (smooth, symmetric, comfortable).
- **For fastest computation:** Use `"quadratic"` (very similar feel, ~10% cheaper than sigmoid).
- **For best scientific smoothness:** Use `"smoothstep"` or `"cubic_hermite"` (zero acceleration corners).
- **For responsive feel:** Use `"exponential"` (fast onset, good for drones/fast systems).
- **For delicate/surgical tasks:** Use `"inverse_exponential"` (very gentle initial rise, safe start).
- **Lower `ramp_time`** (e.g., 0.3–0.5 s) for quick ramp-up; **higher values** (≥ 2 s) for very gentle onset.
- **Reduce `scale_max` below 1.0** to limit maximum displacement: e.g., 0.5 caps offset at half the unscaled value.
- **Set `scale_max = 0`** to freeze position at joystick activation (offset always scales to 0).
- The scale resets to ~0 whenever the joystick drops below `reference_drift_joystick_threshold_`, allowing for soft "catch" behavior on re-engagement.

---

### Output and Marker Visualization

| Parameter | Type | Default | Description |
|---|---|---|---|
| `vel_cmd_topic` | string | `"/vel_cmd"` | Topic on which the node publishes the output `geometry_msgs/Twist` velocity command. |
| `enable_marker_visualization` | bool | `true` | Publish RViz markers for debugging. Set to `false` in production to reduce overhead. |
| `vel_cmd_marker_frame_id` | string | `"base_link"` | TF frame for all markers. Should match the robot base frame. |
| `vel_cmd_marker_topic` | string | `"/vel_cmd_marker"` | Arrow marker showing the current commanded velocity, rooted at the desired position (red arrow). |
| `vel_cmd_marker_scale_x/y/z` | float | 0.03/0.07/0.07 | Arrow shaft diameter, head diameter, and head length (m). |
| `desired_position_marker_topic` | string | `"/desired_position_marker"` | Sphere marker at `x_des = x_ref + Δx` (blue sphere). |
| `desired_position_marker_scale` | float | 0.07 | Sphere diameter (m). |
| `reference_position_marker_topic` | string | `"/reference_position_marker"` | Sphere marker at `x_ref` (orange sphere). |
| `reference_position_marker_scale` | float | 0.07 | Sphere diameter (m). |
| `joystick_linear_marker_topic` | string | `"/joystick_linear_marker"` | Arrow marker showing the current raw joystick linear input, rooted at `x_ref` (yellow arrow). |

**RViz tip:** Add all five marker topics with Fixed Frame set to `vel_cmd_marker_frame_id`. The three spheres/arrows form an intuitive picture: orange sphere = resting reference point, blue sphere = current fractional target, yellow arrow = live joystick input at the reference, red arrow = resulting commanded velocity.

---

## Tuning Quick Reference

### Recommended first-time tuning order

1. **Set `dt`** to match the actual timer frequency (e.g. `0.01` for 100 Hz).
2. **Disable adaptive gain** (`normalize_gain_for_alpha: false`) and set `fractional_offset_gain` manually to get rough motion at moderate joystick deflection.
3. **Tune `alpha`**: start at `1.0` (standard integrator), lower toward `0.5–0.8` for smoother, more persistent motion. In controller mode, tune `linear_alpha` and `angular_alpha` separately.
4. **Enable adaptive gain** (`normalize_gain_for_alpha: true`, mode `"dt"`, `"perceptual"`, or `"geometric_transition"`) once the alpha range is settled, replacing manual `fractional_offset_gain` with `v_max` set to the desired maximum speed or `k_0`/`k_1` set to the desired transition endpoints.
5. **Tune reference drift**: set `use_reference_drift: true`, mode `"first_order"`, and adjust `reference_first_order_rate` until the joystick return-to-centre timing feels natural.
6. *(Optional)* Enable dynamic alpha for variable-sensitivity control across the joystick range. In controller mode, use `use_dynamic_alpha_linear` and/or `use_dynamic_alpha_angular`.

### Symptom-to-parameter map

| Symptom | Likely cause | Adjustment |
|---|---|---|
| Motion too slow / unresponsive | `fractional_offset_gain` or `v_max` too low | Increase `fractional_offset_gain` (or `v_max` if adaptive gain is on) |
| Motion jerky / oscillatory | `fractional_offset_gain` too high, or `dt` mismatch | Lower `fractional_offset_gain`; verify `dt` matches the actual timer rate |
| Motion feels sticky, slow to start | `alpha` too low | Increase `alpha` toward 1.0; in controller mode, adjust `linear_alpha` or `angular_alpha` for the affected channel |
| End-effector keeps drifting after joystick release | `reference_first_order_rate` too low | Increase `reference_first_order_rate` |
| Joystick must be held to maintain position (springs back) | `reference_first_order_rate` too high | Decrease `reference_first_order_rate` |
| Response overshoots on direction reversal | `alpha` above 1 | Lower `alpha` below 1.0; in controller mode, adjust the affected channel |
| GL approximation artifacts at low `alpha` | `memory_length` too short | Increase `memory_length` (≥ 2× settling time in samples) |
| Velocity amplitude changes when `alpha` is modified | Adaptive gain disabled | Enable `normalize_gain_for_alpha: true` |

See configuration examples in:
- `config/franka_params.yaml`
- `config/explorer_params.yaml`
- `config/node_params.yaml`

## Usage

### Controller mode
1. Load the controller config in `controller_manager`
2. Spawn `fractional_teleoperation_controller`
3. Publish teleop commands on `/teleop_cmd`

### Node mode
1. Launch `fractional_teleoperation_node` with `config/node_params.yaml`
2. Publish teleop commands on `/teleop_cmd`
3. Read velocity commands on the configured `vel_cmd_topic` (default `/vel_cmd`)

### Mouse joystick + node mode
1. `ros2 launch fractional_teleoperation test_fractional_teleoperation_node_mouse_joystick.launch.py`
2. A browser joystick page opens automatically — drag to move the end-effector
3. Read velocity commands on `/vel_cmd`

See [Mouse Joystick Teleoperation](#mouse-joystick-teleoperation) for the full topic flow, all launch arguments, and verification commands.

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

Live hand pipeline test launch (rosbridge + compressed image republish + hand landmarks + hand joystick + fractional teleop):

Open and refresh atb each launch the file 
```
camera.html
```

```bash
ros2 launch fractional_teleoperation test_fractional_teleoperation_node_hand_live.launch.py
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

2D launch with optional RGB+depth fusion in `hand_landmarks_node`:

```bash
ros2 launch fractional_teleoperation test_video_2d_hand_joystick_simple_launch.py \
  folder_path:=/absolute/path/to/videos \
  use_depth:=true \
  depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info
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
use_depth:=true
depth_topic:=/camera/aligned_depth_to_color/image_raw
camera_info_topic:=/camera/color/camera_info
depth_time_tolerance_ms:=10.0
depth_min_m:=0.05
depth_max_m:=2.0
```

## Mouse Joystick Teleoperation

The `mouse_joystick_interface` package provides a browser-based 2D joystick that publishes `extender_msgs/TeleopCommand` on `/teleop_cmd`. Only the linear x/y axes are populated (mode = `TRANSLATION`); angular commands are always zero.

### Topic flow — node pipeline

```
Browser joystick UI
      │  HTTP POST /teleop  (x, y clamped to [-1, 1])
      ▼
mouse_joystick_interface node
      │  extender_msgs/TeleopCommand  →  /teleop_cmd
      ▼
fractional_teleoperation_node
      │  geometry_msgs/Twist  →  /vel_cmd  (configurable)
      ▼
Downstream consumer (simulator, controller, etc.)
```

### Combined launch — node + mouse joystick

```bash
ros2 launch fractional_teleoperation test_fractional_teleoperation_node_mouse_joystick.launch.py
```

The fractional node loads `config/node_params.yaml` automatically. Available arguments:

| Argument | Default | Description |
|---|---|---|
| `mouse_host` | `127.0.0.1` | HTTP bind address for the joystick web page |
| `mouse_port` | `8765` | HTTP bind port |
| `auto_open_browser` | `true` | Open the joystick page automatically in the default browser |

### Standalone mouse joystick only

```bash
ros2 launch mouse_joystick_interface mouse_joystick_launch.py
```

| Argument | Default | Description |
|---|---|---|
| `host` | `127.0.0.1` | HTTP bind address |
| `port` | `8765` | HTTP bind port |
| `auto_open_browser` | `true` | Open the joystick page automatically |
| `teleop_topic` | `teleop_cmd` | Output topic name (relative) |

Then run the fractional node separately:

```bash
ros2 run fractional_teleoperation fractional_teleoperation_node \
  --ros-args --params-file src/controllers/fractional_teleoperation/config/node_params.yaml
```

### Mouse + controller pipeline

The controller subscribes to `/teleop_cmd` directly — start the mouse joystick alongside the controller stack:

```bash
# Terminal 1 — controller stack
ros2 launch fractional_teleoperation fractional_teleoperation.launch.py

# Terminal 2 — mouse joystick
ros2 launch mouse_joystick_interface mouse_joystick_launch.py teleop_topic:=/teleop_cmd
```

> **Note:** The mouse joystick sends 2D input (x/y only). z-axis motion requires a different input device.

### Verifying the pipeline

```bash
# Confirm joystick commands are arriving
ros2 topic echo /teleop_cmd

# Confirm velocity commands are produced
ros2 topic echo /vel_cmd

# Visualise in RViz: add the marker topics listed in the Node Parameter Reference
# Fixed Frame should match vel_cmd_marker_frame_id (default: base_link)
```

## Offline Fractional Behavior Visualization

You can generate comparison plots for multiple `alpha` values without running ROS nodes.
The script simulates two joystick scenarios and saves figures in `fig/` by default:

- Circle input in the joystick plane with configurable radius (`0..1`) and linear velocity
- Step input along x from `0` to `step_size` (`0..1`)

Run from the workspace root:

```bash
python3 src/controllers/fractional_teleoperation/scripts/visualize_fractional_teleoperation.py \
  --alphas 0.2 0.5 0.8 1.0 \
  --circle-radius 0.6 \
  --circle-velocity 0.8 \
  --step-size 0.8
```

Optional output directory:

```bash
python3 src/controllers/fractional_teleoperation/scripts/visualize_fractional_teleoperation.py \
  --output-dir src/controllers/fractional_teleoperation/fig
```

## Dependencies

- ROS 2
- `controller_interface`
- `pluginlib`
- `robot_interfaces`
- `extender_msgs`
- `Eigen3`

## Controller Parameter Appendix

The `fractional_teleoperation_controller` is a `ros2_control` plugin sharing the same fractional core as the node. Parameters are declared in the controller manager YAML or the robot-specific config files (`config/franka_params.yaml`, `config/explorer_params.yaml`).

### Controller fractional parameters

The controller uses separate fractional order state for linear and angular command channels. Configure both `linear_alpha` and `angular_alpha` explicitly in controller YAML.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `linear_alpha` | float | `1.5` | Fractional order for the linear command path. |
| `angular_alpha` | float | `1.5` | Fractional order for the angular command path. |
| `linear_fractional_offset_gain` | float | `0.1` | Manual gain for the linear path when `linear_normalize_gain_for_alpha` is `false`. |
| `angular_fractional_offset_gain` | float | `0.1` | Manual gain for the angular path when `angular_normalize_gain_for_alpha` is `false`. |
| `linear_normalize_gain_for_alpha` | bool | `true` | Recompute linear gain from the current linear alpha. |
| `angular_normalize_gain_for_alpha` | bool | `true` | Recompute angular gain from the current angular alpha. |
| `linear_alpha_gain_normalization_mode` | string | `"dt"` | Formula for adaptive linear gain: `"dt"`, `"perceptual"`, or `"geometric_transition"`. |
| `angular_alpha_gain_normalization_mode` | string | `"dt"` | Formula for adaptive angular gain: `"dt"`, `"perceptual"`, or `"geometric_transition"`. |
| `linear_v_max` / `angular_v_max` | float | `1.0` / `1.0` | Channel-specific `v_max` values for `"dt"` and `"perceptual"` gain modes. |
| `linear_t_ref` / `angular_t_ref` | float | `1.0` / `1.0` | Channel-specific reference times for `"perceptual"` gain mode. |
| `linear_k_0` / `angular_k_0` | float | `1.0` / `1.0` | Channel-specific position-amplification endpoints for `"geometric_transition"`. |
| `linear_k_1` / `angular_k_1` | float | `1.0` / `1.0` | Channel-specific velocity-gain endpoints for `"geometric_transition"`. |
| `use_dynamic_alpha_linear` | bool | `false` | Enable dynamic alpha for the linear path using `||u_linear||`. |
| `use_dynamic_alpha_angular` | bool | `false` | Enable dynamic alpha for the angular path using `||u_angular||`. |
| `linear_alpha_min` / `linear_alpha_max` | float | `0.0` / `1.0` | Dynamic alpha range for the linear path. |
| `angular_alpha_min` / `angular_alpha_max` | float | `0.0` / `1.0` | Dynamic alpha range for the angular path. |
| `linear_l_0` / `linear_l_max` | float | `0.1` / `1.0` | Linear joystick norm range used to interpolate dynamic linear alpha. |
| `angular_l_0` / `angular_l_max` | float | `0.1` / `1.0` | Angular joystick norm range used to interpolate dynamic angular alpha. |
| `alpha_threshold` | float | `0.001` | Minimum alpha change before refreshing that channel's GL coefficients. |

All other tuning rules from the [Node Parameter Reference](#node-parameter-reference) apply with the channel-specific alpha substituted for `alpha`.

When dynamic alpha is enabled for one channel, only that channel's current alpha, GL coefficient cache, and adaptive gain are updated from that channel's joystick norm. The other channel can remain fixed, dynamic, or use a different dynamic range.

### Controller-specific parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_type` | string | `"explorer_velocity"` | Robot interface type. Determines how Cartesian velocity commands are mapped to joint commands. See the `robot_interfaces` package for available types. |
| `base_frame` | string | *(required)* | TF name of the robot base frame. Used by the robot interface for forward kinematics and Jacobian computation. |
| `tool_frame` | string | *(required)* | TF name of the tool/end-effector frame. |
| `command_names` | string[] | `[]` | List of `ros2_control` joint command interface names to drive. Must match the hardware interface configuration. |
| `robot_description` | string | *(from param server)* | URDF passed to the robot interface for kinematics initialisation. Typically provided by `robot_state_publisher`. |

### Quick tuning notes

- `global_linear_velocity_saturation` and `global_angular_velocity_saturation` are the hard safety caps; set them first.
- `output_velocity_scale` is the coarse speed-tuning knob underneath those caps.
- Tune `linear_alpha` and `angular_alpha` independently: start both at `1.0`, then lower the channel that should feel smoother or more persistent.
- Tune `linear_fractional_offset_gain` and `angular_fractional_offset_gain` independently when adaptive normalization is disabled.
- Use channel-specific gain-normalization parameters when translation and rotation need different speed ranges or geometric-transition endpoints.
- Use `use_dynamic_alpha_linear` and `use_dynamic_alpha_angular` independently when translation and rotation need different low/high-deflection behavior.
- `dt` must match the `ros2_control` update rate configured in the hardware interface or controller manager.
- There is no reference drift: to stop motion the user must return the joystick to zero and wait for the GL history to decay (time ≈ `memory_length × dt` seconds).
- For a full controller configuration example see `config/explorer_params.yaml` (Explorer + qontrol chain) or `config/franka_params.yaml`.

## References

- Podlubny, I. (1999). *Fractional Differential Equations*. Academic Press.
- Oustaloup, A., et al. (2000). Frequency-band complex noninteger differentiator.
