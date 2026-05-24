# dogv2_mujoco

Minimal standalone MuJoCo simulation for DogV2 quadruped robot with RL policy inference.

Extracted from [rl_sar](https://github.com/TonyXYJ/rl_sar), stripped to essentials: one robot, MuJoCo only, no ROS.

## Quick Start

```bash
# 1. Download dependencies (MuJoCo 3.2.7 + LibTorch 2.3.0 CPU)
bash scripts/download_mujoco.sh
bash scripts/download_libtorch.sh

# 2. Build
bash build.sh

# 3. Run
bash run.sh
```

## Controls

| Key | Action |
|-----|--------|
| `0` | Stand up (GetUp) |
| `1` `2` `3`... | Switch policy config (auto-detected from `policy/` dirs) |
| `9` | Lie down (GetDown) |
| `P` | Passive mode |
| `W/S` | Forward / Backward |
| `A/D` | Left / Right |
| `Q/E` | Turn left / Turn right |
| `Space` | Stop all movement |
| `R` | Reset simulation |
| `L` | Toggle recording (starts/stops CSV log) |
| `Enter` | Pause / Resume |

> The robot auto-stands on startup. Press a policy number key (e.g. `1`) to enter RL locomotion.

## Switch Policies

Policy configs are auto-detected from `policy/*/config.yaml` at startup:

```
[1] cts_onnx
[2] dreamwaq_onnx
[3] parkour_hurdle_onnx
```

Press the number key to switch config and enter RL mode. The simulation resets and
the robot stands up using the new config automatically.

### Recording

Press `L` to start recording joint data (dof_pos + target_dof_pos, all 12 joints).
Press `L` again to stop. The data is written to `records/record_<timestamp>.csv`.

```csv
step,dof_pos_0,...,dof_pos_11,target_dof_pos_0,...,target_dof_pos_11
0,0.0451,...,-0.0387,-0.0717,...,0.1523
```

The real-time terminal display shows recording status and steps:

```
Status: RECORDING  |  Speed: 1.00  |  Steps: 542  |  Policy: [1] cts_onnx
```

### Plotting

Requires Python 3.12+ with [uv](https://docs.astral.sh/uv/). Run once:

```bash
uv add pandas matplotlib
```

Plot the latest recording:

```bash
uv run python plot_record.py
```

Or plot a specific file:

```bash
uv run python plot_record.py records/record_20250101_120000.csv
```

This opens a 4×3 subplot figure showing dof_pos (solid) vs target_dof_pos (dashed)
for each of the 12 joints.

### Config YAML

All parameters in a single `config.yaml` under the `params:` key:

```yaml
params:
  dt: 0.005
  decimation: 4
  num_of_dofs: 12

  model_name: "dog_v2_cts.pt"    # .pt = TorchScript, .onnx = ONNX (auto-detect)
  observations: ["commands", "gravity_vec", "ang_vel", "dof_pos", "dof_vel", "actions"]
  observations_history: [19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
  observations_history_priority: "time"

  action_scale: [0.25, ...]
  rl_kp: [20.0, ...]
  rl_kd: [0.5, ...]
  # ... see policy/cts/config.yaml for full example
```

### Joint Order

12 joints are ordered by leg pairs, each with hip/thigh/knee:

| Index | Leg   | Joint    |
|-------|-------|----------|
| 0-2   | FL (front-left)  | hip, thigh, knee |
| 3-5   | FR (front-right) | hip, thigh, knee |
| 6-8   | RL (rear-left)   | hip, thigh, knee |
| 9-11  | RR (rear-right)  | hip, thigh, knee |

To tune knee PD gains only, multiply indices 2, 5, 8, 11 by the desired factor.

## System Dependencies

- CMake >= 3.10, C++17 compiler
- glfw3, yaml-cpp, TBB, pthread

Ubuntu:

```bash
sudo apt install cmake libglfw3-dev libyaml-cpp-dev libtbb-dev
```

## Directory Structure

```
dogv2_mujoco/
├── CMakeLists.txt
├── build.sh / run.sh
├── scripts/              # download_mujoco.sh, download_libtorch.sh
├── src/
│   ├── main.cpp          # entry point + MuJoCo sim loop
│   ├── sim_mujoco.hpp    # RL_Sim class
│   ├── fsm_dog_v2.hpp    # FSM: Passive → GetUp → RL → GetDown
│   └── core/             # RL SDK, obs buffer, inference runtime, etc.
├── thirdparty/           # MuJoCo viewer + joystick
├── models/               # MuJoCo XML + STL meshes
├── policy/               # YAML config + model weights
├── records/              # recorded CSV data (gitignored)
├── plot_record.py        # plot recordings
├── pyproject.toml        # Python dependencies (uv)
├── uv.lock
└── lib/                  # prebuilt MuJoCo + LibTorch (downloaded)
```

## License

Apache-2.0 (MuJoCo simulate code from DeepMind, rl_sar by Ziqi Fan)
