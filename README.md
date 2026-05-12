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
| `1` | RL locomotion |
| `9` | Lie down (GetDown) |
| `P` | Passive mode |
| `W/S` | Forward / Backward |
| `A/D` | Left / Right |
| `Q/E` | Turn left / Turn right |
| `Space` | Stop all movement |
| `R` | Reset simulation |
| `Enter` | Pause / Resume |

## Switch Policies

Place your policy files under `policy/<name>/`:

```
policy/
├── cts/
│   ├── config.yaml      # policy config
│   └── dog_v2_cts.pt    # TorchScript model
├── cts_onnx/
│   └── policy.onnx      # ONNX model (auto-detected by extension)
└── wtw/
    ├── config.yaml
    └── model.onnx        # ONNX model (auto-detected by extension)
```

### Sync Policy from Remote

```bash
rsync -avz wufy@100.108.115.42:~/projects/go2_rl_gym/logs/dog_v2_cts/exported/policies/policy.onnx policy/cts_onnx/policy.onnx
```

Then run with policy name:

```bash
./run.sh policy/wtw/config.yaml
```

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
└── lib/                  # prebuilt MuJoCo + LibTorch (downloaded)
```

## License

Apache-2.0 (MuJoCo simulate code from DeepMind, rl_sar by Ziqi Fan)
