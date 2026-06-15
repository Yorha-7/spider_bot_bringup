# Contributing

## Quick start

```bash
# 1. Install dependencies (once)
bash scripts/install_jazzy.sh

# 2. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 3. Build
colcon build --symlink-install
source install/setup.bash

# 4. Run tests
colcon test
colcon test-result --verbose
```

## Code style

### C++

- **Standard:** C++17
- **Linter:** `ament_clang_format` (Google style)
- **Runtime nodes only** — launch files stay Python/XML
- No comments unless documenting a public API

### Python

- **Linter:** `ament_flake8` + `ament_pep257`
- Used for launch files, configs, and the hardware bridge node

### Firmware (STM32 Arduino sketches)

- `Arduino.h` + `Wire.h` only — no third-party sensor libraries
- Register-level I2C for MPU6050 and PCA9685

## Commit conventions

Refer to [`.commitlintrc.yml`](./.commitlintrc.yml):

| Type | Usage |
|---|---|
| `feat` | New feature |
| `fix` | Bug fix |
| `refactor` | Code change with no behaviour change |
| `chore` | Tooling, CI, dependencies |
| `docs` | Documentation only |
| `test` | Adding or fixing tests |
| `ci` | CI configuration |

Rules:
- Header max **72 characters**
- No trailing period
- **No `Co-Authored-By: <AI>`** or any AI attribution trailers — author is always the contributor

## Pull requests

- **One module per PR** — matches the decomposition in [`PLAN.md`](./PLAN.md)
- Branch from `main`, name as `feat/<module>` or `fix/<module>`
- PR must pass **all CI checks** (`ros-ci` + `compliance`)
- Self-verify against the acceptance gate in [`PLAN.md §7`](./PLAN.md#7-self-verification--acceptance-gates) before opening
- Do not merge your own PR — another contributor reviews first

## Adding a new package

1. Create the package:
   - `ament_cmake` for C++ nodes
   - `ament_python` for Python nodes
2. Add the package name to `.github/workflows/ros-ci.yml` package list
3. Add the package name to `.github/workflows/compliance.yml` file-checks

## Project structure

```
├── spider_msgs/                       # Custom msg/srv interfaces
├── big_bertha_description/            # URDF, meshes, ros2_control config
├── big_bertha_policy_controller/      # C++ ONNX gait controller
├── big_bertha_sim_bringup/            # Gazebo simulation stack
└── big_bertha_hardware_bringup/       # Physical hardware drivers
    └── firmware/                      # STM32 App Lab projects
```

See [`PLAN.md`](./PLAN.md) for the full architecture and module breakdown.
