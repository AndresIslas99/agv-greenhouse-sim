# agv_sim_drive

C++ drive-shaping node that emulates ODrive-realistic wheel behavior.

## Key Files

- `src/sim_drive_shaping_node.cpp` — Main node (C++17)
- `config/drive_shaping_params.yaml` — Shaping parameters
- `launch/drive_shaping.launch.py` — Standalone launch

## Rules

- C++17 with `-Wall -Wextra -Wpedantic`
- Subscribes to `/agv/cmd_vel`, publishes `/agv/shaped_cmd_vel`
- Must preserve topic name parity with real drive pipeline
