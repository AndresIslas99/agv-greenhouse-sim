# agv_sim_nav

Nav2 parameter configuration for the greenhouse simulation.

## Key Files

- `config/nav2_params.yaml` — Full Nav2 stack params (SmacPlannerHybrid + RegulatedPurePursuit)
- `config/collision_monitor.yaml` — Collision monitor configuration

## Rules

- Parameters must work with the greenhouse aisle geometry (0.8m wide aisles)
- Planner and controller settings are tuned for the AGV's diff-drive kinematics
- Do not add Nav2 nodes here — they are launched from agv_sim_bringup
