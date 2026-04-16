"""Open the greenhouse USD at Isaac Sim startup."""
import omni.usd

USD_PATH = "/home/andres/agv-sim/src/agv_isaac_sim/worlds/greenhouse_with_robot.usd"
omni.usd.get_context().open_stage(USD_PATH)
print(f"[AGV] Opened {USD_PATH}")
