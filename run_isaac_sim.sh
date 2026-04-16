#!/bin/bash
# Launch Isaac Sim with perf flags for AGV simulation.
# Disables main-loop rate limit so physics can hit 200 Hz,
# enables async rendering so camera renders don't block physics,
# keeps RTX Real-Time mode (not path tracing).
#
# Usage:  ./run_isaac_sim.sh
# Then press Play ▶ when the greenhouse loads.

OPEN_SCRIPT="/home/andres/agv-sim/scripts/open_greenhouse.py"

exec isaacsim \
  --exec "$OPEN_SCRIPT" \
  --/app/runLoops/main/rateLimitEnabled=false \
  --/app/player/useFixedTimeStepping=true \
  --/app/asyncRendering=true \
  --/app/asyncRenderingLowLatency=true \
  --/app/hydraEngine/waitIdle=false \
  --/physics/suppressReadback=true \
  --/physics/updateToUsd=false \
  --/physics/updateVelocitiesToUsd=false \
  --/rtx/ecoMode/enabled=false \
  --/rtx/rendermode="RaytracedLighting"
