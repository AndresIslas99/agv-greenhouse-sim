#!/bin/bash
# Supervisor wrapper for Isaac Sim — keeps it alive and restartable.
#
# The FastAPI endpoint POST /sim/restart signals this script by:
#   1. Writing /tmp/agv_restart_isaac (flag file)
#   2. Sending SIGTERM to the isaacsim process
# This script detects the exit, waits briefly, and relaunches.
#
# Isaac Sim now auto-plays on load (open_greenhouse.py), so a restart
# means the simulation resumes without any manual intervention.
#
# Usage:
#   ./run_isaac_supervised.sh          # foreground
#   nohup ./run_isaac_supervised.sh &  # background (logs to nohup.out)
#
# Env vars:
#   AGV_AUTO_PLAY=0   disable auto-play (require manual Play button)
#   RESTART_DELAY=5   seconds to wait before relaunching (default 5)

set -u

RESTART_FLAG="/tmp/agv_restart_isaac"
RESTART_DELAY="${RESTART_DELAY:-5}"
CRASH_DELAY=10

echo "[supervisor] AGV Isaac Sim supervisor started (PID $$)"
echo "[supervisor] restart flag: $RESTART_FLAG"

while true; do
    rm -f "$RESTART_FLAG"

    echo ""
    echo "=========================================="
    echo "[supervisor] Launching Isaac Sim..."
    echo "=========================================="

    # run_isaac_sim.sh uses 'exec isaacsim ...' so the child process
    # IS the isaacsim binary — when it exits, we land here.
    ./run_isaac_sim.sh
    EXIT_CODE=$?

    echo "[supervisor] Isaac Sim exited (code $EXIT_CODE)"

    if [ -f "$RESTART_FLAG" ]; then
        echo "[supervisor] Restart requested via API — relaunching in ${RESTART_DELAY}s..."
        rm -f "$RESTART_FLAG"
        sleep "$RESTART_DELAY"
    else
        echo "[supervisor] Unexpected exit — relaunching in ${CRASH_DELAY}s..."
        sleep "$CRASH_DELAY"
    fi
done
