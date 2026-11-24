#!/usr/bin/env bash
# Start Conveyor Dashboard V2
# Accessible at http://localhost:8000 (or http://<IP>:8000 from network)
# Manual testing console: http://localhost:8000/manual
cd "$(dirname "$0")"
export MPLCONFIGDIR=/tmp/matplotlib_conveyor

# Use virtual environment if present
if [ -d "venv" ]; then
    exec venv/bin/uvicorn server:app --host 0.0.0.0 --port 8000 "$@"
elif [ -d "../.venv" ]; then
    exec ../.venv/bin/uvicorn server:app --host 0.0.0.0 --port 8000 "$@"
else
    exec uvicorn server:app --host 0.0.0.0 --port 8000 "$@"
fi
