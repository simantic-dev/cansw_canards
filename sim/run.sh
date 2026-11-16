#!/bin/sh
# usage: sim/run.sh [VIRTUAL_SECONDS] [SCENARIO]   (default 5, sim/scenario.yaml). Needs `sim` (or SIM=...) and sim/build.sh done.
# Output in sim/out/: servo.csv, can3.csv (peer logs), sim.log.
set -e
cd "$(dirname "$0")/.."
SIM=${SIM:-sim}
[ -f sim/build/canard.elf ] || { echo "build first: sim/build.sh"; exit 1; }
mkdir -p sim/out
export SIM_OUT=sim/out
"$SIM" run --scenario "${2:-sim/scenario.yaml}" --timeout "${1:-5}" --on-fault continue > sim/out/stdout.txt 2> sim/out/sim.log
echo "peer logs: sim/out/servo.csv sim/out/can3.csv; sim.log has warnings and timing"
