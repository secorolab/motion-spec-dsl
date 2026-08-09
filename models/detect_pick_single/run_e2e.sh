#!/usr/bin/env zsh
# One-shot detect_pick_single e2e: clean slate, mock server, program, goal, events, result.
# Usage: ./run_e2e.sh [--no-mock]   (--no-mock = exercise the perception retry path yourself)
set -u
cd "$(dirname "$0")"
source /home/batsy/work/ms/setup-grc.zsh >/dev/null 2>&1

# Kill every leftover piece of a previous attempt, by exact process identity.
for pid in $(pgrep -x main); do
    grep -q detect_pick_single "/proc/$pid/cmdline" 2>/dev/null && kill -9 "$pid"
done
pkill -9 -f "[m]ock_locate_server" 2>/dev/null
pkill -9 -f "[b]dd_client.py" 2>/dev/null
sleep 1

if [ "${1:-}" != "--no-mock" ]; then
    ros2 run aruco_perception mock_locate_server --ros-args -p poses_file:="$(pwd)/mock_poses.yml" &
    MOCK=$!
fi

GEN=$(ls -d plan02_generation/detect_pick_single/*/ | tail -1)
motion-spec run "$GEN" --cwd "$GRC_WS" &
RUN=$!

# The client waits for the action server itself, streams /bdd/events, exits on the result.
python3 bdd_client.py
STATUS=$?

wait "$RUN" 2>/dev/null
[ -n "${MOCK:-}" ] && kill "$MOCK" 2>/dev/null
exit $STATUS
