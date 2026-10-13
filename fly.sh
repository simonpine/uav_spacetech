#!/usr/bin/env bash
#
# One command to fly the tiltrotor.
#
#   ./fly.sh                 simulator + GUI, keyboard pilot in its own window
#   ./fly.sh --headless      no Gazebo GUI (still flyable, you get the HUD)
#   ./fly.sh --demo          fly the scripted mission instead of the keyboard
#   ./fly.sh --rviz          add RViz next to Gazebo
#
# The keyboard pilot needs a real terminal, so it cannot be a launch child - it
# gets its own window here. If no terminal emulator is available the script
# prints the one command to run by hand.
#
# No `set -u`: the ROS 2 setup scripts read unset variables by design and would
# abort the moment they are sourced.
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

ROS_SETUP=/opt/ros/humble/setup.bash
GUI=true
RVIZ=false
MODE=keyboard

for arg in "$@"; do
  case "$arg" in
    --headless) GUI=false ;;
    --demo)     MODE=demo ;;
    --rviz)     RVIZ=true ;;
    -h|--help)  sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

[ -f "$ROS_SETUP" ] || { echo "ROS 2 Humble not found at $ROS_SETUP" >&2; exit 1; }
# shellcheck source=/dev/null
source "$ROS_SETUP"

if [ ! -f install/setup.bash ]; then
  echo "== first run, building the workspace =="
  colcon build --symlink-install
fi
# shellcheck source=/dev/null
source install/setup.bash

# Two simulators on the same DDS domain both publish /vtol/odom and both listen
# to /vtol/command/motor_speed. The controller then sees a position that jumps
# between two aircraft, which looks exactly like the physics exploding - airspeed
# in the thousands, altitude flicking to zero. Refuse to be the second one.
if pgrep -f "ign gazebo|gz sim" >/dev/null 2>&1; then
  echo "A Gazebo instance is already running:" >&2
  pgrep -af "ign gazebo|gz sim" | sed 's/^/  /' >&2
  cat >&2 <<'MSG'

Two simulators on one ROS domain fight over the same topics. Stop the other one
first (SIGTERM, not -9, or it orphans its DDS shared memory):

    pkill -f "ign gazebo"; pkill -f "vtol_control/flight_controller"

Or give this one a domain of its own:

    ROS_DOMAIN_ID=42 ./fly.sh

MSG
  exit 1
fi

# FastDDS leaves its shared-memory segments behind when a node is SIGKILLed.
# Enough of them and discovery silently stops working: every topic goes quiet
# while the processes sit there at full CPU looking healthy, and the aircraft
# takes off at a tenth of the commanded climb rate because it is only receiving
# some of its IMU samples. Only safe to clear when nothing is using them.
if ! pgrep -f "ros2|rviz2|_ros|rclpy" >/dev/null 2>&1; then
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
fi

cleanup() {
  trap - EXIT INT TERM
  # SIGTERM, not SIGKILL: a killed node orphans its DDS shared memory.
  [ -n "${SIM_PID:-}" ] && kill -TERM "$SIM_PID" 2>/dev/null || true
  sleep 2
  # ros2 launch does not always reap the Gazebo server on the way out.
  pkill -TERM -f "ign gazebo" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== starting the simulator (gui=$GUI) =="
ros2 launch vtol_control vtol_sim.launch.py \
     gui:="$GUI" rviz:="$RVIZ" &
SIM_PID=$!

# Wait for the controller to announce itself rather than sleeping blindly.
echo -n "== waiting for the flight controller "
for _ in $(seq 1 60); do
  if timeout 2 ros2 topic echo /vtol/status --once >/dev/null 2>&1; then
    echo "- up =="
    break
  fi
  echo -n "."
  sleep 1
done

if [ "$MODE" = demo ]; then
  echo "== flying the scripted mission =="
  ros2 run vtol_control demo_flight
  exit $?
fi

TELEOP="cd '$HERE' && source '$ROS_SETUP' && source install/setup.bash && \
ros2 run vtol_control keyboard_teleop; echo; echo '[keyboard pilot exited]'; \
read -r -p 'press enter to close'"

if command -v gnome-terminal >/dev/null 2>&1; then
  gnome-terminal --title="VTOL keyboard pilot" -- bash -lc "$TELEOP" || true
elif command -v xterm >/dev/null 2>&1; then
  xterm -title "VTOL keyboard pilot" -geometry 130x28 -e bash -lc "$TELEOP" &
else
  cat <<EOF

  No terminal emulator found. Open a second terminal yourself and run:

      cd $HERE && source $ROS_SETUP && source install/setup.bash
      ros2 run vtol_control keyboard_teleop

EOF
fi

cat <<'EOF'

  ============================================================
   SPACE   toggle DRONE <-> AIRPLANE
   ENTER   arm / disarm        T  auto take-off    L  auto land
   W/S     pitch   A/D roll    Q/E yaw             R/F power
  ============================================================
  Ctrl-C here shuts the simulator down.

EOF

wait "$SIM_PID"
