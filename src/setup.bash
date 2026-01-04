#!/usr/bin/env bash
# file: robot_runner.sh
# Usage:
#   ./robot_runner.sh 1 3
#   ./robot_runner.sh            # then type: 1 2 3
#   ./robot_runner.sh -l         # list only
#   ./robot_runner.sh -h         # help

set -u  # (no `-e` so one failure doesn't stop the rest)

# --- config you might tweak ---
PKG="robot_description"
LAUNCH_FILE="launch.py"

# --- menu: number -> [description, command] ---
declare -A DESC CMD

DESC[1]="Launch Gazebo + robot + Nav2"
CMD[1]="ros2 launch \"$PKG\" \"$LAUNCH_FILE\""

DESC[2]="Run RViz2"
CMD[2]="rviz2"

DESC[3]="Run rqt_graph"
CMD[3]="rqt_graph"

DESC[4]="Show TF tree (saves frames.pdf)"
CMD[4]="ros2 run tf2_tools view_frames"

DESC[5]="Watch odom TF (10s)"
CMD[5]="timeout 10s ros2 run tf2_ros tf2_echo odom base_link"

print_menu() {
  echo "Available commands:"
  for k in $(printf "%s\n" "${!DESC[@]}" | sort -n); do
    printf "  %d - %s\n" "$k" "${DESC[$k]}"
  done
}

usage() {
  cat <<EOF
robot_runner.sh - pick tasks by number (space-separated).
Examples:
  ./robot_runner.sh 1 2 3
  ./robot_runner.sh        # interactive prompt
Options:
  -l    list menu and exit
  -h    show this help
EOF
}

# --- flags ---
if [[ "${1-}" == "-h" ]]; then usage; exit 0; fi
if [[ "${1-}" == "-l" ]]; then print_menu; exit 0; fi

# --- gather choices ---
choices=()
if (( $# == 0 )); then
  print_menu
  read -r -p "Enter numbers (e.g., 1 2 3): " -a choices
else
  choices=("$@")
fi

# --- run selected commands ---
failures=0
pids=()

for ch in "${choices[@]}"; do
  if [[ -z "${DESC[$ch]-}" ]]; then
    echo "Skip: '$ch' is not a valid option." >&2
    continue
  fi

  echo ">>> [$ch] ${DESC[$ch]}"
  # Run GUI/long-lived apps in background so you can select multiple
  eval "${CMD[$ch]}" >/dev/null 2>&1 &
  pids+=($!)
done

# brief summary
if ((${#pids[@]})); then
  echo "Started ${#pids[@]} task(s) in background: ${pids[*]}"
  echo "Use 'ps -fp ${pids[*]}' to view, or 'kill ${pids[*]}' to stop."
else
  echo "No tasks started."
fi
