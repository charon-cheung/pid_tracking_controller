#!/usr/bin/env bash
# Disposable latency investigation helper; never publishes motion commands.
set -eo pipefail
usage() {
  echo 'Usage: bash record.sh [--sim] OUTPUT_BAG [CMD_TOPIC [ODOM_TOPIC]]'
  echo 'Robot defaults: /cmd_vel_nav /dev/odometry/raw/dist'
  echo 'Webots --sim: /cmd_vel_smoothed /odom; bag timestamps use /clock.'
  echo 'Ctrl-C finishes recording. Existing bags are never overwritten.'
  echo 'Use the same ROS domain/DDS environment as the publishers.'
}
sim_args=()
cmd_default=/cmd_vel_nav
odom_default=/dev/odometry/raw/dist
if [[ ${1:-} == --sim ]]; then
  shift
  sim_args=(--use-sim-time)
  cmd_default=/cmd_vel_smoothed
  odom_default=/odom
fi
if [[ ${1:-} == --help ]]; then usage; exit 0; fi
if [[ $# -lt 1 || $# -gt 3 ]]; then usage >&2; exit 2; fi
printf '[latency] Loading ROS 2 Jazzy environment...\n'
if ! source /opt/ros/jazzy/setup.bash; then
  echo '[error] Failed to source /opt/ros/jazzy/setup.bash' >&2
  exit 1
fi
set -u
bag_path=$1
# Also accept a literal ~/ prefix supplied through a quoted argument.
if [[ "$bag_path" == '~/'* ]]; then bag_path="$HOME/${bag_path:2}"; fi
cmd_topic=${2:-$cmd_default}
odom_topic=${3:-$odom_default}
printf '[latency] Output: %s\n[latency] Topics: %s %s\n' "$bag_path" "$cmd_topic" "$odom_topic"
printf '[latency] ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s\n' "${ROS_DOMAIN_ID:-0}" "${RMW_IMPLEMENTATION:-default}"
if [[ -e "$bag_path" ]]; then
  echo "[error] Refusing to overwrite: $bag_path" >&2
  exit 1
fi
query_type() {
  local topic=$1 result code
  printf '[latency] Checking %s (up to 10 seconds)...\n' "$topic" >&2
  if result=$(timeout 10s ros2 topic type --no-daemon --spin-time 2 "$topic" 2>&1); then
    printf '%s\n' "$result"
  else
    code=$?
    printf '[error] Cannot query %s (exit %s).\n%s\n' "$topic" "$code" "$result" >&2
    if [[ $code == 124 ]]; then echo '[error] Topic discovery timed out.' >&2; fi
    echo '[hint] Check running publishers, topic names, ROS_DOMAIN_ID and DDS network configuration.' >&2
    echo '[hint] For Webots use: bash record.sh --sim OUTPUT_BAG' >&2
    return "$code"
  fi
}
cmd_type=$(query_type "$cmd_topic")
odom_type=$(query_type "$odom_topic")
if [[ "$cmd_type" != geometry_msgs/msg/TwistStamped && "$cmd_type" != geometry_msgs/msg/Twist ]]; then
  printf '[error] %s: expected TwistStamped or Twist; got: %s\n' "$cmd_topic" "$cmd_type" >&2
  exit 1
fi
if [[ "$odom_type" != nav_msgs/msg/Odometry ]]; then
  printf '[error] %s: expected Odometry; got: %s\n' "$odom_topic" "$odom_type" >&2
  exit 1
fi
if [[ ${#sim_args[@]} -gt 0 ]]; then
  clock_type=$(query_type /clock)
  if [[ "$clock_type" != rosgraph_msgs/msg/Clock ]]; then
    printf '[error] --sim requires /clock (Clock); got: %s\n' "$clock_type" >&2
    exit 1
  fi
  echo '[latency] Simulation clock selected; keep the simulation running (not paused).'
fi
printf '[latency] Types: %s / %s\n[latency] Starting recorder; Ctrl-C to finish.\n' "$cmd_type" "$odom_type"
# QoS follows actual publishers. No compression or per-message text formatting.
exec ros2 bag record --storage mcap --storage-preset-profile fastwrite \
  --max-cache-size 8388608 --output "$bag_path" "${sim_args[@]}" --topics "$cmd_topic" "$odom_topic"
