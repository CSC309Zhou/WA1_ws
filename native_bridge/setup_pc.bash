# Source in every PC ROS2 client shell after building the workspace.
_wa1_native_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
_wa1_setup="$_wa1_native_dir/../install/local_setup.bash"
if [[ ! -f "$_wa1_setup" ]]; then
  echo "WA1 ROS2 workspace is not built: $_wa1_setup" >&2
  unset _wa1_native_dir _wa1_setup
  return 1
fi
source "$_wa1_setup"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$_wa1_native_dir/config/fastdds_pc.xml"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$_wa1_native_dir/../log/native_bridge}"
mkdir -p "$ROS_LOG_DIR"
unset _wa1_native_dir _wa1_setup
