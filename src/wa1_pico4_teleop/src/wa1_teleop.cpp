#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <json/json.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pico4_ros2/msg/xr_frame.hpp>
#include <upperlimb/msg/dual_pose.hpp>
#include <upperlimb/msg/pose.hpp>
#include <upperlimb/msg/uplimb_state.hpp>
#include <upperlimb/srv/servo.hpp>
#include <upperlimb/srv/move_j.hpp>

#include "wa1_pico4_teleop/teleop_math.hpp"

using namespace std::chrono_literals;
using Steady = std::chrono::steady_clock;
namespace math = wa1_pico4_teleop;

namespace {
const std::vector<std::string> LEFT_JOINTS = {"Shoulder_Y_L", "Shoulder_X_L", "Shoulder_Z_L",
  "Elbow_L", "Wrist_Z_L", "Wrist_Y_L", "Wrist_X_L"};
const std::vector<std::string> RIGHT_JOINTS = {"Shoulder_Y_R", "Shoulder_X_R", "Shoulder_Z_R",
  "Elbow_R", "Wrist_Z_R", "Wrist_Y_R", "Wrist_X_R"};
// Identical to the installed PICO ee_delta_teleop R_XR_TO_ROOT matrix.
const math::Quaternion XR_TO_ROOT{0.5, -0.5, -0.5, 0.5};

math::Pose from_tcp(const upperlimb::msg::Pose & pose) {
  return {{pose.position.x, pose.position.y, pose.position.z},
    {pose.quaternion.x, pose.quaternion.y, pose.quaternion.z, pose.quaternion.w}};
}

void to_ros(const math::Pose & from, geometry_msgs::msg::Pose & to) {
  to.position.x = from.position.x;
  to.position.y = from.position.y;
  to.position.z = from.position.z;
  to.orientation.x = from.orientation.x;
  to.orientation.y = from.orientation.y;
  to.orientation.z = from.orientation.z;
  to.orientation.w = from.orientation.w;
}

std::string state_name(int state) {
  switch (state) {
    case 0: return "IDLE";
    case 1: return "RESETTING";
    case 2: return "CONFIGURING";
    case 3: return "STREAMING";
    case 4: return "STOPPING";
    case 5: return "CLEARING";
    case 6: return "MOVING_INITIAL";
    case 8: return "RECOVERING";
    default: return "FAULT";
  }
}
}  // namespace

class Wa1Teleop final : public rclcpp::Node {
 public:
  Wa1Teleop() : Node("wa1_pico4_teleop") {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    enable_control_ = declare_parameter<bool>("enable_control", true, fixed);
    pose_layout_ = declare_parameter<std::string>("pose_layout", "xyz_xyzw", fixed);
    translation_scale_ = declare_parameter<double>("translation_scale", 1.0, fixed);
    grip_on_ = declare_parameter<double>("grip_on", 0.8, fixed);
    grip_off_ = declare_parameter<double>("grip_off", 0.6, fixed);
    input_timeout_ms_ = declare_parameter<int>("input_timeout_ms", 100, fixed);
    feedback_timeout_ms_ = declare_parameter<int>("feedback_timeout_ms", 250, fixed);
    rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0, fixed);
    initial_left_ = declare_parameter<std::vector<double>>("initial_left", {}, fixed);
    initial_right_ = declare_parameter<std::vector<double>>("initial_right", {}, fixed);
    movej_v_ = declare_parameter<double>("movej_v", 0.2, fixed);
    movej_acc_ = declare_parameter<double>("movej_acc", 0.2, fixed);
    preset_tolerance_rad_ = declare_parameter<double>("preset_tolerance_rad", 0.03, fixed);
    max_xr_step_m_ = declare_parameter<double>("max_xr_step_m", 0.2, fixed);
    max_xr_step_rad_ = declare_parameter<double>("max_xr_step_rad", 1.0, fixed);
    if ((pose_layout_ != "xyz_xyzw" && pose_layout_ != "xyz_wxyz" &&
         pose_layout_ != "xyzw_xyz") ||
        !std::isfinite(translation_scale_) || translation_scale_ < 0.5 ||
        translation_scale_ > 1.5 ||
        !(grip_on_ > grip_off_ && grip_on_ <= 1.0 && grip_off_ >= 0.0) ||
        !(rate_hz_ >= 10.0 && rate_hz_ <= 200.0) ||
        !(input_timeout_ms_ > 0 && feedback_timeout_ms_ > 0) ||
        !(movej_v_ > 0.0 && movej_acc_ > 0.0 && preset_tolerance_rad_ > 0.0) ||
        !(max_xr_step_m_ > 0.0 && max_xr_step_rad_ > 0.0) ||
        !valid_joint_target(initial_left_) || !valid_joint_target(initial_right_)) {
      throw std::invalid_argument("Invalid teleoperation configuration");
    }
    const auto sensor_qos = rclcpp::QoS(1).best_effort();
    xr_sub_ = create_subscription<pico4_ros2::msg::XrFrame>(
      "/pico4/frame", sensor_qos,
      [this](pico4_ros2::msg::XrFrame::ConstSharedPtr msg) {on_xr(*msg);});
    left_tcp_sub_ = create_subscription<upperlimb::msg::Pose>(
      "/upperlimb/tcp_pose/left", sensor_qos,
      [this](upperlimb::msg::Pose::ConstSharedPtr msg) {update_tcp(left_, *msg);});
    right_tcp_sub_ = create_subscription<upperlimb::msg::Pose>(
      "/upperlimb/tcp_pose/right", sensor_qos,
      [this](upperlimb::msg::Pose::ConstSharedPtr msg) {update_tcp(right_, *msg);});
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/upperlimb/joint_states", sensor_qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joints(*msg);});
    occupancy_sub_ = create_subscription<std_msgs::msg::Int8>(
      "/upperlimb/occupancy", sensor_qos,
      [this](std_msgs::msg::Int8::ConstSharedPtr msg) {
        occupancy_ = msg->data; occupancy_at_ = Steady::now();
      });
    mode_sub_ = create_subscription<upperlimb::msg::UplimbState>(
      "/upperlimb/uplimb_state", sensor_qos,
      [this](upperlimb::msg::UplimbState::ConstSharedPtr msg) {
        robot_mode_ = msg->cmd_num; mode_at_ = Steady::now();
      });
    bridge_sub_ = create_subscription<std_msgs::msg::String>(
      "/upperlimb/bridge/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {on_bridge_status(msg->data);});
    left_preview_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/wa1/teleop/target/left", sensor_qos);
    right_preview_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/wa1/teleop/target/right", sensor_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/wa1/teleop/status", rclcpp::QoS(1).reliable().transient_local());
    left_trigger_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/wa1/teleop/trigger/left", sensor_qos);
    right_trigger_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/wa1/teleop/trigger/right", sensor_qos);

    if (enable_control_) {
      command_pub_ = create_publisher<upperlimb::msg::DualPose>(
        "/cmd/upperlimb/servol/dual_arm", rclcpp::QoS(1).reliable());
      set_client_ = create_client<upperlimb::srv::Servo>("/upperlimb/set_servo_params");
      clear_client_ = create_client<upperlimb::srv::Servo>("/upperlimb/clear_servo_params");
      stop_client_ = create_client<std_srvs::srv::Trigger>("/upperlimb/stop");
      reset_client_ = create_client<std_srvs::srv::Trigger>("/upperlimb/bridge/reset_fault");
      movej_client_ = create_client<upperlimb::srv::MoveJ>("/upperlimb/movej/dual_arm");
    }
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0/rate_hz_)),
      [this] {tick();});
    RCLCPP_INFO(get_logger(), "WA1 PICO4 teleop ready: %s; scope=dual_arm; "
      "preset=%s; translation_scale=%.2f; sensitivity=%d",
      enable_control_ ? "CONTROL ENABLED" : "PREVIEW ONLY",
      enable_control_ ? "startup initial" : "disabled", translation_scale_,
      math::kTeleopSensitivityLevel);
  }

 private:
  enum class ControlState {IDLE=0, RESETTING=1, CONFIGURING=2, STREAMING=3,
    STOPPING=4, CLEARING=5, MOVING_INITIAL=6, RECOVERING=8, FAULT=9};
  struct Side {
    math::Pose tcp, xr_ref, tcp_ref, target, hold, last_xr;
    math::PoseRateLimiter limiter;
    Steady::time_point tcp_at{};
    bool tcp_valid{false}, grip_active{false}, target_valid{false},
      hold_valid{false}, last_xr_valid{false};
  };

  static bool valid_joint_target(const std::vector<double> & values) {
    return values.size() == 7 &&
      std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
  }

  bool fresh(Steady::time_point stamp, int ttl_ms) const {
    return stamp != Steady::time_point{} &&
      Steady::now() - stamp < std::chrono::milliseconds(ttl_ms);
  }

  void enter(ControlState state) {
    control_state_ = state;
    state_at_ = Steady::now();
  }

  void update_tcp(Side & side, const upperlimb::msg::Pose & msg) {
    const auto pose = from_tcp(msg);
    if (!math::valid_pose(pose)) {side.tcp_valid = false; return;}
    side.tcp = pose;
    side.tcp.orientation = math::normalized(side.tcp.orientation);
    side.tcp_valid = true;
    side.tcp_at = Steady::now();
  }

  void on_joints(const sensor_msgs::msg::JointState & msg) {
    if (msg.name.size() != 19 || msg.position.size() != 19) {return;}
    std::map<std::string, double> positions;
    for (size_t i = 0; i < msg.name.size(); ++i) {
      if (!std::isfinite(msg.position[i])) {return;}
      positions.emplace(msg.name[i], msg.position[i]);
    }
    if (positions.size() != 19) {return;}
    for (const auto & name : LEFT_JOINTS) {if (!positions.count(name)) {return;}}
    for (const auto & name : RIGHT_JOINTS) {if (!positions.count(name)) {return;}}
    joints_ = std::move(positions);
    joint_at_ = Steady::now();
  }

  bool reached_preset(const std::vector<double> & left,
                      const std::vector<double> & right) const {
    if (!fresh(joint_at_, feedback_timeout_ms_)) {return false;}
    for (size_t i = 0; i < 7; ++i) {
      if (std::abs(joints_.at(LEFT_JOINTS[i]) - left[i]) > preset_tolerance_rad_ ||
          std::abs(joints_.at(RIGHT_JOINTS[i]) - right[i]) > preset_tolerance_rad_) {
        return false;
      }
    }
    return true;
  }

  void on_bridge_status(const std::string & text) {
    Json::CharReaderBuilder builder;
    Json::Value value;
    std::string error;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(text.data(), text.data()+text.size(), &value, &error) || !value.isObject()) {
      return;
    }
    bridge_read_only_ = value.get("read_only", true).asBool();
    bridge_ready_ = value.get("feedback_ready", false).asBool();
    bridge_fault_ = value.get("stream_fault", false).asBool();
    bridge_master_ = value.get("ros1_master_uri", "").asString();
    const Json::Value & connections = value["ros1_command_connections"];
    bridge_connection_ = connections.isObject() ? connections.get(
      "/zj_humanoid/upperlimb/servol/dual_arm", 0).asInt() : 0;
    bridge_at_ = Steady::now();
  }

  void update_arm(Side & side, const pico4_ros2::msg::XrController & controller,
                  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher) {
    math::Pose xr;
    if (!math::decode_raw_pose(controller.tracked_pose.pose, pose_layout_, xr)) {
      side.grip_active = false; side.target_valid = false; return;
    }
    if (side.last_xr_valid) {
      const auto & a = xr.position;
      const auto & b = side.last_xr.position;
      const double distance = std::hypot(std::hypot(a.x-b.x, a.y-b.y), a.z-b.z);
      const double angle = math::angular_distance(xr.orientation, side.last_xr.orientation);
      if (distance > max_xr_step_m_ || angle > max_xr_step_rad_) {
        side.last_xr = xr;
        const bool was_active = side.grip_active;
        side.grip_active = false;
        side.target_valid = false;
        release_required_ = true;
        if (was_active && control_state_ == ControlState::STREAMING) {
          halt("PICO4 tracking pose jumped; release both grips before rearming");
        }
        return;
      }
    }
    side.last_xr = xr;
    side.last_xr_valid = true;
    const double clutch = clutch_value(controller);
    const bool pressed = side.grip_active ? clutch > grip_off_ :
      clutch >= grip_on_;
    if (!pressed) {
      if (side.grip_active && side.tcp_valid && fresh(side.tcp_at, feedback_timeout_ms_)) {
        side.hold = side.tcp;
        side.hold_valid = true;
        side.limiter.reset(side.tcp);
      }
      side.grip_active = false; side.target_valid = false; return;
    }
    if (!side.grip_active) {
      if (!side.tcp_valid || !fresh(side.tcp_at, feedback_timeout_ms_)) {return;}
      side.xr_ref = xr;
      side.tcp_ref = side.tcp;
      side.hold = side.tcp;
      side.hold_valid = true;
      side.limiter.reset(side.tcp);
      side.grip_active = true;
    }
    try {
      side.target = math::map_controller_delta(
        side.xr_ref, xr, side.tcp_ref, XR_TO_ROOT, translation_scale_);
    } catch (const std::exception &) {
      side.target_valid = false; return;
    }
    side.target_valid = true;
    geometry_msgs::msg::PoseStamped preview;
    preview.header.stamp = now();
    preview.header.frame_id = "BASE";
    to_ros(side.target, preview.pose);
    publisher->publish(preview);
  }

  void on_xr(const pico4_ros2::msg::XrFrame & frame) {
    const int64_t receive_age_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      Steady::now().time_since_epoch()).count() - frame.pc_receive_monotonic_ns;
    if (frame.input != 1 || frame.pc_receive_monotonic_ns <= 0 ||
        receive_age_ns < 0 || receive_age_ns > int64_t(input_timeout_ms_)*1000000 ||
        (enable_control_ && frame.device_id == "replay") ||
        (enable_control_ && frame.pc_receive_monotonic_ns <= last_pc_receive_ns_)) {
      return;
    }
    // The headset timestamp may repeat across fresh SDK callbacks. PC receive time
    // still orders those frames and preserves button release/press transitions.
    if (enable_control_ && last_device_stamp_ns_ != 0 &&
        frame.device_timestamp_ns <= last_device_stamp_ns_) {
      ++device_timestamp_nonmonotonic_count_;
    }
    last_device_stamp_ns_ = frame.device_timestamp_ns;
    last_pc_receive_ns_ = frame.pc_receive_monotonic_ns;
    last_xr_at_ = Steady::now();
    xr_timed_out_ = false;
    left_grip_value_ = clutch_value(frame.left_controller);
    right_grip_value_ = clutch_value(frame.right_controller);
    std_msgs::msg::Float32 left_trigger, right_trigger;
    left_trigger.data = frame.left_controller.trigger;
    right_trigger.data = frame.right_controller.trigger;
    left_trigger_pub_->publish(left_trigger);
    right_trigger_pub_->publish(right_trigger);
    const bool a_button = frame.right_controller.primary_button;
    const bool a_pressed = a_button_seen_ && a_button && !a_button_raw_;
    const bool first_a_frame = !a_button_seen_;
    a_button_seen_ = true;
    a_button_raw_ = a_button;
    if (first_a_frame) {
      a_button_last_action_ = a_button ? "initial_button_held" : "initial_button_released";
    }
    if (!initial_done_ && first_a_frame && a_button) {
      RCLCPP_WARN(get_logger(), "PICO4 A is held; release it before startup MoveJ");
    }
    if (a_pressed) {
      ++a_button_press_count_;
      release_required_ = true;
      if (control_state_ == ControlState::FAULT) {
        begin_fault_recovery();
        a_button_last_action_ = control_state_ == ControlState::RECOVERING ?
          "recovery_started" : "recovery_refused";
      } else if (control_state_ == ControlState::RECOVERING) {
        a_button_last_action_ = "ignored_recovering";
      } else {
        unlocked_ = !unlocked_;
        a_button_last_action_ = unlocked_ ? "unlocked" : "locked";
        if (!unlocked_ && control_state_ == ControlState::STREAMING) {
          halt("A button locked teleoperation");
        } else if (control_state_ == ControlState::MOVING_INITIAL &&
                   unlocked_ != initial_move_from_b_) {
          fault("A lock changed during initial-position MoveJ");
        }
        if (!unlocked_) {pending_return_ = false;}
      }
      RCLCPP_INFO(get_logger(), "A press #%llu: action=%s unlocked=%d state=%s "
        "release_required=%d bridge_fault=%d",
        static_cast<unsigned long long>(a_button_press_count_), a_button_last_action_.c_str(),
        unlocked_, state_name(static_cast<int>(control_state_)).c_str(),
        release_required_, bridge_fault_);
    }
    const bool b_button = frame.right_controller.secondary_button;
    if (!b_button) {
      b_hold_since_ = Steady::time_point{};
      b_consumed_ = false;
    } else if (b_hold_since_ == Steady::time_point{}) {
      b_hold_since_ = Steady::now();
    } else if (!b_consumed_ && Steady::now() - b_hold_since_ >= 1s) {
      b_consumed_ = true;
      if (enable_control_ && unlocked_ && initial_done_) {
        pending_return_ = true;
        release_required_ = true;
        if (control_state_ == ControlState::STREAMING) {
          halt("B held one second; stopping ServoL before return to initial position");
        }
      }
    }
    update_arm(left_, frame.left_controller, left_preview_pub_);
    update_arm(right_, frame.right_controller, right_preview_pub_);
    // A timeout or failed service must require a physical release before rearming.
    if (!pending_return_ && std::isfinite(left_grip_value_) &&
        std::isfinite(right_grip_value_) && left_grip_value_ < grip_off_ &&
        right_grip_value_ < grip_off_) {release_required_ = false;}
  }

  double clutch_value(const pico4_ros2::msg::XrController & controller) const {
    return controller.gripper;
  }

  bool ready_for_control(bool allow_fault) const {
    return fresh(last_xr_at_, input_timeout_ms_) &&
      fresh(left_.tcp_at, feedback_timeout_ms_) && left_.tcp_valid &&
      fresh(right_.tcp_at, feedback_timeout_ms_) && right_.tcp_valid &&
      fresh(joint_at_, feedback_timeout_ms_) &&
      fresh(occupancy_at_, feedback_timeout_ms_) && occupancy_ == 0 &&
      fresh(mode_at_, feedback_timeout_ms_) && robot_mode_ != 15 &&
      fresh(bridge_at_, 2000) && !bridge_read_only_ && bridge_ready_ &&
      bridge_master_ == "http://192.168.217.1:11311" &&
      (allow_fault || !bridge_fault_) && bridge_connection_ == 1;
  }

  std::string control_block_reason() const {
    if (!enable_control_) {return "Control disabled";}
    if (control_state_ == ControlState::STREAMING) {return "";}
    if (control_state_ != ControlState::IDLE) {
      return "Waiting in " + state_name(static_cast<int>(control_state_));
    }
    if (!initial_done_ && (!a_button_seen_ || !fresh(last_xr_at_, input_timeout_ms_))) {
      return "Waiting for a fresh PICO4 A state before startup MoveJ";
    }
    if (!initial_done_ && a_button_raw_) {return "Release PICO4 A before startup MoveJ";}
    if (!initial_done_ && unlocked_) {return "Press A to lock before startup MoveJ";}
    if (!initial_done_) {return "Waiting for initial-position MoveJ";}
    if (pending_return_) {return "Waiting for B return MoveJ";}
    if (!fresh(last_xr_at_, input_timeout_ms_)) {return "PICO4 input is stale";}
    if (!unlocked_) {return "A software lock is engaged";}
    if (release_required_) {return "Waiting for both grips to be released";}
    if (!fresh(left_.tcp_at, feedback_timeout_ms_) || !left_.tcp_valid ||
        !fresh(right_.tcp_at, feedback_timeout_ms_) || !right_.tcp_valid ||
        !fresh(joint_at_, feedback_timeout_ms_)) {
      return "Joint or TCP feedback is stale";
    }
    if (!fresh(occupancy_at_, feedback_timeout_ms_) || occupancy_ != 0) {
      return "WA1 upper limb is occupied or occupancy feedback is stale";
    }
    if (!fresh(mode_at_, feedback_timeout_ms_) || robot_mode_ != 0) {
      return "WA1 upper limb is not STOPPED";
    }
    if (!fresh(bridge_at_, 2000) || bridge_read_only_ || !bridge_ready_ ||
        bridge_master_ != "http://192.168.217.1:11311" || bridge_connection_ != 1) {
      return "Bridge feedback or ServoL connection is not ready";
    }
    if (bridge_fault_) {return "Waiting for bridge stream fault reset";}
    if (!any_active()) {return "Hold a grip to start teleoperation";}
    if (!set_client_->service_is_ready() || !clear_client_->service_is_ready() ||
        !stop_client_->service_is_ready() || !reset_client_->service_is_ready() ||
        command_pub_->get_subscription_count() != 1) {
      return "ServoL publisher or services are not ready";
    }
    return "";
  }

  bool ready_for_preset(bool require_xr) const {
    return (!require_xr || fresh(last_xr_at_, input_timeout_ms_)) &&
      fresh(joint_at_, feedback_timeout_ms_) &&
      fresh(occupancy_at_, feedback_timeout_ms_) && occupancy_ == 0 &&
      fresh(mode_at_, feedback_timeout_ms_) && robot_mode_ == 0 &&
      fresh(bridge_at_, 2000) && !bridge_read_only_ && bridge_ready_ &&
      bridge_master_ == "http://192.168.217.1:11311" &&
      fresh(left_.tcp_at, feedback_timeout_ms_) && left_.tcp_valid &&
      fresh(right_.tcp_at, feedback_timeout_ms_) && right_.tcp_valid;
  }

  bool any_active() const {
    return (left_.grip_active && left_.target_valid) ||
      (right_.grip_active && right_.target_valid);
  }

  bool selected_grips_released() const {
    return std::isfinite(left_grip_value_) && left_grip_value_ <= grip_off_ &&
      std::isfinite(right_grip_value_) && right_grip_value_ <= grip_off_;
  }

  upperlimb::srv::Servo::Request::SharedPtr servo_params() const {
    auto request = std::make_shared<upperlimb::srv::Servo::Request>();
    request->v = 0.0;
    request->acc = 0.0;
    request->time = 1.0/rate_hz_;
    request->lookahead_time = 0.2;
    request->gain = 100;
    request->arm_type = 3;
    return request;
  }

  void begin_reset() {
    if (!reset_client_->service_is_ready() || robot_mode_ != 0) {return;}
    enter(ControlState::RESETTING);
    reset_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (control_state_ != ControlState::RESETTING) {return;}
        try {
          if (!future.get()->success) {fault("Bridge stream fault reset failed"); return;}
          bridge_fault_ = false;
          enter(ControlState::IDLE);
        } catch (const std::exception & error) {
          fault(std::string("Bridge stream fault reset failed: ") + error.what());
        }
      });
  }

  std::string recovery_block_reason() const {
    if (!enable_control_) {return "Control is disabled";}
    if (!initial_done_) {return "Initial position has not been confirmed";}
    if (fault_cleanup_started_) {return "Fault cleanup is still in progress";}
    if (!fresh(last_xr_at_, input_timeout_ms_)) {return "PICO4 input is stale";}
    if (!std::isfinite(left_grip_value_) || !std::isfinite(right_grip_value_) ||
        left_grip_value_ >= grip_off_ || right_grip_value_ >= grip_off_) {
      return "Release both grips before pressing A to recover";
    }
    if (!fresh(joint_at_, feedback_timeout_ms_) ||
        !fresh(left_.tcp_at, feedback_timeout_ms_) || !left_.tcp_valid ||
        !fresh(right_.tcp_at, feedback_timeout_ms_) || !right_.tcp_valid ||
        !fresh(occupancy_at_, feedback_timeout_ms_)) {
      return "WA1 joint, TCP or occupancy feedback is stale";
    }
    if (occupancy_ != 0) {return "WA1 upper limb is occupied";}
    if (!fresh(mode_at_, feedback_timeout_ms_) || robot_mode_ != 0) {
      return "WA1 upper limb is not STOPPED";
    }
    if (!fresh(bridge_at_, 2000) || bridge_read_only_ || !bridge_ready_ ||
        bridge_master_ != "http://192.168.217.1:11311" || bridge_connection_ != 1) {
      return "Bridge feedback or ServoL connection is not ready";
    }
    if ((configured_ && !clear_client_->service_is_ready()) ||
        (bridge_fault_ && !reset_client_->service_is_ready())) {
      return "Recovery service is not ready";
    }
    return "";
  }

  void recovery_failed(const std::string & reason) {
    if (control_state_ != ControlState::RECOVERING) {return;}
    unlocked_ = false;
    release_required_ = true;
    reason_ = reason;
    enter(ControlState::FAULT);
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
  }

  void recovery_complete() {
    if (control_state_ != ControlState::RECOVERING) {return;}
    const auto blocked = recovery_block_reason();
    if (!blocked.empty()) {recovery_failed("Recovery interrupted: " + blocked); return;}
    bridge_fault_ = false;
    unlocked_ = true;
    release_required_ = true;
    pending_return_ = false;
    reason_ = "Recovered after manual WA1 protection release; regrip to control";
    enter(ControlState::IDLE);
    RCLCPP_INFO(get_logger(), "%s", reason_.c_str());
  }

  void reset_bridge_for_recovery() {
    if (control_state_ != ControlState::RECOVERING) {return;}
    if (!bridge_fault_) {recovery_complete(); return;}
    reset_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (control_state_ != ControlState::RECOVERING) {return;}
        try {
          if (!future.get()->success) {
            recovery_failed("Bridge stream fault reset rejected during recovery"); return;
          }
          bridge_fault_ = false;
          recovery_complete();
        } catch (const std::exception & error) {
          recovery_failed(std::string("Bridge fault reset failed during recovery: ") +
            error.what());
        }
      });
  }

  void begin_fault_recovery() {
    const auto blocked = recovery_block_reason();
    if (!blocked.empty()) {
      RCLCPP_WARN(get_logger(), "A recovery request refused: %s", blocked.c_str());
      return;
    }
    release_required_ = true;
    enter(ControlState::RECOVERING);
    if (!configured_) {reset_bridge_for_recovery(); return;}
    clear_client_->async_send_request(servo_params(),
      [this](rclcpp::Client<upperlimb::srv::Servo>::SharedFuture future) {
        if (control_state_ != ControlState::RECOVERING) {return;}
        try {
          if (!future.get()->success) {
            recovery_failed("Clear servo parameters rejected during recovery"); return;
          }
          configured_ = false;
          reset_bridge_for_recovery();
        } catch (const std::exception & error) {
          recovery_failed(std::string("Clear servo parameters failed during recovery: ") +
            error.what());
        }
      });
  }

  void begin_initial_move(bool from_b_button) {
    if (!ready_for_preset(from_b_button) || !movej_client_->service_is_ready() ||
        !stop_client_->service_is_ready()) {return;}
    if (bridge_fault_) {begin_reset(); return;}
    auto request = std::make_shared<upperlimb::srv::MoveJ::Request>();
    request->joints = initial_left_;
    request->joints.insert(request->joints.end(), initial_right_.begin(), initial_right_.end());
    request->v = movej_v_;
    request->acc = movej_acc_;
    request->t = 0.0;
    request->is_async = false;
    request->arm_type = 3;
    preset_rpc_done_ = false;
    release_required_ = true;
    initial_move_from_b_ = from_b_button;
    if (from_b_button) {++b_return_movej_requests_;}
    enter(ControlState::MOVING_INITIAL);
    RCLCPP_INFO(get_logger(), "Calling dual-arm MoveJ to operator-defined initial position: %s "
      "(v=%.3f, acc=%.3f)", from_b_button ? "B button" : "startup", movej_v_, movej_acc_);
    movej_client_->async_send_request(request,
      [this](rclcpp::Client<upperlimb::srv::MoveJ>::SharedFuture future) {
        if (control_state_ != ControlState::MOVING_INITIAL) {return;}
        try {
          const auto response = future.get();
          if (!response->success) {
            fault("Dual-arm MoveJ to initial position rejected: " + response->message); return;
          }
          preset_rpc_done_ = true;
        } catch (const std::exception & error) {
          fault(std::string("Dual-arm MoveJ to initial position failed: ") + error.what());
        }
      });
  }

  void begin_control() {
    if (!set_client_->service_is_ready() || !clear_client_->service_is_ready() ||
        !stop_client_->service_is_ready() || !reset_client_->service_is_ready()) {return;}
    if (command_pub_->get_subscription_count() != 1) {return;}
    if (bridge_fault_) {begin_reset(); return;}
    configure_servo();
  }

  void configure_servo() {
    if (!any_active() || !unlocked_ || !ready_for_control(false)) {
      enter(ControlState::IDLE); return;
    }
    enter(ControlState::CONFIGURING);
    set_client_->async_send_request(servo_params(),
      [this](rclcpp::Client<upperlimb::srv::Servo>::SharedFuture future) {
        try {
          if (!future.get()->success) {fault("Set servo parameters failed"); return;}
          configured_ = true;
          if (control_state_ != ControlState::CONFIGURING) {
            fault("Late set servo response after control state changed"); return;
          }
          if (!any_active() || !unlocked_ || !ready_for_control(false)) {
            halt("Input changed before servo stream"); return;
          }
          left_.hold = left_.tcp;
          right_.hold = right_.tcp;
          left_.hold_valid = right_.hold_valid = true;
          left_.limiter.reset(left_.tcp);
          right_.limiter.reset(right_.tcp);
          last_command_at_ = Steady::now();
          enter(ControlState::STREAMING);
        } catch (const std::exception & error) {
          fault(std::string("Set servo parameters failed: ") + error.what());
        }
      });
  }

  void halt(const std::string & reason, bool require_release = true) {
    if (control_state_ == ControlState::STOPPING || control_state_ == ControlState::CLEARING ||
        control_state_ == ControlState::FAULT || control_state_ == ControlState::IDLE) {return;}
    if (require_release) {release_required_ = true;}
    reason_ = reason;
    if (!configured_) {enter(ControlState::IDLE); return;}
    enter(ControlState::STOPPING);
    if (!stop_client_->service_is_ready()) {fault("Stop service unavailable"); return;}
    stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          if (!future.get()->success) {fault("WA1 stop service rejected request", true); return;}
          if (control_state_ == ControlState::STOPPING) {clear_servo();}
        } catch (const std::exception & error) {
          fault(std::string("WA1 stop service failed: ") + error.what(), true);
        }
      });
  }

  void clear_servo() {
    enter(ControlState::CLEARING);
    if (!clear_client_->service_is_ready()) {fault("Clear servo parameters service unavailable"); return;}
    clear_client_->async_send_request(servo_params(),
      [this](rclcpp::Client<upperlimb::srv::Servo>::SharedFuture future) {
        try {
          if (!future.get()->success) {fault("Clear servo parameters failed", true); return;}
          configured_ = false;
          if (control_state_ == ControlState::CLEARING) {
            // A grip pressed during stop/clear starts from the latest hand and TCP poses.
            left_.grip_active = right_.grip_active = false;
            left_.target_valid = right_.target_valid = false;
            enter(ControlState::IDLE);
          }
        } catch (const std::exception & error) {
          fault(std::string("Clear servo parameters failed: ") + error.what(), true);
        }
      });
  }

  void clear_after_fault(uint64_t token) {
    if (token != fault_cleanup_token_) {return;}
    if (!clear_client_ || !clear_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Servo parameters may remain configured; clear service unavailable");
      fault_cleanup_started_ = false;
      return;
    }
    clear_client_->async_send_request(servo_params(),
      [this, token](rclcpp::Client<upperlimb::srv::Servo>::SharedFuture future) {
        if (token != fault_cleanup_token_) {return;}
        try {
          if (future.get()->success) {configured_ = false;}
          else {RCLCPP_ERROR(get_logger(), "Fault cleanup: clear servo parameters rejected");}
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Fault cleanup: clear servo parameters failed: %s", error.what());
        }
        fault_cleanup_started_ = false;
      });
  }

  void fault(const std::string & reason, bool stop_already_attempted = false) {
    const bool preset_moving = control_state_ == ControlState::MOVING_INITIAL;
    reason_ = reason;
    release_required_ = true;
    unlocked_ = false;
    pending_return_ = false;
    enter(ControlState::FAULT);
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
    if (preset_moving && stop_client_ && stop_client_->service_is_ready()) {
      stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    if (!configured_ || fault_cleanup_started_) {return;}
    fault_cleanup_started_ = true;
    fault_cleanup_at_ = Steady::now();
    const uint64_t token = ++fault_cleanup_token_;
    if (stop_already_attempted || !stop_client_ || !stop_client_->service_is_ready()) {
      if (!stop_already_attempted) {
        RCLCPP_ERROR(get_logger(), "Fault cleanup: WA1 stop service unavailable");
      }
      clear_after_fault(token);
      return;
    }
    stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this, token](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        if (token != fault_cleanup_token_) {return;}
        try {
          if (!future.get()->success) {RCLCPP_ERROR(get_logger(), "Fault cleanup: WA1 stop rejected");}
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Fault cleanup: WA1 stop failed: %s", error.what());
        }
        clear_after_fault(token);
      });
  }

  void publish_command() {
    upperlimb::msg::DualPose command;
    const bool left_active = left_.grip_active && left_.target_valid;
    const bool right_active = right_.grip_active && right_.target_valid;
    const auto current_time = Steady::now();
    const double elapsed = last_command_at_ == Steady::time_point{} ? 1.0/rate_hz_ :
      std::chrono::duration<double>(current_time-last_command_at_).count();
    last_command_at_ = current_time;
    const double dt = std::clamp(elapsed, 1e-6, 2.0/rate_hz_);
    to_ros(left_active ? left_.limiter.step(left_.target, dt, math::kTeleopMotionLimits) :
      left_.hold, command.left_arm_pose);
    to_ros(right_active ? right_.limiter.step(right_.target, dt, math::kTeleopMotionLimits) :
      right_.hold, command.right_arm_pose);
    command_pub_->publish(command);
    ++commands_published_;
  }

  void tick() {
    if (!fresh(last_xr_at_, input_timeout_ms_)) {
      if (!xr_timed_out_ && last_xr_at_ != Steady::time_point{}) {
        xr_timed_out_ = true;
        ++xr_timeout_count_;
      }
      left_.grip_active = right_.grip_active = false;
      left_.target_valid = right_.target_valid = false;
      left_.last_xr_valid = right_.last_xr_valid = false;
      if (control_state_ == ControlState::STREAMING) {halt("XR input timeout");}
      release_required_ = true;
      pending_return_ = false;
    }
    if (control_state_ == ControlState::FAULT && fault_cleanup_started_ &&
        !fresh(fault_cleanup_at_, 3000)) {
      ++fault_cleanup_token_;
      fault_cleanup_started_ = false;
      RCLCPP_WARN(get_logger(), "Fault cleanup timed out; a new A press can retry after WA1 is STOPPED");
    }
    if (control_state_ == ControlState::RECOVERING && !fresh(state_at_, 6000)) {
      recovery_failed("Teleoperation recovery service response timeout");
    }
    if (control_state_ == ControlState::MOVING_INITIAL) {
      if (robot_mode_ == 15) {
        fault("WA1 entered PROTECTED during initial-position MoveJ");
      } else if (!fresh(state_at_, 60000)) {
        fault("Initial-position MoveJ did not finish within 60 s");
      } else if (preset_rpc_done_ && robot_mode_ == 0 &&
                 reached_preset(initial_left_, initial_right_)) {
        initial_done_ = true;
        if (initial_move_from_b_) {pending_return_ = false;}
        reason_ = initial_move_from_b_ ? "Returned to teleoperation initial position" :
          "Teleoperation initial position reached at startup";
        release_required_ = true;
        enter(ControlState::IDLE);
      }
    }
    if ((control_state_ == ControlState::RESETTING ||
         control_state_ == ControlState::CONFIGURING ||
         control_state_ == ControlState::STOPPING ||
         control_state_ == ControlState::CLEARING) &&
        !fresh(state_at_, 3000)) {
      const bool stop_attempted = control_state_ == ControlState::STOPPING ||
        control_state_ == ControlState::CLEARING;
      fault("Teleoperation service response timeout", stop_attempted);
    }
    if (enable_control_) {
      if (control_state_ == ControlState::IDLE) {
        if (!initial_done_) {
          if (a_button_seen_ && !a_button_raw_ && !unlocked_ && ready_for_preset(true)) {
            begin_initial_move(false);
          }
        } else if (pending_return_) {
          if (unlocked_ && ready_for_preset(true)) {begin_initial_move(true);}
        } else if (unlocked_ && bridge_fault_ && !release_required_ && !any_active() &&
                   ready_for_preset(true)) {
          // A previously latched PC bridge watchdog can be cleared while both grips are released.
          begin_reset();
        } else if (!release_required_ && unlocked_ && any_active() &&
                   ready_for_control(true) && robot_mode_ == 0) {
          begin_control();
        }
      } else if (control_state_ == ControlState::STREAMING) {
        if (!unlocked_ || !any_active() || !left_.hold_valid || !right_.hold_valid ||
            !ready_for_control(false) ||
            (robot_mode_ != 0 && robot_mode_ != 14)) {
          const bool normal_grip_release = unlocked_ && !any_active() &&
            selected_grips_released() && left_.hold_valid && right_.hold_valid &&
            ready_for_control(false) && (robot_mode_ == 0 || robot_mode_ == 14);
          halt(normal_grip_release ? "Selected grips released" :
            "Input or WA1 feedback no longer permits ServoL", !normal_grip_release);
        } else {publish_command();}
      }
    }
    publish_status();
  }

  void publish_status() {
    Json::Value status;
    status["control_enabled"] = enable_control_;
    status["control_scope"] = "dual_arm";
    status["translation_scale"] = translation_scale_;
    status["sensitivity_level"] = math::kTeleopSensitivityLevel;
    status["motion_limits"]["linear_speed_m_s"] = math::kTeleopMotionLimits.linear_speed;
    status["motion_limits"]["angular_speed_rad_s"] = math::kTeleopMotionLimits.angular_speed;
    status["motion_limits"]["linear_acceleration_m_s2"] =
      math::kTeleopMotionLimits.linear_acceleration;
    status["motion_limits"]["angular_acceleration_rad_s2"] =
      math::kTeleopMotionLimits.angular_acceleration;
    status["software_unlocked"] = unlocked_;
    status["a_button_seen"] = a_button_seen_;
    status["a_button_raw"] = a_button_raw_;
    status["a_button_press_count"] = Json::UInt64(a_button_press_count_);
    status["a_button_last_action"] = a_button_last_action_;
    status["release_required"] = release_required_;
    status["xr_timeout_count"] = Json::UInt64(xr_timeout_count_);
    status["xr_device_timestamp_nonmonotonic_count"] =
      Json::UInt64(device_timestamp_nonmonotonic_count_);
    status["recovery_ready"] = control_state_ == ControlState::FAULT &&
      recovery_block_reason().empty();
    status["recovery_block_reason"] = control_state_ == ControlState::FAULT ?
      recovery_block_reason() : "";
    status["left_grip_active"] = left_.grip_active;
    status["right_grip_active"] = right_.grip_active;
    status["initial_done"] = initial_done_;
    status["b_return_enabled"] = true;
    status["b_return_movej_requests"] = Json::UInt64(b_return_movej_requests_);
    status["pending_return"] = pending_return_;
    status["initial_move_source"] = control_state_ == ControlState::MOVING_INITIAL ?
      (initial_move_from_b_ ? "b_button" : "startup") : "";
    status["b_hold_ms"] = b_hold_since_ == Steady::time_point{} ? 0 :
      std::chrono::duration_cast<std::chrono::milliseconds>(Steady::now()-b_hold_since_).count();
    status["left_target_valid"] = left_.target_valid;
    status["right_target_valid"] = right_.target_valid;
    status["xr_fresh"] = fresh(last_xr_at_, input_timeout_ms_);
    status["left_tcp_fresh"] = fresh(left_.tcp_at, feedback_timeout_ms_);
    status["right_tcp_fresh"] = fresh(right_.tcp_at, feedback_timeout_ms_);
    status["joint_fresh"] = fresh(joint_at_, feedback_timeout_ms_);
    status["bridge_ready"] = bridge_ready_;
    status["bridge_stream_fault"] = bridge_fault_;
    status["robot_mode"] = robot_mode_;
    status["control_state"] = state_name(static_cast<int>(control_state_));
    status["control_block_reason"] = control_block_reason();
    status["servo_configured"] = configured_;
    status["commands_published"] = Json::UInt64(commands_published_);
    status["preset_initial_left"] = Json::arrayValue;
    status["preset_initial_right"] = Json::arrayValue;
    for (double value : initial_left_) {status["preset_initial_left"].append(value);}
    for (double value : initial_right_) {status["preset_initial_right"].append(value);}
    status["reason"] = reason_;
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    std_msgs::msg::String message;
    message.data = Json::writeString(writer, status);
    status_pub_->publish(message);
  }

  bool enable_control_{true}, a_button_seen_{false}, a_button_raw_{false};
  bool unlocked_{false};
  bool release_required_{false}, configured_{false}, fault_cleanup_started_{false};
  bool xr_timed_out_{false};
  bool initial_done_{false}, pending_return_{false}, b_consumed_{false};
  bool preset_rpc_done_{false}, initial_move_from_b_{false};
  std::string pose_layout_, reason_, bridge_master_;
  std::string a_button_last_action_{"none"};
  double translation_scale_{1.0}, grip_on_{0.8}, grip_off_{0.6}, rate_hz_{100.0};
  double movej_v_{0.2}, movej_acc_{0.2}, preset_tolerance_rad_{0.03};
  double max_xr_step_m_{0.2}, max_xr_step_rad_{1.0};
  std::vector<double> initial_left_, initial_right_;
  std::map<std::string, double> joints_;
  int input_timeout_ms_{100}, feedback_timeout_ms_{250};
  Side left_, right_;
  Steady::time_point last_xr_at_{}, joint_at_{}, occupancy_at_{}, mode_at_{}, bridge_at_{},
    state_at_{}, b_hold_since_{}, fault_cleanup_at_{}, last_command_at_{};
  double left_grip_value_{NAN}, right_grip_value_{NAN};
  int64_t last_pc_receive_ns_{0}, last_device_stamp_ns_{0};
  int occupancy_{-1}, robot_mode_{-1}, bridge_connection_{0};
  bool bridge_read_only_{true}, bridge_ready_{false}, bridge_fault_{false};
  uint64_t commands_published_{0};
  uint64_t fault_cleanup_token_{0}, xr_timeout_count_{0};
  uint64_t device_timestamp_nonmonotonic_count_{0};
  uint64_t a_button_press_count_{0};
  uint64_t b_return_movej_requests_{0};
  ControlState control_state_{ControlState::IDLE};
  rclcpp::Subscription<pico4_ros2::msg::XrFrame>::SharedPtr xr_sub_;
  rclcpp::Subscription<upperlimb::msg::Pose>::SharedPtr left_tcp_sub_, right_tcp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr occupancy_sub_;
  rclcpp::Subscription<upperlimb::msg::UplimbState>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bridge_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_preview_pub_, right_preview_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_trigger_pub_, right_trigger_pub_;
  rclcpp::Publisher<upperlimb::msg::DualPose>::SharedPtr command_pub_;
  rclcpp::Client<upperlimb::srv::Servo>::SharedPtr set_client_, clear_client_;
  rclcpp::Client<upperlimb::srv::MoveJ>::SharedPtr movej_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_, reset_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Wa1Teleop>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "wa1_teleop: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
