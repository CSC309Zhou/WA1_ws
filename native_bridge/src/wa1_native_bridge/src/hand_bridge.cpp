#include "wa1_native_bridge/hand_bridge.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

namespace wa1_native_bridge {
namespace {

constexpr const char * kRos1Prefix = "/zj_humanoid/hand/";
constexpr const char * kRos2Prefix = "/hand/";

struct Service {
  std::string suffix;
  std::string package;
  std::string type;
};

struct Topic {
  std::string suffix;
  std::string ros1_type;
  std::string ros2_type;
};

}  // namespace

HandBridge::HandBridge(ros::NodeHandle ros1_node)
: ros1_node_(std::move(ros1_node)),
  data_node_(std::make_shared<rclcpp::Node>("wa1_hand_bridge"))
{
  const std::vector<Service> services = {
    {"joint_switch/left", "hand", "HandJoint"},
    {"joint_switch/right", "hand", "HandJoint"},
    {"joint_switch/dual", "hand", "HandJoint"},
    {"gesture_switch/left", "hand", "Gesture"},
    {"gesture_switch/right", "hand", "Gesture"},
    {"gesture_switch/dual", "hand", "Gesture"},
    {"finger_pressures/left/zero", "std_srvs", "Trigger"},
    {"finger_pressures/right/zero", "std_srvs", "Trigger"},
  };
  std::vector<std::string> remaps{"--ros-args"};
  for (const auto & service : services) {
    remaps.insert(remaps.end(), {
      "-r",
      std::string(kRos1Prefix) + service.suffix + ":=" +
      kRos2Prefix + service.suffix,
    });
  }
  rclcpp::NodeOptions options;
  options.use_global_arguments(false).arguments(remaps);
  service_node_ = std::make_shared<rclcpp::Node>("wa1_hand_services", options);
  for (const auto & service : services) {
    auto factory = ros1_bridge::get_service_factory(
      "ros1", service.package, service.type);
    if (!factory) {
      throw std::runtime_error(
        "Missing hand service factory: " + service.package + "/" + service.type);
    }
    service_handles_.push_back(factory->service_bridge_2_to_1(
      ros1_node_, service_node_, std::string(kRos1Prefix) + service.suffix));
    service_factories_.push_back(std::move(factory));
  }
  service_executor_.add_node(service_node_);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  const std::vector<Topic> topics = {
    {"joint_states", "sensor_msgs/JointState", "sensor_msgs/msg/JointState"},
    {"finger_pressures/left", "hand/PressureSensor", "hand/msg/PressureSensor"},
    {"finger_pressures/right", "hand/PressureSensor", "hand/msg/PressureSensor"},
  };
  for (const auto & topic : topics) {
    uplinks_.push_back(ros1_bridge::create_bridge_from_1_to_2(
      ros1_node_, data_node_, topic.ros1_type,
      std::string(kRos1Prefix) + topic.suffix, 1, topic.ros2_type,
      std::string(kRos2Prefix) + topic.suffix, qos));
  }
  RCLCPP_INFO(data_node_->get_logger(), "WA1 hand bridge ready: 3 topics, 8 services");
}

HandBridge::~HandBridge()
{
  stop();
}

void HandBridge::start()
{
  if (service_thread_.joinable()) {
    return;
  }
  service_thread_ = std::thread([this]() {
    while (rclcpp::ok()) {
      try {
        service_executor_.spin();
        break;
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          data_node_->get_logger(), "Hand service forwarding failed: %s", error.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  });
}

void HandBridge::stop()
{
  if (service_thread_.joinable()) {
    service_executor_.cancel();
    service_thread_.join();
  }
}

}  // namespace wa1_native_bridge
