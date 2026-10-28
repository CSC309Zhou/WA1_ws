#pragma once

#include <memory>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <ros1_bridge/bridge.hpp>

namespace wa1_native_bridge {

class HandBridge {
 public:
  explicit HandBridge(ros::NodeHandle ros1_node);
  ~HandBridge();

  void start();
  void stop();

 private:
  ros::NodeHandle ros1_node_;
  rclcpp::Node::SharedPtr data_node_;
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::executors::SingleThreadedExecutor service_executor_;
  std::thread service_thread_;
  std::vector<ros1_bridge::Bridge1to2Handles> uplinks_;
  std::vector<std::unique_ptr<ros1_bridge::ServiceFactoryInterface>> service_factories_;
  std::vector<ros1_bridge::ServiceBridge2to1> service_handles_;
};

}  // namespace wa1_native_bridge
