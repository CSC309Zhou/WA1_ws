#pragma once

#include <string>
#include <pico4_ros2/msg/xr_frame.hpp>

namespace pico4_ros2 {
// Parses the SDK JSON into raw XR fields without assigning a robot frame.
bool parse_xr_json(const std::string & json, msg::XrFrame & frame, std::string & error);
}  // namespace pico4_ros2
