#include "pico4_ros2/xr_json_parser.hpp"

#include <cmath>
#include <cctype>
#include <cstdlib>
#include <limits>
#include <memory>
#include <json/json.h>

namespace pico4_ros2 {
namespace {
const Json::Value & field(const Json::Value & value, const char * primary, const char * alternate = nullptr) {
  if (value.isObject() && value.isMember(primary)) {return value[primary];}
  if (alternate && value.isObject() && value.isMember(alternate)) {return value[alternate];}
  static const Json::Value null_value;
  return null_value;
}

bool number(const Json::Value & value, float & out) {
  if (!value.isNumeric()) {return false;}
  const double v = value.asDouble();
  if (!std::isfinite(v) || std::abs(v) > std::numeric_limits<float>::max()) {return false;}
  out = static_cast<float>(v);
  return true;
}

bool optional_number(const Json::Value & value, float & out) {
  if (value.isNull()) {out = 0.0f; return true;}
  return number(value, out);
}

bool optional_int(const Json::Value & value, int32_t & out) {
  if (value.isNull()) {out = 0; return true;}
  if (!value.isIntegral()) {return false;}
  const auto n = value.asInt64();
  if (n < std::numeric_limits<int32_t>::min() || n > std::numeric_limits<int32_t>::max()) {return false;}
  out = static_cast<int32_t>(n);
  return true;
}

bool optional_bool(const Json::Value & value, bool & out) {
  if (value.isNull()) {out = false; return true;}
  if (!value.isBool() && !value.isIntegral()) {return false;}
  out = value.asBool();
  return true;
}

bool parse_pose(const Json::Value & source, msg::XrPose & out, std::string & error) {
  const Json::Value & values = source.isArray() ? source : field(source, "pose", "Pose");
  if (values.isArray() && values.size() == 7) {
    for (Json::ArrayIndex i = 0; i < 7; ++i) {
      if (!number(values[i], out.pose[i])) {
        error = "Non-finite or invalid pose component"; return false;
      }
    }
  } else if (values.isString()) {
    // XRoboToolkit 1.0.0.0 sends pose as seven comma-separated numbers.
    const std::string csv = values.asString();
    const char * current = csv.c_str();
    const char * const end = current + csv.size();
    for (size_t i = 0; i < out.pose.size(); ++i) {
      while (current < end && std::isspace(static_cast<unsigned char>(*current))) {++current;}
      char * next = nullptr;
      const double component = std::strtod(current, &next);
      if (next == current || !std::isfinite(component) ||
          std::abs(component) > std::numeric_limits<float>::max()) {
        error = "Non-finite or invalid CSV pose component"; return false;
      }
      out.pose[i] = static_cast<float>(component);
      current = next;
      while (current < end && std::isspace(static_cast<unsigned char>(*current))) {++current;}
      if (i + 1 < out.pose.size()) {
        if (current == end || *current != ',') {
          error = "CSV pose must contain seven numbers"; return false;
        }
        ++current;
      }
    }
    if (current != end) {error = "CSV pose has extra components"; return false;}
  } else {
    error = "Head/Controller pose must contain seven numbers";
    return false;
  }
  const Json::Value & status = field(source, "status", "Status");
  if (status.isNull()) {
    out.status = -1;  // SDK did not provide a tracking status for this pose.
  } else if (!optional_int(status, out.status)) {
    error = "Invalid tracking status"; return false;
  }
  return true;
}

bool parse_controller(const Json::Value & value, msg::XrController & out, std::string & error) {
  if (!value.isObject()) {error = "Controller entry must be an object"; return false;}
  if (!optional_number(field(value, "axisX", "axis_x"), out.axis_x) ||
      !optional_number(field(value, "axisY", "axis_y"), out.axis_y) ||
      !optional_number(field(value, "grip", "gripper"), out.gripper) ||
      !optional_number(field(value, "trigger"), out.trigger) ||
      !optional_bool(field(value, "axisClick", "axis_click"), out.axis_click) ||
      !optional_bool(field(value, "primaryButton", "primary_button"), out.primary_button) ||
      !optional_bool(field(value, "secondaryButton", "secondary_button"), out.secondary_button) ||
      !optional_bool(field(value, "menuButton", "menu_button"), out.menu_button)) {
    error = "Invalid controller axis or button field"; return false;
  }
  return parse_pose(value, out.tracked_pose, error);
}
}  // namespace

bool parse_xr_json(const std::string & json, msg::XrFrame & frame, std::string & error) {
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  Json::Value root;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  if (!reader->parse(json.data(), json.data() + json.size(), &root, &error)) {return false;}
  if (!root.isObject()) {error = "SDK JSON root must be an object"; return false;}
  if (root.isMember("functionName")) {
    if (root["functionName"].asString() != "Tracking") {
      error = "Non-tracking SDK event"; return false;
    }
    const Json::Value & payload = root["value"];
    if (payload.isString()) {
      const std::string nested = payload.asString();
      if (!reader->parse(nested.data(), nested.data() + nested.size(), &root, &error)) {
        error = "Invalid Tracking value JSON: " + error;
        return false;
      }
    } else if (payload.isObject()) {
      root = payload;
    } else {
      error = "Tracking value must be a JSON object or JSON string"; return false;
    }
    if (!root.isObject()) {error = "Tracking value must contain an object"; return false;}
  }
  if (!root.isMember("Head") && root.isMember("data")) {root = root["data"];}
  const Json::Value & timestamp = field(root, "timeStampNs", "timestamp_ns");
  if (!timestamp.isNull()) {
    try {
      if (timestamp.isString()) {
        size_t consumed = 0;
        const auto text = timestamp.asString();
        frame.device_timestamp_ns = std::stoll(text, &consumed);
        if (consumed != text.size()) {error = "Invalid device timestamp"; return false;}
      } else if (timestamp.isIntegral()) {
        frame.device_timestamp_ns = timestamp.asInt64();
      } else {
        error = "Invalid device timestamp";
        return false;
      }
    } catch (const std::exception &) {error = "Invalid device timestamp"; return false;}
  }
  if (!optional_int(field(root, "Input", "input"), frame.input) ||
      !optional_int(field(root, "Battery", "battery"), frame.battery)) {
    error = "Invalid Input/Battery"; return false;
  }
  if (!parse_pose(field(root, "Head", "head"), frame.head, error)) {return false;}
  const Json::Value & controllers = field(root, "Controller", "controllers");
  const Json::Value * left = nullptr, * right = nullptr;
  if (controllers.isArray() && controllers.size() >= 2) {
    left = &controllers[0]; right = &controllers[1];
  } else if (controllers.isObject()) {
    left = &field(controllers, "Left", "left");
    right = &field(controllers, "Right", "right");
  }
  if (!left || !right || !parse_controller(*left, frame.left_controller, error) ||
      !parse_controller(*right, frame.right_controller, error)) {
    if (error.empty()) {error = "Expected left and right Controller entries";}
    return false;
  }
  error.clear();
  return true;
}
}  // namespace pico4_ros2
