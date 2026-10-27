#include <cstdint>
#include <PXREARobotSDK.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <json/json.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "pico4_ros2/msg/xr_frame.hpp"
#include "pico4_ros2/xr_json_parser.hpp"

using namespace std::chrono_literals;

namespace {
int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class XrReceiver final : public rclcpp::Node {
 public:
  XrReceiver() : Node("pico4_xr_receiver") {
    replay_path_ = declare_parameter<std::string>("replay_jsonl", "");
    replay_loop_ = declare_parameter<bool>("replay_loop", false);
    const double replay_hz = declare_parameter<double>("replay_rate_hz", 60.0);
    publish_raw_ = declare_parameter<bool>("publish_raw_json", false);
    const std::string default_sdk = ament_index_cpp::get_package_share_directory("pico4_ros2") +
      "/vendor/roboticsservice/SDK/x64/libPXREARobotSDK.so";
    sdk_path_ = declare_parameter<std::string>("sdk_library", default_sdk);
    frame_pub_ = create_publisher<pico4_ros2::msg::XrFrame>(
      "/pico4/frame", rclcpp::QoS(1).best_effort());
    raw_pub_ = create_publisher<std_msgs::msg::String>(
      "/pico4/raw_json", rclcpp::QoS(1).best_effort());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/pico4/status", rclcpp::QoS(1).reliable().transient_local());
    status_timer_ = create_wall_timer(1s, [this] {publish_status();});
    if (!replay_path_.empty()) {
      load_replay();
      if (!(replay_hz > 0.0 && replay_hz <= 500.0)) {
        throw std::invalid_argument("replay_rate_hz must be in (0, 500]");
      }
      replay_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / replay_hz)),
        [this] {publish_next_replay();});
      RCLCPP_INFO(get_logger(), "XR JSONL replay ready: %zu frames; no SDK connection", replay_lines_.size());
    } else {
      sdk_timer_ = create_wall_timer(2s, [this] {ensure_sdk();});
      ensure_sdk();
    }
  }

  ~XrReceiver() override {
    if (sdk_initialized_ && deinit_) {deinit_();}
    if (sdk_handle_) {dlclose(sdk_handle_);}
  }

 private:
  using InitFn = decltype(&PXREAInit);
  using DeinitFn = decltype(&PXREADeinit);

  static void sdk_callback(void * context, PXREAClientCallbackType type, int status, void * user_data) {
    auto * self = static_cast<XrReceiver *>(context);
    if (self) {self->on_sdk_event(type, status, user_data);}
  }

  void on_sdk_event(PXREAClientCallbackType type, int, void * user_data) {
    if (type == PXREAServerConnect) {server_connected_ = true; return;}
    if (type == PXREAServerDisconnect) {server_connected_ = false; return;}
    if (type == PXREADeviceFind || type == PXREADeviceConnect) {device_online_ = true; return;}
    if (type == PXREADeviceMissing) {device_online_ = false; return;}
    if (type != PXREADeviceStateJson || !user_data) {return;}
    const auto * state = static_cast<const PXREADevStateJson *>(user_data);
    const size_t size = strnlen(state->stateJson, sizeof(state->stateJson));
    if (!size || size == sizeof(state->stateJson)) {++parse_errors_; return;}
    const std::string json(state->stateJson, size);
    const std::string id(state->devID, strnlen(state->devID, sizeof(state->devID)));
    publish_json(json, id);
  }

  void publish_json(const std::string & json, const std::string & id) {
    if (publish_raw_) {
      std_msgs::msg::String raw;
      raw.data = json;
      raw_pub_->publish(raw);
    }
    pico4_ros2::msg::XrFrame frame;
    frame.header.stamp = now();
    frame.header.frame_id = "pico4_tracking";
    frame.device_id = id;
    frame.pc_receive_monotonic_ns = monotonic_ns();
    std::string error;
    if (!pico4_ros2::parse_xr_json(json, frame, error)) {
      if (error == "Non-tracking SDK event") {return;}
      ++parse_errors_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "XR JSON parse failed: %s. Enable publish_raw_json to inspect device data.", error.c_str());
      return;
    }
    last_frame_ns_ = frame.pc_receive_monotonic_ns;
    ++frames_;
    device_online_ = true;
    frame_pub_->publish(frame);
  }

  void ensure_sdk() {
    if (sdk_initialized_) {return;}
    if (!sdk_handle_) {
      sdk_handle_ = dlopen(sdk_path_.c_str(), RTLD_NOW | RTLD_LOCAL);
      if (!sdk_handle_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Cannot load XR SDK: %s", dlerror());
        return;
      }
      init_ = reinterpret_cast<InitFn>(dlsym(sdk_handle_, "PXREAInit"));
      deinit_ = reinterpret_cast<DeinitFn>(dlsym(sdk_handle_, "PXREADeinit"));
      if (!init_ || !deinit_) {throw std::runtime_error("XR SDK lacks PXREAInit/PXREADeinit");}
    }
    const int result = init_(this, &XrReceiver::sdk_callback, PXREAFullMask);
    if (result == 0) {
      sdk_initialized_ = true;
      RCLCPP_INFO(get_logger(), "XR SDK initialized; waiting for PICO4 input");
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "XR SDK initialization returned %d; will retry", result);
    }
  }

  void load_replay() {
    std::ifstream file(replay_path_);
    if (!file) {throw std::runtime_error("Cannot open replay_jsonl: " + replay_path_);}
    for (std::string line; std::getline(file, line);) {
      if (!line.empty()) {replay_lines_.push_back(std::move(line));}
    }
    if (replay_lines_.empty()) {throw std::runtime_error("Replay JSONL is empty");}
  }

  void publish_next_replay() {
    if (replay_index_ >= replay_lines_.size()) {
      if (!replay_loop_) {return;}
      replay_index_ = 0;
    }
    publish_json(replay_lines_[replay_index_++], "replay");
  }

  void publish_status() {
    Json::Value status;
    status["mode"] = replay_path_.empty() ? "live_sdk" : "jsonl_replay";
    status["sdk_initialized"] = sdk_initialized_.load();
    status["server_connected"] = server_connected_.load();
    status["device_online"] = device_online_.load();
    status["frames"] = Json::UInt64(frames_.load());
    status["parse_errors"] = Json::UInt64(parse_errors_.load());
    const int64_t last = last_frame_ns_.load();
    status["last_frame_age_ms"] = last > 0 ? (monotonic_ns() - last) / 1000000.0 : -1.0;
    std_msgs::msg::String message;
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "";
    message.data = Json::writeString(writer, status);
    status_pub_->publish(message);
  }

  std::string replay_path_, sdk_path_;
  bool replay_loop_{false}, publish_raw_{false};
  std::vector<std::string> replay_lines_;
  size_t replay_index_{0};
  void * sdk_handle_{nullptr};
  InitFn init_{nullptr};
  DeinitFn deinit_{nullptr};
  std::atomic<bool> sdk_initialized_{false}, server_connected_{false}, device_online_{false};
  std::atomic<uint64_t> frames_{0}, parse_errors_{0};
  std::atomic<int64_t> last_frame_ns_{0};
  rclcpp::Publisher<pico4_ros2::msg::XrFrame>::SharedPtr frame_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_, status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_, replay_timer_, sdk_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<XrReceiver>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "pico4_xr_receiver: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
