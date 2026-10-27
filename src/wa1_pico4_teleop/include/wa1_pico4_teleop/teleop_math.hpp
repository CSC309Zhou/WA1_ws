#pragma once

#include <array>
#include <string>

namespace wa1_pico4_teleop {
struct Vector3 {double x{}, y{}, z{};};
struct Quaternion {double x{}, y{}, z{}, w{1.0};};
struct Pose {Vector3 position; Quaternion orientation;};
struct MotionLimits {
  double linear_speed{}, angular_speed{}, linear_acceleration{}, angular_acceleration{};
};
inline constexpr int kTeleopSensitivityLevel = 4;
inline constexpr MotionLimits kTeleopMotionLimits{0.40, 2.00, 1.6, 8.0};

class PoseRateLimiter {
 public:
  void reset(const Pose & pose);
  Pose step(const Pose & target, double dt, const MotionLimits & limits);
  const Pose & pose() const {return pose_;}

 private:
  Pose pose_{};
  Vector3 linear_velocity_{}, angular_velocity_{};
  bool initialized_{false};
};

bool decode_raw_pose(const std::array<float, 7> & raw, const std::string & layout, Pose & pose);
bool valid_pose(const Pose & pose);
Quaternion normalized(const Quaternion & q);
Quaternion multiply(const Quaternion & a, const Quaternion & b);
Quaternion inverse(const Quaternion & q);
Vector3 rotate(const Quaternion & q, const Vector3 & vector);
double angular_distance(const Quaternion & a, const Quaternion & b);
Pose map_controller_delta(const Pose & xr_at_trigger, const Pose & xr_now,
  const Pose & tcp_at_trigger, const Quaternion & xr_to_root, double translation_scale);
}  // namespace wa1_pico4_teleop
