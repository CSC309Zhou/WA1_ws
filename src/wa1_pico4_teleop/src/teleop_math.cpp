#include "wa1_pico4_teleop/teleop_math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wa1_pico4_teleop {
namespace {
double norm_sq(const Quaternion & q) {return q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;}
double norm(const Vector3 & v) {return std::hypot(std::hypot(v.x, v.y), v.z);}
Vector3 difference(const Vector3 & a, const Vector3 & b) {
  return {a.x-b.x, a.y-b.y, a.z-b.z};
}
Vector3 add(const Vector3 & a, const Vector3 & b) {
  return {a.x+b.x, a.y+b.y, a.z+b.z};
}
Vector3 scaled(const Vector3 & v, double factor) {
  return {v.x*factor, v.y*factor, v.z*factor};
}
Vector3 approach_velocity(const Vector3 & current, const Vector3 & desired,
  double max_speed, double max_acceleration, double dt) {
  const Vector3 delta = difference(desired, current);
  const double delta_norm = norm(delta);
  const double allowed_change = max_acceleration*dt;
  const Vector3 next = add(current, delta_norm > allowed_change ?
    scaled(delta, allowed_change/delta_norm) : delta);
  const double speed = norm(next);
  return speed > max_speed ? scaled(next, max_speed/speed) : next;
}
}  // namespace

void PoseRateLimiter::reset(const Pose & pose) {
  if (!valid_pose(pose)) {throw std::invalid_argument("Invalid limiter reset pose");}
  pose_ = pose;
  pose_.orientation = normalized(pose_.orientation);
  linear_velocity_ = {};
  angular_velocity_ = {};
  initialized_ = true;
}

Pose PoseRateLimiter::step(const Pose & target, double dt, const MotionLimits & limits) {
  if (!initialized_ || !valid_pose(target) || !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(limits.linear_speed) || limits.linear_speed <= 0.0 ||
      !std::isfinite(limits.angular_speed) || limits.angular_speed <= 0.0 ||
      !std::isfinite(limits.linear_acceleration) || limits.linear_acceleration <= 0.0 ||
      !std::isfinite(limits.angular_acceleration) || limits.angular_acceleration <= 0.0) {
    throw std::invalid_argument("Invalid limiter state, target, timestep or limits");
  }
  const Vector3 linear_error = difference(target.position, pose_.position);
  const double distance = norm(linear_error);
  const Vector3 direct_linear_velocity = scaled(linear_error, 1.0/dt);
  const double linear_accel_step = limits.linear_acceleration*dt;
  // Settle exactly only if the final step and the following stop both obey
  // the acceleration limit. A suddenly moved target may otherwise be crossed
  // briefly while the commanded velocity decelerates.
  if (norm(direct_linear_velocity) <= std::min(limits.linear_speed, linear_accel_step) &&
      norm(difference(direct_linear_velocity, linear_velocity_)) <= linear_accel_step) {
    pose_.position = target.position;
    linear_velocity_ = direct_linear_velocity;
  } else {
    const double wanted_speed = distance > 1e-12 ? std::min(limits.linear_speed,
      std::sqrt(2.0*limits.linear_acceleration*distance)) : 0.0;
    const Vector3 wanted = distance > 1e-12 ?
      scaled(linear_error, wanted_speed/distance) : Vector3{};
    linear_velocity_ = approach_velocity(linear_velocity_, wanted,
      limits.linear_speed, limits.linear_acceleration, dt);
    pose_.position = add(pose_.position, scaled(linear_velocity_, dt));
  }

  Quaternion error = multiply(normalized(target.orientation), inverse(pose_.orientation));
  if (error.w < 0.0) {
    error = {-error.x, -error.y, -error.z, -error.w};
  }
  const Vector3 error_vector{error.x, error.y, error.z};
  const double vector_norm = norm(error_vector);
  const double angle = 2.0*std::atan2(vector_norm, std::max(0.0, error.w));
  const Vector3 axis = vector_norm > 1e-12 ?
    scaled(error_vector, 1.0/vector_norm) : Vector3{};
  const Vector3 direct_angular_velocity = scaled(axis, angle/dt);
  const double angular_accel_step = limits.angular_acceleration*dt;
  if (norm(direct_angular_velocity) <= std::min(limits.angular_speed, angular_accel_step) &&
      norm(difference(direct_angular_velocity, angular_velocity_)) <= angular_accel_step) {
    pose_.orientation = normalized(target.orientation);
    angular_velocity_ = direct_angular_velocity;
  } else {
    const double wanted_speed = std::min(limits.angular_speed,
      std::sqrt(2.0*limits.angular_acceleration*angle));
    angular_velocity_ = approach_velocity(angular_velocity_, scaled(axis, wanted_speed),
      limits.angular_speed, limits.angular_acceleration, dt);
    const double step_angle = norm(angular_velocity_)*dt;
    if (step_angle > 0.0) {
      const Vector3 step_axis = scaled(angular_velocity_, 1.0/norm(angular_velocity_));
      const double sine = std::sin(step_angle/2.0);
      const Quaternion step_q{step_axis.x*sine, step_axis.y*sine,
        step_axis.z*sine, std::cos(step_angle/2.0)};
      pose_.orientation = normalized(multiply(step_q, pose_.orientation));
    }
  }
  return pose_;
}

bool valid_pose(const Pose & p) {
  const double n = norm_sq(p.orientation);
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
    std::isfinite(p.position.z) && std::isfinite(n) && n > 0.5 && n < 1.5;
}

Quaternion normalized(const Quaternion & q) {
  const double n = norm_sq(q);
  if (!std::isfinite(n) || n < 1e-12) {throw std::invalid_argument("Invalid quaternion");}
  const double k = 1.0/std::sqrt(n);
  return {q.x*k, q.y*k, q.z*k, q.w*k};
}

Quaternion multiply(const Quaternion & a, const Quaternion & b) {
  return {
    a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
    a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
    a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
  };
}

Quaternion inverse(const Quaternion & q) {
  const double n = norm_sq(q);
  if (!std::isfinite(n) || n < 1e-12) {throw std::invalid_argument("Invalid quaternion");}
  return {-q.x/n, -q.y/n, -q.z/n, q.w/n};
}

Vector3 rotate(const Quaternion & q, const Vector3 & v) {
  const auto unit = normalized(q);
  const auto out = multiply(multiply(unit, {v.x, v.y, v.z, 0.0}), inverse(unit));
  return {out.x, out.y, out.z};
}

double angular_distance(const Quaternion & a, const Quaternion & b) {
  const auto delta = normalized(multiply(normalized(a), inverse(normalized(b))));
  return 2.0 * std::acos(std::clamp(std::abs(delta.w), 0.0, 1.0));
}

bool decode_raw_pose(const std::array<float, 7> & raw, const std::string & layout, Pose & pose) {
  for (float value : raw) {if (!std::isfinite(value)) {return false;}}
  if (layout == "xyz_xyzw") {
    pose = {{raw[0], raw[1], raw[2]}, {raw[3], raw[4], raw[5], raw[6]}};
  } else if (layout == "xyz_wxyz") {
    pose = {{raw[0], raw[1], raw[2]}, {raw[4], raw[5], raw[6], raw[3]}};
  } else if (layout == "xyzw_xyz") {
    pose = {{raw[4], raw[5], raw[6]}, {raw[0], raw[1], raw[2], raw[3]}};
  } else {return false;}
  if (!valid_pose(pose)) {return false;}
  pose.orientation = normalized(pose.orientation);
  return true;
}

Pose map_controller_delta(const Pose & xr_ref, const Pose & xr_now,
  const Pose & tcp_ref, const Quaternion & xr_to_root, double scale) {
  if (!valid_pose(xr_ref) || !valid_pose(xr_now) || !valid_pose(tcp_ref) ||
      !std::isfinite(scale) || scale <= 0.0) {
    throw std::invalid_argument("Invalid pose or scale");
  }
  const Quaternion q = normalized(xr_to_root);
  // Match the official getCtrlPoseRoot + processSide sequence: convert both
  // controller poses with the fixed R, then subtract the grip-time reference.
  const Vector3 root_ref = rotate(q, xr_ref.position);
  const Vector3 root_now = rotate(q, xr_now.position);
  const Quaternion root_ref_q = normalized(multiply(multiply(q,
    xr_ref.orientation), inverse(q)));
  const Quaternion root_now_q = normalized(multiply(multiply(q,
    xr_now.orientation), inverse(q)));
  const Vector3 root_delta{root_now.x - root_ref.x,
    root_now.y - root_ref.y, root_now.z - root_ref.z};
  const Quaternion delta_root = multiply(root_now_q, inverse(root_ref_q));
  Pose target;
  target.position = {tcp_ref.position.x + scale*root_delta.x,
    tcp_ref.position.y + scale*root_delta.y,
    tcp_ref.position.z + scale*root_delta.z};
  target.orientation = normalized(multiply(delta_root, tcp_ref.orientation));
  return target;
}
}  // namespace wa1_pico4_teleop
