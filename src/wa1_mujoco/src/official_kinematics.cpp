#include <pico_runtime/uplimb/UplimbController.h>

#include <Eigen/Geometry>
#include <cmath>
#include <cstring>
#include <exception>
#include <mutex>
#include <string>
#include <vector>

namespace {
std::mutex library_mutex;
bool initialized = false;
thread_local std::string last_error;
using Pose6 = Eigen::Matrix<double, 6, 1>;

bool valid_pose(const double* pose) {
  if (!pose) return false;
  for (int i = 0; i < 7; ++i) {
    if (!std::isfinite(pose[i])) return false;
  }
  return true;
}

Pose6 to_vendor_pose(const double* pose) {
  // Public API: position (m) followed by quaternion (x,y,z,w).
  // Official library: position followed by yaw,pitch,roll (ZYX).
  Eigen::Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
  q.normalize();
  const Eigen::Vector3d ypr = q.toRotationMatrix().eulerAngles(2, 1, 0);
  Pose6 result;
  result << pose[0], pose[1], pose[2], ypr[0], ypr[1], ypr[2];
  return result;
}

void from_vendor_pose(const Pose6& pose, double* output) {
  const Eigen::Quaterniond q =
      Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitX());
  output[0] = pose[0]; output[1] = pose[1]; output[2] = pose[2];
  output[3] = q.x(); output[4] = q.y(); output[5] = q.z(); output[6] = q.w();
}

bool valid_side(int side) { return side == 1 || side == 2; }
}  // namespace

extern "C" {

const char* wa1_last_error() { return last_error.c_str(); }

int wa1_init(const char* config_path) {
  std::lock_guard<std::mutex> guard(library_mutex);
  if (initialized) return 0;
  if (!config_path || !*config_path) {
    last_error = "configuration path is empty";
    return -1;
  }
  try {
    init(config_path);
    initialized = true;
    last_error.clear();
    return 0;
  } catch (const std::exception& error) {
    last_error = error.what();
    return -1;
  }
}

// Vendor joint order: L7, R7, Neck_Z, Neck_Y, Lifting_Z, Waist_Z, Waist_Y.
int wa1_fk(const double* q19, int side, double* pose7) {
  if (!initialized || !q19 || !pose7 || !valid_side(side)) {
    last_error = "FK requires init, 19 joints, an output, and side 1 or 2";
    return -1;
  }
  for (int i = 0; i < 19; ++i) {
    if (!std::isfinite(q19[i])) {
      last_error = "FK joint is not finite";
      return -1;
    }
  }
  std::lock_guard<std::mutex> guard(library_mutex);
  try {
    const Eigen::Map<const Eigen::VectorXd> q(q19, 19);
    const auto poses = getForwardKinematics(q, side);
    if (poses.empty()) {
      last_error = "official FK returned no pose";
      return -1;
    }
    from_vendor_pose(poses.front(), pose7);
    last_error.clear();
    return 0;
  } catch (const std::exception& error) {
    last_error = error.what();
    return -1;
  }
}

// Target pose is expressed in BASE, normalized to a zero lift/waist state.
// Solutions are contiguous groups of 7 angles; phi has one value per group.
int wa1_ik_zero_torso(const double* pose7, int side, double q7,
                      double* solutions, double* phi, int capacity) {
  if (!initialized || !valid_side(side) || !valid_pose(pose7) ||
      !std::isfinite(q7) || !solutions || !phi || capacity < 1) {
    last_error = "invalid IK input";
    return -1;
  }
  const double qnorm = std::sqrt(pose7[3]*pose7[3] + pose7[4]*pose7[4] +
                                 pose7[5]*pose7[5] + pose7[6]*pose7[6]);
  if (qnorm < 1e-8) {
    last_error = "IK quaternion has zero norm";
    return -1;
  }
  std::lock_guard<std::mutex> guard(library_mutex);
  try {
    std::vector<std::vector<Eigen::VectorXd>> groups;
    std::vector<std::vector<double>> arm_angles;
    getInverseKinematics(to_vendor_pose(pose7), q7, groups, arm_angles, side);
    int count = 0;
    for (size_t arm = 0; arm < groups.size(); ++arm) {
      for (size_t i = 0; i < groups[arm].size(); ++i) {
        if (count >= capacity || groups[arm][i].size() != 7 ||
            arm >= arm_angles.size() || i >= arm_angles[arm].size()) {
          last_error = "IK output buffer too small or inconsistent";
          return -1;
        }
        std::memcpy(solutions + count * 7, groups[arm][i].data(), 7*sizeof(double));
        phi[count] = arm_angles[arm][i];
        ++count;
      }
    }
    last_error.clear();
    return count;
  } catch (const std::exception& error) {
    last_error = error.what();
    return -1;
  }
}

}  // extern "C"
