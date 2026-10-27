#include <cmath>
#include <stdexcept>
#include <gtest/gtest.h>
#include "wa1_pico4_teleop/teleop_math.hpp"

using namespace wa1_pico4_teleop;

TEST(TeleopMath, CapturedStartDoesNotJumpAndTranslationRotatesIntoBase) {
  const Pose xr_ref{{1.0, 2.0, 3.0}, {0, 0, 0, 1}};
  const Pose tcp_ref{{0.38, 0.27, 0.55}, {0, 0, 0, 1}};
  const Quaternion xr_to_base{0, 0, std::sqrt(0.5), std::sqrt(0.5)};
  const auto start = map_controller_delta(xr_ref, xr_ref, tcp_ref, xr_to_base, 1.0);
  EXPECT_NEAR(start.position.x, tcp_ref.position.x, 1e-12);
  EXPECT_NEAR(start.position.y, tcp_ref.position.y, 1e-12);
  EXPECT_NEAR(angular_distance(start.orientation, tcp_ref.orientation), 0.0, 1e-12);
  Pose xr_now = xr_ref;
  xr_now.position.x += 0.01;
  const auto target = map_controller_delta(xr_ref, xr_now, tcp_ref, xr_to_base, 1.0);
  EXPECT_NEAR(target.position.x, tcp_ref.position.x, 1e-12);
  EXPECT_NEAR(target.position.y, tcp_ref.position.y + 0.01, 1e-12);
}

TEST(TeleopMath, OrientationDeltaUsesWorldFrame) {
  const Pose ref{{0, 0, 0}, {0, 0, 0, 1}};
  Pose now = ref;
  now.orientation = {0, 0, std::sin(0.1), std::cos(0.1)};
  const Pose tcp{{0, 0, 0}, {0, 0, 0, 1}};
  const auto target = map_controller_delta(ref, now, tcp, {0, 0, 0, 1}, 1.0);
  EXPECT_NEAR(angular_distance(target.orientation, tcp.orientation), 0.2, 1e-12);
}

TEST(TeleopMath, OfficialFixedAxisMap) {
  const Quaternion fixed{0.5, -0.5, -0.5, 0.5};
  // Rows of the official R_XR_TO_ROOT matrix: [0,0,-1], [-1,0,0], [0,1,0].
  const auto x = rotate(fixed, {1.0, 0.0, 0.0});
  const auto y = rotate(fixed, {0.0, 1.0, 0.0});
  const auto z = rotate(fixed, {0.0, 0.0, 1.0});
  EXPECT_NEAR(x.x, 0.0, 1e-12);
  EXPECT_NEAR(x.y, -1.0, 1e-12);
  EXPECT_NEAR(x.z, 0.0, 1e-12);
  EXPECT_NEAR(y.x, 0.0, 1e-12);
  EXPECT_NEAR(y.y, 0.0, 1e-12);
  EXPECT_NEAR(y.z, 1.0, 1e-12);
  EXPECT_NEAR(z.x, -1.0, 1e-12);
  EXPECT_NEAR(z.y, 0.0, 1e-12);
  EXPECT_NEAR(z.z, 0.0, 1e-12);

  const Pose xr_ref{{0, 0, 0}, {0, 0, 0, 1}};
  const Pose tcp_ref{{0.38, 0.27, 0.55}, {0, 0, 0, 1}};
  Pose xr_now = xr_ref;
  xr_now.position.z = -0.02;
  const auto target = map_controller_delta(xr_ref, xr_now, tcp_ref, fixed, 1.0);
  EXPECT_NEAR(target.position.x, 0.40, 1e-12);
  EXPECT_NEAR(target.position.y, 0.27, 1e-12);
  EXPECT_NEAR(target.position.z, 0.55, 1e-12);
}

TEST(TeleopMath, TranslationScaleDoesNotChangeRotation) {
  const Pose ref{{0, 0, 0}, {0, 0, 0, 1}};
  Pose now = ref;
  now.position.x = 0.1;
  now.orientation = {0, 0, std::sin(0.2), std::cos(0.2)};
  const Pose tcp{{0.38, 0.27, 0.55}, {0, 0, 0, 1}};
  const auto half = map_controller_delta(ref, now, tcp, {0, 0, 0, 1}, 0.5);
  const auto one_and_half = map_controller_delta(ref, now, tcp, {0, 0, 0, 1}, 1.5);
  EXPECT_NEAR(half.position.x, 0.43, 1e-12);
  EXPECT_NEAR(one_and_half.position.x, 0.53, 1e-12);
  EXPECT_NEAR(angular_distance(half.orientation, tcp.orientation), 0.4, 1e-12);
  EXPECT_NEAR(angular_distance(one_and_half.orientation, tcp.orientation), 0.4, 1e-12);
}

TEST(TeleopMath, FixedTrackingLimitsMatchValidatedLevelFour) {
  EXPECT_EQ(kTeleopSensitivityLevel, 4);
  EXPECT_DOUBLE_EQ(kTeleopMotionLimits.linear_speed, 0.40);
  EXPECT_DOUBLE_EQ(kTeleopMotionLimits.angular_speed, 2.00);
  EXPECT_DOUBLE_EQ(kTeleopMotionLimits.linear_acceleration, 1.6);
  EXPECT_DOUBLE_EQ(kTeleopMotionLimits.angular_acceleration, 8.0);
}

TEST(TeleopMath, RateLimiterRespectsSpeedAndAcceleration) {
  const auto limits = kTeleopMotionLimits;
  const Pose start{{0, 0, 0}, {0, 0, 0, 1}};
  const Pose target{{2, 0, 0}, {0, 0, std::sin(1.0), std::cos(1.0)}};
  PoseRateLimiter limiter;
  limiter.reset(start);
  Pose previous = start;
  double previous_linear_speed = 0.0;
  double previous_angular_speed = 0.0;
  constexpr double dt = 0.01;
  for (int i = 0; i < 100; ++i) {
    const Pose command = limiter.step(target, dt, limits);
    const double linear_speed = (command.position.x-previous.position.x)/dt;
    const double angular_speed = angular_distance(command.orientation, previous.orientation)/dt;
    EXPECT_GE(linear_speed, 0.0);
    EXPECT_LE(linear_speed, limits.linear_speed+1e-9);
    EXPECT_LE(angular_speed, limits.angular_speed+1e-9);
    EXPECT_LE(linear_speed-previous_linear_speed, limits.linear_acceleration*dt+1e-9);
    EXPECT_LE(angular_speed-previous_angular_speed, limits.angular_acceleration*dt+1e-9);
    previous = command;
    previous_linear_speed = linear_speed;
    previous_angular_speed = angular_speed;
  }
  EXPECT_LT(previous.position.x, target.position.x);
  EXPECT_GT(previous.position.x, 0.0);
}

TEST(TeleopMath, RateLimiterConvergesAndResetDropsOutstandingMotion) {
  const auto limits = kTeleopMotionLimits;
  const Pose start{{0.38, 0.27, 0.55}, {0, 0, 0, 1}};
  const Pose target{{0.48, 0.27, 0.55}, {0, 0, std::sin(0.2), std::cos(0.2)}};
  PoseRateLimiter limiter;
  limiter.reset(start);
  for (int i = 0; i < 300; ++i) {
    const Pose command = limiter.step(target, 0.01, limits);
    EXPECT_LE(command.position.x, target.position.x+0.002);
  }
  EXPECT_NEAR(limiter.pose().position.x, target.position.x, 1e-8);
  EXPECT_NEAR(angular_distance(limiter.pose().orientation, target.orientation), 0.0, 1e-8);

  limiter.reset(start);  // Released grip: discard the old target.
  const Pose held = limiter.step(start, 0.01, limits);
  EXPECT_NEAR(held.position.x, start.position.x, 1e-12);
  EXPECT_NEAR(angular_distance(held.orientation, start.orientation), 0.0, 1e-12);
}

TEST(TeleopMath, RateLimiterKeepsAccelerationBoundedThroughStopAndReversal) {
  const auto limits = kTeleopMotionLimits;
  const Pose start{{0, 0, 0}, {0, 0, 0, 1}};
  const Pose forward{{0.1, 0, 0}, {0, 0, std::sin(0.2), std::cos(0.2)}};
  const Pose reverse{{-0.1, 0, 0}, {0, 0, std::sin(-0.2), std::cos(-0.2)}};
  PoseRateLimiter limiter;
  limiter.reset(start);
  Pose previous = start;
  double previous_linear_velocity = 0.0;
  double previous_angular_velocity = 0.0;
  constexpr double dt = 0.01;
  for (int i = 0; i < 600; ++i) {
    const Pose command = limiter.step(i < 180 ? forward : reverse, dt, limits);
    const double linear_velocity = (command.position.x-previous.position.x)/dt;
    const auto yaw = [](const Quaternion & q) {return 2.0*std::atan2(q.z, q.w);};
    const double angular_velocity =
      (yaw(command.orientation)-yaw(previous.orientation))/dt;
    EXPECT_LE(std::abs(linear_velocity), limits.linear_speed+1e-8);
    EXPECT_LE(std::abs(angular_velocity), limits.angular_speed+1e-8);
    EXPECT_LE(std::abs(linear_velocity-previous_linear_velocity)/dt,
      limits.linear_acceleration+1e-8);
    EXPECT_LE(std::abs(angular_velocity-previous_angular_velocity)/dt,
      limits.angular_acceleration+1e-8);
    previous = command;
    previous_linear_velocity = linear_velocity;
    previous_angular_velocity = angular_velocity;
  }
  EXPECT_NEAR(limiter.pose().position.x, reverse.position.x, 1e-8);
  EXPECT_NEAR(angular_distance(limiter.pose().orientation, reverse.orientation), 0.0, 1e-8);
}
