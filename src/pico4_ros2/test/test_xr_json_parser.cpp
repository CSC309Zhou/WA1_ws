#include <gtest/gtest.h>
#include "pico4_ros2/xr_json_parser.hpp"

TEST(XrJsonParser, ParsesVendorFieldNamesAndKeepsRawPoseOrder) {
  const std::string sample = R"({"timeStampNs":42,"Input":3,"Battery":82,
    "Head":{"pose":[0,1,2,0,0,0,1],"status":1},
    "Controller":[
      {"axisX":0.2,"axisY":-0.1,"axisClick":true,"grip":0.4,"trigger":0.9,
       "primaryButton":false,"secondaryButton":true,"menuButton":false,
       "pose":[1,2,3,0,0,0,1],"status":2},
      {"axisX":0,"axisY":0,"grip":0,"trigger":0,
       "pose":[4,5,6,0,0,0,1],"status":3}]})";
  pico4_ros2::msg::XrFrame frame;
  std::string error;
  ASSERT_TRUE(pico4_ros2::parse_xr_json(sample, frame, error)) << error;
  EXPECT_EQ(frame.device_timestamp_ns, 42);
  EXPECT_EQ(frame.input, 3);
  EXPECT_FLOAT_EQ(frame.left_controller.tracked_pose.pose[0], 1.0f);
  EXPECT_FLOAT_EQ(frame.right_controller.tracked_pose.pose[0], 4.0f);
  EXPECT_FLOAT_EQ(frame.left_controller.gripper, 0.4f);
  EXPECT_TRUE(frame.left_controller.secondary_button);
}

TEST(XrJsonParser, RejectsBrokenPoseWithoutPublishingPartialFrame) {
  const std::string sample = R"({"Head":{"pose":[0,1]},"Controller":[]})";
  pico4_ros2::msg::XrFrame frame;
  std::string error;
  EXPECT_FALSE(pico4_ros2::parse_xr_json(sample, frame, error));
  EXPECT_FALSE(error.empty());
}

TEST(XrJsonParser, ParsesObservedSdkEnvelopeAndCommaSeparatedPose) {
  const std::string sample = R"XR({"functionName":"Tracking","value":"{\"timeStampNs\":123456789,\"Input\":1,\"Battery\":80,\"Head\":{\"pose\":\"0, 1, 2, 0, 0, 0, 1\",\"status\":3},\"Controller\":{\"left\":{\"pose\":\"-0.2,1.3,0.3,0,0,0,1\",\"trigger\":0.95},\"right\":{\"pose\":\"0.2,1.3,0.3,0,0,0,1\",\"grip\":0.4}}}"})XR";
  pico4_ros2::msg::XrFrame frame;
  std::string error;
  ASSERT_TRUE(pico4_ros2::parse_xr_json(sample, frame, error)) << error;
  EXPECT_EQ(frame.device_timestamp_ns, 123456789);
  EXPECT_EQ(frame.input, 1);
  EXPECT_EQ(frame.head.status, 3);
  EXPECT_FLOAT_EQ(frame.head.pose[1], 1.0f);
  EXPECT_FLOAT_EQ(frame.left_controller.tracked_pose.pose[0], -0.2f);
  EXPECT_FLOAT_EQ(frame.left_controller.trigger, 0.95f);
  EXPECT_FLOAT_EQ(frame.right_controller.gripper, 0.4f);
  EXPECT_EQ(frame.left_controller.tracked_pose.status, -1);
  EXPECT_EQ(frame.right_controller.tracked_pose.status, -1);
}

TEST(XrJsonParser, RejectsCsvPoseWithWrongDimension) {
  const std::string sample = R"({"Head":{"pose":"0,1,2,0,0,1"},"Controller":{}})";
  pico4_ros2::msg::XrFrame frame;
  std::string error;
  EXPECT_FALSE(pico4_ros2::parse_xr_json(sample, frame, error));
  EXPECT_EQ(error, "CSV pose must contain seven numbers");
}
