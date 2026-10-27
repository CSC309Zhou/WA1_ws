# Raw SDK pose array. Its component order and coordinate frame must be verified
# with a live headset before a robot-specific package uses it for control.
float32[7] pose
# -1 means the SDK frame did not contain a status for this pose.
int32 status
