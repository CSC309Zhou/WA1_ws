"""WA1 operator-test joint order and conservative example limits.

These limits intersect the supplied URDF and teleoperation-guide examples;
they are not verified controller limits. Lifting_Z uses metres, others radians.
This module has no ROS dependency and is not loaded by deployed robot nodes.
"""

PARTS = {
    'left_arm': (('Shoulder_Y_L', 'Shoulder_X_L', 'Shoulder_Z_L', 'Elbow_L',
                  'Wrist_Z_L', 'Wrist_Y_L', 'Wrist_X_L'), 1),
    'right_arm': (('Shoulder_Y_R', 'Shoulder_X_R', 'Shoulder_Z_R', 'Elbow_R',
                   'Wrist_Z_R', 'Wrist_Y_R', 'Wrist_X_R'), 2),
    'neck': (('Neck_Z', 'Neck_Y'), 4),
    'waist': (('Waist_Z', 'Waist_Y'), 8),
    'lifting': (('Lifting_Z',), 16),
}

TARGET_LIMITS = {
    'Shoulder_Y_L': (-3.197, 1.145),
    'Shoulder_X_L': (0.0, 3.026),
    'Shoulder_Z_L': (-2.83, 2.83),
    'Elbow_L': (-2.0, 0.07),
    'Wrist_Z_L': (-2.855, 2.855),
    'Wrist_Y_L': (-1.487, 1.2566),
    'Wrist_X_L': (-1.487, 1.487),
    'Shoulder_Y_R': (-3.197, 1.145),
    'Shoulder_X_R': (-3.026, 0.0),
    'Shoulder_Z_R': (-2.83, 2.83),
    'Elbow_R': (-2.0, 0.07),
    'Wrist_Z_R': (-2.855, 2.855),
    'Wrist_Y_R': (-1.487, 1.2566),
    'Wrist_X_R': (-1.487, 1.487),
    'Neck_Z': (-0.733, 0.733),
    'Neck_Y': (-0.1693, 0.6),
    'Waist_Z': (-2.018, 3.0892),
    'Waist_Y': (-0.478, 0.733),
    'Lifting_Z': (0.002, 0.206),
}
