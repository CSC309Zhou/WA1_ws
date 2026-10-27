#!/usr/bin/env python3
"""PC loopback-only ROS1 robot fixture; cannot connect to the real ROS master."""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client

assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11321'
assert os.environ.get('ROS_IP') == '127.0.0.1'
master = subprocess.Popen(['roscore', '-p', '11321'])
for _ in range(100):
    try:
        xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/wa1_mock')
        break
    except OSError:
        time.sleep(0.1)
else:
    master.terminate()
    raise RuntimeError('Private ROS Master did not start')

import rospy
import upperlimb.msg as msg
import upperlimb.srv as srv
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, SetBool

rospy.init_node('wa1_pc_mock')
events = Path('/evidence/mock_events.jsonl').open('w', buffering=1)
lock = threading.Lock()
def record(kind, **fields):
    with lock:
        events.write(json.dumps(dict(kind=kind, received_ns=time.time_ns(), received_steady_ns=time.monotonic_ns(), **fields)) + '\n')
def convert(value):
    if hasattr(value, '__slots__'):
        return {slot: convert(getattr(value, slot)) for slot in value.__slots__}
    if isinstance(value, (tuple, list)):
        return [convert(v) for v in value]
    return value

base = '/zj_humanoid/upperlimb/'
parts = [('left_arm', 7), ('right_arm', 7), ('dual_arm', 14), ('neck', 2), ('waist', 2), ('lifting', 1), ('whole_body', 19)]
subscribers = []
services = []
for part, _ in parts:
    for mode, cls in [('servoj', msg.Joints), ('speedj', msg.SpeedJ)]:
        if mode == 'speedj' and part == 'whole_body':
            continue
        def callback(m, endpoint=mode+'/'+part):
            record('command', endpoint=endpoint, data=convert(m))
        subscribers.append(rospy.Subscriber(base+mode+'/'+part, cls, callback, queue_size=1, tcp_nodelay=True))

def add_service(suffix, cls):
    def callback(req):
        record('service', endpoint=suffix, request=convert(req))
        if cls is srv.MoveJ and not req.is_async and req.t>0:
            time.sleep(min(req.t,1.0))
        response = cls._response_class()
        if hasattr(response, 'success'):
            response.success = True
        if hasattr(response, 'message'):
            response.message = 'mock:' + suffix
        if cls is srv.FK:
            response.pose.position.x = req.joints[0] if req.joints else 0.0
            response.pose.orientation.w = 1.0
        if cls is srv.IK:
            response.nums = 1
            response.joints = [msg.Joints(joint=[0.1]*7)]
            response.phi = [req.q7]
        return response
    services.append(rospy.Service(base+suffix, cls, callback))

for part, _ in parts:
    for mode, cls in [('movej',srv.MoveJ), ('movej_by_path',srv.MoveJByPath)]:
        add_service(mode+'/'+part, cls)
for side in ['left_arm', 'right_arm']:
    for mode, cls in [('FK',srv.FK), ('IK',srv.IK), ('is_singular',srv.IsSingular)]:
        add_service(mode+'/'+side, cls)
for name, cls in [('set_servo_params',srv.Servo),('clear_servo_params',srv.Servo),
                  ('enable_speedj',SetBool),('stop',Trigger),('unlock',Trigger)]:
    add_service(name, cls)
for part in ['left_arm','right_arm','dual_arm']:
    for mode, cls in [('movel',srv.MoveL),('movel_by_path',srv.MoveLByPath),('movej_by_pose',srv.MoveJByPose)]:
        add_service(mode+'/'+part, cls)
    for mode, cls in [('servol',msg.DualPose),('speedl',msg.SpeedL)]:
        subscribers.append(rospy.Subscriber(base+mode+'/'+part, cls,
            lambda m, endpoint=mode+'/'+part:record('command',endpoint=endpoint,data=convert(m)),
            queue_size=1,tcp_nodelay=True))
add_service('enable_speedl',SetBool)

names = ['Shoulder_Y_L','Shoulder_X_L','Shoulder_Z_L','Elbow_L','Wrist_Z_L','Wrist_Y_L','Wrist_X_L',
         'Shoulder_Y_R','Shoulder_X_R','Shoulder_Z_R','Elbow_R','Wrist_Z_R','Wrist_Y_R','Wrist_X_R',
         'Neck_Z','Neck_Y','Waist_Z','Waist_Y','Lifting_Z']
joint_pub=rospy.Publisher(base+'joint_states',JointState,queue_size=1)
state_pub=rospy.Publisher(base+'uplimb_state',msg.UplimbState,queue_size=1)
occupancy_pub=rospy.Publisher(base+'occupancy_state',Int8,queue_size=1)
pose_left=rospy.Publisher(base+'tcp_pose/left_arm',msg.Pose,queue_size=1)
pose_right=rospy.Publisher(base+'tcp_pose/right_arm',msg.Pose,queue_size=1)
speed_pub=rospy.Publisher(base+'tcp_speed/dual_arm',msg.TcpSpeed,queue_size=1)
# Foreign topics exist only in this PC fixture, to prove the bridge excludes them.
foreign_image=rospy.Publisher('/wa1_mock/camera/image',Image,queue_size=1)
foreign_cmd=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
seq=0
def publish(_):
    global seq
    seq+=1
    state=JointState()
    state.header.seq=seq
    state.header.stamp=rospy.Time.now()
    state.name=names
    state.position=[0.0]*19
    state.velocity=[0.0]*19
    state.effort=[0.0]*19
    joint_pub.publish(state)
    occupancy_pub.publish(Int8(data=0))
    status=msg.UplimbState();status.header=state.header;status.cmd_name='STOPPED'
    state_pub.publish(status)
    pose=msg.Pose();pose.header=state.header;pose.quaternion.w=1.0
    pose_left.publish(pose);pose_right.publish(pose)
    speed=msg.TcpSpeed();speed.header=state.header;speed_pub.publish(speed)
    if seq%20==0:
        foreign_image.publish(Image());foreign_cmd.publish(Twist())
timer=rospy.Timer(rospy.Duration(0.005),publish)
record('ready')
try:
    rospy.spin()
finally:
    master.send_signal(signal.SIGINT)
    try: master.wait(timeout=5)
    except subprocess.TimeoutExpired: master.kill()
    events.close()
