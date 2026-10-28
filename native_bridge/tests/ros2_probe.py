#!/usr/bin/env python3
"""ROS2 PC client for private mock regression or a zero-command real handshake."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import struct
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, TimeReference
from std_msgs.msg import String
from std_srvs.srv import Trigger
from upperlimb.msg import Joints, SpeedJ, Pose, UplimbState, TcpSpeed
from upperlimb.srv import Servo, MoveJ, FK, MoveJByPath, MoveJByPose, MoveL

def distribution(values):
    if not values:return {'n':0}
    ordered=sorted(values)
    return dict(n=len(values),median=statistics.median(values),p95=ordered[int((len(ordered)-1)*.95)],
                p99=ordered[int((len(ordered)-1)*.99)],max=max(values),min=min(values))

def main():
    parser=argparse.ArgumentParser()
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--mock-control',action='store_true')
    parser.add_argument('--latency-probe',action='store_true',
                        help='Check optional PC-only bridge ServoJ command timing in mock mode')
    mode.add_argument('--handshake-only',action='store_true',help='Real ROS1 subscriber handshakes, no commands or service calls')
    parser.add_argument('--seconds',type=float,default=10)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--events',type=Path)
    args=parser.parse_args()
    assert not args.latency_probe or args.mock_control
    if args.mock_control:
        assert os.environ.get('ROS_DOMAIN_ID')=='83', 'Commands only allowed on private mock DDS domain 83'
        assert args.events and args.events.is_file()
    rclpy.init()
    node=rclpy.create_node('wa1_pc_ros2_probe')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    samples=[];timing={};seen={};statuses=[];extra={};entities=[];command_timing=[]
    def joint(m):
        now=time.time_ns();key=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        samples.append(dict(stamp_ns=key,received_ns=now,steady_ns=time.monotonic_ns(),names=list(m.name),
                            position=list(m.position),velocity_size=len(m.velocity),effort_size=len(m.effort)))
        seen[key]=samples[-1]['steady_ns']
    def trace(m):timing[m.header.stamp.sec*10**9+m.header.stamp.nanosec]=m.time_ref.sec*10**9+m.time_ref.nanosec
    entities.append(node.create_subscription(JointState,'/upperlimb/joint_states',joint,qos))
    entities.append(node.create_subscription(TimeReference,'/upperlimb/bridge/joint_receive_time',trace,qos))
    if args.latency_probe:
        entities.append(node.create_subscription(
            String, '/upperlimb/bridge/servoj_left_arm_timing',
            lambda m:command_timing.append(json.loads(m.data)),
            QoSProfile(depth=512,reliability=ReliabilityPolicy.BEST_EFFORT)))
    status_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    entities.append(node.create_subscription(String,'/upperlimb/bridge/status',lambda m:statuses.append(json.loads(m.data)),status_qos))
    for suffix,cls in [('tcp_pose/left',Pose),('tcp_pose/right',Pose),('uplimb_state',UplimbState),('tcp_speed/dual',TcpSpeed)]:
        entities.append(node.create_subscription(cls,'/upperlimb/'+suffix,lambda m,s=suffix:extra.__setitem__(s,extra.get(s,0)+1),qos))
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=min(.01,max(0,until-time.monotonic())))
    def call(cls,name,request):
        client=node.create_client(cls,name);entities.append(client)
        assert client.wait_for_service(timeout_sec=8),name+' unavailable'
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=8)
        assert future.done() and future.result() is not None,name+' timed out'
        return future.result()
    def events():
        return [json.loads(s) for s in args.events.read_text().splitlines() if s.strip()]
    spin(3)
    report={'mode':'private_mock_control' if args.mock_control else 'real_robot_handshake_no_commands',
            'dds_domain':os.environ.get('ROS_DOMAIN_ID','0'),'jetson_modified':False,'motion_on_real_robot':False}
    if args.mock_control:
        assert statuses and statuses[-1]['feedback_ready'] and not statuses[-1]['read_only'],statuses[-1:]
        assert statuses[-1]['ros1_master_uri'].rstrip('/')=='http://127.0.0.1:11321',statuses[-1]
        response=call(Trigger,'/upperlimb/safety_lock',Trigger.Request())
        assert response.success is False and 'not in safety lock' in response.message,response
        response=call(FK,'/upperlimb/FK/left_arm',FK.Request(joints=[.123]+[0.0]*6))
        assert response.success and abs(response.pose.position.x-.123)<1e-12
        response=call(Servo,'/upperlimb/set_servo_params',Servo.Request(v=.1,acc=.2,time=.01,lookahead_time=.02,gain=100,arm_type=31))
        assert response.success
        response=call(MoveJ,'/upperlimb/movej/lifting',MoveJ.Request(joints=[.01],v=.001,acc=.001,t=1.,is_async=True,arm_type=16))
        assert response.success
        response=call(MoveJByPath,'/upperlimb/movej_by_path/neck',MoveJByPath.Request(
            path=[Joints(joint=[.01,.02]),Joints(joint=[.02,.03])],time=1.,timestamp=[0.,1.],is_async=True,arm_type=4))
        assert response.success
        # Fixed-size message arrays in services exercise the upstream generator fix.
        request=MoveJByPose.Request();request.pose[0].position.x=.11;request.pose[0].orientation.w=1.
        request.pose[1].position.x=.22;request.pose[1].orientation.w=1.;request.q7=[.3,.4];request.v=.1;request.acc=.2
        assert call(MoveJByPose,'/upperlimb/movej_by_pose/dual_arm',request).success
        request=MoveL.Request();request.pose[0].position.x=.33;request.pose[0].orientation.w=1.
        request.pose[1].position.x=.44;request.pose[1].orientation.w=1.;request.v=.1;request.acc=.2
        assert call(MoveL,'/upperlimb/movel/dual_arm',request).success
        parts=[('left_arm',7,1),('right_arm',7,2),('dual_arm',14,3),('neck',2,4),('waist',2,8),('lifting',1,16),('whole_body',19,31)]
        pubs={p:node.create_publisher(Joints,'/cmd/upperlimb/servoj/'+p,1) for p,_,_ in parts}
        speedpubs={p:node.create_publisher(SpeedJ,'/cmd/upperlimb/speedj/'+p,1) for p,_,_ in parts if p!='whole_body'}
        slow_client=node.create_client(MoveJ,'/upperlimb/movej/neck')
        assert slow_client.wait_for_service(timeout_sec=5)
        entities.append(slow_client)
        spin(2)
        assert all(node.count_subscribers('/cmd/upperlimb/servoj/'+p)==1 for p,_,_ in parts), 'Unexpected duplicate/missing control receiver'
        for side in ('left_arm','right_arm','dual_arm'):
            for mode in ('servol','speedl'):
                topic='/cmd/upperlimb/'+mode+'/'+side
                assert node.count_subscribers(topic)==1, 'Missing Cartesian command receiver: '+topic
        service_names={name for name,_ in node.get_service_names_and_types()}
        assert '/upperlimb/enable_speedl' in service_names, 'Missing Cartesian mode service'
        for side in ('left_arm','right_arm','dual_arm'):
            for mode in ('movel','movel_by_path','movej_by_pose'):
                service='/upperlimb/'+mode+'/'+side
                assert service in service_names, 'Missing Cartesian service: '+service
        before=len([e for e in events() if e['kind']=='command'])
        for _ in range(6):pubs['left_arm'].publish(Joints(joint=[0.]*6));spin(.01)
        for _ in range(6):pubs['left_arm'].publish(Joints(joint=[float('nan')]*7));spin(.01)
        assert len([e for e in events() if e['kind']=='command'])==before,'Invalid commands reached ROS1'
        published={}
        for p,size,mask in parts:
            for index in range(5):
                value=.01+mask*.001+index*.000001
                published[(p,round(value,9))]=time.monotonic_ns()
                pubs[p].publish(Joints(joint=[value]*size));spin(.01)
        for p,size,mask in parts:
            if p=='whole_body':continue
            speedpubs[p].publish(SpeedJ(joint_speed=[.001]*size,acc=.1,arm_type=mask));spin(.01)
        # 200Hz PC ROS2 -> C++ bridge -> PC ROS1 receiver, same host clock.
        start=time.monotonic();deadline=start
        left_stream_bits=set()
        seq=0
        while time.monotonic()-start<args.seconds:
            value=.1+seq*.000001
            published[('dual_arm',round(value,9))]=time.monotonic_ns()
            pubs['dual_arm'].publish(Joints(joint=[value]*14));seq+=1
            if args.latency_probe:
                left_stream_bits.add(f'{struct.unpack("=Q",struct.pack("=d",value))[0]:016x}')
                pubs['left_arm'].publish(Joints(joint=[value]*7))
            deadline+=.005
            spin(max(0,deadline-time.monotonic()))
        if args.latency_probe:
            observed_bits={row['target_bits'] for row in command_timing}
            matched=len(left_stream_bits & observed_bits)
            assert matched >= .9*len(left_stream_bits),(matched,len(left_stream_bits))
            report['left_arm_stream_timing_match']={'matched':matched,
                                                    'published_unique':len(left_stream_bits)}
        end=time.monotonic_ns()
        slow_future=slow_client.call_async(MoveJ.Request(joints=[0.,0.],v=.01,acc=.01,t=.8,is_async=False,arm_type=4))
        spin(.4)
        data=events();commands=[e for e in data if e['kind']=='command']
        for p,_,_ in parts:assert any(e['endpoint']=='servoj/'+p for e in commands),p
        if args.latency_probe:
            assert command_timing, 'No left-arm bridge timing messages'
            measured=command_timing[0]
            for key in ('callback_entry_ns','ros1_publish_start_ns','ros1_publish_return_ns'):
                assert isinstance(measured[key],int) and measured[key]>0,key
            assert (measured['callback_entry_ns'] <= measured['ros1_publish_start_ns'] <=
                    measured['ros1_publish_return_ns']),measured
            expected_bits=f'{struct.unpack("=Q",struct.pack("=d",.011))[0]:016x}'
            assert any(row['target_bits']==expected_bits for row in command_timing),command_timing[:2]
            report['left_arm_timing_samples']=len(command_timing)
        for p,size,mask in parts:
            if p=='whole_body':continue
            received=next(e for e in commands if e['endpoint']=='speedj/'+p)
            assert received['data']['arm_type']==mask and received['data']['joint_speed']==[.001]*size
        delays=[]
        for e in commands:
            if not e['endpoint'].startswith('servoj/'):continue
            p=e['endpoint'].split('/')[1];value=round(e['data']['joint'][0],9)
            key=(p,value)
            if key in published:delays.append((e['received_steady_ns']-published[key])/1e6)
        stop=[e for e in data if e['kind']=='service' and e['endpoint']=='stop']
        assert stop,'Stream watchdog did not request stop'
        assert 0 < (stop[0]['received_steady_ns']-end)/1e6 < 350, 'Stop request exceeded PC watchdog budget'
        count=len(commands)
        for _ in range(5):pubs['dual_arm'].publish(Joints(joint=[.2]*14));spin(.01)
        assert len([e for e in events() if e['kind']=='command'])==count,'Fault did not inhibit commands'
        assert call(Trigger,'/upperlimb/bridge/reset_fault',Trigger.Request()).success
        pubs['dual_arm'].publish(Joints(joint=[.21]*14));spin(.05)
        assert len([e for e in events() if e['kind']=='command'])>count
        recorded=[e for e in data if e['kind']=='service']
        fixed=next(e for e in recorded if e['endpoint']=='movej_by_pose/dual_arm')['request']
        assert fixed['q7']==[.3,.4] and fixed['pose'][1]['position']['x']==.22
        path=next(e for e in recorded if e['endpoint']=='movej_by_path/neck')['request']
        assert path['path'][1]['joint']==[.02,.03]
        report.update(command_latency_ms=distribution(delays),published_samples=len(published),
            received_matching_samples=len(delays),target_command_hz=200,
            watchdog_stop_after_stream_end_ms=(stop[0]['received_steady_ns']-end)/1e6,
            checked=['7 servoj groups including lift/whole_body','6 speedj groups and masks',
                'Cartesian services and command receivers available by default','read-only safety_lock response',
                'FK service response',
                'Servo and MoveJ requests','nested MoveJByPath','fixed Pose[2]/q7[2]',
                'invalid dimension and NaN rejection','stream timeout stop during blocked motion RPC',
                'fault inhibits and explicit reset'])
    else:
        spin(args.seconds)
        assert statuses and not statuses[-1]['read_only'] and statuses[-1]['sent']==0,statuses[-1:]
        connections=statuses[-1]['ros1_command_connections']
        assert len(connections)==13 and all(v>=1 for v in connections.values()),connections
        report['ros1_control_handshakes']=connections
        report['commands_published']=0
    ordered=[s['steady_ns'] for s in samples]
    intervals=[(b-a)/1e6 for a,b in zip(ordered,ordered[1:])]
    pipeline=[(seen[k]-timing[k])/1e6 for k in seen.keys() & timing.keys()]
    foreign=[n for n,_ in node.get_topic_names_and_types() if n.startswith('/zj_humanoid/sensor') or n in ('/cmd_vel','/wa1_mock/camera/image')]
    assert not foreign,foreign
    assert len(samples)>100
    assert all(len(s['names'])==19 and len(s['position'])==19 and all(math.isfinite(x) for x in s['position']) for s in samples)
    report.update(received_joint_samples=len(samples),joint_names=samples[-1]['names'],
        observed_hz=(len(samples)-1)*1e9/(ordered[-1]-ordered[0]),state_interval_ms=distribution(intervals),
        pc_ros1_callback_to_host_ros2_callback_ms=distribution(pipeline),
        source_stamp_age_ms=distribution([(s['received_ns']-s['stamp_ns'])/1e6 for s in samples]),
        forwarding_latency_clock='PC shared Linux CLOCK_MONOTONIC; source stamp is only a correlation key',
        custom_state_samples=extra,status=statuses[-1] if statuses else None,
        excluded_foreign_topics=foreign,first_joint_sample=samples[0],last_joint_sample=samples[-1])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
