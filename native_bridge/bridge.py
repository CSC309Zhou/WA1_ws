#!/usr/bin/env python3
"""Manage only this project's PC-side bridge; never SSH or write robot parameters."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
WS = HERE.parent
DOCKER = [sys.executable, str(HERE / 'pc_docker.py')]
IMAGE = 'wa1-upperlimb-bridge:humble'
LOCAL_BASE_IMAGE = 'wa1_ros1_bridge_humble:latest'


def inspect_container(name):
    result = subprocess.run(DOCKER + ['container', 'inspect', name, '--format', '{{json .}}'],
                            cwd=WS, capture_output=True, text=True)
    if result.returncode:
        return None
    return json.loads(result.stdout)

def main():
    p = argparse.ArgumentParser()
    p.add_argument('action', choices=['build', 'start', 'stop', 'logs'])
    p.add_argument('--base-image', default=os.environ.get('WA1_BASE_IMAGE', LOCAL_BASE_IMAGE),
                   help='Base image for build; use a registry digest for release builds')
    p.add_argument('--pc-ip', default=os.environ.get('WA1_PC_IP', '192.168.217.10'),
                   help='PC address reachable by the robot ROS1 nodes')
    p.add_argument('--master-uri', default=os.environ.get('WA1_ROS1_MASTER_URI',
                                                         'http://192.168.217.1:11311'),
                   help='Robot ROS1 Master URI')
    p.add_argument('--pico-ip', default=os.environ.get('WA1_PICO_IP', '192.168.217.66'),
                   help='Address for the robot pico.zjrx.com ROS1 host alias')
    p.add_argument('--jetson-ip', default=os.environ.get('WA1_JETSON_IP', '192.168.217.1'),
                   help='Address for the robot jzrobot-a ROS1 host alias')
    p.add_argument('--latency-probe', action='store_true',
                   help='Publish optional PC-only left-arm ServoJ callback timing')
    p.add_argument('--mock', action='store_true', help='Use PC loopback ROS Master :11321, DDS domain 83')
    p.add_argument('--domain', type=int, default=0)
    args = p.parse_args()
    name = 'wa1-upperlimb-test-bridge' if args.mock else 'wa1-upperlimb-bridge'
    if args.action == 'build':
        command = ['build', '--network', 'host', '--progress', 'plain', '-f',
                   'native_bridge/Dockerfile', '--build-arg', f'BASE_IMAGE={args.base_image}',
                   '-t', IMAGE, '.']
    elif args.action == 'start':
        ip = '127.0.0.1' if args.mock else args.pc_ip
        master = 'http://127.0.0.1:11321' if args.mock else args.master_uri
        domain = 83 if args.mock else args.domain
        extra_hosts = [] if args.mock else [f'pico.zjrx.com:{args.pico_ip}',
                                             f'jzrobot-a:{args.jetson_ip}']
        command = ['run', '-d', '--name', name, '--network', 'host', '--ipc', 'host', '--init',
                   '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'ROS_LOG_DIR=/tmp/wa1-bridge-log',
                   '--label', 'wa1.project=upperlimb-native-bridge']
        for host in extra_hosts:
            command += ['--add-host', host]
        command += ['-e', f'ROS_MASTER_URI={master}', '-e', f'ROS_IP={ip}',
                   '-e', f'ROS_DOMAIN_ID={domain}', IMAGE,
                   'ros2', 'run', 'wa1_native_bridge', 'upperlimb_bridge', '--ros-args',
                   '-p', 'include_lift:=true']
        if args.latency_probe:
            command += ['-p', 'latency_probe:=true']
        existing = inspect_container(name)
        if existing:
            env = dict(value.split('=', 1) for value in existing['Config']['Env'] if '=' in value)
            expected_cmd = command[command.index(IMAGE) + 1:]
            same = (existing['State']['Running'] and existing['Config']['Image'] == IMAGE and
                    existing['Config']['Cmd'] == expected_cmd and
                    existing['Config']['Labels'].get('wa1.project') == 'upperlimb-native-bridge' and
                    existing['HostConfig']['NetworkMode'] == 'host' and
                    existing['HostConfig']['IpcMode'] == 'host' and
                    set(existing['HostConfig'].get('ExtraHosts') or []) == set(extra_hosts) and
                    all(env.get(key) == value for key, value in {
                        'ROS_MASTER_URI': master, 'ROS_IP': ip,
                        'ROS_DOMAIN_ID': str(domain)}.items()))
            if same:
                print(f'{name} 已按相同配置运行；无需再次启动。')
                return 0
            print(f'{name} 已存在且配置或状态不同。请先查看 logs；确定要切换时运行 '
                  f'python3 WA1_ws/native_bridge/bridge.py stop'
                  f'{" --mock" if args.mock else ""}，再执行 start。', file=sys.stderr)
            return 2
    elif args.action == 'logs':
        command = ['logs', '--tail', '100', name]
    else:
        # Only the fixed, project-owned name is accepted. No wildcard cleanup.
        r = subprocess.run(DOCKER + ['stop', '--timeout', '5', name], cwd=WS)
        if r.returncode:
            return r.returncode
        command = ['container', 'rm', name]
    return subprocess.run(DOCKER + command, cwd=WS).returncode

if __name__ == '__main__':
    raise SystemExit(main())
