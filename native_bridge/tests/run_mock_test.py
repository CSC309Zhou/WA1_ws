#!/usr/bin/env python3
"""Run the PC-only ROS1/ROS2 upperlimb integration test on loopback.

This script never uses the robot Master. Only the two fixed mock container names
are started and stopped; an existing container with either name is left alone.
"""
import argparse
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
NATIVE = HERE.parent
WS = NATIVE.parent
DOCKER = [sys.executable, str(NATIVE / 'pc_docker.py')]
MANAGER = [sys.executable, str(NATIVE / 'bridge.py')]
FIXTURE = 'wa1-upperlimb-mock'
BRIDGE = 'wa1-upperlimb-test-bridge'


def run(argv, **kwargs):
    subprocess.run(argv, cwd=WS, check=True, **kwargs)


def container_exists(name):
    result = subprocess.run(DOCKER + ['container', 'inspect', name], cwd=WS,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return result.returncode == 0


def wait_for_master():
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(('127.0.0.1', 11321), timeout=.5):
                return
        except OSError:
            time.sleep(.2)
    raise RuntimeError('PC mock ROS1 Master did not start on 127.0.0.1:11321')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20,
                        help='Duration of 200 Hz mock stream (default: 20)')
    parser.add_argument('--latency-probe', action='store_true',
                        help='Also validate the optional PC bridge left-arm timing topic')
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    if container_exists(FIXTURE) or container_exists(BRIDGE):
        raise RuntimeError('A fixed-name mock container already exists; inspect it before retrying')
    output_dir = NATIVE / 'results'
    output_dir.mkdir(exist_ok=True)
    events = output_dir / 'mock_events.jsonl'
    result = output_dir / 'mock_control_results.json'
    fixture_started = bridge_started = False
    try:
        run(DOCKER + [
            'run', '-d', '--name', FIXTURE, '--network', 'host', '--init',
            '--user', f'{os.getuid()}:{os.getgid()}',
            '-e', 'HOME=/tmp/wa1-home', '-e', 'ROS_HOME=/tmp/wa1-ros',
            '-e', 'ROS_MASTER_URI=http://127.0.0.1:11321',
            '-e', 'ROS_IP=127.0.0.1', '-e', 'ROS_DOMAIN_ID=83',
            '--mount', f'type=bind,src={HERE},dst=/tests,readonly',
            '--mount', f'type=bind,src={output_dir},dst=/evidence',
            '--entrypoint', '/bin/bash', 'wa1-upperlimb-bridge:humble',
            '-lc', 'source /wa1/ros1_interfaces/install/setup.bash; '
                   'exec python3 /tests/mock_ros1.py'])
        fixture_started = True
        wait_for_master()
        run(MANAGER + ['start', '--mock'] +
            (['--latency-probe'] if args.latency_probe else []))
        bridge_started = True
        command = ('source native_bridge/setup_pc.bash; '
                   'exec python3 native_bridge/tests/ros2_probe.py '
                   '--mock-control --seconds "$1" --events "$2" --output "$3"' +
                   (' --latency-probe' if args.latency_probe else ''))
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '83'
        run(['/bin/bash', '-c', command, 'wa1-mock-test', str(args.seconds),
             str(events), str(result)], env=env)
        print(f'PC-only test passed; report: {result}')
    finally:
        if bridge_started:
            run(MANAGER + ['stop', '--mock'])
        if fixture_started:
            run(DOCKER + ['stop', '--timeout', '5', FIXTURE])
            run(DOCKER + ['container', 'rm', FIXTURE])


if __name__ == '__main__':
    main()
