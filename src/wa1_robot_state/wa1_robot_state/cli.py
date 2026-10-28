"""Inspect the WA1 upper-limb state and explicitly request safety unlock."""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from upperlimb.msg import UplimbState


STATE_TOPIC = '/upperlimb/uplimb_state'
LOCK_SERVICE = '/upperlimb/safety_lock'
UNLOCK_SERVICE = '/upperlimb/unlock'
# ROS2 Python generates .msg "byte" constants as one-byte bytes objects.
PROTECTED_MODE = UplimbState.PROTECTED[0]


class StateClient(Node):
    def __init__(self):
        super().__init__('wa1_robot_state_cli')
        self.state = None
        self.state_received_at = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(UplimbState, STATE_TOPIC, self._on_state, qos)
        self.lock_client = self.create_client(Trigger, LOCK_SERVICE)
        self.unlock_client = self.create_client(Trigger, UNLOCK_SERVICE)

    def _on_state(self, message):
        self.state = message
        self.state_received_at = time.monotonic()

    def wait_state(self, timeout, after=0.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.state is not None and self.state_received_at > after:
                return self.state
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        raise RuntimeError(f'No fresh {STATE_TOPIC} message within {timeout:g} s')

    def call_trigger(self, client, timeout):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service {client.srv_name} unavailable')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Service {client.srv_name} did not return within {timeout:g} s')
        return future.result()


def state_result(client, state):
    return {
        'cmd_num': int(state.cmd_num),
        'cmd_name': state.cmd_name,
        'protected': int(state.cmd_num) == PROTECTED_MODE,
        'iddp_status': bool(state.iddp_status),
        'left_arm_is_singular': bool(state.left_arm_is_singular),
        'right_arm_is_singular': bool(state.right_arm_is_singular),
        'sample_age_ms': round((time.monotonic() - client.state_received_at) * 1000, 1),
        'robot_stamp': {'sec': state.header.stamp.sec, 'nanosec': state.header.stamp.nanosec},
    }


def lock_result(response):
    # The robot uses Trigger.success as the lock-state bit, including False when
    # the service call succeeded and the safety lock is inactive.
    return {'locked': bool(response.success), 'message': response.message}


def run_status(client, timeout):
    lock = client.call_trigger(client.lock_client, timeout)
    state = client.wait_state(timeout, after=time.monotonic())
    return {'upperlimb': state_result(client, state), 'safety_lock': lock_result(lock)}


def run_unlock(client, timeout):
    lock = client.call_trigger(client.lock_client, timeout)
    state = client.wait_state(timeout, after=time.monotonic())
    before = {'upperlimb': state_result(client, state), 'safety_lock': lock_result(lock)}
    if not lock.success and int(state.cmd_num) != PROTECTED_MODE:
        return {'action': 'already_unlocked', 'before': before, 'after': before}

    response = client.call_trigger(client.unlock_client, timeout)
    if not response.success:
        raise RuntimeError(f'{UNLOCK_SERVICE} rejected the request: {response.message}')

    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            state = client.wait_state(min(0.3, remaining), after=time.monotonic())
        except RuntimeError:
            continue
        lock = client.call_trigger(client.lock_client, min(1.0, max(0.1, deadline - time.monotonic())))
        last = {'upperlimb': state_result(client, state), 'safety_lock': lock_result(lock)}
        if not lock.success and int(state.cmd_num) != PROTECTED_MODE:
            return {'action': 'unlock_requested', 'unlock_message': response.message,
                    'before': before, 'after': last}
        rclpy.spin_once(client, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
    raise RuntimeError(f'Unlock was accepted but unlocked feedback was not confirmed: {last}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=5.0, metavar='SECONDS',
                        help='Maximum time for each state or service wait (default: 5)')
    parser.add_argument('action', choices=('status', 'lock', 'unlock'))
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error('--timeout must be positive')

    rclpy.init()
    client = StateClient()
    try:
        if args.action == 'status':
            result = run_status(client, args.timeout)
        elif args.action == 'lock':
            result = {'safety_lock': lock_result(client.call_trigger(client.lock_client, args.timeout))}
        else:
            result = run_unlock(client, args.timeout)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except RuntimeError as error:
        print(f'wa1_state: {error}', file=sys.stderr)
        return 1
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
