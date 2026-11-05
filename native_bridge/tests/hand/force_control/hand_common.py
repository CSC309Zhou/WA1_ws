"""Shared feedback validation, terminal handling, and exit reset for hand tests."""

from contextlib import contextmanager
import math
import os
import select
import signal
import sys
import termios
import time
import tty


JOINTS = ('THUMB_MP', 'THUMB_CMC', 'INDEX_MCP', 'MIDDLE_MCP', 'RING_MCP', 'LITTLE_MCP')
LIMITS = ((-0.75, 0.75), (-0.31, 1.54)) + ((0.0, 1.36),) * 4

# Right-hand reference explicitly selected from the robot reading on 2026-11-05.
# Order follows JOINTS; runtime startup readings must not overwrite this reference.
RIGHT_RESET_RAD = (
    -0.089, -0.000,
    0.346, 0.346,
    0.348, 0.346,
)


def positions(values):
    values = list(values)
    if len(values) != 6:
        raise ValueError('Expected six hand positions')
    for name, q, (lo, hi) in zip(JOINTS, values, LIMITS):
        if not math.isfinite(q) or not lo <= q <= hi:
            raise ValueError(f'{name}={q:.8g} rad; allowed [{lo}, {hi}] rad')
    return values


def reset_reference(side, startup):
    return positions(RIGHT_RESET_RAD if side == 'right' else startup)


class JointFeedback:
    def __init__(self, side, timeout=0.5):
        self.names = [name + '_' + side.upper() for name in JOINTS]
        self.timeout = timeout
        self.q = None
        self.received_at = None
        self.error = None
        self.invalid_since = None

    def update(self, msg):
        now = time.monotonic()
        try:
            if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
                raise ValueError('Malformed joint names/positions')
            by_name = dict(zip(msg.name, msg.position))
            q = positions([by_name[name] for name in self.names])
        except (KeyError, ValueError) as exc:
            self.error = str(exc)
            if self.invalid_since is None:
                self.invalid_since = now
            return
        self.q, self.received_at = q, now
        self.error = self.invalid_since = None

    def check(self, now):
        if self.invalid_since is not None and now - self.invalid_since >= self.timeout:
            raise RuntimeError(f'Invalid joint feedback persisted for {self.timeout} s: {self.error}')
        if self.received_at is not None and now - self.received_at > self.timeout:
            raise RuntimeError(f'No valid joint feedback for {self.timeout} s')


def create_hand_client(node_name, side, feedback_timeout=0.5, force_callback=None):
    """Create the common ROS2 endpoints; ROS imports stay lazy for CLI help."""
    from hand.msg import PressureSensor
    from hand.srv import HandJoint
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState

    class HandClient(Node):
        def __init__(self):
            super().__init__(node_name)
            self.joints = JointFeedback(side, feedback_timeout)
            self.pressure = self.force_received_at = None
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(JointState, '/hand/joint_states', self.joints.update, qos)
            self.create_subscription(PressureSensor, '/hand/finger_pressures/' + side, self.on_force, qos)
            self.client = self.create_client(HandJoint, '/hand/joint_switch/' + side)

        def on_force(self, msg):
            self.pressure = list(msg.pressure)
            self.force_received_at = time.monotonic()
            if force_callback is not None:
                force_callback(msg)

        def force_snapshot(self, now):
            if self.pressure is None:
                return {'force_status': 'not_received', 'pressure_raw': None, 'force_N': None}
            age = now - self.force_received_at
            valid = bool(self.pressure) and all(math.isfinite(x) for x in self.pressure)
            return {
                'force_status': 'invalid' if not valid else ('stale' if age > 0.5 else 'fresh'),
                'force_age_s': age,
                'pressure_indices': list(range(len(self.pressure))),
                'pressure_raw': [x if math.isfinite(x) else None for x in self.pressure],
                'force_N': [x * 0.1 for x in self.pressure] if valid else None,
            }

    return HandClient()


def send_target(node, target):
    """Validate all six positions immediately before every request, including reset."""
    from hand.srv import HandJoint
    request = HandJoint.Request()
    request.q = positions(target)
    return node.client.call_async(request)


def poll_request(future, sent_at, timeout):
    if time.monotonic() - sent_at > timeout:
        raise RuntimeError('HandJoint response timed out; remote execution is unknown')
    if not future.done():
        return None
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError('HandJoint failed: ' + getattr(response, 'message', 'no response'))
    return response


@contextmanager
def keyboard(required=False):
    if not sys.stdin.isatty():
        if required:
            raise RuntimeError('Run this script in an interactive terminal')
        yield
        return
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def read_key():
    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        return os.read(sys.stdin.fileno(), 1).decode('ascii', errors='ignore').lower()
    return None


@contextmanager
def exit_signals():
    """Keep the ROS context alive while Ctrl+C/SIGTERM enters reset cleanup."""
    def interrupted(signum, frame):
        raise KeyboardInterrupt

    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for sig in previous:
            signal.signal(sig, interrupted)
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def reset_on_exit(node, target, pending, execute, timeout=3.0):
    """Reset to the selected reference, returning False if completion cannot be verified."""
    if target is None:
        print('No valid startup position captured; no reset request sent.', flush=True)
        return True
    if not execute:
        print(f'PREVIEW_ONLY: reset target_rad={target}; no request sent.', flush=True)
        return True
    import rclpy

    def wait(future, deadline):
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        return future.done()

    try:
        positions(target)
        if not rclpy.ok():
            raise RuntimeError('ROS context is unavailable')
        if pending is not None and not wait(pending, time.monotonic() + timeout):
            raise RuntimeError('Previous request still pending; reset not sent to avoid overlapping requests')
        if not node.client.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('HandJoint service unavailable')
        print('Resetting hand to reference positions...', flush=True)
        future = send_target(node, target)
        if not wait(future, time.monotonic() + timeout):
            raise RuntimeError('Reset response timed out; remote execution is unknown')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('Reset rejected: ' + getattr(response, 'message', 'no response'))
        acknowledged_at = time.monotonic()
        deadline = acknowledged_at + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            feedback = node.joints
            if (feedback.error is None and feedback.received_at is not None
                    and feedback.received_at > acknowledged_at
                    and max(abs(a - b) for a, b in zip(feedback.q, target)) <= 0.03):
                print('Reset verified: hand reached reference positions (tolerance 0.03 rad).', flush=True)
                return True
        raise RuntimeError('Reset accepted, but return to reference positions was not verified')
    except Exception as exc:
        print('RESET FAILED: ' + str(exc), file=sys.stderr, flush=True)
        return False
