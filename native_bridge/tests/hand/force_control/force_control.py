#!/usr/bin/env python3
"""Maintain four fingertip forces via bounded HandJoint position increments."""

import argparse
import json
import math
import sys
import time

from hand_common import (LIMITS, create_hand_client, exit_signals, keyboard,
                         poll_request, positions, read_key, reset_on_exit, reset_reference, send_target)


DEFAULTS = dict(
    side='right', targets=(0.17, 0.17, 0.17, 0.16), kp=0.1, max_step=0.05,
    deadband=0.01, max_force=2.0, rate=5.0, duration=None,
    feedback_timeout=0.5, service_timeout=1.0,
)


class Controller:
    def __init__(self, targets, indices, kp=DEFAULTS['kp'], max_step=DEFAULTS['max_step'],
                 deadband=DEFAULTS['deadband'], max_force=DEFAULTS['max_force'],
                 feedback_timeout=DEFAULTS['feedback_timeout']):
        if len(targets) != 4 or any(
            not math.isfinite(x) or not 0 <= x < max_force for x in targets
        ):
            raise ValueError('Four target forces must be finite, nonnegative and below max-force')
        if len(indices) != 4 or len(set(indices)) != 4 or any(
            not isinstance(i, int) or i < 0 for i in indices
        ):
            raise ValueError('Four distinct nonnegative pressure indices are required')
        for value in (kp, max_step, max_force, feedback_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Gain, step, force cutoff and timeout must be finite and positive')
        if not math.isfinite(deadband) or deadband < 0:
            raise ValueError('Deadband must be finite and nonnegative')
        self.targets, self.indices = list(targets), list(indices)
        self.kp, self.max_step, self.deadband = kp, max_step, deadband
        self.max_force, self.feedback_timeout = max_force, feedback_timeout
        self.target = None

    def start(self, q):
        self.target = positions(q)

    def step(self, q, pressure, now, joint_time, force_time):
        """Propose a step; callers commit target only when dispatching a request."""
        positions(q)
        if self.target is None:
            raise ValueError('Controller has not been initialized')
        if any(not math.isfinite(t) or not 0 <= now - t <= self.feedback_timeout
               for t in (joint_time, force_time)):
            raise ValueError('Stale joint or fingertip force feedback')
        if max(self.indices) >= len(pressure):
            raise ValueError('Pressure array does not contain the configured indices')
        forces = [float(pressure[i]) * 0.1 for i in self.indices]
        if any(not math.isfinite(f) for f in forces):
            raise ValueError('Nonfinite fingertip force feedback')
        if any(abs(f) >= self.max_force for f in forces):
            raise ValueError('Fingertip force cutoff reached; exiting force control')
        target = self.target.copy()
        for i, (force, desired) in enumerate(zip(forces, self.targets), 2):
            error = desired - force
            delta = 0 if abs(error) <= self.deadband + 1e-12 else max(
                -self.max_step, min(self.max_step, self.kp * error))
            lower, upper = LIMITS[i]
            target[i] = max(lower, min(upper, target[i] + delta))
        return target, forces


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.set_defaults(**DEFAULTS)
    parser.add_argument('--side', choices=('left', 'right'),
                        help='Hand to control')
    parser.add_argument('--targets', type=float, nargs=4,
                        metavar='N', help='Target N: index, middle, ring, little')
    parser.add_argument('--pressure-indices', type=int, nargs=4, required=True,
                        metavar='I', help='Verified zero-based indices: index, middle, ring, little')
    parser.add_argument('--kp', type=float, help='rad/N per update')
    parser.add_argument('--max-step', type=float, help='rad per update')
    parser.add_argument('--deadband', type=float, help='N')
    parser.add_argument('--max-force', type=float,
                        help='Absolute force cutoff N on any controlled finger ')
    parser.add_argument('--rate', type=float, help='Maximum update Hz')
    parser.add_argument('--duration', type=float,
                        help='Optional run seconds; default: run until q, Ctrl+C, or a fault')
    parser.add_argument('--feedback-timeout', type=float, help='Valid feedback timeout, seconds')
    parser.add_argument('--service-timeout', type=float, help='Service response timeout, seconds')
    parser.add_argument('--execute', action='store_true', help='Enable real HandJoint requests')
    args = parser.parse_args(argv)
    try:
        make_controller(args)
        for value in (args.rate, args.service_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Rate and service timeout must be finite and positive')
        if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
            raise ValueError('Duration must be finite and positive when specified')
        if args.rate > 20:
            raise ValueError('Rate must be at most 20 Hz for this position-service test')
    except ValueError as exc:
        parser.error(str(exc))
    return args


def make_controller(args):
    return Controller(args.targets, args.pressure_indices, args.kp, args.max_step,
                      args.deadband, args.max_force, args.feedback_timeout)


class ForceFeedback:
    """Retain valid force samples and latch overforce before rejecting malformed frames."""
    def __init__(self, args):
        self.args = args
        self.pressure = self.force_time = None
        self.force_error = self.force_invalid_since = self.force_fault = None

    def on_force(self, msg):
        now = time.monotonic()
        # A malformed companion channel must not hide a finite overforce event.
        for i in self.args.pressure_indices:
            if (i < len(msg.pressure) and math.isfinite(msg.pressure[i])
                    and abs(msg.pressure[i] * 0.1) >= self.args.max_force):
                self.force_fault = f'Fingertip force cutoff reached on pressure[{i}]'
        if max(self.args.pressure_indices) >= len(msg.pressure) or any(
            not math.isfinite(msg.pressure[i]) for i in self.args.pressure_indices
            if i < len(msg.pressure)
        ):
            self.force_error = 'Missing or nonfinite configured fingertip force channels'
            if self.force_invalid_since is None:
                self.force_invalid_since = now
            return
        self.pressure = list(msg.pressure)
        self.force_time = now
        self.force_error = self.force_invalid_since = None

    def check(self, joints):
        if self.force_fault:
            raise RuntimeError(self.force_fault)
        now = time.monotonic()
        joints.check(now)
        if self.force_invalid_since is not None and now - self.force_invalid_since >= self.args.feedback_timeout:
            raise RuntimeError('Invalid fingertip force feedback persisted: ' + self.force_error)
        if self.force_time is not None and now - self.force_time > self.args.feedback_timeout:
            raise RuntimeError('No fresh valid fingertip force feedback')


def run_ros(args):
    # Lazy imports allow --help without a sourced ROS environment.
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    force_feedback = ForceFeedback(args)
    node = create_hand_client('wa1_hand_force_control', args.side, args.feedback_timeout,
                              force_callback=force_feedback.on_force)
    controller = make_controller(args)
    future = None
    initial = None
    reset_target = None
    reset_ok = True
    try:
        # Service discovery blocks callbacks; obtain fresh startup positions afterwards.
        if args.execute and not node.client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('HandJoint service is unavailable')
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and (node.joints.q is None or force_feedback.pressure is None
                              or node.joints.error or force_feedback.force_error):
            force_feedback.check(node.joints)
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out waiting for joint and fingertip force feedback')
            rclpy.spin_once(node, timeout_sec=0.05)
        if not rclpy.ok():
            return
        force_feedback.check(node.joints)
        initial = node.joints.q.copy()
        reset_target = reset_reference(args.side, initial)
        controller.start(initial)
        # Validate both streams before emitting a preview or sending motion requests.
        controller.step(node.joints.q, force_feedback.pressure, time.monotonic(), node.joints.received_at, force_feedback.force_time)
        print(json.dumps({
            'mode': 'EXECUTE' if args.execute else 'PREVIEW_ONLY',
            'side': args.side, 'targets_N': args.targets,
            'pressure_indices': args.pressure_indices,
            'thumb_targets_rad': controller.target[:2], 'rate_Hz': args.rate,
            'initial_rad': initial, 'reset_target_rad': reset_target, 'exit_key': 'q (or Ctrl+C)',
        }), flush=True)
        started = time.monotonic()
        next_tick, sent_at, last_log = started, -math.inf, -math.inf
        with keyboard():
            while rclpy.ok() and (args.duration is None or time.monotonic() - started < args.duration):
                rclpy.spin_once(node, timeout_sec=0.01)
                if read_key() == 'q':
                    break
                force_feedback.check(node.joints)
                now = time.monotonic()
                if not args.execute:
                    controller.start(initial[:2] + node.joints.q[2:])
                proposed, forces = controller.step(
                    node.joints.q, force_feedback.pressure, now, node.joints.received_at, force_feedback.force_time)
                # Monitor force/freshness even when a service response is pending.
                if future is not None:
                    if poll_request(future, sent_at, args.service_timeout) is None:
                        continue
                    future = None
                if (node.joints.error or force_feedback.force_error or now < next_tick
                        or min(node.joints.received_at, force_feedback.force_time) <= sent_at):
                    continue
                if now - last_log >= 0.5:
                    print(json.dumps({'force_N': forces, 'target_rad': proposed,
                                      'mode': 'EXECUTE' if args.execute else 'PREVIEW_ONLY'}), flush=True)
                    last_log = now
                if args.execute and proposed != controller.target:
                    future = send_target(node, proposed)
                    controller.target = proposed
                    sent_at = now
                next_tick = now + 1.0 / args.rate

    except KeyboardInterrupt:
        print('Exit requested.', flush=True)
    finally:
        try:
            reset_ok = reset_on_exit(node, reset_target, future, args.execute)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return reset_ok


def main():
    args = parse_args()
    try:
        with exit_signals():
            return 0 if run_ros(args) else 1
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print('Force control stopped: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
