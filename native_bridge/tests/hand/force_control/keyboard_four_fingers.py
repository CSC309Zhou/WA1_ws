#!/usr/bin/env python3
"""Use c/o to close/open four fingers by one position step; q exits."""

import argparse
import json
import math
import sys
import time

from hand_common import (LIMITS, create_hand_client, exit_signals, keyboard,
                         poll_request, read_key, reset_on_exit, reset_reference, send_target)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), default='right')
    parser.add_argument('--step', type=float, default=0.05, help='Angle per keypress, rad (default: 0.05)')
    parser.add_argument('--execute', action='store_true', help='Send real HandJoint requests')
    args = parser.parse_args()
    if not math.isfinite(args.step) or not 0 < args.step <= 0.1:
        parser.error('--step must be finite and in (0, 0.1] rad')
    return args


def run(args):
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = create_hand_client('wa1_four_finger_keyboard', args.side)
    future = None
    initial = None
    reset_target = None
    reset_ok = True
    try:
        if args.execute and not node.client.wait_for_service(timeout_sec=3):
            raise RuntimeError('HandJoint service is unavailable')
        deadline = time.monotonic() + 5
        while rclpy.ok() and node.joints.q is None:
            node.joints.check(time.monotonic())
            if time.monotonic() >= deadline:
                raise RuntimeError('No valid hand joint feedback received' +
                                   (': ' + node.joints.error if node.joints.error else ''))
            rclpy.spin_once(node, timeout_sec=0.05)
        if not rclpy.ok():
            return
        node.joints.check(time.monotonic())
        initial = node.joints.q.copy()
        reset_target = reset_reference(args.side, initial)
        thumb = initial[:2]
        mode = 'EXECUTE' if args.execute else 'PREVIEW_ONLY'
        print(f'{mode}: {args.side}; c=close, o=open, p=positions, f=force, q=quit; step={args.step} rad', flush=True)
        print(json.dumps({'joint_names': node.joints.names, 'current_rad': node.joints.q,
                          'thumb_targets_rad': thumb, 'initial_rad': initial,
                          'reset_target_rad': reset_target}), flush=True)
        sent_at, last_key = -math.inf, -math.inf
        with keyboard(required=True):
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                node.joints.check(now)
                if future is not None:
                    response = poll_request(future, sent_at, 1.0)
                    if response is not None:
                        print('HandJoint accepted: ' + response.message, flush=True)
                        future = None
                key = read_key()
                if key is None:
                    continue
                if key == 'q':
                    break
                if key == 'p':
                    print(json.dumps({'current_rad': node.joints.q}), flush=True)
                    continue
                if key == 'f':
                    print(json.dumps({'side': args.side, **node.force_snapshot(now)}), flush=True)
                    continue
                if key not in ('c', 'o'):
                    continue
                if node.joints.error:
                    continue
                if future is not None or node.joints.received_at <= sent_at:
                    print('Waiting for service response or new joint feedback; key ignored.', flush=True)
                    continue
                if now - last_key < 0.2:
                    continue
                last_key = now
                delta = args.step if key == 'c' else -args.step
                target = thumb + [max(lo, min(hi, q + delta))
                                  for q, (lo, hi) in zip(node.joints.q[2:], LIMITS[2:])]
                print(json.dumps({'mode': mode, 'action': 'close' if key == 'c' else 'open',
                                  'current_rad': node.joints.q, 'target_rad': target}), flush=True)
                if args.execute:
                    future = send_target(node, target)
                    sent_at = now
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
    if not sys.stdin.isatty():
        print('Run this script in an interactive terminal.', file=sys.stderr)
        return 1
    try:
        with exit_signals():
            return 0 if run(args) else 1
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print('Keyboard control stopped: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
