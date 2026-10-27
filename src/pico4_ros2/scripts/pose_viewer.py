#!/usr/bin/python3
"""Local, read-only browser viewer for PICO4 XR poses."""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from pico4_ros2.msg import XrFrame
from viewer_math import DEFAULT_AXIS_MAP, axis_matrix, display_pose


class PoseStore:
    def __init__(self, axis_tokens):
        self.axis_tokens = tuple(axis_tokens)
        self.matrix = axis_matrix(self.axis_tokens)
        self.lock = threading.Lock()
        self.origin_raw = None
        self.latest_frame = None
        self.latest = None
        self.last_arrival = None
        self.rate_hz = 0.0
        self.sequence = 0
        self.origin_epoch = 0
        self.rejected_frames = 0
        self.device_id = None

    @staticmethod
    def _controller_inputs(controller):
        return {
            'axis_x': float(controller.axis_x),
            'axis_y': float(controller.axis_y),
            'gripper': float(controller.gripper),
            'trigger': float(controller.trigger),
            'axis_click': bool(controller.axis_click),
            'primary_button': bool(controller.primary_button),
            'secondary_button': bool(controller.secondary_button),
            'menu_button': bool(controller.menu_button),
        }

    def _pose(self, raw_pose, status):
        try:
            value = display_pose(raw_pose, self.origin_raw, self.matrix)
            value['status'] = int(status)
            return value
        except ValueError:
            return None

    def _make_latest(self, frame):
        return {
            'sequence': self.sequence,
            'origin_epoch': self.origin_epoch,
            'axis_map': self.axis_tokens,
            'device_id': frame.device_id,
            'frame_id': frame.header.frame_id,
            # JavaScript numbers cannot exactly represent nanosecond int64 values.
            'device_timestamp_ns': str(frame.device_timestamp_ns),
            'pc_receive_monotonic_ns': str(frame.pc_receive_monotonic_ns),
            'battery': int(frame.battery),
            'input': int(frame.input),
            'poses': {
                'head': self._pose(frame.head.pose, frame.head.status),
                'left': self._pose(frame.left_controller.tracked_pose.pose,
                                   frame.left_controller.tracked_pose.status),
                'right': self._pose(frame.right_controller.tracked_pose.pose,
                                    frame.right_controller.tracked_pose.status),
            },
            'inputs': {
                'left': self._controller_inputs(frame.left_controller),
                'right': self._controller_inputs(frame.right_controller),
            },
        }

    def update(self, frame):
        arrival = time.monotonic()
        with self.lock:
            head_raw = list(frame.head.pose)
            if frame.device_id != self.device_id:
                self.device_id = frame.device_id
                self.origin_raw = None
                self.origin_epoch += 1
                self.rate_hz = 0.0
                self.last_arrival = None
            if self.origin_raw is None:
                try:
                    display_pose(head_raw, head_raw[:3], self.matrix)
                except ValueError:
                    self.rejected_frames += 1
                    return
                self.origin_raw = head_raw[:3]
            elif self._pose(head_raw, frame.head.status) is None:
                self.rejected_frames += 1
                return
            if self.last_arrival is not None and arrival > self.last_arrival:
                sample_hz = 1.0 / (arrival - self.last_arrival)
                self.rate_hz = sample_hz if self.rate_hz == 0.0 else 0.9 * self.rate_hz + 0.1 * sample_hz
            self.last_arrival = arrival
            self.latest_frame = frame
            self.sequence += 1
            self.latest = self._make_latest(frame)

    def recenter(self):
        with self.lock:
            if self.latest_frame is None:
                return False
            self.origin_raw = list(self.latest_frame.head.pose[:3])
            self.origin_epoch += 1
            self.latest = self._make_latest(self.latest_frame)
            return True

    def snapshot(self):
        with self.lock:
            age_ms = None if self.last_arrival is None else (time.monotonic() - self.last_arrival) * 1000.0
            state = dict(self.latest) if self.latest is not None else {
                'axis_map': self.axis_tokens,
                'origin_epoch': self.origin_epoch,
                'poses': {'head': None, 'left': None, 'right': None},
                'inputs': {},
                'sequence': 0,
            }
            state['frame_age_ms'] = age_ms
            state['fresh'] = age_ms is not None and age_ms < 250.0
            state['receive_rate_hz'] = self.rate_hz
            state['rejected_frames'] = self.rejected_frames
            return state


class ViewerServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class ViewerHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, _format, *_args):
        pass

    def _reply(self, code, content_type, body):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code, payload):
        body = json.dumps(payload, allow_nan=False, separators=(',', ':')).encode('utf-8')
        self._reply(code, 'application/json; charset=utf-8', body)

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/events':
            self._events()
            return
        if path == '/api/state':
            self._json(200, self.server.store.snapshot())
            return
        assets = {
            '/': ('index.html', 'text/html; charset=utf-8'),
            '/viewer.js': ('viewer.js', 'text/javascript; charset=utf-8'),
            '/viewer.css': ('viewer.css', 'text/css; charset=utf-8'),
        }
        if path not in assets:
            self._reply(404, 'text/plain; charset=utf-8', b'Not found')
            return
        name, content_type = assets[path]
        try:
            self._reply(200, content_type, (self.server.asset_root / name).read_bytes())
        except OSError:
            self._reply(500, 'text/plain; charset=utf-8', b'Viewer asset unavailable')

    def do_POST(self):
        if self.path != '/api/recenter':
            self._reply(404, 'text/plain; charset=utf-8', b'Not found')
            return
        ok = self.server.store.recenter()
        self._json(200 if ok else 409, {'ok': ok})

    def _events(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        try:
            while not self.server.stopping.is_set():
                state = self.server.store.snapshot()
                data = json.dumps(state, allow_nan=False, separators=(',', ':'))
                self.wfile.write(('data: ' + data + '\n\n').encode('utf-8'))
                self.wfile.flush()
                time.sleep(1.0 / self.server.display_hz)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


class PoseViewer(Node):
    def __init__(self):
        super().__init__('pico4_pose_viewer')
        axis_tokens = self.declare_parameter('world_from_xr_axes', list(DEFAULT_AXIS_MAP)).value
        port = self.declare_parameter('port', 8765).value
        display_hz = self.declare_parameter('display_hz', 20.0).value
        if not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError('port must be an integer in [1, 65535]')
        if not 1.0 <= display_hz <= 60.0:
            raise ValueError('display_hz must be in [1, 60]')
        self.store = PoseStore(axis_tokens)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(XrFrame, '/pico4/frame', self.store.update, qos)
        self.httpd = ViewerServer(('127.0.0.1', port), ViewerHandler)
        self.httpd.store = self.store
        self.httpd.asset_root = Path(get_package_share_directory('pico4_ros2')) / 'web'
        self.httpd.display_hz = float(display_hz)
        self.httpd.stopping = threading.Event()
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'Read-only pose viewer: http://127.0.0.1:{port}  '
            f'axis map={list(axis_tokens)}; subscribed only to /pico4/frame')

    def close(self):
        self.httpd.stopping.set()
        self.httpd.shutdown()
        self.httpd.server_close()
        self.thread.join(timeout=2.0)


def main():
    rclpy.init()
    node = None
    try:
        node = PoseViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close()
            except KeyboardInterrupt:
                # ros2 launch can send a second SIGINT during HTTP shutdown.
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
