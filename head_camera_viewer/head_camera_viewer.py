#!/usr/bin/env python3
"""Display the WA1 head RealSense color and aligned depth streams on this PC."""

from __future__ import annotations

import argparse
import base64
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import websocket


COLOR_TOPIC = "/zj_humanoid/sensor/realsense_head/color/image_raw/compressed"
DEPTH_TOPIC = (
    "/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/"
    "image_raw/compressedDepth"
)
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
WEB_ROOT = Path(__file__).resolve().parent / "web"


class StreamState:
    def __init__(self) -> None:
        self.changed = threading.Condition()
        self.sequence = 0
        self.jpeg: bytes | None = None
        self.arrivals: deque[float] = deque(maxlen=90)
        self.last_arrival: float | None = None
        self.width = 0
        self.height = 0
        self.preview_width = 0
        self.preview_height = 0
        self.source_bytes = 0
        self.valid_percent: float | None = None
        self.connected = False
        self.error = ""
        self.decode_errors = 0
        self.frame_id = ""

    def set_connection(self, connected: bool, error: str = "") -> None:
        with self.changed:
            self.connected = connected
            self.error = error
            self.changed.notify_all()

    def note_bad_frame(self, error: Exception) -> None:
        with self.changed:
            self.decode_errors += 1
            self.error = f"Frame decode: {error}"

    def update(
        self,
        jpeg: bytes,
        width: int,
        height: int,
        source_bytes: int,
        frame_id: str,
        valid_percent: float | None = None,
        preview_size: tuple[int, int] | None = None,
    ) -> None:
        now = time.monotonic()
        with self.changed:
            self.sequence += 1
            self.jpeg = jpeg
            self.arrivals.append(now)
            self.last_arrival = now
            self.width = width
            self.height = height
            self.preview_width, self.preview_height = preview_size or (width, height)
            self.source_bytes = source_bytes
            self.valid_percent = valid_percent
            self.frame_id = frame_id
            self.error = ""
            self.changed.notify_all()

    def latest_after(self, sequence: int, stop: threading.Event) -> tuple[int, bytes] | None:
        with self.changed:
            self.changed.wait_for(
                lambda: self.sequence > sequence or stop.is_set(), timeout=1.0
            )
            if stop.is_set() or self.sequence <= sequence or self.jpeg is None:
                return None
            return self.sequence, self.jpeg

    def wake(self) -> None:
        with self.changed:
            self.changed.notify_all()

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self.changed:
            age_ms = (
                None
                if self.last_arrival is None
                else round((now - self.last_arrival) * 1000, 1)
            )
            arrivals = self.arrivals
            fps = (
                (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
                if len(arrivals) > 1 and arrivals[-1] > arrivals[0]
                else 0.0
            )
            fresh = age_ms is not None and age_ms < 2000
            state = "live" if self.connected and fresh else "waiting" if self.connected else "offline"
            return {
                "state": state,
                "connected": self.connected,
                "fps": round(fps, 1) if fresh else 0.0,
                "width": self.width,
                "height": self.height,
                "preview_width": self.preview_width,
                "preview_height": self.preview_height,
                "frame_age_ms": age_ms,
                "source_kib": round(self.source_bytes / 1024, 1),
                "valid_percent": self.valid_percent,
                "frames": self.sequence,
                "decode_errors": self.decode_errors,
                "frame_id": self.frame_id,
                "error": self.error,
            }


def decode_color(
    message: dict, known_size: tuple[int, int] | None = None
) -> tuple[bytes, int, int, int, float | None, tuple[int, int]]:
    if "jpeg" not in str(message.get("format", "")).lower():
        raise ValueError(f"Unexpected color format: {message.get('format')}")
    data = base64.b64decode(message["data"], validate=True)
    if not data.startswith(b"\xff\xd8"):
        raise ValueError("Color payload is not JPEG")
    if known_size is None:
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("JPEG image could not be decoded")
        height, width = image.shape[:2]
    else:
        width, height = known_size
    return data, width, height, len(data), None, (width, height)


def decode_depth(
    message: dict, min_m: float, max_m: float, preview_width: int
) -> tuple[bytes, int, int, int, float, tuple[int, int]]:
    image_format = str(message.get("format", "")).lower()
    if "16uc1" not in image_format or "compresseddepth png" not in image_format:
        raise ValueError(f"Unexpected depth format: {message.get('format')}")
    data = base64.b64decode(message["data"], validate=True)
    offset = data.find(PNG_SIGNATURE, 0, 32)
    if offset < 0:
        raise ValueError("CompressedDepth PNG signature missing")
    depth_mm = cv2.imdecode(
        np.frombuffer(data[offset:], dtype=np.uint8), cv2.IMREAD_UNCHANGED
    )
    if depth_mm is None or depth_mm.ndim != 2 or depth_mm.dtype != np.uint16:
        raise ValueError("Expected a 16-bit single-channel depth image")

    height, width = depth_mm.shape
    valid_percent = round(float(np.count_nonzero(depth_mm)) / depth_mm.size * 100.0, 1)
    if width > preview_width:
        preview_height = round(height * preview_width / width)
        depth_mm = cv2.resize(
            depth_mm, (preview_width, preview_height), interpolation=cv2.INTER_NEAREST
        )
    valid = depth_mm > 0
    intensity = np.clip(
        (max_m * 1000.0 - depth_mm.astype(np.float32))
        * (255.0 / ((max_m - min_m) * 1000.0)),
        0,
        255,
    ).astype(np.uint8)
    rendered = cv2.applyColorMap(intensity, cv2.COLORMAP_TURBO)
    rendered[~valid] = (18, 18, 18)
    ok, encoded = cv2.imencode(
        ".jpg", rendered, [int(cv2.IMWRITE_JPEG_QUALITY), 82]
    )
    if not ok:
        raise ValueError("Could not encode the depth preview")
    rendered_height, rendered_width = depth_mm.shape
    return (
        encoded.tobytes(), width, height, len(data), valid_percent,
        (rendered_width, rendered_height),
    )


class CameraReceiver(threading.Thread):
    def __init__(
        self,
        kind: str,
        host: str,
        port: int,
        state: StreamState,
        stop: threading.Event,
        depth_range: tuple[float, float],
        depth_preview_width: int,
    ) -> None:
        super().__init__(name=f"head-camera-{kind}", daemon=True)
        self.kind = kind
        self.url = f"ws://{host}:{port}"
        self.topic = COLOR_TOPIC if kind == "color" else DEPTH_TOPIC
        self.state = state
        self.stop = stop
        self.depth_range = depth_range
        self.depth_preview_width = depth_preview_width
        self.color_size: tuple[int, int] | None = None
        self.color_frames = 0

    def run(self) -> None:
        while not self.stop.is_set():
            connection = None
            try:
                connection = websocket.create_connection(self.url, timeout=4)
                connection.settimeout(0.5)
                connection.send(
                    json.dumps(
                        {
                            "op": "subscribe",
                            "id": f"head_camera_viewer_{self.kind}",
                            "topic": self.topic,
                            "type": "sensor_msgs/CompressedImage",
                            "queue_length": 1,
                        }
                    )
                )
                self.color_size = None
                self.color_frames = 0
                self.state.set_connection(True)
                while not self.stop.is_set():
                    try:
                        raw = connection.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    if not raw:
                        raise ConnectionError("ROSBridge closed the connection")
                    packet = json.loads(raw)
                    if packet.get("op") == "status" and packet.get("level") == "error":
                        raise RuntimeError(str(packet.get("msg", "ROSBridge subscription error")))
                    if packet.get("op") != "publish" or packet.get("topic") != self.topic:
                        continue
                    try:
                        body = packet["msg"]
                        if self.kind == "color":
                            jpeg, width, height, source_bytes, valid_percent, preview_size = (
                                decode_color(
                                    body,
                                    self.color_size if self.color_frames % 90 else None,
                                )
                            )
                            self.color_size = (width, height)
                            self.color_frames += 1
                        else:
                            jpeg, width, height, source_bytes, valid_percent, preview_size = (
                                decode_depth(body, *self.depth_range, self.depth_preview_width)
                            )
                        self.state.update(
                            jpeg,
                            width,
                            height,
                            source_bytes,
                            str(body.get("header", {}).get("frame_id", "")),
                            valid_percent,
                            preview_size,
                        )
                    except (KeyError, TypeError, ValueError, cv2.error) as error:
                        self.state.note_bad_frame(error)
            except Exception as error:
                if not self.stop.is_set():
                    self.state.set_connection(False, f"{type(error).__name__}: {error}")
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except OSError:
                        pass
                if self.stop.is_set():
                    self.state.set_connection(False)
            self.stop.wait(1.0)


class ViewerServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class ViewerHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, _format: str, *_args: object) -> None:
        pass

    def reply(self, code: int, content_type: str, body: bytes) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path == "/api/status":
            body = json.dumps(self.server.snapshot(), separators=(",", ":")).encode()
            self.reply(200, "application/json; charset=utf-8", body)
            return
        if path in ("/stream/color.mjpg", "/stream/depth.mjpg"):
            kind = "color" if "color" in path else "depth"
            self.stream(self.server.streams[kind])
            return
        assets = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/app.css": ("app.css", "text/css; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
        }
        if path not in assets:
            self.reply(404, "text/plain; charset=utf-8", b"Not found")
            return
        name, content_type = assets[path]
        try:
            self.reply(200, content_type, (WEB_ROOT / name).read_bytes())
        except OSError:
            self.reply(500, "text/plain; charset=utf-8", b"Viewer asset unavailable")

    def stream(self, state: StreamState) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        sequence = 0
        try:
            while not self.server.stop_event.is_set():
                latest = state.latest_after(sequence, self.server.stop_event)
                if latest is None:
                    continue
                sequence, jpeg = latest
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--jetson-host",
        default=os.environ.get("WA1_CAMERA_HOST", "192.168.217.100"),
        help="Jetson address reachable from this PC (default: 192.168.217.100)",
    )
    parser.add_argument("--rosbridge-port", type=int, default=9090)
    parser.add_argument("--ui-port", type=int, default=8767)
    parser.add_argument("--depth-min-m", type=float, default=0.3)
    parser.add_argument("--depth-max-m", type=float, default=5.0)
    parser.add_argument("--depth-preview-width", type=int, default=800)
    args = parser.parse_args()
    if not 1 <= args.rosbridge_port <= 65535 or not 1 <= args.ui_port <= 65535:
        parser.error("ports must be in [1, 65535]")
    if not 0 < args.depth_min_m < args.depth_max_m <= 65:
        parser.error("depth range must satisfy 0 < min < max <= 65 metres")
    if not 320 <= args.depth_preview_width <= 1920:
        parser.error("--depth-preview-width must be in [320, 1920]")
    if not args.jetson_host or any(ch in args.jetson_host for ch in "/:@ "):
        parser.error("--jetson-host must be an IP address or hostname")

    cv2.setNumThreads(1)
    streams = {"color": StreamState(), "depth": StreamState()}
    stop = threading.Event()
    server = ViewerServer(("127.0.0.1", args.ui_port), ViewerHandler)
    server.streams = streams
    server.stop_event = stop

    def snapshot() -> dict:
        return {
            "robot_host": args.jetson_host,
            "rosbridge_port": args.rosbridge_port,
            "depth_min_m": args.depth_min_m,
            "depth_max_m": args.depth_max_m,
            "streams": {kind: state.snapshot() for kind, state in streams.items()},
        }

    server.snapshot = snapshot
    receivers = [
        CameraReceiver(
            kind, args.jetson_host, args.rosbridge_port, streams[kind], stop,
            (args.depth_min_m, args.depth_max_m), args.depth_preview_width,
        )
        for kind in streams
    ]
    for receiver in receivers:
        receiver.start()
    print(f"Head camera viewer: http://127.0.0.1:{args.ui_port}", flush=True)
    print(
        f"ROSBridge: ws://{args.jetson_host}:{args.rosbridge_port} "
        "(color + aligned depth, read only)",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for state in streams.values():
            state.wake()
        server.server_close()
        for receiver in receivers:
            receiver.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
