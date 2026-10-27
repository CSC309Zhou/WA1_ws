"use strict";

const COLORS = {
  head: "#f1e3ae", left: "#ffa669", right: "#6ee1df",
  x: "#fa7979", y: "#8ad97f", z: "#75a9ff",
  grid: "#303236", quiet: "#a7a9ad"
};
const NAMES = {head: "头显", left: "左手柄", right: "右手柄"};
const spatial = document.getElementById("spatial");
// At zoom=1 the shorter canvas side covers 1.25 m; frame data never changes the scale.
const METERS_PER_SHORT_SIDE = 1.25;
const camera = {yaw: -0.72, pitch: 0.62, zoom: 1, panX: 0, panY: 0};
const trails = {head: [], left: [], right: []};
let frame = null;
let originEpoch = -1;
let lastSequence = -1;
let streamOpen = false;
let dragMode = null;
let dragPointerId = null;
let dragX = 0;
let dragY = 0;
let lastDraw = 0;

function clearTrails() {
  for (const key of Object.keys(trails)) trails[key] = [];
}

function formatNumber(value, digits = 3) {
  return Number.isFinite(value) ? value.toFixed(digits) : "—";
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function setConnection(kind, label) {
  const element = document.getElementById("connection");
  element.className = `connection ${kind}`;
  element.lastElementChild.textContent = label;
}

function updateReadout(key, pose) {
  const card = document.getElementById(`${key}-card`);
  if (!pose) {
    card.querySelector(".position").textContent = "暂无有效位姿";
    card.querySelector(".quaternion").textContent = "—";
    card.querySelector(".extra").textContent = "—";
    return;
  }
  const position = pose.position.map(value => formatNumber(value));
  const raw = pose.raw_pose.map(value => formatNumber(value));
  card.querySelector(".position").textContent = `X ${position[0]} · Y ${position[1]} · Z ${position[2]} m`;
  card.querySelector(".quaternion").textContent =
    `原始 XYZ [${raw.slice(0, 3).join(", ")}] · qXYZW [${raw.slice(3).join(", ")}]`;
  const status = pose.status === -1 ? "SDK 未提供" : String(pose.status);
  card.querySelector(".extra").textContent = `跟踪状态 ${status}`;
}

function updateControllerInputs(key, input) {
  const panel = document.getElementById(`${key}-controls`);
  for (const item of panel.querySelectorAll(".button-state")) {
    const value = input?.[item.dataset.input];
    item.classList.toggle("is-active", value === true);
    item.querySelector("strong").textContent =
      typeof value === "boolean" ? (value ? "按下" : "松开") : "—";
  }
  for (const item of panel.querySelectorAll(".analog")) {
    const value = input?.[item.dataset.input];
    item.querySelector("strong").textContent = formatNumber(value);
    item.querySelector(".analog-fill").style.width =
      `${Number.isFinite(value) ? clamp(value, 0, 1) * 100 : 0}%`;
  }
  const x = input?.axis_x;
  const y = input?.axis_y;
  panel.querySelector(".joystick-value").textContent = `X ${formatNumber(x)} · Y ${formatNumber(y)}`;
  const dot = panel.querySelector(".joystick-pad i");
  dot.style.left = `${50 + (Number.isFinite(x) ? clamp(x, -1, 1) * 32 : 0)}%`;
  dot.style.top = `${50 - (Number.isFinite(y) ? clamp(y, -1, 1) * 32 : 0)}%`;
}

function updateUi() {
  if (!frame) return;
  if (!streamOpen) setConnection("stale", "页面数据连接中断");
  else if (frame.fresh) setConnection("live", "PICO4 帧更新中");
  else setConnection("stale", "等待新位姿帧");
  document.getElementById("connection").title = frame.device_id || "";
  document.getElementById("rate").textContent =
    frame.receive_rate_hz > 0 ? `${formatNumber(frame.receive_rate_hz, 1)} Hz` : "—";
  document.getElementById("age").textContent = frame.frame_age_ms === null
    ? "—" : `${formatNumber(frame.frame_age_ms, 0)} ms`;
  document.getElementById("battery").textContent =
    Number.isFinite(frame.battery) ? `${frame.battery}%` : "—";
  document.getElementById("axis-map").textContent =
    frame.axis_map ? frame.axis_map.join(" · ") : "—";
  document.getElementById("recenter").disabled = frame.sequence === 0;
  for (const key of Object.keys(NAMES)) updateReadout(key, frame.poses?.[key]);
  for (const key of ["left", "right"]) updateControllerInputs(key, frame.inputs?.[key]);
}

function onFrame(next) {
  if (next.origin_epoch !== originEpoch) {
    originEpoch = next.origin_epoch;
    clearTrails();
  }
  if (next.sequence !== lastSequence && next.fresh) {
    for (const key of Object.keys(NAMES)) {
      const pose = next.poses?.[key];
      if (pose) {
        trails[key].push(pose.position.slice());
        if (trails[key].length > 80) trails[key].shift();
      }
    }
    lastSequence = next.sequence;
  }
  frame = next;
  updateUi();
}

function canvasContext(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(canvas.clientWidth));
  const height = Math.max(1, Math.round(canvas.clientHeight));
  if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  return {ctx, width, height};
}

function spatialProject(point, width, height, scale) {
  const [x, y, z] = point;
  const c = Math.cos(camera.yaw), s = Math.sin(camera.yaw);
  const across = c * x + s * y;
  const depth = -s * x + c * y;
  return [width * 0.5 + camera.panX + across * scale,
    height * 0.62 + camera.panY - (depth * Math.sin(camera.pitch) + z * Math.cos(camera.pitch)) * scale];
}

function line(ctx, a, b, color, width = 1, alpha = 1) {
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(a[0], a[1]);
  ctx.lineTo(b[0], b[1]);
  ctx.stroke();
  ctx.restore();
}

function arrow(ctx, a, b, color, label = "") {
  line(ctx, a, b, color, 2.2);
  const angle = Math.atan2(b[1] - a[1], b[0] - a[0]);
  const size = 6;
  const left = [b[0] - size * Math.cos(angle - 0.48), b[1] - size * Math.sin(angle - 0.48)];
  const right = [b[0] - size * Math.cos(angle + 0.48), b[1] - size * Math.sin(angle + 0.48)];
  line(ctx, left, b, color, 2.2);
  line(ctx, right, b, color, 2.2);
  if (label) {
    ctx.fillStyle = color;
    ctx.font = "11px system-ui";
    ctx.fillText(label, b[0] + 5, b[1] - 5);
  }
}

function drawTrail(ctx, points, project, color) {
  if (!document.getElementById("show-trails").checked || points.length < 2) return;
  for (let i = 1; i < points.length; i++) {
    line(ctx, project(points[i - 1]), project(points[i]), color, 1.8, 0.15 + 0.55 * i / points.length);
  }
}

function drawPose(ctx, pose, project, key) {
  if (!pose) return;
  const center = project(pose.position);
  const axes = ["x", "y", "z"];
  for (let i = 0; i < 3; i++) {
    const end = pose.position.map((value, j) => value + pose.basis[i][j] * 0.09);
    arrow(ctx, center, project(end), COLORS[axes[i]]);
  }
  ctx.beginPath();
  ctx.arc(center[0], center[1], 6.5, 0, 2 * Math.PI);
  ctx.fillStyle = COLORS[key];
  ctx.fill();
  ctx.strokeStyle = "#0c0d0f";
  ctx.lineWidth = 2;
  ctx.stroke();
  ctx.fillStyle = COLORS[key];
  ctx.font = "600 12px system-ui";
  ctx.fillText(NAMES[key], center[0] + 9, center[1] - 9);
}

function drawSpatial() {
  const {ctx, width, height} = canvasContext(spatial);
  const scale = Math.min(width, height) / METERS_PER_SHORT_SIDE * camera.zoom;
  const project = point => spatialProject(point, width, height, scale);
  if (document.getElementById("show-world").checked) {
    const span = 1.0;
    const gridStep = 0.1;
    for (let value = -span; value <= span + 0.001; value += gridStep) {
      line(ctx, project([-span, value, 0]), project([span, value, 0]), COLORS.grid, 0.8, 0.75);
      line(ctx, project([value, -span, 0]), project([value, span, 0]), COLORS.grid, 0.8, 0.75);
    }
    const origin = project([0, 0, 0]);
    arrow(ctx, origin, project([0.18, 0, 0]), COLORS.x, "+X 前");
    arrow(ctx, origin, project([0, 0.18, 0]), COLORS.y, "+Y 左");
    arrow(ctx, origin, project([0, 0, 0.18]), COLORS.z, "+Z 上");
  }
  for (const key of Object.keys(NAMES)) {
    if (key === "head" && !document.getElementById("show-head").checked) continue;
    drawTrail(ctx, trails[key], project, COLORS[key]);
    drawPose(ctx, frame?.poses?.[key], project, key);
  }
  if (!frame?.poses?.head) {
    ctx.fillStyle = COLORS.quiet;
    ctx.font = "14px system-ui";
    ctx.textAlign = "center";
    ctx.fillText("等待 /pico4/frame 位姿数据", width * 0.5, height * 0.47);
    ctx.textAlign = "start";
  }
}

function draw(now) {
  if (now - lastDraw >= 50) {
    drawSpatial();
    lastDraw = now;
  }
  requestAnimationFrame(draw);
}

const events = new EventSource("/events");
events.onopen = () => { streamOpen = true; updateUi(); };
events.onmessage = event => {
  streamOpen = true;
  try { onFrame(JSON.parse(event.data)); }
  catch (error) { console.error("Invalid viewer state", error); }
};
events.onerror = () => { streamOpen = false; updateUi(); };

document.getElementById("recenter").addEventListener("click", async () => {
  const response = await fetch("/api/recenter", {method: "POST"});
  if (!response.ok) console.warn("Cannot recenter before receiving a frame");
});

spatial.addEventListener("pointerdown", event => {
  if (event.button !== 0 && event.button !== 2) return;
  dragMode = event.button === 2 || event.shiftKey ? "pan" : "orbit";
  dragPointerId = event.pointerId;
  dragX = event.clientX;
  dragY = event.clientY;
  spatial.setPointerCapture(event.pointerId);
});
spatial.addEventListener("pointermove", event => {
  if (dragMode === null || event.pointerId !== dragPointerId) return;
  const dx = event.clientX - dragX;
  const dy = event.clientY - dragY;
  if (dragMode === "pan") {
    camera.panX += dx;
    camera.panY += dy;
  } else {
    camera.yaw += dx * 0.008;
    camera.pitch = clamp(camera.pitch + dy * 0.005, 0.1, 1.4);
  }
  dragX = event.clientX;
  dragY = event.clientY;
});
function stopDrag(event) {
  if (event.pointerId !== dragPointerId) return;
  dragMode = null;
  dragPointerId = null;
}
spatial.addEventListener("pointerup", stopDrag);
spatial.addEventListener("pointercancel", stopDrag);
spatial.addEventListener("lostpointercapture", stopDrag);
spatial.addEventListener("contextmenu", event => event.preventDefault());
spatial.addEventListener("wheel", event => {
  event.preventDefault();
  const pixels = event.deltaY * (event.deltaMode === 1 ? 16 :
    event.deltaMode === 2 ? spatial.clientHeight : 1);
  const previousZoom = camera.zoom;
  camera.zoom = clamp(previousZoom * Math.exp(-pixels * 0.0015), 0.25, 8);
  const factor = camera.zoom / previousZoom;
  const rect = spatial.getBoundingClientRect();
  camera.panX += (event.clientX - rect.left - rect.width * 0.5 - camera.panX) * (1 - factor);
  camera.panY += (event.clientY - rect.top - rect.height * 0.62 - camera.panY) * (1 - factor);
}, {passive: false});

requestAnimationFrame(draw);
