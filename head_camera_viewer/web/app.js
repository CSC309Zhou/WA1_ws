"use strict";

const byId = (id) => document.getElementById(id);
const setText = (id, value) => { byId(id).textContent = value; };

function ageText(ageMs) {
  if (ageMs === null || ageMs === undefined) return "—";
  return ageMs < 1000 ? Math.round(ageMs) + " ms" : (ageMs / 1000).toFixed(1) + " s";
}

function resolutionText(stream) {
  return stream.width && stream.height ? stream.width + " × " + stream.height : "—";
}

function updateStream(kind, stream) {
  const live = stream.state === "live";
  byId(kind + "-card").classList.toggle("is-live", live);
  setText(kind + "-fps", live ? stream.fps.toFixed(1) : "—");
  setText(kind + "-resolution", resolutionText(stream));
  setText(kind + "-footer-resolution", resolutionText(stream));
  setText(kind + "-age", ageText(stream.frame_age_ms));
  setText(kind + "-footer-state", live ? "画面正常" : stream.connected ? "等待新帧" : "未连接");
  setText(kind + "-placeholder-text", stream.connected ? "等待来自头部相机的新帧" : "等待相机连接");
  if (kind === "color") {
    setText("color-size", stream.source_kib ? stream.source_kib.toFixed(1) + " KiB" : "—");
  } else {
    setText("depth-preview-resolution", stream.preview_width ?
      stream.preview_width + " × " + stream.preview_height : "—");
    setText("depth-valid", stream.valid_percent === null ? "—" : stream.valid_percent.toFixed(1) + "%");
  }
}

function updateStatus(data) {
  const color = data.streams.color;
  const depth = data.streams.depth;
  updateStream("color", color);
  updateStream("depth", depth);
  setText("source-address", data.robot_host + ":" + data.rosbridge_port);
  setText("depth-range-label",
    "近 " + data.depth_min_m.toFixed(1) + " m · 远 " + data.depth_max_m.toFixed(1) + " m");

  const liveCount = [color, depth].filter((stream) => stream.state === "live").length;
  const anyConnected = color.connected || depth.connected;
  const state = liveCount === 2 ? "live" : liveCount === 1 ? "partial" : "waiting";
  const message = liveCount === 2 ? "双路画面正常" :
    liveCount === 1 ? "部分画面正常" : anyConnected ? "等待相机画面" : "连接中断";
  byId("global-status").dataset.state = state;
  setText("global-status-text", message);
  setText("link-state", message);
  const detail = [color.error, depth.error].find(Boolean);
  setText("link-detail", detail || (liveCount === 2 ? "彩色与对齐深度均在更新" : "等待 Jetson 传输新帧"));
}

async function refresh() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error("HTTP " + response.status);
    updateStatus(await response.json());
  } catch (error) {
    byId("global-status").dataset.state = "waiting";
    setText("global-status-text", "查看器连接中断");
    setText("link-state", "查看器连接中断");
    setText("link-detail", String(error));
  } finally {
    window.setTimeout(refresh, 600);
  }
}

refresh();
