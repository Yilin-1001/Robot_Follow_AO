import "./styles.css";

type Point = [number, number, number];

type SafeCommand = {
  vx: number;
  vy: number;
  omega: number;
};

type ZoneState = {
  point_count: number;
  min_distance_m: number | null;
  occupied: boolean;
  blocked: boolean;
};

type ZoneBox = {
  x_min: number;
  x_max: number;
  y_min: number;
  y_max: number;
};

type VisualizationPayload = {
  frame_id: string;
  radar_valid: boolean;
  radar_age_s: number;
  raw_count: number;
  filtered_count: number;
  points: Point[];
  zones: Record<string, ZoneState>;
  zone_bounds: Record<string, ZoneBox>;
  decision: {
    kind: string;
    reason: string;
    safe_command: SafeCommand;
  };
};

type ServerMessage =
  | { type: "frame"; payload: VisualizationPayload }
  | { type: "status"; payload: { level: string; message: string } }
  | { type: "error"; payload: { message: string } };

const canvasQuery = document.querySelector<HTMLCanvasElement>("#bev");
if (!canvasQuery) throw new Error("missing canvas");
const canvas: HTMLCanvasElement = canvasQuery;
const context = canvasQuery.getContext("2d");
if (!context) throw new Error("missing canvas context");
const ctx: CanvasRenderingContext2D = context;

const defaultZoneBoxes: Record<string, ZoneBox> = {
  front: { x_min: 0.2, x_max: 1.0, y_min: -0.35, y_max: 0.35 },
  left: { x_min: -0.05, x_max: 0.7, y_min: 0.35, y_max: 0.75 },
  right: { x_min: -0.05, x_max: 0.7, y_min: -0.75, y_max: -0.35 },
  rear: { x_min: -0.6, x_max: -0.1, y_min: -0.35, y_max: 0.35 },
  rotation: { x_min: -0.65, x_max: 0.65, y_min: -0.75, y_max: 0.75 },
};

const viewRadiusM = 3.0;
let socket: WebSocket | null = null;
let selectedTarget: { x: number; y: number } | null = null;

function meterToPixel(x: number, y: number) {
  const scale = canvas.width / (viewRadiusM * 2);
  return { px: canvas.width / 2 - y * scale, py: canvas.height / 2 - x * scale };
}

function pixelToMeter(px: number, py: number) {
  const scale = canvas.width / (viewRadiusM * 2);
  return { x: (canvas.height / 2 - py) / scale, y: (canvas.width / 2 - px) / scale };
}

function drawGrid() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#020617";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = "#1f2937";
  ctx.lineWidth = 1;
  for (let m = -viewRadiusM; m <= viewRadiusM; m += 0.5) {
    const a = meterToPixel(-viewRadiusM, m);
    const b = meterToPixel(viewRadiusM, m);
    ctx.beginPath();
    ctx.moveTo(a.px, a.py);
    ctx.lineTo(b.px, b.py);
    ctx.stroke();
    const c = meterToPixel(m, -viewRadiusM);
    const d = meterToPixel(m, viewRadiusM);
    ctx.beginPath();
    ctx.moveTo(c.px, c.py);
    ctx.lineTo(d.px, d.py);
    ctx.stroke();
  }
}

function drawZone(name: string, state: ZoneState | undefined, bounds: Record<string, ZoneBox>) {
  const box = bounds[name] ?? defaultZoneBoxes[name];
  if (!box) return;
  const a = meterToPixel(box.x_min, box.y_min);
  const b = meterToPixel(box.x_max, box.y_max);
  const left = Math.min(a.px, b.px);
  const top = Math.min(a.py, b.py);
  const width = Math.abs(b.px - a.px);
  const height = Math.abs(b.py - a.py);
  ctx.fillStyle = state?.blocked ? "rgba(239, 68, 68, 0.20)" : state?.occupied ? "rgba(245, 158, 11, 0.16)" : "rgba(148, 163, 184, 0.06)";
  ctx.strokeStyle = state?.blocked ? "#ef4444" : state?.occupied ? "#f59e0b" : "#64748b";
  ctx.lineWidth = name === "rotation" ? 1 : 2;
  ctx.fillRect(left, top, width, height);
  ctx.strokeRect(left, top, width, height);
}

function drawRobot(command?: SafeCommand) {
  const center = meterToPixel(0, 0);
  ctx.save();
  ctx.translate(center.px, center.py);
  ctx.fillStyle = "#e5e7eb";
  ctx.beginPath();
  ctx.moveTo(0, -22);
  ctx.lineTo(14, 16);
  ctx.lineTo(-14, 16);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
  if (command) {
    const end = meterToPixel(command.vx * 3, command.vy * 3);
    ctx.strokeStyle = "#22c55e";
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(center.px, center.py);
    ctx.lineTo(end.px, end.py);
    ctx.stroke();
  }
}

function drawPoints(points: Point[]) {
  for (const [x, y] of points) {
    const p = meterToPixel(x, y);
    const d = Math.hypot(x, y);
    ctx.fillStyle = d <= 0.45 ? "#ef4444" : d <= 1.0 ? "#f59e0b" : "#38bdf8";
    ctx.beginPath();
    ctx.arc(p.px, p.py, 2.4, 0, Math.PI * 2);
    ctx.fill();
  }
}

function drawSelectedTarget() {
  if (!selectedTarget) return;
  const p = meterToPixel(selectedTarget.x, selectedTarget.y);
  ctx.strokeStyle = "#a3e635";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(p.px, p.py, 10, 0, Math.PI * 2);
  ctx.stroke();
}

function setText(id: string, value: string) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function updatePanel(payload: VisualizationPayload) {
  setText("status", payload.radar_valid ? "radar valid" : "radar invalid");
  setText("frame-id", payload.frame_id || "-");
  setText("radar-age", `${payload.radar_age_s} s`);
  setText("raw-count", String(payload.raw_count));
  setText("filtered-count", String(payload.filtered_count));
  setText("decision-kind", payload.decision.kind);
  setText("decision-reason", payload.decision.reason);
  const cmd = payload.decision.safe_command;
  setText("safe-command", `vx ${cmd.vx}, vy ${cmd.vy}, omega ${cmd.omega}`);
  setText("selected-target", selectedTarget ? `x ${selectedTarget.x.toFixed(2)}, y ${selectedTarget.y.toFixed(2)}` : "none");
  const root = document.getElementById("zones");
  if (!root) return;
  root.innerHTML = "";
  for (const name of ["front", "left", "right", "rear", "rotation"]) {
    const zone = payload.zones[name];
    const status = zone?.blocked ? "blocked" : zone?.occupied ? "occupied" : "clear";
    const div = document.createElement("div");
    div.className = `zone ${status}`;
    div.innerHTML = `<strong>${name}</strong><span>${status}</span><span>points ${zone?.point_count ?? "-"}</span><span>min ${zone?.min_distance_m ?? "-"} m</span>`;
    root.appendChild(div);
  }
}

function render(payload: VisualizationPayload) {
  const zoneBounds = payload.zone_bounds ?? defaultZoneBoxes;
  drawGrid();
  drawZone("rotation", payload.zones.rotation, zoneBounds);
  drawZone("front", payload.zones.front, zoneBounds);
  drawZone("left", payload.zones.left, zoneBounds);
  drawZone("right", payload.zones.right, zoneBounds);
  drawZone("rear", payload.zones.rear, zoneBounds);
  drawPoints(payload.points);
  drawSelectedTarget();
  drawRobot(payload.decision.safe_command);
  updatePanel(payload);
}

function connect() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${scheme}://${window.location.host}/ws`);
  socket.onopen = () => setText("status", "connected");
  socket.onmessage = (event) => {
    const message = JSON.parse(event.data) as ServerMessage;
    if (message.type === "frame") render(message.payload);
    if (message.type === "status") setText("status", message.payload.message);
    if (message.type === "error") setText("status", message.payload.message);
  };
  socket.onclose = () => {
    setText("status", "reconnecting");
    window.setTimeout(connect, 1000);
  };
}

canvas.addEventListener("pointerdown", (event) => {
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const target = pixelToMeter((event.clientX - rect.left) * scaleX, (event.clientY - rect.top) * scaleY);
  selectedTarget = target;
  setText("selected-target", `x ${target.x.toFixed(2)}, y ${target.y.toFixed(2)}`);
  socket?.send(JSON.stringify({ type: "select_target", x: target.x, y: target.y }));
});

drawGrid();
drawRobot();
connect();
