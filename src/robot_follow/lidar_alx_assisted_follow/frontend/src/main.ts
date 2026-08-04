/** LiDAR + ALX assisted-follow monitor. */

interface Point2D { x: number; y: number; }
type Point3D = [number, number, number];

interface Command {
  vx: number;
  vy: number;
  omega: number;
  reason: string;
}

interface ALXRetreatZone {
  enabled: boolean;
  start_distance_m: number;
  exit_distance_m: number;
  active: boolean;
}

interface ClearanceZone {
  body_half_length_m: number;
  body_half_width_m: number;
  front_rear_hard_clearance_m: number;
  side_hard_clearance_m: number;
  slowdown_clearance_m: number;
  obstacle_x_min_m: number;
  obstacle_x_max_m: number;
  obstacle_abs_y_max_m: number;
  automatic_escape: { enabled: boolean; probe_distance_m: number; max_duration_s: number };
  clearance_recovery: { enabled: boolean; active: boolean; exit_margin_m: number; max_duration_s: number };
}

interface StatusFrame {
  version: number;
  cloud_sequence: number;
  frame_id: string;
  points: Point3D[];
  point_height_range?: { min_m: number; max_m: number; offset_m: number };
  target_exclusion?: { kind: string; center_xy: [number, number]; radius_m: number } | null;
  lateral_avoidance_zone?: {
    kind: string;
    start_xy: [number, number];
    end_xy: [number, number];
    half_width_m: number;
  } | null;
  alx_retreat_zone?: ALXRetreatZone;
  clearance_zone?: ClearanceZone;
  alx?: {
    body_xy: [number, number] | null;
    age_s: number | null;
    stamp_s: number | null;
    offset_ms: number | null;
    synchronized: boolean;
  };
  wristband?: { state: string; age_ms: number | null; fresh: boolean };
  cloud?: { age_ms: number | null; fresh: boolean };
  lifecycle_state: string;
  follow_enabled: boolean;
  motion_state: string;
  start?: { state: string; reason: string | null; timestamp_s: number | null };
  stop?: { requested: boolean; kind: string | null; reason: string | null; strategy_reason?: string | null };
  command?: { requested: Command; dispatched: Command | null };
  output_mode?: string;
  calibration_status: string;
}

type ServerMessage =
  | { type: "frame"; payload: StatusFrame }
  | { type: "error"; payload: { code: string; message: string } }
  | { type: "ack" };

const GRID_SPACING_M = 1.0;
const GRID_COLOR = "#21262d";
const POINT_COLOR = "#9fdcff";
const ALX_COLOR = "#3fb950";
const EXCLUSION_COLOR = "#bc8cff";
const AVOIDANCE_ZONE_COLOR = "#f0b429";
const ALX_RETREAT_COLOR = "#ff9f43";
const HARD_CLEARANCE_COLOR = "#f85149";
const SLOWDOWN_CLEARANCE_COLOR = "#f0b429";
const OBSTACLE_WINDOW_COLOR = "#58a6ff";
const HEIGHT_LEGEND_WIDTH_PX = 104;
const HEIGHT_LEGEND_HEIGHT_PX = 8;
const MAX_SCALE = 80;
const MIN_SCALE = 28;
const VIEWPORT_PADDING_PX = 28;
const MAX_ALX_TRAIL_POINTS = 40;

interface Viewport { centerX: number; centerY: number; scale: number; }

class LiDARMonitor {
  private ws: WebSocket | null = null;
  private readonly canvas = document.getElementById("lidar-canvas") as HTMLCanvasElement;
  private readonly ctx = this.canvas.getContext("2d")!;
  private latestFrame: StatusFrame | null = null;
  private reconnectAttempts = 0;
  private alxTrail: Point2D[] = [];

  private readonly connectionState = document.getElementById("connection-state")!;
  private readonly followEnabled = document.getElementById("follow-enabled")!;
  private readonly motionState = document.getElementById("motion-state")!;
  private readonly wristbandState = document.getElementById("wristband-state")!;
  private readonly stopReason = document.getElementById("stop-reason")!;
  private readonly requestedVx = document.getElementById("requested-vx")!;
  private readonly requestedVy = document.getElementById("requested-vy")!;
  private readonly requestedOmega = document.getElementById("requested-omega")!;
  private readonly dispatchedVx = document.getElementById("dispatched-vx")!;
  private readonly dispatchedVy = document.getElementById("dispatched-vy")!;
  private readonly dispatchedOmega = document.getElementById("dispatched-omega")!;
  private readonly startBtn = document.getElementById("start-btn") as HTMLButtonElement;
  private readonly stopBtn = document.getElementById("stop-btn") as HTMLButtonElement;
  private readonly errorMessage = document.getElementById("error-message")!;

  constructor() {
    this.startBtn.addEventListener("click", () => this.sendControl("start_follow"));
    this.stopBtn.addEventListener("click", () => this.sendControl("stop_follow"));
    window.addEventListener("resize", () => { this.resizeCanvas(); this.render(); });
    this.resizeCanvas();
    this.setConnection(false);
    this.connect();
  }

  private connect(): void {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    this.ws = new WebSocket(`${protocol}//${window.location.host}/ws`);
    this.ws.onopen = () => {
      this.reconnectAttempts = 0;
      this.setConnection(true);
      this.clearError();
    };
    this.ws.onmessage = (event: MessageEvent) => {
      try { this.handleMessage(JSON.parse(event.data) as ServerMessage); }
      catch (error) { console.error("Failed to parse server message", error); }
    };
    this.ws.onclose = () => {
      this.setConnection(false);
      const delay = Math.min(1000 * 2 ** ++this.reconnectAttempts, 10000);
      window.setTimeout(() => this.connect(), delay);
    };
  }

  private setConnection(connected: boolean): void {
    this.connectionState.textContent = connected ? "CONNECTED" : "CONNECTING...";
    this.connectionState.className = `role-badge ${connected ? "connected" : "connecting"}`;
    this.startBtn.disabled = !connected;
    this.stopBtn.disabled = !connected;
  }

  private handleMessage(msg: ServerMessage): void {
    if (msg.type === "frame") {
      this.latestFrame = msg.payload;
      this.updateALXTrail(msg.payload);
      this.updateDisplay();
      this.render();
    } else if (msg.type === "error") {
      this.showError(msg.payload.code, msg.payload.message);
    } else {
      this.clearError();
    }
  }

  private updateDisplay(): void {
    const frame = this.latestFrame;
    if (!frame) return;
    this.followEnabled.textContent = frame.follow_enabled ? "ENABLED" : "DISABLED";
    this.followEnabled.style.color = frame.follow_enabled ? "#3fb950" : "#8b949e";
    this.motionState.textContent = frame.motion_state;

    const wristband = frame.wristband;
    this.wristbandState.textContent = wristband
      ? `${wristband.state}${wristband.fresh ? "" : " (STALE)"}` : "UNKNOWN";
    this.wristbandState.style.color = wristband?.fresh && wristband.state === "ON" ? "#3fb950" : "#d29922";

    const stop = frame.stop;
    this.stopReason.textContent = stop?.requested
      ? `${stop.kind ?? "STOP"}: ${stop.reason ?? "unknown"}`
      : stop?.strategy_reason ?? "None";
    this.stopReason.style.color = stop?.requested ? "#f85149" : "";

    this.setCommand(this.requestedVx, this.requestedVy, this.requestedOmega, frame.command?.requested);
    this.setCommand(this.dispatchedVx, this.dispatchedVy, this.dispatchedOmega, frame.command?.dispatched);
  }

  private setCommand(vx: HTMLElement, vy: HTMLElement, omega: HTMLElement, command?: Command | null): void {
    vx.textContent = command?.vx.toFixed(2) ?? "0.00";
    vy.textContent = command?.vy.toFixed(2) ?? "0.00";
    omega.textContent = command?.omega.toFixed(2) ?? "0.00";
  }

  private updateALXTrail(frame: StatusFrame): void {
    const alx = frame.alx;
    if (!alx?.body_xy || !alx.synchronized) { this.alxTrail = []; return; }
    const [x, y] = alx.body_xy;
    const previous = this.alxTrail[this.alxTrail.length - 1];
    if (!previous || previous.x !== x || previous.y !== y) this.alxTrail.push({ x, y });
    if (this.alxTrail.length > MAX_ALX_TRAIL_POINTS) this.alxTrail.shift();
  }

  private resizeCanvas(): void {
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.max(1, Math.floor(rect.width));
    this.canvas.height = Math.max(1, Math.floor(rect.height));
  }

  private getViewport(): Viewport {
    return { centerX: this.canvas.width / 2, centerY: this.canvas.height / 2, scale: this.getVisibleScale() };
  }

  private getVisibleScale(): number {
    const bounds = { minX: -2, maxX: 2, minY: -2, maxY: 2 };
    const include = (x: number, y: number): void => {
      if (Number.isFinite(x) && Number.isFinite(y)) {
        bounds.minX = Math.min(bounds.minX, x); bounds.maxX = Math.max(bounds.maxX, x);
        bounds.minY = Math.min(bounds.minY, y); bounds.maxY = Math.max(bounds.maxY, y);
      }
    };
    this.latestFrame?.points.forEach((point) => include(point[0], point[1]));
    const alx = this.latestFrame?.alx?.body_xy;
    if (alx) include(alx[0], alx[1]);
    const retreat = this.latestFrame?.alx_retreat_zone;
    if (retreat?.enabled) {
      include(retreat.exit_distance_m, retreat.exit_distance_m);
      include(-retreat.exit_distance_m, -retreat.exit_distance_m);
    }
    const clearance = this.latestFrame?.clearance_zone;
    if (clearance) {
      include(clearance.obstacle_x_min_m, clearance.obstacle_abs_y_max_m);
      include(clearance.obstacle_x_max_m, -clearance.obstacle_abs_y_max_m);
      if (clearance.automatic_escape.enabled) include(
        -clearance.body_half_length_m - clearance.automatic_escape.probe_distance_m,
        clearance.body_half_width_m,
      );
    }
    this.alxTrail.forEach((point) => include(point.x, point.y));
    const width = Math.max(1, this.canvas.width - VIEWPORT_PADDING_PX * 2);
    const height = Math.max(1, this.canvas.height - VIEWPORT_PADDING_PX * 2);
    return Math.max(MIN_SCALE, Math.min(MAX_SCALE, Math.min(width / (Math.max(Math.abs(bounds.minX), Math.abs(bounds.maxX)) * 2), height / (Math.max(Math.abs(bounds.minY), Math.abs(bounds.maxY)) * 2))));
  }

  private toCanvas(point: Point2D, viewport: Viewport): Point2D {
    return { x: viewport.centerX - point.y * viewport.scale, y: viewport.centerY - point.x * viewport.scale };
  }

  private render(): void {
    this.resizeCanvas();
    this.ctx.fillStyle = "#0b0e14";
    this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    this.drawGrid();
    this.drawALXTrail();
    if (this.latestFrame) {
      if (this.latestFrame.clearance_zone) {
        this.drawObstacleWindow(this.latestFrame.clearance_zone);
        this.drawClearanceZones(this.latestFrame.clearance_zone);
        this.drawEscapeProbe(this.latestFrame.clearance_zone);
      }
      if (this.latestFrame.alx_retreat_zone?.enabled) this.drawALXRetreatZone(this.latestFrame.alx_retreat_zone);
      this.drawPoints(this.latestFrame.points, this.latestFrame.point_height_range);
      if (this.latestFrame.lateral_avoidance_zone) this.drawLateralAvoidanceZone(this.latestFrame.lateral_avoidance_zone);
      if (this.latestFrame.target_exclusion) this.drawExclusion(this.latestFrame.target_exclusion);
      const alx = this.latestFrame.alx;
      if (alx?.body_xy) this.drawALX(alx.body_xy, alx.synchronized);
      this.drawHeightLegend(this.latestFrame.point_height_range);
    }
    this.drawRobotHeading();
  }

  private drawGrid(): void {
    const viewport = this.getViewport();
    const step = GRID_SPACING_M * viewport.scale;
    this.ctx.strokeStyle = GRID_COLOR; this.ctx.lineWidth = 1;
    for (let x = viewport.centerX % step; x < this.canvas.width; x += step) { this.ctx.beginPath(); this.ctx.moveTo(x, 0); this.ctx.lineTo(x, this.canvas.height); this.ctx.stroke(); }
    for (let y = viewport.centerY % step; y < this.canvas.height; y += step) { this.ctx.beginPath(); this.ctx.moveTo(0, y); this.ctx.lineTo(this.canvas.width, y); this.ctx.stroke(); }
  }

  private drawPoints(
    points: Point3D[],
    heightRange?: { min_m: number; max_m: number; offset_m: number },
  ): void {
    const viewport = this.getViewport();
    points.forEach((point) => {
      const p = this.toCanvas({ x: point[0], y: point[1] }, viewport);
      this.ctx.fillStyle = heightRange
        ? this.heightColor(point[2] + heightRange.offset_m, heightRange)
        : POINT_COLOR;
      this.ctx.beginPath(); this.ctx.arc(p.x, p.y, 1, 0, 2 * Math.PI); this.ctx.fill();
    });
  }

  private heightColor(
    heightM: number,
    range: { min_m: number; max_m: number; offset_m: number },
  ): string {
    const span = Math.max(Number.EPSILON, range.max_m - range.min_m);
    const ratio = Math.min(1, Math.max(0, (heightM - range.min_m) / span));
    // Red (low) -> yellow/green -> cyan -> blue (high).
    return `hsl(${220 * ratio}, 92%, 62%)`;
  }

  private drawHeightLegend(range?: { min_m: number; max_m: number; offset_m: number }): void {
    if (!range) return;
    const x = 14; const y = this.canvas.height - 42;
    const gradient = this.ctx.createLinearGradient(x, y, x + HEIGHT_LEGEND_WIDTH_PX, y);
    for (const ratio of [0, 0.25, 0.5, 0.75, 1]) gradient.addColorStop(ratio, this.heightColor(range.min_m + (range.max_m - range.min_m) * ratio, range));
    this.ctx.save();
    this.ctx.fillStyle = "rgba(11, 14, 20, 0.82)";
    this.ctx.fillRect(x - 6, y - 17, HEIGHT_LEGEND_WIDTH_PX + 12, 35);
    this.ctx.fillStyle = gradient;
    this.ctx.fillRect(x, y, HEIGHT_LEGEND_WIDTH_PX, HEIGHT_LEGEND_HEIGHT_PX);
    this.ctx.font = "10px monospace";
    this.ctx.fillStyle = "#c9d1d9";
    this.ctx.fillText("POINT HEIGHT", x, y - 6);
    this.ctx.fillText(`${range.min_m.toFixed(2)} m`, x, y + 19);
    const maxLabel = `${range.max_m.toFixed(2)} m`;
    this.ctx.fillText(maxLabel, x + HEIGHT_LEGEND_WIDTH_PX - this.ctx.measureText(maxLabel).width, y + 19);
    this.ctx.restore();
  }

  private drawExclusion(exclusion: { center_xy: [number, number]; radius_m: number }): void {
    const viewport = this.getViewport(); const center = this.toCanvas({ x: exclusion.center_xy[0], y: exclusion.center_xy[1] }, viewport);
    this.ctx.strokeStyle = EXCLUSION_COLOR; this.ctx.lineWidth = 2; this.ctx.setLineDash([5, 4]);
    this.ctx.beginPath(); this.ctx.arc(center.x, center.y, exclusion.radius_m * viewport.scale, 0, 2 * Math.PI); this.ctx.stroke(); this.ctx.setLineDash([]);
  }

  private drawALXRetreatZone(zone: ALXRetreatZone): void {
    const viewport = this.getViewport();
    const center = this.toCanvas({ x: 0, y: 0 }, viewport);
    this.ctx.save();
    this.ctx.strokeStyle = ALX_RETREAT_COLOR;
    this.ctx.lineWidth = zone.active ? 2.5 : 1.5;
    this.ctx.setLineDash(zone.active ? [] : [6, 4]);
    this.ctx.beginPath(); this.ctx.arc(center.x, center.y, zone.start_distance_m * viewport.scale, 0, 2 * Math.PI); this.ctx.stroke();
    this.ctx.globalAlpha = 0.55;
    this.ctx.beginPath(); this.ctx.arc(center.x, center.y, zone.exit_distance_m * viewport.scale, 0, 2 * Math.PI); this.ctx.stroke();
    this.ctx.globalAlpha = 1;
    this.ctx.setLineDash([]);
    this.ctx.font = "10px monospace";
    this.ctx.fillStyle = ALX_RETREAT_COLOR;
    this.ctx.fillText(`ALX RETREAT <${zone.start_distance_m.toFixed(2)} m`, center.x + 8, center.y - zone.start_distance_m * viewport.scale - 6);
    this.ctx.restore();
  }

  private drawObstacleWindow(zone: ClearanceZone): void {
    const viewport = this.getViewport();
    const corners = [
      { x: zone.obstacle_x_min_m, y: -zone.obstacle_abs_y_max_m },
      { x: zone.obstacle_x_max_m, y: -zone.obstacle_abs_y_max_m },
      { x: zone.obstacle_x_max_m, y: zone.obstacle_abs_y_max_m },
      { x: zone.obstacle_x_min_m, y: zone.obstacle_abs_y_max_m },
    ].map((point) => this.toCanvas(point, viewport));
    this.ctx.save();
    this.ctx.strokeStyle = OBSTACLE_WINDOW_COLOR;
    this.ctx.globalAlpha = 0.5;
    this.ctx.lineWidth = 1;
    this.ctx.setLineDash([3, 5]);
    this.ctx.beginPath(); this.ctx.moveTo(corners[0].x, corners[0].y);
    corners.slice(1).forEach((corner) => this.ctx.lineTo(corner.x, corner.y));
    this.ctx.closePath(); this.ctx.stroke();
    this.ctx.setLineDash([]);
    this.ctx.font = "10px monospace";
    this.ctx.fillStyle = OBSTACLE_WINDOW_COLOR;
    this.ctx.fillText("APF WINDOW", corners[0].x + 5, corners[0].y + 13);
    this.ctx.restore();
  }

  private drawClearanceZones(zone: ClearanceZone): void {
    const viewport = this.getViewport();
    const hardBoundary = this.clearanceBoundary(zone, (dx, dy) => (
      dy > dx ? zone.side_hard_clearance_m : zone.front_rear_hard_clearance_m
    ), viewport);
    const slowdownBoundary = this.clearanceBoundary(zone, () => zone.slowdown_clearance_m, viewport);
    this.ctx.save();
    this.drawClosedPath(slowdownBoundary);
    this.ctx.strokeStyle = SLOWDOWN_CLEARANCE_COLOR;
    this.ctx.lineWidth = 1;
    this.ctx.setLineDash([4, 4]);
    this.ctx.stroke();
    this.ctx.setLineDash([]);
    this.drawClosedPath(hardBoundary);
    this.ctx.fillStyle = "rgba(248, 81, 73, 0.10)";
    this.ctx.fill();
    this.ctx.strokeStyle = HARD_CLEARANCE_COLOR;
    this.ctx.lineWidth = 1.75;
    this.ctx.stroke();
    const body = [
      { x: -zone.body_half_length_m, y: -zone.body_half_width_m },
      { x: zone.body_half_length_m, y: -zone.body_half_width_m },
      { x: zone.body_half_length_m, y: zone.body_half_width_m },
      { x: -zone.body_half_length_m, y: zone.body_half_width_m },
    ].map((point) => this.toCanvas(point, viewport));
    this.ctx.fillStyle = "rgba(139, 148, 158, 0.40)";
    this.ctx.strokeStyle = "#c9d1d9";
    this.ctx.lineWidth = 1;
    this.ctx.beginPath(); this.ctx.moveTo(body[0].x, body[0].y);
    body.slice(1).forEach((corner) => this.ctx.lineTo(corner.x, corner.y));
    this.ctx.closePath(); this.ctx.fill(); this.ctx.stroke();
    this.ctx.font = "10px monospace";
    this.ctx.fillStyle = HARD_CLEARANCE_COLOR;
    this.ctx.fillText("HARD CLEARANCE", body[1].x + 7, body[1].y - 7);
    this.ctx.restore();
  }

  private drawEscapeProbe(zone: ClearanceZone): void {
    if (!zone.automatic_escape.enabled) return;
    const viewport = this.getViewport();
    const startX = -zone.body_half_length_m;
    const endX = startX - zone.automatic_escape.probe_distance_m;
    const halfWidth = zone.body_half_width_m;
    const corners = [
      { x: startX, y: -halfWidth }, { x: endX, y: -halfWidth },
      { x: endX, y: halfWidth }, { x: startX, y: halfWidth },
    ].map((point) => this.toCanvas(point, viewport));
    this.ctx.save();
    this.ctx.fillStyle = "rgba(255, 159, 67, 0.10)";
    this.ctx.strokeStyle = ALX_RETREAT_COLOR;
    this.ctx.lineWidth = 1.25;
    this.ctx.setLineDash([5, 3]);
    this.ctx.beginPath(); this.ctx.moveTo(corners[0].x, corners[0].y);
    corners.slice(1).forEach((corner) => this.ctx.lineTo(corner.x, corner.y));
    this.ctx.closePath(); this.ctx.fill(); this.ctx.stroke();
    this.ctx.setLineDash([]);
    this.ctx.font = "10px monospace";
    this.ctx.fillStyle = ALX_RETREAT_COLOR;
    this.ctx.fillText(`ESCAPE PROBE ${zone.automatic_escape.probe_distance_m.toFixed(2)} m`, corners[1].x + 5, corners[1].y - 6);
    this.ctx.restore();
  }

  private clearanceBoundary(
    zone: ClearanceZone,
    required: (dx: number, dy: number) => number,
    viewport: Viewport,
  ): Point2D[] {
    const points: Point2D[] = [];
    const maxRequired = Math.max(zone.front_rear_hard_clearance_m, zone.side_hard_clearance_m, zone.slowdown_clearance_m);
    const upperBound = Math.hypot(zone.body_half_length_m, zone.body_half_width_m) + maxRequired * 2 + 1;
    for (let index = 0; index < 96; index += 1) {
      const angle = (index / 96) * Math.PI * 2;
      const cos = Math.cos(angle); const sin = Math.sin(angle);
      let low = 0; let high = upperBound;
      for (let iteration = 0; iteration < 32; iteration += 1) {
        const distance = (low + high) / 2;
        const dx = Math.max(Math.abs(distance * cos) - zone.body_half_length_m, 0);
        const dy = Math.max(Math.abs(distance * sin) - zone.body_half_width_m, 0);
        if (Math.hypot(dx, dy) < required(dx, dy)) low = distance; else high = distance;
      }
      points.push(this.toCanvas({ x: high * cos, y: high * sin }, viewport));
    }
    return points;
  }

  private drawClosedPath(points: Point2D[]): void {
    this.ctx.beginPath();
    this.ctx.moveTo(points[0].x, points[0].y);
    points.slice(1).forEach((point) => this.ctx.lineTo(point.x, point.y));
    this.ctx.closePath();
  }

  private drawLateralAvoidanceZone(zone: { start_xy: [number, number]; end_xy: [number, number]; half_width_m: number }): void {
    const viewport = this.getViewport();
    const start = { x: zone.start_xy[0], y: zone.start_xy[1] };
    const end = { x: zone.end_xy[0], y: zone.end_xy[1] };
    const length = Math.hypot(end.x - start.x, end.y - start.y);
    if (length <= Number.EPSILON) return;
    const normal = { x: -(end.y - start.y) / length, y: (end.x - start.x) / length };
    const corners = [
      { x: start.x + normal.x * zone.half_width_m, y: start.y + normal.y * zone.half_width_m },
      { x: end.x + normal.x * zone.half_width_m, y: end.y + normal.y * zone.half_width_m },
      { x: end.x - normal.x * zone.half_width_m, y: end.y - normal.y * zone.half_width_m },
      { x: start.x - normal.x * zone.half_width_m, y: start.y - normal.y * zone.half_width_m },
    ].map((corner) => this.toCanvas(corner, viewport));
    this.ctx.save();
    this.ctx.fillStyle = "rgba(240, 180, 41, 0.08)";
    this.ctx.strokeStyle = AVOIDANCE_ZONE_COLOR;
    this.ctx.lineWidth = 1.5;
    this.ctx.setLineDash([6, 4]);
    this.ctx.beginPath();
    this.ctx.moveTo(corners[0].x, corners[0].y);
    corners.slice(1).forEach((corner) => this.ctx.lineTo(corner.x, corner.y));
    this.ctx.closePath();
    this.ctx.fill();
    this.ctx.stroke();
    this.ctx.setLineDash([]);
    this.ctx.font = "11px monospace";
    this.ctx.fillStyle = AVOIDANCE_ZONE_COLOR;
    this.ctx.fillText(`TARGET CORRIDOR · ${length.toFixed(2)} m`, corners[0].x + 5, corners[0].y + 14);
    this.ctx.restore();
  }

  private drawALXTrail(): void {
    if (this.alxTrail.length < 2) return;
    const viewport = this.getViewport(); this.ctx.save(); this.ctx.strokeStyle = "rgba(63, 185, 80, 0.55)"; this.ctx.lineWidth = 2; this.ctx.setLineDash([5, 4]); this.ctx.beginPath();
    this.alxTrail.forEach((point, index) => { const p = this.toCanvas(point, viewport); if (index === 0) this.ctx.moveTo(p.x, p.y); else this.ctx.lineTo(p.x, p.y); });
    this.ctx.stroke(); this.ctx.restore();
  }

  private drawALX(bodyXY: [number, number], synchronized: boolean): void {
    const point = this.toCanvas({ x: bodyXY[0], y: bodyXY[1] }, this.getViewport()); const color = synchronized ? ALX_COLOR : "#d29922";
    this.ctx.fillStyle = color; this.ctx.beginPath(); this.ctx.arc(point.x, point.y, 6, 0, 2 * Math.PI); this.ctx.fill(); this.ctx.font = "11px monospace"; this.ctx.fillText(`ALX (${bodyXY[0].toFixed(2)}, ${bodyXY[1].toFixed(2)})`, point.x + 9, point.y - 9);
  }

  private drawRobotHeading(): void {
    const viewport = this.getViewport(); this.ctx.save(); this.ctx.translate(viewport.centerX, viewport.centerY); this.ctx.fillStyle = "#8b949e"; this.ctx.strokeStyle = "#c9d1d9"; this.ctx.lineWidth = 1.25; this.ctx.beginPath(); this.ctx.moveTo(0, -18); this.ctx.lineTo(13, -2); this.ctx.lineTo(6, -2); this.ctx.lineTo(6, 15); this.ctx.lineTo(-6, 15); this.ctx.lineTo(-6, -2); this.ctx.lineTo(-13, -2); this.ctx.closePath(); this.ctx.fill(); this.ctx.stroke(); this.ctx.restore();
  }

  private sendControl(type: "start_follow" | "stop_follow"): void {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify({ type }));
  }
  private showError(code: string, message: string): void { this.errorMessage.textContent = `[${code}] ${message}`; this.errorMessage.classList.remove("hidden"); }
  private clearError(): void { this.errorMessage.classList.add("hidden"); }
}

window.addEventListener("DOMContentLoaded", () => { new LiDARMonitor(); });
