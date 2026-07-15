/**
 * LiDAR + ALX Assisted Follow Monitor
 * Main TypeScript entry point
 */

interface Point2D {
  x: number;
  y: number;
}

// The backend serializes NumPy point clouds with ``tolist()``, so each point
// arrives as a JSON tuple-like array: [x, y, z].
type Point3D = [number, number, number];

interface StatusFrame {
  version: number;
  cloud_sequence: number;
  frame_id: string;
  points: Point3D[];
  target_points: Point3D[];
  target_gate?: {
    kind: string;
    center_xy: [number, number];
    radial_radius_m: number;
    tangential_radius_m: number;
  };
  alx?: {
    body_xy: [number, number] | null;
    age_s: number;
    synchronized: boolean;
  };
  lifecycle_state: string;
  lock_state: string;
  confidence?: {
    c_t: number;
    c_g: number;
    c_a: number;
    total: number;
  };
  stop?: {
    requested: boolean;
    kind: string | null;
    reason: string | null;
  };
  dispatched_command?: {
    vx: number;
    vy: number;
    omega: number;
    reason: string;
  };
  calibration_status: string;
}

interface RoleMessage {
  type: "role";
  payload: { role: "controller" | "observer" };
}

interface FrameMessage {
  type: "frame";
  payload: StatusFrame;
}

interface ErrorMessage {
  type: "error";
  payload: {
    code: string;
    message: string;
  };
}

interface AckMessage {
  type: "ack";
}

type ServerMessage = RoleMessage | FrameMessage | ErrorMessage | AckMessage;

// Canvas rendering constants
const GRID_SPACING_M = 1.0;
const GRID_COLOR = "#21262d";
const POINT_COLOR = "#9fdcff";
const TARGET_COLOR = "#9ecc3b";
const ALX_COLOR = "#3fb950";
const GATE_COLOR = "#bc8cff";
const CONNECTION_COLOR = "rgba(158, 204, 59, 0.18)";
const CONNECTION_BORDER_COLOR = "rgba(158, 204, 59, 0.82)";
const MAX_SCALE = 80; // pixels per meter
const MIN_SCALE = 28;
const VIEWPORT_PADDING_PX = 28;
const MAX_ALX_TRAIL_POINTS = 40;

interface Viewport {
  centerX: number;
  centerY: number;
  scale: number;
}

class LiDARMonitor {
  private ws: WebSocket | null = null;
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private role: "controller" | "observer" | null = null;
  private latestFrame: StatusFrame | null = null;
  private reconnectAttempts = 0;
  private selectedTarget: Point2D | null = null;
  private alxTrail: Point2D[] = [];

  // UI Elements
  private roleDisplay: HTMLElement;
  private lifecycleState: HTMLElement;
  private lockState: HTMLElement;
  private confidenceValue: HTMLElement;
  private stopReason: HTMLElement;
  private dispatchedVx: HTMLElement;
  private dispatchedVy: HTMLElement;
  private dispatchedOmega: HTMLElement;
  private startBtn: HTMLButtonElement;
  private stopBtn: HTMLButtonElement;
  private errorMessage: HTMLElement;

  constructor() {
    this.canvas = document.getElementById("lidar-canvas") as HTMLCanvasElement;
    this.ctx = this.canvas.getContext("2d")!;

    this.roleDisplay = document.getElementById("role-display")!;
    this.lifecycleState = document.getElementById("lifecycle-state")!;
    this.lockState = document.getElementById("lock-state")!;
    this.confidenceValue = document.getElementById("confidence-value")!;
    this.stopReason = document.getElementById("stop-reason")!;
    this.dispatchedVx = document.getElementById("dispatched-vx")!;
    this.dispatchedVy = document.getElementById("dispatched-vy")!;
    this.dispatchedOmega = document.getElementById("dispatched-omega")!;
    this.startBtn = document.getElementById("start-btn") as HTMLButtonElement;
    this.stopBtn = document.getElementById("stop-btn") as HTMLButtonElement;
    this.errorMessage = document.getElementById("error-message")!;

    this.setupEventHandlers();
    this.resizeCanvas();
    this.connect();
  }

  private setupEventHandlers(): void {
    // Canvas click handler (controller only)
    this.canvas.addEventListener("click", (e: MouseEvent) => {
      if (this.role !== "controller") {
        return;
      }

      const rect = this.canvas.getBoundingClientRect();
      const scaleX = this.canvas.width / rect.width;
      const scaleY = this.canvas.height / rect.height;
      const canvasX = (e.clientX - rect.left) * scaleX;
      const canvasY = (e.clientY - rect.top) * scaleY;

      const viewport = this.getViewport();
      const worldX = (canvasX - viewport.centerX) / viewport.scale;
      const worldY = (viewport.centerY - canvasY) / viewport.scale;

      this.selectedTarget = { x: worldX, y: worldY };
      this.sendSelectTarget(worldX, worldY);
    });

    // Control buttons
    this.startBtn.addEventListener("click", () => {
      if (this.role === "controller") {
        this.sendStartFollow();
      }
    });

    this.stopBtn.addEventListener("click", () => {
      if (this.role === "controller") {
        this.sendStopFollow();
      }
    });

    // Window resize
    window.addEventListener("resize", () => {
      this.resizeCanvas();
      this.render();
    });
  }

  private connect(): void {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    this.ws = new WebSocket(wsUrl);

    this.ws.onopen = () => {
      console.log("WebSocket connected");
      this.reconnectAttempts = 0;
      this.clearError();
    };

    this.ws.onmessage = (event: MessageEvent) => {
      try {
        const msg: ServerMessage = JSON.parse(event.data);
        this.handleMessage(msg);
      } catch (err) {
        console.error("Failed to parse message:", event.data, err);
      }
    };

    this.ws.onerror = (event: Event) => {
      console.error("WebSocket error:", event);
    };

    this.ws.onclose = () => {
      console.log("WebSocket closed, reconnecting...");
      this.role = null;
      this.updateRoleDisplay();
      this.reconnect();
    };
  }

  private reconnect(): void {
    this.reconnectAttempts++;
    const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 10000);
    setTimeout(() => this.connect(), delay);
  }

  private handleMessage(msg: ServerMessage): void {
    switch (msg.type) {
      case "role":
        this.role = msg.payload.role;
        this.updateRoleDisplay();
        break;

      case "frame":
        this.latestFrame = msg.payload;
        this.updateALXTrail(msg.payload);
        this.updateDisplay();
        this.render();
        break;

      case "error":
        this.showError(msg.payload.code, msg.payload.message);
        break;

      case "ack":
        this.clearError();
        break;
    }
  }

  private updateRoleDisplay(): void {
    this.roleDisplay.textContent = this.role ? this.role.toUpperCase() : "CONNECTING...";
    this.roleDisplay.className = `role-badge ${this.role || "connecting"}`;

    // Enable/disable control buttons
    const canControl = this.role === "controller";
    this.startBtn.disabled = !canControl;
    this.stopBtn.disabled = !canControl;
  }

  private updateDisplay(): void {
    if (!this.latestFrame) return;

    this.lifecycleState.textContent = this.latestFrame.lifecycle_state;
    this.lockState.textContent = this.latestFrame.lock_state;

    if (this.latestFrame.confidence) {
      this.confidenceValue.textContent = this.latestFrame.confidence.total.toFixed(2);
    } else {
      this.confidenceValue.textContent = "0.00";
    }

    if (this.latestFrame.stop && this.latestFrame.stop.reason) {
      this.stopReason.textContent = this.latestFrame.stop.reason;
      this.stopReason.style.color = "#f85149";
    } else {
      this.stopReason.textContent = "None";
      this.stopReason.style.color = "";
    }

    if (this.latestFrame.dispatched_command) {
      this.dispatchedVx.textContent = this.latestFrame.dispatched_command.vx.toFixed(2);
      this.dispatchedVy.textContent = this.latestFrame.dispatched_command.vy.toFixed(2);
      this.dispatchedOmega.textContent = this.latestFrame.dispatched_command.omega.toFixed(2);
    } else {
      this.dispatchedVx.textContent = "0.00";
      this.dispatchedVy.textContent = "0.00";
      this.dispatchedOmega.textContent = "0.00";
    }
  }

  private render(): void {
    this.resizeCanvas();

    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;

    // Clear
    ctx.fillStyle = "#0b0e14";
    ctx.fillRect(0, 0, width, height);

    // Draw grid
    this.drawGrid();

    // Draw the recent ALX path before the point cloud so its current position
    // remains easy to identify even in a dense scan.
    this.drawALXTrail();

    // Draw points
    if (this.latestFrame) {
      this.drawPoints(this.latestFrame.points, POINT_COLOR, 1);
      this.drawPoints(this.latestFrame.target_points || [], TARGET_COLOR, 2);

      // A target is drawn from the validated lock center, not directly from
      // the browser click.  This makes the visual link match the target that
      // the control loop is actually tracking.
      if (this.latestFrame.target_gate && this.hasActiveTarget()) {
        this.drawTargetConnection(this.latestFrame.target_gate);
      }

      // Draw gate ellipse
      if (this.latestFrame.target_gate) {
        this.drawGate(this.latestFrame.target_gate);
      }

      // Draw ALX position
      if (this.latestFrame.alx && this.latestFrame.alx.body_xy) {
        this.drawALX(this.latestFrame.alx.body_xy, this.latestFrame.alx.synchronized);
      }
    }

    // Draw selected target click position
    if (this.selectedTarget) {
      this.drawClick(this.selectedTarget);
    }
  }

  private hasActiveTarget(): boolean {
    const lockState = this.latestFrame?.lock_state;
    return lockState === "candidate" || lockState === "following";
  }

  private updateALXTrail(frame: StatusFrame): void {
    const alx = frame.alx;
    if (!alx?.body_xy || !alx.synchronized) {
      this.alxTrail = [];
      return;
    }

    const [x, y] = alx.body_xy;
    const previous = this.alxTrail[this.alxTrail.length - 1];
    if (previous && previous.x === x && previous.y === y) {
      return;
    }

    this.alxTrail.push({ x, y });
    if (this.alxTrail.length > MAX_ALX_TRAIL_POINTS) {
      this.alxTrail.shift();
    }
  }

  private resizeCanvas(): void {
    const rect = this.canvas.getBoundingClientRect();
    const nextWidth = Math.max(1, Math.floor(rect.width));
    const nextHeight = Math.max(1, Math.floor(rect.height));

    if (this.canvas.width !== nextWidth || this.canvas.height !== nextHeight) {
      this.canvas.width = nextWidth;
      this.canvas.height = nextHeight;
    }
  }

  private getViewport(): Viewport {
    const centerX = this.canvas.width / 2;
    const centerY = this.canvas.height / 2;
    const scale = this.getVisibleScale();
    return { centerX, centerY, scale };
  }

  private getVisibleScale(): number {
    const bounds = this.getVisibleBounds();
    const availableWidth = Math.max(1, this.canvas.width - VIEWPORT_PADDING_PX * 2);
    const availableHeight = Math.max(1, this.canvas.height - VIEWPORT_PADDING_PX * 2);
    const spanX = Math.max(1, Math.max(Math.abs(bounds.minX), Math.abs(bounds.maxX)) * 2);
    const spanY = Math.max(1, Math.max(Math.abs(bounds.minY), Math.abs(bounds.maxY)) * 2);
    const fitScale = Math.min(availableWidth / spanX, availableHeight / spanY);

    return Math.max(MIN_SCALE, Math.min(MAX_SCALE, fitScale));
  }

  private getVisibleBounds(): {
    minX: number;
    maxX: number;
    minY: number;
    maxY: number;
  } {
    const bounds = { minX: -2, maxX: 2, minY: -2, maxY: 2 };
    const include = (x: number, y: number): void => {
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        return;
      }
      bounds.minX = Math.min(bounds.minX, x);
      bounds.maxX = Math.max(bounds.maxX, x);
      bounds.minY = Math.min(bounds.minY, y);
      bounds.maxY = Math.max(bounds.maxY, y);
    };

    if (this.latestFrame) {
      for (const point of this.latestFrame.points) {
        include(point[0], point[1]);
      }
      for (const point of this.latestFrame.target_points || []) {
        include(point[0], point[1]);
      }

      const gate = this.latestFrame.target_gate;
      if (gate) {
        include(gate.center_xy[0] - gate.radial_radius_m, gate.center_xy[1] - gate.tangential_radius_m);
        include(gate.center_xy[0] + gate.radial_radius_m, gate.center_xy[1] + gate.tangential_radius_m);
      }

      const alx = this.latestFrame.alx?.body_xy;
      if (alx) {
        include(alx[0], alx[1]);
      }
    }

    for (const point of this.alxTrail) {
      include(point.x, point.y);
    }
    if (this.selectedTarget) {
      include(this.selectedTarget.x, this.selectedTarget.y);
    }

    return bounds;
  }

  private toCanvas(point: Point2D, viewport: Viewport): Point2D {
    return {
      x: viewport.centerX + point.x * viewport.scale,
      y: viewport.centerY - point.y * viewport.scale,
    };
  }

  private drawGrid(): void {
    const ctx = this.ctx;
    const viewport = this.getViewport();
    ctx.strokeStyle = GRID_COLOR;
    ctx.lineWidth = 1;

    // Vertical lines
    const gridStep = GRID_SPACING_M * viewport.scale;
    for (let x = viewport.centerX % gridStep; x < this.canvas.width; x += gridStep) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, this.canvas.height);
      ctx.stroke();
    }

    // Horizontal lines
    for (let y = viewport.centerY % gridStep; y < this.canvas.height; y += gridStep) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(this.canvas.width, y);
      ctx.stroke();
    }

    // Draw origin (robot position)
    ctx.fillStyle = "#484f58";
    ctx.beginPath();
    ctx.arc(viewport.centerX, viewport.centerY, 4, 0, 2 * Math.PI);
    ctx.fill();
  }

  private drawPoints(points: Point3D[], color: string, size: number): void {
    const ctx = this.ctx;
    const viewport = this.getViewport();
    ctx.fillStyle = color;

    for (const pt of points) {
      const point = this.toCanvas({ x: pt[0], y: pt[1] }, viewport);

      ctx.beginPath();
      ctx.arc(point.x, point.y, size, 0, 2 * Math.PI);
      ctx.fill();
    }
  }

  private drawGate(gate: {
    kind: string;
    center_xy: [number, number];
    radial_radius_m: number;
    tangential_radius_m: number;
  }): void {
    const ctx = this.ctx;
    const viewport = this.getViewport();
    const center = this.toCanvas({ x: gate.center_xy[0], y: gate.center_xy[1] }, viewport);
    const radiusX = gate.radial_radius_m * viewport.scale;
    const radiusY = gate.tangential_radius_m * viewport.scale;

    ctx.strokeStyle = GATE_COLOR;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.ellipse(center.x, center.y, radiusX, radiusY, 0, 0, 2 * Math.PI);
    ctx.stroke();
  }

  private drawTargetConnection(gate: {
    kind: string;
    center_xy: [number, number];
    radial_radius_m: number;
    tangential_radius_m: number;
  }): void {
    const ctx = this.ctx;
    const [targetX, targetY] = gate.center_xy;
    const viewport = this.getViewport();
    const end = this.toCanvas({ x: targetX, y: targetY }, viewport);
    const dx = end.x - viewport.centerX;
    const dy = end.y - viewport.centerY;
    const length = Math.hypot(dx, dy);

    if (length < 1) {
      return;
    }

    // Use the current tracking gate to size a stable, readable corridor.
    const halfWidth = Math.max(12, Math.min(28, gate.tangential_radius_m * viewport.scale));
    const heading = Math.atan2(dy, dx);

    ctx.save();
    ctx.translate(viewport.centerX, viewport.centerY);
    ctx.rotate(heading);
    ctx.fillStyle = CONNECTION_COLOR;
    ctx.strokeStyle = CONNECTION_BORDER_COLOR;
    ctx.lineWidth = 1.5;
    ctx.fillRect(0, -halfWidth, length, halfWidth * 2);
    ctx.strokeRect(0, -halfWidth, length, halfWidth * 2);
    ctx.restore();

    ctx.fillStyle = TARGET_COLOR;
    ctx.beginPath();
    ctx.arc(end.x, end.y, 5, 0, 2 * Math.PI);
    ctx.fill();
  }

  private drawALXTrail(): void {
    if (this.alxTrail.length < 2) {
      return;
    }

    const ctx = this.ctx;
    const viewport = this.getViewport();
    ctx.save();
    ctx.strokeStyle = "rgba(63, 185, 80, 0.55)";
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    this.alxTrail.forEach((point, index) => {
      const canvasPoint = this.toCanvas(point, viewport);
      if (index === 0) {
        ctx.moveTo(canvasPoint.x, canvasPoint.y);
      } else {
        ctx.lineTo(canvasPoint.x, canvasPoint.y);
      }
    });
    ctx.stroke();
    ctx.restore();
  }

  private drawALX(body_xy: [number, number], synchronized: boolean): void {
    const ctx = this.ctx;
    const viewport = this.getViewport();
    const point = this.toCanvas({ x: body_xy[0], y: body_xy[1] }, viewport);

    ctx.fillStyle = synchronized ? ALX_COLOR : "#d29922";
    ctx.beginPath();
    ctx.arc(point.x, point.y, 6, 0, 2 * Math.PI);
    ctx.fill();

    ctx.fillStyle = synchronized ? ALX_COLOR : "#d29922";
    ctx.font = "11px monospace";
    ctx.fillText(`ALX (${body_xy[0].toFixed(2)}, ${body_xy[1].toFixed(2)})`, point.x + 9, point.y - 9);
  }

  private drawClick(target: Point2D): void {
    const ctx = this.ctx;
    const viewport = this.getViewport();
    const point = this.toCanvas(target, viewport);

    // Crosshair
    ctx.strokeStyle = TARGET_COLOR;
    ctx.lineWidth = 1;

    ctx.beginPath();
    ctx.moveTo(point.x - 8, point.y);
    ctx.lineTo(point.x + 8, point.y);
    ctx.moveTo(point.x, point.y - 8);
    ctx.lineTo(point.x, point.y + 8);
    ctx.stroke();
  }

  private sendSelectTarget(x: number, y: number): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    const msg = { type: "select_target", x, y };
    this.ws.send(JSON.stringify(msg));
  }

  private sendStartFollow(): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    const msg = { type: "start_follow" };
    this.ws.send(JSON.stringify(msg));
  }

  private sendStopFollow(): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      return;
    }

    const msg = { type: "stop_follow" };
    this.ws.send(JSON.stringify(msg));
  }

  private showError(code: string, message: string): void {
    this.errorMessage.textContent = `[${code}] ${message}`;
    this.errorMessage.classList.remove("hidden");
  }

  private clearError(): void {
    this.errorMessage.classList.add("hidden");
  }
}

// Initialize on load
window.addEventListener("DOMContentLoaded", () => {
  new LiDARMonitor();
});
