/**
 * LiDAR + ALX Assisted Follow Monitor
 * Main TypeScript entry point
 */

interface Point2D {
  x: number;
  y: number;
}

interface Point3D extends Point2D {
  z: number;
}

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
const POINT_COLOR = "#8b949e";
const TARGET_COLOR = "#9ecc3b";
const ALX_COLOR = "#3fb950";
const GATE_COLOR = "#bc8cff";
const SCALE = 80; // pixels per meter
const CENTER_X = 400;
const CENTER_Y = 300;

class LiDARMonitor {
  private ws: WebSocket | null = null;
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private role: "controller" | "observer" | null = null;
  private latestFrame: StatusFrame | null = null;
  private reconnectAttempts = 0;
  private selectedTarget: Point2D | null = null;

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

      // Convert to world coordinates (robot-centric)
      const worldX = (canvasX - CENTER_X) / SCALE;
      const worldY = (CENTER_Y - canvasY) / SCALE;

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
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;

    // Clear
    ctx.fillStyle = "#0b0e14";
    ctx.fillRect(0, 0, width, height);

    // Draw grid
    this.drawGrid();

    // Draw points
    if (this.latestFrame) {
      this.drawPoints(this.latestFrame.points, POINT_COLOR, 1);
      this.drawPoints(this.latestFrame.target_points || [], TARGET_COLOR, 2);

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

  private drawGrid(): void {
    const ctx = this.ctx;
    ctx.strokeStyle = GRID_COLOR;
    ctx.lineWidth = 1;

    // Vertical lines
    for (let x = CENTER_X % (GRID_SPACING_M * SCALE); x < this.canvas.width; x += GRID_SPACING_M * SCALE) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, this.canvas.height);
      ctx.stroke();
    }

    // Horizontal lines
    for (let y = CENTER_Y % (GRID_SPACING_M * SCALE); y < this.canvas.height; y += GRID_SPACING_M * SCALE) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(this.canvas.width, y);
      ctx.stroke();
    }

    // Draw origin (robot position)
    ctx.fillStyle = "#484f58";
    ctx.beginPath();
    ctx.arc(CENTER_X, CENTER_Y, 4, 0, 2 * Math.PI);
    ctx.fill();
  }

  private drawPoints(points: Point3D[], color: string, size: number): void {
    const ctx = this.ctx;
    ctx.fillStyle = color;

    for (const pt of points) {
      const canvasX = CENTER_X + pt.x * SCALE;
      const canvasY = CENTER_Y - pt.y * SCALE;

      ctx.beginPath();
      ctx.arc(canvasX, canvasY, size, 0, 2 * Math.PI);
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
    const centerX = CENTER_X + gate.center_xy[0] * SCALE;
    const centerY = CENTER_Y - gate.center_xy[1] * SCALE;
    const radiusX = gate.radial_radius_m * SCALE;
    const radiusY = gate.tangential_radius_m * SCALE;

    ctx.strokeStyle = GATE_COLOR;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.ellipse(centerX, centerY, radiusX, radiusY, 0, 0, 2 * Math.PI);
    ctx.stroke();
  }

  private drawALX(body_xy: [number, number], synchronized: boolean): void {
    const ctx = this.ctx;
    const canvasX = CENTER_X + body_xy[0] * SCALE;
    const canvasY = CENTER_Y - body_xy[1] * SCALE;

    ctx.fillStyle = synchronized ? ALX_COLOR : "#d29922";
    ctx.beginPath();
    ctx.arc(canvasX, canvasY, 6, 0, 2 * Math.PI);
    ctx.fill();
  }

  private drawClick(target: Point2D): void {
    const ctx = this.ctx;
    const canvasX = CENTER_X + target.x * SCALE;
    const canvasY = CENTER_Y - target.y * SCALE;

    // Crosshair
    ctx.strokeStyle = TARGET_COLOR;
    ctx.lineWidth = 1;

    ctx.beginPath();
    ctx.moveTo(canvasX - 8, canvasY);
    ctx.lineTo(canvasX + 8, canvasY);
    ctx.moveTo(canvasX, canvasY - 8);
    ctx.lineTo(canvasX, canvasY + 8);
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