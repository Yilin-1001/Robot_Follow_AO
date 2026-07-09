"""G1_Mid360_AO 核心数据类型（Phase 0）。

纯数据结构 + 枚举 + 工厂类方法，零算法逻辑。
对应主方案 docs/G1_Mid360_AO.md 第 8 节。
后续 Task 在本文件追加 CommandKind / DecisionKind / Direction /
SafetyState / MotionCommand / PointCloudFrame / ZoneResult /
ObstacleReport / AvoidanceDecision / FrameDebug。
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np


class CommandKind(str, Enum):
    """运动命令类型（§8.1）。"""

    MOVE = "move"
    STOP = "stop"


class DecisionKind(str, Enum):
    """避障策略类型（§8.5）。"""

    NORMAL = "normal"
    SLOWDOWN = "slowdown"
    AVOID_LEFT = "avoid_left"
    AVOID_RIGHT = "avoid_right"
    ROTATE = "rotate"
    STOP = "stop"


class Direction(str, Enum):
    """planner 建议方向（§8.5 recommended_direction）。"""

    FORWARD = "forward"
    LEFT = "left"
    RIGHT = "right"
    BACK = "back"
    STOP = "stop"


class SafetyState(str, Enum):
    """系统控制状态（§6.5）。"""

    AUTO_AO = "AUTO_AO"
    REMOTE_OVERRIDE = "REMOTE_OVERRIDE"
    SAFE_STOP = "SAFE_STOP"
    ESTOP = "ESTOP"


@dataclass(frozen=True)
class MotionCommand:
    """运动命令（§8.1）。

    字段顺序：必需字段在前、带默认值在后（满足 dataclass
    「非默认字段不能跟在默认字段后」的约束；字段集合与语义与主方案一致）。
    """

    timestamp: float
    kind: CommandKind
    source: str
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    reason: str = ""

    @classmethod
    def stop(cls, timestamp, *, source, reason=""):
        """构造 stop 命令，强制三向速度为 0（§13 硬约束）。"""
        return cls(
            timestamp=timestamp,
            kind=CommandKind.STOP,
            source=source,
            vx=0.0,
            vy=0.0,
            omega=0.0,
            reason=reason,
        )

    @classmethod
    def move(cls, timestamp, *, source, vx=0.0, vy=0.0, omega=0.0, reason=""):
        """构造 move 命令，保留给定速度。"""
        return cls(
            timestamp=timestamp,
            kind=CommandKind.MOVE,
            source=source,
            vx=vx,
            vy=vy,
            omega=omega,
            reason=reason,
        )


@dataclass(frozen=True)
class PointCloudFrame:
    """点云帧（§8.2）。

    valid/empty_is_clear 表达两种合法状态：
    无错误空点云 -> clear；雷达掉线/解析失败/TF 失败/超时 -> invalid。
    是否为错误的判断属于 Phase 1/2 预处理逻辑，本类只承载状态。
    注：含 np.ndarray，实例不可哈希（本阶段不哈希）。
    """

    timestamp: float
    frame_id: str
    points_xyz: np.ndarray
    valid: bool
    empty_is_clear: bool
    reason: str = ""
    received_at_s: Optional[float] = None

    @classmethod
    def empty_clear(cls, timestamp, *, frame_id, reason=""):
        """无错误空点云：合法的 clear 状态。"""
        return cls(
            timestamp=timestamp,
            frame_id=frame_id,
            points_xyz=np.empty((0, 3), dtype=np.float64),
            valid=True,
            empty_is_clear=True,
            reason=reason,
        )

    @classmethod
    def invalid(cls, timestamp, *, frame_id, reason):
        """雷达掉线/解析失败/TF 失败/超时：invalid 状态。"""
        return cls(
            timestamp=timestamp,
            frame_id=frame_id,
            points_xyz=np.empty((0, 3), dtype=np.float64),
            valid=False,
            empty_is_clear=False,
            reason=reason,
        )


@dataclass(frozen=True)
class ZoneResult:
    """单个安全区的占用结果（§8.3）。"""

    name: str
    point_count: int
    min_x_m: Optional[float]
    min_distance_m: Optional[float]
    occupied: bool = False
    blocked: bool = False

    @classmethod
    def empty(cls, name):
        """空/未占用的 zone。"""
        return cls(
            name=name,
            point_count=0,
            min_x_m=None,
            min_distance_m=None,
            occupied=False,
            blocked=False,
        )


@dataclass(frozen=True)
class ObstacleReport:
    """环境事实接口（§8.4）：只描述点云质量与 zone 占用，不含推荐方向。"""

    timestamp: float
    valid: bool
    front: ZoneResult
    left: ZoneResult
    right: ZoneResult
    rear: ZoneResult
    rotation: ZoneResult
    emergency_stop: bool = False
    reason: str = ""

    @classmethod
    def clear(cls, timestamp):
        """全部 zone 未占用的合法 clear 报告。"""
        return cls(
            timestamp=timestamp,
            valid=True,
            front=ZoneResult.empty("front"),
            left=ZoneResult.empty("left"),
            right=ZoneResult.empty("right"),
            rear=ZoneResult.empty("rear"),
            rotation=ZoneResult.empty("rotation"),
            emergency_stop=False,
            reason="",
        )

    @classmethod
    def invalid(cls, timestamp, *, reason):
        """雷达无效/超时报告（valid=False）。"""
        return cls(
            timestamp=timestamp,
            valid=False,
            front=ZoneResult.empty("front"),
            left=ZoneResult.empty("left"),
            right=ZoneResult.empty("right"),
            rear=ZoneResult.empty("rear"),
            rotation=ZoneResult.empty("rotation"),
            emergency_stop=False,
            reason=reason,
        )

    @classmethod
    def front_blocked(cls, timestamp, *, min_distance_m):
        """Boxed-in 报告：前方 + 左右两侧均 blocked。

        AO 运行时适配测试用的工厂。真实 stateful planner 在仅前方受阻、侧边
        空闲时会选择侧向避让（move）而非 stop；要让 planner 落到 safe_stop，
        需要构造「三面受阻」报告，使 choose_avoidance_side 返回 None。
        ``min_distance_m`` 同时作为前方 zone 的最近障碍距离。
        """
        def _blocked(name):
            return ZoneResult(
                name=name,
                point_count=1,
                min_x_m=min_distance_m,
                min_distance_m=min_distance_m,
                occupied=True,
                blocked=True,
            )

        return cls(
            timestamp=timestamp,
            valid=True,
            front=_blocked("front"),
            left=_blocked("left"),
            right=_blocked("right"),
            rear=ZoneResult.empty("rear"),
            rotation=ZoneResult.empty("rotation"),
            emergency_stop=False,
            reason="front_blocked",
        )


@dataclass(frozen=True)
class AvoidanceDecision:
    """避障策略结果（§8.5），由 SafetySupervisor/Planner/Limiter 共同产生。"""

    timestamp: float
    safe_command: MotionCommand
    decision_kind: DecisionKind
    recommended_direction: Direction
    slowdown_ratio: float = 1.0
    triggered_safety_rules: tuple = ()
    reason: str = ""

    @classmethod
    def stop(cls, timestamp, *, source, reason, rules=()):
        """构造 stop 决策：safe_command 为 stop，方向 stop。"""
        return cls(
            timestamp=timestamp,
            safe_command=MotionCommand.stop(
                timestamp=timestamp, source=source, reason=reason
            ),
            decision_kind=DecisionKind.STOP,
            recommended_direction=Direction.STOP,
            slowdown_ratio=1.0,
            triggered_safety_rules=rules,
            reason=reason,
        )


@dataclass(frozen=True)
class FrameDebug:
    """单帧日志的内存结构（§8.6），§17 CSV 字段是它的文本展开。"""

    frame_index: int
    timestamp: float
    point_count_raw: int
    point_count_filtered: int
    obstacle_report: ObstacleReport
    desired_command: MotionCommand
    avoidance_decision: AvoidanceDecision
    safety_state: SafetyState
    avoidance_state: str = "stateless"
    radar_age_s: Optional[float] = None
    alx_age_s: Optional[float] = None
    loop_dt_s: Optional[float] = None
    # Bypass mode latency chain (for testing robot-follow end-to-end delay)
    command_age_s: Optional[float] = None  # Time since desired_command was generated
    dispatch_elapsed_ms: Optional[float] = None  # G1 SDK dispatch() call duration
