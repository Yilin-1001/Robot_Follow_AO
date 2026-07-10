import signal
import subprocess

from robot_follow.ao.config import AORadarServiceConfig
from robot_follow.ao.live.radar_service_manager import RadarServiceManager, RadarServiceState


class FakeProcess:
    def __init__(self, name: str, pid: int) -> None:
        self.name = name
        self.pid = pid
        self.terminated = False
        self.killed = False

    def poll(self):
        return None

    def terminate(self) -> None:
        self.terminated = True

    def kill(self) -> None:
        self.killed = True

    def wait(self, timeout=None) -> int:
        return 0


class FakePopenFactory:
    def __init__(self) -> None:
        self.commands: list[str] = []
        self.processes: list[FakeProcess] = []

    def __call__(self, command: str, shell: bool, start_new_session: bool):
        self.commands.append(command)
        process = FakeProcess(command, pid=1000 + len(self.processes))
        self.processes.append(process)
        return process


def missing_process_group(_pgid, _signal) -> None:
    raise ProcessLookupError


def test_ensure_running_starts_livox_then_fast_lio_once() -> None:
    popen = FakePopenFactory()
    manager = RadarServiceManager(
        AORadarServiceConfig(
            livox_start_command="start_livox",
            fast_lio_start_command="start_fast_lio",
            idle_stop_s=5.0,
        ),
        popen_factory=popen,
        signal_process_group=missing_process_group,
    )

    manager.on_position_frame(10.0)
    manager.ensure_running()
    manager.ensure_running()

    assert popen.commands == ["start_livox", "start_fast_lio"]
    assert manager.state == RadarServiceState.RUNNING


def test_idle_stop_uses_position_frame_age_not_motion_command_age() -> None:
    popen = FakePopenFactory()
    manager = RadarServiceManager(
        AORadarServiceConfig(
            livox_start_command="start_livox",
            fast_lio_start_command="start_fast_lio",
            idle_stop_s=5.0,
        ),
        popen_factory=popen,
        signal_process_group=missing_process_group,
    )

    manager.on_position_frame(10.0)
    manager.ensure_running()
    manager.stop_if_idle(now=14.9)

    assert manager.state == RadarServiceState.RUNNING
    assert popen.processes[0].terminated is False

    manager.stop_if_idle(now=15.1)

    assert manager.state == RadarServiceState.STOPPED
    assert popen.processes[0].terminated is True
    assert popen.processes[1].terminated is True


def test_stop_signals_entire_ros_process_groups() -> None:
    popen = FakePopenFactory()
    signals = []
    manager = RadarServiceManager(
        AORadarServiceConfig(
            livox_start_command="start_livox",
            fast_lio_start_command="start_fast_lio",
        ),
        popen_factory=popen,
        signal_process_group=lambda pgid, sig: signals.append((pgid, sig)),
    )

    manager.ensure_running()
    manager.stop()

    assert signals == [
        (popen.processes[1].pid, signal.SIGTERM),
        (popen.processes[0].pid, signal.SIGTERM),
    ]


def test_stop_kills_process_group_after_timeout() -> None:
    class TimeoutProcess(FakeProcess):
        def wait(self, timeout=None) -> int:
            if timeout != 1.0:
                raise subprocess.TimeoutExpired(self.name, timeout)
            return 0

    process = TimeoutProcess("fast_lio", pid=2000)
    signals = []
    manager = RadarServiceManager(
        AORadarServiceConfig(stop_timeout_s=0.1),
        signal_process_group=lambda pgid, sig: signals.append((pgid, sig)),
    )

    manager._stop_process(process)

    assert signals == [
        (process.pid, signal.SIGTERM),
        (process.pid, signal.SIGKILL),
    ]


def test_missing_start_command_raises_clear_error() -> None:
    manager = RadarServiceManager(AORadarServiceConfig())
    manager.on_position_frame(10.0)

    try:
        manager.ensure_running()
    except ValueError as exc:
        assert "livox_start_command" in str(exc)
    else:
        raise AssertionError("expected ValueError")
