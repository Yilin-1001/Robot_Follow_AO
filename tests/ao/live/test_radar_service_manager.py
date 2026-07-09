from robot_follow.ao.config import AORadarServiceConfig
from robot_follow.ao.live.radar_service_manager import RadarServiceManager, RadarServiceState


class FakeProcess:
    def __init__(self, name: str) -> None:
        self.name = name
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
        process = FakeProcess(command)
        self.processes.append(process)
        return process


def test_ensure_running_starts_livox_then_fast_lio_once() -> None:
    popen = FakePopenFactory()
    manager = RadarServiceManager(
        AORadarServiceConfig(
            livox_start_command="start_livox",
            fast_lio_start_command="start_fast_lio",
            idle_stop_s=5.0,
        ),
        popen_factory=popen,
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


def test_missing_start_command_raises_clear_error() -> None:
    manager = RadarServiceManager(AORadarServiceConfig())
    manager.on_position_frame(10.0)

    try:
        manager.ensure_running()
    except ValueError as exc:
        assert "livox_start_command" in str(exc)
    else:
        raise AssertionError("expected ValueError")
