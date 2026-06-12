# Run Logging Implementation Design

本文档定义机器人现场测试的 run logging 方案。目标是每次测试自动生成一个完整、可回传、可复盘的 run 目录，方便通过 SSH/rsync 拉回本地进行代码优化。

## 1. 目标

每一次运行都生成独立目录，包含：

```text
command.txt       实际运行命令
run_config.json   当前运行参数
metadata.json     主机、时间、Python 版本、git commit 等环境信息
frames.csv        每帧 ALX / pipeline / command 数据
console.log       控制台输出
summary.md        本轮测试摘要
```

核心原则：

```text
一轮测试 = 一个 run 目录
所有现场数据可通过 rsync 一次性回传
不依赖 Git 也能复现测试参数
不覆盖历史测试结果
```

## 2. 推荐目录结构

```text
logs/
  runs/
    2026-06-12_153012_closed_loop/
      command.txt
      run_config.json
      metadata.json
      frames.csv
      console.log
      summary.md
```

run id 建议格式：

```text
YYYY-MM-DD_HHMMSS_<run-name>
```

示例：

```text
2026-06-12_153012_closed_loop
2026-06-12_160045_alx_dry_run
2026-06-12_163210_single_move
```

## 3. CLI 设计

建议在 `robot_follow.main` 中新增参数：

```text
--run-dir PATH       指定本轮 run 输出目录
--run-name NAME      自动生成 run 目录时使用的名称
```

使用方式：

```bash
conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --duration 10 \
  --run-name alx_dry_run
```

自动生成：

```text
logs/runs/2026-06-12_153012_alx_dry_run/
```

也可以显式指定：

```bash
conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --duration 10 \
  --run-dir logs/runs/manual_test_001
```

如果同时传 `--csv` 和 `--run-dir`：

```text
优先使用 --csv 指定的路径
同时仍然生成 command.txt / run_config.json / metadata.json / summary.md
```

如果只传 `--run-dir`，程序自动设置：

```text
csv_path = <run-dir>/frames.csv
console_log_path = <run-dir>/console.log
```

## 4. 文件内容定义

### 4.1 command.txt

记录实际运行命令，方便现场复现。

示例：

```text
conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --net-iface enp2s0 \
  --enable-motion \
  --i-understand-this-moves-robot \
  --max-vx 0.03 \
  --max-omega 0.10 \
  --duration 10 \
  --run-dir logs/runs/2026-06-12_153012_closed_loop
```

实现建议：

```python
" ".join(shlex.quote(part) for part in sys.argv)
```

如果通过 `conda run` 启动，Python 进程只能看到 `python -m robot_follow.main ...` 之后的参数。完整 conda 命令可由外层脚本写入，或者在 `command.txt` 中记录 `sys.argv`。

### 4.2 run_config.json

记录当前运行参数。该文件用于本地分析和复现实验。

示例：

```json
{
  "port": "/dev/ttyUSB0",
  "baudrate": 115200,
  "hz": 40.0,
  "stale_timeout_s": 0.5,
  "duration_s": 10.0,
  "frame_limit": null,
  "print_every_s": 0.5,
  "csv_path": "frames.csv",
  "motion": {
    "enabled": true,
    "dry_run": false,
    "net_iface": "enp2s0",
    "max_vx": 0.03,
    "max_omega": 0.1,
    "yaw_sign": 1
  }
}
```

当前 `main.py` 还没有真实 G1 参数时，先记录已有字段：

```json
{
  "port": "/dev/ttyUSB0",
  "baudrate": 115200,
  "hz": 40.0,
  "stale_timeout_s": 0.5,
  "duration_s": 10.0,
  "frame_limit": null,
  "print_every_s": 0.5,
  "csv_path": "frames.csv"
}
```

### 4.3 metadata.json

记录运行环境，方便判断现场结果对应的代码版本和机器状态。

建议字段：

```json
{
  "run_id": "2026-06-12_153012_closed_loop",
  "created_at_local": "2026-06-12T15:30:12+08:00",
  "hostname": "jetson-orin",
  "platform": "Linux-...",
  "python_version": "3.10.20",
  "python_executable": "/home/yilin/miniconda3/envs/robot-follow/bin/python",
  "cwd": "/home/yilin/Robot_Follow",
  "git_commit": "abc1234",
  "git_branch": "robot-g1-onsite-test",
  "git_dirty": true
}
```

如果现场不用 Git，`git_commit` 可以为空，但仍应记录：

```json
{
  "git_available": false
}
```

### 4.4 frames.csv

当前 `main.py` 已经支持 CSV。建议继续保留现有字段：

```text
frame
elapsed_s
status
alx_fps
alx_age_ms
timestamp
distance_m
azimuth_deg
target_x_m
target_y_m
valid
command_kind
command_reason
vx
vy
omega
```

后续接入 G1 bridge 后建议增加字段：

```text
sdk_error
bridge_state
bridge_last_code
bridge_last_error
```

这些字段对现场排查非常重要。

### 4.5 console.log

记录控制台输出，包括：

```text
RUN_CONFIG
FRAME
EVENT
summary
exception / traceback
```

实现方式有两种：

1. 外层 shell 使用 `tee`：

```bash
conda run -n robot-follow python -m robot_follow.main ... \
  2>&1 | tee logs/runs/001/console.log
```

2. 程序内部 emit 同时写 stdout 和文件。

推荐先用程序内部实现，避免现场忘记 `tee`。

### 4.6 summary.md

程序结束时自动生成基础摘要，现场人员可以再手工补充。

示例：

```markdown
# Run 2026-06-12_153012_closed_loop

## Result

- Status: completed
- Frames: 400
- Moves: 382
- Stops: 18
- Last command:

## Config

- port: /dev/ttyUSB0
- hz: 40.0
- duration: 10.0
- csv: frames.csv

## Observations

- Pass/Fail:
- Safety issue:
- Notes:
```

## 5. 实现方案

### 5.1 新增数据结构

建议新增：

```python
@dataclass(frozen=True)
class RunLogConfig:
    run_dir: Path | None = None
    run_name: str | None = None
```

或者直接把 `run_dir` / `run_name` 加到 `LiveALXConfig`：

```python
@dataclass(frozen=True)
class LiveALXConfig:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    hz: float = 40.0
    stale_timeout_s: float = 0.5
    duration_s: Optional[float] = None
    frame_limit: Optional[int] = None
    print_every_s: float = 0.5
    csv_path: Optional[Path] = None
    run_dir: Optional[Path] = None
    run_name: Optional[str] = None
```

### 5.2 Run 目录创建

建议实现：

```python
def resolve_run_dir(config: LiveALXConfig, *, clock: Callable[[], float]) -> Path | None:
    if config.run_dir is not None:
        return config.run_dir
    if config.run_name is None:
        return None
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    safe_name = sanitize_run_name(config.run_name)
    return Path("logs/runs") / f"{stamp}_{safe_name}"
```

`sanitize_run_name` 只允许：

```text
a-z A-Z 0-9 _ -
```

避免 run name 里出现路径穿越或空格。

### 5.3 Emit 同时写 stdout 和 console.log

当前 `run_live_alx_simulation` 已有 `emit` 注入点。可以利用它实现 tee。

```python
class TeeEmitter:
    def __init__(self, path: Path):
        self.path = path
        self._handle = path.open("a", encoding="utf-8")

    def __call__(self, line: str) -> None:
        print(line)
        self._handle.write(line + "\n")
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()
```

在 `main()` 中：

```python
run_dir = resolve_run_dir(config, clock=time.monotonic)
if run_dir is not None:
    run_dir.mkdir(parents=True, exist_ok=False)
    emit = TeeEmitter(run_dir / "console.log")
else:
    emit = print

try:
    run_live_alx_simulation(config, emit=emit)
finally:
    if hasattr(emit, "close"):
        emit.close()
```

注意：`exist_ok=False` 可以避免覆盖历史 run。若目录已存在，应报错或要求用户换 `--run-name`。

### 5.4 自动设置 CSV 路径

如果用户传了 `--run-dir` 或 `--run-name`，但没有传 `--csv`：

```python
csv_path = run_dir / "frames.csv"
```

如果用户显式传了 `--csv`，尊重用户输入。

### 5.5 写 run_config.json

```python
def write_run_config(path: Path, config: LiveALXConfig) -> None:
    data = {
        "port": config.port,
        "baudrate": config.baudrate,
        "hz": config.hz,
        "stale_timeout_s": config.stale_timeout_s,
        "duration_s": config.duration_s,
        "frame_limit": config.frame_limit,
        "print_every_s": config.print_every_s,
        "csv_path": str(config.csv_path) if config.csv_path else None,
    }
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
```

后续加入 G1 参数后，把 motion 参数嵌入 `"motion"` 字段。

### 5.6 写 metadata.json

建议采集：

```python
def collect_metadata(run_id: str) -> dict:
    return {
        "run_id": run_id,
        "created_at_local": datetime.now().astimezone().isoformat(),
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "python_version": platform.python_version(),
        "python_executable": sys.executable,
        "cwd": str(Path.cwd()),
        "git_commit": git_output(["git", "rev-parse", "HEAD"]),
        "git_branch": git_output(["git", "branch", "--show-current"]),
        "git_dirty": bool(git_output(["git", "status", "--porcelain"])),
    }
```

`git_output` 必须容忍失败：

```python
def git_output(args: list[str]) -> str | None:
    try:
        result = subprocess.run(args, capture_output=True, text=True, check=True)
    except Exception:
        return None
    return result.stdout.strip()
```

### 5.7 写 command.txt

```python
def write_command(path: Path) -> None:
    command = " ".join(shlex.quote(part) for part in sys.argv)
    path.write_text(command + "\n", encoding="utf-8")
```

### 5.8 写 summary.md

在 `run_live_alx_simulation` 返回 `LiveALXRunResult` 后写：

```python
def write_summary(path: Path, result: LiveALXRunResult, config: LiveALXConfig) -> None:
    text = f"""# Run Summary

## Result

- Frames: {len(result.frames)}
- Moves: {result.move_count}
- Stops: {result.stop_count}
- Last: {result.last}

## Config

- port: {config.port}
- baudrate: {config.baudrate}
- hz: {config.hz}
- stale_timeout_s: {config.stale_timeout_s}
- duration_s: {config.duration_s}
- frame_limit: {config.frame_limit}
- csv_path: {config.csv_path}

## Observations

- Pass/Fail:
- Safety issue:
- Notes:
"""
    path.write_text(text, encoding="utf-8")
```

## 6. 运行示例

### 6.1 ALX dry-run

```bash
conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --duration 20 \
  --run-name alx_dry_run
```

生成：

```text
logs/runs/2026-06-12_153012_alx_dry_run/
```

### 6.2 ALX + G1 闭环

```bash
conda run -n robot-follow python -m robot_follow.main \
  --port /dev/ttyUSB0 \
  --net-iface enp2s0 \
  --enable-motion \
  --i-understand-this-moves-robot \
  --max-vx 0.03 \
  --max-omega 0.10 \
  --duration 10 \
  --run-name closed_loop
```

## 7. SSH 回传

回传全部 run：

```bash
rsync -av jetson:~/Robot_Follow/logs/runs/ \
  /home/yilin/Code/Robot_Follow/logs/runs/
```

只回传某一轮：

```bash
rsync -av jetson:~/Robot_Follow/logs/runs/2026-06-12_153012_closed_loop/ \
  /home/yilin/Code/Robot_Follow/logs/runs/2026-06-12_153012_closed_loop/
```

## 8. 本地分析流程

回传后先看：

```text
summary.md
run_config.json
console.log
frames.csv
```

推荐顺序：

1. 看 `summary.md`，确认是否完成、帧数、move/stop 数。
2. 看 `run_config.json`，确认现场参数。
3. 看 `console.log`，确认是否有 `data_timeout`、`sdk_error`、watchdog、异常。
4. 看 `frames.csv`，分析 `status`、`alx_age_ms`、`command_reason`、`vx`、`omega`。

## 9. 实现优先级

建议分两步实现。

### Phase 1: 最小可用

必须实现：

```text
--run-dir
--run-name
frames.csv 自动路径
console.log
run_config.json
metadata.json
summary.md
```

### Phase 2: 接入 G1 后增强

增加：

```text
motion 参数记录
bridge_state
sdk_error
SDK return code
watchdog event
StopMove / Move 事件日志
```

## 10. 注意事项

1. run 目录不能覆盖已有目录。
2. `logs/runs/` 不建议提交到 Git，除非只提交小的 `summary.md` 和配置文件。
3. 大 CSV 和 console log 推荐通过 SSH/rsync 存档。
4. 现场不用 Git 时，`metadata.json` 和 `command.txt` 是复现测试的最低要求。
5. 真实机器人测试必须保留 `summary.md`，便于本地复盘和参数回传。
