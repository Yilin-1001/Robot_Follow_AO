"""Logged test runner for repeatable Robot_Follow field tests.

This wrapper runs ``robot_follow.main`` while saving console output, CSV data,
the exact command, and metadata under one run directory whose name includes the
test environment, test type, and local timestamp. A single zip archive can be
created with ``--zip`` when needed for transfer or sharing.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import traceback
import zipfile
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, TextIO

from robot_follow.main import main as robot_follow_main


@dataclass(frozen=True)
class RunLogPaths:
    """Filesystem paths created for one logged test run."""

    run_id: str
    run_dir: Path
    csv_path: Path
    console_log_path: Path
    command_path: Path
    metadata_path: Path
    archive_path: Path


class Tee:
    """Write text to multiple text streams."""

    def __init__(self, *streams: TextIO) -> None:
        self._streams = streams

    def write(self, data: str) -> int:
        for stream in self._streams:
            stream.write(data)
            stream.flush()
        return len(data)

    def flush(self) -> None:
        for stream in self._streams:
            stream.flush()


def slugify(value: str) -> str:
    """Return a stable filename-safe slug."""
    slug = re.sub(r"[^A-Za-z0-9]+", "_", value.strip()).strip("_").lower()
    if not slug:
        raise ValueError("slug value cannot be empty")
    return slug


def build_run_log_paths(
    *,
    logs_root: Path,
    test_env: str,
    test_type: str,
    timestamp: datetime | None = None,
) -> RunLogPaths:
    """Build canonical paths for one run.

    Filenames include ``timestamp``, ``test_env``, and ``test_type``.
    """
    now = timestamp or datetime.now()
    ts = now.strftime("%Y%m%d_%H%M%S")
    env_slug = slugify(test_env)
    type_slug = slugify(test_type)
    run_id = f"{ts}_{env_slug}_{type_slug}"
    run_dir = logs_root / "runs" / run_id
    return RunLogPaths(
        run_id=run_id,
        run_dir=run_dir,
        csv_path=run_dir / f"{run_id}_data.csv",
        console_log_path=run_dir / f"{run_id}_console.log",
        command_path=run_dir / f"{run_id}_command.txt",
        metadata_path=run_dir / f"{run_id}_metadata.json",
        archive_path=logs_root / f"{run_id}.zip",
    )


def strip_csv_args(argv: list[str]) -> list[str]:
    """Remove user-provided ``--csv`` so logged filenames stay canonical."""
    stripped: list[str] = []
    skip_next = False
    for arg in argv:
        if skip_next:
            skip_next = False
            continue
        if arg == "--csv":
            skip_next = True
            continue
        if arg.startswith("--csv="):
            continue
        stripped.append(arg)
    return stripped


def robot_args_with_csv(robot_args: list[str], csv_path: Path) -> list[str]:
    """Return robot_follow.main args with the generated CSV path appended."""
    return [*strip_csv_args(robot_args), "--csv", str(csv_path)]


def write_command_file(path: Path, argv: list[str]) -> None:
    """Persist the exact command in a shell-readable single line."""
    path.write_text(" ".join(_shell_quote(arg) for arg in argv) + "\n", encoding="utf-8")


def write_metadata(
    path: Path,
    *,
    run_id: str,
    test_env: str,
    test_type: str,
    robot_args: list[str],
    csv_path: Path,
    console_log_path: Path,
    exit_code: int,
    archive_path: Path | None = None,
) -> None:
    """Write structured metadata for later analysis."""
    payload = {
        "run_id": run_id,
        "test_env": test_env,
        "test_type": test_type,
        "robot_args": robot_args,
        "csv_path": str(csv_path),
        "console_log_path": str(console_log_path),
        "archive_path": str(archive_path) if archive_path is not None else None,
        "exit_code": exit_code,
    }
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _shell_quote(value: str) -> str:
    if re.fullmatch(r"[A-Za-z0-9_@%+=:,./-]+", value):
        return value
    return "'" + value.replace("'", "'\"'\"'") + "'"


def write_archive(paths: RunLogPaths) -> None:
    """Create a single zip file containing all run artifacts."""
    with zipfile.ZipFile(paths.archive_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in (
            paths.csv_path,
            paths.console_log_path,
            paths.command_path,
            paths.metadata_path,
        ):
            if path.exists():
                archive.write(path, arcname=path.name)


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run robot_follow.main and store console output, CSV data, command, "
            "and metadata under logs/runs/<timestamp>_<env>_<type>/. Use "
            "--zip to also create logs/<timestamp>_<env>_<type>.zip."
        )
    )
    parser.add_argument("--test-env", required=True,
                        help="Test environment label, e.g. local, jetson, lab_a.")
    parser.add_argument("--test-type", required=True,
                        help="Test type label, e.g. alx_dry_run, closed_loop.")
    parser.add_argument("--logs-root", type=Path, default=Path("logs"),
                        help="Root log directory (default: logs).")
    parser.add_argument("--zip", action="store_true", default=False,
                        help="Also create a single zip archive under logs/.")
    parser.add_argument(
        "robot_args",
        nargs=argparse.REMAINDER,
        help="Arguments passed to robot_follow.main after '--'.",
    )
    args = parser.parse_args(argv)
    if args.robot_args and args.robot_args[0] == "--":
        args.robot_args = args.robot_args[1:]
    return args


def run_logged_test(args: argparse.Namespace) -> int:
    paths = build_run_log_paths(
        logs_root=args.logs_root,
        test_env=args.test_env,
        test_type=args.test_type,
    )
    paths.run_dir.mkdir(parents=True, exist_ok=False)

    robot_args = robot_args_with_csv(list(args.robot_args), paths.csv_path)
    wrapper_command = [
        "python",
        "-m",
        "robot_follow.run_logger",
        "--test-env",
        args.test_env,
        "--test-type",
        args.test_type,
        "--logs-root",
        str(args.logs_root),
        "--",
        *robot_args,
    ]
    write_command_file(paths.command_path, wrapper_command)

    exit_code = 0
    with paths.console_log_path.open("w", encoding="utf-8") as console_file:
        tee_out = Tee(sys.stdout, console_file)
        tee_err = Tee(sys.stderr, console_file)
        with redirect_stdout(tee_out), redirect_stderr(tee_err):
            print(f"run_id={paths.run_id}")
            print(f"run_dir={paths.run_dir}")
            print(f"csv={paths.csv_path}")
            print(f"console_log={paths.console_log_path}")
            if args.zip:
                print(f"archive={paths.archive_path}")
            print("robot_args=" + " ".join(_shell_quote(arg) for arg in robot_args))
            try:
                robot_follow_main(robot_args)
            except SystemExit as exc:
                exit_code = int(exc.code) if isinstance(exc.code, int) else 1
            except Exception:
                exit_code = 1
                traceback.print_exc()

    write_metadata(
        paths.metadata_path,
        run_id=paths.run_id,
        test_env=args.test_env,
        test_type=args.test_type,
        robot_args=robot_args,
        csv_path=paths.csv_path,
        console_log_path=paths.console_log_path,
        archive_path=paths.archive_path if args.zip else None,
        exit_code=exit_code,
    )
    if args.zip:
        write_archive(paths)
    return exit_code


def main(argv: Iterable[str] | None = None) -> None:
    args = parse_args(argv)
    raise SystemExit(run_logged_test(args))


if __name__ == "__main__":
    main()
