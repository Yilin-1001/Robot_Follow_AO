---
name: robot-follow-conda-run-guard
description: Use when working in the Robot_Follow project before running project code, tests, scripts, Python commands, package installs, or hardware-facing commands that depend on the Python runtime.
---

# Robot Follow Conda Run Guard

## Purpose

Before executing Robot_Follow code, verify the current Python runtime and ask the user which conda environment from `conda env list` should be used.

This project may run against hardware-facing code, serial protocols, and Python dependencies where the wrong environment can produce misleading failures or affect connected devices.

## Required Pre-Run Check

Before every project-code execution, do this first:

1. Run:

```bash
conda env list
```

2. Run:

```bash
echo "$CONDA_DEFAULT_ENV"
which python
python --version
```

3. Report the active environment, Python path, and Python version.

4. Ask the user which environment from `conda env list` to use.

5. Wait for the user's answer before running the code command.

## Running Commands After Confirmation

Use the selected environment explicitly:

```bash
conda run -n <env-name> <command>
```

Examples:

```bash
conda run -n robot_follow pytest
conda run -n robot_follow python -m robot_follow.serial_reader
conda run -n robot_follow python scripts/check_device.py
```

Do not rely on a previously activated shell environment unless the user explicitly tells you to use it.

## When To Repeat Confirmation

Repeat the pre-run check when:

- Starting a new Claude Code session.
- Running project code after switching tasks.
- Running tests, Python scripts, package installs, or hardware-facing commands.
- The user asks to run code and no environment has been confirmed in the current conversation.
- The active environment or Python path differs from the previously confirmed environment.

If the user has already confirmed an environment in the current conversation and the active context has not changed, briefly state that confirmed environment before running:

```text
Using previously confirmed conda environment: <env-name>.
```

## If Conda Is Unavailable

If `conda env list` fails or `conda` is unavailable:

1. Do not run the project code.
2. Report the failure.
3. Ask the user how they want to provide or activate the correct environment.

## Scope

This guard applies to commands that execute or modify the Python runtime, including:

- `python ...`
- `pytest ...`
- `pip install ...`
- `python -m ...`
- project scripts
- commands that talk to serial devices, robot hardware, sensors, or controllers

It does not apply to read-only inspection commands such as `ls`, `rg`, `sed`, `git status`, or reading documentation.
