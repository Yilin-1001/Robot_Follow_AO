from pathlib import Path

import pytest

from robot_follow.ao.live.cli import build_runtime_from_config


def test_real_g1_requires_explicit_confirmation(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        "ao_radar:\n"
        "  output:\n"
        "    mode: g1\n"
        "    require_real_g1_confirmation: true\n"
        "    confirm_real_g1: false\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="confirm_real_g1"):
        build_runtime_from_config(path)
