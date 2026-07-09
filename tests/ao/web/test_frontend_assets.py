from pathlib import Path


FRONTEND = Path("src/robot_follow/ao/web/frontend")


def test_frontend_sources_exist():
    assert (FRONTEND / "package.json").exists()
    assert (FRONTEND / "tsconfig.json").exists()
    assert (FRONTEND / "index.html").exists()
    assert (FRONTEND / "src/main.ts").exists()
    assert (FRONTEND / "src/styles.css").exists()


def test_frontend_uses_websocket_and_pointer_events():
    main = (FRONTEND / "src/main.ts").read_text(encoding="utf-8")
    assert "new WebSocket" in main
    assert "pointerdown" in main
    assert "select_target" in main
