"""Dashboard backend: reads what the plugin leaves on disk; read-only."""
import importlib.util
import json
from pathlib import Path

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")
from fastapi import FastAPI  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402

from conftest import ROOT, Harness  # noqa: E402


@pytest.fixture
def api(tmp_path, monkeypatch):
    home = tmp_path / "home"
    data = home / "plugin-data" / "agent-plugin-hermes-ros-abc123"
    data.mkdir(parents=True)
    monkeypatch.setenv("HERMES_HOME", str(home))
    spec = importlib.util.spec_from_file_location("hros_plugin_api", ROOT / "dashboard" / "plugin_api.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod, "_hermes_home", lambda: home)
    app = FastAPI()
    app.include_router(mod.router, prefix="/api/plugins/hermes-ros")
    return TestClient(app), data


def test_empty_state_is_valid(api):
    client, _ = api
    s = client.get("/api/plugins/hermes-ros/state").json()
    assert s["ok"] and s["blocked"] == [] and s["pose"] is None and s["snapshot"] is None
    assert s["limits"]["max_linear_mps"] == 0.5 and s["estop"] == {"latched": False}


def test_dashboard_shows_what_the_agent_did(api):
    client, data = api
    bot = Harness(data, {"camera": {"output_dir": ""}})
    assert bot.call("ros_drive", {"linear": 0.3, "duration_s": 1})["ok"]
    assert bot.call("ros_drive", {"linear": 3.0, "duration_s": 1})["blocked"]
    assert bot.call("ros_camera_snapshot")["ok"]
    bot.call("ros_stop", {"estop": True, "reason": "dashboard test"})
    s = client.get("/api/plugins/hermes-ros/state").json()
    assert s["pose"]["x"] > 0.2 and s["pose_age_s"] is not None
    assert s["blocked_total"] == 1 and s["blocked"][0]["rule"] == "over_speed_linear"
    assert s["estop"]["latched"] and s["estop"]["reason"] == "dashboard test"
    assert s["snapshot"]["data_url"].startswith("data:image/png;base64,")
    again = client.get("/api/plugins/hermes-ros/state", params={"since": s["snapshot"]["mtime"]}).json()
    assert "data_url" not in again["snapshot"]  # unchanged frame is not re-sent


def test_garbage_files_never_break_the_endpoint(api):
    client, data = api
    (data / "blocked.jsonl").write_text("not json\n{\"tool\": \"ros_drive\", \"rule\": \"x\", \"reason\": \"y\"}\n")
    (data / "estop.json").write_text("{broken")
    (data / "last_pose.json").write_text("[]")
    s = client.get("/api/plugins/hermes-ros/state").json()
    assert s["ok"] and len(s["blocked"]) == 1


def test_state_shows_wss_and_a_remote_plaintext_warning(tmp_path, monkeypatch):
    from hermes_ros.config import DEFAULTS, deep_merge

    def client_for(over):
        home = tmp_path / "home"
        data = home / "plugin-data" / "hermes-ros-tls"
        data.mkdir(parents=True, exist_ok=True)
        monkeypatch.setenv("HERMES_HOME", str(home))
        spec = importlib.util.spec_from_file_location("hros_plugin_api_tls", ROOT / "dashboard" / "plugin_api.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        monkeypatch.setattr(mod, "_hermes_home", lambda: home)
        monkeypatch.setattr(mod, "_settings", lambda: deep_merge(DEFAULTS, over))
        app = FastAPI()
        app.include_router(mod.router, prefix="/api/plugins/hermes-ros")
        return TestClient(app)

    remote = client_for({"backend": "rosbridge", "rosbridge": {"host": "robot.lan"}}).get("/api/plugins/hermes-ros/state").json()
    assert remote["rosbridge"] == "ws://robot.lan:9090" and remote["encrypted"] is False
    assert "public internet" in remote["transport_warning"] and "plaintext" in remote["transport_warning"]

    secure = client_for({"backend": "rosbridge", "rosbridge": {"host": "robot.lan", "tls": True}}).get(
        "/api/plugins/hermes-ros/state").json()
    assert secure["rosbridge"] == "wss://robot.lan:9090" and secure["encrypted"] is True
    assert secure["transport_warning"] is None


def test_manifest_points_at_real_files():
    m = json.loads((ROOT / "dashboard" / "manifest.json").read_text())
    for key in ("entry", "css", "api"):
        assert (ROOT / "dashboard" / m[key]).is_file(), key
    assert m["name"] == "hermes-ros" and m["tab"]["path"].startswith("/")
