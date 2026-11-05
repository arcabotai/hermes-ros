"""Registration contract: what the plugin tells Hermes it provides must match what it registers."""
import inspect
from pathlib import Path

import pytest
import yaml

import hermes_ros
from hermes_ros import config, schemas

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = yaml.safe_load((ROOT / "plugin.yaml").read_text())


class FakeState:
    def __init__(self, d):
        self.data_dir = d


class FakeCtx:
    def __init__(self, tmp_path, cfg=None):
        self.tools, self.hooks, self.commands, self.skills, self.unload = {}, {}, {}, {}, []
        self.state = FakeState(tmp_path)
        self._cfg = cfg or {}

    def get_config(self, key, default=None):
        return self._cfg.get(key, default)

    def register_tool(self, **kw):
        self.tools[kw["name"]] = kw

    def register_hook(self, name, fn):
        self.hooks.setdefault(name, []).append(fn)

    def register_command(self, name, handler, **kw):
        self.commands[name] = (handler, kw)

    def register_skill(self, name, path, description=""):
        self.skills[name] = Path(path)

    def on_unload(self, fn):
        self.unload.append(fn)


@pytest.fixture
def ctx(tmp_path):
    c = FakeCtx(tmp_path)
    hermes_ros.register(c)
    return c


def test_registered_tools_match_manifest(ctx):
    assert sorted(ctx.tools) == sorted(MANIFEST["provides_tools"]) and len(ctx.tools) == 9


def test_every_tool_has_schema_handler_toolset(ctx):
    for name, t in ctx.tools.items():
        assert t["toolset"] == "ros" and callable(t["handler"])
        sch = t["schema"]
        assert sch["name"] == name and len(sch["description"]) > 40
        assert sch["parameters"]["type"] == "object"
        sig = inspect.signature(t["handler"])
        assert any(p.kind is p.VAR_KEYWORD for p in sig.parameters.values())  # handler(args, **kwargs)


def test_required_params_exist_in_properties():
    for s in schemas.ALL:
        props = s["parameters"].get("properties", {})
        for req in s["parameters"].get("required", []):
            assert req in props, (s["name"], req)


def test_pre_tool_call_hook_registered_and_accepts_kwargs(ctx):
    assert MANIFEST["provides_hooks"] == ["pre_tool_call"] and len(ctx.hooks["pre_tool_call"]) == 1
    hook = ctx.hooks["pre_tool_call"][0]
    assert any(p.kind is p.VAR_KEYWORD for p in inspect.signature(hook).parameters.values())
    assert hook(tool_name="web_search", args={}, task_id="t", session_id="s") is None  # never touches other tools
    assert hook(tool_name="ros_drive", args={"linear": 9, "duration_s": 1}, task_id="t")["action"] == "block"


def test_skill_and_command_registered(ctx):
    assert ctx.skills["ros-safe-operation"].read_text().startswith("---")
    assert "ros" in ctx.commands


def test_register_opens_no_connections(tmp_path, monkeypatch):
    import socket

    def no_net(*a, **k):
        raise AssertionError("register() must not touch the network")
    monkeypatch.setattr(socket.socket, "connect", no_net)
    monkeypatch.setattr(socket, "create_connection", no_net)
    hermes_ros.register(FakeCtx(tmp_path, {"backend": "rosbridge"}))


def test_manifest_basics():
    assert MANIFEST["name"] == "hermes-ros" and MANIFEST["manifest_version"] == 2
    assert MANIFEST["license"] == "MIT" and (ROOT / "LICENSE").exists()
    for dep in MANIFEST["python_dependencies"]:
        assert "<" in dep, "dependencies need an upper bound (catalog rule)"


def test_config_schema_defaults_equal_runtime_defaults():
    schema = MANIFEST["config_schema"]
    assert set(schema) == set(config.DEFAULTS)
    for key, spec in schema.items():
        assert spec["default"] == config.DEFAULTS[key], key


def test_user_config_overrides_are_merged_over_defaults(tmp_path):
    from hermes_ros.runtime import Runtime
    rt = Runtime(lambda k, d=None: {"safety": {"max_linear_mps": 0.1}}.get(k, d), tmp_path)
    s = rt.settings.snapshot()
    assert s["safety"]["max_linear_mps"] == 0.1 and s["safety"]["max_angular_rps"] == 1.0  # partial dicts keep other defaults
    assert s["backend"] == "mock"


def test_broken_config_values_fall_back_safely(tmp_path):
    from hermes_ros.runtime import Runtime
    rt = Runtime(lambda k, d=None: (_ for _ in ()).throw(RuntimeError("bad config")), tmp_path)
    assert rt.settings.snapshot()["safety"]["max_linear_mps"] == 0.5


def test_slash_command_human_controls(ctx):
    handler, _ = ctx.commands["ros"]
    assert "backend" in handler("status")
    assert "LATCHED" in handler("estop too close to the wall")
    status = handler("status")
    assert "LATCHED" in status and "too close" in status
    assert hermes_ros.get_runtime().interlock.latched
    assert "cleared" in handler("clear")
    assert not hermes_ros.get_runtime().interlock.latched
    assert "No blocked" in handler("log")
    hermes_ros.get_runtime().interlock.record_block("ros_drive", {"linear": 9}, "over_speed_linear", "too fast")
    assert "over_speed_linear" in handler("log 5")
    assert "MEDIA:" in handler("snapshot")
    assert "unknown subcommand" in handler("frobnicate")
