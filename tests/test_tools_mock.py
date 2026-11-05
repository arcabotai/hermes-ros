"""Tools against the built-in mock robot (no ROS, no network)."""
import json
import math
import struct
import zlib

import pytest


def decode_png(data):
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, idat = 8, b""
    w = h = 0
    while pos < len(data):
        n, tag = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + n]
        if tag == b"IHDR":
            w, h = struct.unpack(">II", body[:8])
        elif tag == b"IDAT":
            idat += body
        pos += 12 + n
    raw = zlib.decompress(idat)
    stride = 1 + w * 3
    px = []
    for row in range(h):
        assert raw[row * stride] == 0  # filter None (our encoder)
        px.append(raw[row * stride + 1:(row + 1) * stride])
    return w, h, px


def red_pixels(png):
    w, h, rows = decode_png(png)
    n = 0
    for r in rows:
        for i in range(0, len(r), 3):
            if r[i] > 180 and r[i + 1] < 90 and r[i + 2] < 90:
                n += 1
    return n


def red_centroid(png):
    w, h, rows = decode_png(png)
    cols = [i // 3 for r in rows for i in range(0, len(r), 3) if r[i] > 180 and r[i + 1] < 90 and r[i + 2] < 90]
    return sum(cols) / len(cols)


def snapshot_bytes(h):
    from pathlib import Path
    res = h.call("ros_camera_snapshot")
    assert res["ok"], res
    return res, Path(res["path"]).read_bytes()


def test_list_topics_services_nodes(robot):
    res = robot.call("ros_list")
    names = {t["name"] for t in res["topics"]}
    assert {"/cmd_vel", "/odom", "/camera/image_raw"} <= names
    assert any(s["name"] == "/mock/reset" for s in res["services"])
    assert res["nodes"] and res["backend"] == "mock"
    only = robot.call("ros_list", {"kind": "topics", "filter": "odom"})
    assert [t["name"] for t in only["topics"]] == ["/odom"] and "services" not in only


def test_echo_odom_and_summarises_big_messages(robot):
    res = robot.call("ros_echo", {"topic": "/odom"})
    assert res["message"]["pose"]["pose"]["position"]["x"] == 0.0
    big = robot.call("ros_echo", {"topic": "/camera/image_raw"})
    assert len(json.dumps(big)) < 3000  # image bytes are summarised, not dumped into the context
    with pytest.raises(AssertionError):
        assert robot.call("ros_echo", {"topic": "/nope"})["ok"]


def test_get_pose_reports_xy_yaw(robot):
    pose = robot.call("ros_get_pose")["pose"]
    assert (pose["x"], pose["y"], pose["yaw_deg"]) == (0.0, 0.0, 0.0)


def test_drive_straight_changes_pose_as_kinematics_predict(robot):
    r = robot.call("ros_drive", {"linear": 0.4, "duration_s": 2.5})
    assert r["ok"] and r["end_reason"] == "completed"
    assert r["pose_after"]["x"] == pytest.approx(1.0, abs=0.03) and abs(r["pose_after"]["y"]) < 0.01
    assert r["displacement_m"] == pytest.approx(1.0, abs=0.03)


def test_turn_in_place_changes_heading(robot):
    r = robot.call("ros_drive", {"angular": 0.5, "duration_s": math.pi / 2 / 0.5})
    assert r["pose_after"]["yaw_deg"] == pytest.approx(90, abs=3)
    assert r["displacement_m"] < 0.01


def test_robot_is_stopped_after_a_timed_command(robot):
    robot.call("ros_drive", {"linear": 0.4, "duration_s": 1})
    p1 = robot.call("ros_get_pose")["pose"]
    robot.rt.backend().sleep(3)  # time passes with no command
    p2 = robot.call("ros_get_pose")["pose"]
    assert p1["x"] == pytest.approx(p2["x"], abs=1e-6)


def test_wall_collision_stops_the_robot_and_reports_it(make):
    h = make({"safety": {"max_duration_s": 60}})
    r = None
    for _ in range(4):
        r = h.call("ros_drive", {"linear": 0.5, "duration_s": 5})
    assert r["collided"] and "wall" in r["collided"]
    assert r["pose_after"]["x"] < 5.0  # never through the wall


def test_cone_is_solid(make):
    h = make({"mock": {"cones": [{"x": 1.0, "y": 0.0}]}})
    r = h.call("ros_drive", {"linear": 0.5, "duration_s": 4})
    assert r["collided"] and "cone" in r["collided"]
    assert r["pose_after"]["x"] < 1.0


def test_drive_to_the_red_cone_end_to_end(robot):
    """The demo storyline: look, turn toward the cone, drive up to it, confirm with the camera."""
    _, before = snapshot_bytes(robot)
    assert red_pixels(before) > 0 and red_centroid(before) < 110  # cone is in view, off to the left of centre (+y is left)
    bearing = math.atan2(2.0, 3.0)
    robot.call("ros_drive", {"angular": 0.5, "duration_s": bearing / 0.5})
    _, facing = snapshot_bytes(robot)
    assert abs(red_centroid(facing) - 160) < 25  # after turning toward it, it is centred in the picture
    robot.call("ros_drive", {"linear": 0.5, "duration_s": 5})  # 2.5 m
    robot.call("ros_drive", {"linear": 0.3, "duration_s": 2})
    pose = robot.call("ros_get_pose")["pose"]
    dist = math.hypot(3.0 - pose["x"], 2.0 - pose["y"])
    assert 0.3 < dist < 0.8, dist
    _, close = snapshot_bytes(robot)
    assert red_pixels(close) > red_pixels(facing) * 3  # much bigger in view


def test_camera_snapshot_is_a_valid_png_file_with_media_hints(robot):
    res, data = snapshot_bytes(robot)
    w, h, _ = decode_png(data)
    assert (w, h) == (res["width"], res["height"]) == (320, 240)
    assert res["mime"] == "image/png" and res["media_tag"] == f"MEDIA:{res['path']}"
    assert "vision_analyze" in res["next_step"]


def test_camera_snapshots_are_pruned(make):
    h = make({"camera": {"keep_last": 3}})
    for _ in range(6):
        h.call("ros_camera_snapshot")
    assert len(list(h.rt.snapshot_dir().glob("ros-snapshot-*"))) == 3


def test_camera_inline_mode_returns_multimodal_envelope(make):
    h = make({"camera": {"return_mode": "inline"}})
    out = h.handlers["ros_camera_snapshot"]({})
    assert out["_multimodal"] and out["content"][1]["image_url"]["url"].startswith("data:image/png;base64,")


def test_service_calls(robot):
    assert robot.call("ros_call_service", {"service": "/mock/get_cones"})["response"]["cones"] == [{"x": 3.0, "y": 2.0}]
    robot.call("ros_drive", {"linear": 0.4, "duration_s": 2})
    assert robot.call("ros_call_service", {"service": "/mock/reset"})["response"]["success"]
    assert robot.call("ros_get_pose")["pose"]["x"] == 0.0
    tp = robot.call("ros_call_service", {"service": "/mock/teleport", "request": {"x": 1.0, "y": 1.0, "yaw": 0.0}})
    assert tp["response"]["success"]


def test_unknown_service_is_a_clean_error_not_a_crash(make):
    h = make({"safety": {"write_allowlist": {"topics": ["/cmd_vel"], "services": ["*"]}}})
    r = h.call("ros_call_service", {"service": "/nope"})
    assert r["ok"] is False and "unknown service" in r["error"]


def test_publish_non_motion_topic_when_allowlisted(make):
    h = make({"safety": {"write_allowlist": {"topics": ["/cmd_vel", "/chatter"], "services": []}}})
    r = h.call("ros_publish", {"topic": "/chatter", "msg_type": "std_msgs/String", "message": {"data": "hi"}})
    assert r["ok"] and r["published"] == 1


def test_publish_timed_twist_moves_robot(robot):
    r = robot.call("ros_publish", {"topic": "/cmd_vel", "msg_type": "geometry_msgs/Twist", "duration_s": 2, "rate_hz": 10,
                                   "message": {"linear": {"x": 0.25, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}})
    assert r["ok"] and r["published"] == 20
    assert robot.call("ros_get_pose")["pose"]["x"] == pytest.approx(0.5, abs=0.03)


def test_stop_without_latch_keeps_motion_allowed(robot):
    assert robot.call("ros_stop")["ok"]
    assert robot.call("ros_drive", {"linear": 0.1, "duration_s": 1})["ok"]


def test_status_summarises_everything_a_human_needs(robot):
    robot.call("ros_drive", {"linear": 9})
    s = robot.call("ros_status")
    assert s["limits"]["max_linear_mps"] == 0.5 and s["estop"]["latched"] is False
    assert s["counters"]["blocked"] == 1 and s["recent_blocked"][0]["rule"] == "over_speed_linear"


def test_mock_state_persists_between_runtimes(make, tmp_path):
    a = make()
    a.call("ros_drive", {"linear": 0.4, "duration_s": 2})
    from hermes_ros.runtime import Runtime
    from hermes_ros.tools import make_handlers
    rt2 = Runtime(None, tmp_path)
    pose = json.loads(make_handlers(rt2)["ros_get_pose"]({}))["pose"]
    assert pose["x"] == pytest.approx(0.8, abs=0.03)  # a new CLI process resumes where the robot was


def test_unknown_backend_is_a_clean_error(make):
    h = make({"backend": "warp-drive"})
    r = h.call("ros_get_pose")
    assert r["ok"] is False and "unknown backend" in r["error"]


def test_tools_never_raise_on_garbage_args(robot):
    for tool in robot.handlers:
        for bad in (None, [], "x", {"topic": 5, "linear": [1], "duration_s": "soon"}):
            out = robot.handlers[tool](bad) if tool not in ("ros_drive", "ros_publish", "ros_call_service") else robot.handlers[tool](bad)
            assert isinstance(out, (str, dict))
