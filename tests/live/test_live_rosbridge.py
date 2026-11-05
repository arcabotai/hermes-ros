"""End-to-end against a REAL rosbridge + ROS 2 (turtlesim + synthetic camera). Opt-in.

    HERMES_ROS_LIVE=localhost:9090 pytest tests/live

Needs the demo stack from docker/ (or scripts/blaxel_e2e.py, which builds it in a Blaxel sandbox).
"""
import json
import math
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from helpers import red_centroid, red_pixels  # noqa: E402

LIVE = os.environ.get("HERMES_ROS_LIVE")
pytestmark = pytest.mark.skipif(not LIVE, reason="set HERMES_ROS_LIVE=host:port to run against a live rosbridge")

CONE = (8.5, 7.5)


def overrides():
    host, _, port = (LIVE or "localhost:9090").partition(":")
    return {
        "backend": "rosbridge",
        "rosbridge": {"host": host, "port": int(port or 9090)},
        "topics": {"cmd_vel": "/turtle1/cmd_vel", "odom": "/turtle1/pose", "odom_type": "turtlesim/msg/Pose",
                   "camera": "/camera/image_raw"},
        "safety": {"max_linear_mps": 1.0, "max_angular_rps": 1.5, "max_duration_s": 6.0,
                   "write_allowlist": {"topics": ["/turtle1/cmd_vel"], "services": ["/turtle1/teleport_absolute"]},
                   "geofence": {"enabled": True, "x_min": 1.0, "x_max": 10.0, "y_min": 1.0, "y_max": 10.0,
                                "margin_m": 0.2}},
    }


@pytest.fixture(scope="module")
def bot(tmp_path_factory):
    from conftest import Harness
    h = Harness(tmp_path_factory.mktemp("live"), overrides())
    yield h
    h.rt.shutdown()


def teleport(bot, x, y, th):
    r = bot.call("ros_call_service", {"service": "/turtle1/teleport_absolute", "request": {"x": x, "y": y, "theta": th}})
    assert r["ok"], r


def pose(bot):
    r = bot.call("ros_get_pose")
    assert r["ok"], r
    return r["pose"]


def test_discovery_sees_the_turtlesim_graph(bot):
    r = bot.call("ros_list")
    assert r["ok"] and r["backend"] == "rosbridge"
    topics = {t["name"]: t["type"] for t in r["topics"]}
    assert "/turtle1/cmd_vel" in topics and "/turtle1/pose" in topics and "/camera/image_raw" in topics
    assert "Twist" in topics["/turtle1/cmd_vel"]
    assert any("turtlesim" in n for n in r["nodes"])
    assert any(s["name"] == "/turtle1/teleport_absolute" for s in r["services"])


def test_echo_and_pose(bot):
    teleport(bot, 5.5, 5.5, 0.0)
    echo = bot.call("ros_echo", {"topic": "/turtle1/pose"})
    assert echo["ok"] and abs(echo["message"]["x"] - 5.5) < 0.05
    p = pose(bot)
    assert abs(p["x"] - 5.5) < 0.05 and abs(p["yaw_rad"]) < 0.05 and p["age_s"] < 2


def test_camera_snapshot_from_real_image_topic(bot):
    teleport(bot, 5.5, 5.5, 0.0)
    r = bot.call("ros_camera_snapshot")
    assert r["ok"], r
    data = Path(r["path"]).read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n" and (r["width"], r["height"]) == (320, 240)
    assert "Image" in r["source"]
    comp = bot.call("ros_camera_snapshot", {"topic": "/camera/image_raw/compressed"})
    assert comp["ok"] and comp["mime"] == "image/png"


def test_drive_changes_real_pose(bot):
    teleport(bot, 3.0, 5.5, 0.0)
    r = bot.call("ros_drive", {"linear": 1.0, "duration_s": 2.0})
    assert r["ok"] and r["end_reason"] == "completed", r
    assert 1.4 < r["displacement_m"] < 2.4, r["displacement_m"]
    assert pose(bot)["x"] > 4.5


def test_robot_actually_stops_after_command(bot):
    teleport(bot, 3.0, 5.5, 0.0)
    bot.call("ros_drive", {"linear": 1.0, "duration_s": 1.0})
    a = pose(bot)["x"]
    bot.rt.backend().sleep(1.0)
    assert abs(pose(bot)["x"] - a) < 0.05


def test_interlock_blocks_overspeed_and_turtle_does_not_move(bot):
    teleport(bot, 3.0, 5.5, 0.0)
    r = bot.call("ros_drive", {"linear": 8.0, "duration_s": 3})
    assert r["blocked"] and "over_speed_linear" in r["message"]
    bot.rt.backend().sleep(0.5)
    assert abs(pose(bot)["x"] - 3.0) < 0.05


def test_geofence_with_live_pose_blocks_wall_run(bot):
    teleport(bot, 8.0, 5.5, 0.0)
    r = bot.call("ros_drive", {"linear": 1.0, "duration_s": 5.0})  # would reach x=13
    assert r["blocked"] and "geofence" in r["message"]
    assert abs(pose(bot)["x"] - 8.0) < 0.05


def test_topic_and_service_allowlists(bot):
    r = bot.call("ros_publish", {"topic": "/turtle2/cmd_vel", "msg_type": "geometry_msgs/Twist",
                                 "message": {"linear": {"x": 0.1}, "angular": {"z": 0}}, "duration_s": 1})
    assert r["blocked"]
    assert bot.call("ros_call_service", {"service": "/reset"})["blocked"]


def test_estop_latch_end_to_end(bot):
    teleport(bot, 3.0, 5.5, 0.0)
    assert bot.call("ros_stop", {"estop": True, "reason": "live test"})["ok"]
    assert bot.call("ros_drive", {"linear": 0.2, "duration_s": 1})["blocked"]
    bot.rt.interlock.clear_estop("human")
    assert bot.call("ros_drive", {"linear": 0.2, "duration_s": 1})["ok"]


def test_drive_to_the_cone_by_looking(bot):
    """Closed loop like the agent does: snapshot -> bearing -> turn -> advance, verified by the real camera feed."""
    teleport(bot, 4.0, 4.0, 0.0)
    first = bot.call("ros_camera_snapshot")
    assert red_pixels(Path(first["path"]).read_bytes()) > 0, "cone should be visible from the start pose"
    for _ in range(6):
        p = pose(bot)
        dx, dy = CONE[0] - p["x"], CONE[1] - p["y"]
        dist = math.hypot(dx, dy)
        if dist < 1.0:
            break
        err = math.atan2(math.sin(math.atan2(dy, dx) - p["yaw_rad"]), math.cos(math.atan2(dy, dx) - p["yaw_rad"]))
        if abs(err) > 0.08:
            assert bot.call("ros_drive", {"angular": 1.0 if err > 0 else -1.0, "duration_s": min(abs(err) / 1.0, 3.0)})["ok"]
        bot.call("ros_drive", {"linear": 1.0, "duration_s": min(max(dist - 0.8, 0.3), 3.0)})
    p = pose(bot)
    assert math.hypot(CONE[0] - p["x"], CONE[1] - p["y"]) < 1.3, p
    last = bot.call("ros_camera_snapshot")
    png = Path(last["path"]).read_bytes()
    assert red_pixels(png) > red_pixels(Path(first["path"]).read_bytes()) * 2
    assert abs(red_centroid(png) - 160) < 40  # roughly centred
