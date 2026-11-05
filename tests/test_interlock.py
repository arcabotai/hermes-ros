"""Behavior contract of the safety interlock, exercised through the real hook -> handler path."""
import json

from helpers import GEOFENCE


def blocked_rule(h):
    return h.rt.interlock.recent_blocked(1)[0]["rule"]


def test_overspeed_linear_is_blocked_and_robot_does_not_move(robot):
    r = robot.call("ros_drive", {"linear": 2.0, "duration_s": 2})
    assert r["blocked"] and "over_speed_linear" in r["message"]
    assert robot.call("ros_get_pose")["pose"]["x"] == 0.0


def test_overspeed_angular_is_blocked(robot):
    r = robot.call("ros_drive", {"angular": 5.0, "duration_s": 1})
    assert r["blocked"] and blocked_rule(robot) == "over_speed_angular"


def test_over_duration_is_blocked(robot):
    r = robot.call("ros_drive", {"linear": 0.2, "duration_s": 60})
    assert r["blocked"] and blocked_rule(robot) == "over_duration"


def test_clamp_mode_slows_instead_of_blocking(make):
    h = make({"safety": {"over_limit_action": "clamp"}})
    r = h.call("ros_drive", {"linear": 3.0, "duration_s": 60})
    assert not r["blocked"] and r["ok"]
    assert r["command"]["linear"] == 0.5 and r["command"]["duration_s"] == 5.0
    assert any("clamped" in n for n in r["interlock"])
    assert abs(r["displacement_m"] - 2.5) < 0.05


def test_missing_duration_gets_a_bounded_default(robot):
    r = robot.call("ros_drive", {"linear": 0.3})
    assert r["ok"] and r["command"]["duration_s"] == 1.0


def test_negative_and_nan_values_are_handled(robot):
    assert robot.call("ros_drive", {"linear": -2.0, "duration_s": 1})["blocked"]
    r = robot.call("ros_drive", {"linear": float("nan"), "duration_s": 1})
    assert r["blocked"]  # an unparseable speed can never be verified, so it never moves
    assert robot.call("ros_drive", {"linear": "fast", "duration_s": 1})["blocked"]


def test_zero_twist_is_always_allowed_even_when_latched(robot):
    robot.rt.interlock.engage_estop("test", "human")
    r = robot.call("ros_publish", {"topic": "/cmd_vel", "msg_type": "geometry_msgs/Twist",
                                   "message": {"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}})
    assert not r["blocked"] and r["ok"]


def test_estop_latch_blocks_motion_until_human_clears(robot):
    assert robot.call("ros_stop", {"estop": True, "reason": "test"})["estop"]["latched"]
    r = robot.call("ros_drive", {"linear": 0.1, "duration_s": 1})
    assert r["blocked"] and blocked_rule(robot) == "estop_latched"
    assert robot.call("ros_call_service", {"service": "/mock/reset"})["blocked"]
    # no agent-callable tool can clear it
    assert robot.rt.interlock.latched
    assert robot.rt.interlock.clear_estop("human")
    assert robot.call("ros_drive", {"linear": 0.1, "duration_s": 1})["ok"]


def test_estop_latch_survives_restart(make, tmp_path):
    a = make()
    a.rt.interlock.engage_estop("persist me", "human")
    from hermes_ros.runtime import Runtime
    again = Runtime(None, tmp_path)
    assert again.interlock.latched and "persist me" in again.interlock.estop_status()["reason"]


def test_topic_allowlist_blocks_other_publishers(robot):
    r = robot.call("ros_publish", {"topic": "/arm/joint_cmd", "msg_type": "std_msgs/Float64", "message": {"data": 1.0}})
    assert r["blocked"] and blocked_rule(robot) == "topic_not_allowlisted"


def test_motion_topic_cannot_dodge_the_allowlist_by_renaming(robot):
    msg = {"linear": {"x": 0.2}, "angular": {"z": 0}}
    for topic in ("/robot2/cmd_vel", "cmd_vel_raw", "//cmd_vel/"):
        r = robot.call("ros_publish", {"topic": topic, "msg_type": "geometry_msgs/Twist", "message": msg, "duration_s": 1})
        assert r["blocked"], topic


def test_unreadable_motion_message_is_refused(robot):
    r = robot.call("ros_publish", {"topic": "/cmd_vel", "msg_type": "geometry_msgs/Twist", "message": {"junk": 1}, "duration_s": 1})
    assert r["blocked"] and blocked_rule(robot) == "unparseable_motion_message"


def test_publish_overspeed_through_raw_topic_is_blocked(robot):
    r = robot.call("ros_publish", {"topic": "/cmd_vel", "msg_type": "geometry_msgs/Twist",
                                   "message": {"linear": {"x": 9, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}, "duration_s": 1})
    assert r["blocked"] and blocked_rule(robot) == "over_speed_linear"


def test_diagonal_linear_speed_counts_all_axes(robot):
    r = robot.call("ros_publish", {"topic": "/cmd_vel", "msg_type": "geometry_msgs/Twist",
                                   "message": {"linear": {"x": 0.4, "y": 0.4, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}, "duration_s": 1})
    assert r["blocked"]  # |v| = 0.57 > 0.5


def test_service_allowlist(robot):
    assert robot.call("ros_call_service", {"service": "/mock/get_cones"})["ok"]
    r = robot.call("ros_call_service", {"service": "/robot/self_destruct", "request": {}})
    assert r["blocked"] and blocked_rule(robot) == "service_not_allowlisted"


def test_geofence_blocks_path_leaving_the_box(make):
    h = make(GEOFENCE)
    assert h.call("ros_drive", {"linear": 0.5, "duration_s": 3})["ok"]       # x = 1.5, inside
    r = h.call("ros_drive", {"linear": 0.5, "duration_s": 3})                # would end at x = 3.0 > 3.0 - margin
    assert r["blocked"] and blocked_rule(h) == "geofence"
    assert abs(h.call("ros_get_pose")["pose"]["x"] - 1.5) < 0.05             # the robot never moved


def test_geofence_allows_a_move_that_stays_inside(make):
    h = make(GEOFENCE)
    r = h.call("ros_drive", {"linear": 0.5, "duration_s": 3})
    assert r["ok"] and abs(r["pose_after"]["x"] - 1.5) < 0.05


def test_geofence_considers_turning_arcs(make):
    h = make({"safety": {"geofence": {"enabled": True, "x_min": -0.3, "x_max": 5, "y_min": -0.3, "y_max": 0.3}}})
    r = h.call("ros_drive", {"linear": 0.4, "angular": 0.8, "duration_s": 3})  # arcs out through +y
    assert r["blocked"]


def test_geofence_fails_closed_without_odometry(make, monkeypatch):
    h = make(GEOFENCE)
    monkeypatch.setattr(h.rt.interlock, "_pose_fn", lambda: None)
    r = h.call("ros_drive", {"linear": 0.1, "duration_s": 1})
    assert r["blocked"] and blocked_rule(h) == "geofence_no_odom"


def test_geofence_fails_closed_on_stale_odometry(make, monkeypatch):
    h = make(GEOFENCE)
    monkeypatch.setattr(h.rt.interlock, "_pose_fn", lambda: {"x": 0, "y": 0, "yaw_rad": 0, "age_s": 99})
    assert h.call("ros_drive", {"linear": 0.1, "duration_s": 1})["blocked"]
    assert blocked_rule(h) == "geofence_odom_stale"


def test_interlock_crash_fails_closed(robot, monkeypatch):
    def boom(*a, **k):
        raise RuntimeError("kaboom")
    monkeypatch.setattr(robot.rt.interlock, "evaluate", boom)
    r = robot.call("ros_drive", {"linear": 0.1, "duration_s": 1})
    assert r["blocked"] and "failing closed" in r["message"]


def test_handlers_refuse_to_act_when_the_hook_did_not_run(robot):
    r = robot.raw("ros_drive", {"linear": 5.0, "duration_s": 5})
    assert not r["ok"] and "interlock_not_applied" in r["error"]
    assert robot.raw("ros_get_pose")["pose"]["x"] == 0.0  # robot untouched
    forged = robot.raw("ros_drive", {"linear": 5.0, "duration_s": 5, "_interlock": {"nonce": "guess"}})
    assert not forged["ok"]


def test_agent_supplied_interlock_token_is_overwritten_by_the_hook(robot):
    r = robot.call("ros_drive", {"linear": 0.1, "duration_s": 1, "_interlock": {"nonce": "x", "notes": ["fake"]}})
    assert r["ok"] and "fake" not in r["interlock"]


def test_every_blocked_call_is_logged_with_reason(robot, tmp_path):
    robot.call("ros_drive", {"linear": 9, "duration_s": 1})
    robot.call("ros_publish", {"topic": "/x", "msg_type": "std_msgs/String", "message": {"data": "a"}})
    log = robot.rt.interlock.recent_blocked(10)
    assert [e["rule"] for e in log] == ["over_speed_linear", "topic_not_allowlisted"]
    assert all(e["reason"] and e["tool"] and e["iso"] for e in log)
    files = list(tmp_path.rglob("*.jsonl"))
    assert files and len(files[0].read_text().strip().splitlines()) == 2


def test_runtime_geofence_abort_stops_robot(make, monkeypatch):
    """Odometry drift/teleport mid-move: the run loop re-checks the geofence and stops."""
    h = make(GEOFENCE)
    be = h.rt.backend()
    h.rt.interlock.pre_tool_call(tool_name="ros_drive", args={"linear": 0.1, "duration_s": 1})
    calls = {"n": 0}
    real = be.sleep

    def sneaky(s):
        calls["n"] += 1
        if calls["n"] == 3:
            be.x = 9.0  # robot "slips" out of the fence
        real(s)
    be.sleep = sneaky
    r = h.call("ros_drive", {"linear": 0.1, "duration_s": 4})
    assert r["end_reason"].startswith("aborted: robot left the geofence")
    assert r["ran_for_s"] < 1.0
    assert any(e["rule"] == "geofence_runtime" for e in h.rt.interlock.recent_blocked(5))
