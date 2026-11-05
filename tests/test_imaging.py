from hermes_ros import imaging
from hermes_ros.backend_rosbridge import parse_pose


def test_png_roundtrip_header():
    png = imaging.encode_png(4, 2, bytes(range(24)))
    assert imaging.png_size(png) == (4, 2) and imaging.sniff_image(png)[:2] == ("image/png", "png")


def test_ros_image_message_rgb8_and_bgr8_and_mono_decode():
    import base64
    rgb = bytes([255, 0, 0, 0, 255, 0])  # red, green pixels
    msg = {"width": 2, "height": 1, "encoding": "rgb8", "step": 6, "is_bigendian": 0, "data": base64.b64encode(rgb).decode()}
    assert imaging.png_size(imaging.image_msg_to_png(msg)) == (2, 1)
    msg2 = dict(msg, encoding="bgr8")
    assert imaging.image_msg_to_png(msg2) != imaging.image_msg_to_png(msg)  # channel order handled
    mono = {"width": 2, "height": 1, "encoding": "mono8", "step": 2, "is_bigendian": 0, "data": base64.b64encode(b"\x00\xff").decode()}
    assert imaging.png_size(imaging.image_msg_to_png(mono)) == (2, 1)


def test_unsupported_encoding_is_a_clear_error():
    import pytest
    from hermes_ros.backend_base import BackendError
    with pytest.raises((BackendError, ValueError)):
        imaging.image_msg_to_png({"width": 1, "height": 1, "encoding": "bayer_rggb8", "step": 1, "data": "AA=="})


def test_parse_pose_odometry_quaternion():
    import math
    msg = {"header": {"frame_id": "odom"}, "child_frame_id": "base_link",
           "pose": {"pose": {"position": {"x": 1.5, "y": -2.0, "z": 0}, "orientation": {"x": 0, "y": 0, "z": math.sin(0.4), "w": math.cos(0.4)}}},
           "twist": {"twist": {"linear": {"x": 0.2}, "angular": {"z": 0.1}}}}
    p = parse_pose(msg)
    assert (p["x"], p["y"]) == (1.5, -2.0) and abs(p["yaw_rad"] - 0.8) < 1e-6 and p["linear_vel"] == 0.2


# --- camera / cone visibility (behaviour contract for the "drive to the red cone" demo) ---------------------------

import math  # noqa: E402


def _red_pixels(x, y, yaw, cones, half=5.0):
    buf = imaging.render_forward(x, y, yaw, half, half, cones, raw_rgb=True)
    return sum(1 for i in range(0, len(buf), 3) if buf[i] > 150 and buf[i + 1] < 80 and buf[i + 2] < 80)


def test_cone_in_view_shows_red_pixels():
    assert _red_pixels(0, 0, 0, [(3.5, 0.8)]) > 150          # ahead and slightly left
    assert _red_pixels(0, 0, math.atan2(0.8, 3.5), [(3.5, 0.8)]) > 150   # dead centre


def test_cone_out_of_view_shows_no_red():
    cone = [(3.5, 0.8)]
    assert _red_pixels(0, 0, math.pi, cone) == 0               # behind the robot
    assert _red_pixels(0, 0, -math.pi / 2, cone) == 0          # off to the side, outside the FOV
    assert _red_pixels(0, 0, 0, []) == 0                       # no cone in the world at all


def test_cone_hidden_by_geometry_is_not_drawn_through_a_wall():
    # cone outside the arena wall behind the wall as seen from the camera: must stay hidden
    assert _red_pixels(0, 0, 0, [(6.0, 0.0)]) == 0


def test_cone_outside_fov_edge_is_consistent_with_hfov():
    # bearing just inside / just outside half the horizontal FOV (1.5 rad)
    inside, outside = 0.70, 0.85
    c = [(3.0 * math.cos(0.0), 3.0 * math.sin(0.0))]
    assert _red_pixels(0, 0, -inside, c) > 0
    assert _red_pixels(0, 0, -outside, c) == 0


def test_default_spawn_has_the_cone_in_the_first_snapshot_and_it_stays_in_view_when_driving_straight(tmp_path):
    from conftest import Harness
    h = Harness(tmp_path)
    cfg = h.rt.settings.snapshot()["mock"]
    x, y, yaw = cfg["start"]["x"], cfg["start"]["y"], cfg["start"]["yaw"]
    cones = [(c["x"], c["y"]) for c in cfg["cones"]]
    assert _red_pixels(x, y, yaw, cones, cfg.get("half_x", 5.0)) > 150
    for step in (0.5, 1.0, 1.5, 2.0):   # driving straight ahead toward the cone
        assert _red_pixels(x + step, y, yaw, cones) > 0, step
