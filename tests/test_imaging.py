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
