"""Tool schemas (what the model reads)."""

_SAFETY = ("Writes pass through the hermes-ros safety interlock (speed/duration limits, optional geofence, "
           "write allowlist, e-stop latch); a refused call returns the rule and reason - do not retry the same call, "
           "adjust it or tell the user. ")

ROS_LIST = {
    "name": "ros_list",
    "description": "List what the connected robot exposes over ROS: topics (with message types), services and/or nodes. "
                   "Start here to discover cmd_vel, odom and camera topics. Read-only.",
    "parameters": {"type": "object", "properties": {
        "kind": {"type": "string", "enum": ["topics", "services", "nodes", "all"], "default": "all"},
        "filter": {"type": "string", "description": "Case-insensitive substring to filter names/types (e.g. 'cmd_vel', 'camera')."},
    }},
}

ROS_ECHO = {
    "name": "ros_echo",
    "description": "Read ONE message from a ROS topic and return it (large arrays/images are summarised; a LaserScan such as /scan also gets "
                   "`scan_summary`: nearest obstacle and the clearance in eight 45-degree sectors). Read-only.",
    "parameters": {"type": "object", "properties": {
        "topic": {"type": "string", "description": "Topic name, e.g. /odom"},
        "msg_type": {"type": "string", "description": "Optional message type; auto-detected when omitted."},
        "timeout_s": {"type": "number", "default": 5},
    }, "required": ["topic"]},
}

ROS_DRIVE = {
    "name": "ros_drive",
    "description": "Drive the robot: publish a velocity command (forward `linear` m/s, turn `angular` rad/s, + is left/CCW) "
                   "for `duration_s` seconds, then automatically send a stop. Prefer this over ros_publish for movement. "
                   "Use small, short moves and check ros_get_pose / ros_camera_snapshot after each. " + _SAFETY +
                   "Returns the pose before/after.",
    "parameters": {"type": "object", "properties": {
        "linear": {"type": "number", "description": "Forward speed in m/s (negative = reverse)."},
        "angular": {"type": "number", "description": "Turn rate in rad/s (positive = counter-clockwise / left)."},
        "duration_s": {"type": "number", "description": "How long to move. Defaults to a short safe value if omitted."},
        "topic": {"type": "string", "description": "Velocity topic; defaults to the configured cmd_vel."},
    }},
}

ROS_PUBLISH = {
    "name": "ros_publish",
    "description": "Publish an arbitrary ROS message to an allowlisted topic, optionally repeated at `rate_hz` for `duration_s`. "
                   "For robot motion use ros_drive instead. " + _SAFETY,
    "parameters": {"type": "object", "properties": {
        "topic": {"type": "string"},
        "msg_type": {"type": "string", "description": "e.g. geometry_msgs/Twist, std_msgs/String"},
        "message": {"type": "object", "description": "The message body as JSON."},
        "duration_s": {"type": "number", "description": "Repeat the message for this long (Twist messages are followed by a zero-velocity stop)."},
        "rate_hz": {"type": "number", "default": 10},
    }, "required": ["topic", "msg_type", "message"]},
}

ROS_CALL_SERVICE = {
    "name": "ros_call_service",
    "description": "Call a ROS service on the allowlist (e.g. reset, teleport). " + _SAFETY,
    "parameters": {"type": "object", "properties": {
        "service": {"type": "string"},
        "srv_type": {"type": "string", "description": "Optional service type; auto-detected when omitted."},
        "request": {"type": "object", "description": "Request fields as JSON.", "default": {}},
        "timeout_s": {"type": "number", "default": 8},
    }, "required": ["service"]},
}

ROS_GET_POSE = {
    "name": "ros_get_pose",
    "description": "Get the robot's current pose from odometry: x, y (m), yaw (rad/deg), velocities, frame, data age and collision flag. Read-only.",
    "parameters": {"type": "object", "properties": {
        "topic": {"type": "string", "description": "Odometry topic; defaults to the configured odom."},
        "timeout_s": {"type": "number", "default": 5},
    }},
}

ROS_CAMERA_SNAPSHOT = {
    "name": "ros_camera_snapshot",
    "description": "Grab ONE frame from the robot camera (sensor_msgs/Image or CompressedImage), save it to a PNG/JPEG file and "
                   "return the path. Then call vision_analyze on that path (image_url=<path>) to see/describe it, and put the "
                   "returned MEDIA: line in your reply to show the picture to the user. Read-only.",
    "parameters": {"type": "object", "properties": {
        "topic": {"type": "string", "description": "Camera topic; defaults to the configured camera."},
        "timeout_s": {"type": "number", "default": 6},
    }},
}

ROS_STOP = {
    "name": "ros_stop",
    "description": "STOP the robot immediately (burst of zero-velocity commands). With estop=true also LATCH the e-stop: all "
                   "motion and service calls are then blocked until the human clears it with /ros clear. Never blocked by the "
                   "interlock. Use estop=true whenever something looks unsafe or the user says stop/halt/emergency.",
    "parameters": {"type": "object", "properties": {
        "estop": {"type": "boolean", "default": False, "description": "Latch the e-stop (only a human can clear it)."},
        "reason": {"type": "string"},
    }},
}

ROS_STATUS = {
    "name": "ros_status",
    "description": "Show the robot link and safety state: backend, connection, active limits, geofence, allowlists, e-stop latch "
                   "and the most recent blocked calls. Read-only. Call before the first move of a session.",
    "parameters": {"type": "object", "properties": {}},
}

ALL = [ROS_LIST, ROS_ECHO, ROS_DRIVE, ROS_PUBLISH, ROS_CALL_SERVICE, ROS_GET_POSE, ROS_CAMERA_SNAPSHOT, ROS_STOP, ROS_STATUS]
