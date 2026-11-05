---
name: ros-safe-operation
description: How to operate a physical or simulated ROS robot safely with the hermes-ros tools - pre-flight, small verified moves, reading refusals, e-stop discipline.
---

# Operating a robot safely with hermes-ros

You are controlling something that can hit a wall, a person or itself. The `hermes-ros` plugin enforces hard limits in a
`pre_tool_call` interlock, but the limits are the *last* line of defence. You are the first. Work like a careful pilot.

## Pre-flight (once per session)
1. `ros_status` - confirm the backend (`mock` vs `rosbridge`), the active limits, the geofence and whether the e-stop is latched.
2. `ros_list kind=topics filter=cmd_vel` / `odom` / `camera` - confirm the topics actually exist before you command anything.
3. `ros_get_pose` - know where the robot is and that odometry is fresh (`age_s` small).
4. `ros_camera_snapshot`, then `vision_analyze` on the returned path - look before you move. Say what you see.

## Moving: the loop
Never plan a long blind route. Repeat: **look → decide one small move → `ros_drive` → verify → look again**.
- Keep each move short (1-3 s) and slow (well under the speed limits). Turn first, then drive straight.
- After every move read `pose_after`, `displacement_m` and `collided`. If displacement is not what you expected, stop and re-assess.
- Approach targets (a cone, a wall, a person) in small steps; stop **at least ~0.5 m before** an obstacle, never "until you touch it".
- To get to something you can see: estimate its bearing from the picture, rotate to face it (`angular`), re-snapshot to confirm, then advance.
- Positive `angular` turns left (counter-clockwise). Negative `linear` reverses; you cannot see behind the robot, so avoid it.

## When the interlock says no
A refusal looks like `hermes-ros interlock BLOCKED ... [rule]: reason`. It is information, not an obstacle to route around.
- **Do not** retry the identical call, split a forbidden move into many "legal" pieces, add topics to the allowlist, publish to a
  different topic to dodge a rule, or use another tool (terminal, python) to talk to the robot. That defeats the safety model.
- `over_speed_*` / `over_duration`: re-issue slower / shorter, or explain to the user why the request is too aggressive.
- `geofence` / `geofence_no_odom` / `geofence_odom_stale`: the move would leave the allowed area or the robot's position is unknown.
  Choose a different direction/length, or tell the user.
- `*_not_allowlisted`: you may not write there. Tell the user what you wanted to do and why; only a human changes the allowlist.
- `estop_latched`: stop trying to move. Tell the user the e-stop is engaged and that only they can clear it with `/ros clear`.

## E-stop discipline
- If the user says stop / halt / wait / no, or anything looks unsafe (collision, odd odometry, lost camera, an obstacle you did not expect):
  call `ros_stop` with `estop=true` **first**, explain afterwards.
- `ros_stop` with `estop=false` only zeroes velocity (a brake); use it between steps if unsure.
- Never try to clear the latch; you cannot, and you must not look for a way.

## Honesty
- Report what the robot actually did (use the returned poses and `collided`), not what you intended.
- If the camera or odometry is unavailable, say so and do not guess positions. Without odometry do not drive.
- Share pictures with the user by including the `MEDIA:<path>` line from `ros_camera_snapshot` in your reply.

## Reference: tools
`ros_status` · `ros_list` · `ros_echo` · `ros_get_pose` · `ros_camera_snapshot` (read-only) ·
`ros_drive` · `ros_publish` · `ros_call_service` (interlocked writes) · `ros_stop` (always allowed).
