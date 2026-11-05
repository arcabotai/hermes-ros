# hermes-ros — under-90-second demo storyboard

Two recordings. Both must say, out loud, that the robot is **simulated** and that the interlock covers **plugin tools only**.

Target length: **under 90 seconds** of picture. No music required. Do not claim this is a certified safety system, a sandbox, or a physical robot.

## A. Hermes chat (needs Hermes installed — coordinator records this)

Pre-roll, off camera:

```bash
hermes plugins install arcabotai/hermes-ros
hermes plugins enable hermes-ros
```

Leave the default `backend: mock`. Optional: `plugins.entries.hermes-ros.settings.safety.geofence.enabled: true`.
Open one terminal on `hermes chat -t ros,vision` and, beside it, the Robot tab in `hermes dashboard`.
The dashboard is read-only. Clear a latched e-stop with `/ros clear`, not from the page.

| t (s) | On screen | Say / do |
|---|---|---|
| 0–8 | Title: **hermes-ros — simulated robot, plugin-tool interlock.** Then the terminal prompt. | "This is a simulated robot inside the hermes-ros plugin. No hardware. The speed limit applies to the plugin's ros tools, not to a terminal and not to the robot's own controller." |
| 8–18 | Type: *What can you see?* The agent calls `ros_camera_snapshot`. The PNG shows in the Robot tab: open floor, a red cone. | "It grabs one camera frame from the mock. The cone is in front of it." |
| 18–42 | Type: *Drive to the red cone.* Snapshot, small `ros_drive`, pose, snapshot. The cone gets larger. Stop in front of it. | "Look, one small move, check the pose. Each drive is speed-limited and ends with a zero command." |
| 42–62 | Type: *Floor it, 3 meters per second, straight ahead.* Result is red: linear speed 3.00 m/s exceeds 0.50 m/s. The blocked-call row appears. The pose does not jump. | "The model asked for 3 m/s. The pre_tool_call hook refused that tool call and logged it. The robot did not move." |
| 62–78 | Type `/ros estop`. Banner shows latched. Ask the agent to drive slowly. Blocked: e-stop latched. | "A human latches the e-stop. The agent cannot clear it. Only `/ros clear` does, and I am not doing that on camera." |
| 78–88 | End card: `github.com/arcabotai/hermes-ros`. Three lines: simulated mock; interlock is plugin tools only; default rosbridge is unencrypted and local. | "Not a sandbox. Not a hardware safety system. If you point it at a real robot, keep an independent e-stop, and do not put plaintext rosbridge on the public internet." |

If the cone approach runs long, cut middle tool calls in the edit. Keep the first frame, the last frame, the 3 m/s refusal, and the e-stop. Those four have to stay.

## B. Offline recording (no Hermes — this repo can run it)

This is the same checks through the real tool handlers, not a Hermes chat session. Use it when Hermes is not installed. It is not a substitute shot for the dashboard tab.

```bash
python3 demo/mock_demo.py --present --out /tmp/hermes-ros-demo
```

`--present` pauses between beats so the terminal stays readable and the whole run stays under 90 seconds. Frames land in `/tmp/hermes-ros-demo/` as `camera-start.png`, `camera-at-cone.png`, and `topdown.png`. Open those files in an image viewer when the script prints them. Do not draw a fake dashboard over the terminal.

What the run shows, in order: simulated-robot banner, first camera frame with the cone, drive steps and pose, second frame, rejected 3 m/s and 30 s command, e-stop latch, a further drive refused, then the line that the interlock is plugin tools only.

## What not to say

* Do not say the model cannot get past the limit by any means. It cannot get past it through `ros_drive`. A terminal can still open rosbridge.
* Do not call the mock, or turtlesim, a physical robot.
* Do not show a public or non-local `ws://` rosbridge.
