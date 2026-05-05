# Pose Library — How It Works

A minimal "programming by demonstration" feature added to `index.html`.
The user tele-ops the arm into a target configuration, clicks **Save Pose**,
and a button is added to the playback row that replays that exact configuration.

This doc focuses on the ROS interactions. The teleop buttons and pose-button
rendering are simple DOM glue and not covered.

## Scope: joint values, not Cartesian poses

Lab 13 talks about saving end-effector poses (`link_grasp_center` in some
reference frame) and using `tf_buffer.lookup_transform` / `tf_buffer.transform`
to relocate them at replay time. We deliberately skip that:

- We save **arm, wrist, and gripper joints** (`joint_lift`, `wrist_extension`,
  `joint_gripper_finger_left`, `joint_wrist_yaw`, `joint_wrist_pitch`,
  `joint_wrist_roll`) — six values total.
- These are 1-DOF revolute/prismatic joints in joint space — there is no
  "frame" to choose. Replaying the same joint values reproduces the same
  end-effector pose *relative to `base_link`* by construction.
- Therefore: no TF, no ArUco, no `tf2_web_republisher`. Just `/stretch/joint_states`
  in and `FollowJointTrajectory` out.

If we ever needed to relocate poses w.r.t. the world or an external marker,
TF would have to come back in. For purely arm/gripper repeatability it does
not.

## Transport: raw rosbridge over WebSocket

The page opens a single `WebSocket("wss://localhost:9090")` connection to
`rosbridge_server`. Every ROS interaction is a JSON envelope sent over that
socket. No `roslibjs` dependency — we hand-write the four ops we need:
`subscribe`, `publish`, `call_service`, `send_goal`.

> Note: `send_goal` is the action op exposed by Hello Robot's
> `roslibjs`-compatible fork of rosbridge. Upstream rosbridge uses
> `send_action_goal`; if you swap server, this op name has to change.

## Save path

### 1. Subscribe to joint states

On socket open:

```json
{ "op": "subscribe",
  "topic": "/stretch/joint_states",
  "type": "sensor_msgs/msg/JointState" }
```

`stretch_driver` publishes this at ~30 Hz. Each `publish` callback we extract
six values by name:

| Joint name (in `name[]`)        | Stored as       | Units |
|---------------------------------|-----------------|-------|
| `joint_lift`                    | `liftCurrent`   | m     |
| `wrist_extension`               | `extCurrent`    | m     |
| `joint_gripper_finger_left`     | `gripCurrent`   | rad   |
| `joint_wrist_yaw`               | `yawCurrent`    | rad   |
| `joint_wrist_pitch`             | `pitchCurrent`  | rad   |
| `joint_wrist_roll`              | `rollCurrent`   | rad   |

Two non-obvious choices:

- **`wrist_extension`** is published by the driver as a single virtual joint
  (sum of the four telescoping segments `joint_arm_l0..l3`). We use it
  directly because the trajectory action also accepts it as a joint name —
  the driver expands and contracts symmetrically.
- **`joint_gripper_finger_left`** is what the driver actually publishes;
  there is *no* `gripper_aperture` in `/stretch/joint_states`. The trajectory
  action accepts the finger joint name on write, so save and replay
  round-trip cleanly without a units conversion. (The right finger mirrors
  the left, so capturing one is sufficient.)

### 2. Snapshot on click

`Save Pose` reads the latest cached values and pushes
`{ name, lift, ext, grip, yaw, pitch, roll }` into `localStorage` under
`stretch_simple_poses_v1`. No ROS traffic at all on save — it's a pure
read of the in-memory cache populated by the subscription. (If `joint_states`
hasn't arrived yet, any of the six values are `null` and save aborts with a
warning.)

`localStorage` keeps the library across reloads. Clearing site data wipes
it.

## Replay path

### 1. Switch to navigation mode

Stretch's driver has multiple modes; trajectory commands need
`navigation_mode`. We call:

```json
{ "op": "call_service",
  "service": "/switch_to_navigation_mode",
  "type": "std_srvs/srv/Trigger" }
```

This is idempotent — calling it when already in nav mode is a no-op. We do
it on every replay so the user can recover if a gamepad/external controller
flipped the mode in the meantime.

### 2. Send a single trajectory point

```json
{ "op": "send_goal",
  "action_name": "/stretch_controller/follow_joint_trajectory",
  "action_type": "control_msgs/action/FollowJointTrajectory",
  "goal_msg": {
    "trajectory": {
      "joint_names": [
        "joint_lift",
        "wrist_extension",
        "joint_gripper_finger_left",
        "joint_wrist_yaw",
        "joint_wrist_pitch",
        "joint_wrist_roll"
      ],
      "points": [{
        "positions": [<lift>, <ext>, <grip>, <yaw>, <pitch>, <roll>],
        "time_from_start": { "secs": 3, "nsecs": 0 }
      }]
    }
  } }
```

`stretch_driver` (via `stretch_controller`) interpolates from the current
joint state to the requested positions over `time_from_start`. Three seconds
is a comfortable default — long enough that motion is smooth on a heavy
payload, short enough not to feel sluggish. All six joints move
concurrently and arrive at the same time, so the wrist orientation reaches
its target in lockstep with the lift/extension/gripper.

**Schema-tolerant replay.** Older saves (before wrist fields were added) are
still in `localStorage` and don't have `yaw / pitch / roll`. `playPose`
filters out any joint whose stored value is not a number, so legacy poses
replay using whatever subset they captured instead of crashing the goal
construction. The same defensiveness applies to the playback button's
hover-tooltip rendering.

### 3. Why a single point and not a trajectory

For one saved pose, one point is enough. If we ever extend this to play a
*sequence* of saved poses, the cleanest implementation is a single goal
with multiple points and increasing `time_from_start` values — the
controller will interpolate continuously through them without us having to
wait on action results between points.

## Runstop and safety

Motion commands are silently dropped while the runstop is engaged
(`/is_runstopped == true`, white button flashing). Joint-state subscription
keeps working because reading sensors does not require motors. This is why
the page can show live readouts but not move the arm. To check runstop
state from a terminal:

```bash
ros2 topic echo /is_runstopped --once
```

## Coexistence with the Xbox gamepad

The Hello-Robot gamepad teleop runs as its own ROS node and talks to the
*same* `FollowJointTrajectory` action. ROS allows multiple clients; the
last goal wins. Practically:

- **Save** is read-only on ROS — gamepad and web app never conflict here.
- **Replay** sends a goal. If the user touches the gamepad during replay,
  the gamepad's goal preempts and motion stops. That's actually the
  desired safety behavior.

The web app does not publish `/stretch/cmd_vel` and does not call mode
switches except on its own button presses, so it does not interfere with
gamepad arm or base control while idle.

## Storage format

`localStorage["stretch_simple_poses_v1"]` is a JSON array:

```json
[
  { "name": "shelf",
    "lift": 0.92, "ext": 0.18, "grip": 0.00,
    "yaw":  0.00, "pitch": -0.20, "roll": 0.00 },
  { "name": "handoff",
    "lift": 0.78, "ext": 0.05, "grip": 0.06,
    "yaw":  1.57, "pitch":  0.00, "roll": 0.00 }
]
```

Entries saved before the wrist fields existed will be missing
`yaw / pitch / roll`. They still load and replay (with whatever joints
they did capture) — no migration needed. To wipe the library entirely:

```js
localStorage.removeItem("stretch_simple_poses_v1")
```

The schema is versioned in the key (`_v1`) so a future change to the joint
set or units can bump the key without colliding with existing saves.

## ROS surface in summary

The base-relative library uses three endpoints:

| Direction | Name                                            | Type                                      |
|-----------|-------------------------------------------------|-------------------------------------------|
| Subscribe | `/stretch/joint_states`                         | `sensor_msgs/msg/JointState`              |
| Service   | `/switch_to_navigation_mode`                    | `std_srvs/srv/Trigger`                    |
| Action    | `/stretch_controller/follow_joint_trajectory`   | `control_msgs/action/FollowJointTrajectory` |

The world library (next section) adds four `std_msgs/String` topics that
the browser uses to talk to a Python helper node.

# World- and Marker-Frame Pose Library

A second library that records gripper pose in **any TF frame** — most
usefully `odom` (the room) or an ArUco marker frame like `target_frame`.
After moving either the robot or the marker, replaying a saved pose
commands a *different* set of joint values calculated to put the gripper
back at the same physical spot.

The frame is chosen at save time:

- `odom` → "fixed in the room." Robot can move; replay still goes to the
  same physical room location (within reach).
- `target_frame` (or any other ArUco marker frame) → "fixed relative to
  the marker." Marker can move; replay tracks the marker.
- `map` → same as `odom` but uses SLAM-corrected localization. Only
  available if you're running Nav2 / AMCL.

Critically, the code path is **identical** for all three. tf2 walks
whatever chain exists between the chosen frame and `link_grasp_center`;
the rest of the math doesn't care.

The browser cannot call ROS's TF Python API directly, so the heavy lifting
runs in `pose_library_server.py` — a small ROS2 node that uses the two
functions Lab 13 explicitly recommends:

- `tf_buffer.lookup_transform(target, source, time)` — capture EE pose at
  save time.
- `tf_buffer.transform(pose_stamped, target_frame)` — project a saved pose
  into the current `base_link` at replay time.

The browser is a thin client: it sends a name string to save / play /
delete and renders a list of names the server publishes back. No TF math
in JS, no joint math in JS, no FK assumptions.

## Architecture

```
                          rosbridge
  Browser ────────────────────────────────────────► pose_library_server.py
   │  pub  /pose_library/save  (std_msgs/String)         │
   │  pub  /pose_library/play  (std_msgs/String)         │
   │  pub  /pose_library/delete (std_msgs/String)        │
   │  sub  /pose_library/names (std_msgs/String, latched)│
                                                         │
                                  tf2_ros.Buffer ────────┤
                                  tf2_geometry_msgs.transform
                                  ActionClient ──────────┤
                                                         ▼
                                   /stretch_controller/follow_joint_trajectory
```

Plain `std_msgs/String` topics intentionally — no custom `.srv` or `.msg`
types means no ament package and no build step. Names are JSON-encoded
where the payload is a list (`/pose_library/names`).

## Save path

1. Browser publishes a payload to `/pose_library/save`. The format is
   `frame|name` (e.g. `target_frame|grasp_mug`); a bare `name` defaults
   to `odom`. The frame dropdown in the UI controls which frame goes in
   front of the pipe.
2. Server callback runs **two** `lookup_transform` calls:
   - `<frame> → link_grasp_center` — the EE pose to remember, in the
     frame the user picked.
   - `base_link → link_grasp_center` — the EE pose *as seen from the
     base at save time*. We need this so replay can compute deltas
     without reimplementing Stretch's forward kinematics.
3. Server snapshots the same six joint values from `/stretch/joint_states`
   that the browser-side library uses.
4. Server appends `{name, frame, world_pose, base_pose_at_save,
   joints_at_save}` to `~/.stretch_pose_library.json` and republishes the
   names list.

The two `lookup_transform` calls are why we don't need any FK code — at
save time we already know exactly what `base_link → link_grasp_center`
was, so replay only needs the *change* relative to that.

The first lookup is also where the frame choice matters. tf2 walks
whatever chain exists in the TF tree:

- `odom → base_link → link_lift → … → link_grasp_center`
- `target_frame → camera_color_optical_frame → … → link_grasp_center`
  (when the marker is visible to the head camera)

Either chain composes the same way and gives back a `TransformStamped`.
The server stores the result without caring how it was assembled.

## Replay path

1. Browser publishes the pose name to `/pose_library/play`.
2. Server constructs a `PoseStamped` whose `header.frame_id` is the saved
   `entry["frame"]` (e.g. `odom` or `target_frame`) and whose body is the
   stored `world_pose`.
3. Server calls **`tf_buffer.transform(pose_stamped, "base_link")`** to
   reproject the saved EE position into the *current* base frame. This
   single call accounts for *whichever* of these moved between save and
   replay:
   - the **robot base** (`odom → base_link` changes via wheel odometry)
   - the **marker** (`target_frame` itself moves in `odom` when it gets
     picked up and put down somewhere new — the detector publishes the
     new pose continuously)
   - **both at once**

   tf2 doesn't care which frames moved; it just composes the live chain.
4. Server takes the translation delta `Δ = ee_in_base_now -
   base_pose_at_save.translation` and applies the lab's joint heuristic
   directly on top of the saved joint values:

   | Delta component       | Joint adjustment                                |
   |-----------------------|-------------------------------------------------|
   | `Δz`  (height)        | `joint_lift += Δz`                              |
   | `Δy`  (sideways)      | `wrist_extension -= Δy` (arm extends in `-y` of `base_link`, so an EE point that's now less-negative-y means retract) |
   | `Δx`  (forward)       | *not corrected* — would require driving the base |
   | `Δyaw` (base rotation)| `joint_wrist_yaw += Δyaw` (keep gripper pointing the same direction in the world) |

   Pitch and roll of the wrist are kept at saved values.
5. Lift and extension are clamped to joint limits. If `|Δx| > 5 cm` the
   server logs a warning — Stretch's arm extends sideways only, so the
   forward component is unreachable without driving the base. The arm
   still moves to the best in-reach approximation.
6. Server sends a `FollowJointTrajectory` goal to the same action server
   the browser teleop uses.

Server logs print both the saved joint values and the commanded joint
values for every replay, which is exactly the comparison the demo wants
to show.

## What the demo shows

There are two demos for the world library and one for marker-relative.

### Demo A — world frame, robot moves

1. Bring up the stack:
   ```bash
   python3 pose_library_server.py
   ```
2. Tele-op the arm to a target configuration. In the browser pick
   `odom (world)` from the dropdown, type a name (e.g. `shelf`), click
   **Save World Pose**.
3. Drive the base around — turns and forwards — using the gamepad or the
   web base buttons.
4. Click the `shelf` button.
5. Joint readouts land on **different** values than were saved (the
   server log shows the deltas). Gripper ends at the same room location
   (within reach).

For comparison, save the same configuration as a base-relative pose
using the original library. Replaying *that* after driving recovers the
saved joint values, putting the gripper somewhere new in the room. Side
by side, this is the whole point of saving in `odom`.

### Demo B — marker frame, marker moves

1. In addition to the teleop stack and `pose_library_server.py`, start
   the ArUco detector:
   ```bash
   ros2 launch stretch_core stretch_aruco.launch.py
   ```
2. Place the printed marker (id=1, 86 mm) somewhere visible to the head
   camera. Confirm with `ros2 run tf2_ros tf2_echo base_link target_frame`
   that the frame is live.
3. Tele-op the arm to point the gripper at a spot near the marker. Pick
   `target_frame (ArUco id 1)` from the dropdown, save with a name like
   `pickup`.
4. **Move the marker** to a new location in the room (don't move the
   robot).
5. Click the `pickup` button. The arm goes to the same offset relative
   to the marker — i.e., it follows the marker.

Combined: save in `target_frame`, then move both the robot *and* the
marker. Replay recovers the same gripper-to-marker offset regardless of
where either has gone, as long as the marker is still visible at replay
time.

## Storage format (server-side library)

`~/.stretch_pose_library.json` is a `{name → entry}` dict:

```json
{
  "shelf": {
    "name": "shelf",
    "frame": "odom",
    "world_pose": {
      "translation": { "x": 1.42, "y": 0.31, "z": 0.95 },
      "rotation":    { "x": 0.0,  "y": 0.0,  "z": 0.71, "w": 0.71 }
    },
    "base_pose_at_save": {
      "translation": { "x": 0.18, "y": -0.16, "z": 0.95 },
      "rotation":    { "x": 0.0,  "y": 0.0,   "z": 0.0,  "w": 1.0 }
    },
    "joints_at_save": {
      "joint_lift": 0.92, "wrist_extension": 0.18,
      "joint_gripper_finger_left": 0.0,
      "joint_wrist_yaw": 0.0, "joint_wrist_pitch": -0.20, "joint_wrist_roll": 0.0
    }
  },
  "pickup_from_marker": {
    "name": "pickup_from_marker",
    "frame": "target_frame",
    "world_pose": {
      "translation": { "x": 0.05, "y": 0.0, "z": 0.10 },
      "rotation":    { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    },
    "base_pose_at_save": {
      "translation": { "x": 0.06, "y": -0.55, "z": 0.74 },
      "rotation":    { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
    },
    "joints_at_save": { "joint_lift": 0.50, "wrist_extension": 0.20, "joint_gripper_finger_left": 0.0,
                        "joint_wrist_yaw": 0.0, "joint_wrist_pitch": 0.0, "joint_wrist_roll": 0.0 }
  }
}
```

The same schema works for both kinds of save — `frame` is the only field
that differs in meaning. `world_pose` is always "EE in `frame`," whatever
`frame` is.

Persistence is on the robot, not the browser — so any client connecting
to the same robot sees the same library.

## ROS surface (world library)

| Direction | Name                          | Type                | Notes |
|-----------|-------------------------------|---------------------|-------|
| Subscribe | `/pose_library/save`          | `std_msgs/String`   | `frame\|name` (frame defaults to `odom`) |
| Subscribe | `/pose_library/play`          | `std_msgs/String`   | name to replay |
| Subscribe | `/pose_library/delete`        | `std_msgs/String`   | name to remove |
| Publish   | `/pose_library/names`         | `std_msgs/String`   | JSON array, latched (TRANSIENT_LOCAL) |
| Subscribe | `/stretch/joint_states`       | `sensor_msgs/JointState` | for joint snapshots |
| Action    | `/stretch_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | replay goals |
| TF        | `tf2_ros.Buffer` + `TransformListener` | (`/tf`, `/tf_static`) | save + replay |

## Frame choice and limits

The save message accepts any TF frame name as the `frame` part of
`frame|name`. Practical options:

- `odom` — default, room-relative.
- `map` — better than `odom` if you're running Nav2 / AMCL with a map,
  because it's drift-corrected. No code change needed; just pick it
  from the dropdown.
- `target_frame` (and other ArUco frames) — requires the detector to be
  running and the marker to be visible at *both* save and replay time.
  Add more entries to `stretch_marker_dict.yaml` (each one becomes a TF
  frame named after its `name` field) to use multiple landmarks.

The forward component of replay is intentionally not corrected. Driving
the base during replay is genuinely hazardous (open-loop on `cmd_vel` is
brittle, Nav2 needs a map and goals) and outside the scope of the demo.

# How to Run the Whole Stack

Everything below runs on slinky (the Stretch). The pose library code
lives at `~/beaf_ws/pose_library_server.py`.

## Required processes

Five terminals, in order. Each is a foreground process — leave it
running and open a new tab/SSH session for the next step.

### 1. Teleop interface (driver + rosbridge + cameras)

```bash
cd ~/ament_ws/src/stretch_web_teleop
./launch_interface.sh
```

This brings up `stretch_driver`, `rosbridge_server` on
`wss://localhost:9090`, the RealSense cameras, and the WebRTC video
republishers. After it's running, press the white button on top of the
robot to release the runstop (light goes from flashing to solid).

Sanity check from another terminal:
```bash
ros2 topic hz /stretch/joint_states     # ~30 Hz
ros2 topic echo /is_runstopped --once   # data: false
```

### 2. Pose library server

```bash
cd ~/beaf_ws
python3 pose_library_server.py
```

Logs:
```
[INFO] [pose_library_server]: pose library ready (N saved); storage=/home/hello-robot/.stretch_pose_library.json
```

This handles the world / marker save+replay topics. Without it, the
"Pose Library (world)" section of the browser does nothing — but the
base-relative `localStorage` library still works, since it's
browser-only.

### 3. (Optional) ArUco detector

Only needed if you want to save poses relative to a marker like
`target_frame`.

```bash
ros2 launch stretch_core stretch_aruco.launch.py
```

The detector reads `stretch_marker_dict.yaml` from
`$(ros2 pkg prefix stretch_core)/share/stretch_core/config/`. Edits to
that file require restarting the detector. Edits to the source-space
copy in `~/ament_ws/src/...` require a `colcon build --packages-select
stretch_core` first.

Verify:
```bash
ros2 topic echo /aruco/marker_array --once          # marker detected
ros2 run tf2_ros tf2_echo base_link target_frame    # frame is on TF
```

The marker frame only appears in `/tf` while the marker is in view of
the head camera. Moving the marker out of view makes the frame
disappear within ~200 ms.

### 4. Browser

On your laptop (with an SSH tunnel forwarding port 9090 to slinky), or
on slinky itself with a browser, open `index.html`. First time only:
visit `https://localhost:9090` and click through the self-signed cert
warning so the WebSocket can connect.

The page should show:
- `connected` status
- live joint readouts under each section heading
- two pose libraries: base-relative and world

### 5. (Optional) Gamepad teleop

If you want to drive with the Xbox controller in parallel with the web
buttons, just plug it in — the gamepad node starts automatically with
`launch_interface.sh`. The web app's base buttons and the gamepad
share `/stretch/cmd_vel`; whichever moves last wins.

## Day-to-day usage

### Saving a base-relative pose (localStorage)

1. Tele-op the arm into the configuration you want.
2. Type a name in the **Pose Library (base-relative)** input.
3. Click **Save Pose**.
4. A button appears in the playback row. Click it any time to replay.
   Replay works regardless of where the base has driven, but reproduces
   the same *joint* values, not the same world location.

### Saving a world (`odom`) pose

1. Tele-op into position.
2. In the **Pose Library (world)** section: pick `odom (world)` from
   the dropdown, type a name, click **Save World Pose**.
3. The button appears below. Drive the base around. Click the button —
   joints adjust to track the same room location.

### Saving a marker-relative pose

1. Make sure the ArUco detector (terminal 3) is running.
2. Place the marker so the head camera can see it. Verify with
   `tf2_echo` that `target_frame` is broadcasting.
3. Tele-op the gripper to a useful offset relative to the marker.
4. In the **Pose Library (world)** section: pick `target_frame (ArUco
   id 1)`, type a name, click **Save World Pose**.
5. Move the marker (and/or the robot). Make sure the marker is still in
   view of the head camera. Click the button. The arm tracks the
   marker.

### Deleting a pose

Click the `✕` next to its button. Both libraries support delete; both
update immediately. The world library deletes from
`~/.stretch_pose_library.json` on slinky.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Status stays `connecting…` | rosbridge not up, or cert not accepted | Visit `https://localhost:9090` and accept cert; check terminal 1 |
| Joint readouts show `—` | wrong joint_states topic | Confirm `ros2 topic hz /stretch/joint_states` shows ~30 Hz |
| Arm buttons do nothing, but readouts update | runstop engaged | Press white button on robot until solid |
| `waiting for joint_states` on save | subscription not delivered yet | Reload the page after the driver is up |
| World save errors `frame 'target_frame' does not exist` | marker not visible / detector not running | Start terminal 3, point head camera at marker |
| World replay errors `Lookup would require extrapolation into the future` | `header.stamp = now()` raced TF | Already fixed (uses `Time()` = latest); restart server if you see this |
| World replay does the wrong thing in `wrist_extension` direction | sign of `Δy` correction | Already fixed; pull latest `pose_library_server.py` |
