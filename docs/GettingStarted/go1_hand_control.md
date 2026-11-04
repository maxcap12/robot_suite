# 🐕 Go1 Hand Control (Docker)

This page explains everything needed to control a **Unitree Go1** with hand gestures using the `robot_suite` in Docker, and how to reproduce the setup on a new machine.

## Overview


Everything runs in a single `go1_suite` container (ROS 2 Jazzy):

| Component | What |
| --------- | ---- |
| **Go1 driver** | [unitree_ros](https://github.com/snt-arg/unitree_ros), cloned into `drivers/unitree_ros` and built by `bootstrap.sh go1`. Connects to the robot and listens to `/cmd_vel` (Twist), `/stand_up` and `/stand_down` (Empty). |
| **RealSense driver** | `realsense2_camera`, publishes `/camera/camera/color/image_raw`. |
| **Robot suite** | Hand gestures plugin, behaviour tree (`go1_bt`), control station UI. |

Files involved in this repo:

| File | Purpose |
| ---- | ------- |
| `docker/go1/Dockerfile` | Image for the suite (`robot_suite:go1`). |
| `docker-compose.yml` → `go1_suite` | Container config (display, USB for the RealSense, host network). |
| `bootstrap.sh go1` | Installs dependencies, clones the Go1 driver (pinned version) and builds the workspace (called by the Dockerfile). |
| `robot_bringup/launch/go1_launch.py` | Starts the Go1 driver, RealSense, BT, hand gestures plugin and control station. |
| `robot_bringup/config/go1_params.yaml` | Topics and parameters (camera topic, BT name, control station keys...). |
| `robot_bt/robot_bt/bt/go1_bt.py` | Behaviour tree: mode selection + hand gestures control. |
| `robot_bt/robot_bt/behaviours/go1/actions/go1_gestures_interpreter_action.py` | Gestures → `/cmd_vel`, `/stand_up`, `/stand_down`. Speeds are defined at the top of the class. |

## 1. Prerequisites (host machine)

- Linux with **Docker** and the **Docker Compose plugin** (`docker compose version`).
- Your user in the `docker` group (`groups` should list `docker`).
- An X display for the control station window (works on X11 and on Wayland through XWayland).
- An **Intel RealSense** camera plugged into the host (USB 3).
- Network access to the Go1:
    - Ethernet: the host needs an address in `192.168.123.0/24` (robot at `192.168.123.161`),
    - Wi‑Fi: connect to the Go1 hotspot (robot at `192.168.12.1`) and launch with `wifi:=true`.

## 2. Go1 driver

The driver ([unitree_ros](https://github.com/snt-arg/unitree_ros)) is installed by `bootstrap.sh go1`:

- it is cloned with its `unitree_legged_sdk` submodule into `drivers/unitree_ros` (`drivers/` is git-ignored),
- it is pinned to commit `2e8c3e0`. To use another version, set `UNITREE_ROS_VERSION` when running the bootstrap, or delete `drivers/unitree_ros` and edit the default in `bootstrap.sh`,
- if `drivers/unitree_ros` already exists (e.g. a local checkout with your own changes), it is reused as-is and copied into the image.

It is started by `go1_launch.py` with its default parameters (`drivers/unitree_ros/config/params.yaml`).

!!! note "The driver exits if the robot is not reachable"

    At startup the driver pings the robot and stops with `CONNECTION IS NOT AVAILABLE` if there is no answer.
    The rest of the suite keeps running. Connect to the robot first, then (re)start the container.

!!! note "Running the driver somewhere else"

    If the driver already runs elsewhere (on the robot, or in another container), launch with `use_driver:=false`.
    It must then use host networking and the same `ROS_DOMAIN_ID`. The `go1_suite` container sets
    `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`: without it, two host-network containers with separate IPC namespaces
    discover each other but silently drop messages (Fast DDS tries to use shared memory).

## 3. Build the suite image

From the root of the repository:

```bash
docker compose build go1_suite
```

The first build takes a while (PyTorch, MediaPipe, ...). Later builds reuse the Python dependency layer as long as `requirements.txt` doesn't change.

## 4. Run

1. Display access: `docker compose` mounts the X11 cookie of your session (`$XAUTHORITY`) in the container,
   so the control station window opens without extra steps. If `XAUTHORITY` is not set in your shell
   (`echo $XAUTHORITY`), allow containers to open windows instead (once per login session):

    ```bash
    xhost +local:docker
    ```

1. Turn on the Go1 and connect this machine to it (Ethernet or Wi‑Fi, see [Prerequisites](#1-prerequisites-host-machine)). Check with `ping 192.168.123.161` (or `192.168.12.1`).

1. Start the suite:

    ```bash
    docker compose up go1_suite
    ```

    This runs `ros2 launch robot_bringup go1_launch.py`, which starts the driver and the camera and opens the control station window.

    Launch arguments (use `docker compose run --rm go1_suite ros2 launch robot_bringup go1_launch.py <arg>:=<value>`):

    | Argument | Default | Description |
    | -------- | ------- | ----------- |
    | `use_driver` | `true` | Start the Go1 driver. Set to `false` if it runs elsewhere. |
    | `wifi` | `false` | Connect to the robot over Wi‑Fi (`192.168.12.1`) instead of Ethernet (`192.168.123.161`). |
    | `use_realsense` | `true` | Start the RealSense driver. Set to `false` if another node publishes the images. |
    | `use_control_station` | `true` | Start the pygame UI (needs a display). |

1. Click on the control station window and press:

    | Key | Action |
    | --- | ------ |
    | `h` | **Hand control mode** (gestures drive the robot) |
    | `m` or `k` | Manual (keyboard) mode: hand control off, drive with `w`/`s` forward/backward, `a`/`d` left/right, `←`/`→` rotate |
    | `t` | Stand up (works in any mode) |
    | `l` | Stand down (works in any mode) |
    | `q` | Quit |

    The suite **starts in manual mode**: the robot doesn't move until you press `h`.

    The window shows the robot battery (`soc` of `/bms_state`, published by the driver; 0 % until the driver is connected).
    The topic, message type and field are the `battery_*` parameters of `remote_controller` in `go1_params.yaml`.

## 5. Gestures

Both hands must be visible to the camera. Any other combination, a single hand, or no hand stops the robot.

| Left hand | Right hand | Action |
| --------- | ---------- | ------ |
| Open_Palm | Pointing_Up | Move forward |
| Open_Palm | Victory | Move backward |
| Closed_Fist | Pointing_Up | Move left |
| Pointing_Up | Closed_Fist | Move right |
| Closed_Fist | Thumb_Up | Rotate left |
| Thumb_Up | Closed_Fist | Rotate right |
| Open_Palm | Thumb_Up | Stand up |
| Open_Palm | Thumb_Down | Stand down |

!!! warning "Safety"

    - Velocity commands are only sent while hand mode is active. When you leave hand mode (or the BT stops ticking the gestures), the robot is stopped.
    - Stand up / stand down: one command per gesture or key press (holding it doesn't repeat it), at most every 2 s.
    - While the robot is lying down, and during 2 s after a stand up, no velocity command is sent (gestures and `w`/`a`/`s`/`d` are ignored). Before a stand up/down the robot is stopped and the command waits ~0.5 s. This is needed because the driver switches the robot to walking mode on every `/cmd_vel` (and on its own 0.4 s after the last one), which would interrupt the posture change: the robot would jump up very fast and look unstable. These times are `posture_transition_time` (control station, `go1_params.yaml`) and the class attributes of `Go1GesturesInterpreterAction`.
    - If no new gesture arrives for 0.5 s (hand out of frame, camera disconnected...), the robot stops.
    - Default speeds are 0.3 m/s forward, 0.2 m/s sideways, 0.6 rad/s rotation. Change them at the top of `Go1GesturesInterpreterAction`.
    - Always keep the Unitree remote at hand to take over.

## 6. Checking that everything works

Open a shell in the running container:

```bash
docker exec -it go1_suite bash
```

- Nodes are up: `ros2 node list` should show `/unitree_ros_node`, `/bt_server`, `/landmark_detector_node`, `/landmark_annotator_node`, `/remote_controller` and the camera node.
- Camera images arrive: `ros2 topic hz /camera/camera/color/image_raw`.
- Gestures are detected (hand mode on): `ros2 topic echo /hand/landmarks --field right_hand.gesture`.
- Commands are sent: `ros2 topic echo /cmd_vel`.
- The driver is connected: `ros2 topic hz /odom` (published by the driver), and `ros2 topic info /cmd_vel` shows 1 subscription.

**Testing without a camera or robot**: switch to hand mode and publish fake gestures:

```bash
ros2 launch robot_bringup go1_launch.py use_driver:=false use_realsense:=false use_control_station:=false &
ros2 topic pub --once /key_pressed std_msgs/msg/String "{data: 'h'}"
ros2 topic pub -r 10 /hand/landmarks hand_gestures_msgs/msg/Landmarks \
  "{left_hand: {handedness: 'Left', gesture: 'Open_Palm'}, right_hand: {handedness: 'Right', gesture: 'Pointing_Up'}}"
# in another shell: ros2 topic echo /cmd_vel   -> linear.x: 0.3
```

## Troubleshooting

| Problem | Fix |
| ------- | --- |
| `Authorization required, but no authorization protocol specified`, `cannot open display` / no control station window | Check that `DISPLAY` and `XAUTHORITY` are set in the shell running `docker compose` (not with `sudo`, which drops them), or run `xhost +local:docker` on the host. Recreate the container after logging in again (`docker compose up --force-recreate go1_suite`): the cookie file changes at each login on Wayland. |
| Logs flooded with the behaviour tree | Set `print_tree: true` only for debugging in `robot_bringup/config/go1_params.yaml` (`bt_server`), it prints the whole tree at every tick. |
| RealSense not found | Plug it in **before** starting the container or restart it (`docker compose restart go1_suite`). Check with `rs-enumerate-devices` or `lsusb` inside the container. Use a USB 3 port/cable. |
| Driver log shows `CONNECTION IS NOT AVAILABLE` and `process has died` | The robot is not reachable: check the cable/Wi‑Fi and `ping` the robot, use `wifi:=true` for Wi‑Fi, then restart the container. |
| Driver running elsewhere doesn't receive commands | Same `ROS_DOMAIN_ID` on both sides, both with `--net=host`. See the notes in [Go1 driver](#2-go1-driver). |
| Other robots/people on the same network receive the commands | Use a dedicated `ROS_DOMAIN_ID` (e.g. `ROS_DOMAIN_ID=42 docker compose up go1_suite`). |
| Robot stands up abruptly / looks unstable when standing up | Something else publishes `/cmd_vel` continuously (check `ros2 topic info /cmd_vel -v`): any `/cmd_vel` interrupts the stand-up motion. |
| Robot doesn't react to `/cmd_vel` | Make sure the robot is standing (`t` or the stand-up gesture) and that the driver is connected (check its logs). |
