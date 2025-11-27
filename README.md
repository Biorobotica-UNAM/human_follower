# human_follower

`human_follower` is a ROS 2 package that makes a mobile robot follow a person using the estimated leg position provided by a leg-detection node (e.g., `leg_finder`).  
The node subscribes to the detected leg pose, computes proportional control, and publishes velocity commands to `/cmd_vel`.

---

## Features
- Subscribes to a leg pose topic (`/hri/leg_finder/leg_pose`).
- Publishes velocity commands to `/cmd_vel`.
- Uses TF2 for frame transformations.
- Simple proportional controller (configurable via parameters).
- Always running; only acts when a valid leg pose is received.

---

## Dependencies
Make sure the following are installed and sourced:

- ROS 2 Humble
- `geometry_msgs`
- `std_msgs`
- `tf2` / `tf2_ros`
- A leg-detection node publishing a `geometry_msgs/PointStamped`

---

## Build

```bash
cd ~/catkin_ws
colcon build --packages-select human_follower
source install/setup.bash
```

---

## Run

### 1. Start the launch node
```bash
ros2 launch human_follower human_follower.launch.xml
```

---

## Parameters

| Name            | Type  | Default | Description                         |
|-----------------|--------|---------|-------------------------------------|
| `control_alpha` | float  | 0.6548  | Linear gain                         |
| `control_beta`  | float  | 0.3     | Angular gain                        |
| `max_linear`    | float  | 0.3     | Max linear velocity (m/s)           |
| `max_angular`   | float  | 0.7     | Max angular velocity (rad/s)        |

---

## Topics

### Subscribed
- **`/hri/leg_finder/leg_pose`** — `geometry_msgs/PointStamped`

### Published
- **`/cmd_vel`** — `geometry_msgs/Twist`

### Service
- **`/hri/human_tracking/enable`** - `std_srvs/SetBool`


---
