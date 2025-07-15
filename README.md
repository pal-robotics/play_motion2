# PlayMotion2

This repository consists of `play_motion2`, a tool to play and handle pre-recorded motions in ROS 2,
and its associated messages in `play_motion2_msgs`.

PlayMotion2 allows executing simultaneous trajectories in multiple groups of joints.

## Prerequisites

The motions yaml file should follow the following format:

```yaml
/play_motion2_mgr:
  ros__parameters:
    motions:
      motion_1:
        joints: [joint1, joint2]
        positions: [0.0, 0.0,
                    0.5, 0.25]
        times_from_start: [0.5, 1.0]
        meta:
          name: Motion_1
          usage: example
          description: 'Example Motion'

      motion_2:
        ...

      motion_3:
        ...
```

The field `positions` of the `motion_1` there contains two rows and two columns:
- Each column corresponds to one joint, so the first column is for `joint1` and the second one for `joint2`
- Each row is one position to reach. Then, `motion_1` consists of 2 trajectories for the specified joints:
  - Trajectory 1: both `joint1` and `joint2` move from their starting position until reach their respective point `0.0`.
  - Trajectory 2: `joint1` goes from `0.0` to `0.5` and `joint2` moves from `0.0` to `0.25`.

The field `times_from_start` specifies the time to reach each position from the starting time
In case of `motion_1` the joints will move to the first position in 0.5s, and to the second position in 0.5s.

However, since the starting position of the joint could be further than expected, an approach time to the first position is calculated for safety reasons. Then, if the calculated approach time for `motion_1` is lower than `0.5`, the first trajectory will last 0.5s, but if it's higher, it will take the approach time calculated.

You can also configure the motion planner as follows:

```yaml
/play_motion2_executor:
  ros__parameters:
    motion_planner:
      disable_motion_planning: false
      planning_groups: # Sorted by order of preference
        - planning_group_1
        - planning_group_2
      exclude_from_planning_joints:
        - joint_1
        - joint_2
        - joint_3
      joint_tolerance: 0.01

      # Non-planning parameters
      approach_velocity: 0.5
      approach_min_duration: 0.5
```

The `approach_velocity` and `approach_min_duration` parameters are used for non-planned approaches.
- `approach_velocity`: Maximum velocity for the approach. The default value is `0.5`.
- `approach_min_duration`: Minimum duration for the approach. The default value is `0.5`.

The parameter `disable_motion_planning` is used to decide whether to use planning or not. The default value is `false`, so the planning is enabled for the approach.

The rest of the parameters are used when planning is enabled:
- `planning_groups`: defines the MoveIt 2 groups to plan the approach of the motions. This parameter is mandatory when using planning.
- `exclude_from_planning_joints`: List of joints that are excluded for planning.
- `joint_tolerance`: Joint tolerance set when planning the approach. The default value is `0.01`.

When using planning, the approach to the first position will be planned using MoveIt 2. Then, the planned trajectory is combined with the original motion in order to safely reach the first position of the motion and perform it.

> **Disclaimer**: It is strongly recommended to use planning, otherwise there might be collisions when reaching the first position of the motion.

## Usage

In order to use PlayMotion2, it has to be started with the following command

```bash
ros2 launch play_motion2 play_motion2.launch.py motions_file:=PATH_TO_MOTIONS_YAML motion_planner_config:=PATH_TO_MOTION_PLANNER_CONFIG [use_sim_time:=true]
```

Launcher parameters:
- `motions_file`: Path to motions file. This parameter is mandatory.
- `motion_planner_config`: Path to motion planner configuration. This parameter is mandatory.
- `use_sim_time`: `true` or `false`. Decide whether to use simulation time or not.

Once started, motion goals can be sent from another terminal:
```bash
ros2 run play_motion2 run_motion <motion_name> [<skip_planning> <timeout>]
```
where
```
<motion_name>    - Name of the motion to run.
<skip_planning>  - Whether to skip planning for approaching to the first position or not. (default: false)
<timeout>        - Timeout (in seconds) to wait for the motion. (default: 120)
```

or calling directly the action with the `ros2cli`:
```bash
ros2 action send_goal /play_motion2 play_motion2_msgs/action/PlayMotion2 "{motion_name: '', skip_planning: false}"
```

Take in account that the goals will be planned or not depending on the following combinations:
| `disable_motion_planning` | `skip_planning` | Result         |
| :-----------------------: | :-------------: | :------------: |
| `false`                   | `false`         | Using planning |
| `false`                   | `true`          | Not planned    |
| `true`                    | `false`         | Goal rejected  |
| `true`                    | `true`          | Not planned    |

PlayMotion2 also includes the following services:

- To list the available motions

```bash
ros2 service call /play_motion2/list_motions play_motion2_msgs/srv/ListMotions
```

- To check if a motion is ready to be executed or not

```bash
ros2 service call /play_motion2/is_motion_ready play_motion2_msgs/srv/IsMotionReady "motion_key: ''"
```

- To get the information of a motion

```bash
ros2 service call /play_motion2/get_motion_info play_motion2_msgs/srv/GetMotionInfo "motion_key: ''"
```

- To add a new motion
```bash
ros2 service call /play_motion2/add_motion play_motion2_msgs/srv/AddMotion "motion:
  key: ''
  name: ''
  usage: ''
  description: ''
  joints: []
  positions: []
  times_from_start: []
overwrite: false"
```

- To remove an existing motion
```bash
ros2 service call /play_motion2/remove_motion play_motion2_msgs/srv/RemoveMotion "motion_key: ''"
```
