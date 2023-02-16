# PlayMotion2

This repository consists of `play_motion2`, a tool to play and handle pre-recorded motions in ROS2,
and its associated messages in `play_motion2_msgs`.
PlayMotion2 allows executing simultaneous trajectories in multiple groups of joints.

## Prerequisites

The motions yaml file should follow the following format:

```yaml
/play_motion2:
  ros__parameters:

    # params for first position approach
    approach_velocity: 0.5        # default 0.5
    approach_min_duration: 0.0    # default 0.0

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
  - Trajectory 1: both `joint1` and `joint2` move from their starting position until reach their respective point 0.0
  - Trajectory 2: `joint1` goes from 0.0 to 0.5 and `joint2` moves from 0.0 to 0.25.

The field `times_from_start` specifies the time to reach each position from the starting time
In case of `motion_1` the joints will move to the first position in 0.5s, and to the second position in 0.5s.

However, since the starting position of the joint could be further than expected,
an approach time to the first position is calculated for safety reasons.
Then, if the approach time for `motion_1` is lower than 0.5, the first trajectory will last 0.5s,
but if it's higher, it will take the approach time calculated.

The approach velocity and minimum duration can be chosen with their respective parameters.

## Usage

In order to use Play Motion 2, it has to be started with the following command

```bash
ros2 launch play_motion2 play_motion2.launch.py play_motion2_config:=PATH_TO_MOTIONS_YAML [use_sim_time:=false]
```

Once started, motion goals can be sent from another terminal:

> **Disclaimer**: In the current version all motions are executed WITHOUT planning.
>
> Planning capability will be available in the future.

```bash
ros2 action send_goal /play_motion2 play_motion2_msgs/action/PlayMotion2 "motion_name: motion_1"
```

PlayMotion2 also includes two services:

- To list the available motions

```bash
ros2 service call /play_motion2/list_motions play_motion2_msgs/srv/ListMotions
```

- To check if a motion is ready to be executed or not

```bash
ros2 service call /play_motion2/is_motion_ready play_motion2_msgs/srv/IsMotionReady "motion_key: motion_1"
```
