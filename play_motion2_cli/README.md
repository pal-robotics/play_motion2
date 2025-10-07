# play_motion2_cli

Command line interface for managing the play_motion2 aplication.

## list

The `list` verb is used for listing the existing motions.

Arguments:
- `--is-ready`, `-r`:Additionally show if the motion is ready.

Usage:
```bash
ros2 play_motion list [--is-ready | -r ]
```

## info

The `info` verb is used for displaying information about a motion. By default, it displays the key, the description and the joints. It can also display more specific information.

Arguments:
- `motion_name` : The name of the motion to obtain the information. This argument is required.
- `--verbose`, `-v`: Prints detailed information like the motion name, usage, joint positions, and times from start.

Usage:
```bash
ros2 play_motion info <motion_name> [--verbose | -v ]
```

## run

The `run` verb is used for execute a specified motion.

Arguments:
- `motion_name` : The name of the motion to execute. This argument is required.
- `--skip-planning`: Whether to skip planning for approaching to the first position or not.

Usage:
```bash
ros2 play_motion run <motion_name> [--skip-planning]
```