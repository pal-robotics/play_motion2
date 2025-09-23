# play_motion2_cli

Command line interface for managing the play_motion2 aplication.

## list

The `list` verb is used for listing the existing motions.

Arguments:
- `--motion-ready`, `-r`:Additionally show if the motion is ready.

Usage:
```bash
ros2 playmotion list [--motion-ready | -r ]
```

## info

The `info` verb is used for displaying information about a motion. By default, it displays the key, the descritpion and the joints. It can also display more specific information.

Arguments:
- `--motion_name` : The name of the motion to obtain the information. This argument is required.
- `--verbose`, `-v`: Prints detailed information like the motion name, usage, joint positions, and times from start.

Usage:
```bash
ros2 playmotion info <motion_name> [--verbose | -v ]
```

## run

The `run` verb is used for execute a specified motion.

Arguments:
- `--motion_name` : The name of the motion to execute. This argument is required.

Usage:
```bash
ros2 playmotion run <motion_name>
```