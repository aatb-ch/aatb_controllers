# aatb_controllers

Repo for ROS2 controllers used in AATB's artworks but applicable to other fields requiring smooth, on-the-fly trajectories towards discrete target positions.

Tested in Humble.

## ConstrainedPositionController

Position controller that applies velocity, acceleration, and jerk limits using Ruckig before forwarding commands to hardware. Receives `std_msgs/Float64MultiArray` commands like JointGroupPositionController but with smooth constrained motion. Useful for servoing to desired position as fast as possible while respecting robot's kinematic limits.

## Build

```bash
colcon build --packages-select aatb_controllers
```

## Testing

Launch simulated UR10e:
```bash
ros2 launch aatb_controllers test_ur10e.launch.py
```

open rviz example 

Send test command:
```bash
ros2 topic pub --once /constrained_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.8, 2.0, -1.2, 0.3, 1.57]}"
```

## License

See LICENSE
