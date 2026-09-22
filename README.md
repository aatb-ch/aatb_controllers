# aatb_controllers

Repo for ROS2 controllers used in AATB's artworks but applicable to other fields requiring smooth, on-the-fly trajectories towards discrete target positions.

Tested in Humble.

## ConstrainedPositionController

Position controller that applies velocity, acceleration, and jerk limits using Ruckig before forwarding commands to hardware. Receives `std_msgs/Float64MultiArray` commands like JointGroupPositionController but with smooth constrained motion. Useful for servoing to desired position as fast as possible while respecting robot's kinematic limits.

## Parameters

| parameter | type | default | |
|---|---|---|---|
| `joints` | string[] | *required* | Joint names, in the order commands arrive in. |
| `velocity_limits` | double[] | *required* | Per joint, rad/s. One entry per joint. |
| `acceleration_limits` | double[] | *required* | Per joint, rad/s². |
| `jerk_limits` | double[] | *required* | Per joint, rad/s³. |
| `control_cycle_time` | double | `0.001` | Seconds. **Must match the controller_manager `update_rate`**, since it is the timestep Ruckig integrates with. |
| `interface_name` | string | `position` | Command interface to claim. |
| `speed_scaling.state_interface` | string | `""` | Optional. A state interface carrying a speed override, e.g. `speed_scaling/speed_scaling_factor` on a UR. Empty disables it. |
| `max_tracking_error` | double | `0.35` | Radians. How far the commanded position may run ahead of the measured one before the trajectory is re-seeded from the machine. |
| `position_limits.min` | double[] | `[]` | Optional hard joint window, absolute radians, one per joint. Empty disables. |
| `position_limits.max` | double[] | `[]` | Upper bound of the same window. |

### `speed_scaling.state_interface`

The override is applied as a **change of clock**: the trajectory is planned at
the nominal limits above and the controller steps `control_cycle_time * s` of
trajectory time per real cycle. A scaling of 0 is therefore a standstill, and
the override can be moved freely mid-motion without invalidating anything.

Do not be tempted to scale the limits instead (`vel*s, acc*s², jerk*s³`). That
is the correct transform for planning a *new* trajectory, but applied to one
already in flight it hands Ruckig a carried-over `current_velocity` and
`current_acceleration` produced under the previous cycle's limits, and the s²/s³
terms collapse the deceleration authority far faster than the velocity it has to
arrest. On a UR10e that produced an arm which crossed its target without
decelerating and ran to the joint limit.

The value is validated before use: anything above 1.5 is treated as a percentage
and divided by 100, then it is clamped to `[0, 1]`. The raw value is logged once
at startup. This matters because conventions differ — on a UR the *state
interface* is a fraction while the matching broadcaster *topic* publishes
percent.

### `max_tracking_error`

Ruckig integrates from its own output, so without this nothing ties the
commanded position to the machine. Under a speed override the robot
deliberately cannot keep up, and an open-loop generator will drift away from it
without bound. Beyond this error the controller logs which joint, and re-seeds
`current_position` from the measured state so the trajectory replans from where
the arm actually is.

Set it larger than your normal following error and smaller than a distance you
would not want the arm to travel unnoticed.

### `position_limits`

A hard joint window enforced inside the control loop, on the target *and* on the
value written to hardware, before `pass_to_input` so a clamp also bounds the
state Ruckig carries into the next cycle. A clamp is logged as an error: it is
meant to be unreachable, so reaching it means something upstream has failed.

If a joint is measured **outside** its window the controller holds and writes
nothing, naming the joint. Clamping the setpoint into the window while the arm
is outside it would command a move of the full distance between the two, which
is how an arm parked at a limit gets asked to make a full turn. Move it back
inside with another controller first (on a UR, `scaled_joint_trajectory_controller`).

> Ruckig's own `max_position`/`min_position` are **not** used, and setting them
> is not an alternative: in the community build they are read only by
> `calculator_online.hpp`, so they would look like a safety bound and do nothing.

## Behaviour notes

- The trajectory is re-seeded from the measured position on activation, and the
  command interfaces are pre-seeded so `write()` sends a sane value before the
  first `update()`.
- Ruckig's result is checked **before** its output is written. A failed solve
  holds the last command rather than putting its output on the wire.
- Commands containing NaN or infinity are rejected with an error.

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
