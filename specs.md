# aatb_controllers

a ros2 package including custom controllers for aatb's art practice involving robots.

the first goal of this repo is to create a ros2 controller, similar to a JointGroupPositionController, taking in simple std_msgs:Float64MultiArray messages, but -crucially- not forwarding them as-is. It uses the Ruckig motion generator to first pass the command setpoints to it as desired target, and then forwards the resulting constrained target to the hardware.

this is for linux only.

# goal

create a ConstrainedPositionController that behaves similarly to a JointGroupPositionController, but uses Ruckig with a config file set constrains and joints. look up https://github.com/ros-controls/ros2_controllers/tree/master/position_controllers and model it to work similarly, using Ruckig.