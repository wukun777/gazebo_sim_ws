# roarm_gazebo_plugins

Gazebo Classic 11 / ROS 2 Humble motor-force plugin for `roarm_quad`.

The plugin subscribes to:

- `/roarm_quad/rotor_0/cmd_force`
- `/roarm_quad/rotor_1/cmd_force`
- `/roarm_quad/rotor_2/cmd_force`
- `/roarm_quad/rotor_3/cmd_force`

It stores the latest four `geometry_msgs/msg/Wrench` commands and applies each
positive `force.z` at the configured rotor position during every Gazebo physics
step. It also publishes the resulting body-frame wrench at:

- `/roarm_quad/applied_wrench`

If commands stop for longer than the SDF `command_timeout`, thrust is set to
zero automatically.
