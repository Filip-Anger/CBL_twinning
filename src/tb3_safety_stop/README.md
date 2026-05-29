### Safey Stop
#### Function: stop when moving if obstacle is in path

#### Implementation
- Gazebo simulation eats at /cmd_vel_sim
- The twin_safety_node takes in /cmd_vel_raw and publishes to /cmd_vel for real robot and cmd_vel_sim for simulated gazebo instance
