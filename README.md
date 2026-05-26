## How to Use Packages

### Safety Stop

1. **Start the simulation:**

```bash
   ros2 launch tb3_safety_stop gazebo_twin.launch.py

```

2. **Start the safety node:**

```bash
   ros2 launch tb3_safety_stop twin_safety.launch.py 

```

3. **Run teleop publishing to `cmd_vel_raw`:**

```bash
   ros2 run turtlebot3_teleop teleop_keyboard /cmd_vel:=/cmd_vel_raw

```

```

```
