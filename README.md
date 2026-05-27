## How to Use Packages

**When things don't work, try these:**
```bash
   cd /ws
   source /opt/ros/jazzy/setup.bash 		
   colcon build					
   source install/setup.bash
   export TURTLEBOT3_MODEL=burger
```
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

### Record Border
Connected to the robot, ready to drive

1. **Start the map recording:**

```bash
   ros2 launch border_recorder border_recorder.launch.py
```

os2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

2. **Call the /save_border service:**

```bash
   ros2 service call /save_border std_srvs/srv/Trigger
```

