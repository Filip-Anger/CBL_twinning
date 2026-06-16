## How to Use Packages

**When things don't work, try these:**
```bash
   cd /ws
   source /opt/ros/jazzy/setup.bash 		
   colcon build					
   source install/setup.bash
   export TURTLEBOT3_MODEL=burger
```
### Final PoC simple start-up

**Simulation:**

```bash
   ros2 launch plant_mapper simulation_all.launch.py

```

**Physical robot:**

```bash
   ros2 launch plant_mapper physical_all.launch.py

```

### Final PoC simple start-up

**Simulation:**

```bash
   ros2 launch my_tb3_world new_world.launch.py
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/mapFiles/playground.yaml
   ros2 run plant-mapper farm_twin
   ros2 run plant-mapper farm_navigator
   ros2 run plant-mapper battery
```

**Physical robot:**

```bash
   ros2 launch plant_mapper physical_all.launch.py

```

3. **Run teleop publishing to `cmd_vel_raw`:**

```bash
   ros2 run turtlebot3_teleop teleop_keyboard /cmd_vel:=/cmd_vel_raw
```

### Manual start-up
Connected to the robot, ready to drive

1. **Start the map recording:**

```bash
   ros2 launch border_recorder border_recorder.launch.py
```
2. **Call the /save_border service:**

```bash
   ros2 service call /save_border std_srvs/srv/Trigger
```


launch SLAM maping
```bash
   ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
    
save map
```bash
       ros2 run nav2_map_server map_saver_cli -f src/mapFiles/playground
```


SLAM navigation
```bash
       ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/mapFiles/playground.yaml
```


Waypoint follower errors:
```bash
[rviz2-2] [INFO] [1780426287.326918303] [rviz_navigation_dialog_action_client]: NavigateThroughPoses will be called using the BT Navigator's default behavior tree.
[component_container_isolated-1] [INFO] [1780426287.328151830] [bt_navigator]: Begin navigating from current location through 3 poses to (0.56, -2.60)
[component_container_isolated-1] [ERROR] [1780426288.424466960] [transformPoseInTargetFrame]: Failed to transform from  to map
[component_container_isolated-1] [WARN] [1780426288.424610975] [planner_server]: GridBased plugin failed to plan from (0.01, -0.05) to (0.00, 0.00): "Unable to transform poses to global frame"
[component_container_isolated-1] [WARN] [1780426288.424678615] [planner_server]: [compute_path_to_pose] [ActionServer] Aborting handle.
[component_container_isolated-1] [WARN] [1780426288.448548996] [bt_navigator]: [navigate_through_poses] [ActionServer] Aborting handle.
[component_container_isolated-1] [ERROR] [1780426288.448750517] [bt_navigator]: Goal failed
```


