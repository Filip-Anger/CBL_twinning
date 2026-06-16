## How to Use

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
   ros2 launch turtlebot3_bringup robot.launch.py
   ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=src/mapFiles/playground.yaml
   ros2 run plant-mapper farm_twin
   ros2 run plant-mapper farm_navigator
   ros2 run plant-mapper battery
```

**When things don't work, try these:**
```bash
   cd /ws
   source /opt/ros/jazzy/setup.bash 		
   colcon build					
   source install/setup.bash
   export TURTLEBOT3_MODEL=burger
```

