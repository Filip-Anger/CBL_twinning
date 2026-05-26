Ah, I see exactly what you mean now! You want the raw Markdown code blocks *inside* a code block so you can copy the exact text formatting for just that specific snippet, without it rendering on the screen.

Here is the exact Markdown code for your snippet:

```text
# Usage

## Connect and Bring Up the Robot
```bash
ssh turtlebot@192.168.8.38
ros2 launch bringup

```

---

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
