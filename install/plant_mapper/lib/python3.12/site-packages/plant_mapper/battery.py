import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
import tf2_ros
import json
import time
import math
from nav2_simple_commander.robot_navigator import BasicNavigator

# Physical constant for TurtleBot3 battery (11.1V, 1.8Ah)
TOTAL_BATTERY_JOULES = 71928.0

def get_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def estimate_true_energy(path_or_distance, max_vel=0.22, base_power=7.0, mass=1.0, mu=0.02, eff=0.65):
    if isinstance(path_or_distance, (int, float)):
        distance = float(path_or_distance)
    else:
        # Calculate path distance from Nav2 Path
        distance = 0.0
        for i in range(len(path_or_distance.poses) - 1):
            p1 = path_or_distance.poses[i].pose.position
            p2 = path_or_distance.poses[i+1].pose.position
            distance += ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)**0.5
            
    # 2. Estimate duration (average speed is max_vel/2.0 as per instructions)
    est_time = distance / (max_vel / 2.0) if distance > 0.0 else 0.0
    
    # 3. Static consumption (Pi + LiDAR + OpenCR idle)
    static_joules = base_power * est_time
    
    # 4. Dynamic consumption (Adjusted for gear/motor efficiency)
    mechanical_work = (mu * mass * 9.81 * (max_vel / 2.0)) * est_time
    electrical_dynamic_joules = mechanical_work / eff
    
    total_joules = static_joules + electrical_dynamic_joules
    return total_joules

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery')
        
        # Subscriptions
        self.battery_state_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_state_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        self.nav_feedback_sub = self.create_subscription(
            NavigateToPose_FeedbackMessage,
            '/navigate_to_pose/_action/feedback',
            self.nav_feedback_callback,
            10
        )
        
        # Publishers
        self.robot_battery_pub = self.create_publisher(
            Float32,
            '/robot_battery',
            10
        )
        self.robot_return_energy_pub = self.create_publisher(
            Float32,
            '/robot_return_energy',
            10
        )
        self.waypoints_pub = self.create_publisher(
            String,
            '/farm_waypoints',
            10
        )
        self.nav_status_pub = self.create_publisher(
            String,
            '/nav_status',
            10
        )
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize BasicNavigator (for path planning)
        self.get_logger().info("Initializing BasicNavigator for path planning...")
        self.navigator = BasicNavigator()
        
        # Node states
        self.current_battery_percent = 100.0
        self.last_battery_msg_time = time.time()
        self.last_battery_callback_time = 0.0
        self.last_battery_process_time = 0.0
        self.last_action_distance_remaining = None
        self.last_feedback_time = 0.0
        
        # Fallback Odom/AMCL poses
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0
        self.has_amcl = False
        
        # Starting point (charging base)
        self.x_start = None
        self.y_start = None
        self.yaw_start = None
        
        # Flag to track if return home mission is active
        self.is_returning = False
        
        # Timers
        # 1. Publish battery state to dashboard once per 5 seconds (throttled)
        self.battery_publish_timer = self.create_timer(1.0, self.battery_publish_loop)
        
        # 2. Main battery check loop (every 60 seconds)
        self.battery_check_timer = self.create_timer(60.0, self.battery_check_loop)
        
        self.get_logger().info("Battery Management Node initialized and running.")

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.odom_x = pose.position.x
        self.odom_y = pose.position.y
        self.odom_yaw = get_yaw_from_quaternion(pose.orientation)

    def amcl_callback(self, msg):
        pose = msg.pose.pose
        self.amcl_x = pose.position.x
        self.amcl_y = pose.position.y
        self.amcl_yaw = get_yaw_from_quaternion(pose.orientation)
        self.has_amcl = True

    def get_robot_pose(self):
        # 1. Try direct map-frame lookups using correct clock type
        map_frames = [
            ('map', 'base_link'),
            ('map', 'base_footprint'),
        ]
        for map_frame, base_frame in map_frames:
            try:
                t0 = rclpy.time.Time(clock_type=self.get_clock().clock_type)
                trans = self.tf_buffer.lookup_transform(
                    map_frame,
                    base_frame,
                    t0,
                    rclpy.duration.Duration(seconds=0.05)
                )
                pos = trans.transform.translation
                ori = trans.transform.rotation
                yaw = get_yaw_from_quaternion(ori)
                return {"x": pos.x, "y": pos.y, "yaw": yaw}
            except Exception:
                continue

        # 2. Try composed lookup to avoid extrapolation failures between map->odom and odom->base
        base_frames = ['base_link', 'base_footprint']
        for base_frame in base_frames:
            try:
                t0 = rclpy.time.Time(clock_type=self.get_clock().clock_type)
                t_mo = self.tf_buffer.lookup_transform('map', 'odom', t0, rclpy.duration.Duration(seconds=0.05))
                t_ob = self.tf_buffer.lookup_transform('odom', base_frame, t0, rclpy.duration.Duration(seconds=0.05))
                
                x_mo = t_mo.transform.translation.x
                y_mo = t_mo.transform.translation.y
                yaw_mo = get_yaw_from_quaternion(t_mo.transform.rotation)
                
                x_ob = t_ob.transform.translation.x
                y_ob = t_ob.transform.translation.y
                yaw_ob = get_yaw_from_quaternion(t_ob.transform.rotation)
                
                cos_yaw = math.cos(yaw_mo)
                sin_yaw = math.sin(yaw_mo)
                x_mb = x_mo + (x_ob * cos_yaw - y_ob * sin_yaw)
                y_mb = y_mo + (x_ob * sin_yaw + y_ob * cos_yaw)
                yaw_mb = yaw_mo + yaw_ob
                return {"x": x_mb, "y": y_mb, "yaw": yaw_mb}
            except Exception:
                continue

        # 3. Fall back to AMCL pose topic if available
        if self.has_amcl:
            return {"x": self.amcl_x, "y": self.amcl_y, "yaw": self.amcl_yaw}

        # 4. Try unlocalized odom TF lookups
        odom_frames = [
            ('odom', 'base_link'),
            ('odom', 'base_footprint'),
        ]
        for odom_frame, base_frame in odom_frames:
            try:
                t0 = rclpy.time.Time(clock_type=self.get_clock().clock_type)
                trans = self.tf_buffer.lookup_transform(
                    odom_frame,
                    base_frame,
                    t0,
                    rclpy.duration.Duration(seconds=0.05)
                )
                pos = trans.transform.translation
                ori = trans.transform.rotation
                yaw = get_yaw_from_quaternion(ori)
                return {"x": pos.x, "y": pos.y, "yaw": yaw}
            except Exception:
                continue

        # 5. Fall back to odom topic
        return {"x": self.odom_x, "y": self.odom_y, "yaw": self.odom_yaw}

    def nav_feedback_callback(self, msg):
        self.last_action_distance_remaining = msg.feedback.distance_remaining
        self.last_feedback_time = time.time()

    def battery_state_callback(self, msg):
        now = time.time()
        # Throttle processing to at most once per 5 seconds
        if now - self.last_battery_callback_time >= 5.0:
            self.last_battery_callback_time = now
            self.last_battery_msg_time = now
            if not math.isnan(msg.percentage):
                val = msg.percentage
                # Support both 0-1 and 0-100 formats
                if val <= 1.0:
                    val *= 100.0
                self.current_battery_percent = val
                self.get_logger().info(f"Received battery state: {self.current_battery_percent:.1f}%")
                
                # Publish to /robot_battery
                pub_msg = Float32()
                pub_msg.data = float(self.current_battery_percent)
                self.robot_battery_pub.publish(pub_msg)
                
                # Record base starting pose if not done
                if self.x_start is None:
                    pose = self.get_robot_pose()
                    if pose is not None:
                        self.x_start = pose["x"]
                        self.y_start = pose["y"]
                        self.yaw_start = pose["yaw"]
                        self.get_logger().info(f"Recorded charging base starting point: (x: {self.x_start:.2f}, y: {self.y_start:.2f})")

    def battery_publish_loop(self):
        now_time = time.time()
        # Mock drain if no battery state publisher has run for 10 seconds
        if now_time - self.last_battery_msg_time > 10.0:
            # Drain mock battery by 0.5% every 5 seconds (0.1% per second)
            self.current_battery_percent = max(0.0, self.current_battery_percent - 0.5)
            
            # Publish mock battery state once per 5 seconds
            if now_time - self.last_battery_process_time >= 5.0:
                self.last_battery_process_time = now_time
                msg = Float32()
                msg.data = float(self.current_battery_percent)
                self.robot_battery_pub.publish(msg)
                
                # Record base if not recorded yet
                if self.x_start is None:
                    pose = self.get_robot_pose()
                    if pose is not None:
                        self.x_start = pose["x"]
                        self.y_start = pose["y"]
                        self.yaw_start = pose["yaw"]
                        self.get_logger().info(f"Recorded charging base starting point: (x: {self.x_start:.2f}, y: {self.y_start:.2f})")

    def battery_check_loop(self):
        if self.x_start is None:
            self.get_logger().warn("Cannot check battery returns: Charging base position not initialized yet.")
            return
            
        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn("Cannot check battery returns: Current robot position unknown.")
            return
            
        # Determine the return distance
        return_distance = None
        
        # If returning home and we have recent action feedback, use it
        if self.is_returning and self.last_action_distance_remaining is not None and (time.time() - self.last_feedback_time < 5.0):
            return_distance = float(self.last_action_distance_remaining)
            self.get_logger().info(f"Using return distance from action feedback: {return_distance:.2f}m")
        else:
            # Calculate path back to charging base
            start_pose = PoseStamped()
            start_pose.header.frame_id = 'map'
            start_pose.header.stamp = self.get_clock().now().to_msg()
            start_pose.pose.position.x = float(pose["x"])
            start_pose.pose.position.y = float(pose["y"])
            start_pose.pose.position.z = 0.0
            start_pose.pose.orientation.x = 0.0
            start_pose.pose.orientation.y = 0.0
            start_pose.pose.orientation.z = 0.0
            start_pose.pose.orientation.w = 1.0
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(self.x_start)
            goal_pose.pose.position.y = float(self.y_start)
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0
            
            # Calculate return path using Nav2 ComputePathToPose (via BasicNavigator)
            path = None
            try:
                path = self.navigator.getPath(start_pose, goal_pose)
            except Exception as e:
                self.get_logger().warn(f"Failed to query getPath: {e}. Falling back to straight-line path.")
                
            # If path calculation fails, build mock path for straight-line distance
            if path is None:
                path = Path()
                path.poses = [start_pose, goal_pose]
                
            return_distance = path
            
        # Calculate energy cost in percentage
        total_joules = estimate_true_energy(return_distance)
        needed_to_return_percent = (total_joules / TOTAL_BATTERY_JOULES) * 100.0
        
        # Publish needed return energy to /robot_return_energy for display
        return_msg = Float32()
        return_msg.data = float(needed_to_return_percent)
        self.robot_return_energy_pub.publish(return_msg)
        
        self.get_logger().info(f"Battery check: level={self.current_battery_percent:.1f}%, needed_to_return={needed_to_return_percent:.1f}%")
        
        # Auto-reset self.is_returning if battery is reset/charged
        if self.is_returning and self.current_battery_percent > (needed_to_return_percent + 30.0):
            self.is_returning = False
            self.get_logger().info("Battery charged/reset. Resuming normal operations.")
        
        # Evaluate return condition: current <= needed to return + 25%
        # Only trigger once
        if not self.is_returning and self.current_battery_percent <= (needed_to_return_percent + 25.0):
            self.get_logger().warn("LOW BATTERY TRIGGERED! Forcing robot to return home immediately.")
            self.is_returning = True
            
            # Publish warning message to /nav_status to update GUI and show in log console
            status_msg = String()
            status_msg.data = json.dumps({
                "status": "returning to charge",
                "message": f"Low battery warning! Battery at {self.current_battery_percent:.1f}%. Returning to charging base."
            })
            self.nav_status_pub.publish(status_msg)
            
            # Send the robot home by publishing to /farm_waypoints
            home_wp = [{
                "x": float(self.x_start),
                "y": float(self.y_start),
                "type": "charge"
            }]
            wp_msg = String()
            wp_msg.data = json.dumps(home_wp)
            self.waypoints_pub.publish(wp_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
