import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import json
import time

class FarmNavigator(Node):

    def __init__(self):
        super().__init__('farm_navigator')

        # Subscriptions
        self.waypoints_sub = self.create_subscription(
            String,
            '/farm_waypoints',
            self.waypoints_callback,
            10
        )

        # Publishers
        self.plant_info_pub = self.create_publisher(
            String,
            '/plant_info',
            10
        )
        self.nav_status_pub = self.create_publisher(
            String,
            '/nav_status',
            10
        )

        # Initialize BasicNavigator
        self.get_logger().info("Initializing BasicNavigator...")
        self.navigator = BasicNavigator()

        # State machine variables
        self.pending_waypoints = None
        self.active_waypoints = None
        self.current_wp_idx = -1
        self.state = "IDLE"  # IDLE, INIT_NAV2, START_WAYPOINT, MONITOR_NAV, WAITING
        self.wait_start_time = 0.0

        self.get_logger().info("FarmNavigator Node initialized and running in single-threaded mode.")

    def waypoints_callback(self, msg):
        try:
            wps = json.loads(msg.data)
            if not wps:
                self.get_logger().warn("Empty waypoints list received. Ignoring.")
                return

            self.get_logger().info(f"Received new mission request with {len(wps)} waypoints.")
            # Set pending waypoints to be processed in the main thread loop
            self.pending_waypoints = wps
        except Exception as e:
            self.get_logger().error(f"Failed to parse waypoints: {e}")

    def publish_status(self, status, message, wp_idx=-1):
        status_data = {
            "status": status,
            "message": message,
            "current_waypoint_index": wp_idx
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.nav_status_pub.publish(msg)
        self.get_logger().info(message)

    def step(self):
        # 1. Check if a new mission has been requested
        if self.pending_waypoints is not None:
            # If we were already navigating, cancel the active task
            if self.state in ["MONITOR_NAV", "WAITING", "START_WAYPOINT"]:
                self.get_logger().info("Cancelling current navigation task to start new mission...")
                self.navigator.cancelTask()
            
            self.active_waypoints = self.pending_waypoints
            self.pending_waypoints = None
            self.current_wp_idx = 0
            self.state = "INIT_NAV2"

        # 2. State Machine transitions
        if self.state == "INIT_NAV2":
            self.publish_status("Idle", "Connecting to Nav2 action servers...", -1)
            self.navigator.waitUntilNav2Active()
            self.state = "START_WAYPOINT"

        elif self.state == "START_WAYPOINT":
            if self.active_waypoints and self.current_wp_idx < len(self.active_waypoints):
                wp = self.active_waypoints[self.current_wp_idx]
                self.publish_status(
                    "Navigating",
                    f"Navigating to waypoint {self.current_wp_idx + 1}/{len(self.active_waypoints)} at (x: {wp['x']:.2f}, y: {wp['y']:.2f})...",
                    self.current_wp_idx
                )

                # Define navigation goal pose
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(wp['x'])
                pose.pose.position.y = float(wp['y'])
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0

                self.navigator.goToPose(pose)
                self.state = "MONITOR_NAV"
            else:
                self.publish_status("Complete", "Mission finished! All waypoints visited successfully.", -1)
                self.active_waypoints = None
                self.state = "IDLE"

        elif self.state == "MONITOR_NAV":
            # isTaskComplete() spins the navigator node internally to check action status
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                wp = self.active_waypoints[self.current_wp_idx]

                if result == TaskResult.SUCCEEDED:
                    self.publish_status(
                        "Arrived",
                        f"Arrived at waypoint {self.current_wp_idx + 1}/{len(self.active_waypoints)} at (x: {wp['x']:.2f}, y: {wp['y']:.2f}). Executing crop scan...",
                        self.current_wp_idx
                    )

                    # Trigger crop scan
                    trigger_msg = String()
                    trigger_msg.data = json.dumps({
                        "waypoint_index": self.current_wp_idx,
                        "x": wp["x"],
                        "y": wp["y"]
                    })
                    self.plant_info_pub.publish(trigger_msg)

                    self.wait_start_time = time.time()
                    self.state = "WAITING"

                elif result == TaskResult.CANCELED:
                    self.publish_status("Idle", f"Mission cancelled at waypoint {self.current_wp_idx + 1}.", -1)
                    self.active_waypoints = None
                    self.state = "IDLE"

                elif result == TaskResult.FAILED:
                    self.publish_status("Idle", f"Failed to navigate to waypoint {self.current_wp_idx + 1}.", -1)
                    self.active_waypoints = None
                    self.state = "IDLE"

        elif self.state == "WAITING":
            # Pause to simulate taking action/scanning
            if time.time() - self.wait_start_time >= 3.0:
                self.current_wp_idx += 1
                self.state = "START_WAYPOINT"

def main(args=None):
    rclpy.init(args=args)
    node = FarmNavigator()
    
    try:
        # Single-threaded loop: spin once to process subscription calls, then advance state
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.step()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
