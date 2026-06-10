import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros

import http.server
import socketserver
import threading
import json
import os
import queue
import math
import random
from ament_index_python.packages import get_package_share_directory

def get_yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True

class FarmTwin(Node):

    def __init__(self):
        super().__init__('farm_twin')

        self.declare_parameter('map_yaml', '/home/falinux/CBL_twinning/src/mapFiles/playground.yaml')
        self.declare_parameter('map_pgm', '/home/falinux/CBL_twinning/src/mapFiles/playground.pgm')
        self.declare_parameter('map_rotation', 0)
        self.declare_parameter('port', 8080)

        self.map_yaml = self.get_parameter('map_yaml').value
        self.map_pgm = self.get_parameter('map_pgm').value
        self.map_rotation = self.get_parameter('map_rotation').value
        self.port = self.get_parameter('port').value

        # Locate static web assets
        self.web_dir = self.locate_web_dir()
        self.get_logger().info(f"Web interface assets directory: {self.web_dir}")

        # Map Metadata (parsed from yaml)
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.width_pixels = 100
        self.height_pixels = 100

        # Load map info and convert PGM to PNG
        self.load_map_data()

        # Database to store plant scan data
        self.plants = []

        # Waypoints and status
        self.waypoints = []
        self.nav_status = "System Idle"
        self.current_waypoint_index = -1

        # Fallback Odom coordinates
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # Subscriptions
        self.plant_info_sub = self.create_subscription(
            String,
            '/plant_info',
            self.plant_info_callback,
            10
        )
        self.nav_status_sub = self.create_subscription(
            String,
            '/nav_status',
            self.nav_status_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Fallback AMCL coordinates
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0
        self.has_amcl = False

        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # Publisher for waypoints
        self.waypoints_pub = self.create_publisher(
            String,
            '/farm_waypoints',
            10
        )

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # SSE stream clients
        self.sse_clients = []
        self.sse_lock = threading.Lock()
        self.running = True

        # Start HTTP Server in background thread
        self.server_thread = threading.Thread(target=self.run_http_server, daemon=True)
        self.server_thread.start()

        # Timer to broadcast robot pose periodically (10Hz)
        self.pose_timer = self.create_timer(0.1, self.broadcast_pose)

    def locate_web_dir(self):
        # 1. Check workspace source folder for live editing
        src_path = '/home/falinux/CBL_twinning/src/plant_mapper/plant_mapper/web'
        if os.path.exists(src_path):
            return src_path
        # 2. Fallback to package share directory
        try:
            share_dir = get_package_share_directory('plant_mapper')
            return os.path.join(share_dir, 'web')
        except Exception:
            return os.path.abspath(os.path.join(os.path.dirname(__file__), 'web'))

    def load_map_data(self):
        # 1. Parse YAML file using safe fallback in case PyYAML is missing
        try:
            if os.path.exists(self.map_yaml):
                with open(self.map_yaml, 'r') as f:
                    for line in f:
                        line = line.strip()
                        if line.startswith('resolution:'):
                            self.resolution = float(line.split(':')[1].strip())
                        elif line.startswith('origin:'):
                            origin_str = line.split(':')[1].strip().strip('[]')
                            parts = [float(x) for x in origin_str.split(',')]
                            if len(parts) >= 2:
                                self.origin_x = parts[0]
                                self.origin_y = parts[1]
                self.get_logger().info(f"Parsed map: resolution={self.resolution}, origin=({self.origin_x}, {self.origin_y})")
            else:
                self.get_logger().warn(f"Map yaml file not found: {self.map_yaml}")
        except Exception as e:
            self.get_logger().error(f"Error parsing map yaml: {e}")

        # 2. Convert PGM to PNG and get dimensions
        png_path = os.path.join(self.web_dir, 'map.png')
        try:
            if os.path.exists(self.map_pgm):
                from PIL import Image
                with Image.open(self.map_pgm) as img:
                    img.save(png_path, 'PNG')
                    self.width_pixels, self.height_pixels = img.size
                self.get_logger().info(f"Converted map to PNG: {png_path} ({self.width_pixels}x{self.height_pixels} px)")
            else:
                self.get_logger().warn(f"Map pgm file not found: {self.map_pgm}")
        except Exception as e:
            self.get_logger().error(f"Error converting map image: {e}")

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

    def plant_info_callback(self, msg):
        pose = self.get_robot_pose()

        # Parse message payload
        try:
            payload = json.loads(msg.data)
            action_type = payload.get("type", "scan")
        except Exception:
            action_type = "scan"

        if action_type == "spray":
            self.send_log(
                "FarmTwin",
                f"Robot arrived and sprayed pesticide at (x: {pose['x']:.2f}, y: {pose['y']:.2f}).",
                "success"
            )
            self.broadcast_sse({
                "spray_event": {
                    "x": pose["x"],
                    "y": pose["y"]
                },
                "robot_pose": pose
            })
            return

        # Generate mock plant metrics
        dryness = random.uniform(10.0, 90.0)
        pest = random.uniform(0.0, 100.0)

        # Check if we already have a plant scanned close to this position
        updated = False
        for p in self.plants:
            dist = math.hypot(p["x"] - pose["x"], p["y"] - pose["y"])
            if dist < 0.25:
                p["dryness"] = dryness
                p["pest"] = pest
                updated = True
                break

        if not updated:
            self.plants.append({
                "x": pose["x"],
                "y": pose["y"],
                "dryness": dryness,
                "pest": pest
            })

        self.send_log(
            "FarmTwin",
            f"Robot arrived and scanned plant at (x: {pose['x']:.2f}, y: {pose['y']:.2f}). "
            f"Dryness: {dryness:.1f}%, Pest: {pest:.1f}%",
            "success"
        )

        self.broadcast_sse({
            "plants": self.plants,
            "robot_pose": pose
        })

    def nav_status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            self.nav_status = status_data.get("status", "Unknown")
            self.current_waypoint_index = status_data.get("current_waypoint_index", -1)
            
            self.send_log("Navigator", status_data.get("message", self.nav_status), "info")
            self.broadcast_sse({
                "nav_status": self.nav_status,
                "current_waypoint_index": self.current_waypoint_index
            })
        except Exception:
            self.nav_status = msg.data
            self.send_log("Navigator", msg.data, "info")
            self.broadcast_sse({
                "nav_status": self.nav_status
            })

    def broadcast_pose(self):
        if not self.running:
            return
        pose = self.get_robot_pose()
        self.broadcast_sse({
            "robot_pose": pose
        })

    def send_log(self, source, message, level="info"):
        if level == "error":
            self.get_logger().error(f"[{source}] {message}")
        elif level == "warn":
            self.get_logger().warn(f"[{source}] {message}")
        else:
            self.get_logger().info(f"[{source}] {message}")

        self.broadcast_sse({
            "log": {
                "source": source,
                "message": message,
                "level": level
            }
        })

    def broadcast_sse(self, data):
        with self.sse_lock:
            for q in self.sse_clients:
                q.put(data)

    def run_http_server(self):
        node = self
        class DashboardHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=node.web_dir, **kwargs)

            def end_headers(self):
                self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
                super().end_headers()

            def do_GET(self):
                if self.path == '/api/state':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    state = {
                        "config": {
                            "resolution": node.resolution,
                            "origin_x": node.origin_x,
                            "origin_y": node.origin_y,
                            "width_pixels": node.width_pixels,
                            "height_pixels": node.height_pixels
                        },
                        "plants": node.plants,
                        "robot_pose": node.get_robot_pose(),
                        "nav_status": node.nav_status,
                        "current_waypoint_index": node.current_waypoint_index
                    }
                    self.wfile.write(json.dumps(state).encode('utf-8'))

                elif self.path == '/api/stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/event-stream')
                    self.send_header('Cache-Control', 'no-cache')
                    self.send_header('Connection', 'keep-alive')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()

                    q = queue.Queue()
                    with node.sse_lock:
                        node.sse_clients.append(q)

                    try:
                        # Push initial state on connect
                        initial_state = {
                            "config": {
                                "resolution": node.resolution,
                                "origin_x": node.origin_x,
                                "origin_y": node.origin_y,
                                "width_pixels": node.width_pixels,
                                "height_pixels": node.height_pixels
                            },
                            "plants": node.plants,
                            "robot_pose": node.get_robot_pose(),
                            "nav_status": node.nav_status,
                            "current_waypoint_index": node.current_waypoint_index
                        }
                        self.wfile.write(f"data: {json.dumps(initial_state)}\n\n".encode('utf-8'))
                        self.wfile.flush()

                        while node.running:
                            try:
                                event_data = q.get(timeout=0.5)
                                self.wfile.write(f"data: {json.dumps(event_data)}\n\n".encode('utf-8'))
                                self.wfile.flush()
                            except queue.Empty:
                                # keepalive ping
                                self.wfile.write(b": keepalive\n\n")
                                self.wfile.flush()
                    except (ConnectionResetError, BrokenPipeError):
                        pass
                    finally:
                        with node.sse_lock:
                            if q in node.sse_clients:
                                node.sse_clients.remove(q)
                else:
                    super().do_GET()

            def do_POST(self):
                content_length = int(self.headers['Content-Length'] or 0)
                body = self.rfile.read(content_length).decode('utf-8') if content_length > 0 else ""

                if self.path == '/api/send_waypoints':
                    try:
                        node.waypoints = json.loads(body)
                        node.send_log(
                            "FarmTwin",
                            f"Received {len(node.waypoints)} waypoints from dashboard. Routing mission...",
                            "info"
                        )
                        
                        # Publish waypoints list to /farm_waypoints
                        msg = String()
                        msg.data = json.dumps(node.waypoints)
                        node.waypoints_pub.publish(msg)
                        
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
                    except Exception as e:
                        self.send_response(400)
                        self.end_headers()
                        self.wfile.write(f"Error parsing waypoints: {e}".encode('utf-8'))

                elif self.path == '/api/trigger_scan':
                    pose = node.get_robot_pose()
                    dryness = random.uniform(10.0, 90.0)
                    pest = random.uniform(0.0, 100.0)
                    
                    updated = False
                    for p in node.plants:
                        dist = math.hypot(p["x"] - pose["x"], p["y"] - pose["y"])
                        if dist < 0.25:
                            p["dryness"] = dryness
                            p["pest"] = pest
                            updated = True
                            break
                    if not updated:
                        node.plants.append({
                            "x": pose["x"],
                            "y": pose["y"],
                            "dryness": dryness,
                            "pest": pest
                        })
                    
                    # Sync scanned plant to Python pest grid
                    row, col = node.physical_to_grid(pose["x"], pose["y"])
                    pest_spread.on_plant_scanned(row, col, pest)

                    node.send_log(
                        "FarmTwin",
                        f"Manual crop scan triggered at current position (x: {pose['x']:.2f}, y: {pose['y']:.2f}). "
                        f"Dryness: {dryness:.1f}%, Pest: {pest:.1f}%",
                        "success"
                    )
                    
                    node.broadcast_sse({
                        "plants": node.plants
                    })
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(json.dumps({
                        "status": "ok",
                        "x": pose["x"],
                        "y": pose["y"]
                    }).encode('utf-8'))
 
                elif self.path == '/api/clear':
                    # Reset database and planned waypoints
                    node.plants = []
                    node.waypoints = []
                    node.nav_status = "System Idle"
                    node.current_waypoint_index = -1
                    
                    node.send_log("FarmTwin", "Reset digital twin databases.", "warn")
                    node.broadcast_sse({
                        "plants": node.plants,
                        "nav_status": node.nav_status,
                        "current_waypoint_index": node.current_waypoint_index
                    })
                    
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))

                elif self.path == '/api/set_rotation':
                    try:
                        data = json.loads(body)
                        angle = int(data.get("rotation", 0))
                        if angle in [0, 90, 180, 270]:
                            node.map_rotation = angle
                            node.load_map_data()
                            
                            node.send_log("FarmTwin", f"Map rotation updated to {angle}°.", "info")
                            node.broadcast_sse({
                                "config": {
                                    "resolution": node.resolution,
                                    "origin_x": node.origin_x,
                                    "origin_y": node.origin_y,
                                    "width_pixels": node.width_pixels,
                                    "height_pixels": node.height_pixels,
                                    "map_rotation": node.map_rotation
                                }
                            })
                            
                            self.send_response(200)
                            self.send_header('Content-Type', 'application/json')
                            self.send_header('Access-Control-Allow-Origin', '*')
                            self.end_headers()
                            self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
                        else:
                            self.send_response(400)
                            self.end_headers()
                            self.wfile.write(b"Invalid rotation. Must be 0, 90, 180, or 270.")
                    except Exception as e:
                        self.send_response(400)
                        self.end_headers()
                        self.wfile.write(f"Error: {e}".encode('utf-8'))
                else:
                    self.send_response(404)
                    self.end_headers()

            def do_OPTIONS(self):
                self.send_response(200)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()

        port = self.port
        bound = False
        while not bound and port < self.port + 100:
            try:
                self.httpd = ThreadingHTTPServer(('0.0.0.0', port), DashboardHTTPRequestHandler)
                self.port = port
                bound = True
            except OSError as e:
                if e.errno == 98:
                    node.get_logger().info(f"Port {port} is in use, trying {port+1}...")
                    port += 1
                else:
                    node.get_logger().error(f"Failed to bind to port {port}: {e}")
                    return

        if not bound:
            node.get_logger().error("Could not find a free port to bind to.")
            return

        self.get_logger().info(f"Starting web server on http://localhost:{self.port} ...")
        try:
            self.httpd.serve_forever()
        except Exception as e:
            self.get_logger().info(f"Web server stopped: {e}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'httpd'):
            self.httpd.shutdown()
            self.httpd.server_close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FarmTwin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
