#!/usr/bin/env python3
import json
import time
import urllib.request
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class WeatherAdapter(Node):
    """
    Fetches current weather from Open-Meteo and publishes DT context:
      /twin/context/rainy              (Bool)
      /twin/limits/speed_scale         (Float32)  e.g., 1.0 dry, 0.6 rainy
      /twin/limits/stop_distance_add   (Float32)  e.g., 0.0 dry, 0.15 rainy
      /twin/weather/debug              (String)
    Fail-safe:
      - keep last good values for max_stale_s
      - after that, switch to conservative defaults (rainy profile)
    """
    def __init__(self):
        super().__init__("weather_adapter")

        # Location (set to your campus city)
        self.declare_parameter("latitude", 52.37)
        self.declare_parameter("longitude", 4.90)

        # Update + timeouts
        self.declare_parameter("update_period_s", 60.0)
        self.declare_parameter("http_timeout_s", 3.0)
        self.declare_parameter("max_stale_s", 300.0)  # keep last for 5 minutes

        # Thresholds (simple & robust)
        self.declare_parameter("rain_mm_threshold", 0.0)     # rainy if rain > this (mm/h)
        self.declare_parameter("visibility_m_threshold", 500.0)  # optional (meters)
        self.declare_parameter("wind_ms_threshold", 12.0)    # optional (m/s)

        # Profiles
        self.declare_parameter("speed_scale_dry", 1.0)
        self.declare_parameter("speed_scale_rain", 0.6)      # your example
        self.declare_parameter("stop_add_dry", 0.0)
        self.declare_parameter("stop_add_rain", 0.15)        # your example

        # Demo/testing (no internet needed)
        self.declare_parameter("force_rainy", False)

        self.pub_rainy = self.create_publisher(Bool, "/twin/context/rainy", 10)
        self.pub_speed = self.create_publisher(Float32, "/twin/limits/speed_scale", 10)
        self.pub_stop_add = self.create_publisher(Float32, "/twin/limits/stop_distance_add", 10)
        self.pub_debug = self.create_publisher(String, "/twin/weather/debug", 10)

        self.last_good_time: Optional[float] = None
        self.last_rainy: bool = False
        self.last_speed: float = float(self.get_parameter("speed_scale_dry").value)
        self.last_stop_add: float = float(self.get_parameter("stop_add_dry").value)

        period = float(self.get_parameter("update_period_s").value)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info("WeatherAdapter started (Open-Meteo, no API key).")

    def tick(self):
        if bool(self.get_parameter("force_rainy").value):
            rainy = True
            info = "FORCED rainy (force_rainy:=true)"
            self._publish(rainy, info, good=True)
            return

        rainy, dbg, ok = self._fetch_open_meteo()
        if ok:
            self._publish(rainy, dbg, good=True)
            return

        # Fetch failed -> fail-safe behavior
        max_stale = float(self.get_parameter("max_stale_s").value)
        now = time.monotonic()
        if self.last_good_time is not None and (now - self.last_good_time) <= max_stale:
            # keep last good
            self._publish(self.last_rainy, f"{dbg} | using cached last-good", good=False)
        else:
            # conservative defaults (rainy profile)
            self._publish(True, f"{dbg} | stale>max_stale -> conservative(rainy)", good=False, force_profile="rain")

    def _publish(self, rainy: bool, dbg: str, good: bool, force_profile: Optional[str] = None):
        # choose profile
        if force_profile == "rain":
            rainy = True

        speed = float(self.get_parameter("speed_scale_rain").value) if rainy else float(self.get_parameter("speed_scale_dry").value)
        stop_add = float(self.get_parameter("stop_add_rain").value) if rainy else float(self.get_parameter("stop_add_dry").value)

        self.pub_rainy.publish(Bool(data=rainy))
        self.pub_speed.publish(Float32(data=speed))
        self.pub_stop_add.publish(Float32(data=stop_add))

        msg = String()
        msg.data = f"[good={good}] rainy={rainy} speed_scale={speed} stop_add={stop_add} | {dbg}"
        self.pub_debug.publish(msg)

        if good:
            self.last_good_time = time.monotonic()
            self.last_rainy = rainy
            self.last_speed = speed
            self.last_stop_add = stop_add

    def _fetch_open_meteo(self) -> Tuple[bool, str, bool]:
        lat = float(self.get_parameter("latitude").value)
        lon = float(self.get_parameter("longitude").value)
        timeout = float(self.get_parameter("http_timeout_s").value)

        rain_thr = float(self.get_parameter("rain_mm_threshold").value)
        vis_thr = float(self.get_parameter("visibility_m_threshold").value)
        wind_thr = float(self.get_parameter("wind_ms_threshold").value)

        # Open-Meteo supports "current" variables such as rain, wind_speed_10m, visibility. :contentReference[oaicite:3]{index=3}
        url = (
            "https://api.open-meteo.com/v1/forecast"
            f"?latitude={lat}&longitude={lon}"
            "&current=rain,visibility,wind_speed_10m"
            "&timezone=auto"
        )

        try:
            with urllib.request.urlopen(url, timeout=timeout) as resp:
                data = json.loads(resp.read().decode("utf-8"))

            cur = data.get("current", {})
            rain = cur.get("rain", None)  # mm/h
            vis = cur.get("visibility", None)  # meters
            wind = cur.get("wind_speed_10m", None)  # usually km/h unless configured; treat threshold accordingly if you change units

            # Simple logic: "bad weather" if rain OR low visibility OR high wind
            rainy = False
            reasons = []

            if isinstance(rain, (int, float)) and float(rain) > rain_thr:
                rainy = True
                reasons.append(f"rain={rain}")

            if isinstance(vis, (int, float)) and float(vis) < vis_thr:
                rainy = True
                reasons.append(f"visibility={vis}")

            if isinstance(wind, (int, float)) and float(wind) > wind_thr:
                rainy = True
                reasons.append(f"wind_speed_10m={wind}")

            reason_txt = ", ".join(reasons) if reasons else "conditions within thresholds"
            return rainy, f"Open-Meteo OK ({reason_txt})", True

        except Exception as e:
            return False, f"Open-Meteo fetch FAILED: {e}", False


def main():
    rclpy.init()
    node = WeatherAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

