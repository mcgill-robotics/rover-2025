#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

def meters_per_deg_lat():
    # Good enough for small movements
    return 110_574.0

def meters_per_deg_lon(lat_deg: float):
    # Scales with latitude
    return 111_320.0 * math.cos(math.radians(lat_deg))

class WobblyTurnPublisher(Node):
    """
    Publishes [depth, lat, lon] that trace a noisy circle around a center point.
    - angle increases each tick (simulated turning)
    - radius has small random wobble
    - center drifts slightly over time
    - heading jitter so it's not perfectly uniform
    """
    def __init__(self):
        super().__init__('wobbly_turn_publisher')
        self.pub = self.create_publisher(Float32MultiArray, 'roverGPSData', 10)

        # --- Base params you can tweak ---
        self.center_lat = 51.470500
        self.center_lon = -112.75175
        self.depth_mean = 3.0                 # meters
        self.depth_noise = 0.05               # ±m

        self.radius_m = 25.0                  # circle radius in meters
        self.radius_wobble_m = 1.5            # fast small wobble
        self.radius_drift_per_s = 0.003       # very slow radius change over time

        self.angular_speed_deg_s = 20.0       # how fast we "turn" (deg/s)
        self.heading_jitter_deg = 1.0         # small random heading jitter

        self.center_drift_m_per_s = 0.01      # center drifts a hair over time
        self.center_drift_heading_deg = random.uniform(0, 360)

        self.rate_hz = 5.0
        self.dt = 1.0 / self.rate_hz

        # --- State ---
        self.angle_deg = random.uniform(0, 360)
        self.t = 0.0

        self.get_logger().info("Wobbly turning GPS simulator started.")
        self.timer = self.create_timer(self.dt, self.on_tick)

    def on_tick(self):
        self.t += self.dt

        # Slowly drift the center so the path isn't perfectly centered forever
        drift_distance = self.center_drift_m_per_s * self.dt
        self.center_drift_heading_deg += random.uniform(-0.2, 0.2)  # meander a bit
        cx_off_m = drift_distance * math.cos(math.radians(self.center_drift_heading_deg))
        cy_off_m = drift_distance * math.sin(math.radians(self.center_drift_heading_deg))

        # Convert center drift meters -> degrees
        lat_m_per_deg = meters_per_deg_lat()
        lon_m_per_deg = meters_per_deg_lon(self.center_lat)
        self.center_lat += cy_off_m / lat_m_per_deg
        self.center_lon += cx_off_m / lon_m_per_deg

        # Radius slow drift + fast wobble
        self.radius_m += self.radius_drift_per_s * self.dt * (1 if random.random() < 0.5 else -1)
        wobble = self.radius_wobble_m * math.sin(2 * math.pi * 0.7 * self.t)  # 0.7 Hz wobble
        r = max(1.0, self.radius_m + wobble)  # keep > 1m

        # Heading advance + jitter
        jitter = random.uniform(-self.heading_jitter_deg, self.heading_jitter_deg)
        self.angle_deg = (self.angle_deg + self.angular_speed_deg_s * self.dt + jitter) % 360.0

        # Polar -> local meters
        x_m = r * math.cos(math.radians(self.angle_deg))  # east (+x)
        y_m = r * math.sin(math.radians(self.angle_deg))  # north (+y)

        # Local meters -> degrees around current center
        lat = self.center_lat + (y_m / lat_m_per_deg)
        lon = self.center_lon + (x_m / meters_per_deg_lon(self.center_lat))

        # Depth with a little noise
        depth = self.depth_mean + random.uniform(-self.depth_noise, self.depth_noise)

        # Publish
        msg = Float32MultiArray()
        msg.data = [float(depth), float(lat), float(lon)]
        self.pub.publish(msg)

        # Optional: log less often to not spam
        if int(self.t * self.rate_hz) % int(self.rate_hz) == 0:
            self.get_logger().info(f"Published: depth={depth:.2f}, lat={lat:.6f}, lon={lon:.6f}")

def main(args=None):
    rclpy.init(args=args)
    node = WobblyTurnPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
