import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, NavSatFix

class T(Node):
    def __init__(self):
        super().__init__('test_vuelo_sensores')
        self.clouds = 0
        self.alts = []
        self.create_subscription(PointCloud2, '/vtol/depth_camera/points',
            self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/vtol/navsat',
            lambda m: self.alts.append(m.altitude), qos_profile_sensor_data)
    def on_cloud(self, m):
        if m.width * m.height > 0:
            self.clouds += 1

rclpy.init(); n = T(); t0 = time.time()
while time.time() - t0 < 40:          # observa 40 s del vuelo
    rclpy.spin_once(n, timeout_sec=0.2)

assert n.clouds > 0, "FALLA: la cámara no publicó nubes"
assert len(n.alts) > 0, "FALLA: el GPS no publicó"
subida = max(n.alts) - min(n.alts)
assert subida > 1.0, f"FALLA: el GPS no refleja el vuelo (variación {subida:.2f} m)"
print(f"OK: {n.clouds} nubes, GPS varió {subida:.1f} m durante el vuelo")
rclpy.shutdown()
