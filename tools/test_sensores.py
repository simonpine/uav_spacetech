import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, NavSatFix

class T(Node):
    def __init__(self):
        super().__init__('test_sensores')
        self.cloud = self.gps = None
        self.create_subscription(PointCloud2, '/vtol/depth_camera/points',
            lambda m: setattr(self, 'cloud', m), qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/vtol/navsat',
            lambda m: setattr(self, 'gps', m), qos_profile_sensor_data)

rclpy.init(); n = T(); t0 = time.time()
while time.time() - t0 < 20 and (n.cloud is None or n.gps is None):
    rclpy.spin_once(n, timeout_sec=0.5)

assert n.cloud is not None, "FALLA: no llega la nube de puntos"
assert n.cloud.width * n.cloud.height > 0, "FALLA: nube vacía"
assert n.cloud.header.frame_id.endswith("depth_camera_link"), f"frame inesperado: {n.cloud.header.frame_id}"
assert n.gps is not None, "FALLA: no llega el GPS"
assert abs(n.gps.latitude - 4.7110) < 0.01, f"FALLA: latitud inesperada {n.gps.latitude}"
print(f"OK: nube {n.cloud.width}x{n.cloud.height}, GPS {n.gps.latitude:.5f}, {n.gps.longitude:.5f}")
rclpy.shutdown()