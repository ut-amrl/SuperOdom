import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class Check(Node):
    def __init__(self):
        super().__init__("check_livox_timestamp")
        self.sub = self.create_subscription(PointCloud2, "/livox/lidar", self.cb, 10)
        self.get_logger().info("Listening to /livox/lidar ... (will exit after 1 msg)")

    def cb(self, msg: PointCloud2):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        pts = point_cloud2.read_points(
            msg,
            field_names=("x","y","z","intensity","tag","line","timestamp"),
            skip_nans=False
        )

        # Handle either generator-like or numpy array return type
        if isinstance(pts, np.ndarray):
            if pts.size == 0:
                self.get_logger().error("No points in cloud")
                rclpy.shutdown()
                return
            first = pts[0]
            # If structured array, first is a tuple-like already
            x,y,z,intensity,tag,line,timestamp = first.tolist() if hasattr(first, "tolist") else first
        else:
            first = next(iter(pts), None)
            if first is None:
                self.get_logger().error("No points in cloud")
                rclpy.shutdown()
                return
            x,y,z,intensity,tag,line,timestamp = first

        self.get_logger().info(f"header.stamp (sec) = {stamp_sec:.9f}")
        self.get_logger().info(f"first point timestamp raw = {timestamp!r}  type={type(timestamp)}")
        self.get_logger().info(f"first point timestamp - header.stamp = {float(timestamp) - stamp_sec:.9f}")

        # sample min/max of timestamps (works for both)
        if isinstance(pts, np.ndarray):
            ts = pts["timestamp"] if pts.dtype.names and "timestamp" in pts.dtype.names else pts[:, -1]
            ts = ts.astype(np.float64)
            sample = ts[:2001]
        else:
            sample = []
            for i, p in enumerate(point_cloud2.read_points(msg, field_names=("timestamp",), skip_nans=False)):
                sample.append(float(p[0]))
                if i >= 2000:
                    break
            sample = np.array(sample, dtype=np.float64)

        self.get_logger().info(f"sample timestamp min={sample.min():.9f} max={sample.max():.9f}")
        self.get_logger().info(f"sample (ts - header.stamp) min={(sample.min()-stamp_sec):.9f} max={(sample.max()-stamp_sec):.9f}")

        rclpy.shutdown()

def main():
    rclpy.init()
    rclpy.spin(Check())

if __name__ == "__main__":
    main()
