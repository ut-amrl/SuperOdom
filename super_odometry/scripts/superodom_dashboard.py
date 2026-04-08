#!/usr/bin/env python3
import math
import os
import resource
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String
from super_odometry_msgs.msg import OptimizationStats
from tf2_ros import Buffer, TransformException, TransformListener


BOX_WIDTH = 76


def clamp_text(text: str, width: int) -> str:
    if len(text) <= width:
        return text
    return text[: max(0, width - 3)] + "..."


def format_bool(value):
    if value is None:
        return "WAIT"
    return "OK" if value else "BAD"


def fmt_float(value, digits=3):
    if value is None:
        return "-"
    return f"{value:.{digits}f}"


def fmt_age(value):
    if value is None or value < 0.0 or not math.isfinite(value):
        return "-"
    return f"{value:.2f}s"


def quat_to_rpy(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def make_border(char="-"):
    return "+" + char * (BOX_WIDTH - 2) + "+"


def make_row(left="", right=""):
    inner = BOX_WIDTH - 4
    if right:
        remaining = max(1, inner - len(right) - 1)
        left = clamp_text(left, remaining)
        content = f"{left}{' ' * (inner - len(left) - len(right))}{right}"
    else:
        content = clamp_text(left, inner)
        content = content + " " * (inner - len(content))
    return f"| {content} |"


class RateTracker:
    def __init__(self, maxlen=200):
        self.times = deque(maxlen=maxlen)
        self.last_msg_time = None
        self.last_recv_time = None

    def tick(self, now_wall, msg_stamp=None):
        self.times.append(now_wall)
        self.last_recv_time = now_wall
        self.last_msg_time = msg_stamp

    def hz(self):
        if len(self.times) < 2:
            return None
        dt = self.times[-1] - self.times[0]
        if dt <= 0.0:
            return None
        return (len(self.times) - 1) / dt

    def age(self, now_ros):
        if self.last_msg_time is None:
            return None
        try:
            age = (now_ros - self.last_msg_time).nanoseconds / 1e9
        except Exception:
            return None
        if age < 0.0 or age > 1e6:
            return None
        return age

    def recent(self, now_wall, timeout):
        if self.last_recv_time is None:
            return False
        return (now_wall - self.last_recv_time) <= timeout


class StatsTracker:
    def __init__(self):
        self.count = 0
        self.sum_ms = 0.0
        self.max_ms = 0.0

    def update(self, elapsed_seconds):
        if elapsed_seconds is None:
            return
        value_ms = max(0.0, elapsed_seconds * 1000.0)
        self.count += 1
        self.sum_ms += value_ms
        self.max_ms = max(self.max_ms, value_ms)

    @property
    def average_ms(self):
        if self.count == 0:
            return None
        return self.sum_ms / self.count


class CpuTracker:
    def __init__(self):
        self.prev_wall = None
        self.prev_cpu = None
        self.current_cores = None
        self.samples = 0
        self.sum_cores = 0.0
        self.max_cores = 0.0

    def update(self, now_wall):
        now_cpu = time.process_time()
        if self.prev_wall is not None and self.prev_cpu is not None:
            wall_dt = now_wall - self.prev_wall
            cpu_dt = now_cpu - self.prev_cpu
            if wall_dt > 1e-6 and cpu_dt >= 0.0:
                self.current_cores = cpu_dt / wall_dt
                self.samples += 1
                self.sum_cores += self.current_cores
                self.max_cores = max(self.max_cores, self.current_cores)
        self.prev_wall = now_wall
        self.prev_cpu = now_cpu

    @property
    def average_cores(self):
        if self.samples == 0:
            return None
        return self.sum_cores / self.samples


def current_rss_mb():
    try:
        with open("/proc/self/status", "r", encoding="utf-8") as status_file:
            for line in status_file:
                if line.startswith("VmRSS:"):
                    kb = float(line.split()[1])
                    return kb / 1024.0
    except OSError:
        return None
    return None


class SuperOdomDashboard(Node):
    def __init__(self):
        super().__init__("superodom_dashboard")

        self.declare_parameter("PROJECT_NAME", "super_odometry")
        self.declare_parameter("imu_topic", "/vectornav/imu")
        self.declare_parameter("laser_topic", "/livox/lidar")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("lidar_frame", "lidar_link")
        self.declare_parameter("imu_frame_rectified", "imu_link_rect")
        self.declare_parameter("lidar_frame_rectified", "lidar_link_rect")
        self.declare_parameter("dashboard_refresh_hz", 5.0)
        self.declare_parameter("dashboard_timeout_sec", 1.5)

        self.project_name = self.get_parameter("PROJECT_NAME").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.laser_topic = self.get_parameter("laser_topic").value
        self.world_frame = self.get_parameter("world_frame").value
        self.base_link_frame = self.get_parameter("base_link_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.imu_frame_rectified = self.get_parameter("imu_frame_rectified").value
        self.lidar_frame_rectified = self.get_parameter("lidar_frame_rectified").value
        self.refresh_hz = float(self.get_parameter("dashboard_refresh_hz").value)
        self.timeout_sec = float(self.get_parameter("dashboard_timeout_sec").value)
        if self.refresh_hz <= 0.0:
            self.refresh_hz = 5.0

        self.start_wall = time.monotonic()
        self.display_stream = self._open_display_stream()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.imu_rate = RateTracker()
        self.lidar_rate = RateTracker()
        self.state_rate = RateTracker()
        self.map_rate = RateTracker()
        self.stats_rate = RateTracker()
        self.elev_rate = RateTracker()
        self.opt_time_stats = StatsTracker()
        self.cpu_tracker = CpuTracker()

        self.health_status = None
        self.prediction_source = None
        self.last_state = None
        self.last_map = None
        self.last_twist = None
        self.last_stats = None

        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.create_subscription(PointCloud2, self.laser_topic, self.lidar_cb, 10)
        self.create_subscription(Odometry, f"{self.project_name}/state_estimation2", self.state_cb, 10)
        self.create_subscription(Odometry, f"{self.project_name}/laser_odometry", self.map_cb, 10)
        self.create_subscription(PoseStamped, f"{self.project_name}/pose", self.pose_cb, 10)
        self.create_subscription(TwistStamped, f"{self.project_name}/twist", self.twist_cb, 10)
        self.create_subscription(Bool, f"{self.project_name}/state_estimation_health", self.health_cb, 10)
        self.create_subscription(String, f"{self.project_name}/prediction_source", self.prediction_cb, 10)
        self.create_subscription(OptimizationStats, f"{self.project_name}/super_odometry_stats", self.stats_cb, 10)
        self.create_subscription(GridMap, f"{self.project_name}/elevation_map", self.elev_cb, 2)

        self.create_timer(1.0 / self.refresh_hz, self.render)

    def _open_display_stream(self):
        try:
            return open("/dev/tty", "w", buffering=1, encoding="utf-8", errors="replace")
        except OSError:
            return sys.stdout

    def _write_screen(self, text):
        try:
            self.display_stream.write(text)
            self.display_stream.flush()
        except OSError:
            if self.display_stream is not sys.stdout:
                self.display_stream = sys.stdout
                self.display_stream.write(text)
                self.display_stream.flush()

    def imu_cb(self, msg):
        self.imu_rate.tick(time.monotonic(), msg.header.stamp)

    def lidar_cb(self, msg):
        self.lidar_rate.tick(time.monotonic(), msg.header.stamp)

    def state_cb(self, msg):
        self.state_rate.tick(time.monotonic(), msg.header.stamp)
        self.last_state = msg

    def map_cb(self, msg):
        self.map_rate.tick(time.monotonic(), msg.header.stamp)
        self.last_map = msg

    def pose_cb(self, _msg):
        return

    def twist_cb(self, msg):
        self.last_twist = msg

    def health_cb(self, msg):
        self.health_status = bool(msg.data)

    def prediction_cb(self, msg):
        self.prediction_source = msg.data

    def stats_cb(self, msg):
        self.stats_rate.tick(time.monotonic(), msg.header.stamp)
        self.last_stats = msg
        self.opt_time_stats.update(msg.time_elapsed)

    def elev_cb(self, msg):
        self.elev_rate.tick(time.monotonic(), msg.header.stamp)

    def tf_ok(self, target, source):
        try:
            self.tf_buffer.lookup_transform(target, source, rclpy.time.Time(), timeout=Duration(seconds=0.01))
            return True
        except TransformException:
            return False

    def pose_components(self, odom_msg):
        if odom_msg is None:
            return None
        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        return p.x, p.y, p.z, math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def twist_components(self, twist_msg):
        if twist_msg is None:
            return None
        t = twist_msg.twist
        return t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z

    def distance_to_origin(self, odom_msg):
        if odom_msg is None:
            return None
        p = odom_msg.pose.pose.position
        return math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)

    def render(self):
        now_wall = time.monotonic()
        now_ros = self.get_clock().now()
        elapsed = now_wall - self.start_wall
        self.cpu_tracker.update(now_wall)

        imu_ok = self.imu_rate.recent(now_wall, self.timeout_sec)
        lidar_ok = self.lidar_rate.recent(now_wall, self.timeout_sec)
        state_ok = self.state_rate.recent(now_wall, self.timeout_sec)
        map_ok = self.map_rate.recent(now_wall, self.timeout_sec)
        stats_ok = self.stats_rate.recent(now_wall, self.timeout_sec)

        state_pose = self.pose_components(self.last_state)
        twist_vals = self.twist_components(self.last_twist)
        stats = self.last_stats
        load1, _, _ = os.getloadavg()
        rss_mb = current_rss_mb()
        max_rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0

        lines = [
            make_border("-"),
            make_row("SuperOdom Live Dashboard", f"Elapsed Time: {elapsed:6.2f} s"),
            make_border("-"),
            make_row(time.strftime("%a %b %d %H:%M:%S %Y"), f"Project: {self.project_name}"),
            make_row(
                f"Sensor Rates: LiDAR @ {fmt_float(self.lidar_rate.hz(), 2)} Hz, IMU @ {fmt_float(self.imu_rate.hz(), 2)} Hz",
                f"Health: {format_bool(self.health_status)}",
            ),
            make_border("="),
        ]

        if state_pose is None:
            lines.append(make_row("Position      {map} [xyz] :: waiting for state_estimation2"))
            lines.append(make_row("Orientation   {map} [rpy] :: waiting for state_estimation2"))
        else:
            lines.append(make_row(
                f"Position      {{map}} [xyz] :: {state_pose[0]: .3f} {state_pose[1]: .3f} {state_pose[2]: .3f}"
            ))
            lines.append(make_row(
                f"Orientation   {{map}} [rpy] :: {state_pose[3]: .2f} {state_pose[4]: .2f} {state_pose[5]: .2f} deg"
            ))

        if twist_vals is None:
            lines.append(make_row("Lin Velocity  {B}   [xyz] :: waiting for twist"))
            lines.append(make_row("Ang Velocity  {B}   [xyz] :: waiting for twist"))
        else:
            lines.append(make_row(
                f"Lin Velocity  {{B}}   [xyz] :: {twist_vals[0]: .3f} {twist_vals[1]: .3f} {twist_vals[2]: .3f}"
            ))
            lines.append(make_row(
                f"Ang Velocity  {{B}}   [xyz] :: {twist_vals[3]: .3f} {twist_vals[4]: .3f} {twist_vals[5]: .3f}"
            ))

        if stats is None:
            lines.append(make_row("Registration          :: waiting for optimization stats"))
            lines.append(make_row("Prediction Source     :: " + (self.prediction_source or "-")))
        else:
            lines.append(make_row(
                f"Registration          :: iter {stats.n_iterations}, plane matches {stats.plane_match_success}, latency {stats.latency:.3f}s"
            ))
            lines.append(make_row("Prediction Source     :: " + (self.prediction_source or "-")))

        lines.append(make_row(""))
        lines.append(make_row(
            f"Distance Traveled  :: {fmt_float(stats.total_translation if stats else None, 3)} meters"
        ))
        lines.append(make_row(
            f"Distance to Origin :: {fmt_float(self.distance_to_origin(self.last_state), 3)} meters"
        ))
        lines.append(make_row(
            f"State Update Rate  :: {fmt_float(self.state_rate.hz(), 2)} Hz", 
            f"LIO Rate: {fmt_float(self.map_rate.hz(), 2)} Hz"
        ))

        if stats is None:
            lines.append(make_row("Optimization Time  :: - ms"))
            lines.append(make_row("Cores Utilized     :: - cores"))
            lines.append(make_row("CPU Load           :: -"))
            lines.append(make_row("RAM Allocation     :: - MB"))
            lines.append(make_row("Uncertainty        :: waiting for optimization stats"))
        else:
            lines.append(make_row(
                f"Optimization Time  :: {stats.time_elapsed * 1000.0:.2f} ms  // Avg: {fmt_float(self.opt_time_stats.average_ms, 2)} / Max: {self.opt_time_stats.max_ms:.2f}"
            ))
            lines.append(make_row(
                f"Cores Utilized     :: {fmt_float(self.cpu_tracker.current_cores, 2)} cores  // Avg: {fmt_float(self.cpu_tracker.average_cores, 2)} / Max: {fmt_float(self.cpu_tracker.max_cores, 2)}"
            ))
            lines.append(make_row(
                f"CPU Load           :: {load1:.2f} (1m avg)",
                f"Latency: {stats.latency:.2f} s"
            ))
            lines.append(make_row(
                f"RAM Allocation     :: {fmt_float(rss_mb, 2)} MB  // Max: {fmt_float(max_rss_mb, 2)} MB"
            ))
            lines.append(make_row(
                f"Uncertainty        :: xyz [{stats.uncertainty_x:.3f} {stats.uncertainty_y:.3f} {stats.uncertainty_z:.3f}]"
            ))
            lines.append(make_row(
                f"                       rpy [{math.degrees(stats.uncertainty_roll):.2f} {math.degrees(stats.uncertainty_pitch):.2f} {math.degrees(stats.uncertainty_yaw):.2f}] deg"
            ))

        lines.append(make_border("="))
        elev_ok = self.elev_rate.recent(now_wall, self.timeout_sec)
        lines.append(make_row(
            f"Inputs   :: IMU {format_bool(imu_ok)} @ {fmt_float(self.imu_rate.hz(), 1)} Hz, age {fmt_age(self.imu_rate.age(now_ros))}",
            f"LiDAR {format_bool(lidar_ok)} @ {fmt_float(self.lidar_rate.hz(), 1)} Hz",
        ))
        lines.append(make_row(
            f"Outputs  :: State {format_bool(state_ok)} @ {fmt_float(self.state_rate.hz(), 1)} Hz",
            f"LIO {format_bool(map_ok)} @ {fmt_float(self.map_rate.hz(), 1)} Hz",
        ))
        lines.append(make_row(
            f"Elev Map :: {format_bool(elev_ok)} @ {fmt_float(self.elev_rate.hz(), 1)} Hz",
        ))
        lines.append(make_row(
            f"TF       :: map->base {format_bool(self.tf_ok(self.world_frame, self.base_link_frame))}, base->lidar {format_bool(self.tf_ok(self.base_link_frame, self.lidar_frame))}, base->imu {format_bool(self.tf_ok(self.base_link_frame, self.imu_frame))}"
        ))
        lines.append(make_row(
            f"Rectified:: lidar->rect {format_bool(self.tf_ok(self.lidar_frame, self.lidar_frame_rectified))}, imu->rect {format_bool(self.tf_ok(self.imu_frame, self.imu_frame_rectified))}"
        ))
        lines.append(make_border("-"))

        self._write_screen("\x1b[2J\x1b[H")
        self._write_screen("\n".join(lines))
        self._write_screen("\n")


def main(args=None):
    rclpy.init(args=args)
    node = SuperOdomDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "display_stream", None) not in (None, sys.stdout):
            try:
                node.display_stream.close()
            except OSError:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
