import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ============================================================
# Shell wrapper for ``gz topic -e``
# Search order: 1) alongside this .py file, 2) installed package dir.
# ============================================================
_CANDIDATE_SH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "gz_topic_reader.sh",
)
if not os.path.isfile(_CANDIDATE_SH):
    # Fall back to package install directory (for colcon/debuild)
    _CANDIDATE_SH = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "gz_topic_reader.sh",
    )
if not os.path.isfile(_CANDIDATE_SH):
    # Last resort: look relative to current working directory
    _CANDIDATE_SH = os.path.join(os.getcwd(), "gz_topic_reader.sh")
_GZ_TOPIC_READER_SH = _CANDIDATE_SH


def _find_gz_scan_topics() -> list[str]:
    """Return active Gazebo scan/laser/lidar topics.

    NOTE: PX4's gazebo_lidar_plugin uses transport::Node::Advertise which does
    NOT register as a named Publisher visible via ``gz topic -i``.
    Therefore we return ALL lidar/laser-related topics without filtering.
    """
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=10,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return []
    all_topics = [t.strip() for t in result.stdout.splitlines() if t.strip()]
    # PX4 lidar topics: e.g.
    #   /gazebo/default/iris_rplidar/link/lidar      (from model://lidar)
    #   /gazebo/default/iris_rplidar/link/rplidar    (from model://rplidar)
    #   /gazebo/default/iris_rplidar/lidar/link/laser/scan (from ROS plugin, if available)
    candidates = [t for t in all_topics if any(kw in t.lower()
                  for kw in ("scan", "laser", "lidar", "rplidar"))]
    # No publisher check — PX4 plugins don't show up in gz topic -i publishers
    return sorted(candidates)


class GazeboScanMinDistance(Node):
    """Compute minimum obstacle distance from a Gazebo transport scan topic.

    Supports two Gazebo message formats:
      1) Range (single-point distance sensor, e.g. from libgazebo_lidar_plugin.so):
            current_distance: <float>
      2) LaserScan (360° lidar):
            ranges: [<float>, <float>, ...]

    The subscriber uses ``gz topic -e`` (piped subprocess) to read live data.
    """

    def __init__(self) -> None:
        super().__init__("gz_scan_min_distance")

        self.declare_parameter(
            "gz_scan_topic",
            "/gazebo/default/iris_rplidar/link/rplidar",
        )
        self.declare_parameter("distance_topic", "/perception/min_obstacle_distance")
        self.declare_parameter("no_obstacle_distance_m", 20.0)

        self._gz_scan_topic = self.get_parameter("gz_scan_topic").value
        distance_topic = self.get_parameter("distance_topic").value
        self._no_obstacle_distance_m = float(
            self.get_parameter("no_obstacle_distance_m").value
        )

        self._publisher = self.create_publisher(Float32, distance_topic, 10)

        # ---- Patterns for message parsing ----
        self._ranges_start_pattern = re.compile(r"\branges:\s*\[(.*?)?\]?\s*$")
        self._float_pattern = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")

        self._current_distance_pattern = re.compile(
            r"current_distance:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
        )

        self._log_buffer: list[str] = []
        self._max_log_lines = 80
        self._stderr_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "gz_topic_reader_stderr.log",
        )

        # ---- Process and reader state ----
        self._process: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None
        self._got_data = False
        self._shutdown_event = threading.Event()
        self._process_lock = threading.Lock()
        self._retry_timer: threading.Timer | None = None

        # ---- Start subscriber process ----
        proc_env = self._clean_env()
        self._start_process(proc_env)

        self._reader = threading.Thread(target=self._read_gz_topic, daemon=True)
        self._reader.start()

        # ---- Fallback timer: if no data within 15 seconds, try discovery ----
        self._no_data_timer = self.create_timer(15.0, self._check_got_data)

        # ---- Process restart timer (fast poll since -n 1 exits after each msg) ----
        self._health_timer = self.create_timer(1.0, self._check_process_health)

        self.get_logger().info(
            f"Reading Gazebo scan topic: {self._gz_scan_topic} -> {distance_topic}"
        )
        self.get_logger().info("Waiting for first message...")
        self.get_logger().info(
            f"Subprocess stderr -> {self._stderr_path}      "
            f"(check there if no data arrives)"
        )

    # ---------------------------------------------------------------
    @staticmethod
    def _clean_env() -> dict[str, str]:
        # Strip ALL ROS2 and Python paths that could conflict with Gazebo protobuf.
        # The gz_topic_reader.sh wrapper does env -i (full clean), but this gives a
        # fallback if the shell script fails for any reason.
        env = {}
        keep_keys = [
            "PATH", "HOME", "USER", "DISPLAY", "XAUTHORITY",
            "GAZEBO_MASTER_URI", "GAZEBO_MODEL_PATH", "GAZEBO_PLUGIN_PATH",
            "GAZEBO_RESOURCE_PATH", "GAZEBO_MODEL_DATABASE_URI",
        ]
        for k in keep_keys:
            if k in os.environ:
                env[k] = os.environ[k]
        # Minimal PATH with gz binary
        env["PATH"] = "/usr/local/bin:/usr/bin:/bin"
        # Minimal LD path (Gazebo plugins only)
        env["LD_LIBRARY_PATH"] = "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu"
        # Ensure no ROS2 variables leak
        for k in list(os.environ.keys()):
            if "ROS" in k.upper():
                env.pop(k, None)
        return env

    # ---------------------------------------------------------------
    def _start_process(self, proc_env: dict[str, str]) -> None:
        with self._process_lock:
            if self._process is not None and self._process.poll() is None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._process.kill()

            self._wrapper_cmd = [
                _GZ_TOPIC_READER_SH,
                self._gz_scan_topic,
            ]

            with open(self._stderr_path, "w") as stderr_f:
                stderr_f.write(f"=== gz_topic_reader.sh {self._gz_scan_topic} ===\n")

            self._process = subprocess.Popen(
                self._wrapper_cmd,
                stdout=subprocess.PIPE,
                stderr=open(self._stderr_path, "a"),
                text=True,
                bufsize=1,
                env=proc_env,
            )

    # ---------------------------------------------------------------
    def _terminate_process(self) -> None:
        with self._process_lock:
            if self._process is not None and self._process.poll() is None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._process.kill()

    # ---------------------------------------------------------------
    def _publish_distance(self, distance: float) -> None:
        msg = Float32()
        msg.data = distance
        self._publisher.publish(msg)

    # ---------------------------------------------------------------
    def _check_got_data(self) -> None:
        """Called by ROS2 timer after 15 seconds; if no data, try discovery."""
        if self._got_data:
            self._no_data_timer.cancel()
            return

        self.get_logger().warning(
            "No data received yet. Probing for active Gazebo scan topics..."
        )
        active = _find_gz_scan_topics()
        if not active:
            self.get_logger().warning(
                "No active scan/laser/lidar topics found. "
                "Retrying in 30 seconds..."
            )
            self._schedule_retry()
            return

        self.get_logger().info(f"Active topics found: {active}")

        # If the configured topic is in the list but just slow, wait more
        if self._gz_scan_topic in active:
            self.get_logger().info(
                f"Configured topic {self._gz_scan_topic} IS active, "
                "just waiting for first data..."
            )
            self._schedule_retry()
            return

        # Try the first active topic
        new_topic = active[0]
        self.get_logger().warning(
            f"Switching from {self._gz_scan_topic} to auto-discovered "
            f"{new_topic}"
        )
        self._gz_scan_topic = new_topic
        self._terminate_process()
        self._start_process(self._clean_env())
        self._reader = threading.Thread(target=self._read_gz_topic, daemon=True)
        self._reader.start()
        self._schedule_retry()

    # ---------------------------------------------------------------
    def _schedule_retry(self) -> None:
        """Schedule another check in 30 seconds."""
        if self._shutdown_event.is_set():
            return
        if self._retry_timer is not None:
            self._retry_timer.cancel()
        self._retry_timer = threading.Timer(30.0, self._retry_discovery)
        self._retry_timer.daemon = True
        self._retry_timer.start()

    # ---------------------------------------------------------------
    def _retry_discovery(self) -> None:
        """Check again later if we still haven't received data."""
        if self._got_data or self._shutdown_event.is_set():
            return
        active = _find_gz_scan_topics()
        if not active:
            self.get_logger().warning(
                "Still no active scan topics. Will retry in 30 seconds..."
            )
            self._schedule_retry()
            return

        new_topic = active[0]
        if new_topic != self._gz_scan_topic:
            self.get_logger().warning(
                f"Switching to {new_topic} (was {self._gz_scan_topic})"
            )
            self._gz_scan_topic = new_topic
            self._terminate_process()
            self._start_process(self._clean_env())
            self._reader = threading.Thread(target=self._read_gz_topic, daemon=True)
            self._reader.start()

        self._schedule_retry()

    # ---------------------------------------------------------------
    def _check_process_health(self) -> None:
        """Periodic health check: restart process if it exited prematurely."""
        if self._shutdown_event.is_set():
            return
        with self._process_lock:
            if self._process is None:
                return
            rc = self._process.poll()
        if rc is not None:
            # NOTE: PX4's gazebo_lidar_plugin does NOT register a named Publisher
            # visible via ``gz topic -i``, so we skip any publisher check and simply
            # restart the reader subprocess.
            if self._got_data:
                self.get_logger().debug(
                    f"gz topic -e exited (code {rc}) after data received, "
                    "restarting reader..."
                )
            else:
                self.get_logger().warning(
                    f"gz topic -e exited with code {rc}, no data yet. "
                    "Restarting reader..."
                )
            self._start_process(self._clean_env())
            self._reader = threading.Thread(target=self._read_gz_topic, daemon=True)
            self._reader.start()

    # ---------------------------------------------------------------
    def _read_gz_topic(self) -> None:
        if self._process is None or self._process.stdout is None:
            return

        collecting_ranges = False
        received_first = False
        lines_since_last_publish = 0
        scan_distances: list[float] = []
        current_scan_seen = False

        def publish_scan_min(scan_seen: bool = False) -> None:
            nonlocal received_first, lines_since_last_publish, scan_distances

            if scan_distances:
                distance = min(scan_distances)
            elif scan_seen:
                distance = self._no_obstacle_distance_m
            else:
                return

            scan_distances = []
            self._publish_distance(distance)
            lines_since_last_publish = 0

            if not received_first:
                received_first = True
                self._got_data = True
                self.get_logger().info(
                    f"Received first Gazebo LaserScan range: "
                    f"min distance = {distance:.2f} m"
                )

        try:
            for line in self._process.stdout:
                stripped = line.rstrip("\n")
                lines_since_last_publish += 1

                if len(self._log_buffer) < self._max_log_lines:
                    self._log_buffer.append(stripped)

                # Gazebo Classic prints LaserScanStamped messages as a stream of
                # repeated ``time { ... } scan { ... ranges: ... }`` blocks.
                # Publish the previous scan's minimum when a new message starts.
                if stripped.startswith("time {"):
                    publish_scan_min(scan_seen=current_scan_seen)
                    current_scan_seen = False
                    collecting_ranges = False
                    continue

                if stripped.startswith("scan {"):
                    current_scan_seen = True

                # ---- Format 1: Range (current_distance) ----
                m = self._current_distance_pattern.search(stripped)
                if m is not None:
                    try:
                        distance = float(m.group(1))
                    except (ValueError, TypeError):
                        continue
                    if distance > 0.0 and not (distance > 1e8):
                        self._publish_distance(distance)
                        lines_since_last_publish = 0
                        if not received_first:
                            received_first = True
                            self._got_data = True
                            self.get_logger().info(
                                f"Received first Gazebo Range (current_distance): "
                                f"{distance:.2f} m"
                            )
                    continue

                # ---- Format 2: LaserScan (ranges) ----
                if "ranges:" in stripped:
                    current_scan_seen = True
                    collecting_ranges = True
                    if "[" in stripped:
                        payload = self._ranges_start_pattern.search(stripped)
                        if payload is None:
                            continue
                        candidates = self._float_pattern.findall(payload.group(1))
                    else:
                        candidates = self._float_pattern.findall(
                            stripped.split("ranges:", 1)[1])
                elif collecting_ranges:
                    candidates = self._float_pattern.findall(stripped)
                    if not candidates:
                        if stripped.lstrip().startswith(("]", "[")):
                            continue
                        collecting_ranges = False
                        continue
                else:
                    continue

                for c in candidates:
                    try:
                        d = float(c)
                    except (ValueError, TypeError):
                        continue
                    if d > 0.0:
                        scan_distances.append(d)

                if lines_since_last_publish > 100:
                    if scan_distances:
                        self.get_logger().debug(
                            f"Current scan min distance: {min(scan_distances):.2f} m",
                            throttle_duration_sec=5.0,
                        )

            publish_scan_min(scan_seen=True)
        except Exception:
            # Subprocess pipe broken or process killed - normal during shutdown/restart
            pass

        # ---------------------------------------------------------------
        # Subprocess ended
        # ---------------------------------------------------------------
        if not received_first and not self._shutdown_event.is_set():
            # Read stderr for clues
            stderr_text = ""
            try:
                with open(self._stderr_path) as f:
                    stderr_text = f.read()
            except Exception:
                stderr_text = "(could not read stderr file)"

            self.get_logger().error(
                "NO data received from Gazebo scan topic!\n"
                f"  Topic: {self._gz_scan_topic}\n"
                f"  Subprocess: {' '.join(self._wrapper_cmd)}\n"
                f"  Return code: {self._process.returncode if self._process else -1}\n"
                "  Possible causes:\n"
                "    1. Topic name is wrong – "
                "use `gz topic -l | grep -E 'scan|laser|lidar'` to discover\n"
                "    2. Lidar plugin (libgazebo_lidar_plugin.so) not loaded\n"
                "    3. Sensor not publishing data yet\n"
                "    4. gz cannot reach gzserver (GAZEBO_MASTER_URI wrong?)\n"
                f"  First {len(self._log_buffer)} lines of stdout:\n"
                + "\n".join(f"    |{l}" for l in self._log_buffer)
                + "\n"
                "  Stderr from wrapper script:\n"
                + "".join(f"    |{l}" for l in stderr_text.splitlines(True))
            )

    # ---------------------------------------------------------------
    def destroy_node(self) -> bool:
        self._shutdown_event.set()
        if self._retry_timer is not None:
            self._retry_timer.cancel()
        self._terminate_process()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = GazeboScanMinDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
