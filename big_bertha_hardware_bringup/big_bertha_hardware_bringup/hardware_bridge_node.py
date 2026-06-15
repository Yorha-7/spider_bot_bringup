import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

try:
    from arduino_router_bridge import Bridge
except ImportError:
    Bridge = None


class HardwareBridgeNode(Node):
    def __init__(self):
        super().__init__("hardware_bridge")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("imu_frame_id", "imu_link"),
                ("imu_rate", 200.0),
                ("servo_count", 12),
                ("servo_freq", 50),
                ("servo_angle_min", -1.57),
                ("servo_angle_max", 1.57),
                ("servo_pulse_min", 500),
                ("servo_pulse_max", 2500),
            ],
        )

        self._imu_frame = self.get_parameter("imu_frame_id").value
        self._imu_rate = self.get_parameter("imu_rate").value

        self._servo_count = self.get_parameter("servo_count").value
        self._servo_freq = self.get_parameter("servo_freq").value
        self._angle_min = self.get_parameter("servo_angle_min").value
        self._angle_max = self.get_parameter("servo_angle_max").value
        self._pulse_min = self.get_parameter("servo_pulse_min").value
        self._pulse_max = self.get_parameter("servo_pulse_max").value

        self._bridge = None
        self._init_bridge()

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._imu_pub = self.create_publisher(Imu, "/imu", imu_qos)
        self._cmd_sub = self.create_subscription(
            Float64MultiArray,
            "/position_controller/commands",
            self._cmd_callback,
            1,
        )
        self._ping_srv = self.create_service(
            Trigger, "/hardware/ping", self._ping_callback
        )

        period = 1.0 / self._imu_rate
        self._imu_timer = self.create_timer(period, self._publish_imu)

        self.get_logger().info(
            f"Started | IMU @ {self._imu_rate} Hz on /{self._imu_frame}"
            f" | {self._servo_count} servos via Bridge"
        )

    def _init_bridge(self):
        if Bridge is None:
            self.get_logger().warning(
                "arduino_router_bridge not found — running in mock mode"
            )
            self._bridge = MockBridge()
            return
        try:
            self._bridge = Bridge()
            self.get_logger().info("Arduino Router Bridge connected")
        except Exception as e:
            self.get_logger().error(f"Bridge init failed: {e}")
            self._bridge = MockBridge()

    def _publish_imu(self):
        if self._bridge is None:
            return
        try:
            raw = self._bridge.call("read_imu")
        except Exception as e:
            self.get_logger().warn(f"read_imu failed: {e}", throttle_duration=5.0)
            return

        parts = raw.strip().split()
        if len(parts) < 6:
            return

        gx = float(parts[0])
        gy = float(parts[1])
        gz = float(parts[2])
        ax = float(parts[3])
        ay = float(parts[4])
        az = float(parts[5])

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        self._imu_pub.publish(msg)

    def _cmd_callback(self, msg: Float64MultiArray):
        if self._bridge is None:
            return
        if len(msg.data) < self._servo_count:
            self.get_logger().warn(
                f"cmd has {len(msg.data)} values, expected {self._servo_count}"
            )
            return

        rads = msg.data[: self._servo_count]
        pulse_str = " ".join(str(self._rad_to_pulse(r)) for r in rads)

        try:
            self._bridge.call("set_servos", pulse_str)
        except Exception as e:
            self.get_logger().warn(f"set_servos failed: {e}", throttle_duration=2.0)

    def _rad_to_pulse(self, rad: float) -> int:
        t = (rad - self._angle_min) / (self._angle_max - self._angle_min)
        t = max(0.0, min(1.0, t))
        return int(self._pulse_min + t * (self._pulse_max - self._pulse_min))

    def _ping_callback(self, request, response):
        if self._bridge is None:
            response.success = False
            response.message = "bridge not available"
            return response
        try:
            result = self._bridge.call("ping")
            response.success = result.strip() == "ok"
            response.message = result.strip()
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


class MockBridge:
    def call(self, fn, *args):
        if fn == "read_imu":
            return "0.0 0.0 0.0 0.0 0.0 9.81"
        return "ok"


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
