import socket
import time

import msgpack
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class RouterBridge:
    def __init__(self, sock_path="/var/run/arduino-router.sock"):
        self._sock_path = sock_path
        self._sock = None
        self._next_id = 1
        self._connect()

    def _connect(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        sock.connect(self._sock_path)
        self._sock = sock

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def call(self, method, *args):
        if self._sock is None:
            raise RuntimeError("not connected")
        rpc_id = self._next_id
        self._next_id += 1
        req = msgpack.packb([0, rpc_id, method, list(args)])
        try:
            self._sock.sendall(req)
        except (BrokenPipeError, ConnectionError, OSError):
            self.close()
            self._connect()
            self._sock.sendall(req)

        self._sock.settimeout(1.0)
        resp = b""
        try:
            while True:
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
                resp += chunk
                try:
                    msgpack.unpackb(resp, raw=False,
                                    max_array_len=16, max_map_len=16,
                                    max_str_len=4096)
                    break
                except (msgpack.exceptions.UnpackValueError,
                        msgpack.exceptions.OutOfData):
                    continue
                except (ValueError, msgpack.exceptions.ExtraData,
                        msgpack.exceptions.FormatError):
                    break
        except socket.timeout:
            if not resp:
                raise

        if not resp:
            raise RuntimeError("empty response from router")

        result = msgpack.unpackb(resp, raw=False,
                                 max_array_len=16, max_map_len=16,
                                 max_str_len=4096)
        if len(result) < 4:
            raise RuntimeError(f"invalid RPC response: {result}")
        if result[2] is not None:
            raise RuntimeError(f"RPC error: {result[2]}")
        return result[3]


class MockBridge:
    def call(self, fn, *args):
        if fn == "read_imu":
            return "0.0 0.0 0.0 0.0 0.0 9.81"
        return "ok"

    def close(self):
        pass


class HardwareBridgeNode(Node):
    def __init__(self):
        super().__init__("hardware_bridge")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("imu_frame_id", "imu_link"),
                ("imu_rate", 30.0),
                ("servo_count", 12),
                ("servo_freq", 50),
                ("servo_angle_min", -1.57),
                ("servo_angle_max", 1.57),
                ("servo_pulse_min", 500),
                ("servo_pulse_max", 2500),
                ("router_socket", "/var/run/arduino-router.sock"),
                ("mock", False),
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
        self._mock = self.get_parameter("mock").value

        self._bridge = None
        self._init_bridge()

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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
            f" | {self._servo_count} servos"
            f" | {'MOCK' if self._mock else 'HW'}"
        )

    def _init_bridge(self):
        if self._mock:
            self.get_logger().warning("Mock mode enabled — no hardware")
            self._bridge = MockBridge()
            return

        try:
            sock_path = self.get_parameter("router_socket").value
            self._bridge = RouterBridge(sock_path)
            self.get_logger().info(f"Connected to router at {sock_path}")
        except Exception as e:
            self.get_logger().error(f"RouterBridge init failed: {e}")
            self.get_logger().warning("Falling back to mock mode")
            self._bridge = MockBridge()

    def _publish_imu(self):
        if self._bridge is None:
            return
        try:
            t0 = time.monotonic()
            raw = self._bridge.call("read_imu")
            dt = (time.monotonic() - t0) * 1000
        except Exception as e:
            self.get_logger().warn(
                f"read_imu failed: {e}", throttle_duration_sec=5.0
            )
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
            self.get_logger().warn(
                f"set_servos failed: {e}", throttle_duration_sec=2.0
            )

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


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._bridge is not None:
            node._bridge.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
