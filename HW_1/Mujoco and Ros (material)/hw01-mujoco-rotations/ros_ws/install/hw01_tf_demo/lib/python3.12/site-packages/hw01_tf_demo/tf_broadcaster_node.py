"""
tf_broadcaster_node.py -- HW1 Part 2, Task 3.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def Rx(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


ELEMENTARY_ROTATIONS = {"x": Rx, "y": Ry, "z": Rz}

STEP_SEQUENCE = [
    ("z", np.deg2rad(90)),
    ("x", np.deg2rad(90)),
    ("y", np.deg2rad(60)),
]


def R_to_quat_xyzw(R):
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


class Hw01TfBroadcaster(Node):
    def __init__(self):
        super().__init__("hw01_tf_broadcaster")
        self.declare_parameter("compose_frame", "current")
        self.declare_parameter("step_period", 1.0)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.R_body = np.eye(3)
        self.step_index = 0

        step_period = self.get_parameter("step_period").value
        self.timer = self.create_timer(step_period, self.on_timer)
        self.get_logger().info(
            "hw01_tf_broadcaster started. Try: "
            "ros2 param set /hw01_tf_broadcaster compose_frame fixed"
        )

    def on_timer(self):
        self.broadcast_frame("world", "space_frame", np.eye(3))

        axis, angle = STEP_SEQUENCE[self.step_index % len(STEP_SEQUENCE)]
        R_step = ELEMENTARY_ROTATIONS[axis](angle)
        frame = self.get_parameter("compose_frame").value

        if frame == "current":
            self.R_body = self.R_body @ R_step
        else:
            self.R_body = R_step @ self.R_body

        self.step_index += 1

        self.broadcast_frame("world", "body_frame", self.R_body)

    def broadcast_frame(self, parent, child, R):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0 if child == "body_frame" else 0.0
        qx, qy, qz, qw = R_to_quat_xyzw(R)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Hw01TfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
