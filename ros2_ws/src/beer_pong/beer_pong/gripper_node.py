#!/usr/bin/env python3
"""
Gripper Control Node

- Listens on "release_signal" (std_msgs/String)
- Talks to Kinova gripper via pymoveit2.GripperInterface
- Handles inverted hardware:
    OPEN / RELEASE -> physically OPEN gripper  (calls close())
    CLOSE          -> physically CLOSE gripper (calls open())
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymoveit2.gripper_interface import GripperInterface


class GripperNode(Node):
    def __init__(self):
        super().__init__("gripper_node")

        # Initialize gripper interface
        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.80],
                closed_gripper_joint_positions=[0.01],
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=True,
            )
            self.get_logger().info("✅ GripperInterface initialized.")
        except Exception as e:
            self.get_logger().error(f"❌ GripperInterface init failed: {e}")
            self.gripper = None

        # Subscribe to release_signal from beer_pong_throw
        self.subscription = self.create_subscription(
            String,
            "release_signal",
            self.release_callback,
            10,
        )

        self.get_logger().info("🎮 GripperNode ready. Listening on 'release_signal'.")

    # ---------------- Gripper wrappers (inverted) ---------------- #

    def _phys_open(self):
        """Physically OPEN the gripper (hardware inverted: close() -> open)."""
        if self.gripper is None:
            self.get_logger().warn("Cannot OPEN gripper: gripper is None.")
            return
        try:
            self.get_logger().info("Gripper PHYSICAL OPEN (calling gripper.close())")
            self.gripper.close()
        except Exception as e:
            self.get_logger().error(f"Gripper PHYSICAL OPEN failed: {e}")

    def _phys_close(self):
        """Physically CLOSE the gripper (hardware inverted: open() -> close)."""
        if self.gripper is None:
            self.get_logger().warn("Cannot CLOSE gripper: gripper is None.")
            return
        try:
            self.get_logger().info("Gripper PHYSICAL CLOSE (calling gripper.open())")
            self.gripper.open()
        except Exception as e:
            self.get_logger().error(f"Gripper PHYSICAL CLOSE failed: {e}")

    # ---------------- Topic callback ---------------- #

    def release_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"[GripperNode] Received command: {cmd}")

        if cmd == "OPEN":
            self._phys_open()
        elif cmd == "CLOSE":
            self._phys_close()
        elif cmd == "RELEASE":
            self.get_logger().info("RELEASE command -> PHYSICAL OPEN for ball release")
            self._phys_open()
        else:
            self.get_logger().warn(f"Unknown gripper command: '{cmd}'")


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("GripperNode shutting down.")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
