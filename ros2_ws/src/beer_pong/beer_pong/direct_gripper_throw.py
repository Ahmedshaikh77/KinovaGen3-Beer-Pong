#!/usr/bin/env python3
"""
Beer Pong System - DIRECT ACTION CLIENT VERSION
Uses raw action calls instead of GripperInterface for better reliability
"""
import time
import threading
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
from control_msgs.action import GripperCommand
from std_msgs.msg import String


def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    p.orientation.w = float(qw)
    return p


class BeerPongThrow(Node):
    def __init__(self):
        super().__init__("beer_pong_throw")

        self.declare_parameter("task", "throw_ball")
        self.release_pub = self.create_publisher(String, "release_signal", 10)

        # MoveIt2 for arm control
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )
        
        self.moveit2.max_velocity = 3.0
        self.moveit2.max_acceleration = 3.0
        
        # DIRECT ACTION CLIENT for gripper (instead of GripperInterface)
        self.get_logger().info("Creating gripper action client...")
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/gen3_lite_2f_gripper_controller/gripper_cmd'
        )
        
        self.get_logger().info("Waiting for gripper action server...")
        if self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("✓ Gripper action server ready!")
            self.gripper_available = True
        else:
            self.get_logger().error("✗ Gripper action server not available!")
            self.gripper_available = False

        self.get_logger().info("Waiting for MoveIt services...")
        time.sleep(2.0)

        self.ball_pickup_pos = (0.35, 0.00, 0.06)
        self.cup_pos = (2.50, 0.00, 0.05)
        self.ground_z = -0.01
        self.q_down = (-0.7071, 0.7071, 0.0, 0.0)
        self.j_retract = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.touch_links = ["end_effector_link", "right_finger_bottom_link", "left_finger_bottom_link"]
        self.ball_released = False

    def send_gripper_command(self, position, max_effort=100.0):
        """Send gripper command using direct action client"""
        if not self.gripper_available:
            self.get_logger().warn("Gripper not available - skipping")
            return False
        
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        
        self.get_logger().info(f"Sending gripper command: position={position:.2f}")
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result() is not None:
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=3.0)
                
                if result_future.result() is not None:
                    self.get_logger().info(f"✓ Gripper command completed")
                    return True
        
        self.get_logger().error("✗ Gripper command failed")
        return False

    def open_gripper(self):
        """Open gripper - position 0.80"""
        self.get_logger().info("🔓 Opening gripper (position 0.80)")
        success = self.send_gripper_command(0.80)
        if success:
            time.sleep(2.0)
            self.get_logger().info("✓ Gripper opened")

    def close_gripper(self):
        """Close gripper - position 0.01"""
        self.get_logger().info("🔒 Closing gripper (position 0.01)")
        success = self.send_gripper_command(0.01)
        if success:
            time.sleep(2.0)
            self.get_logger().info("✓ Gripper closed")

    def move_to_pose(self, pose: Pose, cartesian=False):
        self.get_logger().info(f"Moving to: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
        time.sleep(0.2)
        try:
            if cartesian:
                self.moveit2.move_to_pose(
                    position=[pose.position.x, pose.position.y, pose.position.z],
                    quat_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                    cartesian=True, cartesian_max_step=0.01, cartesian_fraction_threshold=0.0,
                )
            else:
                self.moveit2.move_to_pose(
                    position=[pose.position.x, pose.position.y, pose.position.z],
                    quat_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                )
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"Motion failed: {e}")
            return False

    def move_to_joints(self, joints):
        self.get_logger().info(f"Moving to joints: {[f'{math.degrees(j):.1f}°' for j in joints]}")
        try:
            self.moveit2.move_to_configuration(joint_positions=joints)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"Joint motion failed: {e}")
            return False

    def attach_ball(self):
        try:
            self.moveit2.attach_collision_object("ball", "end_effector_link", self.touch_links)
            self.get_logger().info("✓ Ball attached")
        except Exception as e:
            self.get_logger().warn(f"Attach failed: {e}")

    def detach_ball(self):
        try:
            self.moveit2.detach_collision_object("ball")
            self.get_logger().info("✓ Ball detached")
            self.ball_released = True
        except Exception as e:
            self.get_logger().warn(f"Detach failed: {e}")
            self.ball_released = True

    def monitor_release_angle(self, target_angle_deg=0.0, tolerance_deg=3.0):
        target = math.radians(target_angle_deg)
        tol = math.radians(tolerance_deg)
        released = False

        def check_angle():
            nonlocal released
            if released:
                return
            try:
                if hasattr(self.moveit2, 'joint_state') and self.moveit2.joint_state and len(self.moveit2.joint_state.position) > 2:
                    angle = self.moveit2.joint_state.position[2]
                    if abs(angle - target) < tol:
                        self.get_logger().info(f"🎯 RELEASE at joint_3 = {math.degrees(angle):.1f}°")
                        self.release_pub.publish(String(data="RELEASE"))
                        self.detach_ball()
                        # Open gripper directly here
                        self.send_gripper_command(0.80)  # Open
                        released = True
                        timer.cancel()
            except Exception as e:
                self.get_logger().warn(f"Angle check error: {e}")

        timer = self.create_timer(0.01, check_angle)
        return timer

    def add_scene(self):
        self.get_logger().info("Setting up scene...")
        time.sleep(0.5)
        
        self.moveit2.add_collision_box(id="ground_plane", size=(4.0, 2.0, 0.02), position=(1.5, 0.0, self.ground_z - 0.01), quat_xyzw=(0, 0, 0, 1), frame_id="base_link")
        time.sleep(0.1)
        self.moveit2.add_collision_box(id="block_1", size=(0.04, 0.04, 0.04), position=(0.35, 0.0, 0.02), quat_xyzw=(0, 0, 0, 1), frame_id="base_link")
        time.sleep(0.1)
        self.moveit2.add_collision_sphere(id="ball", radius=0.02, position=self.ball_pickup_pos, frame_id="base_link")
        time.sleep(0.1)
        self.moveit2.add_collision_cylinder(id="target_cup", height=0.10, radius=0.045, position=(2.50, 0.0, 0.05), quat_xyzw=(0, 0, 0, 1), frame_id="base_link")
        time.sleep(0.1)
        
        for i, pos in enumerate([(2.45, -0.10, 0.05), (2.45, 0.10, 0.05), (2.40, -0.15, 0.05), (2.40, 0.0, 0.05), (2.40, 0.15, 0.05)]):
            self.moveit2.add_collision_cylinder(id=f"cup_{i+1}", height=0.10, radius=0.045, position=pos, quat_xyzw=(0, 0, 0, 1), frame_id="base_link")
            time.sleep(0.05)
        
        self.get_logger().info("✓ Scene ready")

    def throw_ball(self):
        self.get_logger().info("="*60)
        self.get_logger().info("🍺 BEER PONG THROW - DIRECT ACTION VERSION")
        self.get_logger().info("="*60)
        
        self.move_to_joints(self.j_retract)
        time.sleep(0.5)
        
        self.get_logger().info("\n--- PICKUP ---")
        self.open_gripper()
        
        pre_grasp = make_pose(self.ball_pickup_pos[0], self.ball_pickup_pos[1], self.ball_pickup_pos[2] + 0.15, *self.q_down)
        if self.move_to_pose(pre_grasp):
            grasp = make_pose(*self.ball_pickup_pos, *self.q_down)
            self.move_to_pose(grasp, cartesian=False)
        
        self.close_gripper()
        time.sleep(0.5)
        self.attach_ball()
        time.sleep(0.2)
        
        lift = make_pose(self.ball_pickup_pos[0], self.ball_pickup_pos[1], self.ball_pickup_pos[2] + 0.20, *self.q_down)
        self.move_to_pose(lift, cartesian=False)
        
        self.get_logger().info("\n--- THROW ---")
        
        wind_up = self.j_retract.copy()
        wind_up[0] = math.radians(0)
        wind_up[2] = math.radians(-135)
        
        self.get_logger().info("⚡ Wind-up to -135°")
        self.move_to_joints(wind_up)
        time.sleep(0.5)
        
        throw = self.j_retract.copy()
        throw[0] = math.radians(0)
        throw[2] = math.radians(100)
        
        self.get_logger().info("🎯 Release monitoring...")
        self.monitor_release_angle(target_angle_deg=-25.0, tolerance_deg=5.0)
        
        def execute_throw():
            self.get_logger().info("🚀 THROW!")
            self.move_to_joints(throw)
        
        threading.Thread(target=execute_throw, daemon=True).start()
        time.sleep(3.5)
        
        if not self.ball_released:
            self.detach_ball()
            self.send_gripper_command(0.80)  # Open
        
        time.sleep(0.5)
        
        self.get_logger().info("\n--- RETURN ---")
        self.move_to_joints(self.j_retract)
        self.get_logger().info("✓ Complete!")


def main(args=None):
    rclpy.init(args=args)
    node = BeerPongThrow()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    task = node.get_parameter("task").get_parameter_value().string_value
    try:
        if task == "add_scene":
            node.add_scene()
        elif task == "throw_ball":
            node.add_scene()
            time.sleep(1.0)
            node.throw_ball()
        elif task == "retract":
            node.move_to_joints(node.j_retract)
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(1.0)
        rclpy.shutdown()


if __name__ == "__main__":
    main()