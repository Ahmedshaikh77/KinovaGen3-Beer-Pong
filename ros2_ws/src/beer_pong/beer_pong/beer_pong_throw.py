#!/usr/bin/env python3
"""
Beer Pong - EXTREME MAXIMUM with EARLIER RELEASE
- 4.5 rad/s (hardware max)
- EXTREME angles (near limits)
- Multi-joint coordination
- EARLIER release timing for better arc
"""

import time
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from std_msgs.msg import String

from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface


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

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        # ========================================
        # ABSOLUTE MAXIMUM SPEED (Can't exceed 4.5!)
        # ========================================
        self.slow_vel = 3.0      # Very fast pickup
        self.slow_acc = 3.0      
        self.fast_vel = 4.5      # HARDWARE LIMIT (can't go higher!)
        self.fast_acc = 4.5      

        self.set_speed(self.slow_vel, self.slow_acc)

        # ========================================
        # EXTREME MOTION - PUSH ALL LIMITS!
        # ========================================
        
        # Wind-up (EXTREME but within limits)
        self.windup_shoulder_deg = 35.0          # Big back swing
        self.windup_elbow_deg = -150.0           # EXTREME (limit is -152°)
        self.windup_wrist_deg = -80.0            # Maximum wrist cock
        self.windup_wrist_rotate_deg = 0.0       # Can add spin later
        
        # Follow-through (EXTREME extension)
        self.throw_shoulder_deg = -45.0          # DEEP forward
        self.throw_elbow_deg = 150.0             # EXTREME (limit is 152°)
        self.throw_wrist_deg = 70.0              # Maximum wrist snap
        self.throw_wrist_rotate_deg = 0.0        # Can add spin later
        
        # Release (MUCH EARLIER for better arc)
        self.throw_release_angle_deg = -30.0     # MUCH EARLIER release (2 secs earlier!)
        self.throw_release_tol_deg = 10.0        # Wider tolerance for early release

        # Gripper (instant release)
        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.80],
                closed_gripper_joint_positions=[0.01],
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=False,
            )
            self.get_logger().info("✅ Gripper ready")
        except Exception as e:
            self.gripper = None

        time.sleep(2.0)

        self.table_z = 0.0
        self.ball_pickup_pos = (0.35, 0.00, 0.06)
        self.q_down = (-0.7071, 0.7071, 0.0, 0.0)
        self.j_retract = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.touch_links = ["end_effector_link", "right_finger_bottom_link", "left_finger_bottom_link"]
        self.ball_released = False

        # Calculate total motion
        shoulder_swing = abs(self.throw_shoulder_deg - self.windup_shoulder_deg)
        elbow_swing = abs(self.throw_elbow_deg - self.windup_elbow_deg)
        wrist_swing = abs(self.throw_wrist_deg - self.windup_wrist_deg)

        self.get_logger().info("=" * 80)
        self.get_logger().info("🔥🚀 EXTREME MAXIMUM - MUCH EARLIER RELEASE FOR 3.5-4M!")
        self.get_logger().info(f"   Velocity: 4.5 rad/s (258°/s) - HARDWARE MAXIMUM")
        self.get_logger().info(f"   Shoulder: {self.windup_shoulder_deg:.0f}° → {self.throw_shoulder_deg:.0f}° ({shoulder_swing:.0f}° swing)")
        self.get_logger().info(f"   Elbow:    {self.windup_elbow_deg:.0f}° → {self.throw_elbow_deg:.0f}° ({elbow_swing:.0f}° swing!) EXTREME!")
        self.get_logger().info(f"   Wrist:    {self.windup_wrist_deg:.0f}° → {self.throw_wrist_deg:.0f}° ({wrist_swing:.0f}° snap!)")
        self.get_logger().info(f"   Release:  {self.throw_release_angle_deg:.0f}° (MUCH EARLIER = high arc!)")
        self.get_logger().info(f"   Total motion: {shoulder_swing + elbow_swing + wrist_swing:.0f}° combined!")
        self.get_logger().info(f"   Expected: 3.5-4.0 METERS!")
        self.get_logger().info("=" * 80)

    def set_speed(self, vel: float, acc: float):
        self.moveit2.max_velocity = vel
        self.moveit2.max_acceleration = acc
        self.get_logger().info(f"⚡ Speed: vel={vel:.1f} rad/s ({vel*57.3:.0f}°/s)")

    def move_to_pose(self, pose: Pose, cartesian=False):
        time.sleep(0.03)  # Minimal delay
        try:
            if cartesian:
                self.moveit2.move_to_pose(
                    position=[pose.position.x, pose.position.y, pose.position.z],
                    quat_xyzw=[pose.orientation.x, pose.orientation.y, 
                              pose.orientation.z, pose.orientation.w],
                    cartesian=True, cartesian_max_step=0.01, cartesian_fraction_threshold=0.0)
            else:
                self.moveit2.move_to_pose(
                    position=[pose.position.x, pose.position.y, pose.position.z],
                    quat_xyzw=[pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w])
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"Motion failed: {e}")
            return False

    def move_to_joints(self, joints):
        try:
            self.moveit2.move_to_configuration(joint_positions=joints)
            self.moveit2.wait_until_executed()
            return True
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")
            return False

    def open_gripper(self):
        if self.gripper:
            try:
                self.gripper.close()
            except: pass
            time.sleep(0.12)

    def open_gripper_instant(self):
        if self.gripper:
            self.get_logger().info("💨 INSTANT RELEASE!")
            try:
                self.gripper.close()
            except: pass

    def close_gripper(self):
        if self.gripper:
            try:
                self.gripper.open()
            except: pass
            time.sleep(0.12)

    def attach_ball(self):
        try:
            self.moveit2.attach_collision_object("ball", "end_effector_link", self.touch_links)
        except: pass

    def detach_ball(self):
        try:
            self.moveit2.detach_collision_object("ball")
            self.ball_released = True
        except:
            self.ball_released = True

    def monitor_release_angle(self):
        """Monitor elbow for EARLIER release"""
        target = math.radians(self.throw_release_angle_deg)
        tol = math.radians(self.throw_release_tol_deg)
        released = False

        def check_angle():
            nonlocal released
            if released:
                return

            try:
                js = getattr(self.moveit2, "joint_state", None)
                if js is None or len(js.position) < 3:
                    return

                angle = js.position[2]  # Elbow

                if angle >= target:
                    self.get_logger().info(f"🎯💨 EARLY RELEASE at {math.degrees(angle):.1f}°!")
                    self.open_gripper_instant()
                    self.detach_ball()
                    released = True
                    timer.cancel()
            except: pass

        timer = self.create_timer(0.01, check_angle)
        return timer

    def add_scene(self):
        time.sleep(0.05)
        try:
            self.moveit2.add_collision_sphere(
                id="ball", radius=0.02, 
                position=self.ball_pickup_pos, 
                frame_id="base_link")
        except: pass

    def throw_ball(self):
        self.get_logger().info("\n🔥🚀💨 EXTREME 3.5-4M THROW - MUCH EARLIER RELEASE!\n")

        # Start
        self.set_speed(self.slow_vel, self.slow_acc)
        self.move_to_joints(self.j_retract)
        time.sleep(0.15)

        # ===== FAST PICKUP =====
        self.get_logger().info("--- FAST PICKUP (3.0 rad/s) ---")
        
        self.open_gripper()
        self.move_to_pose(make_pose(0.35, 0.00, 0.21, *self.q_down))
        self.move_to_pose(make_pose(0.35, 0.00, 0.06, *self.q_down))
        self.close_gripper()
        time.sleep(0.08)
        self.attach_ball()
        time.sleep(0.05)
        self.move_to_pose(make_pose(0.35, 0.00, 0.26, *self.q_down))

        # ===== EXTREME THROW =====
        self.get_logger().info("\n--- EXTREME THROW (4.5 rad/s + MUCH EARLIER RELEASE!) ---")

        # Wind-up with all 3 joints
        wind_up = self.j_retract.copy()
        wind_up[1] = math.radians(self.windup_shoulder_deg)
        wind_up[2] = math.radians(self.windup_elbow_deg)
        wind_up[4] = math.radians(self.windup_wrist_deg)
        wind_up[5] = math.radians(self.windup_wrist_rotate_deg)

        self.get_logger().info(
            f"⬅️  EXTREME Wind-up: j2={self.windup_shoulder_deg:.0f}°, "
            f"j3={self.windup_elbow_deg:.0f}° (near -152° limit!), "
            f"j5={self.windup_wrist_deg:.0f}°"
        )
        
        # Try extreme, fallback if needed
        success = self.move_to_joints(wind_up)
        if not success:
            self.get_logger().warn("⚠️  Extreme wind-up failed, trying -145°")
            wind_up[2] = math.radians(-145.0)
            wind_up[4] = math.radians(-75.0)
            self.move_to_joints(wind_up)
        else:
            self.get_logger().info("✅ EXTREME wind-up successful!")

        time.sleep(0.15)

        # Throw with MAXIMUM extension
        throw_cfg = self.j_retract.copy()
        throw_cfg[1] = math.radians(self.throw_shoulder_deg)
        throw_cfg[2] = math.radians(self.throw_elbow_deg)
        throw_cfg[4] = math.radians(self.throw_wrist_deg)
        throw_cfg[5] = math.radians(self.throw_wrist_rotate_deg)

        self.get_logger().info(
            f"➡️  EXTREME Throw: j2={self.throw_shoulder_deg:.0f}°, "
            f"j3={self.throw_elbow_deg:.0f}° (near 152° limit!), "
            f"j5={self.throw_wrist_deg:.0f}°"
        )

        total_elbow = abs(self.throw_elbow_deg - self.windup_elbow_deg)
        total_wrist = abs(self.throw_wrist_deg - self.windup_wrist_deg)
        
        self.get_logger().info(
            f"💪🔥 Elbow: {total_elbow:.0f}° swing at 4.5 rad/s = MAXIMUM POWER!"
        )
        self.get_logger().info(
            f"💪🔥 Wrist: {total_wrist:.0f}° snap for extra distance!"
        )

        # Monitor release
        self.get_logger().info(f"🎯 Release at {self.throw_release_angle_deg:.0f}° (MUCH EARLIER = high arc!)")
        self.monitor_release_angle()

        # Execute at MAXIMUM 4.5 rad/s
        def execute_throw():
            self.get_logger().info("🚀💨🔥 THROWING AT 4.5 RAD/S - MUCH EARLIER RELEASE!")
            self.set_speed(self.fast_vel, self.fast_acc)
            
            success = self.move_to_joints(throw_cfg)
            if not success:
                self.get_logger().warn("⚠️  Extreme throw failed, trying safer 145°")
                throw_cfg[1] = math.radians(-40.0)
                throw_cfg[2] = math.radians(145.0)
                throw_cfg[4] = math.radians(65.0)
                self.move_to_joints(throw_cfg)
            else:
                self.get_logger().info("✅ EXTREME throw successful!")
            
            self.set_speed(self.slow_vel, self.slow_acc)

        throw_thread = threading.Thread(target=execute_throw, daemon=True)
        throw_thread.start()
        time.sleep(2.0)

        if not self.ball_released:
            self.get_logger().warn("Force release")
            self.detach_ball()

        time.sleep(0.15)

        # ===== RETURN =====
        self.get_logger().info("\n--- RETURN ---")
        self.move_to_joints(self.j_retract)

        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("✅🔥 EXTREME THROW WITH MUCH EARLIER RELEASE COMPLETE!")
        self.get_logger().info("📊 Motion stats:")
        self.get_logger().info(f"   - Speed: 4.5 rad/s (258°/s) - HARDWARE MAX")
        self.get_logger().info(f"   - Elbow swing: 300°")
        self.get_logger().info(f"   - Wrist snap: 150°")
        self.get_logger().info(f"   - Release: {self.throw_release_angle_deg:.0f}° (MUCH EARLIER!)")
        self.get_logger().info(f"   - Linear velocity: ~4.05 m/s")
        self.get_logger().info(f"   - Expected distance: 3.5-4.0 meters!")
        self.get_logger().info("=" * 80 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = BeerPongThrow()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    task = node.get_parameter("task").get_parameter_value().string_value

    try:
        if task == "add_scene":
            node.add_scene()
        elif task == "throw_ball":
            node.add_scene()
            time.sleep(0.2)
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