#!/usr/bin/env python3
"""
Beer Pong - 6 CUPS FINAL with Scene Objects
- Cups touching: diameter = 9cm, center-to-center = 9cm
- Triangle formed by cup centers has 9cm sides
- 4.5 rad/s maximum speed
- Cups added to scene for consistent positioning
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


class BeerPong6Cups(Node):
    def __init__(self):
        super().__init__("beer_pong_6_cups")

        self.declare_parameter("task", "throw_ball")
        self.declare_parameter("pause_between", 5.0)
        self.declare_parameter("cup_distance", 2.0)
        
        self.release_pub = self.create_publisher(String, "release_signal", 10)

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        # Speed settings
        self.slow_vel = 3.0
        self.slow_acc = 3.0
        self.fast_vel = 4.5
        self.fast_acc = 4.5

        self.set_speed(self.slow_vel, self.slow_acc)

        # Throw parameters
        self.windup_shoulder_deg = 35.0
        self.windup_elbow_deg = -150.0
        self.windup_wrist_deg = -80.0
        self.windup_wrist_rotate_deg = 0.0
        
        self.throw_shoulder_deg = -45.0
        self.throw_elbow_deg = 150.0
        self.throw_wrist_deg = 70.0
        self.throw_wrist_rotate_deg = 0.0
        
        self.throw_release_angle_deg = -30.0
        self.throw_release_tol_deg = 10.0

        self.pause_between = self.get_parameter("pause_between").value

        # Gripper
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
        except:
            self.gripper = None

        time.sleep(2.0)

        # Ball pickup position
        self.table_z = 0.0
        self.ball_pickup_pos = (0.35, 0.00, 0.06)

        # ============================================================
        # CUP GEOMETRY - CORRECT EQUILATERAL TRIANGLE
        # ============================================================
        cup_diameter = 0.09  # 9cm
        cup_height = 0.115   # 11.5cm
        cup_radius = cup_diameter / 2  # 4.5cm
        
        # CENTER-TO-CENTER distance when cups touch
        center_to_center = cup_diameter  # 9cm = 0.09m
        
        # For equilateral triangle with side length = 9cm:
        # Height of triangle = side * sqrt(3)/2 = 9 * 0.866 = 7.79cm
        row_spacing = center_to_center * math.sqrt(3) / 2  # 0.0779m = 7.79cm
        
        # Distance from robot to front cup (adjustable)
        distance_to_front = self.get_parameter("cup_distance").value
        
        # ============================================================
        # CALCULATE CUP CENTER POSITIONS (Equilateral Triangle)
        # ============================================================
        # Triangle FLIPPED: 3 cups closest, 1 cup furthest
        #           6        (Back row - 1 cup, FURTHEST)
        #         4   5      (Middle row - 2 cups)
        #       1   2   3    (Front row - 3 cups, CLOSEST)
        
        # Front row (3 cups) - CLOSEST - Y = -9cm, 0, +9cm
        front_row_x = distance_to_front
        front_row_y_left = -center_to_center    # -9cm (Cup 1)
        front_row_y_center = 0.0                # 0cm (Cup 2)
        front_row_y_right = center_to_center    # +9cm (Cup 3)
        
        # Middle row (2 cups) - Y = ±4.5cm
        middle_row_x = front_row_x + row_spacing
        middle_row_y_spacing = center_to_center / 2  # Half of 9cm = 4.5cm
        middle_row_y_left = -middle_row_y_spacing   # -4.5cm (Cup 4)
        middle_row_y_right = middle_row_y_spacing   # +4.5cm (Cup 5)
        
        # Back row (1 cup) - FURTHEST - Y=0 (centered)
        back_row_x = middle_row_x + row_spacing
        back_row_y = 0.0  # Center (Cup 6)
        
        # Store cup physical positions for scene
        self.cup_physical_positions = [
            (front_row_x, front_row_y_left, cup_height/2),    # Cup 1
            (front_row_x, front_row_y_center, cup_height/2),  # Cup 2
            (front_row_x, front_row_y_right, cup_height/2),   # Cup 3
            (middle_row_x, middle_row_y_left, cup_height/2),  # Cup 4
            (middle_row_x, middle_row_y_right, cup_height/2), # Cup 5
            (back_row_x, back_row_y, cup_height/2),           # Cup 6
        ]
        
        self.cup_dimensions = {
            'radius': cup_radius,
            'height': cup_height
        }
        
        # ============================================================
        # CALCULATE ROTATION ANGLES
        # ============================================================
        # Base angles from geometry (FLIPPED triangle)
        cup_1_angle = math.degrees(math.atan2(front_row_y_left, front_row_x))    # Front left
        cup_2_angle = math.degrees(math.atan2(front_row_y_center, front_row_x))  # Front center
        cup_3_angle = math.degrees(math.atan2(front_row_y_right, front_row_x))   # Front right
        cup_4_angle = math.degrees(math.atan2(middle_row_y_left, middle_row_x))  # Middle left
        cup_5_angle = math.degrees(math.atan2(middle_row_y_right, middle_row_x)) # Middle right
        cup_6_angle = math.degrees(math.atan2(back_row_y, back_row_x))           # Back center
        
        # Scale angles for proper physical throw
        # REDUCED to bring cups closer together horizontally
        angle_scale = 2.0  # Reduced from 2.5 for tighter horizontal spacing
        
        # Shift triangle left (positive = left)
        triangle_offset = +8.0  # Shift MORE to the left
        
        # Apply scaling and offset
        cup_1_angle = (cup_1_angle * angle_scale) + triangle_offset
        cup_2_angle = (cup_2_angle * angle_scale) + triangle_offset
        cup_3_angle = (cup_3_angle * angle_scale) + triangle_offset
        cup_4_angle = (cup_4_angle * angle_scale) + triangle_offset
        cup_5_angle = (cup_5_angle * angle_scale) + triangle_offset
        cup_6_angle = (cup_6_angle * angle_scale) + triangle_offset
        
        # ============================================================
        # CUP POSITIONS ARRAY (FLIPPED TRIANGLE)
        # ============================================================
        self.cup_positions = [
            # Front row (3 cups) - CLOSEST
            {"name": "Cup 1 - Front Left", "joint_1": cup_1_angle, "distance": front_row_x},
            {"name": "Cup 2 - Front Center", "joint_1": cup_2_angle, "distance": front_row_x},
            {"name": "Cup 3 - Front Right", "joint_1": cup_3_angle, "distance": front_row_x},
            
            # Middle row (2 cups)
            {"name": "Cup 4 - Middle Left", "joint_1": cup_4_angle, "distance": middle_row_x},
            {"name": "Cup 5 - Middle Right", "joint_1": cup_5_angle, "distance": middle_row_x},
            
            # Back row (1 cup) - FURTHEST
            {"name": "Cup 6 - Back Center", "joint_1": cup_6_angle, "distance": back_row_x},
        ]

        self.q_down = (-0.7071, 0.7071, 0.0, 0.0)
        self.j_retract = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.touch_links = ["end_effector_link", "right_finger_bottom_link", "left_finger_bottom_link"]
        self.ball_released = False

        shoulder_swing = abs(self.throw_shoulder_deg - self.windup_shoulder_deg)
        elbow_swing = abs(self.throw_elbow_deg - self.windup_elbow_deg)
        wrist_swing = abs(self.throw_wrist_deg - self.windup_wrist_deg)

        self.get_logger().info("=" * 80)
        self.get_logger().info("🔥🚀 6 CUPS - FINAL with Scene Objects")
        self.get_logger().info(f"   Velocity: 4.5 rad/s (258°/s) - HARDWARE MAXIMUM")
        self.get_logger().info(f"   Cup diameter: 9cm | Cup height: 11.5cm")
        self.get_logger().info(f"   Cup spacing: 9cm center-to-center (touching)")
        self.get_logger().info(f"   Triangle offset: +8.0° (shifted LEFT)")
        self.get_logger().info(f"   Distance to front cup: {distance_to_front:.2f}m")
        self.get_logger().info(f"   Angle scale: 2.0x (TIGHTER horizontal spacing)")
        self.get_logger().info(f"   Cups in scene: YES (consistent across runs)")
        self.get_logger().info(f"   Motion: {shoulder_swing:.0f}° shoulder + {elbow_swing:.0f}° elbow + {wrist_swing:.0f}° wrist")
        self.get_logger().info(f"   Release: {self.throw_release_angle_deg:.0f}° (EARLY)")
        self.get_logger().info(f"   Pause: {self.pause_between:.1f}s after each throw")
        self.get_logger().info("=" * 80)
        self.get_logger().info("\n   Triangle Formation (FLIPPED - 3 cups closest):")
        self.get_logger().info("          6        (Back - 1 cup, furthest)")
        self.get_logger().info("        4   5      (Middle - 2 cups)")
        self.get_logger().info("      1   2   3    (Front - 3 cups, closest)")
        self.get_logger().info("")
        self.get_logger().info("   Cup Angles:")
        for cup in self.cup_positions:
            self.get_logger().info(f"   {cup['name']}: {cup['joint_1']:+.2f}° at {cup['distance']:.3f}m")
        self.get_logger().info("")

    def set_speed(self, vel: float, acc: float):
        self.moveit2.max_velocity = vel
        self.moveit2.max_acceleration = acc

    def move_to_pose(self, pose: Pose, cartesian=False):
        time.sleep(0.03)
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

                angle = js.position[2]

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
        """Add ball and all 6 cups to the scene"""
        time.sleep(0.05)
        
        # Add ball
        try:
            self.get_logger().info("📍 Adding ball to scene...")
            self.moveit2.add_collision_sphere(
                id="ball", 
                radius=0.02, 
                position=self.ball_pickup_pos, 
                frame_id="base_link"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not add ball: {e}")
        
        # Add all 6 cups to scene
        self.get_logger().info("📍 Adding 6 cups to scene for consistent positioning...")
        for i, (x, y, z) in enumerate(self.cup_physical_positions, start=1):
            try:
                self.moveit2.add_collision_cylinder(
                    id=f"cup_{i}",
                    radius=self.cup_dimensions['radius'],
                    height=self.cup_dimensions['height'],
                    position=(x, y, z),
                    frame_id="base_link"
                )
                self.get_logger().info(f"   Cup {i} added at ({x:.3f}, {y:.3f}, {z:.3f})")
            except Exception as e:
                self.get_logger().warn(f"Could not add cup {i}: {e}")
        
        self.get_logger().info("✅ Scene setup complete!\n")

    def throw_to_cup(self, cup_number: int, cup_info: dict):
        self.ball_released = False
        
        self.get_logger().info(f"\n{'='*80}")
        self.get_logger().info(f"🎯 CUP {cup_number}/6 - {cup_info['name']}")
        self.get_logger().info(f"   Rotation: {cup_info['joint_1']:+.2f}° | Distance: {cup_info['distance']:.3f}m")
        self.get_logger().info(f"{'='*80}\n")

        self.set_speed(self.slow_vel, self.slow_acc)
        self.move_to_joints(self.j_retract)
        time.sleep(0.15)

        # Pickup
        self.get_logger().info("--- FAST PICKUP (3.0 rad/s) ---")
        self.get_logger().info(f"   Position: ({self.ball_pickup_pos[0]:.2f}, {self.ball_pickup_pos[1]:.2f}, {self.ball_pickup_pos[2]:.2f})")
        
        self.open_gripper()
        self.move_to_pose(make_pose(0.35, 0.00, 0.21, *self.q_down))
        # self.move_to_pose(make_pose(0.35, 0.00, 0.06, *self.q_down))
        self.close_gripper()
        time.sleep(0.08)
        self.attach_ball()
        time.sleep(0.05)
        self.move_to_pose(make_pose(0.35, 0.00, 0.26, *self.q_down))

        # Throw
        self.get_logger().info(f"\n--- EXTREME THROW to {cup_info['name']} (4.5 rad/s) ---")

        wind_up = self.j_retract.copy()
        wind_up[0] = math.radians(cup_info['joint_1'])
        wind_up[1] = math.radians(self.windup_shoulder_deg)
        wind_up[2] = math.radians(self.windup_elbow_deg)
        wind_up[4] = math.radians(self.windup_wrist_deg)
        wind_up[5] = math.radians(self.windup_wrist_rotate_deg)

        self.get_logger().info(f"⬅️  Wind-up: Base {cup_info['joint_1']:+.2f}°")
        
        success = self.move_to_joints(wind_up)
        if not success:
            wind_up[2] = math.radians(-145.0)
            wind_up[4] = math.radians(-75.0)
            self.move_to_joints(wind_up)

        time.sleep(0.15)

        throw_cfg = self.j_retract.copy()
        throw_cfg[0] = math.radians(cup_info['joint_1'])
        throw_cfg[1] = math.radians(self.throw_shoulder_deg)
        throw_cfg[2] = math.radians(self.throw_elbow_deg)
        throw_cfg[4] = math.radians(self.throw_wrist_deg)
        throw_cfg[5] = math.radians(self.throw_wrist_rotate_deg)

        self.monitor_release_angle()

        def execute_throw():
            self.get_logger().info(f"🚀💨🔥 THROWING AT 4.5 RAD/S to {cup_info['name']}!")
            self.set_speed(self.fast_vel, self.fast_acc)
            
            success = self.move_to_joints(throw_cfg)
            if not success:
                throw_cfg[1] = math.radians(-40.0)
                throw_cfg[2] = math.radians(145.0)
                throw_cfg[4] = math.radians(65.0)
                self.move_to_joints(throw_cfg)
            
            self.set_speed(self.slow_vel, self.slow_acc)

        throw_thread = threading.Thread(target=execute_throw, daemon=True)
        throw_thread.start()
        time.sleep(2.0)

        if not self.ball_released:
            self.detach_ball()

        time.sleep(0.15)

        self.get_logger().info("\n--- RETURN ---")
        self.move_to_joints(self.j_retract)

        self.get_logger().info(f"\n✅ Cup {cup_number}/6 ({cup_info['name']}) COMPLETE!\n")

    def throw_all_cups(self):
        self.get_logger().info(f"\n🔥🚀💨 STARTING 6-CUP SEQUENCE!\n")
        
        for cup_num, cup_info in enumerate(self.cup_positions, start=1):
            self.throw_to_cup(cup_num, cup_info)
            
            if cup_num < len(self.cup_positions):
                self.get_logger().info(f"\n⏸️  PAUSING {self.pause_between:.1f} SECONDS - PLACE NEXT BALL NOW!")
                self.get_logger().info(f"   Place ball at: ({self.ball_pickup_pos[0]:.2f}, {self.ball_pickup_pos[1]:.2f}, {self.ball_pickup_pos[2]:.2f})")
                self.get_logger().info(f"   Next: Cup {cup_num + 1}/6 - {self.cup_positions[cup_num]['name']}\n")
                time.sleep(self.pause_between)

        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("✅✅✅ ALL 6 CUPS COMPLETE!")
        self.get_logger().info("   9cm equilateral triangle ✓")
        self.get_logger().info("   Cups in scene for consistency ✓")
        self.get_logger().info("=" * 80 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = BeerPong6Cups()

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
            node.throw_all_cups()
        elif task == "retract":
            node.move_to_joints(node.j_retract)
    except KeyboardInterrupt:
        node.get_logger().info("\n⚠️  Interrupted by user")
    finally:
        time.sleep(1.0)
        rclpy.shutdown()


if __name__ == "__main__":
    main()