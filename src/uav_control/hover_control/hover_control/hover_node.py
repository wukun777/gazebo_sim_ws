#!/usr/bin/env python3
"""ROS 2 + Gazebo Classic quadrotor takeoff and hover controller.

This node reads the simulated base_link state from gazebo_msgs/LinkStates and
publishes geometry_msgs/Wrench commands to the four force plugins declared in
the roarm_quad model.sdf.
"""

import math
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import SetJointProperties
from geometry_msgs.msg import Quaternion, Wrench
from rclpy.node import Node


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """Return roll, pitch, yaw from a ROS quaternion."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def solve_linear_system(matrix: Sequence[Sequence[float]],
                        vector: Sequence[float]) -> List[float]:
    """Solve a small dense linear system using pivoted Gauss-Jordan."""
    n = len(vector)
    augmented = [list(matrix[i]) + [float(vector[i])] for i in range(n)]

    for column in range(n):
        pivot = max(range(column, n), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1.0e-9:
            raise RuntimeError("Rotor allocation matrix is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]

        divisor = augmented[column][column]
        augmented[column] = [value / divisor for value in augmented[column]]

        for row in range(n):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                augmented[row][index] - factor * augmented[column][index]
                for index in range(n + 1)
            ]
    return [augmented[row][n] for row in range(n)]


class HoverController(Node):
    """Cascaded altitude/attitude controller with four-rotor allocation."""

    ROTOR_POSITIONS = (
        (0.182, -0.150),   # rotor 0: front right, CCW
        (-0.197, 0.228),   # rotor 1: rear left, CCW
        (0.181, 0.229),    # rotor 2: front left, CW
        (-0.200, -0.1515), # rotor 3: rear right, CW
    )
    ROTOR_DIRECTIONS = (1.0, 1.0, -1.0, -1.0)

    def __init__(self) -> None:
        super().__init__("roarm_quad_hover")

        self.declare_parameter("model_name", "roarm_quad")
        self.declare_parameter("target_height", 1.0)
        self.declare_parameter("takeoff_time", 3.0)
        self.declare_parameter("arm_delay", 1.0)
        self.declare_parameter("mass", 4.485001)
        self.declare_parameter("gravity", 9.81)
        # Composite centre of mass expressed in base_link coordinates.  These
        # values come from the masses, link poses and inertial poses in the
        # supplied SDF.  Rotor moments must be calculated about the COM, not
        # about the base_link origin.
        self.declare_parameter("com_x", -0.04789749)
        self.declare_parameter("com_y", 0.03610987)
        self.declare_parameter("max_thrust_per_rotor", 25.0)
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("max_safe_tilt_deg", 55.0)
        self.declare_parameter("kp_xy", 0.8)
        self.declare_parameter("kd_xy", 1.6)
        self.declare_parameter("max_position_tilt_deg", 8.0)
        # F = k_f * omega^2. omega is also sent to Gazebo for rotor animation.
        self.declare_parameter("thrust_coefficient", 8.0e-4)
        self.declare_parameter("max_visual_speed", 220.0)
        self.declare_parameter("visual_update_rate", 10.0)
        self.declare_parameter("joint_velocity_fmax", 0.005)

        self.model_name = str(self.get_parameter("model_name").value)
        self.target_height = float(self.get_parameter("target_height").value)
        self.takeoff_time = max(0.5, float(self.get_parameter("takeoff_time").value))
        self.arm_delay = max(0.0, float(self.get_parameter("arm_delay").value))
        self.mass = float(self.get_parameter("mass").value)
        self.gravity = float(self.get_parameter("gravity").value)
        self.com_x = float(self.get_parameter("com_x").value)
        self.com_y = float(self.get_parameter("com_y").value)
        self.max_rotor_thrust = float(
            self.get_parameter("max_thrust_per_rotor").value
        )
        control_rate = float(self.get_parameter("control_rate").value)
        self.max_safe_tilt = math.radians(
            float(self.get_parameter("max_safe_tilt_deg").value)
        )
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kd_xy = float(self.get_parameter("kd_xy").value)
        self.max_position_tilt = math.radians(
            float(self.get_parameter("max_position_tilt_deg").value)
        )
        self.thrust_coefficient = float(
            self.get_parameter("thrust_coefficient").value
        )
        self.max_visual_speed = float(
            self.get_parameter("max_visual_speed").value
        )
        self.visual_update_period = 1.0 / max(
            1.0, float(self.get_parameter("visual_update_rate").value)
        )
        self.joint_velocity_fmax = float(
            self.get_parameter("joint_velocity_fmax").value
        )

        # Height-loop gains output total thrust in newtons.
        self.kp_z = 18.0
        self.ki_z = 5.0
        self.kd_z = 12.0

        # Attitude-loop gains output body torque in N*m.
        self.kp_roll = 2.8
        self.ki_roll = 0.80
        self.kd_roll = 0.85
        self.kp_pitch = 2.8
        self.ki_pitch = 0.80
        self.kd_pitch = 0.85
        # Yaw is intentionally more damped than the first version.  The first
        # flight log showed an alternating yaw oscillation whose amplitude was
        # growing, so use a lower proportional gain and stronger damping.
        self.kp_yaw = 0.15
        self.kd_yaw = 0.30
        self.max_yaw_torque = 0.12
        self.yaw_moment_coefficient = 0.015

        self.rotor_publishers = [
            self.create_publisher(
                Wrench,
                f"/{self.model_name}/rotor_{index}/cmd_force",
                10,
            )
            for index in range(4)
        ]
        # This is the ROS equivalent of setting rotor joint velocity in
        # Gazebo's manual Joint Control panel.
        # ROS 2 Gazebo normally exposes /set_joint_properties. Some launch
        # files put all Gazebo services below /gazebo, so support both names.
        self.joint_clients = (
            self.create_client(SetJointProperties, "/set_joint_properties"),
            self.create_client(
                SetJointProperties, "/gazebo/set_joint_properties"
            ),
        )
        self.joint_futures = [None, None, None, None]
        self.visual_service_reported = False

        # Some Gazebo Classic worlds expose /link_states, others use the
        # /gazebo/link_states prefix. Listening to both makes the node portable.
        self.create_subscription(LinkStates, "/link_states", self.state_callback, 10)
        self.create_subscription(
            LinkStates, "/gazebo/link_states", self.state_callback, 10
        )

        self.state = None
        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.start_z: Optional[float] = None
        self.target_yaw: Optional[float] = None
        self.z_integral = 0.0
        self.roll_integral = 0.0
        self.pitch_integral = 0.0
        self.nan_reported = False
        self.tilt_fault_reported = False
        # ROS time can jump when /clock first becomes active.  Take-off ramps,
        # PID dt and log periods use a monotonic clock so a clock jump cannot
        # skip the complete take-off ramp in the first control iteration.
        wall_now = time.monotonic()
        self.last_control_time = wall_now
        self.start_time: Optional[float] = None
        self.last_log_time = wall_now
        self.last_visual_update_time = wall_now

        self.allocation_matrix = self.make_allocation_matrix()
        self.timer = self.create_timer(1.0 / control_rate, self.control_callback)
        self.get_logger().info(
            "Waiting for base_link state; direct Gazebo controller is ready."
        )

    def make_allocation_matrix(self) -> List[List[float]]:
        # Moment is (r - COM) x F:
        # tau_x = (y - com_y)*Fz, tau_y = -(x - com_x)*Fz.
        # The yaw row models each rotor's reaction torque.
        return [
            [1.0, 1.0, 1.0, 1.0],
            [
                position[1] - self.com_y
                for position in self.ROTOR_POSITIONS
            ],
            [
                -(position[0] - self.com_x)
                for position in self.ROTOR_POSITIONS
            ],
            [
                direction * self.yaw_moment_coefficient
                for direction in self.ROTOR_DIRECTIONS
            ],
        ]

    def state_callback(self, message: LinkStates) -> None:
        wanted_names = (
            f"{self.model_name}::base_link",
            f"{self.model_name}/base_link",
            "base_link",
        )
        index = next(
            (message.name.index(name) for name in wanted_names if name in message.name),
            None,
        )
        if index is None:
            return

        pose = message.pose[index]
        twist = message.twist[index]
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self.state = None
            self.publish_zero()
            if not self.nan_reported:
                self.get_logger().error(
                    "Gazebo state contains NaN/Inf. Forces were cut immediately. "
                    "Reset the world and check for initial collision penetration."
                )
                self.nan_reported = True
            return

        self.nan_reported = False
        self.state = (pose, twist)
        if self.start_z is None:
            self.start_x = pose.position.x
            self.start_y = pose.position.y
            self.start_z = pose.position.z
            _, _, self.target_yaw = quaternion_to_euler(pose.orientation)
            self.start_time = time.monotonic()
            self.get_logger().info(
                f"State acquired at z={self.start_z:.3f} m. "
                f"Taking off to z={self.start_z + self.target_height:.3f} m. "
                f"Holding xy=({self.start_x:.3f}, {self.start_y:.3f}) m. "
                f"Using COM=({self.com_x:.4f}, {self.com_y:.4f}) m."
            )

    def desired_z(self, elapsed: float) -> float:
        assert self.start_z is not None
        if elapsed <= self.arm_delay:
            return self.start_z
        progress = clamp(
            (elapsed - self.arm_delay) / self.takeoff_time, 0.0, 1.0
        )
        # Smoothstep avoids a discontinuous desired climb speed.
        smooth = progress * progress * (3.0 - 2.0 * progress)
        return self.start_z + self.target_height * smooth

    def control_callback(self) -> None:
        now = time.monotonic()
        dt = clamp(now - self.last_control_time, 0.001, 0.05)
        self.last_control_time = now

        if (
            self.state is None
            or self.start_x is None
            or self.start_y is None
            or self.start_z is None
            or self.target_yaw is None
            or self.start_time is None
        ):
            self.publish_zero()
            return

        pose, twist = self.state
        roll, pitch, yaw = quaternion_to_euler(pose.orientation)

        # A quadrotor cannot recover reliably after it has fallen onto its
        # back.  More importantly, body-frame +Z thrust points downwards once
        # the model is inverted.  Cut the motors and report the real modelling
        # problem instead of driving the body farther through the floor.
        if abs(roll) > self.max_safe_tilt or abs(pitch) > self.max_safe_tilt:
            self.publish_zero()
            self.z_integral = 0.0
            self.roll_integral = 0.0
            self.pitch_integral = 0.0
            if not self.tilt_fault_reported:
                self.get_logger().error(
                    "Unsafe initial/fallen attitude: "
                    f"roll={math.degrees(roll):.1f} deg, "
                    f"pitch={math.degrees(pitch):.1f} deg. "
                    "Forces are cut. Reset Gazebo and check spawn roll, "
                    "landing-gear contact, and model initial pose."
                )
                self.tilt_fault_reported = True
            return
        self.tilt_fault_reported = False

        elapsed = now - self.start_time
        z_reference = self.desired_z(elapsed)

        z_error = z_reference - pose.position.z

        # Outer horizontal-position PD loop.  It commands a world-frame
        # horizontal acceleration back toward the take-off point.
        x_error = self.start_x - pose.position.x
        y_error = self.start_y - pose.position.y
        ax_command = (
            self.kp_xy * x_error
            - self.kd_xy * twist.linear.x
        )
        ay_command = (
            self.kp_xy * y_error
            - self.kd_xy * twist.linear.y
        )

        # Convert desired world-frame horizontal acceleration into desired
        # body roll and pitch.  For small tilt:
        # ax ~= g*(cos(yaw)*pitch + sin(yaw)*roll)
        # ay ~= g*(sin(yaw)*pitch - cos(yaw)*roll)
        desired_pitch = (
            math.cos(yaw) * ax_command
            + math.sin(yaw) * ay_command
        ) / self.gravity
        desired_roll = (
            math.sin(yaw) * ax_command
            - math.cos(yaw) * ay_command
        ) / self.gravity
        desired_roll = clamp(
            desired_roll,
            -self.max_position_tilt,
            self.max_position_tilt,
        )
        desired_pitch = clamp(
            desired_pitch,
            -self.max_position_tilt,
            self.max_position_tilt,
        )

        roll_error = desired_roll - roll
        pitch_error = desired_pitch - pitch
        yaw_error = wrap_pi(self.target_yaw - yaw)

        self.z_integral = clamp(self.z_integral + z_error * dt, -2.0, 2.0)
        self.roll_integral = clamp(
            self.roll_integral + roll_error * dt, -1.5, 1.5
        )
        self.pitch_integral = clamp(
            self.pitch_integral + pitch_error * dt, -1.5, 1.5
        )

        total_thrust = (
            self.mass * self.gravity
            + self.kp_z * z_error
            + self.ki_z * self.z_integral
            - self.kd_z * twist.linear.z
        )

        # Compensate the loss of vertical lift at modest tilt angles.
        attitude_cosine = max(0.65, math.cos(roll) * math.cos(pitch))
        total_thrust = clamp(
            total_thrust / attitude_cosine,
            0.0,
            4.0 * self.max_rotor_thrust,
        )

        roll_torque = (
            self.kp_roll * roll_error
            + self.ki_roll * self.roll_integral
            - self.kd_roll * twist.angular.x
        )
        pitch_torque = (
            self.kp_pitch * pitch_error
            + self.ki_pitch * self.pitch_integral
            - self.kd_pitch * twist.angular.y
        )
        yaw_torque = clamp(
            self.kp_yaw * yaw_error
            - self.kd_yaw * twist.angular.z,
            -self.max_yaw_torque,
            self.max_yaw_torque,
        )

        try:
            thrusts = solve_linear_system(
                self.allocation_matrix,
                (total_thrust, roll_torque, pitch_torque, yaw_torque),
            )
        except RuntimeError as error:
            self.get_logger().error(str(error))
            self.publish_zero()
            return

        # Saturating individual motors can spoil attitude control, but is much
        # safer than sending negative or unbounded thrust to Gazebo.
        thrusts = [
            clamp(thrust, 0.0, self.max_rotor_thrust) for thrust in thrusts
        ]
        if not all(math.isfinite(thrust) for thrust in thrusts):
            self.publish_zero()
            return

        for index, (publisher, thrust) in enumerate(
            zip(self.rotor_publishers, thrusts)
        ):
            message = Wrench()
            message.force.z = thrust
            message.torque.z = (
                self.ROTOR_DIRECTIONS[index]
                * self.yaw_moment_coefficient
                * thrust
            )
            publisher.publish(message)

        self.update_visual_rotor_speeds(thrusts, now)

        if now - self.last_log_time >= 1.0:
            self.last_log_time = now
            self.get_logger().info(
                f"xyz=({pose.position.x:.2f},{pose.position.y:.2f},"
                f"{pose.position.z:.2f}) m, "
                f"target=({self.start_x:.2f},{self.start_y:.2f},"
                f"{z_reference:.2f}) m, "
                f"vxy=({twist.linear.x:.2f},{twist.linear.y:.2f}) m/s, "
                f"rpy=({math.degrees(roll):.1f},"
                f"{math.degrees(pitch):.1f},{math.degrees(yaw):.1f}) deg, "
                f"rp_ref=({math.degrees(desired_roll):.1f},"
                f"{math.degrees(desired_pitch):.1f}) deg, "
                f"rotors={[round(value, 2) for value in thrusts]} N"
            )

    def update_visual_rotor_speeds(self, thrusts, now) -> None:
        """Drive visible joints; aerodynamic lift still comes from Wrench."""
        elapsed = now - self.last_visual_update_time
        if elapsed < self.visual_update_period:
            return
        self.last_visual_update_time = now

        joint_client = next(
            (
                client for client in self.joint_clients
                if client.service_is_ready()
            ),
            None,
        )
        if joint_client is None:
            if not self.visual_service_reported:
                self.get_logger().warning(
                    "No set_joint_properties service: lift control continues, "
                    "but visual rotors cannot be driven. Ensure gzserver loads "
                    "libgazebo_ros_properties.so."
                )
                self.visual_service_reported = True
            return

        if self.visual_service_reported:
            self.get_logger().info(
                "Gazebo joint service is ready; visual rotor speed enabled."
            )
            self.visual_service_reported = False

        for index, thrust in enumerate(thrusts):
            pending = self.joint_futures[index]
            if pending is not None and not pending.done():
                continue

            omega = math.sqrt(
                max(0.0, thrust) / max(1.0e-9, self.thrust_coefficient)
            )
            omega = clamp(omega, 0.0, self.max_visual_speed)

            request = SetJointProperties.Request()
            request.joint_name = (
                f"{self.model_name}::rotor_{index}_joint"
            )
            request.ode_joint_config.vel = [
                self.ROTOR_DIRECTIONS[index] * omega
            ]
            # Keep visual joint actuation weak so it does not dominate the
            # explicitly modelled rotor reaction torque.
            request.ode_joint_config.fmax = [self.joint_velocity_fmax]
            self.joint_futures[index] = joint_client.call_async(request)

    def publish_zero(self) -> None:
        message = Wrench()
        for publisher in self.rotor_publishers:
            publisher.publish(message)

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self.publish_zero()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HoverController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
