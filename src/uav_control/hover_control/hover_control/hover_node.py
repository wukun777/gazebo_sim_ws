#!/usr/bin/env python3
"""向ROS 2电机—旋翼动力学插件发送四路目标角速度。

修改说明：
1. 删除gz topic子进程调用。
2. 删除PX4的mav_msgs、libmav_msgs.so和Gazebo Transport接口。
3. 改为发布std_msgs/msg/Float64MultiArray。
4. 一条ROS 2消息同时包含四个旋翼目标转速。
5. 本节点不计算升力、不读取飞行状态，也不包含PID。
6. F = motor_constant * omega^2 由Gazebo C++插件计算。
"""

import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray


class MotorSpeedTakeoff(Node):
    """开环四旋翼转速指令节点，不包含飞行闭环控制。"""

    def __init__(self) -> None:
        super().__init__("roarm_quad_motor_speed")

        # 【修改1】四个旋翼的固定目标角速度，单位rad/s。
        #
        # rotor_0：右前 FR，CCW
        # rotor_1：左后 BL，CCW
        # rotor_2：左前 FL，CW
        # rotor_3：右后 BR，CW
        self.declare_parameter(
            "motor_speeds",
            [105.89, 127.61, 104.10, 129.12],
        )

        # 插件的ROS 2转速话题。
        self.declare_parameter(
            "motor_topic",
            "/roarm_quad/motor_speed_cmd",
        )

        # 插件连接成功后，等待一定时间再启动旋翼。
        self.declare_parameter("arm_delay", 1.0)

        # 20Hz发送。必须明显小于SDF中的0.5秒command_timeout。
        self.declare_parameter("publish_period", 0.02)

        self.motor_speeds = [
            float(value)
            for value in self.get_parameter("motor_speeds").value
        ]

        # omega²插件参数，必须与model.sdf保持一致。
        self.motor_constant = 8.0e-4
        self.max_omega = 220.0

        # 姿态内环PD参数。
        self.kp_roll = 0.8
        self.kd_roll = 0.25

        self.kp_pitch = 1.5
        self.kd_pitch = 0.45

        self.kp_yaw = 0.25
        self.kd_yaw = 0.15

        # 姿态控制最大修正力矩。
        self.max_roll_torque = 0.5
        self.max_pitch_torque = 0.7
        self.max_yaw_torque = 0.05

        if len(self.motor_speeds) != 4:
            raise ValueError(
                "motor_speeds必须正好包含4个目标角速度"
            )

        if any(value < 0.0 for value in self.motor_speeds):
            raise ValueError(
                "motor_speeds应使用非负角速度幅值"
            )

        if any(value > 220.0 for value in self.motor_speeds):
            raise ValueError(
                "motor_speeds不能超过SDF设置的220 rad/s"
            )

        self.motor_topic = str(
            self.get_parameter("motor_topic").value
        )

        self.arm_delay = max(
            0.0,
            float(self.get_parameter("arm_delay").value),
        )

        publish_period = float(
            self.get_parameter("publish_period").value
        )

        # 防止发布周期过长，触发插件0.5秒的指令超时。
        publish_period = min(max(publish_period, 0.02), 0.20)

        # 【修改2】与C++插件相匹配的ROS 2 QoS。
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 【修改3】真正的ROS 2发布者。
        self.motor_publisher = self.create_publisher(
            Float64MultiArray,
            self.motor_topic,
            qos,
        )
        
        # 当前姿态。
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        
        self.debug_counter = 0
        

        # 当前角速度。
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0

        # 第一次收到状态后，将当时的yaw作为保持目标。
        self.target_yaw = 0.0
        self.state_received = False

        # 订阅Gazebo模型状态。
        self.state_subscription = self.create_subscription(
            ModelStates,
            "/model_states",
            self.state_callback,
            10,
        )
        
        

        self.plugin_connected = False
        self.connection_time = None
        self.takeoff_command_started = False
        self.waiting_message_printed = False

        self.timer = self.create_timer(
            publish_period,
            self.timer_callback,
        )

        self.get_logger().info(
            "Motor-speed node started with attitude PD feedback; "
            "no Wrench is published."
        )

        self.get_logger().info(
            f"Waiting for motor plugin on {self.motor_topic}"
        )

        self.get_logger().info(
            "Target omega [rad/s] = "
            + str([
                round(value, 2)
                for value in self.motor_speeds
            ])
        )

    
    def state_callback(self, message: ModelStates) -> None:
        """从/model_states读取roarm_quad姿态。"""

        try:
            index = message.name.index("roarm_quad")
        except ValueError:
            return

        pose = message.pose[index]
        twist = message.twist[index]
        
        self.x = pose.position.x
        self.y = pose.position.y
        self.z = pose.position.z

        self.linear_x = twist.linear.x
        self.linear_y = twist.linear.y
        self.linear_z = twist.linear.z
        

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        # 四元数转换为roll。
        self.roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )

        # 四元数转换为pitch。
        sin_pitch = 2.0 * (qw * qy - qz * qx)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))
        self.pitch = math.asin(sin_pitch)

        # 四元数转换为yaw。
        self.yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        # 第一阶段飞机接近水平、初始yaw接近0，
        # 暂时直接使用/model_states的三个角速度分量。
        self.angular_x = twist.angular.x
        self.angular_y = twist.angular.y
        self.angular_z = twist.angular.z

        if not self.state_received:
            self.target_yaw = self.yaw
            self.state_received = True

            self.get_logger().info(
                "Initial attitude received: "
                f"roll={math.degrees(self.roll):.2f} deg, "
                f"pitch={math.degrees(self.pitch):.2f} deg, "
                f"yaw={math.degrees(self.yaw):.2f} deg."
            )
            
            
        self.debug_counter += 1
        
        if self.debug_counter >= 50:
            self.debug_counter = 0

            self.get_logger().info(
                f"x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, "
                f"vx={self.linear_x:.2f}, vy={self.linear_y:.2f}, "
                f"roll={math.degrees(self.roll):.2f} deg, "
                f"pitch={math.degrees(self.pitch):.2f} deg"
    )


        
    
    def publish_motor_speeds(
        self,
        speeds: List[float],
    ) -> None:
        """发布四个旋翼目标角速度。"""

        message = Float64MultiArray()
        message.data = [float(value) for value in speeds]
        self.motor_publisher.publish(message)
        
    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        """数值限幅。"""

        return max(minimum, min(maximum, value))


    @staticmethod
    def wrap_angle(angle: float) -> float:
        """把角度误差限制在[-pi, pi]。"""

        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )


    def calculate_stabilized_speeds(self) -> List[float]:
        """在原有配平转速上加入姿态PD修正。"""

        # 目标roll和pitch均为0。
        roll_error = -self.roll
        pitch_error = -self.pitch

        # yaw保持第一次收到状态时的方向。
        yaw_error = self.wrap_angle(
            self.target_yaw - self.yaw
        )

        # 姿态PD输出三个目标力矩。
        torque_x = (
            self.kp_roll * roll_error
            - self.kd_roll * self.angular_x
        )

        torque_y = (
            self.kp_pitch * pitch_error
            - self.kd_pitch * self.angular_y
        )

        torque_z = (
            self.kp_yaw * yaw_error
            - self.kd_yaw * self.angular_z
        )

        # 限制最大修正，避免第一次测试突然翻转。
        torque_x = self.clamp(
            torque_x,
            -self.max_roll_torque,
            self.max_roll_torque,
        )

        torque_y = self.clamp(
            torque_y,
            -self.max_pitch_torque,
            self.max_pitch_torque,
        )

        torque_z = self.clamp(
            torque_z,
            -self.max_yaw_torque,
            self.max_yaw_torque,
        )

        # 原来的四个配平转速先转换为基础升力。
        base_forces = [
            self.motor_constant * omega * omega
            for omega in self.motor_speeds
        ]

        # 根据当前模型的旋翼位置和质心，
        # 将roll、pitch、yaw力矩分配到四个旋翼。
        delta_forces = [
            (
                -1.32187248 * torque_x
                -1.32013774 * torque_y
                -16.58967309 * torque_z
            ),
            (
                1.32187248 * torque_x
                +1.32013774 * torque_y
                -16.74366025 * torque_z
            ),
            (
                1.31493352 * torque_x
                -1.31146403 * torque_y
                +16.76505538 * torque_z
            ),
            (
                -1.31493352 * torque_x
                +1.31146403 * torque_y
                +16.56827795 * torque_z
            ),
        ]

        max_force = (
            self.motor_constant
            * self.max_omega
            * self.max_omega
        )

        corrected_speeds = []

        for base_force, delta_force in zip(
            base_forces,
            delta_forces,
        ):
            target_force = self.clamp(
                base_force + delta_force,
                0.0,
                max_force,
            )

            target_omega = math.sqrt(
                target_force / self.motor_constant
            )

            corrected_speeds.append(target_omega)

        return corrected_speeds
    
    

    def timer_callback(self) -> None:
        """等待插件连接，延时解锁，然后持续发布目标转速。"""

        subscription_count = (
            self.motor_publisher.get_subscription_count()
        )

        # 【修改4】先确认Gazebo插件已经成为订阅者。
        if subscription_count == 0:
            if not self.waiting_message_printed:
                self.get_logger().warning(
                    "Motor plugin has not subscribed yet; "
                    "keeping motors stopped."
                )
                self.waiting_message_printed = True
            return

        if not self.plugin_connected:
            self.plugin_connected = True
            self.connection_time = time.monotonic()

            self.get_logger().info(
                "Motor plugin subscription detected."
            )

        elapsed = time.monotonic() - self.connection_time

        # 解锁等待阶段持续发送0，防止电机突然启动。
        if elapsed < self.arm_delay:
            self.publish_motor_speeds(
                [0.0, 0.0, 0.0, 0.0]
            )
            return

        # 没有姿态反馈时不启动。
        if not self.state_received:
            self.publish_motor_speeds(
                [0.0, 0.0, 0.0, 0.0]
            )
            return

        # 在原来的配平转速上加入姿态内环修正。
        stabilized_speeds = (
            self.calculate_stabilized_speeds()
        )

        self.publish_motor_speeds(stabilized_speeds)

        if not self.takeoff_command_started:
            self.takeoff_command_started = True
            self.get_logger().info(
                "Four motor-speed commands are now being published."
            )
            self.get_logger().info(
                "Lift is calculated inside "
                "libroarm_quad_motor_model.so using F=kf*omega^2."
            )

    def stop_motors(self) -> None:
        """退出节点前连续发送几次零转速。"""

        if not rclpy.ok():
            return

        self.get_logger().info("Stopping all four motors.")

        for _ in range(5):
            self.publish_motor_speeds(
                [0.0, 0.0, 0.0, 0.0]
            )
            time.sleep(0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorSpeedTakeoff()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
