// RoArm quadrotor motor model for ROS 2 + Gazebo Classic.
//
// 【核心因果链】
// std_msgs/Float64MultiArray目标转速
//   -> 一阶电机响应
//   -> 旋翼关节实际旋转
//   -> F = motor_constant * omega^2
//   -> 旋翼link升力 + 机身反扭矩
//
// 本插件不读取无人机位置/姿态，不含高度或姿态PID，也不接收Wrench。

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace roarm_gazebo_plugins
{

class RoarmQuadMotorModel : public gazebo::ModelPlugin
{
public:
  RoarmQuadMotorModel() = default;

  ~RoarmQuadMotorModel() override
  {
    update_connection_.reset();
  }

  void Load(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // 【修改1】从SDF读取电机模型参数，而不是最大直接升力。
    body_link_name_ = ReadOrDefault<std::string>(
      sdf, "body_link", "base_link");
    command_topic_ = ReadOrDefault<std::string>(
      sdf, "command_topic", "motor_speed_cmd");
    command_timeout_ = ReadOrDefault<double>(
      sdf, "command_timeout", 0.50);
    motor_constant_ = ReadOrDefault<double>(
      sdf, "motor_constant", 8.0e-4);
    moment_constant_ = ReadOrDefault<double>(
      sdf, "moment_constant", 0.015);
    time_constant_up_ = ReadOrDefault<double>(
      sdf, "time_constant_up", 0.15);
    time_constant_down_ = ReadOrDefault<double>(
      sdf, "time_constant_down", 0.25);
    max_rot_velocity_ = ReadOrDefault<double>(
      sdf, "max_rot_velocity", 220.0);
    rotor_velocity_slowdown_sim_ = ReadOrDefault<double>(
      sdf, "rotor_velocity_slowdown_sim", 10.0);

    if (
      motor_constant_ <= 0.0 ||
      moment_constant_ < 0.0 ||
      time_constant_up_ <= 0.0 ||
      time_constant_down_ <= 0.0 ||
      max_rot_velocity_ <= 0.0 ||
      rotor_velocity_slowdown_sim_ <= 0.0)
    {
      RCLCPP_FATAL(
        ros_node_->get_logger(),
        "Invalid motor parameters in model.sdf");
      return;
    }

    body_link_ = model_->GetLink(body_link_name_);
    if (!body_link_) {
      RCLCPP_FATAL(
        ros_node_->get_logger(),
        "Motor plugin: body link '%s' does not exist",
        body_link_name_.c_str());
      return;
    }

    if (model_->IsStatic()) {
      RCLCPP_FATAL(
        ros_node_->get_logger(),
        "Motor plugin: model '%s' is static",
        model_->GetName().c_str());
      return;
    }

    // 【修改2】取得四个真实旋翼link和旋翼joint。
    for (std::size_t i = 0; i < rotor_count_; ++i) {
      const std::string index = std::to_string(i);
      const std::string link_name = ReadOrDefault<std::string>(
        sdf, "rotor_" + index + "_link", "rotor_" + index);
      const std::string joint_name = ReadOrDefault<std::string>(
        sdf,
        "rotor_" + index + "_joint",
        "rotor_" + index + "_joint");

      rotor_links_[i] = model_->GetLink(link_name);
      rotor_joints_[i] = model_->GetJoint(joint_name);

      if (!rotor_links_[i] || !rotor_joints_[i]) {
        RCLCPP_FATAL(
          ros_node_->get_logger(),
          "Motor plugin: rotor_%zu link '%s' or joint '%s' not found",
          i, link_name.c_str(), joint_name.c_str());
        return;
      }
    }

    ReadRotorDirections(sdf);

    // 【修改3】一个ROS 2话题一次发送4个目标角速度，单位rad/s。
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    motor_command_subscription_ =
      ros_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_,
      qos,
      std::bind(
        &RoarmQuadMotorModel::OnMotorCommand,
        this,
        std::placeholders::_1));

    // 诊断话题：用于确认插件内部的实际电机转速。
    actual_speed_publisher_ =
      ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "actual_motor_speed", 10);
    thrust_publisher_ =
      ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "rotor_thrust", 10);

    const gazebo::common::Time now = world_->SimTime();
    last_command_time_ = now;
    last_update_time_ = now;
    last_publish_time_ = now;

    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(
        &RoarmQuadMotorModel::OnUpdate,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Omega-squared motor plugin loaded: topic=%s, "
      "kf=%.6g, km=%.6g, max_omega=%.1f rad/s",
      command_topic_.c_str(),
      motor_constant_,
      moment_constant_,
      max_rot_velocity_);
  }

private:
  static constexpr std::size_t rotor_count_ = 4;

  template<typename T>
  T ReadOrDefault(
    const sdf::ElementPtr & sdf,
    const std::string & name,
    const T & default_value)
  {
    if (!sdf->HasElement(name)) {
      return default_value;
    }
    return sdf->GetElement(name)->Get<T>();
  }

  void ReadRotorDirections(const sdf::ElementPtr & sdf)
  {
    // rotor_0、rotor_1为CCW；rotor_2、rotor_3为CW。
    rotor_directions_ = {1.0, 1.0, -1.0, -1.0};
    if (!sdf->HasElement("rotor_directions")) {
      return;
    }

    std::istringstream stream(
      sdf->GetElement("rotor_directions")->Get<std::string>());
    for (double & direction : rotor_directions_) {
      if (!(stream >> direction)) {
        RCLCPP_WARN(
          ros_node_->get_logger(),
          "Invalid rotor_directions; using [1 1 -1 -1]");
        rotor_directions_ = {1.0, 1.0, -1.0, -1.0};
        return;
      }
      direction = direction >= 0.0 ? 1.0 : -1.0;
    }
  }

  // 【修改4】输入值是四个omega，不是四个Wrench/升力。
  void OnMotorCommand(
    const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (message->data.size() != rotor_count_) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "motor_speed_cmd needs exactly 4 values, received %zu",
        message->data.size());
      return;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    for (std::size_t i = 0; i < rotor_count_; ++i) {
      double omega = message->data[i];
      if (!std::isfinite(omega)) {
        omega = 0.0;
      }
      commanded_omega_[i] =
        std::clamp(omega, 0.0, max_rot_velocity_);
    }
    last_command_time_ = world_->SimTime();

    if (!received_command_) {
      received_command_ = true;
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "First motor-speed command received: "
        "[%.2f %.2f %.2f %.2f] rad/s",
        commanded_omega_[0], commanded_omega_[1],
        commanded_omega_[2], commanded_omega_[3]);
    }
  }

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    double dt = (info.simTime - last_update_time_).Double();
    last_update_time_ = info.simTime;
    if (dt <= 0.0) {
      return;
    }
    dt = std::min(dt, 0.1);

    std::array<double, rotor_count_> target_omega{};
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      target_omega = commanded_omega_;

      const double command_age =
        (info.simTime - last_command_time_).Double();
      if (
        !received_command_ ||
        command_age < 0.0 ||
        command_age > command_timeout_)
      {
        target_omega.fill(0.0);
      }
    }

    // 【修改5】官方算法的核心：
    // 目标omega -> 一阶响应 -> 关节转动 -> kf*omega^2升力。
    for (std::size_t i = 0; i < rotor_count_; ++i) {
      const double time_constant =
        target_omega[i] >= actual_omega_[i] ?
        time_constant_up_ : time_constant_down_;

      const double alpha =
        1.0 - std::exp(-dt / time_constant);
      actual_omega_[i] +=
        alpha * (target_omega[i] - actual_omega_[i]);

      // 为避免Gazebo数值混叠，显示/关节转速缩小slowdown倍。
      // 计算升力时使用未缩小的物理actual_omega。
      const double joint_velocity =
        rotor_directions_[i] *
        actual_omega_[i] /
        rotor_velocity_slowdown_sim_;
      rotor_joints_[i]->SetVelocity(0, joint_velocity);

      // F_i = k_f * omega_i^2
      const double thrust =
        motor_constant_ *
        actual_omega_[i] *
        actual_omega_[i];
      rotor_thrusts_[i] = thrust;

      // 升力施加在对应旋翼link的局部+Z轴，不直接给base_link总升力。
      rotor_links_[i]->SetEnabled(true);
      rotor_links_[i]->AddRelativeForce(
        ignition::math::Vector3d(0.0, 0.0, thrust));

      // 旋翼空气阻力矩反作用到机身，方向与旋翼转向相反。
      const double reaction_torque =
        -rotor_directions_[i] *
        moment_constant_ *
        thrust;
      body_link_->AddRelativeTorque(
        ignition::math::Vector3d(
          0.0, 0.0, reaction_torque));
    }

    PublishDiagnostics(info.simTime);
  }

  void PublishDiagnostics(const gazebo::common::Time & sim_time)
  {
    const double age =
      (sim_time - last_publish_time_).Double();
    if (age >= 0.0 && age < 0.1) {
      return;
    }

    std_msgs::msg::Float64MultiArray speed_message;
    speed_message.data.assign(
      actual_omega_.begin(), actual_omega_.end());
    actual_speed_publisher_->publish(speed_message);

    std_msgs::msg::Float64MultiArray thrust_message;
    thrust_message.data.assign(
      rotor_thrusts_.begin(), rotor_thrusts_.end());
    thrust_publisher_->publish(thrust_message);

    last_publish_time_ = sim_time;
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr body_link_;
  std::array<gazebo::physics::LinkPtr, rotor_count_> rotor_links_;
  std::array<gazebo::physics::JointPtr, rotor_count_> rotor_joints_;
  gazebo::event::ConnectionPtr update_connection_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
  motor_command_subscription_;
  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
  actual_speed_publisher_;
  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
  thrust_publisher_;

  std::string body_link_name_{"base_link"};
  std::string command_topic_{"motor_speed_cmd"};

  double command_timeout_{0.50};
  double motor_constant_{8.0e-4};
  double moment_constant_{0.015};
  double time_constant_up_{0.15};
  double time_constant_down_{0.25};
  double max_rot_velocity_{220.0};
  double rotor_velocity_slowdown_sim_{10.0};

  std::array<double, rotor_count_> rotor_directions_{
    1.0, 1.0, -1.0, -1.0};
  std::array<double, rotor_count_> commanded_omega_{
    0.0, 0.0, 0.0, 0.0};
  std::array<double, rotor_count_> actual_omega_{
    0.0, 0.0, 0.0, 0.0};
  std::array<double, rotor_count_> rotor_thrusts_{
    0.0, 0.0, 0.0, 0.0};

  bool received_command_{false};
  std::mutex command_mutex_;
  gazebo::common::Time last_command_time_;
  gazebo::common::Time last_update_time_;
  gazebo::common::Time last_publish_time_;
};

GZ_REGISTER_MODEL_PLUGIN(RoarmQuadMotorModel)

}  // namespace roarm_gazebo_plugins
