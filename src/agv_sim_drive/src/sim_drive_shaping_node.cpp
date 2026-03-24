#include "agv_sim_drive/sim_drive_shaping_node.hpp"

#include <algorithm>
#include <cmath>

namespace agv_sim_drive {

SimDriveShapingNode::SimDriveShapingNode() : Node("sim_drive_shaping_node") {
  // Declare parameters (matching real odrive_params.yaml)
  this->declare_parameter("wheel_radius", 0.0625);
  this->declare_parameter("track_width", 0.735);
  this->declare_parameter("publish_rate_hz", 50);
  this->declare_parameter("cmd_vel_timeout_ms", 500);
  this->declare_parameter("invert_left", false);
  this->declare_parameter("invert_right", false);
  this->declare_parameter("left_scale", 1.0);
  this->declare_parameter("right_scale", 1.0);
  this->declare_parameter("zero_vel_epsilon", 0.03);
  this->declare_parameter("min_effective_vel", 0.0);
  this->declare_parameter("max_wheel_accel", 1.0);

  // Read parameters
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  track_width_ = this->get_parameter("track_width").as_double();
  publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_int();
  cmd_vel_timeout_ms_ = this->get_parameter("cmd_vel_timeout_ms").as_int();
  invert_left_ = this->get_parameter("invert_left").as_bool();
  invert_right_ = this->get_parameter("invert_right").as_bool();
  left_sign_ = invert_left_ ? -1.0 : 1.0;
  right_sign_ = invert_right_ ? -1.0 : 1.0;
  left_scale_ = this->get_parameter("left_scale").as_double();
  right_scale_ = this->get_parameter("right_scale").as_double();
  zero_vel_epsilon_ = static_cast<float>(this->get_parameter("zero_vel_epsilon").as_double());
  min_effective_vel_ = static_cast<float>(this->get_parameter("min_effective_vel").as_double());
  max_wheel_accel_ = static_cast<float>(this->get_parameter("max_wheel_accel").as_double());

  // Validate
  if (wheel_radius_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "wheel_radius must be > 0, got %f", wheel_radius_);
    throw std::runtime_error("Invalid wheel_radius");
  }
  if (track_width_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "track_width must be > 0, got %f", track_width_);
    throw std::runtime_error("Invalid track_width");
  }

  // Publishers & subscribers
  pub_shaped_ = this->create_publisher<geometry_msgs::msg::Twist>("shaped_cmd_vel", 10);
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&SimDriveShapingNode::on_cmd_vel, this, std::placeholders::_1));
  sub_e_stop_ = this->create_subscription<std_msgs::msg::Bool>(
    "e_stop", 10, std::bind(&SimDriveShapingNode::on_e_stop, this, std::placeholders::_1));

  last_cmd_vel_time_ = this->now();

  // Timer at publish_rate_hz
  auto period = std::chrono::milliseconds(1000 / publish_rate_hz_);
  timer_ = this->create_wall_timer(period,
    std::bind(&SimDriveShapingNode::publish_loop, this));

  RCLCPP_INFO(get_logger(),
    "Drive shaping node started: wheel_radius=%.4f, track_width=%.4f, rate=%d Hz",
    wheel_radius_, track_width_, publish_rate_hz_);
  RCLCPP_INFO(get_logger(),
    "Shaping: zero_vel_epsilon=%.4f, max_wheel_accel=%.4f, min_effective_vel=%.4f",
    zero_vel_epsilon_, max_wheel_accel_, min_effective_vel_);
}

void SimDriveShapingNode::on_cmd_vel(const geometry_msgs::msg::Twist& msg) {
  last_cmd_vel_time_ = this->now();
  last_linear_x_ = msg.linear.x;
  last_angular_z_ = msg.angular.z;
  cmd_vel_received_ = true;
}

void SimDriveShapingNode::on_e_stop(const std_msgs::msg::Bool& msg) {
  if (msg.data && !e_stop_active_) {
    RCLCPP_WARN(get_logger(), "E-STOP ACTIVATED — zeroing drive");
    e_stop_active_ = true;
    prev_left_cmd_ = 0.0f;
    prev_right_cmd_ = 0.0f;
  } else if (!msg.data && e_stop_active_) {
    RCLCPP_INFO(get_logger(), "E-stop released");
    e_stop_active_ = false;
  }
}

void SimDriveShapingNode::publish_loop() {
  geometry_msgs::msg::Twist out;

  // E-stop: publish zero
  if (e_stop_active_) {
    prev_left_cmd_ = 0.0f;
    prev_right_cmd_ = 0.0f;
    pub_shaped_->publish(out);
    return;
  }

  // cmd_vel timeout check
  auto elapsed_ms = (this->now() - last_cmd_vel_time_).nanoseconds() / 1000000;
  if (elapsed_ms > cmd_vel_timeout_ms_) {
    prev_left_cmd_ = 0.0f;
    prev_right_cmd_ = 0.0f;
    pub_shaped_->publish(out);
    return;
  }

  // If we haven't received any cmd_vel yet, publish zero
  if (!cmd_vel_received_) {
    pub_shaped_->publish(out);
    return;
  }

  double linear_x = last_linear_x_;
  double angular_z = last_angular_z_;

  // Differential drive inverse kinematics: m/s → turns/s
  double v_left = (linear_x - angular_z * track_width_ / 2.0) / (wheel_radius_ * 2.0 * M_PI);
  double v_right = (linear_x + angular_z * track_width_ / 2.0) / (wheel_radius_ * 2.0 * M_PI);

  // Apply per-wheel shaping (same logic as real odrive_can_node)
  float left_shaped = apply_wheel_shaping(
    static_cast<float>(v_left), prev_left_cmd_, left_sign_ * left_scale_);
  float right_shaped = apply_wheel_shaping(
    static_cast<float>(v_right), prev_right_cmd_, right_sign_ * right_scale_);

  // Convert shaped turns/s back to body-frame Twist (m/s)
  double wl = static_cast<double>(left_shaped) * wheel_radius_ * 2.0 * M_PI;
  double wr = static_cast<double>(right_shaped) * wheel_radius_ * 2.0 * M_PI;
  out.linear.x = (wl + wr) / 2.0;
  out.angular.z = (wr - wl) / track_width_;

  pub_shaped_->publish(out);
}

// Exact port of apply_wheel_shaping from odrive_can_node.cpp
float SimDriveShapingNode::apply_wheel_shaping(
    float target, float& prev_cmd, double sign_and_scale) {
  // Zero-command bypass
  if (std::abs(target) < zero_vel_epsilon_) {
    prev_cmd = 0.0f;
    return 0.0f;
  }

  float vel = target * static_cast<float>(sign_and_scale);

  // Accel limiter
  float dt = 1.0f / static_cast<float>(publish_rate_hz_);
  float max_dv = max_wheel_accel_ * dt;
  vel = prev_cmd + std::clamp(vel - prev_cmd, -max_dv, max_dv);

  // Min effective velocity (stiction compensation)
  if (min_effective_vel_ > 0.0f && std::abs(vel) > 0.001f
      && std::abs(vel) < min_effective_vel_) {
    vel = std::copysign(min_effective_vel_, vel);
  }

  prev_cmd = vel;
  return vel;
}

}  // namespace agv_sim_drive

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_sim_drive::SimDriveShapingNode>());
  rclcpp::shutdown();
  return 0;
}
