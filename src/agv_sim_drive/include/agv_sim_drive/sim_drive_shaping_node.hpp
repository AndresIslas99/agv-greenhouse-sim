#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

namespace agv_sim_drive {

class SimDriveShapingNode : public rclcpp::Node {
public:
  SimDriveShapingNode();

private:
  // Parameters
  double wheel_radius_;
  double track_width_;
  int publish_rate_hz_;
  int cmd_vel_timeout_ms_;
  bool invert_left_;
  bool invert_right_;
  double left_sign_;
  double right_sign_;
  double left_scale_;
  double right_scale_;
  float zero_vel_epsilon_;
  float min_effective_vel_;
  float max_wheel_accel_;

  // State
  float prev_left_cmd_{0.0f};
  float prev_right_cmd_{0.0f};
  bool e_stop_active_{false};
  rclcpp::Time last_cmd_vel_time_;

  // Latest cmd_vel
  double last_linear_x_{0.0};
  double last_angular_z_{0.0};
  bool cmd_vel_received_{false};

  // Core shaping function (ported from real odrive_can_node)
  float apply_wheel_shaping(float target, float& prev_cmd, double sign_and_scale);

  // Callbacks
  void on_cmd_vel(const geometry_msgs::msg::Twist& msg);
  void on_e_stop(const std_msgs::msg::Bool& msg);
  void publish_loop();

  // Publishers & subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_shaped_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_e_stop_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agv_sim_drive
