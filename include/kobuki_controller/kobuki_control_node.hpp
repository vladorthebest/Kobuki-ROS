#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>  // ДЛЯ AUTO

#include <thread>

namespace kobuki_controller
{

class KobukiControlNode : public rclcpp::Node
{
public:
  KobukiControlNode();  // Конструктор

private:
  // === CALLBACK'И ===
  void mode_callback(const std_msgs::msg::String::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);  // ДЛЯ AUTO
  void timer_callback();
  void keyboard_input_loop();

  // === ROS ENTITIES ===
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;  // ДЛЯ AUTO
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // === СОСТОЯНИЯ ===
  std::string current_mode_;

  // STOP
  double linear_velocity_;
  double angular_velocity_;

  // MANUAL
  double manual_lin_;
  double manual_ang_;

  // AUTO
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  enum class AutoState { APPROACHING, FOLLOWING };
  AutoState auto_state_;
  double desired_distance_;
  double Kp_;
};

}  // namespace kobuki_controller
