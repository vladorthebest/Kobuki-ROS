#include "kobuki_controller/kobuki_control_node.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

namespace kobuki_controller
{

// === ВСПОМОГАТЕЛЬНАЯ ФУНКЦИЯ ===
double get_range(const sensor_msgs::msg::LaserScan::SharedPtr& scan, float angle_deg)
{
  if (!scan) return std::numeric_limits<double>::infinity();

  float angle_rad = angle_deg * M_PI / 180.0;
  int index = static_cast<int>((angle_rad - scan->angle_min) / scan->angle_increment);
  index = std::clamp(index, 0, static_cast<int>(scan->ranges.size()) - 1);

  return scan->ranges[index];
}

// === КОНСТРУКТОР НОДЫ ===
KobukiControlNode::KobukiControlNode()
: Node("kobuki_control_node"),
  current_mode_("STOP"),
  linear_velocity_(0.0),
  angular_velocity_(0.0),
  manual_lin_(0.0),
  manual_ang_(0.0),
  auto_state_(AutoState::APPROACHING),
  desired_distance_(0.3),
  Kp_(2.5)
{
  mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/mode_select", 10, std::bind(&KobukiControlNode::mode_callback, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&KobukiControlNode::odom_callback, this, _1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&KobukiControlNode::scan_callback, this, _1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&KobukiControlNode::timer_callback, this));

  std::thread(&KobukiControlNode::keyboard_input_loop, this).detach();

  RCLCPP_INFO(this->get_logger(), "KobukiControlNode has been started.");
}

// === ОБРАБОТКА РЕЖИМА ===
void KobukiControlNode::mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  current_mode_ = msg->data;

  if (current_mode_ == "AUTO") {
    auto_state_ = AutoState::APPROACHING;
  }

  RCLCPP_INFO(this->get_logger(), "Mode changed to: %s", current_mode_.c_str());
}

// === ОБРАБОТКА ОДОМЕТРИИ ===
void KobukiControlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  linear_velocity_ = msg->twist.twist.linear.x;
  angular_velocity_ = msg->twist.twist.angular.z;
}

// === ОБРАБОТКА ЛАЗЕРА ===
void KobukiControlNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  last_scan_ = msg;
}

// === ГЛАВНЫЙ ЦИКЛ ===
void KobukiControlNode::timer_callback()
{
  geometry_msgs::msg::Twist cmd;

  if (current_mode_ == "STOP") {
    double decel = 0.05;

    if (std::abs(linear_velocity_) > 0.01) {
      cmd.linear.x = linear_velocity_ - decel * ((linear_velocity_ > 0) ? 1 : -1);
    }

    if (std::abs(angular_velocity_) > 0.01) {
      cmd.angular.z = angular_velocity_ - decel * ((angular_velocity_ > 0) ? 1 : -1);
    }

    if (cmd.linear.x != 0.0 || cmd.angular.z != 0.0) {
      cmd_vel_pub_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "STOP mode braking -> lin: %.2f ang: %.2f",
                  cmd.linear.x, cmd.angular.z);
    }

  } else if (current_mode_ == "MANUAL") {
    cmd.linear.x = manual_lin_;
    cmd.angular.z = manual_ang_;
    cmd_vel_pub_->publish(cmd);
  }

  else if (current_mode_ == "AUTO") {
    if (!last_scan_) return;

    double front = get_range(last_scan_, 0.0);
    double side  = get_range(last_scan_, 90.0);

    if (auto_state_ == AutoState::APPROACHING) {
      if (front < 0.5) {
        auto_state_ = AutoState::FOLLOWING;
        RCLCPP_INFO(this->get_logger(), "AUTO: Switching to FOLLOWING");
      } else {
        cmd.linear.x = 0.2;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
        return;
      }
    }

    if (auto_state_ == AutoState::FOLLOWING) {
  double error = 0.0;
  double correction = 0.0;

  // Проверяем, "видим" ли стену сбоку
  if (side < 2.0) {
    error = side - desired_distance_;
    correction = Kp_ * error;

    cmd.linear.x = 0.2;
    cmd.angular.z = std::clamp(correction, -1.0, 1.0);
  } else {
    // Стены нет — поворачиваем влево, чтобы её "найти"
    cmd.linear.x = 0.15;
    cmd.angular.z = 0.5;  // Плавный поворот
    RCLCPP_WARN(this->get_logger(), "AUTO: wall lost! Turning left to reacquire");
  }

  cmd_vel_pub_->publish(cmd);

  RCLCPP_INFO(this->get_logger(), "AUTO: Error: %.2f -> ang.z = %.2f", error, cmd.angular.z);
}


  }
}

// === КЛАВИАТУРНОЕ УПРАВЛЕНИЕ ===
void KobukiControlNode::keyboard_input_loop()
{
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  char key;
  while (rclcpp::ok()) {
    if (read(STDIN_FILENO, &key, 1) < 0) continue;

    if (current_mode_ == "MANUAL") {
      switch (key) {
        case 'w': manual_lin_ += 0.1; break;
        case 's': manual_lin_ -= 0.1; break;
        case 'a': manual_ang_ += 0.1; break;
        case 'd': manual_ang_ -= 0.1; break;
        case 'x': manual_lin_ = 0.0; manual_ang_ = 0.0; break;
        default: break;
      }

      RCLCPP_INFO(this->get_logger(), "MANUAL input -> lin: %.2f ang: %.2f",
                  manual_lin_, manual_ang_);
    }
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

}  // namespace kobuki_controller

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kobuki_controller::KobukiControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
