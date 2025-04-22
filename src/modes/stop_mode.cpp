#include "modes/stop_mode.hpp"

/**
 * @brief Stop mode implementation
 * Handles graceful deceleration of the robot
 */

// === INITIALIZATION ===
StopMode::StopMode(rclcpp::Node* node) : Mode(node) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&StopMode::odom_callback, this, std::placeholders::_1));
}

// === MODE LIFECYCLE ===
void StopMode::start() {
    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
}

void StopMode::stop() {}

// === ODOMETRY HANDLING ===
void StopMode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    linear_velocity_ = msg->twist.twist.linear.x;
    angular_velocity_ = msg->twist.twist.angular.z;
}

// === UPDATE LOGIC ===
void StopMode::update() {
    geometry_msgs::msg::Twist cmd;
    double decel = 0.05;  // Deceleration rate

    // Apply gradual deceleration
    if (std::abs(linear_velocity_) > 0.01) {
        cmd.linear.x = linear_velocity_ - decel * ((linear_velocity_ > 0) ? 1 : -1);
    }

    if (std::abs(angular_velocity_) > 0.01) {
        cmd.angular.z = angular_velocity_ - decel * ((angular_velocity_ > 0) ? 1 : -1);
    }

    // Publish if still moving
    if (cmd.linear.x != 0.0 || cmd.angular.z != 0.0) {
        cmd_vel_pub_->publish(cmd);
        RCLCPP_INFO(node_->get_logger(), "STOP mode braking -> lin: %.2f ang: %.2f",
                    cmd.linear.x, cmd.angular.z);
    }
} 