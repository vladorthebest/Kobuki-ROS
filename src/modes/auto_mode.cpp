#include "modes/auto_mode.hpp"
#include <cmath>
#include <algorithm>

/**
 * @brief Autonomous mode implementation
 * Handles wall-following behavior
 */

// === INITIALIZATION ===
AutoMode::AutoMode(rclcpp::Node* node) : Mode(node) {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&AutoMode::scan_callback, this, std::placeholders::_1));
}

// === MODE LIFECYCLE ===
void AutoMode::start() {
    state_ = State::APPROACHING;
}

void AutoMode::stop() {}

// === SENSOR HANDLING ===
void AutoMode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;
}

double AutoMode::get_range(const sensor_msgs::msg::LaserScan::SharedPtr& scan, float angle_deg) {
    if (!scan) return std::numeric_limits<double>::infinity();

    // Convert angle to index
    float angle_rad = angle_deg * M_PI / 180.0;
    int index = static_cast<int>((angle_rad - scan->angle_min) / scan->angle_increment);
    index = std::clamp(index, 0, static_cast<int>(scan->ranges.size()) - 1);

    return scan->ranges[index];
}

// === UPDATE LOGIC ===
void AutoMode::update() {
    if (!last_scan_) return;

    geometry_msgs::msg::Twist cmd;
    double front = get_range(last_scan_, 0.0);
    double side = get_range(last_scan_, 90.0);

    // State machine for wall following
    if (state_ == State::APPROACHING) {
        // Approach until wall is detected
        if (front < 0.5) {
            state_ = State::FOLLOWING;
            RCLCPP_INFO(node_->get_logger(), "AUTO: Switching to FOLLOWING");
        } else {
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            return;
        }
    }

    if (state_ == State::FOLLOWING) {
        double error = 0.0;
        double correction = 0.0;

        // Wall following behavior
        if (side < 2.0) {
            // Calculate correction based on distance error
            error = side - desired_distance_;
            correction = Kp_ * error;

            cmd.linear.x = 0.2;
            cmd.angular.z = std::clamp(correction, -1.0, 1.0);
        } else {
            // Wall lost - search pattern
            cmd.linear.x = 0.15;
            cmd.angular.z = 0.5;
            RCLCPP_WARN(node_->get_logger(), "AUTO: wall lost! Turning left to reacquire");
        }

        cmd_vel_pub_->publish(cmd);
        RCLCPP_INFO(node_->get_logger(), "AUTO: Error: %.2f -> ang.z = %.2f", error, cmd.angular.z);
    }
} 