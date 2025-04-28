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
    RCLCPP_INFO(node_->get_logger(), "AUTO mode started: APPROACHING state");
}

void AutoMode::stop() {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd); // Zero by default
}

// === SENSOR HANDLING ===
void AutoMode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;
}

AutoMode::LaserReadings AutoMode::get_laser_readings() const {
    LaserReadings readings;

    if (!last_scan_ || last_scan_->ranges.empty()) {
        // Initialize with infinity if scan is not available
        readings.front = std::numeric_limits<double>::infinity();
        readings.front_left = std::numeric_limits<double>::infinity();
        readings.left = std::numeric_limits<double>::infinity();
        readings.front_right = std::numeric_limits<double>::infinity();
        readings.right = std::numeric_limits<double>::infinity();
        return readings;
    }

    size_t n = last_scan_->ranges.size();

    // Index calculations
    size_t idx_front       = n / 2;
    size_t idx_left        = 3 * n / 4;
    size_t idx_right       = n / 4;
    size_t idx_front_left  = (idx_front + idx_left) / 2;
    size_t idx_front_right = (idx_front + idx_right) / 2;

    // Assign readings
    readings.front       = last_scan_->ranges[idx_front];
    readings.left        = last_scan_->ranges[idx_left];
    readings.right       = last_scan_->ranges[idx_right];
    readings.front_left  = last_scan_->ranges[idx_front_left];
    readings.front_right = last_scan_->ranges[idx_front_right];

    RCLCPP_INFO(node_->get_logger(), 
        "Laser readings (m) - Front: %.2f, Front-Left: %.2f, Left: %.2f, Front-Right: %.2f, Right: %.2f",
        readings.front, readings.front_left, readings.left, readings.front_right, readings.right);

    return readings;
}

bool AutoMode::check_safety(const LaserReadings& readings) const {
    return is_safe_distance(readings.front) && 
           is_safe_distance(readings.left) && 
           is_safe_distance(readings.front_left);
}

double AutoMode::calculate_wall_angle(double front_left, double left) const {
    return std::atan2(front_left - left, last_scan_->angle_increment * 45);
}

bool AutoMode::is_wall_aligned(double front, double left) const {
    return left < WALL_DETECT_DISTANCE && std::abs(front - left) < 0.1;
}

// === UPDATE LOGIC ===
void AutoMode::handle_approaching_state(const LaserReadings& readings, geometry_msgs::msg::Twist& cmd) {
    if (readings.front <= WALL_DETECT_DISTANCE) {
        state_ = State::ROTATING;
        RCLCPP_INFO(node_->get_logger(), "Transitioning to ROTATING state");
    } else {
        cmd.linear.x = MAX_LINEAR_SPEED; 
        cmd.angular.z = 0.0;
    }
}

void AutoMode::handle_rotating_state(const LaserReadings& readings, geometry_msgs::msg::Twist& cmd) {
    RCLCPP_INFO(node_->get_logger(), "ROTATING MODE is running");
    cmd.angular.z = MAX_ANGULAR_SPEED;
    if (is_wall_aligned(readings.front, readings.left)) {
        state_ = State::FOLLOWING;
        RCLCPP_INFO(node_->get_logger(), "Transitioning to FOLLOWING state");
    } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = MAX_ANGULAR_SPEED;
    }
}

void AutoMode::handle_following_state(const LaserReadings& readings, geometry_msgs::msg::Twist& cmd) {
    double wall_angle = calculate_wall_angle(readings.front_left, readings.left);
    double distance_error = readings.left - TARGET_WALL_DISTANCE;

    bool approaching_corner = readings.front < WALL_DETECT_DISTANCE || 
                            readings.front_left < TARGET_WALL_DISTANCE;

    // Always maintain minimum forward velocity
    double linear_speed = approaching_corner ? MAX_LINEAR_SPEED * 0.5 : MAX_LINEAR_SPEED;
    linear_speed = std::max(MIN_COMMAND_SPEED, linear_speed);  // Ensure minimum speed

    double angular_velocity = Kp_distance * distance_error + Kp_angle * wall_angle;
    
    // If angular correction is very small, add a minimum rotation to prevent stalling
    if (std::abs(angular_velocity) < 0.01 && std::abs(distance_error) > 0.01) {
        angular_velocity = (distance_error > 0) ? 0.05 : -0.05;
    }

    cmd.linear.x = std::clamp(linear_speed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED);
    cmd.angular.z = std::clamp(angular_velocity, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    RCLCPP_INFO(node_->get_logger(), 
                "Following - Distance error: %.2f, Wall angle: %.2f, Speed: %.2f, Angular: %.2f", 
                distance_error, wall_angle, cmd.linear.x, cmd.angular.z);
}

void AutoMode::update() {
    if (!last_scan_) return;

    geometry_msgs::msg::Twist cmd;
    LaserReadings readings = get_laser_readings();

    if (!check_safety(readings)) {
        cmd.linear.x = 0.0;
        cmd.angular.z = MAX_ANGULAR_SPEED;
        cmd_vel_pub_->publish(cmd);
        RCLCPP_WARN(node_->get_logger(), "Emergency maneuver activated!");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "AUTO_MODE is running");

    switch (state_) {
        case State::APPROACHING:
            handle_approaching_state(readings, cmd);
            break;
        case State::ROTATING:
            handle_rotating_state(readings, cmd);
            break;
        case State::FOLLOWING:
            handle_following_state(readings, cmd);
            break;
    }

    cmd_vel_pub_->publish(cmd);
} 