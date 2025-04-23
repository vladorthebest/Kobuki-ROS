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
    LaserReadings readings{
        get_range(last_scan_, 0.0),    // front
        get_range(last_scan_, 45.0),   // front_left
        get_range(last_scan_, 90.0),   // left
        get_range(last_scan_, -45.0)   // front_right
    };
    return readings;
}

double AutoMode::get_range(const sensor_msgs::msg::LaserScan::SharedPtr& scan, float angle_deg) {
    if (!scan) return std::numeric_limits<double>::infinity();

    float angle_rad = angle_deg * M_PI / 180.0;
    int index = static_cast<int>((angle_rad - scan->angle_min) / scan->angle_increment);
    index = std::clamp(index, 0, static_cast<int>(scan->ranges.size()) - 1);

    return scan->ranges[index];
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
    if (readings.front < WALL_DETECT_DISTANCE) {
        state_ = State::ROTATING;
        RCLCPP_INFO(node_->get_logger(), "Transitioning to ROTATING state");
    } else {
        cmd.linear.x = std::min(MAX_LINEAR_SPEED, 
                               0.5 * (readings.front - WALL_DETECT_DISTANCE));
        cmd.angular.z = 0.0;
    }
}

void AutoMode::handle_rotating_state(const LaserReadings& readings, geometry_msgs::msg::Twist& cmd) {
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

    double linear_speed = approaching_corner ? MAX_LINEAR_SPEED * 0.5 : MAX_LINEAR_SPEED;
    double angular_velocity = Kp_distance * distance_error + Kp_angle * wall_angle;

    cmd.linear.x = std::clamp(linear_speed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED);
    cmd.angular.z = std::clamp(angular_velocity, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    RCLCPP_INFO(node_->get_logger(), 
                "Following - Distance error: %.2f, Wall angle: %.2f", 
                distance_error, wall_angle);
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