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

    // Index calculations
    int idx_front       = std::round((0.0 - last_scan_->angle_min) / last_scan_->angle_increment);
    int idx_left        = std::floor((M_PI/2.0 - last_scan_->angle_min) / last_scan_->angle_increment);
    int idx_right       = std::floor(((2.0 * M_PI - M_PI / 2.0) - last_scan_->angle_min) / last_scan_->angle_increment);
    int idx_front_left  = std::floor((M_PI/4.0 - last_scan_->angle_min) / last_scan_->angle_increment);
    int idx_front_right = std::floor(((2.0 * M_PI - M_PI / 4.0) - last_scan_->angle_min) / last_scan_->angle_increment);
    int idx_back_left   = std::floor((M_PI/4.0 + M_PI - last_scan_->angle_min) / last_scan_->angle_increment);
    int idx_back_right  = std::floor(((2.0 * M_PI + M_PI - M_PI / 4.0) - last_scan_->angle_min) / last_scan_->angle_increment);
    int idx_back       = std::floor((M_PI - last_scan_->angle_min) / last_scan_->angle_increment);
    
    // Assign readings
    readings.left        = last_scan_->ranges[idx_left];
    readings.right       = last_scan_->ranges[idx_right];
    readings.front_left  = last_scan_->ranges[idx_front_left];
    readings.front_right = last_scan_->ranges[idx_front_right];
    readings.back_left  = last_scan_->ranges[idx_back_left];
    readings.back_right = last_scan_->ranges[idx_back_right];
    readings.front       = last_scan_->ranges[idx_front];
    readings.back        = last_scan_->ranges[idx_back];

    RCLCPP_INFO(node_->get_logger(), 
        "Laser readings (m) - Front: %.2f, Front-Left: %.2f, Left: %.2f, Front-Right: %.2f, Right: %.2f",
        readings.front, readings.front_left, readings.left, readings.front_right, readings.right);

    return readings;
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
    float target = readings.right * std::sqrt(2.0f) - 0.05f;
    RCLCPP_INFO(node_->get_logger(), "ROTATING MODE is running");
    if (std::fabs(readings.front_right - target) < 0.02f && readings.front >= WALL_DETECT_DISTANCE) {
        state_ = State::FOLLOWING;
        RCLCPP_INFO(node_->get_logger(), "Transitioning to FOLLOWING state");
    } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = MAX_ANGULAR_SPEED;
    }
}

void AutoMode::handle_following_state(const LaserReadings& readings, geometry_msgs::msg::Twist& cmd) {
    if (readings.front <= WALL_DETECT_DISTANCE) {
        state_ = State::ROTATING;
        RCLCPP_INFO(node_->get_logger(), "Transitioning to ROTATING state");
        return;
    }

    if (readings.right > WALL_DETECT_DISTANCE) {
        bufDistance = readings.back;
        state_ = State::TURNING_CORNER;
        RCLCPP_INFO(node_->get_logger(), "Transitioning to TURNING_CORNER state");
        return;
    }
    cmd.linear.x = MAX_LINEAR_SPEED;
    cmd.angular.z = 0.0;
}

void AutoMode::handle_turning_corner_state(const LaserReadings& readings, geometry_msgs::msg::Twist& cmd) {
    if (bufDistance != 0 && bufDistance - readings.back <= WALL_DETECT_DISTANCE) {
        cmd.linear.x = MAX_LINEAR_SPEED;
        cmd.angular.z = 0.0;
        return;
    }

    double back_right_normal = readings.back_right * std::sqrt(2.0) / 2.0;
    if(back_right_normal > WALL_DETECT_DISTANCE) {
        cmd.linear.x = 0.0;
        cmd.angular.z = MAX_ANGULAR_SPEED;
        return;
    }
    
    if (readings.right > WALL_DETECT_DISTANCE) {
        cmd.linear.x = MAX_LINEAR_SPEED;
        cmd.angular.z = 0.0;
        return;
    }

    state_ = State::FOLLOWING;
}


void AutoMode::update() {
    if (!last_scan_) return;

    geometry_msgs::msg::Twist cmd;
    LaserReadings readings = get_laser_readings();

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
        case State::TURNING_CORNER:
            handle_turning_corner_state(readings, cmd);
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown state!");
            break;
    }

    cmd_vel_pub_->publish(cmd);
}