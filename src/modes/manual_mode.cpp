#include "modes/manual_mode.hpp"
#include <termios.h>
#include <unistd.h>

/**
 * @brief Manual mode implementation
 * Handles keyboard-based robot control
 */

// === INITIALIZATION ===
ManualMode::ManualMode(rclcpp::Node* node) : Mode(node) {}

ManualMode::~ManualMode() {
    stop();
}

// === MODE LIFECYCLE ===
void ManualMode::start() {
    running_ = true;
    keyboard_thread_ = std::thread(&ManualMode::keyboard_input_loop, this);
}

void ManualMode::stop() {
    running_ = false;
    if (keyboard_thread_.joinable()) {
        keyboard_thread_.join();
    }
}

// === UPDATE LOGIC ===
void ManualMode::update() {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = manual_lin_;
    cmd.angular.z = manual_ang_;
    cmd_vel_pub_->publish(cmd);
}

// === KEYBOARD CONTROL ===
void ManualMode::keyboard_input_loop() {
    // Setup terminal for raw input
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Process keyboard input
    while (running_ && rclcpp::ok()) {
        char key;
        if (read(STDIN_FILENO, &key, 1) < 0) continue;

        // Handle keyboard commands
        switch (key) {
            case 'w': manual_lin_ += 0.1; break;  // Forward
            case 's': manual_lin_ -= 0.1; break;  // Backward
            case 'a': manual_ang_ += 0.1; break;  // Turn left
            case 'd': manual_ang_ -= 0.1; break;  // Turn right
            case 'x': manual_lin_ = 0.0; manual_ang_ = 0.0; break;  // Stop
            default: break;
        }

        RCLCPP_INFO(node_->get_logger(), "MANUAL input -> lin: %.2f ang: %.2f",
                    manual_lin_, manual_ang_);
    }

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
} 