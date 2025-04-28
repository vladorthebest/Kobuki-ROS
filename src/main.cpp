#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "modes/mode.hpp"
#include "modes/stop_mode.hpp"
#include "modes/manual_mode.hpp"
#include "modes/auto_mode.hpp"

/**
 * @brief Main controller node for Kobuki robot
 * Manages different operation modes and coordinates robot control
 */
class KobukiController : public rclcpp::Node {
public:
    // === INITIALIZATION ===
    KobukiController() : Node("kobuki_controller") {
        // Initialize modes
        stop_mode_ = std::make_unique<StopMode>(this);
        manual_mode_ = std::make_unique<ManualMode>(this);
        auto_mode_ = std::make_unique<AutoMode>(this);
        
        // Set initial mode to STOP
        current_mode_ = stop_mode_.get();
        current_mode_name_ = "STOP";
        
        // Create mode selection subscriber
        mode_sub_ = create_subscription<std_msgs::msg::String>(
            "/mode_select", 10,
            std::bind(&KobukiController::mode_callback, this, std::placeholders::_1));
            
        // Create update timer
        update_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&KobukiController::timer_callback, this));
            
        RCLCPP_INFO(get_logger(), "KobukiController has been started.");
    }

private:
    // === MODE MANAGEMENT ===
    /**
     * @brief Handles mode change requests
     * Switches between STOP, MANUAL, and AUTO modes
     */
    void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string new_mode = msg->data;
        
        if (new_mode == current_mode_name_) return;
        
        Mode* new_mode_ptr = nullptr;
        if (new_mode == "STOP") new_mode_ptr = stop_mode_.get();
        else if (new_mode == "MANUAL") new_mode_ptr = manual_mode_.get();
        else if (new_mode == "AUTO") new_mode_ptr = auto_mode_.get();
        
        if (new_mode_ptr) {
            if (current_mode_) current_mode_->stop();
            current_mode_ = new_mode_ptr;
            current_mode_name_ = new_mode;
            current_mode_->start();
            RCLCPP_INFO(get_logger(), "Mode changed to: %s", new_mode.c_str());
        }
    }

    // === PERIODIC UPDATE ===
    /**
     * @brief Timer callback for periodic mode updates
     * Calls the current mode's update function
     */
    void timer_callback() {
        if (current_mode_) {
            current_mode_->update();
        }
    }

    // === MEMBER VARIABLES ===
    // Mode management
    Mode* current_mode_{nullptr};
    std::string current_mode_name_;
    std::unique_ptr<StopMode> stop_mode_;
    std::unique_ptr<ManualMode> manual_mode_;
    std::unique_ptr<AutoMode> auto_mode_;
    
    // ROS entities
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::TimerBase::SharedPtr update_timer_;
};

// === MAIN FUNCTION ===
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KobukiController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 