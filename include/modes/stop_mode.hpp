#ifndef STOP_MODE_HPP
#define STOP_MODE_HPP

#include "modes/mode.hpp"
#include <nav_msgs/msg/odometry.hpp>

class StopMode : public Mode {
public:
    explicit StopMode(rclcpp::Node* node);
    
    void start() override;
    void stop() override;
    void update() override;

private:
    double linear_velocity_{0.0};
    double angular_velocity_{0.0};
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

#endif // STOP_MODE_HPP 