#ifndef MODE_HPP
#define MODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Mode {
public:
    explicit Mode(rclcpp::Node* node) : node_(node) {
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
    
    virtual ~Mode() = default;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void update() = 0;

protected:
    rclcpp::Node* node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

#endif // MODE_HPP 