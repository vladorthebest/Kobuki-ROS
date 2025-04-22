#ifndef AUTO_MODE_HPP
#define AUTO_MODE_HPP

#include "modes/mode.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

class AutoMode : public Mode {
public:
    explicit AutoMode(rclcpp::Node* node);
    
    void start() override;
    void stop() override;
    void update() override;

private:
    enum class State { APPROACHING, FOLLOWING };
    State state_{State::APPROACHING};
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    double desired_distance_{0.3};
    double Kp_{2.5};
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    double get_range(const sensor_msgs::msg::LaserScan::SharedPtr& scan, float angle_deg);
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

#endif // AUTO_MODE_HPP 