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
    // States for the wall-following behavior
    enum class State { 
        APPROACHING,  // Initial approach to wall
        ROTATING,     // Aligning with wall
        FOLLOWING     // Following the wall
    };

    // Control parameters
    static constexpr double WALL_DETECT_DISTANCE = 0.5;    // Distance to detect wall
    static constexpr double TARGET_WALL_DISTANCE = 0.3;    // Desired distance from wall
    static constexpr double SAFETY_DISTANCE = 0.2;         // Minimum safe distance
    static constexpr double MAX_LINEAR_SPEED = 0.2;        // Maximum forward speed
    static constexpr double MIN_LINEAR_SPEED = 0.1;        // Minimum forward speed
    static constexpr double MAX_ANGULAR_SPEED = 0.5;       // Maximum rotation speed
    static constexpr double MIN_COMMAND_SPEED = 0.05;      // Minimum speed to overcome friction
    static constexpr double Kp_distance = 2.0;             // Distance control gain
    static constexpr double Kp_angle = 1.5;                // Angle control gain
    // State variables
    State state_{State::APPROACHING};
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    
    // Sensor readings structure
    struct LaserReadings {
        double front;
        double front_left;
        double left;
        double front_right;
        double right;
    };

    // Helper methods for state management
    void handle_approaching_state(const LaserReadings& readings, geometry_msgs::msg::Twist& cmd);
    void handle_rotating_state(const LaserReadings& readings, geometry_msgs::msg::Twist& cmd);
    void handle_following_state(const LaserReadings& readings, geometry_msgs::msg::Twist& cmd);
    
    // Helper methods for calculations
    LaserReadings get_laser_readings() const;
    bool check_safety(const LaserReadings& readings) const;
    double calculate_wall_angle(double front_left, double left) const;
    bool is_wall_aligned(double front, double left) const;
    
    // Basic helper functions
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    bool is_safe_distance(double distance) const { return distance > SAFETY_DISTANCE; }
    
    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Angular constants (in radians)
    static constexpr double FRONT_ANGLE = 0.0;
    static constexpr double LEFT_ANGLE = M_PI / 2.0;        // 90°
    static constexpr double FRONT_LEFT_ANGLE = M_PI / 4.0;  // 45°
    static constexpr double FRONT_RIGHT_ANGLE = -M_PI / 4.0; // -45°
};

#endif // AUTO_MODE_HPP 