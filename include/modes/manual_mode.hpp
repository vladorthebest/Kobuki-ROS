#ifndef MANUAL_MODE_HPP
#define MANUAL_MODE_HPP

#include "modes/mode.hpp"
#include <thread>

class ManualMode : public Mode {
public:
    explicit ManualMode(rclcpp::Node* node);
    ~ManualMode();
    
    void start() override;
    void stop() override;
    void update() override;

private:
    double manual_lin_{0.0};
    double manual_ang_{0.0};
    bool running_{false};
    std::thread keyboard_thread_;
    
    void keyboard_input_loop();
};

#endif // MANUAL_MODE_HPP 