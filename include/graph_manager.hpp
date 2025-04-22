#ifndef GRAPH_MANAGER_HPP
#define GRAPH_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>

class GraphManager {
public:
    explicit GraphManager(rclcpp::Node* node);
    
    // Basic functionality stubs
    void updatePlot();
    void resetPlot();
    void trackDistance(double dx, double dy);
    double getTotalDistance() const;

private:
    rclcpp::Node* node_;
    double total_distance_{0.0};
};

#endif // GRAPH_MANAGER_HPP 