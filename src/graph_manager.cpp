#include "graph_manager.hpp"

GraphManager::GraphManager(rclcpp::Node* node) : node_(node), total_distance_(0.0) {}

void GraphManager::updatePlot() {
    // To be implemented
}

void GraphManager::resetPlot() {
    // To be implemented
}

void GraphManager::trackDistance(double dx, double dy) {
    total_distance_ += std::sqrt(dx*dx + dy*dy);
}

double GraphManager::getTotalDistance() const {
    return total_distance_;
} 