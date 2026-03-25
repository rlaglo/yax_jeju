#include <algorithm>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <cmath>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarLaneNode : public rclcpp::Node{
public:
    LidarLaneNode() : Node("lidar_lane_node"){
        auto qos_profile = rclcpp::QoS(10).best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::laser_scan>(
            "/scan", qos_profile, std::bind(&LidarLaneNode::scan_callback, this, std::placeholders::_1);
        )
    }
}