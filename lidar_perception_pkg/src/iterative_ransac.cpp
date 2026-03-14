#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>

class LidarLaneNode : public rclcpp::Node {
public:
    LidarLaneNode() : Node("lidar_lane_node") {
        auto qos_profile = rclcpp::QoS(10).best_effort();
        
        // create_publisher 함수가 실행된 결과물은 객체 그 자체가 아니라, 메모리 어딘가에 잘 만들어진 객체의 주소(SharedPtr) 를 반환
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile, std::bind(&LidarLaneNode::scan_callback, this, std::placeholders::_1));

        target_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/target_point",10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/perception_markers", 10);

    }
private:
    struct RansacRes{
        std::vector<Eigen::Vector2d> inliers;
        std::vector<Eigen::Vector2d> remaining;
    };

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        std::string frame_id = msg->header.frame_id;
        std::vector<Eigen::Vector2d> points;

        for (size_t i = 0 ; i < msg->ranges.size() ; i++){
            float r = msg->ranges[i];
            if (r>0.1f && r<5.0f){
                float angle = msg->angle_min + i * msg->angle_increment;
                points.emplace_back(r * std::cos(angle), r * std::sin(angle)); // 2차원 좌표 배열 벡터
            }
        }
        if (points.size() < 10) return;

        auto line1 = run_ransac(points);
        auto line2 = run_ransac(line1.remaining);

        visualization_msgs::msg::MarkerArray marker_array;

        if (!line1.inliers.empty() && !line2.inliers.empty()) {
            Eigen::Vector2d center1 = calculate_mean(line1.inliers);
            Eigen::Vector2d center2 = calculate_mean(line2.inliers);
            Eigen::Vector2d target = (center1 + center2) / 2.0;

            geometry_msgs::msg::Point target_msg;
            target_msg.x = target.x();
            target_msg.y = target.y();
            target_msg.z = 0.0;
            target_pub_->publish(target_msg);

            marker_array.markers.push_back(create_marker(frame_id,0,visualization_msgs::msg::Marker::SPHERE, target, {0.0,1.0,1.0,1.0},0.15));
        }
        if (!line1.inliers.empty()){
            marker_array.markers.push_back(create_points_marker(frame_id, 1, line1.inliers, {1.0, 0.0, 0.0, 1.0}));
        }
        if (!line2.inliers.empty()){
            marker_array.markers.push_back(create_points_marker(frame_id, 2,line2.inliers, {0.0, 0.5, 1.0, 1.0}));
        }
        if (!marker_array.markers.empty()) {
                    marker_pub_->publish(marker_array);
        }
    }

    RansacRes run_ransac(const std::vector<Eigen::Vector2d> pts, int iterations = 50, double threshold = 0.05){
        if (pts.size() < 10) return {{}, pts};

        std::vector<Eigen::Vector2d> best_inliers;
        std::vector<bool> best_mask(pts.size(), false);
        int max_inlier_count = 0;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, pts.size()-1);

        for (int i = 0;i < iterations; i++){
            Eigen::Vector2d p1 = pts[dis(gen)];
            Eigen::Vector2d p2 = pts[dis(gen)];

            double a = p1.y() - p2.y();
            double b = p2.x() - p1.x();
            double c = p1.x() * p2.y() - p2.x() * p1.y();

            double norm = std::sqrt(a*a + b*b);
            if (norm < 1e-6) continue;
            
            int current_inlier_count = 0;
            std::vector<bool> current_mask(pts.size(), false);
            
            for (size_t j = 0; j<pts.size(); ++j){
                double dist = std::abs(a*pts[j].x() + b*pts[j].y() + c) / norm;
                if (dist < threshold){
                    current_mask[j] = true;
                    current_inlier_count++;
                }
            }

            if (current_inlier_count > max_inlier_count){
                max_inlier_count = current_inlier_count;
                best_mask = current_mask;
            }
        }
        RansacRes res;
        for (size_t i = 0; i< pts.size();++i){
            if (best_mask[i]) res.inliers.push_back(pts[i]);
            else res.remaining.push_back(pts[i]);
        }
        return res;
    }

    Eigen::Vector2d calculate_mean(const std::vector<Eigen::Vector2d>& pts){
        Eigen::Vector2d sum(0,0);
        for (const auto& p : pts) sum += p;
        return sum / static_cast<double>(pts.size());
    }

    visualization_msgs::msg::Marker create_marker(std::string frame_id, int id, int type, Eigen::Vector2d pos, std::vector<float> color, float scale){
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = this->now();
        m.ns = "perception_target";
        m.id = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = pos.x();
        m.pose.position.y = pos.y();
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.color.r = color[0]; m.color.g = color[1]; m.color.b = color[2]; m.color.a = color[3];
        return m;
    }
    
    visualization_msgs::msg::Marker create_points_marker(std::string frame_id, int id, const std::vector<Eigen::Vector2d>& points, std::vector<float> color){
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = this->now();
        m.ns = "perception_walls";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::POINTS;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x=m.scale.y = 0.03;
        m.color.r = color[0]; m.color.g = color[1]; m.color.b = color[2]; m.color.a = color[3];  
        for (const auto& p : points){
            geometry_msgs::msg::Point pt;
            pt.x = p.x(); pt.y = p.y(); pt.z = 0.0;
            m.points.push_back(pt);
        }
        return m;
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr target_pub_;    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<LidarLaneNode>());
    rclcpp::shutdown();
    return 0;
}

