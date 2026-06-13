#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class PointPublisherNode : public rclcpp::Node {
public:
    PointPublisherNode() : Node("point_publisher_node") {
        // 创建 Marker 发布者，话题名为 /visual_point
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visual_point", 10);
        
        // 创建定时器，以 10Hz (100ms) 频率调用回调函数
        timer_ = this->create_wall_timer(100ms, std::bind(&PointPublisherNode::publish_point, this));
        
        RCLCPP_INFO(this->get_logger(), "Point Publisher Node Started.");
    }

private:
    void publish_point() {
        visualization_msgs::msg::Marker marker;
        
        // 1. 设置头部信息
        marker.header.frame_id = "world";          // 父参考坐标系
        marker.header.stamp = this->get_clock()->now();
        
        // 2. 设置 Marker 的命名空间和 ID（用于区分不同的标记）
        marker.ns = "target_point";
        marker.id = 0;
        
        // 3. 设置标记类型为球体（SPHERE），也可以选择 CUBE（立方体）
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        
        // 4. 设置动作：ADD（创建或更新）
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 5. 【核心】设置你想要显示的 3D 坐标位置
        marker.pose.position.x = 1.0;
        marker.pose.position.y = 2.0;
        marker.pose.position.z = 3.0;
        
        // 姿态保持默认（不旋转）
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        // 6. 设置球体的大小（直径，单位：米）
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        // 7. 设置颜色（RGBA，范围 0.0 ~ 1.0）
        marker.color.r = 1.0; // 红色
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // 不透明度
        
        // 8. 设置有效期，0 表示永久存在，直到接收到新的消息或被删除
        marker.lifetime = rclcpp::Duration(0s);
        
        // 9. 发布消息
        marker_pub_->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointPublisherNode>());
    rclcpp::shutdown();
    return 0;
}

