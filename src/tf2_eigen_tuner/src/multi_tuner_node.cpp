#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// 引入 Eigen 头文件
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

// 定义单个坐标轴的结构体，方便管理参数
struct AxisData {
    std::string frame_id;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;   // 单位：度
    double pitch = 0.0;  // 单位：度
    double yaw = 0.0;    // 单位：度
};

class MultiTunerNode : public rclcpp::Node {
public:
    MultiTunerNode() : Node("multi_tf_tuner_node") {
        // 1. 初始化 TF2 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 2. 初始化两个坐标轴的数据结构
        axis_a_.frame_id = "axis_a";
        axis_b_.frame_id = "axis_b";

        // 3. 声明 ROS 2 参数并设置范围描述（供 rqt_reconfigure 识别为滑块）
        declare_axis_parameters(axis_a_.frame_id);
        declare_axis_parameters(axis_b_.frame_id);

        // 4. 设置参数动态更新的回调函数
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MultiTunerNode::on_parameter_change, this, std::placeholders::_1));

        // 5. 首次从声明中读取参数初始值
        update_local_variables();

        // 6. 创建 30Hz 定时器用于循环广播 TF
        timer_ = this->create_wall_timer(33ms, std::bind(&MultiTunerNode::broadcast_tf, this));
        
        RCLCPP_INFO(this->get_logger(), "Multi-TF Eigen Tuner Node Started.");
    }

private:
    // 声明参数的辅助函数（限制位置 ±5m，角度 ±180度）
    void declare_axis_parameters(const std::string & prefix) {
        rcl_interfaces::msg::ParameterDescriptor desc_xyz;
        desc_xyz.floating_point_range.resize(1);
        desc_xyz.floating_point_range[0].from_value = -5.0;
        desc_xyz.floating_point_range[0].to_value = 5.0;
        desc_xyz.floating_point_range[0].step = 0.05;

        rcl_interfaces::msg::ParameterDescriptor desc_rpy;
        desc_rpy.floating_point_range.resize(1);
        desc_rpy.floating_point_range[0].from_value = -180.0;
        desc_rpy.floating_point_range[0].to_value = 180.0;
        desc_rpy.floating_point_range[0].step = 1.0;

        this->declare_parameter(prefix + ".x", 0.0, desc_xyz);
        this->declare_parameter(prefix + ".y", 0.0, desc_xyz);
        this->declare_parameter(prefix + ".z", 0.0, desc_xyz);
        this->declare_parameter(prefix + ".roll", 0.0, desc_rpy);
        this->declare_parameter(prefix + ".pitch", 0.0, desc_rpy);
        this->declare_parameter(prefix + ".yaw", 0.0, desc_rpy);
    }

    // 动态参数修改回调
    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : parameters) {
            std::string name = param.get_name();
            double val = param.as_double();

            // 解析参数属于哪个轴
            if (name.find("axis_a.") == 0) {
                assign_value(axis_a_, name.substr(7), val);
            } else if (name.find("axis_b.") == 0) {
                assign_value(axis_b_, name.substr(7), val);
            }
        }
        return result;
    }

    void assign_value(AxisData & axis, const std::string & field, double value) {
        if (field == "x") axis.x = value;
        else if (field == "y") axis.y = value;
        else if (field == "z") axis.z = value;
        else if (field == "roll") axis.roll = value;
        else if (field == "pitch") axis.pitch = value;
        else if (field == "yaw") axis.yaw = value;
    }

    // 初始化时同步一次参数
    void update_local_variables() {
        axis_a_.x = this->get_parameter("axis_a.x").as_double();
        axis_a_.y = this->get_parameter("axis_a.y").as_double();
        axis_a_.z = this->get_parameter("axis_a.z").as_double();
        axis_a_.roll = this->get_parameter("axis_a.roll").as_double();
        axis_a_.pitch = this->get_parameter("axis_a.pitch").as_double();
        axis_a_.yaw = this->get_parameter("axis_a.yaw").as_double();

        axis_b_.x = this->get_parameter("axis_b.x").as_double();
        axis_b_.y = this->get_parameter("axis_b.y").as_double();
        axis_b_.z = this->get_parameter("axis_b.z").as_double();
        axis_b_.roll = this->get_parameter("axis_b.roll").as_double();
        axis_b_.pitch = this->get_parameter("axis_b.pitch").as_double();
        axis_b_.yaw = this->get_parameter("axis_b.yaw").as_double();
    }

    // 核心：使用 Eigen 将欧拉角（度）转为四元数并填充 TF 消息
    geometry_msgs::msg::TransformStamped create_tf_msg(const AxisData & axis) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = axis.frame_id;

        // 填充位置
        t.transform.translation.x = axis.x;
        t.transform.translation.y = axis.y;
        t.transform.translation.z = axis.z;

        // 角度转弧度
        double r_rad = axis.roll * M_PI / 180.0;
        double p_rad = axis.pitch * M_PI / 180.0;
        double y_rad = axis.yaw * M_PI / 180.0;

        // 【使用 Eigen 处理姿态】
        // ROS 的 RPY 旋转顺序通常为 Z-Y-X (Yaw-Pitch-Roll)
        Eigen::AngleAxisd roll_angle(r_rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(p_rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(y_rad, Eigen::Vector3d::UnitZ());

        // 结合成一个四元数
        Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;

        // 填充四元数至 ROS 消息
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        return t;
    }

    void broadcast_tf() {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(create_tf_msg(axis_a_));
        transforms.push_back(create_tf_msg(axis_b_));
        
        // 批量广播
        tf_broadcaster_->sendTransform(transforms);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    AxisData axis_a_;
    AxisData axis_b_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiTunerNode>());
    rclcpp::shutdown();
    return 0;
}

