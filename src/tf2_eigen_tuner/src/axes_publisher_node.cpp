#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// 引入 Eigen 头文件进行高效矩阵变换
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class AxesPublisherNode : public rclcpp::Node {
public:
    AxesPublisherNode() : Node("axes_publisher_node") {
        // 1. 创建 MarkerArray 发布者
        axes_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visual_axes", 10);

        // 2. 严密配置参数范围描述符，确保界面不闪烁
        rcl_interfaces::msg::FloatingPointRange range_xyz;
        range_xyz.from_value = -5.0; range_xyz.to_value = 5.0; range_xyz.step = 0.05;
        rcl_interfaces::msg::ParameterDescriptor desc_xyz; desc_xyz.floating_point_range = {range_xyz};

        rcl_interfaces::msg::FloatingPointRange range_scale;
        range_scale.from_value = 0.1; range_scale.to_value = 3.0; range_scale.step = 0.05;
        rcl_interfaces::msg::ParameterDescriptor desc_scale; desc_scale.floating_point_range = {range_scale};

        rcl_interfaces::msg::FloatingPointRange range_rpy;
        range_rpy.from_value = -180.0; range_rpy.to_value = 180.0; range_rpy.step = 1.0;
        rcl_interfaces::msg::ParameterDescriptor desc_rpy; desc_rpy.floating_point_range = {range_rpy};

        // 3. 声明带数字前缀的 7 个参数，强制 rqt 自上而下完美排序
        this->declare_parameter("1_x", 0.0, desc_xyz);
        this->declare_parameter("2_y", 0.0, desc_xyz);
        this->declare_parameter("3_z", 0.0, desc_xyz);
        this->declare_parameter("4_scale", 1.0, desc_scale);
        this->declare_parameter("5_roll", 0.0, desc_rpy);
        this->declare_parameter("6_pitch", 0.0, desc_rpy);
        this->declare_parameter("7_yaw", 0.0, desc_rpy);

        // 4. 初始化局部变量
        x_ = this->get_parameter("1_x").as_double();
        y_ = this->get_parameter("2_y").as_double();
        z_ = this->get_parameter("3_z").as_double();
        scale_ = this->get_parameter("4_scale").as_double();
        roll_ = this->get_parameter("5_roll").as_double();
        pitch_ = this->get_parameter("6_pitch").as_double();
        yaw_ = this->get_parameter("7_yaw").as_double();

        // 5. 首次计算并强行同步四元数缓存
        force_update_rotation_cache();

        // 6. 注册动态参数监听回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&AxesPublisherNode::on_parameter_change, this, std::placeholders::_1));

        // 7. 设置 30Hz 高频定时器渲染（每33ms发布一次），配合缓存实现无感刷新
        timer_ = this->create_wall_timer(33ms, std::bind(&AxesPublisherNode::publish_axes, this));

        RCLCPP_INFO(this->get_logger(), "Ultimate Fluid & Filtered Axes Node Started.");
    }

private:
    // 强制更新旋转缓存的底层私有函数
    void force_update_rotation_cache() {
        double current_r_round = std::round(roll_);
        double current_p_round = std::round(pitch_);
        double current_y_round = std::round(yaw_);

        double r_rad = current_r_round * M_PI / 180.0;
        double p_rad = current_p_round * M_PI / 180.0;
        double y_rad = current_y_round * M_PI / 180.0;

        Eigen::AngleAxisd roll_angle(r_rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(p_rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(y_rad, Eigen::Vector3d::UnitZ());
        
        // 更新全局四元数缓存
        q_global_cached_ = yaw_angle * pitch_angle * roll_angle;

        // 更新防刷历史记录
        last_r_ = current_r_round;
        last_p_ = current_p_round;
        last_y_ = current_y_round;
    }

    // 参数动态修改回调（包含高频去重拦截机制）
    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool rotation_changed = false;

        for (const auto & param : parameters) {
            std::string name = param.get_name();
            if (name == "1_x") x_ = param.as_double();
            else if (name == "2_y") y_ = param.as_double();
            else if (name == "3_z") z_ = param.as_double();
            else if (name == "4_scale") scale_ = param.as_double();
            else if (name == "5_roll")  { roll_ = param.as_double();  rotation_changed = true; }
            else if (name == "6_pitch") { pitch_ = param.as_double(); rotation_changed = true; }
            else if (name == "7_yaw")   { yaw_ = param.as_double();   rotation_changed = true; }
        }

        // 如果触发了旋转参数变更
        if (rotation_changed) {
            double current_r_round = std::round(roll_);
            double current_p_round = std::round(pitch_);
            double current_y_round = std::round(yaw_);

            // 【去重拦截核心】：对比上一次的角度，只有当四舍五入发生实质性整数度跃迁时才放行计算
            if (current_r_round != last_r_ || current_p_round != last_p_ || current_y_round != last_y_) {
                force_update_rotation_cache();
            }
        }

        return result;
    }

    // 创建单个轴箭头的辅助函数（0三角函数算力开销）
    visualization_msgs::msg::Marker create_arrow(int id, double i_qx, double i_qy, double i_qz, double i_qw,
                                                 double r, double g, double b) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "coordinate_axes";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 填充通过防刷过滤后的平移位置
        marker.pose.position.x = x_;
        marker.pose.position.y = y_;
        marker.pose.position.z = z_;

        // 【矩阵乘法核心】：全局缓存四元数 * 各轴自身的固有基础旋转
        Eigen::Quaterniond q_intrinsic(i_qw, i_qx, i_qy, i_qz);
        Eigen::Quaterniond q_final = q_global_cached_ * q_intrinsic;

        marker.pose.orientation.x = q_final.x();
        marker.pose.orientation.y = q_final.y();
        marker.pose.orientation.z = q_final.z();
        marker.pose.orientation.w = q_final.w();

        // 填充动态缩放
        marker.scale.x = scale_; 
        marker.scale.y = 0.05 * scale_; 
        marker.scale.z = 0.05 * scale_;

        // 填充颜色
        marker.color.r = r; marker.color.g = g; marker.color.b = b; marker.color.a = 1.0;
        return marker;
    }

    void publish_axes() {
        visualization_msgs::msg::MarkerArray marker_array;

        // 1. X轴：红色 (固有状态 w=1.0, 保持朝前)
        marker_array.markers.push_back(create_arrow(0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0));

        // 2. Y轴：绿色 (固有状态绕 Z 轴旋转 90 度)
        marker_array.markers.push_back(create_arrow(1, 0.0, 0.0, 0.7071, 0.7071, 0.0, 1.0, 0.0));

        // 3. Z轴：蓝色 (固有状态绕 Y 轴旋转 -90 度)
        marker_array.markers.push_back(create_arrow(2, 0.0, -0.7071, 0.0, 0.7071, 0.0, 0.0, 1.0));

        // 发布拼装好的完美三维坐标轴
        axes_pub_->publish(marker_array);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr axes_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 存储当前参数值
    double x_; double y_; double z_; double scale_;
    double roll_; double pitch_; double yaw_;

    // 防刷去重历史记录变量
    double last_r_ = -999.0;
    double last_p_ = -999.0;
    double last_y_ = -999.0;

    // Eigen 四元数缓存
    Eigen::Quaterniond q_global_cached_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AxesPublisherNode>());
    rclcpp::shutdown();
    return 0;
}

