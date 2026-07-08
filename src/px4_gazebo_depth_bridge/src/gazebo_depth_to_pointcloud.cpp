#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Image.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/image_stamped.pb.h>
#include <gazebo/transport/transport.hh>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class GazeboDepthToPointCloud : public rclcpp::Node
{
public:
  GazeboDepthToPointCloud()
      : Node("gazebo_depth_to_pointcloud")
  {
    declare_parameter("gazebo_topic", "/gazebo/room_obstacles/iris_mid360_sim/mid360_front/link/depth_camera/image");
    declare_parameter("pointcloud_topic", "/sim/mid360/points");
    declare_parameter("frame_id", "world");
    declare_parameter("horizontal_fov_rad", 1.5708);
    declare_parameter("max_range_m", 20.0);
    declare_parameter("stride", 4);
    declare_parameter("x_offset_m", 0.0);
    declare_parameter("y_offset_m", 0.0);
    declare_parameter("z_offset_m", 0.0);
    declare_parameter("yaw_rad", 0.0);
    declare_parameter("pitch_rad", -0.10);
    declare_parameter("roll_rad", 0.0);
    declare_parameter("use_vehicle_pose", false);
    declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    declare_parameter("pose_timeout_s", 1.0);
    declare_parameter("min_world_z_m", -1000.0);
    declare_parameter("max_world_z_m", 1000.0);
    declare_parameter("self_filter_radius_m", 0.0);

    const auto gazebo_topic = get_parameter("gazebo_topic").as_string();
    const auto pointcloud_topic = get_parameter("pointcloud_topic").as_string();
    _frame_id = get_parameter("frame_id").as_string();
    _horizontal_fov = get_parameter("horizontal_fov_rad").as_double();
    _max_range = get_parameter("max_range_m").as_double();
    _stride = std::max<int64_t>(1, get_parameter("stride").as_int());
    _x_offset = get_parameter("x_offset_m").as_double();
    _y_offset = get_parameter("y_offset_m").as_double();
    _z_offset = get_parameter("z_offset_m").as_double();
    _yaw = get_parameter("yaw_rad").as_double();
    _pitch = get_parameter("pitch_rad").as_double();
    _roll = get_parameter("roll_rad").as_double();
    _use_vehicle_pose = get_parameter("use_vehicle_pose").as_bool();
    _pose_timeout = get_parameter("pose_timeout_s").as_double();
    _min_world_z = get_parameter("min_world_z_m").as_double();
    _max_world_z = get_parameter("max_world_z_m").as_double();
    _self_filter_radius = std::max(0.0, get_parameter("self_filter_radius_m").as_double());

    _pub = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 10);
    if (_use_vehicle_pose)
    {
      const auto vehicle_local_position_topic =
          get_parameter("vehicle_local_position_topic").as_string();
      _vehicle_pose_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          vehicle_local_position_topic,
          rclcpp::SensorDataQoS(),
          [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr pose)
          {
            handleVehiclePose(*pose);
          });
      RCLCPP_INFO(
          get_logger(),
          "Using PX4 local pose from %s to publish depth points in %s frame",
          vehicle_local_position_topic.c_str(),
          _frame_id.c_str());
    }

    gazebo::client::setup();
    _gz_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    _gz_node->Init();
    _sub = _gz_node->Subscribe(gazebo_topic, &GazeboDepthToPointCloud::handleDepth, this);

    RCLCPP_INFO(
        get_logger(),
        "Gazebo depth image -> PointCloud2: %s -> %s",
        gazebo_topic.c_str(),
        pointcloud_topic.c_str());
  }

  ~GazeboDepthToPointCloud() override
  {
    if (_sub)
    {
      _sub->Unsubscribe();
    }
    gazebo::client::shutdown();
  }

private:
  void handleDepth(ConstImageStampedPtr &msg)
  {
    const auto &image = msg->image();
    const uint32_t width = image.width();
    const uint32_t height = image.height();
    const uint32_t step = image.step();
    const uint32_t pixel_format = image.pixel_format();
    const std::string &data = image.data();

    if (width == 0 || height == 0 || data.empty())
    {
      return;
    }

    if (!_reported_format)
    {
      RCLCPP_INFO(
          get_logger(),
          "First Gazebo depth image: width=%u height=%u step=%u pixel_format=%u bytes=%zu",
          width,
          height,
          step,
          pixel_format,
          data.size());
      _reported_format = true;
    }

    if (pixel_format != gazebo::common::Image::R_FLOAT32)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Unsupported depth pixel_format=%u. Expected R_FLOAT32=%u.",
          pixel_format,
          static_cast<unsigned>(gazebo::common::Image::R_FLOAT32));
      return;
    }

    const uint32_t row_step = step > 0 ? step : width * sizeof(float);
    const double fx = static_cast<double>(width) / (2.0 * std::tan(_horizontal_fov * 0.5));
    const double fy = fx;
    const double cx = (static_cast<double>(width) - 1.0) * 0.5;
    const double cy = (static_cast<double>(height) - 1.0) * 0.5;

    std::vector<std::array<float, 4>> points;
    points.reserve((width / _stride + 1) * (height / _stride + 1));
    const auto vehicle_pose = latestVehiclePose();

    for (uint32_t v = 0; v < height; v += static_cast<uint32_t>(_stride))
    {
      const size_t row = static_cast<size_t>(v) * row_step;
      for (uint32_t u = 0; u < width; u += static_cast<uint32_t>(_stride))
      {
        const size_t index = row + static_cast<size_t>(u) * sizeof(float);
        if (index + sizeof(float) > data.size())
        {
          continue;
        }

        float depth = 0.0f;
        std::memcpy(&depth, data.data() + index, sizeof(float));
        if (!std::isfinite(depth) || depth <= 0.05f || depth > _max_range)
        {
          continue;
        }

        // Gazebo camera optical frame: +X right, +Y down, +Z forward.
        const double x_cam = (static_cast<double>(u) - cx) * depth / fx;
        const double y_cam = (static_cast<double>(v) - cy) * depth / fy;
        const double z_cam = depth;

        // Convert to vehicle FLU-like camera mount frame: +X forward, +Y left, +Z up.
        const double x = z_cam;
        const double y = -x_cam;
        const double z = -y_cam;

        const auto rotated = rotateMount(x, y, z);
        const std::array<double, 3> body_point = {
            rotated[0] + _x_offset,
            rotated[1] + _y_offset,
            rotated[2] + _z_offset};
        const auto output_point = vehicle_pose.valid ? transformBodyToWorld(body_point, vehicle_pose) : body_point;
        if (!passesWorldFilters(output_point, vehicle_pose))
        {
          continue;
        }
        points.push_back({static_cast<float>(output_point[0]),
                          static_cast<float>(output_point[1]),
                          static_cast<float>(output_point[2]),
                          depth});
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = _frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
    for (const auto &point : points)
    {
      *iter_x = point[0];
      *iter_y = point[1];
      *iter_z = point[2];
      *iter_intensity = point[3];
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }

    _pub->publish(cloud);
  }

  struct VehiclePose
  {
    bool valid{false};
    double x{0.0};
    double y{0.0};
    double z_up{0.0};
    double heading{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  void handleVehiclePose(const px4_msgs::msg::VehicleLocalPosition &msg)
  {
    if (!msg.xy_valid || !msg.z_valid)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(_pose_mutex);
    _vehicle_pose.valid = true;
    _vehicle_pose.x = static_cast<double>(msg.x);
    _vehicle_pose.y = static_cast<double>(msg.y);
    // PX4 local position is NED for altitude. Ego-Planner world uses Z-up.
    _vehicle_pose.z_up = -static_cast<double>(msg.z);
    _vehicle_pose.heading = static_cast<double>(msg.heading);
    _vehicle_pose.stamp = now();
  }

  VehiclePose latestVehiclePose()
  {
    if (!_use_vehicle_pose)
    {
      return VehiclePose();
    }

    std::lock_guard<std::mutex> lock(_pose_mutex);
    if (!_vehicle_pose.valid)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Waiting for PX4 local pose before transforming depth points to world");
      return VehiclePose();
    }

    const double age_s = (now() - _vehicle_pose.stamp).seconds();
    if (age_s > _pose_timeout)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "PX4 local pose is stale (%.2fs); publishing depth points in camera-relative frame",
          age_s);
      return VehiclePose();
    }

    return _vehicle_pose;
  }

  std::array<double, 3> transformBodyToWorld(
      const std::array<double, 3> &body_point,
      const VehiclePose &vehicle_pose) const
  {
    const double c = std::cos(vehicle_pose.heading);
    const double s = std::sin(vehicle_pose.heading);
    return {
        vehicle_pose.x + (body_point[0] * c) - (body_point[1] * s),
        vehicle_pose.y + (body_point[0] * s) + (body_point[1] * c),
        vehicle_pose.z_up + body_point[2]};
  }

  bool passesWorldFilters(
      const std::array<double, 3> &point,
      const VehiclePose &vehicle_pose) const
  {
    if (point[2] < _min_world_z || point[2] > _max_world_z)
    {
      return false;
    }

    if (_self_filter_radius > 0.0 && vehicle_pose.valid)
    {
      const double dx = point[0] - vehicle_pose.x;
      const double dy = point[1] - vehicle_pose.y;
      const double dz = point[2] - vehicle_pose.z_up;
      if (std::sqrt((dx * dx) + (dy * dy) + (dz * dz)) < _self_filter_radius)
      {
        return false;
      }
    }

    return true;
  }

  std::array<double, 3> rotateMount(double x, double y, double z) const
  {
    const double cr = std::cos(_roll);
    const double sr = std::sin(_roll);
    const double cp = std::cos(_pitch);
    const double sp = std::sin(_pitch);
    const double cy = std::cos(_yaw);
    const double sy = std::sin(_yaw);

    const double x1 = x;
    const double y1 = cr * y - sr * z;
    const double z1 = sr * y + cr * z;

    const double x2 = cp * x1 + sp * z1;
    const double y2 = y1;
    const double z2 = -sp * x1 + cp * z1;

    return {
        cy * x2 - sy * y2,
        sy * x2 + cy * y2,
        z2};
  }

  gazebo::transport::NodePtr _gz_node;
  gazebo::transport::SubscriberPtr _sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_pose_sub;
  std::string _frame_id{"world"};
  double _horizontal_fov{1.5708};
  double _max_range{20.0};
  int64_t _stride{4};
  double _x_offset{0.0};
  double _y_offset{0.0};
  double _z_offset{0.0};
  double _yaw{0.0};
  double _pitch{-0.10};
  double _roll{0.0};
  bool _use_vehicle_pose{false};
  double _pose_timeout{1.0};
  double _min_world_z{-1000.0};
  double _max_world_z{1000.0};
  double _self_filter_radius{0.0};
  std::mutex _pose_mutex;
  VehiclePose _vehicle_pose;
  bool _reported_format{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboDepthToPointCloud>());
  rclcpp::shutdown();
  return 0;
}
