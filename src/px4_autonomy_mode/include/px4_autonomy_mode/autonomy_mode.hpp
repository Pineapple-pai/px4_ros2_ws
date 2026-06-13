#pragma once

#include <Eigen/Core>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class Px4AutonomyMode : public px4_ros2::ModeBase
{
public:
  explicit Px4AutonomyMode(rclcpp::Node & node)
  : ModeBase(node, Settings{"ROS2 Autonomy"}.preventArming(true))
  {
    _goto_setpoint = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    node.declare_parameter("mission_size_m", 8.0);
    node.declare_parameter("mission_altitude_m", 4.0);
    node.declare_parameter("hold_time_s", 2.0);
    node.declare_parameter("acceptance_radius_m", 0.6);
    node.declare_parameter("max_horizontal_velocity_m_s", 2.0);
    node.declare_parameter("max_vertical_velocity_m_s", 1.0);
    node.declare_parameter("max_heading_rate_deg_s", 45.0);
    node.declare_parameter("mission_timeout_s", 120.0);
    node.declare_parameter("auto_rtl_after_finish", false);
    node.declare_parameter("obstacle_distance_topic", "/perception/min_obstacle_distance");
    node.declare_parameter("obstacle_stop_distance_m", 2.0);
    node.declare_parameter("obstacle_abort_distance_m", 1.0);
    node.declare_parameter("obstacle_hold_timeout_s", 5.0);
    node.declare_parameter("enable_obstacle_hold", true);
    node.declare_parameter("target_point_topic", "/autonomy/target_ned");
    node.declare_parameter("accept_runtime_target", true);

    node.get_parameter("mission_size_m", _mission_size_m);
    node.get_parameter("mission_altitude_m", _mission_altitude_m);
    node.get_parameter("hold_time_s", _hold_time_s);
    node.get_parameter("acceptance_radius_m", _acceptance_radius_m);
    node.get_parameter("max_horizontal_velocity_m_s", _max_horizontal_velocity_m_s);
    node.get_parameter("max_vertical_velocity_m_s", _max_vertical_velocity_m_s);
    node.get_parameter("max_heading_rate_deg_s", _max_heading_rate_deg_s);
    node.get_parameter("mission_timeout_s", _mission_timeout_s);
    node.get_parameter("auto_rtl_after_finish", _auto_rtl_after_finish);
    node.get_parameter("obstacle_distance_topic", _obstacle_distance_topic);
    node.get_parameter("obstacle_stop_distance_m", _obstacle_stop_distance_m);
    node.get_parameter("obstacle_abort_distance_m", _obstacle_abort_distance_m);
    node.get_parameter("obstacle_hold_timeout_s", _obstacle_hold_timeout_s);
    node.get_parameter("enable_obstacle_hold", _enable_obstacle_hold);
    node.get_parameter("target_point_topic", _target_point_topic);
    node.get_parameter("accept_runtime_target", _accept_runtime_target);

    _obstacle_distance_sub = node.create_subscription<std_msgs::msg::Float32>(
      _obstacle_distance_topic,
      rclcpp::QoS(5),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        _latest_obstacle_distance_m = msg->data;
        _latest_obstacle_stamp = this->node().get_clock()->now();
      });

    _target_point_sub = node.create_subscription<geometry_msgs::msg::PointStamped>(
      _target_point_topic,
      rclcpp::QoS(5),
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (!_accept_runtime_target) {
          return;
        }

        _manual_target_ned_m = Eigen::Vector3f{
          static_cast<float>(msg->point.x),
          static_cast<float>(msg->point.y),
          static_cast<float>(msg->point.z)};
        _manual_target_received = true;

        if (isActive()) {
          _mission_waypoints.clear();
          _mission_waypoints.push_back(*_manual_target_ned_m);
          _waypoint_index = 0;
          _hold_started = false;
          _last_reported_waypoint_index = static_cast<std::size_t>(-1);
          _state = MissionState::Transit;
          RCLCPP_WARN(this->node().get_logger(),
            "Runtime target accepted -> NED [%.2f, %.2f, %.2f]",
            _manual_target_ned_m->x(), _manual_target_ned_m->y(), _manual_target_ned_m->z());
        }
      });
  }

  void onActivate() override
  {
    _start_position_ned_m = _vehicle_local_position->positionNed();
    _hold_started = false;
    _mission_completed_reported = false;
    _rtl_sent = false;
    _waypoint_index = 0;
    _state = MissionState::Transit;
    _last_reported_waypoint_index = static_cast<std::size_t>(-1);
    _mission_start = node().get_clock()->now();
    _hold_start = rclcpp::Time{};
    _obstacle_hold_started = false;
    buildMission();
    RCLCPP_INFO(node().get_logger(),
      "Autonomy mode activated. mission_size=%.2f m altitude=%.2f m acceptance_radius=%.2f m obstacle_topic=%s target_topic=%s",
      _mission_size_m, _mission_altitude_m, _acceptance_radius_m, _obstacle_distance_topic.c_str(),
      _target_point_topic.c_str());
  }

  void onDeactivate() override
  {
    RCLCPP_WARN(node().get_logger(),
      "Autonomy mode deactivated. Vehicle control returns to the newly selected PX4 mode.");
  }

  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;
    if (_mission_waypoints.empty()) {
      completed(px4_ros2::Result::ModeFailureOther);
      return;
    }

    const double elapsed_s = (node().get_clock()->now() - _mission_start).seconds();
    if (_mission_timeout_s > 0.0 && elapsed_s > _mission_timeout_s) {
      RCLCPP_ERROR(node().get_logger(),
        "Mission timeout after %.1f s. Complete the mode and hand control back to PX4.",
        elapsed_s);
      completed(px4_ros2::Result::Timeout);
      return;
    }

    if (handleObstacleLogic()) {
      return;
    }

    const Eigen::Vector3f target = _mission_waypoints[_waypoint_index];
    const Eigen::Vector2f delta_xy = target.head<2>() - _vehicle_local_position->positionNed().head<2>();
    std::optional<float> heading_rad{};
    if (delta_xy.norm() > 0.5f) {
      heading_rad = std::atan2(delta_xy.y(), delta_xy.x());
    }

    if (_waypoint_index != _last_reported_waypoint_index) {
      RCLCPP_INFO(node().get_logger(),
        "Heading to waypoint %zu/%zu -> [%.2f, %.2f, %.2f]",
        _waypoint_index + 1,
        _mission_waypoints.size(),
        target.x(), target.y(), target.z());
      _last_reported_waypoint_index = _waypoint_index;
    }

    _goto_setpoint->update(
      target,
      heading_rad,
      static_cast<float>(_max_horizontal_velocity_m_s),
      static_cast<float>(_max_vertical_velocity_m_s),
      px4_ros2::degToRad(static_cast<float>(_max_heading_rate_deg_s)));

    if (_waypoint_index + 1 < _mission_waypoints.size()) {
      if (positionReached(target)) {
        RCLCPP_INFO(node().get_logger(), "Reached waypoint %zu", _waypoint_index + 1);
        ++_waypoint_index;
      }
      return;
    }

    if (!_hold_started && positionReached(target)) {
      _hold_started = true;
      _state = MissionState::Holding;
      _hold_start = node().get_clock()->now();
      RCLCPP_INFO(node().get_logger(), "Mission end reached, holding in place.");
    }

    if (_hold_started && (node().get_clock()->now() - _hold_start).seconds() >= _hold_time_s) {
      _state = MissionState::Finished;
      if (_mission_completed_reported) {
        return;
      }
      _mission_completed_reported = true;
      if (_auto_rtl_after_finish && !_rtl_sent) {
        _rtl_sent = true;
        RCLCPP_INFO(node().get_logger(),
          "Mission completed. Switch to RTL from QGC to recover the vehicle.");
      }
      RCLCPP_INFO(node().get_logger(),
        "Autonomy mission completed. Switch to Position mode or Land/RTL from RC/QGC as needed.");
      completed(px4_ros2::Result::Success);
    }
  }

private:
  enum class MissionState {
    Transit,
    ObstacleHold,
    Aborted,
    Holding,
    Finished
  };

  bool handleObstacleLogic()
  {
    if (_state == MissionState::Aborted) {
      return true;
    }

    if (!_enable_obstacle_hold) {
      return false;
    }

    if (_latest_obstacle_distance_m <= 0.0f) {
      return false;
    }

    if (_latest_obstacle_distance_m <= static_cast<float>(_obstacle_abort_distance_m)) {
      if (_state != MissionState::Aborted) {
        _state = MissionState::Aborted;
        RCLCPP_ERROR(node().get_logger(),
          "Obstacle too close: %.2f m <= abort threshold %.2f m. Aborting autonomy mode.",
          _latest_obstacle_distance_m, _obstacle_abort_distance_m);
      }
      completed(px4_ros2::Result::ModeFailureOther);
      return true;
    }

    if (_latest_obstacle_distance_m <= static_cast<float>(_obstacle_stop_distance_m)) {
      if (!_obstacle_hold_started) {
        _obstacle_hold_started = true;
        _hold_start = node().get_clock()->now();
        _state = MissionState::ObstacleHold;
        RCLCPP_WARN(node().get_logger(),
          "Obstacle hold triggered: %.2f m <= stop threshold %.2f m.",
          _latest_obstacle_distance_m, _obstacle_stop_distance_m);
      }

      const Eigen::Vector3f hold_target = _vehicle_local_position->positionNed();
      _goto_setpoint->update(
        hold_target,
        std::nullopt,
        0.0f,
        0.0f,
        px4_ros2::degToRad(static_cast<float>(_max_heading_rate_deg_s)));

      if (_obstacle_hold_timeout_s > 0.0 &&
          (node().get_clock()->now() - _hold_start).seconds() >= _obstacle_hold_timeout_s) {
        _state = MissionState::Aborted;
        RCLCPP_ERROR(node().get_logger(),
          "Obstacle hold timeout after %.1f s. Aborting autonomy mode.",
          _obstacle_hold_timeout_s);
        completed(px4_ros2::Result::Timeout);
      }
      return true;
    }

    if (_obstacle_hold_started) {
      _obstacle_hold_started = false;
      _state = MissionState::Transit;
      RCLCPP_INFO(node().get_logger(),
        "Obstacle cleared at %.2f m. Resuming mission.", _latest_obstacle_distance_m);
    }

    return false;
  }

  void buildMission()
  {
    const float x0 = _start_position_ned_m.x();
    const float y0 = _start_position_ned_m.y();
    const float mission_size_m = static_cast<float>(_mission_size_m);
    const float z_target = -static_cast<float>(std::fabs(_mission_altitude_m));

    _mission_waypoints.clear();
    if (_manual_target_received && _manual_target_ned_m.has_value()) {
      _mission_waypoints.push_back(*_manual_target_ned_m);
      return;
    }

    _mission_waypoints.push_back(Eigen::Vector3f{x0, y0, z_target});
    _mission_waypoints.push_back(Eigen::Vector3f{x0 + mission_size_m, y0, z_target});
    _mission_waypoints.push_back(Eigen::Vector3f{x0 + mission_size_m, y0 + mission_size_m, z_target});
    _mission_waypoints.push_back(Eigen::Vector3f{x0, y0 + mission_size_m, z_target});
    _mission_waypoints.push_back(Eigen::Vector3f{x0, y0, z_target});
  }

  bool positionReached(const Eigen::Vector3f & target_position_ned_m) const
  {
    static constexpr float kVelocityThresholdMs = 0.4f;

    const Eigen::Vector3f position_error =
      target_position_ned_m - _vehicle_local_position->positionNed();
    return position_error.norm() < static_cast<float>(_acceptance_radius_m) &&
           _vehicle_local_position->velocityNed().norm() < kVelocityThresholdMs;
  }

  std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _obstacle_distance_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _target_point_sub;

  Eigen::Vector3f _start_position_ned_m{0.0f, 0.0f, 0.0f};
  std::vector<Eigen::Vector3f> _mission_waypoints;
  std::size_t _waypoint_index{0};
  std::size_t _last_reported_waypoint_index{0};

  double _mission_size_m{8.0};
  double _mission_altitude_m{4.0};
  double _hold_time_s{2.0};
  double _acceptance_radius_m{0.6};
  double _max_horizontal_velocity_m_s{2.0};
  double _max_vertical_velocity_m_s{1.0};
  double _max_heading_rate_deg_s{45.0};
  double _mission_timeout_s{120.0};
  bool _auto_rtl_after_finish{false};
  std::string _obstacle_distance_topic{"/perception/min_obstacle_distance"};
  std::string _target_point_topic{"/autonomy/target_ned"};
  double _obstacle_stop_distance_m{2.0};
  double _obstacle_abort_distance_m{1.0};
  double _obstacle_hold_timeout_s{5.0};
  bool _enable_obstacle_hold{true};
  bool _accept_runtime_target{true};
  bool _hold_started{false};
  bool _mission_completed_reported{false};
  bool _rtl_sent{false};
  bool _obstacle_hold_started{false};
  MissionState _state{MissionState::Transit};
  rclcpp::Time _mission_start{};
  rclcpp::Time _hold_start{};
  rclcpp::Time _latest_obstacle_stamp{};
  float _latest_obstacle_distance_m{-1.0f};
  bool _manual_target_received{false};
  std::optional<Eigen::Vector3f> _manual_target_ned_m{};
};
