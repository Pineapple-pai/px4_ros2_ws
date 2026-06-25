#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <yaml-cpp/yaml.h>

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
    node.declare_parameter("front_obstacle_distance_topic", "/perception/front_obstacle_distance");
    node.declare_parameter("left_obstacle_distance_topic", "/perception/left_obstacle_distance");
    node.declare_parameter("right_obstacle_distance_topic", "/perception/right_obstacle_distance");
    node.declare_parameter("up_obstacle_distance_topic", "/perception/up_obstacle_distance");
    node.declare_parameter("down_obstacle_distance_topic", "/perception/down_obstacle_distance");
    node.declare_parameter("obstacle_stop_distance_m", 2.0);
    node.declare_parameter("obstacle_abort_distance_m", 1.0);
    node.declare_parameter("obstacle_hold_timeout_s", 5.0);
    node.declare_parameter("enable_obstacle_hold", true);
    node.declare_parameter("enable_local_avoidance", true);
    node.declare_parameter("avoidance_trigger_distance_m", 3.0);
    node.declare_parameter("avoidance_clearance_m", 2.0);
    node.declare_parameter("avoidance_lateral_offset_m", 1.5);
    node.declare_parameter("avoidance_forward_offset_m", 2.0);
    node.declare_parameter("avoidance_speed_m_s", 0.45);
    node.declare_parameter("obstacle_data_timeout_s", 1.0);
    node.declare_parameter("mission_file", "");
    node.declare_parameter("mission_waypoints_ned", "");
    node.declare_parameter("waypoints_topic", "/autonomy/waypoints_ned");
    node.declare_parameter("target_point_topic", "/autonomy/target_ned");
    node.declare_parameter("accept_runtime_target", true);
    node.declare_parameter("control_source", "mission");
    node.declare_parameter("nav2_cmd_vel_topic", "/cmd_vel");
    node.declare_parameter("nav2_fixed_altitude_m", _mission_altitude_m);
    node.declare_parameter("nav2_cmd_timeout_s", 0.5);
    node.declare_parameter("nav2_lookahead_time_s", 1.0);
    node.declare_parameter("nav2_min_lookahead_m", 0.35);
    node.declare_parameter("nav2_max_cmd_speed_m_s", 0.45);
    node.declare_parameter("nav2_turn_in_place_yaw_rate_rad_s", 0.35);
    node.declare_parameter("nav2_turn_speed_scale", 0.35);
    node.declare_parameter("nav2_turn_max_lookahead_m", 0.25);
    node.declare_parameter("nav2_obstacle_slowdown_distance_m", 3.0);
    node.declare_parameter("nav2_obstacle_min_speed_m_s", 0.12);
    node.declare_parameter("nav2_emergency_retreat_distance_m", 0.8);
    node.declare_parameter("nav2_emergency_retreat_speed_m_s", 0.25);
    node.declare_parameter("nav2_require_ros2_control", true);
    node.declare_parameter("nav2_required_nav_state", -1);
    node.declare_parameter("nav2_vehicle_status_topic", "/fmu/out/vehicle_status_v4");
    node.declare_parameter("nav2_vehicle_control_mode_topic", "/fmu/out/vehicle_control_mode");
    node.declare_parameter("nav2_control_authority_topic", "/autonomy/nav2_control_authority_ok");
    node.declare_parameter("nav2_status_timeout_s", 1.0);

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
    node.get_parameter("front_obstacle_distance_topic", _front_obstacle_distance_topic);
    node.get_parameter("left_obstacle_distance_topic", _left_obstacle_distance_topic);
    node.get_parameter("right_obstacle_distance_topic", _right_obstacle_distance_topic);
    node.get_parameter("up_obstacle_distance_topic", _up_obstacle_distance_topic);
    node.get_parameter("down_obstacle_distance_topic", _down_obstacle_distance_topic);
    node.get_parameter("obstacle_stop_distance_m", _obstacle_stop_distance_m);
    node.get_parameter("obstacle_abort_distance_m", _obstacle_abort_distance_m);
    node.get_parameter("obstacle_hold_timeout_s", _obstacle_hold_timeout_s);
    node.get_parameter("enable_obstacle_hold", _enable_obstacle_hold);
    node.get_parameter("enable_local_avoidance", _enable_local_avoidance);
    node.get_parameter("avoidance_trigger_distance_m", _avoidance_trigger_distance_m);
    node.get_parameter("avoidance_clearance_m", _avoidance_clearance_m);
    node.get_parameter("avoidance_lateral_offset_m", _avoidance_lateral_offset_m);
    node.get_parameter("avoidance_forward_offset_m", _avoidance_forward_offset_m);
    node.get_parameter("avoidance_speed_m_s", _avoidance_speed_m_s);
    node.get_parameter("obstacle_data_timeout_s", _obstacle_data_timeout_s);
    node.get_parameter("mission_file", _mission_file);
    node.get_parameter("mission_waypoints_ned", _mission_waypoints_ned_text);
    node.get_parameter("waypoints_topic", _waypoints_topic);
    node.get_parameter("target_point_topic", _target_point_topic);
    node.get_parameter("accept_runtime_target", _accept_runtime_target);
    node.get_parameter("control_source", _control_source);
    node.get_parameter("nav2_cmd_vel_topic", _nav2_cmd_vel_topic);
    node.get_parameter("nav2_fixed_altitude_m", _nav2_fixed_altitude_m);
    node.get_parameter("nav2_cmd_timeout_s", _nav2_cmd_timeout_s);
    node.get_parameter("nav2_lookahead_time_s", _nav2_lookahead_time_s);
    node.get_parameter("nav2_min_lookahead_m", _nav2_min_lookahead_m);
    node.get_parameter("nav2_max_cmd_speed_m_s", _nav2_max_cmd_speed_m_s);
    node.get_parameter("nav2_turn_in_place_yaw_rate_rad_s", _nav2_turn_in_place_yaw_rate_rad_s);
    node.get_parameter("nav2_turn_speed_scale", _nav2_turn_speed_scale);
    node.get_parameter("nav2_turn_max_lookahead_m", _nav2_turn_max_lookahead_m);
    node.get_parameter("nav2_obstacle_slowdown_distance_m", _nav2_obstacle_slowdown_distance_m);
    node.get_parameter("nav2_obstacle_min_speed_m_s", _nav2_obstacle_min_speed_m_s);
    node.get_parameter("nav2_emergency_retreat_distance_m", _nav2_emergency_retreat_distance_m);
    node.get_parameter("nav2_emergency_retreat_speed_m_s", _nav2_emergency_retreat_speed_m_s);
    node.get_parameter("nav2_require_ros2_control", _nav2_require_ros2_control);
    node.get_parameter("nav2_required_nav_state", _nav2_required_nav_state);
    node.get_parameter("nav2_vehicle_status_topic", _nav2_vehicle_status_topic);
    node.get_parameter("nav2_vehicle_control_mode_topic", _nav2_vehicle_control_mode_topic);
    node.get_parameter("nav2_control_authority_topic", _nav2_control_authority_topic);
    node.get_parameter("nav2_status_timeout_s", _nav2_status_timeout_s);

    if (_control_source != "mission" && _control_source != "nav2_cmd_vel") {
      RCLCPP_WARN(node.get_logger(),
        "Unsupported control_source='%s'. Falling back to 'mission'.",
        _control_source.c_str());
      _control_source = "mission";
    }
    if (!_mission_file.empty()) {
      _configured_mission_waypoints_ned = parseMissionFile(_mission_file);
    }
    if (_configured_mission_waypoints_ned.empty()) {
      _configured_mission_waypoints_ned = parseMissionWaypoints(_mission_waypoints_ned_text);
    }

    _obstacle_distance_sub = node.create_subscription<std_msgs::msg::Float32>(
      _obstacle_distance_topic,
      rclcpp::QoS(5),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        _latest_obstacle_distance_m = msg->data;
        _latest_obstacle_stamp = this->node().get_clock()->now();
      });

    _front_obstacle_distance_sub = node.create_subscription<std_msgs::msg::Float32>(
      _front_obstacle_distance_topic,
      rclcpp::QoS(5),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        _front_obstacle_distance_m = msg->data;
        _front_obstacle_stamp = this->node().get_clock()->now();
      });

    _left_obstacle_distance_sub = node.create_subscription<std_msgs::msg::Float32>(
      _left_obstacle_distance_topic,
      rclcpp::QoS(5),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        _left_obstacle_distance_m = msg->data;
        _left_obstacle_stamp = this->node().get_clock()->now();
      });

    _right_obstacle_distance_sub = node.create_subscription<std_msgs::msg::Float32>(
      _right_obstacle_distance_topic,
      rclcpp::QoS(5),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        _right_obstacle_distance_m = msg->data;
        _right_obstacle_stamp = this->node().get_clock()->now();
      });

    _up_obstacle_distance_sub = node.create_subscription<std_msgs::msg::Float32>(
      _up_obstacle_distance_topic,
      rclcpp::QoS(5),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        _up_obstacle_distance_m = msg->data;
        _up_obstacle_stamp = this->node().get_clock()->now();
      });

    _down_obstacle_distance_sub = node.create_subscription<std_msgs::msg::Float32>(
      _down_obstacle_distance_topic,
      rclcpp::QoS(5),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        _down_obstacle_distance_m = msg->data;
        _down_obstacle_stamp = this->node().get_clock()->now();
      });

    _cmd_vel_sub = node.create_subscription<geometry_msgs::msg::Twist>(
      _nav2_cmd_vel_topic,
      rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        _latest_cmd_vel = *msg;
        _latest_cmd_vel_stamp = this->node().get_clock()->now();
      });

    _vehicle_status_sub = node.create_subscription<px4_msgs::msg::VehicleStatus>(
      _nav2_vehicle_status_topic,
      rclcpp::QoS(5).best_effort(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        _latest_vehicle_status = *msg;
        _latest_vehicle_status_stamp = this->node().get_clock()->now();
      });

    _vehicle_control_mode_sub = node.create_subscription<px4_msgs::msg::VehicleControlMode>(
      _nav2_vehicle_control_mode_topic,
      rclcpp::QoS(5).best_effort(),
      [this](const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
        _latest_vehicle_control_mode = *msg;
        _latest_vehicle_control_mode_stamp = this->node().get_clock()->now();
      });

    _nav2_control_authority_pub = node.create_publisher<std_msgs::msg::Bool>(
      _nav2_control_authority_topic, rclcpp::QoS(5).transient_local());

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
        _manual_mission_received = false;
        _manual_mission_waypoints_ned.clear();

        if (isActive()) {
          _mission_waypoints.clear();
          _mission_waypoints.push_back(makeWaypoint(*_manual_target_ned_m));
          resetMissionProgress();
          RCLCPP_WARN(this->node().get_logger(),
            "Runtime target accepted -> NED [%.2f, %.2f, %.2f]",
            _manual_target_ned_m->x(), _manual_target_ned_m->y(), _manual_target_ned_m->z());
        }
      });

    _waypoints_sub = node.create_subscription<geometry_msgs::msg::PoseArray>(
      _waypoints_topic,
      rclcpp::QoS(5),
      [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (!_accept_runtime_target) {
          return;
        }

        std::vector<MissionWaypoint> waypoints;
        waypoints.reserve(msg->poses.size());
        for (const auto & pose : msg->poses) {
          waypoints.push_back(makeWaypoint(Eigen::Vector3f{
            static_cast<float>(pose.position.x),
            static_cast<float>(pose.position.y),
            static_cast<float>(pose.position.z)}));
        }
        if (waypoints.empty()) {
          RCLCPP_WARN(this->node().get_logger(),
            "Runtime waypoint mission ignored: PoseArray is empty.");
          return;
        }

        _manual_mission_waypoints_ned = waypoints;
        _manual_mission_received = true;
        _manual_target_received = false;
        _manual_target_ned_m.reset();

        if (isActive()) {
          _mission_waypoints = _manual_mission_waypoints_ned;
          resetMissionProgress();
          RCLCPP_WARN(this->node().get_logger(),
            "Runtime waypoint mission accepted: %zu waypoints on %s.",
            _mission_waypoints.size(), _waypoints_topic.c_str());
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
    clearAvoidance();
    buildMission();
    RCLCPP_INFO(node().get_logger(),
      "Autonomy mode activated. control_source=%s waypoints=%zu mission_size=%.2f m altitude=%.2f m acceptance_radius=%.2f m obstacle_topic=%s target_topic=%s waypoints_topic=%s local_avoidance=%s nav2_cmd_vel=%s",
      _control_source.c_str(),
      _mission_waypoints.size(),
      _mission_size_m, _mission_altitude_m, _acceptance_radius_m, _obstacle_distance_topic.c_str(),
      _target_point_topic.c_str(), _waypoints_topic.c_str(), _enable_local_avoidance ? "true" : "false",
      _nav2_cmd_vel_topic.c_str());
  }

  void onDeactivate() override
  {
    RCLCPP_WARN(node().get_logger(),
      "Autonomy mode deactivated. Vehicle control returns to the newly selected PX4 mode.");
  }

  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;
    if (_control_source == "nav2_cmd_vel") {
      updateNav2Setpoint();
      return;
    }

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

    const MissionWaypoint & waypoint = _mission_waypoints[_waypoint_index];
    const Eigen::Vector3f target = waypoint.position;
    const float horizontal_speed_m_s = horizontalSpeedFor(waypoint);
    const float hold_time_s = holdTimeFor(waypoint, _waypoint_index + 1 == _mission_waypoints.size());
    const Eigen::Vector2f delta_xy = target.head<2>() - _vehicle_local_position->positionNed().head<2>();
    std::optional<float> heading_rad{};
    if (delta_xy.norm() > 0.5f) {
      heading_rad = std::atan2(delta_xy.y(), delta_xy.x());
    }

    if (_waypoint_index != _last_reported_waypoint_index) {
      RCLCPP_INFO(node().get_logger(),
        "Heading to waypoint %zu/%zu -> [%.2f, %.2f, %.2f] speed=%.2f m/s hold=%.1f s",
        _waypoint_index + 1,
        _mission_waypoints.size(),
        target.x(), target.y(), target.z(),
        horizontal_speed_m_s,
        hold_time_s);
      _last_reported_waypoint_index = _waypoint_index;
    }

    _goto_setpoint->update(
      target,
      heading_rad,
      _hold_started ? 0.0f : horizontal_speed_m_s,
      static_cast<float>(_max_vertical_velocity_m_s),
      px4_ros2::degToRad(static_cast<float>(_max_heading_rate_deg_s)));

    if (!positionReached(target)) {
      return;
    }

    if (!_hold_started) {
      _hold_started = true;
      _state = MissionState::Holding;
      _hold_start = node().get_clock()->now();
      RCLCPP_INFO(node().get_logger(),
        "Reached waypoint %zu/%zu, holding for %.1f s.",
        _waypoint_index + 1,
        _mission_waypoints.size(),
        hold_time_s);
    }

    if ((node().get_clock()->now() - _hold_start).seconds() < hold_time_s) {
      return;
    }

    _hold_started = false;
    _hold_start = rclcpp::Time{};

    if (_waypoint_index + 1 < _mission_waypoints.size()) {
      ++_waypoint_index;
      _state = MissionState::Transit;
      return;
    }

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

private:
  struct MissionWaypoint {
    Eigen::Vector3f position{0.0f, 0.0f, 0.0f};
    std::optional<float> speed_m_s{};
    std::optional<float> hold_time_s{};
  };

  enum class MissionState {
    Transit,
    ObstacleHold,
    AvoidingObstacle,
    WaitingNav2Command,
    FollowingNav2Command,
    Nav2StaleHold,
    EmergencyStop,
    Aborted,
    Holding,
    Finished
  };

  void updateNav2Setpoint()
  {
    const rclcpp::Time now = node().get_clock()->now();
    const Eigen::Vector3f current_position = _vehicle_local_position->positionNed();
    const float fixed_z_ned = -static_cast<float>(std::fabs(_nav2_fixed_altitude_m));

    if (!nav2ControlAuthorityOk(now)) {
      _state = MissionState::Nav2StaleHold;
      publishGoto(Eigen::Vector3f{current_position.x(), current_position.y(), fixed_z_ned}, 0.0f);
      return;
    }

    const auto emergency = handleNav2EmergencyObstacle();
    if (emergency) {
      return;
    }

    if (_latest_cmd_vel_stamp.nanoseconds() == 0) {
      _state = MissionState::WaitingNav2Command;
      publishGoto(Eigen::Vector3f{current_position.x(), current_position.y(), fixed_z_ned}, 0.0f);
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 2000,
        "Waiting for Nav2 cmd_vel on %s.", _nav2_cmd_vel_topic.c_str());
      return;
    }

    const double cmd_age_s = (now - _latest_cmd_vel_stamp).seconds();
    if (_nav2_cmd_timeout_s > 0.0 && cmd_age_s > _nav2_cmd_timeout_s) {
      _state = MissionState::Nav2StaleHold;
      publishGoto(Eigen::Vector3f{current_position.x(), current_position.y(), fixed_z_ned}, 0.0f);
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
        "Nav2 cmd_vel stale for %.2f s; holding position.", cmd_age_s);
      return;
    }

    const float heading = _vehicle_local_position->heading();
    const float yaw_rate = static_cast<float>(_latest_cmd_vel.angular.z);
    float forward_speed_m_s = static_cast<float>(_latest_cmd_vel.linear.x);
    float left_speed_m_s = static_cast<float>(_latest_cmd_vel.linear.y);
    forward_speed_m_s = std::clamp(
      forward_speed_m_s,
      -static_cast<float>(_nav2_max_cmd_speed_m_s),
      static_cast<float>(_nav2_max_cmd_speed_m_s));
    left_speed_m_s = std::clamp(
      left_speed_m_s,
      -static_cast<float>(_nav2_max_cmd_speed_m_s),
      static_cast<float>(_nav2_max_cmd_speed_m_s));
    float horizontal_speed_m_s = std::hypot(forward_speed_m_s, left_speed_m_s);
    horizontal_speed_m_s = std::min(horizontal_speed_m_s, static_cast<float>(_max_horizontal_velocity_m_s));
    horizontal_speed_m_s = std::min(horizontal_speed_m_s, static_cast<float>(_nav2_max_cmd_speed_m_s));

    const bool sharp_turn = std::fabs(yaw_rate) >= static_cast<float>(_nav2_turn_in_place_yaw_rate_rad_s);
    if (sharp_turn) {
      forward_speed_m_s *= static_cast<float>(_nav2_turn_speed_scale);
      left_speed_m_s *= static_cast<float>(_nav2_turn_speed_scale);
      horizontal_speed_m_s = std::hypot(forward_speed_m_s, left_speed_m_s);
      if (horizontal_speed_m_s < 0.08f) {
        forward_speed_m_s = 0.0f;
        left_speed_m_s = 0.0f;
        horizontal_speed_m_s = 0.0f;
      }
    }

    if (const auto front_distance = freshDistance(_front_obstacle_distance_m, _front_obstacle_stamp)) {
      const float slowdown_distance = static_cast<float>(_nav2_obstacle_slowdown_distance_m);
      if (slowdown_distance > static_cast<float>(_obstacle_stop_distance_m) &&
          *front_distance < slowdown_distance) {
        const float usable_range = slowdown_distance - static_cast<float>(_obstacle_stop_distance_m);
        const float clearance = std::max(0.0f, *front_distance - static_cast<float>(_obstacle_stop_distance_m));
        const float scale = std::clamp(clearance / usable_range, 0.0f, 1.0f);
        const float min_speed = static_cast<float>(_nav2_obstacle_min_speed_m_s);
        if (forward_speed_m_s > 0.0f) {
          forward_speed_m_s *= scale;
        }
        horizontal_speed_m_s = std::hypot(forward_speed_m_s, left_speed_m_s);
        if (scale > 0.35f && horizontal_speed_m_s > 0.02f && std::fabs(left_speed_m_s) < 0.02f) {
          forward_speed_m_s = std::max(forward_speed_m_s, min_speed);
          horizontal_speed_m_s = std::hypot(forward_speed_m_s, left_speed_m_s);
        }
      }
    }

    const float lookahead_time_s = static_cast<float>(std::max(0.1, _nav2_lookahead_time_s));
    const float predicted_yaw_delta = yaw_rate * lookahead_time_s;
    float lookahead_m = horizontal_speed_m_s * lookahead_time_s;
    if (horizontal_speed_m_s > 0.02f) {
      lookahead_m = std::max(static_cast<float>(_nav2_min_lookahead_m), lookahead_m);
      if (sharp_turn) {
        lookahead_m = std::min(lookahead_m, static_cast<float>(_nav2_turn_max_lookahead_m));
      }
    }

    Eigen::Vector3f target = current_position;
    if (horizontal_speed_m_s > 0.02f) {
      const float travel_heading = heading + 0.5f * predicted_yaw_delta;
      const float command_scale = lookahead_m / std::max(horizontal_speed_m_s, 0.001f);
      const float forward_offset_m = forward_speed_m_s * command_scale;
      const float left_offset_m = left_speed_m_s * command_scale;
      target.x() += std::cos(travel_heading) * forward_offset_m - std::sin(travel_heading) * left_offset_m;
      target.y() += std::sin(travel_heading) * forward_offset_m + std::cos(travel_heading) * left_offset_m;
    }
    target.z() = fixed_z_ned;

    std::optional<float> heading_target{};
    if (std::fabs(yaw_rate) > 0.01f) {
      heading_target = heading + predicted_yaw_delta;
    } else if (horizontal_speed_m_s > 0.02f) {
      heading_target = std::atan2(target.y() - current_position.y(), target.x() - current_position.x());
    }

    _state = MissionState::FollowingNav2Command;
    _goto_setpoint->update(
      target,
      heading_target,
      horizontal_speed_m_s,
      static_cast<float>(_max_vertical_velocity_m_s),
      px4_ros2::degToRad(static_cast<float>(_max_heading_rate_deg_s)));
  }

  bool nav2ControlAuthorityOk(const rclcpp::Time & now)
  {
    if (!_nav2_require_ros2_control) {
      publishNav2ControlAuthority(true);
      return true;
    }

    if (_latest_vehicle_status_stamp.nanoseconds() == 0) {
      publishNav2ControlAuthority(false);
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
        "Nav2 control authority unknown: no VehicleStatus on %s.",
        _nav2_vehicle_status_topic.c_str());
      return false;
    }

    const double status_age_s = (now - _latest_vehicle_status_stamp).seconds();
    if (_nav2_status_timeout_s > 0.0 && status_age_s > _nav2_status_timeout_s) {
      publishNav2ControlAuthority(false);
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
        "Nav2 control authority stale: VehicleStatus age %.2f s > %.2f s.",
        status_age_s, _nav2_status_timeout_s);
      return false;
    }

    const uint8_t expected_nav_state =
      _nav2_required_nav_state >= 0
      ? static_cast<uint8_t>(_nav2_required_nav_state)
      : static_cast<uint8_t>(id());
    const bool expected_state = _latest_vehicle_status.nav_state == expected_nav_state;
    const bool armed =
      _latest_vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const bool healthy = !_latest_vehicle_status.failsafe;
    const bool ok = expected_state && armed && healthy && isActive();

    publishNav2ControlAuthority(ok);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
        "Nav2 movement blocked: expected nav_state=%u active=%s, got nav_state=%u armed=%s failsafe=%s. "
        "PX4 manual/RC modes remain under PX4 control; QGC Go-to must be routed through ROS2 Autonomy/Nav2.",
        static_cast<unsigned>(expected_nav_state),
        isActive() ? "true" : "false",
        static_cast<unsigned>(_latest_vehicle_status.nav_state),
        armed ? "true" : "false",
        _latest_vehicle_status.failsafe ? "true" : "false");
      return false;
    }

    if (_latest_vehicle_control_mode_stamp.nanoseconds() != 0) {
      const double control_mode_age_s = (now - _latest_vehicle_control_mode_stamp).seconds();
      if (_nav2_status_timeout_s <= 0.0 || control_mode_age_s <= _nav2_status_timeout_s) {
        RCLCPP_DEBUG_THROTTLE(node().get_logger(), *node().get_clock(), 2000,
          "Nav2 authority ok: nav_state=%u control_mode source_id=%u pos=%s vel=%s offboard=%s auto=%s.",
          static_cast<unsigned>(_latest_vehicle_status.nav_state),
          static_cast<unsigned>(_latest_vehicle_control_mode.source_id),
          _latest_vehicle_control_mode.flag_control_position_enabled ? "true" : "false",
          _latest_vehicle_control_mode.flag_control_velocity_enabled ? "true" : "false",
          _latest_vehicle_control_mode.flag_control_offboard_enabled ? "true" : "false",
          _latest_vehicle_control_mode.flag_control_auto_enabled ? "true" : "false");
      }
    }

    return true;
  }

  void publishNav2ControlAuthority(bool ok)
  {
    if (!_nav2_control_authority_pub) {
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = ok;
    _nav2_control_authority_pub->publish(msg);
  }

  bool handleNav2EmergencyObstacle()
  {
    if (!_enable_obstacle_hold) {
      return false;
    }

    const std::optional<float> obstacle_distance = effectiveObstacleDistance();
    const std::optional<float> forward_obstacle_distance = freshDistance(
      _front_obstacle_distance_m, _front_obstacle_stamp);
    const float distance = forward_obstacle_distance.value_or(
      obstacle_distance.value_or(std::numeric_limits<float>::infinity()));
    if (!std::isfinite(distance)) {
      return false;
    }

    if (distance <= static_cast<float>(_obstacle_abort_distance_m)) {
      _state = MissionState::EmergencyStop;
      Eigen::Vector3f hold_target = _vehicle_local_position->positionNed();
      const float heading = _vehicle_local_position->heading();
      const float retreat_distance = static_cast<float>(std::max(0.0, _nav2_emergency_retreat_distance_m));
      hold_target.x() -= std::cos(heading) * retreat_distance;
      hold_target.y() -= std::sin(heading) * retreat_distance;
      hold_target.z() = -static_cast<float>(std::fabs(_nav2_fixed_altitude_m));
      _goto_setpoint->update(
        hold_target,
        std::nullopt,
        static_cast<float>(_nav2_emergency_retreat_speed_m_s),
        static_cast<float>(_max_vertical_velocity_m_s),
        px4_ros2::degToRad(static_cast<float>(_max_heading_rate_deg_s)));
      RCLCPP_ERROR_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
        "Emergency obstacle retreat in Nav2 mode: %.2f m <= abort threshold %.2f m. "
        "Keeping ROS2 control active and backing away %.2f m.",
        distance, _obstacle_abort_distance_m, retreat_distance);
      return true;
    }

    if (distance <= static_cast<float>(_obstacle_stop_distance_m)) {
      const bool fresh_nav2_cmd =
        _latest_cmd_vel_stamp.nanoseconds() != 0 &&
        (_nav2_cmd_timeout_s <= 0.0 ||
        (node().get_clock()->now() - _latest_cmd_vel_stamp).seconds() <= _nav2_cmd_timeout_s);
      const bool nav2_requests_non_forward_recovery =
        fresh_nav2_cmd &&
        std::fabs(static_cast<float>(_latest_cmd_vel.linear.x)) < 0.03f &&
        (std::fabs(static_cast<float>(_latest_cmd_vel.angular.z)) > 0.05f ||
        std::fabs(static_cast<float>(_latest_cmd_vel.linear.y)) > 0.03f);
      if (nav2_requests_non_forward_recovery) {
        RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
          "Obstacle within stop distance (%.2f m <= %.2f m), but allowing Nav2 non-forward recovery.",
          distance, _obstacle_stop_distance_m);
        return false;
      }

      _state = MissionState::EmergencyStop;
      Eigen::Vector3f hold_target = _vehicle_local_position->positionNed();
      hold_target.z() = -static_cast<float>(std::fabs(_nav2_fixed_altitude_m));
      _goto_setpoint->update(
        hold_target,
        std::nullopt,
        0.0f,
        0.0f,
        px4_ros2::degToRad(static_cast<float>(_max_heading_rate_deg_s)));
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
        "Emergency obstacle hold in Nav2 mode: %.2f m <= stop threshold %.2f m.",
        distance, _obstacle_stop_distance_m);
      return true;
    }

    return false;
  }

  bool handleObstacleLogic()
  {
    if (_state == MissionState::Aborted) {
      return true;
    }

    if (!_enable_obstacle_hold) {
      return false;
    }

    const std::optional<float> obstacle_distance = effectiveObstacleDistance();
    const std::optional<float> forward_obstacle_distance = freshDistance(
      _front_obstacle_distance_m, _front_obstacle_stamp);
    if (!obstacle_distance.has_value() || *obstacle_distance <= 0.0f) {
      return false;
    }

    const float forward_distance = forward_obstacle_distance.value_or(*obstacle_distance);

    if (handleLocalAvoidance()) {
      return true;
    }

    if (forward_distance <= static_cast<float>(_obstacle_abort_distance_m)) {
      if (_state != MissionState::Aborted) {
        _state = MissionState::Aborted;
        RCLCPP_ERROR(node().get_logger(),
          "Forward obstacle too close: %.2f m <= abort threshold %.2f m. Aborting autonomy mode.",
          forward_distance, _obstacle_abort_distance_m);
      }
      completed(px4_ros2::Result::ModeFailureOther);
      return true;
    }

    const float hold_distance = forward_distance;

    if (hold_distance <= static_cast<float>(_obstacle_stop_distance_m)) {
      if (!_obstacle_hold_started) {
        _obstacle_hold_started = true;
        _hold_start = node().get_clock()->now();
        clearAvoidance();
        _state = MissionState::ObstacleHold;
        RCLCPP_WARN(node().get_logger(),
          "Obstacle hold triggered: %.2f m <= stop threshold %.2f m.",
          hold_distance, _obstacle_stop_distance_m);
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
        "Obstacle cleared at %.2f m. Resuming mission.", *obstacle_distance);
    }

    return false;
  }

  bool handleLocalAvoidance()
  {
    if (!_enable_local_avoidance || _mission_waypoints.empty()) {
      return false;
    }

    if (_state == MissionState::AvoidingObstacle && _avoidance_target_ned_m.has_value()) {
      const Eigen::Vector3f avoidance_target = *_avoidance_target_ned_m;
      publishGoto(avoidance_target, static_cast<float>(_avoidance_speed_m_s));

      if (positionReached(avoidance_target)) {
        RCLCPP_INFO(node().get_logger(),
          "Avoidance waypoint reached. Returning to mission waypoint %zu.",
          _waypoint_index + 1);
        clearAvoidance();
        _state = MissionState::Transit;
        return true;
      }

      return true;
    }

    const std::optional<float> front_distance = freshDistance(
      _front_obstacle_distance_m, _front_obstacle_stamp);
    if (!front_distance.has_value() ||
        *front_distance > static_cast<float>(_avoidance_trigger_distance_m)) {
      return false;
    }

    const std::optional<Eigen::Vector3f> avoidance_target = planAvoidanceTarget();
    if (!avoidance_target.has_value()) {
      return false;
    }

    _avoidance_target_ned_m = *avoidance_target;
    _state = MissionState::AvoidingObstacle;
    _obstacle_hold_started = false;
    RCLCPP_WARN(node().get_logger(),
      "Local avoidance planned: front %.2f m <= %.2f m, side=%s, target=[%.2f, %.2f, %.2f]",
      *front_distance, _avoidance_trigger_distance_m, _avoidance_side.c_str(),
      _avoidance_target_ned_m->x(), _avoidance_target_ned_m->y(), _avoidance_target_ned_m->z());
    publishGoto(*_avoidance_target_ned_m, static_cast<float>(_avoidance_speed_m_s));
    return true;
  }

  std::optional<Eigen::Vector3f> planAvoidanceTarget()
  {
    if (_waypoint_index >= _mission_waypoints.size()) {
      return std::nullopt;
    }

    const Eigen::Vector3f position = _vehicle_local_position->positionNed();
    const Eigen::Vector3f mission_target = _mission_waypoints[_waypoint_index].position;
    Eigen::Vector2f forward = mission_target.head<2>() - position.head<2>();
    if (forward.norm() < 0.25f) {
      const float heading = _vehicle_local_position->heading();
      forward = Eigen::Vector2f{std::cos(heading), std::sin(heading)};
    }
    if (forward.norm() < 0.01f) {
      return std::nullopt;
    }
    forward.normalize();

    const float left_distance = freshDistance(_left_obstacle_distance_m, _left_obstacle_stamp)
      .value_or(-1.0f);
    const float right_distance = freshDistance(_right_obstacle_distance_m, _right_obstacle_stamp)
      .value_or(-1.0f);

    const bool left_clear = left_distance >= static_cast<float>(_avoidance_clearance_m);
    const bool right_clear = right_distance >= static_cast<float>(_avoidance_clearance_m);
    if (!left_clear && !right_clear) {
      RCLCPP_WARN_THROTTLE(node().get_logger(), *node().get_clock(), 1000,
        "Local avoidance has no clear side corridor: left=%.2f m right=%.2f m required=%.2f m.",
        left_distance, right_distance, _avoidance_clearance_m);
      return std::nullopt;
    }

    const bool use_left = left_clear && (!right_clear || left_distance >= right_distance);
    const Eigen::Vector2f lateral = use_left
      ? Eigen::Vector2f{-forward.y(), forward.x()}
      : Eigen::Vector2f{forward.y(), -forward.x()};
    _avoidance_side = use_left ? "left" : "right";

    Eigen::Vector3f target = position;
    target.head<2>() += forward * static_cast<float>(_avoidance_forward_offset_m);
    target.head<2>() += lateral * static_cast<float>(_avoidance_lateral_offset_m);
    target.z() = mission_target.z();
    return target;
  }

  void publishGoto(const Eigen::Vector3f & target, float horizontal_speed_m_s)
  {
    const Eigen::Vector2f delta_xy = target.head<2>() - _vehicle_local_position->positionNed().head<2>();
    std::optional<float> heading_rad{};
    if (delta_xy.norm() > 0.5f) {
      heading_rad = std::atan2(delta_xy.y(), delta_xy.x());
    }

    _goto_setpoint->update(
      target,
      heading_rad,
      horizontal_speed_m_s,
      static_cast<float>(_max_vertical_velocity_m_s),
      px4_ros2::degToRad(static_cast<float>(_max_heading_rate_deg_s)));
  }

  void clearAvoidance()
  {
    _avoidance_target_ned_m.reset();
    _avoidance_side = "none";
  }

  std::optional<float> freshDistance(float value, const rclcpp::Time & stamp)
  {
    if (value <= 0.0f || stamp.nanoseconds() == 0) {
      return std::nullopt;
    }
    if (_obstacle_data_timeout_s > 0.0 &&
        (node().get_clock()->now() - stamp).seconds() > _obstacle_data_timeout_s) {
      return std::nullopt;
    }
    return value;
  }

  std::optional<float> effectiveObstacleDistance()
  {
    std::optional<float> best{};
    auto consider = [&best](float value) {
      if (value <= 0.0f) {
        return;
      }
      if (!best.has_value() || value < *best) {
        best = value;
      }
    };

    if (const auto distance = freshDistance(_latest_obstacle_distance_m, _latest_obstacle_stamp)) {
      consider(*distance);
    }
    if (const auto distance = freshDistance(_front_obstacle_distance_m, _front_obstacle_stamp)) {
      consider(*distance);
    }
    return best;
  }

  void buildMission()
  {
    const float x0 = _start_position_ned_m.x();
    const float y0 = _start_position_ned_m.y();
    const float mission_size_m = static_cast<float>(_mission_size_m);
    const float z_target = -static_cast<float>(std::fabs(_mission_altitude_m));

    _mission_waypoints.clear();
    if (_manual_mission_received && !_manual_mission_waypoints_ned.empty()) {
      _mission_waypoints = _manual_mission_waypoints_ned;
      return;
    }

    if (_manual_target_received && _manual_target_ned_m.has_value()) {
      _mission_waypoints.push_back(makeWaypoint(*_manual_target_ned_m));
      return;
    }

    if (!_configured_mission_waypoints_ned.empty()) {
      _mission_waypoints = _configured_mission_waypoints_ned;
      return;
    }

    _mission_waypoints.push_back(makeWaypoint(Eigen::Vector3f{x0, y0, z_target}));
    _mission_waypoints.push_back(makeWaypoint(Eigen::Vector3f{x0 + mission_size_m, y0, z_target}));
    _mission_waypoints.push_back(makeWaypoint(Eigen::Vector3f{x0 + mission_size_m, y0 + mission_size_m, z_target}));
    _mission_waypoints.push_back(makeWaypoint(Eigen::Vector3f{x0, y0 + mission_size_m, z_target}));
    _mission_waypoints.push_back(makeWaypoint(
      Eigen::Vector3f{x0, y0, z_target},
      std::nullopt,
      static_cast<float>(_hold_time_s)));
  }

  void resetMissionProgress()
  {
    _waypoint_index = 0;
    _hold_started = false;
    _mission_completed_reported = false;
    _rtl_sent = false;
    _obstacle_hold_started = false;
    _hold_start = rclcpp::Time{};
    clearAvoidance();
    _last_reported_waypoint_index = static_cast<std::size_t>(-1);
    _mission_start = node().get_clock()->now();
    _state = MissionState::Transit;
  }

  MissionWaypoint makeWaypoint(
    const Eigen::Vector3f & position,
    std::optional<float> speed_m_s = std::nullopt,
    std::optional<float> hold_time_s = std::nullopt) const
  {
    MissionWaypoint waypoint;
    waypoint.position = position;
    if (speed_m_s.has_value() && *speed_m_s > 0.0f) {
      waypoint.speed_m_s = *speed_m_s;
    }
    if (hold_time_s.has_value() && *hold_time_s >= 0.0f) {
      waypoint.hold_time_s = *hold_time_s;
    }
    return waypoint;
  }

  float horizontalSpeedFor(const MissionWaypoint & waypoint) const
  {
    return waypoint.speed_m_s.value_or(static_cast<float>(_max_horizontal_velocity_m_s));
  }

  float holdTimeFor(const MissionWaypoint & waypoint, bool is_last_waypoint) const
  {
    if (waypoint.hold_time_s.has_value()) {
      return *waypoint.hold_time_s;
    }
    return is_last_waypoint ? static_cast<float>(_hold_time_s) : 0.0f;
  }

  std::vector<MissionWaypoint> parseMissionWaypoints(const std::string & text)
  {
    std::string normalized = text;
    for (char & ch : normalized) {
      if (ch == ',' || ch == ';' || ch == '[' || ch == ']' || ch == '(' || ch == ')') {
        ch = ' ';
      }
    }

    std::istringstream stream(normalized);
    std::vector<float> values;
    float value = 0.0f;
    while (stream >> value) {
      values.push_back(value);
    }

    if (values.empty()) {
      return {};
    }

    if (values.size() % 3 != 0) {
      RCLCPP_ERROR(node().get_logger(),
        "mission_waypoints_ned ignored: expected x,y,z triples but got %zu values.",
        values.size());
      return {};
    }

    std::vector<MissionWaypoint> waypoints;
    waypoints.reserve(values.size() / 3);
    for (std::size_t i = 0; i < values.size(); i += 3) {
      waypoints.push_back(makeWaypoint(Eigen::Vector3f{values[i], values[i + 1], values[i + 2]}));
    }
    RCLCPP_INFO(node().get_logger(),
      "Loaded %zu configured mission waypoints from mission_waypoints_ned.",
      waypoints.size());
    return waypoints;
  }

  std::vector<MissionWaypoint> parseMissionFile(const std::string & path)
  {
    std::vector<MissionWaypoint> waypoints;

    try {
      const YAML::Node root = YAML::LoadFile(path);
      if (!root || !root.IsMap()) {
        RCLCPP_ERROR(node().get_logger(),
          "mission_file '%s' ignored: expected a YAML mapping at the top level.",
          path.c_str());
        return {};
      }

      applyMissionFileParameters(root);

      const YAML::Node waypoint_nodes = root["waypoints"];
      if (!waypoint_nodes || !waypoint_nodes.IsSequence()) {
        RCLCPP_ERROR(node().get_logger(),
          "mission_file '%s' contains no 'waypoints' sequence.",
          path.c_str());
        return {};
      }

      waypoints.reserve(waypoint_nodes.size());
      for (std::size_t i = 0; i < waypoint_nodes.size(); ++i) {
        const YAML::Node waypoint_node = waypoint_nodes[i];
        if (!waypoint_node || !waypoint_node.IsMap()) {
          RCLCPP_ERROR(node().get_logger(),
            "mission_file '%s' waypoint %zu ignored: expected a mapping with x/y/z.",
            path.c_str(), i + 1);
          continue;
        }

        const auto x = yamlValue<float>(waypoint_node, "x");
        const auto y = yamlValue<float>(waypoint_node, "y");
        const auto z = yamlValue<float>(waypoint_node, "z");
        if (!x.has_value() || !y.has_value() || !z.has_value()) {
          RCLCPP_ERROR(node().get_logger(),
            "mission_file '%s' waypoint %zu ignored: x/y/z are required.",
            path.c_str(), i + 1);
          continue;
        }

        waypoints.push_back(makeWaypoint(
          Eigen::Vector3f{*x, *y, *z},
          yamlValue<float>(waypoint_node, "speed"),
          yamlValue<float>(waypoint_node, "hold_time")));
      }
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(node().get_logger(),
        "mission_file '%s' ignored: YAML parse error: %s",
        path.c_str(), e.what());
      return {};
    }

    if (waypoints.empty()) {
      RCLCPP_ERROR(node().get_logger(),
        "mission_file '%s' contains no supported waypoints. Expected 'waypoints:' entries with x/y/z.",
        path.c_str());
      return {};
    }

    RCLCPP_INFO(node().get_logger(),
      "Loaded %zu configured mission waypoints from mission_file '%s'. YAML mission parameters and per-waypoint speed/hold_time are applied when present.",
      waypoints.size(), path.c_str());
    return waypoints;
  }

  template<typename T>
  std::optional<T> yamlValue(const YAML::Node & node, const std::string & key)
  {
    const YAML::Node value = node[key];
    if (!value) {
      return std::nullopt;
    }

    try {
      return value.as<T>();
    } catch (const YAML::Exception & e) {
      RCLCPP_ERROR(this->node().get_logger(),
        "mission_file key '%s' ignored: %s",
        key.c_str(), e.what());
      return std::nullopt;
    }
  }

  template<typename T>
  void applyYamlValue(const YAML::Node & root, const std::string & key, T & target)
  {
    if (const auto value = yamlValue<T>(root, key)) {
      target = *value;
    }
  }

  void applyMissionFileParameters(const YAML::Node & root)
  {
    applyYamlValue(root, "mission_altitude_m", _mission_altitude_m);
    applyYamlValue(root, "hold_time_s", _hold_time_s);
    applyYamlValue(root, "acceptance_radius_m", _acceptance_radius_m);
    applyYamlValue(root, "max_horizontal_velocity_m_s", _max_horizontal_velocity_m_s);
    applyYamlValue(root, "max_vertical_velocity_m_s", _max_vertical_velocity_m_s);
    applyYamlValue(root, "max_heading_rate_deg_s", _max_heading_rate_deg_s);
    applyYamlValue(root, "mission_timeout_s", _mission_timeout_s);
    applyYamlValue(root, "auto_rtl_after_finish", _auto_rtl_after_finish);
    applyYamlValue(root, "obstacle_stop_distance_m", _obstacle_stop_distance_m);
    applyYamlValue(root, "obstacle_abort_distance_m", _obstacle_abort_distance_m);
    applyYamlValue(root, "obstacle_hold_timeout_s", _obstacle_hold_timeout_s);
    applyYamlValue(root, "enable_obstacle_hold", _enable_obstacle_hold);
    applyYamlValue(root, "enable_local_avoidance", _enable_local_avoidance);
    applyYamlValue(root, "avoidance_trigger_distance_m", _avoidance_trigger_distance_m);
    applyYamlValue(root, "avoidance_clearance_m", _avoidance_clearance_m);
    applyYamlValue(root, "avoidance_lateral_offset_m", _avoidance_lateral_offset_m);
    applyYamlValue(root, "avoidance_forward_offset_m", _avoidance_forward_offset_m);
    applyYamlValue(root, "avoidance_speed_m_s", _avoidance_speed_m_s);
    applyYamlValue(root, "obstacle_data_timeout_s", _obstacle_data_timeout_s);
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
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _front_obstacle_distance_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _left_obstacle_distance_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _right_obstacle_distance_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _up_obstacle_distance_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _down_obstacle_distance_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _target_point_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr _waypoints_sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr _vehicle_control_mode_sub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _nav2_control_authority_pub;

  Eigen::Vector3f _start_position_ned_m{0.0f, 0.0f, 0.0f};
  std::vector<MissionWaypoint> _mission_waypoints;
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
  std::string _front_obstacle_distance_topic{"/perception/front_obstacle_distance"};
  std::string _left_obstacle_distance_topic{"/perception/left_obstacle_distance"};
  std::string _right_obstacle_distance_topic{"/perception/right_obstacle_distance"};
  std::string _up_obstacle_distance_topic{"/perception/up_obstacle_distance"};
  std::string _down_obstacle_distance_topic{"/perception/down_obstacle_distance"};
  std::string _target_point_topic{"/autonomy/target_ned"};
  std::string _waypoints_topic{"/autonomy/waypoints_ned"};
  std::string _control_source{"mission"};
  std::string _nav2_cmd_vel_topic{"/cmd_vel"};
  std::string _mission_file{};
  std::string _mission_waypoints_ned_text{};
  double _obstacle_stop_distance_m{2.0};
  double _obstacle_abort_distance_m{1.0};
  double _obstacle_hold_timeout_s{5.0};
  bool _enable_obstacle_hold{true};
  bool _enable_local_avoidance{true};
  double _avoidance_trigger_distance_m{3.0};
  double _avoidance_clearance_m{2.0};
  double _avoidance_lateral_offset_m{1.5};
  double _avoidance_forward_offset_m{2.0};
  double _avoidance_speed_m_s{0.45};
  double _obstacle_data_timeout_s{1.0};
  double _nav2_fixed_altitude_m{4.0};
  double _nav2_cmd_timeout_s{0.5};
  double _nav2_lookahead_time_s{1.0};
  double _nav2_min_lookahead_m{0.35};
  double _nav2_max_cmd_speed_m_s{0.45};
  double _nav2_turn_in_place_yaw_rate_rad_s{0.35};
  double _nav2_turn_speed_scale{0.35};
  double _nav2_turn_max_lookahead_m{0.25};
  double _nav2_obstacle_slowdown_distance_m{3.0};
  double _nav2_obstacle_min_speed_m_s{0.12};
  double _nav2_emergency_retreat_distance_m{0.8};
  double _nav2_emergency_retreat_speed_m_s{0.25};
  bool _nav2_require_ros2_control{true};
  int _nav2_required_nav_state{-1};
  std::string _nav2_vehicle_status_topic{"/fmu/out/vehicle_status_v4"};
  std::string _nav2_vehicle_control_mode_topic{"/fmu/out/vehicle_control_mode"};
  std::string _nav2_control_authority_topic{"/autonomy/nav2_control_authority_ok"};
  double _nav2_status_timeout_s{1.0};
  bool _accept_runtime_target{true};
  bool _hold_started{false};
  bool _mission_completed_reported{false};
  bool _rtl_sent{false};
  bool _obstacle_hold_started{false};
  MissionState _state{MissionState::Transit};
  rclcpp::Time _mission_start{};
  rclcpp::Time _hold_start{};
  rclcpp::Time _latest_obstacle_stamp{};
  rclcpp::Time _front_obstacle_stamp{};
  rclcpp::Time _left_obstacle_stamp{};
  rclcpp::Time _right_obstacle_stamp{};
  rclcpp::Time _up_obstacle_stamp{};
  rclcpp::Time _down_obstacle_stamp{};
  rclcpp::Time _latest_cmd_vel_stamp{};
  rclcpp::Time _latest_vehicle_status_stamp{};
  rclcpp::Time _latest_vehicle_control_mode_stamp{};
  float _latest_obstacle_distance_m{-1.0f};
  float _front_obstacle_distance_m{-1.0f};
  float _left_obstacle_distance_m{-1.0f};
  float _right_obstacle_distance_m{-1.0f};
  float _up_obstacle_distance_m{-1.0f};
  float _down_obstacle_distance_m{-1.0f};
  geometry_msgs::msg::Twist _latest_cmd_vel{};
  px4_msgs::msg::VehicleStatus _latest_vehicle_status{};
  px4_msgs::msg::VehicleControlMode _latest_vehicle_control_mode{};
  std::optional<Eigen::Vector3f> _avoidance_target_ned_m{};
  std::string _avoidance_side{"none"};
  std::vector<MissionWaypoint> _configured_mission_waypoints_ned{};
  std::vector<MissionWaypoint> _manual_mission_waypoints_ned{};
  bool _manual_mission_received{false};
  bool _manual_target_received{false};
  std::optional<Eigen::Vector3f> _manual_target_ned_m{};
};
