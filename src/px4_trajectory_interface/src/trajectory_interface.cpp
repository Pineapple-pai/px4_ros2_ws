#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <string>

#include <Eigen/Core>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

class TrajectoryInterface : public rclcpp::Node
{
public:
  TrajectoryInterface()
  : Node("trajectory_interface")
  {
    declare_parameter("position_cmd_topic", "/planning/position_cmd");
    declare_parameter("vehicle_status_topic", "/fmu/out/vehicle_status_v4");
    declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    declare_parameter("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command");
    declare_parameter("vehicle_command_monitor_topic", "/fmu/out/vehicle_command");
    declare_parameter("vehicle_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    declare_parameter("publish_rate_hz", 50.0);
    declare_parameter("command_timeout_s", 0.25);
    declare_parameter("offboard_setpoint_warmup_count", 20);
    declare_parameter("auto_set_offboard", true);
    declare_parameter("auto_arm", false);
    declare_parameter("hold_position_on_timeout", true);
    declare_parameter("suspend_on_external_mode_command", true);
    declare_parameter("require_armed_before_offboard", true);
    declare_parameter("require_local_position_before_offboard", true);
    declare_parameter("max_offboard_start_horizontal_error_m", 1.2);
    declare_parameter("max_offboard_start_vertical_error_m", 0.8);
    declare_parameter("max_position_setpoint_step_horizontal_m", 0.0);
    declare_parameter("max_position_setpoint_step_vertical_m", 0.0);
    declare_parameter("align_planner_frame_to_px4_local", true);

    const auto position_cmd_topic = get_parameter("position_cmd_topic").as_string();
    const auto vehicle_status_topic = get_parameter("vehicle_status_topic").as_string();
    const auto offboard_control_mode_topic = get_parameter("offboard_control_mode_topic").as_string();
    const auto trajectory_setpoint_topic = get_parameter("trajectory_setpoint_topic").as_string();
    const auto vehicle_command_topic = get_parameter("vehicle_command_topic").as_string();
    const auto vehicle_command_monitor_topic =
      get_parameter("vehicle_command_monitor_topic").as_string();
    const auto vehicle_local_position_topic =
      get_parameter("vehicle_local_position_topic").as_string();

    _publish_rate_hz = static_cast<float>(get_parameter("publish_rate_hz").as_double());
    _command_timeout_s = static_cast<float>(get_parameter("command_timeout_s").as_double());
    _offboard_setpoint_warmup_count = get_parameter("offboard_setpoint_warmup_count").as_int();
    _auto_set_offboard = get_parameter("auto_set_offboard").as_bool();
    _auto_arm = get_parameter("auto_arm").as_bool();
    _hold_position_on_timeout = get_parameter("hold_position_on_timeout").as_bool();
    _suspend_on_external_mode_command =
      get_parameter("suspend_on_external_mode_command").as_bool();
    _require_armed_before_offboard = get_parameter("require_armed_before_offboard").as_bool();
    _require_local_position_before_offboard =
      get_parameter("require_local_position_before_offboard").as_bool();
    _max_offboard_start_horizontal_error_m = static_cast<float>(
      get_parameter("max_offboard_start_horizontal_error_m").as_double());
    _max_offboard_start_vertical_error_m = static_cast<float>(
      get_parameter("max_offboard_start_vertical_error_m").as_double());
    _max_position_setpoint_step_horizontal_m = static_cast<float>(
      get_parameter("max_position_setpoint_step_horizontal_m").as_double());
    _max_position_setpoint_step_vertical_m = static_cast<float>(
      get_parameter("max_position_setpoint_step_vertical_m").as_double());
    _align_planner_frame_to_px4_local =
      get_parameter("align_planner_frame_to_px4_local").as_bool();

    _offboard_mode_pub = create_publisher<px4_msgs::msg::OffboardControlMode>(
      offboard_control_mode_topic, 10);
    _trajectory_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      trajectory_setpoint_topic, 10);
    _vehicle_command_pub = create_publisher<px4_msgs::msg::VehicleCommand>(vehicle_command_topic, 10);

    _position_cmd_sub = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      position_cmd_topic,
      10,
      std::bind(&TrajectoryInterface::positionCommandCallback, this, std::placeholders::_1));
    _vehicle_status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic,
      rclcpp::QoS(5).best_effort(),
      std::bind(&TrajectoryInterface::vehicleStatusCallback, this, std::placeholders::_1));
    _vehicle_command_monitor_sub = create_subscription<px4_msgs::msg::VehicleCommand>(
      vehicle_command_monitor_topic,
      rclcpp::QoS(10).best_effort(),
      std::bind(&TrajectoryInterface::vehicleCommandMonitorCallback, this, std::placeholders::_1));
    _vehicle_local_position_sub = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      vehicle_local_position_topic,
      rclcpp::QoS(5).best_effort(),
      std::bind(&TrajectoryInterface::vehicleLocalPositionCallback, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration<double>(1.0 / std::max(1.0f, _publish_rate_hz));
    _timer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
      std::bind(&TrajectoryInterface::publishLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "Listening to %s and publishing PX4 Offboard setpoints on %s.",
      position_cmd_topic.c_str(),
      trajectory_setpoint_topic.c_str());
  }

private:
  void positionCommandCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    if (msg->trajectory_flag == quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_COMPLETED ||
      msg->trajectory_flag == quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_EMPTY)
    {
      activateCompletedHold();
      return;
    }

    if (_resume_on_next_command && !_persistent_control_inhibit) {
      _external_control_inhibited = false;
      _resume_on_next_command = false;
      RCLCPP_INFO(
        get_logger(),
        "Resuming trajectory setpoint output after receiving a fresh planner command.");
    }
    if (_external_control_inhibited) {
      return;
    }

    const Eigen::Vector3f planner_position_enu{
      static_cast<float>(msg->position.x),
      static_cast<float>(msg->position.y),
      static_cast<float>(msg->position.z)};
    maybeAlignPlannerFrame(planner_position_enu);
    const Eigen::Vector3f position_enu = plannerPositionToPx4Enu(planner_position_enu);
    const Eigen::Vector3f velocity_enu{
      static_cast<float>(msg->velocity.x),
      static_cast<float>(msg->velocity.y),
      static_cast<float>(msg->velocity.z)};
    const Eigen::Vector3f acceleration_enu{
      static_cast<float>(msg->acceleration.x),
      static_cast<float>(msg->acceleration.y),
      static_cast<float>(msg->acceleration.z)};

    Eigen::Vector3f filtered_position_enu = position_enu;
    if (_have_vehicle_local_position) {
      filtered_position_enu = clampPositionSetpointStep(position_enu);
    }

    const Eigen::Vector3f position_ned = px4_ros2::positionEnuToNed(filtered_position_enu);
    const Eigen::Vector3f velocity_ned = px4_ros2::positionEnuToNed(velocity_enu);
    const Eigen::Vector3f acceleration_ned = px4_ros2::positionEnuToNed(acceleration_enu);
    const float yaw_ned = px4_ros2::yawEnuToNed(static_cast<float>(msg->yaw));
    const float yaw_rate_ned = px4_ros2::yawRateEnuToNed(static_cast<float>(msg->yaw_dot));

    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position = {position_ned.x(), position_ned.y(), position_ned.z()};
    setpoint.velocity = {velocity_ned.x(), velocity_ned.y(), velocity_ned.z()};
    setpoint.acceleration = {acceleration_ned.x(), acceleration_ned.y(), acceleration_ned.z()};
    setpoint.jerk = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()};
    setpoint.yaw = yaw_ned;
    setpoint.yawspeed = yaw_rate_ned;

    _latest_setpoint = setpoint;
    _latest_command_position_enu = filtered_position_enu;
    _last_command_time = now();
    _warmup_counter = std::min(_warmup_counter + 1, _offboard_setpoint_warmup_count);
    _have_command = true;
    _completed_hold_active = false;
    _control_suspended = _external_control_inhibited;
  }

  void activateCompletedHold()
  {
    if (_external_control_inhibited) {
      _have_command = false;
      _completed_hold_active = false;
      _control_suspended = true;
      return;
    }

    if (_have_vehicle_local_position) {
      const Eigen::Vector3f hold_position_ned = px4_ros2::positionEnuToNed(_latest_vehicle_position_enu);
      _latest_setpoint.position = {
        hold_position_ned.x(),
        hold_position_ned.y(),
        hold_position_ned.z()};
      _latest_command_position_enu = _latest_vehicle_position_enu;
    }

    _latest_setpoint.velocity = {0.0f, 0.0f, 0.0f};
    _latest_setpoint.acceleration = {0.0f, 0.0f, 0.0f};
    _latest_setpoint.yawspeed = 0.0f;
    _last_command_time = now();
    _have_command = true;
    _completed_hold_active = true;
    _control_suspended = false;
  }

  Eigen::Vector3f clampPositionSetpointStep(const Eigen::Vector3f & desired_position_enu) const
  {
    Eigen::Vector3f clamped_position_enu = desired_position_enu;
    Eigen::Vector3f delta = desired_position_enu - _latest_vehicle_position_enu;

    if (_max_position_setpoint_step_horizontal_m > 0.0f) {
      const float horizontal_norm = std::hypot(delta.x(), delta.y());
      if (horizontal_norm > _max_position_setpoint_step_horizontal_m && horizontal_norm > 1e-4f) {
        const float scale = _max_position_setpoint_step_horizontal_m / horizontal_norm;
        clamped_position_enu.x() = _latest_vehicle_position_enu.x() + (delta.x() * scale);
        clamped_position_enu.y() = _latest_vehicle_position_enu.y() + (delta.y() * scale);
      }
    }

    if (_max_position_setpoint_step_vertical_m > 0.0f) {
      const float vertical_delta = desired_position_enu.z() - _latest_vehicle_position_enu.z();
      if (std::fabs(vertical_delta) > _max_position_setpoint_step_vertical_m) {
        clamped_position_enu.z() =
          _latest_vehicle_position_enu.z() +
          std::copysign(_max_position_setpoint_step_vertical_m, vertical_delta);
      }
    }

    return clamped_position_enu;
  }

  void maybeAlignPlannerFrame(const Eigen::Vector3f & planner_position_enu)
  {
    if (!_align_planner_frame_to_px4_local || !_have_vehicle_local_position) {
      return;
    }

    const bool vehicle_in_offboard =
      _have_vehicle_status &&
      _vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    const bool should_realign =
      !_have_planner_to_px4_enu_offset || (_pending_frame_realign && !vehicle_in_offboard);
    if (!should_realign) {
      return;
    }

    _planner_to_px4_enu_offset = _latest_vehicle_position_enu - planner_position_enu;
    _have_planner_to_px4_enu_offset = true;
    _pending_frame_realign = false;
    RCLCPP_INFO(
      get_logger(),
      "Aligned planner frame to PX4 local ENU with offset [%.2f, %.2f, %.2f] m.",
      _planner_to_px4_enu_offset.x(),
      _planner_to_px4_enu_offset.y(),
      _planner_to_px4_enu_offset.z());
  }

  Eigen::Vector3f plannerPositionToPx4Enu(const Eigen::Vector3f & planner_position_enu) const
  {
    if (!_align_planner_frame_to_px4_local || !_have_planner_to_px4_enu_offset) {
      return planner_position_enu;
    }
    return planner_position_enu + _planner_to_px4_enu_offset;
  }

  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    if (_have_vehicle_status &&
      _vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
      msg->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      _control_suspended = true;
    }
    _vehicle_status = *msg;
    _have_vehicle_status = true;

    if (_external_control_inhibited &&
      _vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
    {
      _external_control_inhibited = false;
      _persistent_control_inhibit = false;
      _resume_on_next_command = false;
      _control_suspended = !_have_command;
      _have_planner_to_px4_enu_offset = false;
      _pending_frame_realign = true;
      RCLCPP_INFO(
        get_logger(),
        "Cleared trajectory output inhibit after PX4 reported disarmed.");
    }
  }

  void vehicleCommandMonitorCallback(const px4_msgs::msg::VehicleCommand::SharedPtr msg)
  {
    if (!_suspend_on_external_mode_command) {
      return;
    }

    const auto command = msg->command;
    bool should_suspend = false;
    const char * reason = "";
    bool persistent_inhibit = false;

    if (command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND) {
      should_suspend = true;
      reason = "LAND command";
      persistent_inhibit = true;
    } else if (command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH) {
      should_suspend = true;
      reason = "RTL command";
      persistent_inhibit = true;
    } else if (command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM &&
      msg->param1 < 0.5f)
    {
      should_suspend = true;
      reason = "disarm command";
      persistent_inhibit = true;
    } else if (command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
      std::fabs(msg->param2 - 6.0f) > 0.5f)
    {
      should_suspend = true;
      reason = "non-Offboard mode command";
    }

    if (!should_suspend) {
      return;
    }

    if (!_external_control_inhibited) {
      RCLCPP_WARN(
        get_logger(),
        "Suspending trajectory setpoint output after external %s.", reason);
    }
    _external_control_inhibited = true;
    _persistent_control_inhibit = persistent_inhibit;
    _resume_on_next_command = !persistent_inhibit;
    _control_suspended = true;
    _have_command = false;
    _warmup_counter = 0;
    _pending_frame_realign = true;
  }

  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg->xy_valid || !msg->z_valid) {
      _have_vehicle_local_position = false;
      return;
    }

    // PX4 VehicleLocalPosition is NED. Ego-Planner/position_cmd is expected as ENU.
    _latest_vehicle_position_enu = Eigen::Vector3f{
      static_cast<float>(msg->y),
      static_cast<float>(msg->x),
      static_cast<float>(-msg->z)};
    _have_vehicle_local_position = true;
  }

  void publishLoop()
  {
    if (!_have_command || _control_suspended || _external_control_inhibited) {
      return;
    }

    px4_msgs::msg::TrajectorySetpoint setpoint = _latest_setpoint;
    const bool command_fresh =
      ((now() - _last_command_time).seconds() <= static_cast<double>(_command_timeout_s));

    if (!command_fresh && !_hold_position_on_timeout && !_completed_hold_active) {
      return;
    }

    if (!command_fresh) {
      setpoint.velocity = {0.0f, 0.0f, 0.0f};
      setpoint.acceleration = {0.0f, 0.0f, 0.0f};
      setpoint.yawspeed = 0.0f;
    }

    const auto timestamp_us = static_cast<uint64_t>(now().nanoseconds() / 1000);

    px4_msgs::msg::OffboardControlMode offboard_mode{};
    offboard_mode.timestamp = timestamp_us;
    offboard_mode.position = true;
    offboard_mode.velocity = true;
    offboard_mode.acceleration = true;
    offboard_mode.attitude = false;
    offboard_mode.body_rate = false;
    offboard_mode.thrust_and_torque = false;
    offboard_mode.direct_actuator = false;
    _offboard_mode_pub->publish(offboard_mode);

    setpoint.timestamp = timestamp_us;
    _trajectory_pub->publish(setpoint);

    if (_auto_set_offboard && _warmup_counter >= _offboard_setpoint_warmup_count) {
      maybeSendOffboardRequest(timestamp_us);
    }
    if (_auto_arm && offboardStartGateAllowsModeChange()) {
      maybeSendArmRequest(timestamp_us);
    }
  }

  void maybeSendOffboardRequest(uint64_t timestamp_us)
  {
    if (_have_vehicle_status &&
      _vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      return;
    }

    if (!offboardStartGateAllowsModeChange()) {
      return;
    }

    if (_require_armed_before_offboard &&
      (!_have_vehicle_status ||
      _vehicle_status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Holding automatic Offboard request: vehicle is not armed.");
      return;
    }

    if ((_last_mode_request_time.nanoseconds() != 0) &&
      (now() - _last_mode_request_time).seconds() < 1.0)
    {
      return;
    }

    px4_msgs::msg::VehicleCommand command{};
    command.timestamp = timestamp_us;
    command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    command.param1 = 1.0f;
    command.param2 = 6.0f;
    command.target_system = 1;
    command.target_component = 1;
    command.source_system = 1;
    command.source_component = 1;
    command.from_external = true;
    _vehicle_command_pub->publish(command);
    _last_mode_request_time = now();
  }

  bool offboardStartGateAllowsModeChange()
  {
    if (!_require_local_position_before_offboard) {
      return true;
    }

    if (_have_vehicle_status &&
      _vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      return true;
    }

    if (!_have_vehicle_local_position) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Holding automatic Offboard/arm request: no valid PX4 local position yet.");
      return false;
    }

    const Eigen::Vector3f delta = _latest_command_position_enu - _latest_vehicle_position_enu;
    const float horizontal_error = std::hypot(delta.x(), delta.y());
    const float vertical_error = std::fabs(delta.z());
    if (horizontal_error > _max_offboard_start_horizontal_error_m ||
      vertical_error > _max_offboard_start_vertical_error_m)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Holding automatic Offboard/arm request: planner command and PX4 local ENU differ "
        "(horizontal=%.2f m, vertical=%.2f m; limits %.2f/%.2f m).",
        horizontal_error,
        vertical_error,
        _max_offboard_start_horizontal_error_m,
        _max_offboard_start_vertical_error_m);
      return false;
    }

    return true;
  }

  void maybeSendArmRequest(uint64_t timestamp_us)
  {
    if (_have_vehicle_status &&
      _vehicle_status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
      return;
    }

    if ((_last_arm_request_time.nanoseconds() != 0) &&
      (now() - _last_arm_request_time).seconds() < 1.0)
    {
      return;
    }

    px4_msgs::msg::VehicleCommand command{};
    command.timestamp = timestamp_us;
    command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    command.param1 = 1.0f;
    command.target_system = 1;
    command.target_component = 1;
    command.source_system = 1;
    command.source_component = 1;
    command.from_external = true;
    _vehicle_command_pub->publish(command);
    _last_arm_request_time = now();
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_mode_pub;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_pub;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr _position_cmd_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_monitor_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _vehicle_local_position_sub;
  rclcpp::TimerBase::SharedPtr _timer;

  px4_msgs::msg::TrajectorySetpoint _latest_setpoint{};
  px4_msgs::msg::VehicleStatus _vehicle_status{};
  Eigen::Vector3f _latest_command_position_enu{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _latest_vehicle_position_enu{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f _planner_to_px4_enu_offset{0.0f, 0.0f, 0.0f};
  rclcpp::Time _last_command_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time _last_mode_request_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time _last_arm_request_time{0, 0, RCL_ROS_TIME};
  float _publish_rate_hz{50.0f};
  float _command_timeout_s{0.25f};
  float _max_offboard_start_horizontal_error_m{1.2f};
  float _max_offboard_start_vertical_error_m{0.8f};
  float _max_position_setpoint_step_horizontal_m{0.0f};
  float _max_position_setpoint_step_vertical_m{0.0f};
  int64_t _offboard_setpoint_warmup_count{20};
  int64_t _warmup_counter{0};
  bool _auto_set_offboard{true};
  bool _auto_arm{false};
  bool _hold_position_on_timeout{true};
  bool _suspend_on_external_mode_command{true};
  bool _require_armed_before_offboard{true};
  bool _require_local_position_before_offboard{true};
  bool _align_planner_frame_to_px4_local{true};
  bool _have_command{false};
  bool _have_vehicle_status{false};
  bool _have_vehicle_local_position{false};
  bool _have_planner_to_px4_enu_offset{false};
  bool _pending_frame_realign{true};
  bool _control_suspended{false};
  bool _external_control_inhibited{false};
  bool _persistent_control_inhibit{false};
  bool _resume_on_next_command{false};
  bool _completed_hold_active{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryInterface>());
  rclcpp::shutdown();
  return 0;
}
