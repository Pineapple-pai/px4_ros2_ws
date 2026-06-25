#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>

#include <Eigen/Core>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
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
    declare_parameter("publish_rate_hz", 50.0);
    declare_parameter("command_timeout_s", 0.25);
    declare_parameter("offboard_setpoint_warmup_count", 20);
    declare_parameter("auto_set_offboard", true);
    declare_parameter("auto_arm", false);
    declare_parameter("hold_position_on_timeout", true);

    const auto position_cmd_topic = get_parameter("position_cmd_topic").as_string();
    const auto vehicle_status_topic = get_parameter("vehicle_status_topic").as_string();
    const auto offboard_control_mode_topic = get_parameter("offboard_control_mode_topic").as_string();
    const auto trajectory_setpoint_topic = get_parameter("trajectory_setpoint_topic").as_string();
    const auto vehicle_command_topic = get_parameter("vehicle_command_topic").as_string();

    _publish_rate_hz = static_cast<float>(get_parameter("publish_rate_hz").as_double());
    _command_timeout_s = static_cast<float>(get_parameter("command_timeout_s").as_double());
    _offboard_setpoint_warmup_count = get_parameter("offboard_setpoint_warmup_count").as_int();
    _auto_set_offboard = get_parameter("auto_set_offboard").as_bool();
    _auto_arm = get_parameter("auto_arm").as_bool();
    _hold_position_on_timeout = get_parameter("hold_position_on_timeout").as_bool();

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
    const Eigen::Vector3f position_enu{
      static_cast<float>(msg->position.x),
      static_cast<float>(msg->position.y),
      static_cast<float>(msg->position.z)};
    const Eigen::Vector3f velocity_enu{
      static_cast<float>(msg->velocity.x),
      static_cast<float>(msg->velocity.y),
      static_cast<float>(msg->velocity.z)};
    const Eigen::Vector3f acceleration_enu{
      static_cast<float>(msg->acceleration.x),
      static_cast<float>(msg->acceleration.y),
      static_cast<float>(msg->acceleration.z)};

    const Eigen::Vector3f position_ned = px4_ros2::positionEnuToNed(position_enu);
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
    _last_command_time = now();
    _warmup_counter = std::min(_warmup_counter + 1, _offboard_setpoint_warmup_count);
    _have_command = true;
  }

  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    _vehicle_status = *msg;
    _have_vehicle_status = true;
  }

  void publishLoop()
  {
    if (!_have_command) {
      return;
    }

    px4_msgs::msg::TrajectorySetpoint setpoint = _latest_setpoint;
    const bool command_fresh =
      ((now() - _last_command_time).seconds() <= static_cast<double>(_command_timeout_s));

    if (!command_fresh && !_hold_position_on_timeout) {
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
    if (_auto_arm) {
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
  rclcpp::TimerBase::SharedPtr _timer;

  px4_msgs::msg::TrajectorySetpoint _latest_setpoint{};
  px4_msgs::msg::VehicleStatus _vehicle_status{};
  rclcpp::Time _last_command_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time _last_mode_request_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time _last_arm_request_time{0, 0, RCL_ROS_TIME};
  float _publish_rate_hz{50.0f};
  float _command_timeout_s{0.25f};
  int64_t _offboard_setpoint_warmup_count{20};
  int64_t _warmup_counter{0};
  bool _auto_set_offboard{true};
  bool _auto_arm{false};
  bool _hold_position_on_timeout{true};
  bool _have_command{false};
  bool _have_vehicle_status{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryInterface>());
  rclcpp::shutdown();
  return 0;
}
