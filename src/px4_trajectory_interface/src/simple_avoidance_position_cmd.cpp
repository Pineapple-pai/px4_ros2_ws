#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

class SimpleAvoidancePositionCommand : public rclcpp::Node
{
public:
  SimpleAvoidancePositionCommand()
      : Node("simple_avoidance_position_cmd")
  {
    declare_parameter("odom_topic", "/autonomy/lio_odometry");
    declare_parameter("goal_topic", "/move_base_simple/goal");
    declare_parameter("position_cmd_topic", "/planning/position_cmd");
    declare_parameter("publish_rate_hz", 30.0);
    declare_parameter("max_speed_m_s", 0.75);
    declare_parameter("acceptance_radius_m", 0.35);
    declare_parameter("obstacle_min_x", -2.75);
    declare_parameter("obstacle_max_x", -1.65);
    declare_parameter("obstacle_min_y", -2.10);
    declare_parameter("obstacle_max_y", -0.90);
    declare_parameter("avoidance_clearance_m", 0.70);
    declare_parameter("takeoff_altitude_m", 1.5);

    _max_speed = get_parameter("max_speed_m_s").as_double();
    _acceptance_radius = get_parameter("acceptance_radius_m").as_double();
    _obstacle_min_x = get_parameter("obstacle_min_x").as_double();
    _obstacle_max_x = get_parameter("obstacle_max_x").as_double();
    _obstacle_min_y = get_parameter("obstacle_min_y").as_double();
    _obstacle_max_y = get_parameter("obstacle_max_y").as_double();
    _avoidance_clearance = get_parameter("avoidance_clearance_m").as_double();
    _takeoff_altitude = get_parameter("takeoff_altitude_m").as_double();

    _cmd_pub = create_publisher<quadrotor_msgs::msg::PositionCommand>(
        get_parameter("position_cmd_topic").as_string(), 10);
    _odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        get_parameter("odom_topic").as_string(),
        10,
        std::bind(&SimpleAvoidancePositionCommand::odomCallback, this, std::placeholders::_1));
    _goal_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
        get_parameter("goal_topic").as_string(),
        10,
        std::bind(&SimpleAvoidancePositionCommand::goalCallback, this, std::placeholders::_1));

    const double publish_rate = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    _timer = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / publish_rate)),
        std::bind(&SimpleAvoidancePositionCommand::publishSetpoint, this));

    RCLCPP_INFO(get_logger(), "Simple avoidance fallback publisher is ready.");
  }

private:
  struct Point
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    _current = {
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z};
    _have_odom = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!_have_odom)
    {
      RCLCPP_WARN(get_logger(), "Ignoring goal until odometry is available.");
      return;
    }

    Point goal{
        msg->pose.position.x,
        msg->pose.position.y,
        std::max(msg->pose.position.z, _takeoff_altitude)};

    if (_active && distance(goal, _last_goal) < 0.15)
    {
      return;
    }

    _waypoints.clear();
    _waypoints.push_back({_current.x, _current.y, goal.z});

    if (lineIntersectsObstacle({_current.x, _current.y, goal.z}, goal))
    {
      const double detour_y = chooseDetourY(_current, goal);
      const double detour_x = chooseDetourX(_current, goal);
      _waypoints.push_back({_current.x, detour_y, goal.z});
      _waypoints.push_back({detour_x, detour_y, goal.z});
    }

    _waypoints.push_back(goal);
    _last_goal = goal;
    _active_index = 0;
    _setpoint = _current;
    _active = true;
    _trajectory_id++;
    _last_publish_time = now();

    RCLCPP_INFO(
        get_logger(),
        "Generated fallback path with %zu waypoints to goal (%.2f, %.2f, %.2f).",
        _waypoints.size(),
        goal.x,
        goal.y,
        goal.z);
  }

  bool lineIntersectsObstacle(const Point &a, const Point &b) const
  {
    const int samples = 80;
    for (int i = 0; i <= samples; ++i)
    {
      const double t = static_cast<double>(i) / static_cast<double>(samples);
      const double x = a.x + ((b.x - a.x) * t);
      const double y = a.y + ((b.y - a.y) * t);
      if (x >= _obstacle_min_x && x <= _obstacle_max_x &&
          y >= _obstacle_min_y && y <= _obstacle_max_y)
      {
        return true;
      }
    }
    return false;
  }

  double chooseDetourY(const Point &start, const Point &goal) const
  {
    const double lower_y = _obstacle_min_y - _avoidance_clearance;
    const double upper_y = _obstacle_max_y + _avoidance_clearance;
    const double lower_cost = std::abs(start.y - lower_y) + std::abs(goal.y - lower_y);
    const double upper_cost = std::abs(start.y - upper_y) + std::abs(goal.y - upper_y);
    return lower_cost <= upper_cost ? lower_y : upper_y;
  }

  double chooseDetourX(const Point &start, const Point &goal) const
  {
    const double left_x = _obstacle_min_x - _avoidance_clearance;
    const double right_x = _obstacle_max_x + _avoidance_clearance;
    const double left_cost = std::abs(start.x - left_x) + std::abs(goal.x - left_x);
    const double right_cost = std::abs(start.x - right_x) + std::abs(goal.x - right_x);
    return left_cost <= right_cost ? left_x : right_x;
  }

  static double distance(const Point &a, const Point &b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
  }

  void publishSetpoint()
  {
    if (!_active || !_have_odom || _waypoints.empty())
    {
      return;
    }

    const rclcpp::Time now_time = now();
    double dt = (now_time - _last_publish_time).seconds();
    if (dt <= 0.0 || dt > 0.5)
    {
      dt = 1.0 / 30.0;
    }
    _last_publish_time = now_time;

    while (_active_index < _waypoints.size() &&
           distance(_current, _waypoints[_active_index]) < _acceptance_radius)
    {
      _active_index++;
    }

    if (_active_index >= _waypoints.size())
    {
      _setpoint = _waypoints.back();
      if (distance(_current, _waypoints.back()) < _acceptance_radius)
      {
        _active = false;
      }
    }
    else
    {
      advanceSetpointToward(_current, _waypoints[_active_index], _max_speed * dt);
    }

    publishCommand();
  }

  void stopPublishing()
  {
    _active = false;
    _waypoints.clear();
  }

  void advanceSetpointToward(const Point &start, const Point &target, double max_step)
  {
    const double dx = target.x - start.x;
    const double dy = target.y - start.y;
    const double dz = target.z - start.z;
    const double norm = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
    if (norm <= max_step || norm < 1e-6)
    {
      _setpoint = target;
      return;
    }

    const double scale = max_step / norm;
    _setpoint.x = start.x + (dx * scale);
    _setpoint.y = start.y + (dy * scale);
    _setpoint.z = start.z + (dz * scale);
  }

  void publishCommand()
  {
    quadrotor_msgs::msg::PositionCommand cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "world";
    cmd.position.x = _setpoint.x;
    cmd.position.y = _setpoint.y;
    cmd.position.z = _setpoint.z;

    if (_active_index < _waypoints.size())
    {
      const Point &target = _waypoints[_active_index];
      const double dx = target.x - _setpoint.x;
      const double dy = target.y - _setpoint.y;
      const double dz = target.z - _setpoint.z;
      const double norm = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
      if (norm > 1e-6)
      {
        cmd.velocity.x = _max_speed * dx / norm;
        cmd.velocity.y = _max_speed * dy / norm;
        cmd.velocity.z = _max_speed * dz / norm;
        cmd.yaw = std::atan2(dy, dx);
      }
    }

    cmd.kx = {5.7, 5.7, 6.2};
    cmd.kv = {3.4, 3.4, 4.0};
    cmd.trajectory_id = _trajectory_id;
    cmd.trajectory_flag = _active
      ? quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY
      : quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
    _cmd_pub->publish(cmd);
  }

  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr _cmd_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
  rclcpp::TimerBase::SharedPtr _timer;

  bool _have_odom{false};
  bool _active{false};
  Point _current;
  Point _setpoint;
  Point _last_goal;
  std::vector<Point> _waypoints;
  size_t _active_index{0};
  uint32_t _trajectory_id{0};
  rclcpp::Time _last_publish_time{0, 0, RCL_ROS_TIME};

  double _max_speed{0.75};
  double _acceptance_radius{0.35};
  double _obstacle_min_x{-2.75};
  double _obstacle_max_x{-1.65};
  double _obstacle_min_y{-2.10};
  double _obstacle_max_y{-0.90};
  double _avoidance_clearance{1.00};
  double _takeoff_altitude{1.5};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleAvoidancePositionCommand>());
  rclcpp::shutdown();
  return 0;
}
