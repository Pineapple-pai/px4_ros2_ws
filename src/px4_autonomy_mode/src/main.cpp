#include <memory>
#include <string>

#include <px4_ros2/components/node_with_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "px4_autonomy_mode/autonomy_mode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<Px4AutonomyMode>>(
    "px4_autonomy_mode", true));
  rclcpp::shutdown();
  return 0;
}
