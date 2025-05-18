#include "twist_trajectory.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistTrajectoryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}