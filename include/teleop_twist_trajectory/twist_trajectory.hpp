#ifndef TWIST_TRAJECTORY_HPP
#define TWIST_TRAJECTORY_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>

Class TwistTrajectory : public rclcpp::Node
{
public:
  TwistTrajectory();

private:

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  KDL::JntArray joint_positions_;
};

#endif  // TWIST_TRAJECTORY_HPP