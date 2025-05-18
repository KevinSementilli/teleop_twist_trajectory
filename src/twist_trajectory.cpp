#include "twist_trajectory.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <cstring>

TwistTrajectoryNode::TwistTrajectoryNode()
: Node("twist_to_trajectory")
{
  declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  std::string robot_description = get_parameter("robot_description").as_string();

  // Parse robot
  KDL::Tree robot_tree;
  if (!kdl_parser::treeFromString(robot_description, robot_tree)) {
    RCLCPP_ERROR(get_logger(), "Failed to parse URDF");
    return;
  }

  if (!robot_tree.getChain("base_link", "tool0", chain_)) {
    RCLCPP_ERROR(get_logger(), "Failed to get chain from base_link to tool0");
    return;
  }

  joint_positions_ = KDL::JntArray(chain_.getNrOfJoints());
  ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_, 1e-6);

  // Publishers and subscribers
  trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/r6bot_controller/joint_trajectory", 10);

  twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&TwistTrajectoryNode::twist_callback, this, std::placeholders::_1));
}

void TwistTrajectoryNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  KDL::Twist twist;
  twist.vel.x(msg->linear.x);
  twist.vel.y(msg->linear.y);
  twist.vel.z(msg->linear.z);
  twist.rot.x(msg->angular.x);
  twist.rot.y(msg->angular.y);
  twist.rot.z(msg->angular.z);

  KDL::JntArray joint_velocities(chain_.getNrOfJoints());

  if (ik_vel_solver_->CartToJnt(joint_positions_, twist, joint_velocities) < 0) {
    RCLCPP_WARN(get_logger(), "IK velocity solver failed.");
    return;
  }

  // Create a short trajectory point
  double duration = 0.5; // seconds
  joint_positions_.data += joint_velocities.data * duration;

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.header.stamp = now();
  traj_msg.points.resize(1);
  trajectory_msgs::msg::JointTrajectoryPoint & pt = traj_msg.points[0];

  pt.positions.resize(joint_positions_.rows());
  pt.velocities.resize(joint_velocities.rows());
  pt.time_from_start = rclcpp::Duration::from_seconds(duration);

  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None) {
      traj_msg.joint_names.push_back(chain_.getSegment(i).getJoint().getName());
    }
  }

  std::memcpy(pt.positions.data(), joint_positions_.data.data(), pt.positions.size() * sizeof(double));
  std::memcpy(pt.velocities.data(), joint_velocities.data.data(), pt.velocities.size() * sizeof(double));

  trajectory_pub_->publish(traj_msg);
}
