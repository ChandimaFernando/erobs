/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <pdf_beamtime_interfaces/srv/estimated_pose_msg.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
/*
This needs robot_description and robot_description_semantic parameters somehow.
*/

// Function to convert degrees to radians
double degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}


int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create a node for synchronously grabbing params
  auto parameter_client_node = rclcpp::Node::make_shared("param_client");
  auto parent_parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(parameter_client_node, "move_group");
  // Boiler plate wait block
  while (!parent_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "move_group service not available, waiting again...");
  }

  // Create a client for pose estimator
  auto pose_estimator_client_ =
    parameter_client_node->create_client<pdf_beamtime_interfaces::srv::EstimatedPoseMsg>(
    "pose_service");

  auto request = std::make_shared<pdf_beamtime_interfaces::srv::EstimatedPoseMsg::Request>();
  request->a = 1.0;


  while (!pose_estimator_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  std::vector<double> estimated_pose = {};

  auto result = pose_estimator_client_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(parameter_client_node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto results = result.get()->pose;
    for (const auto & elem : results) {
      estimated_pose.push_back(elem);
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service pose estimator");
  }

  // Get robot config parameters from parameter server
  auto parameters = parent_parameters_client->get_parameters(
    {"robot_description_semantic",
      "robot_description"});

  // create the Node for moveit with
  // auto const node = std::make_shared<rclcpp::Node>(
  //   "hello_moveit",
  //   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  // );

  std::string parameter_value = parameters[0].value_to_string();
  parameter_client_node->declare_parameter<std::string>(
    "robot_description_semantic",
    parameter_value);
  parameter_value = parameters[1].value_to_string();
  parameter_client_node->declare_parameter<std::string>("robot_description", parameter_value);

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  RCLCPP_INFO(logger, "assembling move_group_interface");
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(parameter_client_node, "ur_arm");

  // std::vector<double> joint_goal_degrees =
  // {121.84, -162.0, -28.15, estimated_pose[4], estimated_pose[5], estimated_pose[3]};
  std::vector<double> joint_goal_degrees = {158.66, -123.69, -89, -140.46, 34, 351.0};

  // Vector to hold the converted angles in radians
  std::vector<double> joint_goal_radians(joint_goal_degrees.size());

  // Convert each element from degrees to radians using std::transform
  std::transform(
    joint_goal_degrees.begin(), joint_goal_degrees.end(),
    joint_goal_radians.begin(), degreesToRadians);


  move_group_interface.setJointValueTarget(joint_goal_radians);
  // Create a plan to that target pose
  auto const [planing_success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();
  if (planing_success) {

    move_group_interface.execute(plan);
  }

  std::this_thread::sleep_for(std::chrono::seconds(3));

  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_ =
  //   std::make_shared<tf2_ros::Buffer>(parameter_client_node->get_clock());
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_ =
  //   std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // std::string wrist_3_frame = "wrist_3_link";
  // std::string wrist_2_frame = "wrist_2_link";

  // std::string target_frame = "sample";

  // // Lookup the transform
  // geometry_msgs::msg::TransformStamped transform_wrist_3_sample = tf_buffer_->lookupTransform(
  //   wrist_3_frame,
  //   target_frame,
  //   tf2::TimePointZero);

  // geometry_msgs::msg::TransformStamped transform_wrist_2_sample = tf_buffer_->lookupTransform(
  //   wrist_2_frame,
  //   target_frame,
  //   tf2::TimePointZero);

  // // Convert quaternion to RPY

  // tf2::Quaternion wrist_2_quat(transform_wrist_2_sample.transform.rotation.x,
  //   transform_wrist_2_sample.transform.rotation.y, transform_wrist_2_sample.transform.rotation.z,
  //   transform_wrist_2_sample.transform.rotation.w);

  // double roll, pitch, yaw;
  // tf2::Matrix3x3(wrist_2_quat).getRPY(roll, pitch, yaw);


  // RCLCPP_INFO(
  //   parameter_client_node->get_logger(), "Translation: [%f, %f, %f]",
  //   transform_wrist_3_sample.transform.translation.x,
  //   transform_wrist_3_sample.transform.translation.y,
  //   transform_wrist_3_sample.transform.translation.z);

  // RCLCPP_INFO(
  //   parameter_client_node->get_logger(), "Wrist 2 RPY to sample: [%f, %f, %f]", roll, pitch, yaw);

  // Get the current state of the robot
  // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

  // Create joint constraint
  moveit_msgs::msg::JointConstraint joint_constraint;
  joint_constraint.joint_name = "joint_1";       // replace with your joint name
  joint_constraint.position = estimated_pose[4] * M_PI / 180.0;              // desired position in radians
  joint_constraint.tolerance_above = 0.1;
  joint_constraint.tolerance_below = 0.1;
  joint_constraint.weight = 1.0;

  // Create constraints message
  moveit_msgs::msg::Constraints path_constraints;
  // path_constraints.joint_constraints.push_back(joint_constraint);

  moveit_msgs::msg::JointConstraint joint_2_constraint;
  joint_2_constraint.joint_name = "joint_2";       // replace with your joint name
  joint_2_constraint.position = estimated_pose[5] * M_PI / 180.0;              // desired position in radians
  joint_2_constraint.tolerance_above = 0.1;
  joint_2_constraint.tolerance_below = 0.1;
  joint_2_constraint.weight = 1.0;
  // path_constraints.joint_constraints.push_back(joint_2_constraint);


  RCLCPP_INFO(
    logger, "received pose RPY XYZ: %f %f %f  \t %f %f %f ", estimated_pose[0],
    estimated_pose[1], estimated_pose[2], estimated_pose[3],
    estimated_pose[4], estimated_pose[5]);

  // Set the path constraints to the move group
  move_group_interface.setPathConstraints(path_constraints);

  // // // geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();
  // Set a target Pose
  auto const target_pose = [estimated_pose] {
      geometry_msgs::msg::Pose msg;
      msg.position.x = estimated_pose[3];     // camera xyz from aruco is different from robot's
      msg.position.y = estimated_pose[4];
      msg.position.z = estimated_pose[5];

      tf2::Quaternion quaternion;
      quaternion.setRPY(estimated_pose[0], estimated_pose[1], estimated_pose[2]);
      msg.orientation.x = quaternion.x();
      msg.orientation.y = quaternion.y();
      msg.orientation.z = quaternion.z();
      msg.orientation.w = quaternion.w();

      return msg;
    }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan_cart] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

  // Execute the plan
  if (success) {
    // move_group_interface.execute(plan_cart);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

/*
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/// @brief Create the obstacle environment and an simple action server for the robot to move
class MoveToPose
{
public:
  explicit MoveToPose(
    const std::string & move_group_name, const rclcpp::NodeOptions & options,
    std::string action_name);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getInterruptNodeBaseInterface();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr interrupt_node_;

  moveit::planning_interface::MoveGroupInterface move_group_interface_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /// @brief records home state
  std::vector<double, std::allocator<double>> goal_home_;

};

using moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;

MoveToPose::MoveToPose(
  const std::string & move_group_name = "ur_manipulator",
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  std::string action_name = "pdf_beamtime_action_server")
: node_(std::make_shared<rclcpp::Node>("pdf_beamtime_server", options)),
  interrupt_node_(std::make_shared<rclcpp::Node>("interrupt_server")),
  move_group_interface_(node_, move_group_name),
  planning_scene_interface_()
{

  std::vector<double> joint_goal = {-0.11536, -1.783732, 0.38816, -1.75492, 0.11484, 3.14159};
  move_group_interface_.setJointValueTarget(joint_goal);
  // Create a plan to that target pose

  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface_.plan(msg));

  if (ok) {
    move_group_interface_.execute(msg);
  }

}


rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MoveToPose::getNodeBaseInterface()
// Expose the node base interface so that the node can be added to a component manager.
{
  return node_->get_node_base_interface();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MoveToPose::getInterruptNodeBaseInterface()
// Expose the node base interface of the bluesky interrupt node
// so that the node can be added to a component manager.
{
  return interrupt_node_->get_node_base_interface();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a ROS logger for main scope
  auto const logger = rclcpp::get_logger("hello_moveit");
  using namespace std::chrono_literals;

  // Create a node for synchronously grabbing params
  auto parameter_client_node = rclcpp::Node::make_shared("param_client");
  auto parent_parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(parameter_client_node, "move_group");
  // Boiler plate wait block
  while (!parent_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "move_group service not available, waiting again...");
  }
  // Get robot config parameters from parameter server
  auto parameters = parent_parameters_client->get_parameters(
    {"robot_description_semantic",
      "robot_description"});

  // Set node parameters using NodeOptions
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides(
  {
    {"robot_description_semantic", parameters[0].value_to_string()},
    {"robot_description", parameters[1].value_to_string()}
  });

  rclcpp::executors::MultiThreadedExecutor executor;

  auto beamtime_server = std::make_shared<MoveToPose>(
    "ur_arm",
    node_options);

  executor.add_node(beamtime_server->getNodeBaseInterface());
  executor.add_node(beamtime_server->getInterruptNodeBaseInterface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
*/
