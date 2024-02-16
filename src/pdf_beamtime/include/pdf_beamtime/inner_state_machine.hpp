/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <moveit/move_group_interface/move_group_interface.h>

#include <future>
#include <string>
#include <map>
#include <vector>
#include <rclcpp/node.hpp>
#include <pdf_beamtime/state_enum.hpp>

class InnerStateMachine
{
private:
  rclcpp::Node::SharedPtr node_;
  State external_state_enum_;
  Internal_State internal_state_enum_;
  /// @brief Holds the passed joint target
  std::vector<double> joint_goal_;

  std::vector<std::string> external_state_names_ =
  {"HOME", "PICKUP_APPROACH", "PICKUP", "GRASP_SUCCESS", "GRASP_FAILURE", "PICKUP_RETREAT",
    "PLACE_APPROACH", "PLACE", "RELEASE_SUCCESS", "RELEASE_FAILURE", "PLACE_RETREAT",
    "RETRY_PICKUP"};

  std::vector<std::string> internal_state_names =
  {"RESTING", "MOVING", "PAUSED", "ABORT", "HALT", "STOP"};

public:
  InnerStateMachine(const rclcpp::Node::SharedPtr node, State external_state_enum);

  /// @brief move the robot to the passed joint angles
  /// @param mgi move_group_interface_
  /// @return the error code
  moveit::core::MoveItErrorCode move_robot(
    moveit::planning_interface::MoveGroupInterface & mgi,
    std::vector<double> joint_goal);

  /// @brief state change if paused command was issues
  void pause(moveit::planning_interface::MoveGroupInterface & mgi);
  // State resume(moveit::planning_interface::MoveGroupInterface & mgi);
  /// @todo @ChandimaFernando: rewind the state back to RESTING
  void rewind();

  /// @brief  Set the state to abort. This follows a pause command.
  void abort();

  /// @brief  Set the state to halt. This follows a pause command.
  void halt();

  /// @brief  Set the state to Stop. This follows a pause command.
  void stop();

  /// @brief  Self explantory
  void set_internal_state(Internal_State state);
  Internal_State get_internal_state();
  void set_external_state(State state);

  /// @brief  Gripper object is not set at the pdf_beamtime level
  /// @todo ChandimaFernando
  /// @return Moveit error code
  moveit::core::MoveItErrorCode open_gripper();
  moveit::core::MoveItErrorCode close_gripper();

};
