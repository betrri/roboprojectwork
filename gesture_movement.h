// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/gripper.h>

#include "std_msgs/String.h"

namespace franka_example_controllers {

class JointPositionExampleController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void chatterCallback(const std_msgs::String::ConstPtr msg);
  void stopping(const ros::Time& time);
  void openGripper(double width);
  void closeGripper();
  
  

 private:
  void moveRobot(std::vector<double> move_to_this);
  bool isAtFist();
  bool areWeHere(std::vector<double> position_vector);
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  ros::Duration elapsed_time_;
  std::array<double, 7> initial_pose_{};
  ros::Subscriber sub_;
  std::string latest_gesture_;
  bool gripperClosed_ = false;
};

}  // namespace franka_example_controllers
