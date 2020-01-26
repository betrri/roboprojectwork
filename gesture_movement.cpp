// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/gesture_movement.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/exception.h>



namespace franka_example_controllers {

void JointPositionExampleController::chatterCallback(const std_msgs::String::ConstPtr msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  ROS_ERROR_STREAM("nikkalle, seuraavaks tulee messagen data");
  ROS_ERROR_STREAM(msg->data.c_str());
  latest_gesture_ = msg->data.c_str();

  std::vector<double> move_to_LDown{ 1.4074, -0.0327, 0.2389, -2.4063, -0.0097, 2.3688, -0.6357 };
  std::vector<double> move_to_FistDown{ 1.1772,  0.0930, 0.0376, -2.236, -0.0299, 2.2573, 0.4334 };

  if (latest_gesture_ == "Peace") {
    if (gripperClosed_) {
      openGripper(0.06);
    }
    else {
      closeGripper();
    }
  } else if (latest_gesture_ == "Fist") {
    if (isAtFist()) {
      latest_gesture_ = "moveToFistDown";      
    } else if (areWeHere(move_to_LDown)) {
      latest_gesture_ = "moveToLAndFistAndFistDown"; 
    }
    else {
      latest_gesture_ = "moveToFistAndDown";
    }
  } else if (latest_gesture_ == "Okay") {
    if (areWeHere(move_to_FistDown)) {
      latest_gesture_ = "moveToFistAndStart";      
    }
    else if (areWeHere(move_to_LDown)) {
      latest_gesture_ = "moveToLAndStart"; 
    }
    else {
      latest_gesture_ = "moveToStart";
    }
  } else if (latest_gesture_ == "L") {
    std::vector<double> move_to_L{ 1.4025, -0.0773, 0.2363, -2.0113, -0.0091, 1.9422, -0.6228 };
    if (areWeHere(move_to_L)) {
      latest_gesture_ = "moveToLDown";      
    }
    else if (areWeHere(move_to_FistDown)) {
      latest_gesture_ = "moveToFistAndLAndLDown";
    }
    else {
      latest_gesture_ = "moveToLAndDown";
    }
  }
  
}

bool JointPositionExampleController::isAtFist() {
  std::vector<double> move_to_Fist{ 1.1722, -0.0510, 0.0296, -2.0347, 0.0166, 1.9979, 0.5162 };
  std::vector<double> current_position{position_joint_handles_[0].getPosition(),
		position_joint_handles_[1].getPosition(), position_joint_handles_[2].getPosition(),
		position_joint_handles_[3].getPosition(), position_joint_handles_[4].getPosition(),
		position_joint_handles_[5].getPosition(), position_joint_handles_[6].getPosition()};

  for (int i = 0; i < 7; ++i){
    if (std::abs(current_position[i] - move_to_Fist[i]) > 0.05) {
      return false;
    } 
  }
  return true;
}

bool JointPositionExampleController::areWeHere(std::vector<double> position_vector) {
  std::vector<double> current_position{position_joint_handles_[0].getPosition(),
		position_joint_handles_[1].getPosition(), position_joint_handles_[2].getPosition(),
		position_joint_handles_[3].getPosition(), position_joint_handles_[4].getPosition(),
		position_joint_handles_[5].getPosition(), position_joint_handles_[6].getPosition()};

  for (int i = 0; i < 7; ++i){
    if (std::abs(current_position[i] - position_vector[i]) > 0.05) {
      return false;
    } 
  }
  return true;
}

void JointPositionExampleController::openGripper(double width) {

    franka::Gripper gripper("172.16.0.2");
    ROS_ERROR_STREAM("Nyt liikutetaa gripperii");
    try {
        if (gripper.move(width, 0.01)) {
         ROS_ERROR_STREAM("Gripperin homing on true");
         gripperClosed_ = false;
        } else {
          ROS_ERROR_STREAM("Gripperin homing on false");
        }	
      } catch (franka::Exception const& e) {
        ROS_ERROR_STREAM("Nyt meni pahasti pieleen gripper touhut" << e.what());
      }
  
}

void JointPositionExampleController::closeGripper() {
    franka::Gripper gripper("172.16.0.2");
    ROS_ERROR_STREAM("Nyt liikutetaa gripperii");
    try {
        if (gripper.move(0.02, 0.01)) {
         ROS_ERROR_STREAM("Gripperin homing on true");
         gripperClosed_ = true;
        } else {
          ROS_ERROR_STREAM("Gripperin homing on false");
        }	
      } catch (franka::Exception const& e) {
        ROS_ERROR_STREAM("Nyt meni pahasti pieleen gripper touhut" << e.what());
      }
}

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  ROS_ERROR_STREAM("JointPositionExampleController::init");
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }
  
  sub_ = node_handle.subscribe("chatter", 1000, &JointPositionExampleController::chatterCallback, this);
  ROS_ERROR_STREAM("nikkalle, Nyt subaan sun kanavan ");


  try {
    franka::Gripper gripper("172.16.0.2");

    if (gripper.move(0.06, 0.01)) {
      ROS_ERROR_STREAM("Gripperin homing on true");
    } else {
      ROS_ERROR_STREAM("Gripperin homing on false");
    }	

  } catch (franka::Exception const& e) {
    ROS_ERROR_STREAM("Nyt meni pahasti pieleen gripper touhut" << e.what());
  }


  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      //return false;
      return true;
    }
  }


  return true;

}


void JointPositionExampleController::starting(const ros::Time& /* time */) {
  ROS_ERROR_STREAM("JointPositionExampleController::starting");
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
    ROS_ERROR_STREAM(initial_pose_[i]);
  }
  elapsed_time_ = ros::Duration(0.0);
}

void JointPositionExampleController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {

  std::vector<double> move_to_start{ 0, -1*M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4 };
  std::vector<double> move_to_Palm{ 0, -1*M_PI/4, 0, -3*M_PI/4, 0, M_PI/2, M_PI/4 };
  std::vector<double> move_to_Fist{ 1.1722, -0.0510, 0.0296, -2.0347, 0.0166, 1.9979, 0.5162 };
  std::vector<double> move_to_FistDown{ 1.1772,  0.0930, 0.0376, -2.236, -0.0299, 2.2573, 0.4334 };

  std::vector<double> move_to_L{ 1.4025, -0.0773, 0.2363, -2.0113, -0.0091, 1.9422, -0.6228 };
  std::vector<double> move_to_LDown{ 1.4074, -0.0327, 0.2389, -2.4063, -0.0097, 2.3688, -0.6357 };

  std::vector<double> current_position{position_joint_handles_[0].getPosition(),
		position_joint_handles_[1].getPosition(), position_joint_handles_[2].getPosition(),
		position_joint_handles_[3].getPosition(), position_joint_handles_[4].getPosition(),
		position_joint_handles_[5].getPosition(), position_joint_handles_[6].getPosition()};

  if (latest_gesture_ == "moveToFistDown"){
    moveRobot(move_to_FistDown);
  } 
  else if (latest_gesture_ == "moveToFistAndDown") {
    if (areWeHere(move_to_Fist)) {
      latest_gesture_ = "moveToFistDown";
    } else {
      moveRobot(move_to_Fist);
    }
  }
  else if (latest_gesture_ == "moveToFistAndStart") {
    if (areWeHere(move_to_Fist)) {
      latest_gesture_ = "moveToStart";
    } else {
      moveRobot(move_to_Fist);
    }
  }
  else if (latest_gesture_ == "moveToStart") {
    moveRobot(move_to_start);
  }
  else if (latest_gesture_ == "moveToLDown"){
    moveRobot(move_to_LDown);
  }
  else if (latest_gesture_ == "moveToLAndDown"){
    if (areWeHere(move_to_L)) {
      latest_gesture_ = "moveToLDown";
    } else {
      moveRobot(move_to_L);
    }
  }
  else if (latest_gesture_ == "moveToLAndStart"){
    if (areWeHere(move_to_L)) {
      latest_gesture_ = "moveToStart";
    } else {
      moveRobot(move_to_L);
    }
  }
  else if (latest_gesture_ == "moveToLAndFistAndFistDown") {
    if (areWeHere(move_to_L)) {
      latest_gesture_ = "moveToFistAndDown";
    } else {
      moveRobot(move_to_L);
    }
  }
  else if (latest_gesture_ == "moveToFistAndLAndLDown") {
    if (areWeHere(move_to_Fist)) {
      latest_gesture_ = "moveToLAndDown";
    } else {
      moveRobot(move_to_Fist);
    }
  }
  else if (latest_gesture_ == "Peace") {
  }
  else {
    moveRobot(move_to_start);
  } //else 
  
/*

  rostopic pub -l /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal "{}" 
  robot_ip:=172.16.0.2

  export ROS_MASTER_URI=http://192.168.1.53:11311
  export ROS_IP=192.168.1.53
*/

} //funktio


//Moves robot to set vector location
void JointPositionExampleController::moveRobot(std::vector<double> move_to_this) {
  std::vector<double> current_position{position_joint_handles_[0].getPosition(),
		position_joint_handles_[1].getPosition(), position_joint_handles_[2].getPosition(),
		position_joint_handles_[3].getPosition(), position_joint_handles_[4].getPosition(),
		position_joint_handles_[5].getPosition(), position_joint_handles_[6].getPosition()};

  std::vector<double> error_vector;
  std::vector<double> nextPosition_vector;

  for (int i = 0; i < 7; ++i) {
    error_vector.push_back(move_to_this[i] - current_position[i]);
    error_vector[i] = error_vector[i] * 0.005;
    //Limit speed if too big error
    if (error_vector[i] > 0.001) {
      error_vector[i] = 0.001;
    } else if (error_vector[i] < -0.001) {
      error_vector[i] = -0.001;
    }
    nextPosition_vector.push_back(current_position[i] + error_vector[i]);
  }

  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i].setCommand(nextPosition_vector[i]);      
  }

}

void JointPositionExampleController::stopping(const ros::Time& time){
  ROS_ERROR_STREAM("JointPositionExampleController::stopping: stoppaus kutsuttu");
}
}  // namespace franka_example_controllers


PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
