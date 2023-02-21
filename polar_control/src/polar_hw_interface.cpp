#include <polar_control/polar_hw_interface.h>
#include <iostream>

namespace polar_control_ns
{

PolarHWInterface::PolarHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{

  // state_sub = nh.subscribe("/roboclaw/feedback", 1000, feedbackCallback);
  state_pub = nh.advertise<polar_control::PolarCommand>("/roboclaw/command", 1);
  ROS_INFO("PolarHWInterface constructed.");
}

void PolarHWInterface::feedbackCallback(const polar_control::PolarCommand polarCommand)
{
  std::cout << "feedbackCallback: ";
  for(int i = 0; i < num_joints_; i++){
    encoderFeedback.position[i] = polarCommand.position[i]; // assign read data to moveit joint_position_
    std::cout << encoderFeedback.position[i] << " ";
  }
  std::cout << "\n\n";
}

long PolarHWInterface::map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PolarHWInterface::init()
{
  ROS_INFO("PolarHWInterface initialized.");
}

void PolarHWInterface::read(ros::Duration& elapsed_time)
{  
  // for(int i = 0; i < num_joints_; i++){
  //   joint_position_[i] = encoderFeedback.position[i];
  // } 
  for(int i = 0; i < num_joints_; i++){
    joint_position_[i] = joint_position_command_[i];
  }
}

void PolarHWInterface::write(ros::Duration& elapsed_time)
{
  // printf("\nwrite\n");

  // Safety
  enforceLimits(elapsed_time);

  // Publish info for debug
  polar_control::PolarCommand polar_command;
  for(int i = 0; i < num_joints_; i++){
    // printf("[%s] %f\n", joint_names_[i].c_str(), joint_position_command_[i]);
    polar_command.position[i] = joint_position_command_[i];
    polar_command.joint_names[i] = joint_names_[i];
  }

  polar_command.position[6] = map(polar_command.position[6], -1.5707, 1.5707, 20, 80);
  state_pub.publish(polar_command);

  // // Serial write
  // cmd.polar_joint1 = joint_position_command_[0];
  // cmd.polar_joint2 = joint_position_command_[1] + joint2_offset;
  // cmd.polar_joint3 = joint_position_command_[2] + joint3_offset;
  // cmd.polar_joint4 = joint_position_command_[3];
  // cmd.polar_joint5 = joint_position_command_[4] + joint5_offset;
  // cmd.polar_joint6 = joint_position_command_[5];
  // cmd.polar_hand_joint1 = map(-joint_position_command_[6], 0, 1.0472, 34, 70);
  // // cmd.polar_hand_joint1 = 70;
  // ser.write((const uint8_t*) &cmd, sizeof(struct polar_joints));

}

void PolarHWInterface::enforceLimits(ros::Duration& period)
{
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  // printf("enforceLimits\n");
}

}  // namespace polar_control_ns
