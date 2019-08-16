#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <crane_controllers/crane_tip_velocity_rml_controller.h>

namespace crane_controllers
{
bool CraneTipVelocityRMLController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  crane_tip_velocity_command_interface_ = robot_hardware->get<crane_hw_interface::CraneTipVelocityCommandInterface>();
  if (crane_tip_velocity_command_interface_ == nullptr)
  {
    ROS_ERROR("CraneTipVelocityForwardController: Error getting control interface from hardware!");
    return false;
  }

  // RML
  rml_.reset(new ReflexxesAPI(2, 0.01));
  rml_input_.reset(new RMLPositionInputParameters(2));
  rml_output_.reset(new RMLPositionOutputParameters(2));

  crane_tip_velocity_handle_ = crane_tip_velocity_command_interface_->getHandle("crane_tip");

  command_sub_ = node_handle.subscribe<crane_msgs::CraneTrajectoryPoint>(
      "command", 1, &CraneTipVelocityRMLController::commandCB, this);

  // ROS API: Action interface
  action_server_.reset(new ActionServer(node_handle, "follow_crane_trajectory",
                                        boost::bind(&CraneTipVelocityRMLController::goalCB, this, _1),
                                        boost::bind(&CraneTipVelocityRMLController::cancelCB, this, _1), false));
  action_server_->start();

  return true;
}

void CraneTipVelocityRMLController::starting(const ros::Time& now)
{
  std::array<double, 2> position = crane_tip_velocity_handle_.getPosition();
  std::array<double, 2> velocity = crane_tip_velocity_handle_.getVelocity();

  crane_msgs::CraneTrajectoryPoint command;
  for (std::size_t i = 0; i < 2; ++i)
  {
    command.position.push_back(position[i]);
    command.velocity.push_back(0.0);
    command.max_velocity.push_back(0.1);
    command.max_acceleration.push_back(0.1);
  }
  command_buffer_.initRT(command);

  for (std::size_t i = 0; i < 2; ++i)
  {
    rml_input_->SelectionVector->VecData[i] = true;
  }
}

void CraneTipVelocityRMLController::update(const ros::Time& now, const ros::Duration& period)
{
  crane_msgs::CraneTrajectoryPoint command = *command_buffer_.readFromRT();

  std::array<double, 2> position = crane_tip_velocity_handle_.getPosition();
  std::array<double, 2> velocity = crane_tip_velocity_handle_.getVelocity();

  for (std::size_t i = 0; i < 2; ++i)
  {
    rml_input_->CurrentPositionVector->VecData[i] = position[i];
    rml_input_->CurrentVelocityVector->VecData[i] = velocity[i];

    rml_input_->TargetPositionVector->VecData[i] = command.position[i];
    rml_input_->TargetVelocityVector->VecData[i] = command.velocity[i];
    rml_input_->MaxVelocityVector->VecData[i] = command.max_velocity[i];
    rml_input_->MaxAccelerationVector->VecData[i] = command.max_acceleration[i];
  }

  int result = rml_->RMLPosition(*rml_input_.get(), rml_output_.get(), rml_flags_);
  if (result < 0)
  {
    ROS_ERROR_STREAM("RML error: " << result);
  }

  crane_tip_velocity_handle_.setCommand(
      { rml_output_->NewVelocityVector->VecData[0], rml_output_->NewVelocityVector->VecData[1] });

  // ROS_INFO("%f, %f", rml_output_->NewVelocityVector->VecData[0], rml_output_->NewVelocityVector->VecData[1]);
}

}  // namespace crane_controllers