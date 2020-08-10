#include <robotnik_pad_plugins/ackermann_movement_plugin.h>

namespace pad_plugins
{
PadPluginAckermannMovement::PadPluginAckermannMovement()
{
}

PadPluginAckermannMovement::~PadPluginAckermannMovement()
{
}


void PadPluginAckermannMovement::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  bool required = true;
  pnh_ = ros::NodeHandle(nh, plugin_ns);
  nh_ = ros::NodeHandle();
  button_dead_man_ = 5;
  readParam(pnh_, "config/button_deadman", button_dead_man_, button_dead_man_, required);
  axis_linear_x_ = 1;
  readParam(pnh_, "config/axis_linear_x", axis_linear_x_, axis_linear_x_, required);
  axis_linear_y_ = 0;
  readParam(pnh_, "config/axis_linear_y", axis_linear_y_, axis_linear_y_, required);
  axis_angular_z_ = 2;
  readParam(pnh_, "config/axis_angular_z", axis_angular_z_, axis_angular_z_, required);
  button_kinematic_mode_ = 7;
  readParam(pnh_, "config/button_kinematic_mode", button_kinematic_mode_, button_kinematic_mode_, required);
  button_speed_up_ = 3;
  readParam(pnh_, "config/button_speed_up", button_speed_up_, button_speed_up_, required);
  button_speed_down_ = 1;
  readParam(pnh_, "config/button_speed_down", button_speed_down_, button_speed_down_, required);
  max_linear_speed_ = 1.5;
  readParam(pnh_, "max_linear_speed", max_linear_speed_, max_linear_speed_, required);
  max_angular_speed_ = 3.0;
  readParam(pnh_, "max_angular_speed", max_angular_speed_, max_angular_speed_, required);
  cmd_topic_vel_ = "cmd_vel";
  readParam(pnh_, "cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_, required);
  // if not set, then ackermann mode cannot be used
  wheel_base_ = 0;
  readParam(pnh_, "wheel_base", wheel_base_, wheel_base_);

  // Publishers
  ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>(cmd_topic_vel_, 10);
  pad_status_pub_ = pnh_.advertise<robotnik_pad_msgs::MovementStatus>("status", 10);

  // initialize variables
  current_velocity_level_ = 0.1;
  velocity_level_step_ = 0.1;
  max_velocity_level_ = 1.0;
  min_velocity_level_ = 0.1;
  cmd_ackermann_ = ackermann_msgs::AckermannDrive();
  movement_status_msg_ = robotnik_pad_msgs::MovementStatus();
  kinematic_mode_ = KinematicModes::Ackermann;
}

void PadPluginAckermannMovement::execute(std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_dead_man_].isPressed())
  {
    if (buttons[button_speed_down_].isReleased())
    {
      current_velocity_level_ = std::max(min_velocity_level_, current_velocity_level_ - velocity_level_step_);
      ROS_INFO("PadPluginAckermannMovement::execute: speed down -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }
    else if (buttons[button_speed_up_].isReleased())
    {
      current_velocity_level_ = std::min(max_velocity_level_, current_velocity_level_ + velocity_level_step_);
      ROS_INFO("PadPluginAckermannMovement::execute: speed up -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }

    /*if (buttons[button_kinematic_mode_].isReleased())
    {
      if (kinematic_mode_ == KinematicModes::Differential)
      {
        kinematic_mode_ = KinematicModes::Omnidirectional;
      }
      else if (kinematic_mode_ == KinematicModes::Omnidirectional)
      {
        if (wheel_base_ == 0)  // not set, ackermann mode cannot be selected
        {
          kinematic_mode_ = KinematicModes::Differential;
        }
        else
        {
          kinematic_mode_ = KinematicModes::Ackermann;
        }
      }
      else if (kinematic_mode_ == KinematicModes::Ackermann)
      {
        kinematic_mode_ = KinematicModes::Differential;
      }
    }*/

    cmd_ackermann_.linear.x = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_];
    if (kinematic_mode_ == KinematicModes::Ackermann)
    {
      cmd_ackermann_.angular.z = current_velocity_level_ * max_linear_speed_ * axes[axis_angular_x_]
      cmd_ackermann_.linear.x = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_]
    }
    else
    {
      cmd_ackermann_.angular.z = current_velocity_level_ * max_angular_speed_ * axes[axis_angular_z_];
    }

    if (kinematic_mode_ == KinematicModes::Omnidirectional)
    {
      cmd_ackermann_.linear.y = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_y_];
    }
    else
    {
      cmd_ackermann_.linear.y = 0.0;
    }

    ackermann_pub_.publish(cmd_ackermann_);
  }
  else if (buttons[button_dead_man_].isReleased())
  {
    cmd_ackermann_.linear.x = 0.0;
    cmd_ackermann_.linear.y = 0.0;
    cmd_ackermann_.angular.z = 0.0;

    ackermann_pub_.publish(cmd_ackermann_);
  }

  movement_status_msg_.velocity_level = current_velocity_level_ * 100;
  movement_status_msg_.kinematic_mode = kinematicModeToStr(kinematic_mode_);
  pad_status_pub_.publish(movement_status_msg_);
}

std::string PadPluginAckermannMovement::kinematicModeToStr(int kinematic_mode)
{
  switch (kinematic_mode)
  {
    case KinematicModes::Differential:
      return "differential";
    case KinematicModes::Omnidirectional:
      return "omni";
    case KinematicModes::Ackermann:
      return "ackermann";
    default:
      return "unknown";
  }
}
}  // namespace pad_plugins
