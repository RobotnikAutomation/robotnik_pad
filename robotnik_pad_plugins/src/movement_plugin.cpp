#include <robotnik_pad_plugins/movement_plugin.h>

namespace pad_plugins
{
PadPluginMovement::PadPluginMovement()
{
}

PadPluginMovement::~PadPluginMovement()
{
}

void PadPluginMovement::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  bool required = true;
  pnh_ = ros::NodeHandle(nh, plugin_ns);
  nh_ = nh;

  readParam(pnh_, "config/button_deadman", button_dead_man_, button_dead_man_, required);
  readParam(pnh_, "config/axis_linear_x", axis_linear_x_, axis_linear_x_, required);
  readParam(pnh_, "config/axis_linear_y", axis_linear_y_, axis_linear_y_, required);
  readParam(pnh_, "config/axis_angular_z", axis_angular_z_, axis_angular_z_, required);
  readParam(pnh_, "config/button_kinematic_mode", button_kinematic_mode_, button_kinematic_mode_, required);
  readParam(pnh_, "config/button_speed_up", button_speed_up_, button_speed_up_, required);
  readParam(pnh_, "config/button_speed_down", button_speed_down_, button_speed_down_, required);
  readParam(pnh_, "scale_linear", scale_linear_, scale_linear_, required);
  readParam(pnh_, "scale_angular", scale_angular_, scale_angular_, required);
  readParam(pnh_, "cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_, required);

  // Publishers
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 10);
  pad_status_pub_ = pnh_.advertise<robotnik_pad_msgs::MovementStatus>("status", 10);

  // initialize variables
  current_vel_ = 0.1;
  cmd_twist_ = geometry_msgs::Twist();
  movement_status_msg_ = robotnik_pad_msgs::MovementStatus();
  kinematic_mode_ = Differential;
}

void PadPluginMovement::execute(std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_dead_man_].isPressed())
  {
    if (buttons[button_speed_down_].isReleased())
    {
      if (current_vel_ > 0.1)
      {
        current_vel_ -= 0.1;
        ROS_INFO("Velocity: %f%%", current_vel_ * 100.0);
      }
    }
    if (buttons[button_speed_up_].isReleased())
    {
      if (current_vel_ < 0.9)
      {
        current_vel_ += 0.1;
        ROS_INFO("Velocity: %f%%", current_vel_ * 100.0);
      }
    }

    if (buttons[button_kinematic_mode_].isReleased())
    {
      if (kinematic_mode_ == Differential)
      {
        kinematic_mode_ = Omnidirectional;
      }
      else if (kinematic_mode_ == Omnidirectional)
      {
        kinematic_mode_ = Differential;
      }
    }

    cmd_twist_.linear.x = current_vel_ * scale_linear_ * axes[axis_linear_x_];
    cmd_twist_.angular.z = current_vel_ * scale_angular_ * axes[axis_angular_z_];

    if (kinematic_mode_ == Omnidirectional)
    {
      cmd_twist_.linear.y = current_vel_ * scale_linear_ * axes[axis_linear_y_];
    }
    else
    {
      cmd_twist_.linear.y = 0.0;
    }

    twist_pub_.publish(cmd_twist_);
  }
  else if (buttons[button_dead_man_].isReleased())
  {
    cmd_twist_.linear.x = 0.0;
    cmd_twist_.linear.y = 0.0;
    cmd_twist_.angular.z = 0.0;

    twist_pub_.publish(cmd_twist_);
  }

  movement_status_msg_.velocity_level = current_vel_ * 100;
  movement_status_msg_.kinematic_mode = kinematicModeToStr(kinematic_mode_);
  pad_status_pub_.publish(movement_status_msg_);
}

std::string PadPluginMovement::kinematicModeToStr(int kinematic_mode)
{
  switch (kinematic_mode)
  {
    case Omnidirectional:
      return "omni";
    case Differential:
      return "differential";
    default:
      return "Unknown";
  }
}
}  // namespace