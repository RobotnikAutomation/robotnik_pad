#include <robotnik_pad_plugins/kinova_arm_plugin.h>

namespace pad_plugins
{
PadPluginKinovaArm::PadPluginKinovaArm()
{
}

PadPluginKinovaArm::~PadPluginKinovaArm()
{
}

void PadPluginKinovaArm::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  bool required = true;
  pnh_ = ros::NodeHandle(nh, plugin_ns);
  nh_ = ros::NodeHandle();

  arm_control_topic_name_ = "/robot/j2s6s200_controller/in/cartesian_velocity_with_fingers";
  readParam(pnh_, "arm_control_topic_name", arm_control_topic_name_, arm_control_topic_name_, required);
  set_home_service_name_ = "/robot/j2s6s200_controller/in/home_arm";
  readParam(pnh_, "set_home_service_name", set_home_service_name_, set_home_service_name_, required);
  readParam(pnh_, "max_linear_speed", max_linear_speed_, 2.0, required);
  readParam(pnh_, "max_angular_speed", max_angular_speed_, 3.0, required);
  readParam(pnh_, "config/button_deadman", button_deadman_, 4, required);
  readParam(pnh_, "config/button_movement_deadman", button_movement_deadman_, 5, required);
  readParam(pnh_, "config/button_home_arm", button_home_arm_, 12, required);
  readParam(pnh_, "config/axis_open_gripper", axis_open_gripper_, 4, required);
  readParam(pnh_, "config/axis_close_gripper", axis_close_gripper_, 3, required);
  readParam(pnh_, "config/axis_linear_x_ee", axis_linear_x_, 1, required);
  readParam(pnh_, "config/axis_linear_y_ee", axis_linear_y_, 0, required);
  readParam(pnh_, "config/axis_linear_z_ee", axis_linear_z_, 10, required);
  readParam(pnh_, "config/axis_angular_x_ee", axis_angular_x_, 5, required);
  readParam(pnh_, "config/axis_angular_y_ee", axis_angular_y_, 2, required);
  readParam(pnh_, "config/axis_angular_z_ee", axis_angular_z_, 9, required);
  readParam(pnh_, "config/button_speed_up", button_speed_up_, 3, required);
  readParam(pnh_, "config/button_speed_down", button_speed_down_, 1, required);

  // Publishers
  arm_control_pub_ = nh_.advertise<kinova_msgs::PoseVelocityWithFingers>(arm_control_topic_name_, 10);
  pad_status_pub_ = pnh_.advertise<robotnik_pad_msgs::KinovaArmStatus>("status", 10);

  // Services
  set_home_service_ = nh_.serviceClient<kinova_msgs::HomeArm>(set_home_service_name_);

  // initialize variables
  current_velocity_level_ = 0.1;
  velocity_level_step_ = 0.1;
  max_velocity_level_ = 1.0;
  min_velocity_level_ = 0.1;
  fingers_closure_percentage_ = 0.0;
  arm_control_msg_ = kinova_msgs::PoseVelocityWithFingers();
  arm_status_msg_ = robotnik_pad_msgs::KinovaArmStatus();
}

void PadPluginKinovaArm::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_deadman_].isPressed() and not buttons[button_movement_deadman_].isPressed())
  {
    if (buttons[button_speed_down_].isReleased())
    {
      current_velocity_level_ = std::max(min_velocity_level_, current_velocity_level_ - velocity_level_step_);
      ROS_INFO("PadPluginKinovaArm::execute: speed down -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }
    else if (buttons[button_speed_up_].isReleased())
    {
      current_velocity_level_ = std::min(max_velocity_level_, current_velocity_level_ + velocity_level_step_);
      ROS_INFO("PadPluginKinovaArm::execute: speed up -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }

    if (buttons[button_home_arm_].isReleased())
    {
      kinova_msgs::HomeArm srv_msg;
      set_home_service_.call(srv_msg);
      ROS_INFO("PadPluginKinovaArm::execute: set home");
    }

    fingers_closure_percentage_ -= (axes[axis_open_gripper_] + 1) / 4;
    fingers_closure_percentage_ += (axes[axis_close_gripper_] + 1) / 4;
    if (fingers_closure_percentage_ > 100) {
      fingers_closure_percentage_ = 100;
    } else if (fingers_closure_percentage_ < 0) {
      fingers_closure_percentage_ = 0;
    }
    
    arm_control_msg_.twist_linear_x = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_];
    arm_control_msg_.twist_linear_y = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_y_];
    arm_control_msg_.twist_linear_z = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_z_];

    arm_control_msg_.twist_angular_x = current_velocity_level_ * max_angular_speed_ * axes[axis_angular_x_];
    arm_control_msg_.twist_angular_y = current_velocity_level_ * max_angular_speed_ * axes[axis_angular_y_];
    arm_control_msg_.twist_angular_z = - current_velocity_level_ * max_angular_speed_ * axes[axis_angular_z_] * 2;

    
    arm_control_msg_.fingers_closure_percentage = fingers_closure_percentage_;
    arm_control_pub_.publish(arm_control_msg_);
  }
  else if (buttons[button_deadman_].isReleased())
  {
    arm_control_msg_.twist_linear_x = 0.0;
    arm_control_msg_.twist_linear_y = 0.0;
    arm_control_msg_.twist_linear_z = 0.0;
    arm_control_msg_.twist_angular_x = 0.0;
    arm_control_msg_.twist_angular_y = 0.0;
    arm_control_msg_.twist_angular_z = 0.0;
    arm_control_msg_.fingers_closure_percentage = fingers_closure_percentage_;

    arm_control_pub_.publish(arm_control_msg_);
  }

  arm_status_msg_.velocity_level = current_velocity_level_ * 100;
  arm_status_msg_.fingers_closure_percentage = fingers_closure_percentage_;
  pad_status_pub_.publish(arm_status_msg_);
}

}  // namespace pad_plugins
