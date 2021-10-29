#include <robotnik_pad_plugins/moveit_servo_teleoperation_plugin.h>

namespace pad_plugins
{
PadPluginMoveitServoTeleoperation::PadPluginMoveitServoTeleoperation()
{
}

PadPluginMoveitServoTeleoperation::~PadPluginMoveitServoTeleoperation()
{
}

void PadPluginMoveitServoTeleoperation::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  bool required = true;
  pnh_ = ros::NodeHandle(nh, plugin_ns);
  nh_ = ros::NodeHandle();

  readParam(pnh_, "moveit_servo_twist_topic", moveit_servo_twist_topic_, moveit_servo_twist_topic_, required);
  readParam(pnh_, "finger_velocity_topic", finger_velocity_topic_, finger_velocity_topic_, required);
  readParam(pnh_, "end_effector_frame", end_effector_frame_, end_effector_frame_, required);
  readParam(pnh_, "base_link_frame", base_link_frame_, base_link_frame_, required);
  readParam(pnh_, "max_linear_speed", max_linear_speed_, 1.0, required);
  readParam(pnh_, "max_angular_speed", max_angular_speed_, 2.0, required);
  readParam(pnh_, "rot_deadband", rot_deadband_, 0.1, required);
  readParam(pnh_, "linear_deadband", linear_deadband_, 0.1, required);
  readParam(pnh_, "max_finger_speed", max_finger_speed_, 2000.0, required);
  readParam(pnh_, "config/axis_linear_x_ee", axis_linear_x_, 0, required);
  readParam(pnh_, "config/axis_linear_y_ee", axis_linear_y_, 1, required);
  readParam(pnh_, "config/axis_linear_z_ee", axis_linear_z_, 2, required);
  readParam(pnh_, "config/axis_angular_x_ee", axis_angular_x_, 3, required);
  readParam(pnh_, "config/axis_angular_y_ee", axis_angular_y_, 4, required);
  readParam(pnh_, "config/axis_angular_z_ee", axis_angular_z_, 5, required);
  readParam(pnh_, "config/button_open", button_open_, 6, required);
  readParam(pnh_, "config/button_close", button_close_, 7, required);

  // Publishers
  arm_control_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(moveit_servo_twist_topic_, 10);
  finger_control_pub_ = nh_.advertise<std_msgs::Float32>(finger_velocity_topic_, 10);

  // initialize variables
  finger_vel_msg_ = std_msgs::Float32();
  twist_msg_ = geometry_msgs::TwistStamped();
}

void PadPluginMoveitServoTeleoperation::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{

  // Calculate finger vel in steps/second and fill in finger vel message
  finger_vel_msg_.data = max_finger_speed_*(buttons[0].isPressed() - buttons[1].isPressed());

  if( finger_vel_msg_.data != 0 ){
    finger_control_pub_.publish(finger_vel_msg_);
  } else {

    double orientation_norm = sqrt(pow(axes[axis_angular_x_],2) + pow(axes[axis_angular_y_],2) + pow(axes[axis_angular_z_],2));
    double linear_norm = sqrt(pow(axes[axis_linear_x_],2) + pow(axes[axis_linear_y_],2) + pow(axes[axis_linear_z_],2));
    
    if(orientation_norm > linear_norm){
      // Angular servoing 
      twist_msg_.header.frame_id = end_effector_frame_;

      twist_msg_.twist.linear.x = 0;
      twist_msg_.twist.linear.y = 0;
      twist_msg_.twist.linear.z = 0;

      if(fabs(axes[axis_angular_x_]) < rot_deadband_){
        twist_msg_.twist.angular.x = 0;
      }else {twist_msg_.twist.angular.x = -max_angular_speed_ * axes[axis_angular_x_];}

      if(fabs(axes[axis_angular_y_]) < rot_deadband_){
        twist_msg_.twist.angular.y = 0;
      }else {twist_msg_.twist.angular.y = max_angular_speed_ * axes[axis_angular_y_];}

      if(fabs(axes[axis_angular_z_]) < rot_deadband_){
        twist_msg_.twist.angular.z = 0;
      }else {twist_msg_.twist.angular.z = max_angular_speed_ * axes[axis_angular_z_];}

    } else {
      // Cartesian servoing 
      twist_msg_.header.frame_id = base_link_frame_;

      if(fabs(axes[axis_linear_x_]) < linear_deadband_){
        twist_msg_.twist.linear.x = 0;
      }else {twist_msg_.twist.linear.x = max_linear_speed_ * axes[axis_linear_x_];}

      if(fabs(axes[axis_linear_y_]) < linear_deadband_){
        twist_msg_.twist.linear.y = 0;
      }else {twist_msg_.twist.linear.y = max_linear_speed_ * axes[axis_linear_y_];}

      if(fabs(axes[axis_linear_z_]) < linear_deadband_){
        twist_msg_.twist.linear.z = 0;
      }else {twist_msg_.twist.linear.z = max_linear_speed_ * axes[axis_linear_z_];}

      twist_msg_.twist.angular.x = 0;
      twist_msg_.twist.angular.y = 0;
      twist_msg_.twist.angular.z = 0;
    }

    twist_msg_.header.stamp = ros::Time::now();

    arm_control_pub_.publish(twist_msg_);
  }
}

}  // namespace pad_plugins