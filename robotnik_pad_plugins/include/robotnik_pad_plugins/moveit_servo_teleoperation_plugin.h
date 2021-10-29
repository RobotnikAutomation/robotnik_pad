#ifndef PAD_PLUGIN_MOVEIT_SERVO_TELEOPERATION_H_
#define PAD_PLUGIN_MOVEIT_SERVO_TELEOPERATION_H_

#include <robotnik_pad/generic_pad_plugin.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>


namespace pad_plugins
{
class PadPluginMoveitServoTeleoperation : public GenericPadPlugin
{
public:
  // Probably this should be stablish in generic_pad_plugin

  PadPluginMoveitServoTeleoperation();
  ~PadPluginMoveitServoTeleoperation();

  virtual void initialize(const ros::NodeHandle& nh, const std::string& plugin_ns);
  virtual void execute(const std::vector<Button>& buttons, std::vector<float>& axes);

protected:
  std::string moveit_servo_twist_topic_, finger_velocity_topic_, end_effector_frame_, base_link_frame_;
  double max_linear_speed_, max_angular_speed_, max_finger_speed_, rot_deadband_, linear_deadband_;
  int axis_linear_x_, axis_linear_y_, axis_linear_z_;
  int axis_angular_x_, axis_angular_y_, axis_angular_z_;
  int button_open_, button_close_;

  ros::Publisher arm_control_pub_, finger_control_pub_;

  std_msgs::Float32 finger_vel_msg_;
  geometry_msgs::TwistStamped twist_msg_;

};
}  // namespace pad_plugins
#endif  // PAD_PLUGIN_MOVEIT_SERVO_TELEOPERATION_H_
