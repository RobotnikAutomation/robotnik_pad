#ifndef PAD_PLUGIN_RISING_H_
#define PAD_PLUGIN_RISING_H_

#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Twist.h>
#include <robotnik_pad/generic_pad_plugin.h>
#include <robotnik_pad_msgs/MovementStatus.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
std_msgs::Float64 flipper_b_read;
std_msgs::Float64 flipper_f_read;
namespace pad_plugins
{
class PadPluginRising : public GenericPadPlugin
{
public:

  PadPluginRising();
  ~PadPluginRising();

  virtual void initialize(const ros::NodeHandle& nh, const std::string& plugin_ns);
  virtual void execute(const std::vector<Button>& buttons, std::vector<float>& axes);
  virtual void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

protected:
  int button_dead_man_, axis_linear_x_, axis_linear_y_, axis_angular_z_;
  int button_speed_up_, button_speed_down_, button_kinematic_mode_;
  double max_speed_;
  double max_steering_angle_;
  std::string cmd_topic_vel_;
  std_msgs::Float64 flipper_b_read;
  std_msgs::Float64 flipper_f_read;
double max_linear_speed_, max_angular_speed_;
  ros::Publisher ackermann_pub_, pad_status_pub_, right_wheel_pub_, left_wheel_pub_, front_flipper_wheel_pub_, back_flipper_wheel_pub_, twist_pub_;
  ros::Subscriber joint_states_sub_;
  robotnik_pad_msgs::MovementStatus movement_status_msg_;
  //! current velocity level used to compute the target velocity
  double current_velocity_level_;
  //! max velocity level allowed (Normally 1.0)
  double max_velocity_level_;
  //! min velocity level allowed (Normally 0.1 -> the 10% of max speed level)
  double min_velocity_level_;
  //! defines how much you can increase/decrease the max_velocity_level (Normally 0.1)
  double velocity_level_step_;
geometry_msgs::Twist cmd_twist_;
  ackermann_msgs::AckermannDrive cmd_ackermann_;
double wheel_base_;

};
}
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);  // namespace pad_plugins
#endif  // PAD_PLUGIN_RISING_H_
