#ifndef PAD_PLUGIN_ACKERMANN_MOVEMENT_H_
#define PAD_PLUGIN_ACKERMANN_MOVEMENT_H_

#include <ackermann_msgs/AckermannDrive.h>
#include <robotnik_pad/generic_pad_plugin.h>
#include <robotnik_pad_msgs/MovementStatus.h>

namespace pad_plugins
{
class PadPluginAckermannMovement : public GenericPadPlugin
{
public:
  // Probably this should be stablish in generic_pad_plugin
  enum KinematicMode
  {
    Differential = 0,
    Omnidirectional = 1
  };

  PadPluginAckermannMovement();
  ~PadPluginAckermannMovement();

  virtual void initialize(const ros::NodeHandle& nh, const std::string& plugin_ns);
  virtual void execute(std::vector<Button>& buttons, std::vector<float>& axes);

protected:
  int button_dead_man_, axis_linear_x_, axis_linear_y_, axis_angular_z_, button_kinematic_mode_;
  int button_speed_up_, button_speed_down_;
  double max_linear_speed_, max_angular_speed_;
  std::string cmd_topic_vel_;

  ros::Publisher ackermann_pub_, pad_status_pub_;

  robotnik_pad_msgs::MovementStatus movement_status_msg_;
  //! current velocity level used to compute the target velocity
  double current_velocity_level_;
  //! max velocity level allowed (Normally 1.0)
  double max_velocity_level_;
  //! min velocity level allowed (Normally 0.1 -> the 10% of max speed level)
  double min_velocity_level_;
  //! defines how much you can increase/decrease the max_velocity_level (Normally 0.1)
  double velocity_level_step_;
  ackermann_msgs::AckermannDrive cmd_ackermann_;
  int kinematic_mode_;

protected:
  std::string kinematicModeToStr(int kinematic_mode);
};
}  // namespace pad_plugins
#endif  // PAD_PLUGIN_ELEVATOR_H_