#ifndef PAD_PLUGIN_KINOVA_ARM_GEN3_H_
#define PAD_PLUGIN_KINOVA_ARM_GEN3_H_

#include <kortex_driver/TwistCommand.h>
#include <robotnik_pad/generic_pad_plugin.h>
#include <robotnik_pad_msgs/KinovaArmStatus.h>

namespace pad_plugins
{
class PadPluginKinovaArmGen3 : public GenericPadPlugin
{
public:
  // Probably this should be stablish in generic_pad_plugin

  PadPluginKinovaArmGen3();
  ~PadPluginKinovaArmGen3();

  virtual void initialize(const ros::NodeHandle& nh, const std::string& plugin_ns);
  virtual void execute(const std::vector<Button>& buttons, std::vector<float>& axes);

protected:
  std::string arm_control_topic_name_;
  std::string set_home_service_name_;
  double max_linear_speed_, max_angular_speed_;
  int button_deadman_, button_movement_deadman_, button_home_arm_;
  int axis_open_gripper_, axis_close_gripper_;
  int axis_linear_x_, axis_linear_y_, axis_linear_z_;
  int axis_angular_x_, axis_angular_y_, axis_angular_z_;
  int button_speed_up_, button_speed_down_;

  ros::Publisher arm_control_pub_, pad_status_pub_;

  ros::ServiceClient set_home_service_;

  robotnik_pad_msgs::KinovaArmStatus arm_status_msg_;

  double fingers_closure_percentage_;
  //! current velocity level used to compute the target velocity
  double current_velocity_level_;
  //! max velocity level allowed (Normally 1.0)
  double max_velocity_level_;
  //! min velocity level allowed (Normally 0.1 -> the 10% of max speed level)
  double min_velocity_level_;
  //! defines how much you can increase/decrease the max_velocity_level (Normally 0.1)
  double velocity_level_step_;

  kortex_driver::TwistCommand arm_control_msg_;
};
}  // namespace pad_plugins
#endif  // PAD_PLUGIN_ELEVATOR_H_
