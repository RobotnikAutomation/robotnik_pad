#ifndef PAD_PLUGIN_MOVEMENT_H_
#define PAD_PLUGIN_MOVEMENT_H_

#include <geometry_msgs/Twist.h>
#include <robotnik_pad/generic_pad_plugin.h>

namespace pad_plugins
{
class PadPluginMovement : public GenericPadPlugin
{
public:
  // Probably this should be stablish in generic_pad_plugin
  enum KinematicMode
  {
    Differential = 0,
    Omnidirectional = 1
  };

  PadPluginMovement();
  ~PadPluginMovement();

  virtual void initialize(const ros::NodeHandle& nh, const std::string& plugin_ns);
  virtual void execute(std::vector<Button>& buttons, std::vector<float>& axes);

protected:
  int button_dead_man_, axis_linear_x_, axis_linear_y_, axis_angular_z_, button_kinematic_mode_;
  int button_speed_up_, button_speed_down_;
  double scale_linear_, scale_angular_;

  ros::Publisher twist_pub_;
  double current_vel_;
  geometry_msgs::Twist cmd_twist_;
  int kinematic_mode_;
};
}  // namespace
#endif  // PAD_PLUGIN_ELEVATOR_H_