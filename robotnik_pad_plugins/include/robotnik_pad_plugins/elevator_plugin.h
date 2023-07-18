#ifndef PAD_PLUGIN_ELEVATOR_H_
#define PAD_PLUGIN_ELEVATOR_H_

#include <robotnik_msgs/SetElevator.h>
#include <robotnik_pad/generic_pad_plugin.h>

namespace pad_plugins
{
class PadPluginElevator : public GenericPadPlugin
{
public:
  PadPluginElevator();
  ~PadPluginElevator();

  virtual void initialize(const ros::NodeHandle &nh, const std::string &plugin_ns);
  virtual void execute(const std::vector<Button> &buttons, const std::vector<Axes> &axes);

protected:
  int button_dead_man_;
  int axis_elevator_up_;
	int axis_elevator_down_;

  std::string elevator_service_name_;
  ros::ServiceClient set_elevator_client_;
};
}  // namespace pad_plugins
#endif  // PAD_PLUGIN_ELEVATOR_H_
