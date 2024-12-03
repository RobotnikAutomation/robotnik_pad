#ifndef PAD_PLUGIN_CHANGE_VOGUI_ACKERMANN_H_
#define PAD_PLUGIN_CHANGE_VOGUI_ACKERMANN_H_

#include <robotnik_msgs/SetString.h>
#include <robotnik_pad/generic_pad_plugin.h>

namespace pad_plugins
{
class PadPluginChangeVoguiAckermann : public GenericPadPlugin
{
public:

  PadPluginChangeVoguiAckermann();
  ~PadPluginChangeVoguiAckermann();

  virtual void initialize(const ros::NodeHandle& nh, const std::string& plugin_ns);
  virtual void execute(const std::vector<Button>& buttons, std::vector<float>& axes);

  int button_dead_man_;
  int change_kinematics_;

protected:

  std::string change_kinematics_service_name_;
  ros::ServiceClient change_kinematics_client_;

};


}  // namespace pad_plugins
#endif  // PAD_PLUGIN_CHANGE_VOGUI_ACKERMANN_H_
