#ifndef PAD_PLUGIN_PTZ_H_
#define PAD_PLUGIN_PTZ_H_

#include <robotnik_msgs/set_ptz.h>
#include <robotnik_pad/generic_pad_plugin.h>

namespace pad_plugins
{
class PadPluginPtz : public GenericPadPlugin
{
public:
  PadPluginPtz();
  ~PadPluginPtz();

  virtual void initialize(const ros::NodeHandle &nh, const std::string &plugin_ns);
  virtual void execute(const std::vector<Button> &buttons, std::vector<float> &axes);

protected:
  int button_dead_man_;

  std::string ptz_service_name_;
  ros::ServiceClient set_ptz_client_;
};
}  // namespace pad_plugins
#endif  // PAD_PLUGIN_PTZ_H_