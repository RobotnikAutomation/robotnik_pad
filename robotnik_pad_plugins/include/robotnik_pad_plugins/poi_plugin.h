#ifndef PAD_PLUGIN_POI_H_
#define PAD_PLUGIN_POI_H_

#include <robotnik_msgs/SetString.h>
#include <std_srvs/Trigger.h>
#include <robotnik_pad/generic_pad_plugin.h>

namespace pad_plugins
{
class PadPluginPoi : public GenericPadPlugin
{
public:

  PadPluginPoi();
  ~PadPluginPoi();

  virtual void initialize(const ros::NodeHandle& nh, const std::string& plugin_ns);
  virtual void execute(const std::vector<Button>& buttons, std::vector<float>& axes);

  int button_dead_man_;
  int save_poi_l3_;
  int save_poi_r3_;

  int counter;

protected:

  bool toggle;

  std::string poi_service_name_;
  ros::ServiceClient save_robot_poi_client_;

};


}  // namespace pad_plugins
#endif  // PAD_PLUGIN_ELEVATOR_H_
