#ifndef PAD_PLUGIN_BLKARC_H_
#define PAD_PLUGIN_BLKARC_H_

#include <std_srvs/Trigger.h>
#include <robotnik_pad/generic_pad_plugin.h>

namespace pad_plugins
{
class PadPluginBlkArc : public GenericPadPlugin
{
public:
  PadPluginBlkArc();
  ~PadPluginBlkArc();

  virtual void initialize(const ros::NodeHandle &nh, const std::string &plugin_ns);
  virtual void execute(const std::vector<Button> &buttons, std::vector<float> &axes);

protected:
  int button_dead_man_;
  double blk_arc_capture_;
  double blk_arc_static_capture_;

  std::string blk_arc_start_capture_service_name_;
  std::string blk_arc_stop_capture_service_name_;
  std::string blk_arc_start_static_capture_service_name_;
  std::string blk_arc_stop_static_capture_service_name_;
  ros::ServiceClient set_blk_arc_start_capture_client_;
  ros::ServiceClient set_blk_arc_stop_capture_client_;
  ros::ServiceClient set_blk_arc_start_static_capture_client_;
  ros::ServiceClient set_blk_arc_stop_static_capture_client_;
};
}  // namespace pad_plugins
#endif  // PAD_PLUGIN_BLKARC_H_