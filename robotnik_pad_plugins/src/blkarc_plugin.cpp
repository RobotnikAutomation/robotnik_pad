#include <robotnik_pad_plugins/blkarc_plugin.h>

namespace pad_plugins
{
PadPluginBlkArc::PadPluginBlkArc()
{
}

PadPluginBlkArc::~PadPluginBlkArc()
{
}

void PadPluginBlkArc::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  bool required = true;
  pnh_ = ros::NodeHandle(nh, plugin_ns);

  readParam(pnh_, "config/deadman", button_dead_man_, button_dead_man_, required);
  readParam(pnh_, "config/blk_arc_capture", blk_arc_capture_, blk_arc_capture_, required);
  readParam(pnh_, "config/blk_arc_static_capture", blk_arc_static_capture_, blk_arc_static_capture_, required);
  readParam(pnh_, "blk_arc_start_capture_service_name", blk_arc_start_capture_service_name_, blk_arc_start_capture_service_name_, required);
  readParam(pnh_, "blk_arc_stop_capture_service_name", blk_arc_stop_capture_service_name_, blk_arc_stop_capture_service_name_, required);
  readParam(pnh_, "blk_arc_start_static_capture_service_name", blk_arc_start_static_capture_service_name_, blk_arc_start_static_capture_service_name_, required);
  readParam(pnh_, "blk_arc_stop_static_capture_service_name", blk_arc_stop_static_capture_service_name_, blk_arc_stop_static_capture_service_name_, required);

  // Service client
  set_blk_arc_start_capture_client_ = nh_.serviceClient<std_srvs::Trigger>(blk_arc_start_capture_service_name_);
  set_blk_arc_stop_capture_client_ = nh_.serviceClient<std_srvs::Trigger>(blk_arc_stop_capture_service_name_);
  set_blk_arc_start_static_capture_client_ = nh_.serviceClient<std_srvs::Trigger>(blk_arc_start_static_capture_service_name_);
  set_blk_arc_stop_static_capture_client_ = nh_.serviceClient<std_srvs::Trigger>(blk_arc_stop_static_capture_service_name_);
}

void PadPluginBlkArc::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_dead_man_].isPressed())
  {
    if (axes[blk_arc_capture_] > 0.95)
    {
      std_srvs::Trigger blk_trigger_msg_srv;
      ROS_INFO_NAMED("PadPluginBlkArc", "PadPluginBlkArc::execute: Starting Capture...");
      if (set_blk_arc_start_capture_client_.call(blk_trigger_msg_srv) != true || blk_trigger_msg_srv.response.success != true)
      {
        ROS_ERROR_NAMED("PadPluginBlkArc", "PadPluginBlkArc::execute: Error starting capture");
      }
    }
    if (axes[blk_arc_capture_] < -0.95)
    {
      std_srvs::Trigger blk_trigger_msg_srv;
      ROS_INFO_NAMED("PadPluginBlkArc", "PadPluginBlkArc::execute: Stoping Capture...");
      if (set_blk_arc_stop_capture_client_.call(blk_trigger_msg_srv) != true || blk_trigger_msg_srv.response.success != true)
      {
        ROS_ERROR_NAMED("PadPluginBlkArc", "PadPluginBlkArc::execute: Error stoping capture");
      }
    }
    if (axes[blk_arc_static_capture_] > 0.95)
    {
      std_srvs::Trigger blk_trigger_msg_srv;
      ROS_INFO_NAMED("PadPluginBlkArc", "PadPluginBlkArc::execute: Starting Static Capture...");
      if (set_blk_arc_start_static_capture_client_.call(blk_trigger_msg_srv) != true || blk_trigger_msg_srv.response.success != true)
      {
        ROS_ERROR_NAMED("PadPluginBlkArc", "PadPluginBlkArc::execute: Error starting static capture");
      }
    }
    if (axes[blk_arc_static_capture_] < -0.95)
    {
      std_srvs::Trigger blk_trigger_msg_srv;
      ROS_INFO_NAMED("PadPluginBlkArc", "PadPluginBlkArc::execute: Stoping Static Capture...");
      if (set_blk_arc_stop_static_capture_client_.call(blk_trigger_msg_srv) != true || blk_trigger_msg_srv.response.success != true)
      {
        ROS_ERROR_NAMED("PadPluginBlkArc", "PadPluginBlkArc::execute: Error stoping static capture");
      }
    }
  }
  else if (buttons[button_dead_man_].isReleased())
  {
  }
}

}  // namespace pad_plugins
