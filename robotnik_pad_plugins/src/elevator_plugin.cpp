#include <robotnik_pad_plugins/elevator_plugin.h>

namespace pad_plugins
{
PadPluginElevator::PadPluginElevator()
{
}

PadPluginElevator::~PadPluginElevator()
{
}

void PadPluginElevator::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  bool required = true;
  pnh_ = ros::NodeHandle(nh, plugin_ns);

  readParam(pnh_, "config/deadman", button_dead_man_, button_dead_man_, required);
  readParam(pnh_, "config/axis_elevator", axis_elevator_, axis_elevator_, required);
  readParam(pnh_, "elevator_service_name", elevator_service_name_, elevator_service_name_, required);

  // Service client
  set_elevator_client_ = nh_.serviceClient<robotnik_msgs::SetElevator>(elevator_service_name_);
}

void PadPluginElevator::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_dead_man_].isPressed())
  {
    if (axes[axis_elevator_] > 0.95)
    {
      robotnik_msgs::SetElevator elevator_msg_srv;
      elevator_msg_srv.request.action.action = robotnik_msgs::ElevatorAction::RAISE;
      ROS_INFO_NAMED("PadPluginElevator", "PadPluginElevator::execute: Raising...");
      if (set_elevator_client_.call(elevator_msg_srv) != true || elevator_msg_srv.response.ret != true)
      {
        ROS_ERROR_NAMED("PadPluginElevator", "PadPluginElevator::execute: Error raising");
      }
    }
    if (axes[axis_elevator_] < -0.95)
    {
      robotnik_msgs::SetElevator elevator_msg_srv;
      elevator_msg_srv.request.action.action = robotnik_msgs::ElevatorAction::LOWER;

      ROS_INFO_NAMED("PadPluginElevator", "PadPluginElevator::execute: Lowering...");

      if (set_elevator_client_.call(elevator_msg_srv) != true || elevator_msg_srv.response.ret != true)
      {
        ROS_ERROR_NAMED("PadPluginElevator", "PadPluginElevator::execute: Error lowering");
      }
    }
  }
  else if (buttons[button_dead_man_].isReleased())
  {
  }
}

}  // namespace pad_plugins
