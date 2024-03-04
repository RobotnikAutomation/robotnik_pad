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
  bool not_required = false;

  pnh_ = ros::NodeHandle(nh, plugin_ns);

  readParam(pnh_, "config/deadman", button_dead_man_, button_dead_man_, required);
  readParam(pnh_, "config/axis_elevator", axis_elevator_, axis_elevator_, required);
  readParam(pnh_, "elevator_service_name", elevator_service_name_, elevator_service_name_, required);
  readParam(pnh_, "stop_elevator", stop_elevator_, false, not_required);

  // Service client
  set_elevator_client_ = nh_.serviceClient<robotnik_msgs::SetElevator>(elevator_service_name_);

  elevator_is_running_ = false;
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
      elevator_is_running_ = true;
      if (set_elevator_client_.call(elevator_msg_srv) != true || elevator_msg_srv.response.ret != true)
      {
        ROS_ERROR_NAMED("PadPluginElevator", "PadPluginElevator::execute: Error raising");
      }
    }
    if (axes[axis_elevator_] < -0.95)
    {
      robotnik_msgs::SetElevator elevator_msg_srv;
      elevator_msg_srv.request.action.action = robotnik_msgs::ElevatorAction::LOWER;
      elevator_is_running_ = true;
      ROS_INFO_NAMED("PadPluginElevator", "PadPluginElevator::execute: Lowering...");

      if (set_elevator_client_.call(elevator_msg_srv) != true || elevator_msg_srv.response.ret != true)
      {
        ROS_ERROR_NAMED("PadPluginElevator", "PadPluginElevator::execute: Error lowering");
      }
    }

    if (stop_elevator_==true){

      if (axes[axis_elevator_] > -0.1 && axes[axis_elevator_] < 0.1 && elevator_is_running_ == true)
      {
        robotnik_msgs::SetElevator elevator_msg_srv;
        elevator_msg_srv.request.action.action = robotnik_msgs::ElevatorAction::STOP;
        elevator_is_running_ = false;
        ROS_INFO_NAMED("PadPluginElevator", "PadPluginElevator::execute: Stoping...");

        if (set_elevator_client_.call(elevator_msg_srv) != true || elevator_msg_srv.response.ret != true)
        {
          ROS_ERROR_NAMED("PadPluginElevator", "PadPluginElevator::execute: Error stoping");
        }
      }
    }

  }
  else if (buttons[button_dead_man_].isReleased())
  {
  }
}

}  // namespace pad_plugins
