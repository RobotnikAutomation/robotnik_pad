#include <robotnik_pad_plugins/elevator_action_plugin.h>

namespace pad_plugins
{
PadPluginElevatorAction::PadPluginElevatorAction()
{
}

PadPluginElevatorAction::~PadPluginElevatorAction()
{
}

void PadPluginElevatorAction::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  bool required = true;
  bool not_required = false;

  pnh_ = ros::NodeHandle(nh, plugin_ns);

  readParam(pnh_, "config/deadman", button_dead_man_, button_dead_man_, required);
  readParam(pnh_, "config/axis_elevator", axis_elevator_, axis_elevator_, required);
  readParam(pnh_, "elevator_action_ns", elevator_action_ns_, elevator_action_ns_, required);
  readParam(pnh_, "stop_elevator", stop_elevator_, false, not_required);
  readParam(pnh_, "stop_elevator_dead_man", stop_elevator_dead_man_, false, not_required);
  // Service client
  elevator_action_client_ = std::make_shared
      <actionlib::SimpleActionClient<robotnik_msgs::SetElevatorAction>>(nh_, elevator_action_ns_, true);

  elevator_is_running_ = false;
}

void PadPluginElevatorAction::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{
  if (buttons[button_dead_man_].isPressed())
  {
    if (axes[axis_elevator_] > 0.95)
    {
      if (!elevator_is_running_)
      {
        elevator_is_running_ = sendGoal(robotnik_msgs::ElevatorAction::RAISE);
      }
    }
    if (axes[axis_elevator_] < -0.95)
    {
      if (!elevator_is_running_)
      {
        elevator_is_running_ = sendGoal(robotnik_msgs::ElevatorAction::LOWER);
      }
    }

    if (stop_elevator_)
    {
      if (axes[axis_elevator_] > -0.1 && axes[axis_elevator_] < 0.1 && elevator_is_running_ == true)
      {
        elevator_is_running_ = !sendGoal(robotnik_msgs::ElevatorAction::STOP);
      }
    }

  }
  else if (buttons[button_dead_man_].isReleased())
  {
    if (stop_elevator_dead_man_ && elevator_is_running_)
    {
      elevator_is_running_ = !sendGoal(robotnik_msgs::ElevatorAction::STOP);
    }
  }
}

void PadPluginElevatorAction::actionDoneCb(const actionlib::SimpleClientGoalState& state,  const robotnik_msgs::SetElevatorResultConstPtr& result)
{
  elevator_is_running_ = false;
}

bool PadPluginElevatorAction::sendGoal(const int action)
{
  robotnik_msgs::SetElevatorGoal elevator_goal;
  elevator_goal.action.action = action;
  ROS_INFO_NAMED("PadPluginElevatorAction", "PadPluginElevatorAction::execute: %d", action);
  elevator_action_client_->sendGoal(elevator_goal,
                                  boost::bind(&PadPluginElevatorAction::actionDoneCb, this, _1, _2));
  auto state = elevator_action_client_->getState();
  if (!(state == actionlib::SimpleClientGoalState::PENDING ||
  state == actionlib::SimpleClientGoalState::ACTIVE))
  {
    ROS_ERROR_NAMED("PadPluginElevatorAction", "PadPluginElevatorAction::execute: Goal was not accepted. Current state: %s", state.toString().c_str());
    return false;
  }
  return true;
}


}  // namespace pad_plugins