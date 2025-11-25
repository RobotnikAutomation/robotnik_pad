#ifndef PAD_PLUGIN_ELEVATOR_ACTION_H_
#define PAD_PLUGIN_ELEVATOR_ACTION_H_

#include <robotnik_msgs/ElevatorAction.h>
#include <robotnik_msgs/SetElevatorAction.h>
#include <robotnik_msgs/SetElevatorActionGoal.h>
#include <robotnik_msgs/SetElevatorGoal.h>
#include <robotnik_msgs/SetElevatorResult.h>
#include <robotnik_pad/generic_pad_plugin.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

namespace pad_plugins
{
class PadPluginElevatorAction : public GenericPadPlugin
{
public:
  PadPluginElevatorAction();
  ~PadPluginElevatorAction();

  virtual void initialize(const ros::NodeHandle &nh, const std::string &plugin_ns);
  virtual void execute(const std::vector<Button> &buttons, std::vector<float> &axes);

protected:
  int button_dead_man_;
  double axis_elevator_;
  bool elevator_is_running_;
  bool stop_elevator_;
  bool stop_elevator_dead_man_;

  std::string elevator_action_ns_;
  std::shared_ptr<actionlib::SimpleActionClient<robotnik_msgs::SetElevatorAction>> elevator_action_client_;

  bool sendGoal(const int action);
  void actionDoneCb(const actionlib::SimpleClientGoalState& state,  const robotnik_msgs::SetElevatorResultConstPtr& result);
};
}; // namespace pad_plugins
#endif  // PAD_PLUGIN_ELEVATOR_ACTION_H_