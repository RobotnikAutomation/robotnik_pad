#ifndef PAD_PLUGIN_CHARGE_H
#define PAD_PLUGIN_CHARGE_H

#include <geometry_msgs/msg/twist.hpp>
#include <robotnik_pad/generic_pad_plugin.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robotnik_navigation_msgs/action/charge.hpp>
#include <robotnik_navigation_msgs/action/uncharge.hpp>
#include <action_msgs/msg/goal_status.hpp>

namespace pad_plugins
{
class PadPluginCharge : public GenericPadPlugin
{
using Charge = robotnik_navigation_msgs::action::Charge;
using Uncharge = robotnik_navigation_msgs::action::Uncharge;
using GoalHandleCharge = rclcpp_action::ClientGoalHandle<Charge>;
using GoalHandleUncharge = rclcpp_action::ClientGoalHandle<Uncharge>;

public:
    PadPluginCharge();
    ~PadPluginCharge();
    virtual void initialize(const rclcpp::Node::SharedPtr& node, const std::string& plugin_ns) override;
    virtual void execute(const std::vector<Button>& buttons, std::vector<Axes>& axes) override;

protected:
    void chargeGoalCallback(const GoalHandleCharge::SharedPtr& goal_handle);
    void chargeResultCallback(const GoalHandleCharge::WrappedResult & result);
    void unchargeGoalCallback(const GoalHandleUncharge::SharedPtr & goal_handle);
    void unchargeResultCallback(const GoalHandleUncharge::WrappedResult & result);
    void sendChargeGoal();
    void sendUnchargeGoal();
    void readParams(const std::string ns);
    bool isActionTimedOut();
    void stopAction();

    bool action_running_;
    bool sending_goal_;
    uint8_t button_deadman_;
    uint8_t button_charge_;
    uint8_t button_uncharge_;
    double action_timeout_;

    std::string charge_action_name_;
    std::string uncharge_action_name_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Time action_start_time_;
    
    Charge::Goal charge_goal_;

    rclcpp_action::Client<Charge>::SharedPtr charge_action_client_;
    rclcpp_action::Client<Uncharge>::SharedPtr uncharge_action_client_;
    rclcpp_action::Client<Charge>::SendGoalOptions charge_goal_options_;
    rclcpp_action::Client<Uncharge>::SendGoalOptions uncharge_goal_options_;
    GoalHandleCharge::SharedPtr current_charge_goal_handle_;
    GoalHandleUncharge::SharedPtr current_uncharge_goal_handle_;


};
}  // namespace pad_plugins
#endif // PAD_PLUGIN_CHARGE_H

