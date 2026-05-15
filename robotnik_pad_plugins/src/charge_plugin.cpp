#include <robotnik_pad_plugins/charge_plugin.h>

namespace pad_plugins
{
PadPluginCharge::PadPluginCharge()
: action_running_(false)
, sending_goal_(false)
, button_deadman_(5)
, button_charge_(0)
, button_uncharge_(2)
, action_timeout_(300.0)
, charge_action_name_("charge")
, uncharge_action_name_("uncharge")
{
}

PadPluginCharge::~PadPluginCharge()
{
}

void PadPluginCharge::initialize(const rclcpp::Node::SharedPtr& node, const std::string& plugin_ns)
{
    node_ = node;
    readParams(plugin_ns);

    charge_action_client_ = rclcpp_action::create_client<Charge>(node_, charge_action_name_);
    uncharge_action_client_ = rclcpp_action::create_client<Uncharge>(node_, uncharge_action_name_);

    // CHARGE Callbacks
    charge_goal_options_.goal_response_callback = 
        [this](const GoalHandleCharge::SharedPtr& goal_handle)         
    {
        chargeGoalCallback(goal_handle);
    };

    charge_goal_options_.result_callback =
        [this](const GoalHandleCharge::WrappedResult & result)
    {
        chargeResultCallback(result);
    };

    // UNCHARGE Callbacks
    uncharge_goal_options_.goal_response_callback =
        [this](const GoalHandleUncharge::SharedPtr & goal_handle)
    {
        unchargeGoalCallback(goal_handle);
    };

    uncharge_goal_options_.result_callback =
    [this](const GoalHandleUncharge::WrappedResult & result)
    {
        unchargeResultCallback(result);
    };
}

void PadPluginCharge::execute(const std::vector<Button>& buttons, std::vector<Axes>& axes)
{
    (void)axes; // Unused parameter

    if (sending_goal_)
    {
        return;
    }

    const bool deadman_pressed = buttons[button_deadman_].isPressed();

    if (action_running_)
    {
        if (isActionTimedOut())
        {
            stopAction();
        }

        if (deadman_pressed &&
            (buttons[button_charge_].isReleased() ||
             buttons[button_uncharge_].isReleased()))
        {
            stopAction();
        }

        return;
    }

    if (!deadman_pressed)
    {
        return;
    }

    if (buttons[button_charge_].isReleased())
    {
        sendChargeGoal();
    }

    else if(buttons[button_uncharge_].isReleased())
    {
        sendUnchargeGoal();
    }
}

void PadPluginCharge::readParams(const std::string ns)
{   
    readParam(node_, ns + ".config.button_deadman", button_deadman_, button_deadman_, true);
    readParam(node_, ns + ".config.button_charge", button_charge_, button_charge_, true);
    readParam(node_, ns + ".config.button_uncharge", button_uncharge_, button_uncharge_, true);
    charge_goal_.robot_dock_frame = "robot_base_docking_contact";
    readParam(node_, ns + ".charge.robot_dock_frame", charge_goal_.robot_dock_frame, charge_goal_.robot_dock_frame, true);    
    charge_goal_.dock_frame = "robotnik_marker_1";
    readParam(node_, ns + ".charge.dock_frame", charge_goal_.dock_frame, charge_goal_.dock_frame, true);
    charge_goal_.dock_offset = 0.1;
    readParam(node_, ns + ".charge.dock_offset", charge_goal_.dock_offset, charge_goal_.dock_offset, false);
    charge_goal_.retries = 3;
    readParam(node_, ns + ".charge.retries", charge_goal_.retries, charge_goal_.retries, false);

    readParam(node_, ns + ".charge.action_name", charge_action_name_, charge_action_name_, true);
    readParam(node_, ns + ".uncharge.action_name", uncharge_action_name_, uncharge_action_name_, true);
    readParam(node_, ns + ".action_timeout", action_timeout_, action_timeout_, true);
}

void PadPluginCharge::chargeGoalCallback(const GoalHandleCharge::SharedPtr& goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::chargeGoalCallback: Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "PadPluginCharge::chargeGoalCallback: Goal accepted by server, waiting for result");
        action_running_ = true;
        action_start_time_ = node_->now();
        current_charge_goal_handle_ = goal_handle;
    }
    sending_goal_ = false;
}

void PadPluginCharge::chargeResultCallback(const GoalHandleCharge::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "PadPluginCharge::chargeResultCallback: Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::chargeResultCallback: Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::chargeResultCallback: Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::chargeResultCallback: Unknown result code");
            break;
    }
    action_running_ = false;
}

void PadPluginCharge::unchargeGoalCallback(const GoalHandleUncharge::SharedPtr & goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::unchargeGoalCallback: Goal was rejected by server");
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "PadPluginCharge::unchargeGoalCallback: Goal accepted by server, waiting for result");
        action_running_ = true;
        action_start_time_ = node_->now();
        current_uncharge_goal_handle_ = goal_handle;
    }
    sending_goal_ = false;
}

void PadPluginCharge::unchargeResultCallback(const GoalHandleUncharge::WrappedResult & result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "PadPluginCharge::unchargeResultCallback: Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::unchargeResultCallback: Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::unchargeResultCallback: Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::unchargeResultCallback: Unknown result code");
            break;
    }
    action_running_ = false;
}

void PadPluginCharge::sendChargeGoal()
{
    RCLCPP_INFO(node_->get_logger(), "PadPluginCharge::sendChargeGoal: Charge action requested");

    if (!charge_action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::sendChargeGoal: Action server not available after waiting");
        return;
    }

    sending_goal_ = true;
    charge_action_client_->async_send_goal(charge_goal_, charge_goal_options_);
}

void PadPluginCharge::sendUnchargeGoal()
{
    RCLCPP_INFO(node_->get_logger(), "PadPluginCharge::sendUnchargeGoal: Uncharge action requested");

    if (!uncharge_action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(node_->get_logger(), "PadPluginCharge::sendUnchargeGoal: Action server not available after waiting");
        return;
    }

    sending_goal_ = true;
    Uncharge::Goal uncharge_goal;
    uncharge_action_client_->async_send_goal(uncharge_goal, uncharge_goal_options_);
}

bool PadPluginCharge::isActionTimedOut()
{
    bool condition = action_running_ && (node_->now() - action_start_time_).seconds() > action_timeout_;
    if (condition)
    {
        RCLCPP_WARN(node_->get_logger(), "PadPluginCharge::isActionTimedOut: Action timed out!");
    }
    return condition;
}

void PadPluginCharge::stopAction()
{
    RCLCPP_INFO(node_->get_logger(), "PadPluginCharge::stopAction: Stop charge/uncharge action requested");

    if (action_running_)
    {
        // Cancel charge goal if we have one
        if (current_charge_goal_handle_)
        {
            // Optional: only cancel if it looks cancelable (avoid spamming)
            const auto status = current_charge_goal_handle_->get_status();
            if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
            {
            charge_action_client_->async_cancel_goal(current_charge_goal_handle_);
            }
            current_charge_goal_handle_.reset();
        }

        // Cancel uncharge goal if we have one
        if (current_uncharge_goal_handle_)
        {
            const auto status = current_uncharge_goal_handle_->get_status();
            if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
                status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
            {
            uncharge_action_client_->async_cancel_goal(current_uncharge_goal_handle_);
            }
            current_uncharge_goal_handle_.reset();
        }

        action_running_ = false;
        RCLCPP_INFO(node_->get_logger(), "PadPluginCharge::stopAction: Charge/uncharge action stopped");
    }
}

} // namespace pad_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pad_plugins::PadPluginCharge, pad_plugins::GenericPadPlugin);
