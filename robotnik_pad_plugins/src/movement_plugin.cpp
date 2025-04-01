#include <robotnik_pad_plugins/movement_plugin.h>

namespace pad_plugins
{
PadPluginMovement::PadPluginMovement()
{
}

PadPluginMovement::~PadPluginMovement()
{
}

void PadPluginMovement::initialize(const rclcpp::Node::SharedPtr& node, const std::string& plugin_ns)
{
    node_ = node;
    readParams(plugin_ns);


    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_vel_, 10);

    current_velocity_level_ = 0.1;
    velocity_level_step_ = 0.1;
    max_velocity_level_ = 1;
    min_velocity_level_ = 0.1;

    cmd_twist_ = geometry_msgs::msg::Twist();
    kinematic_mode_ = KinematicMode::DIFFERENTIAL;

    for (unsigned int i = 0; i < axis_accel_watchdog_.size(); i++)
    {
        last_accel_value_.push_back(0.0);
        last_accel_time_.push_back(node_->now());
    }

    watchdog_activated_ = false;
}

void PadPluginMovement::execute(const std::vector<Button>& buttons, std::vector<Axes>& axes)
{
    if (buttons[button_deadman_].isReleased())
    {
        cmd_twist_.linear.x = 0.0;
        cmd_twist_.linear.y = 0.0;
        cmd_twist_.angular.z = 0.0;
        twist_pub_->publish(cmd_twist_);
    }
    else if (buttons[button_deadman_].isPressed())
    {
        // Monitor watchdog
        if (use_accel_watchdog_)
        {
            // If new accelerameter value is different to last, we update watchdog values
            bool check_watchdog = true;
            for (unsigned int i = 0; i < axis_accel_watchdog_.size(); i++)
            {
                if (axes[axis_accel_watchdog_[i]].getValue() != last_accel_value_[i])
                {
                    last_accel_value_[i] = axes[axis_accel_watchdog_[i]].getValue();
                    last_accel_time_[i] = node_->now();
                    watchdog_activated_ = false;
                    check_watchdog = false;
                }
            }

            if (check_watchdog)
            {
                if (watchdog_activated_)
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "PadPluginMovement::execute: Command discarded by accelerometer watchdog!");
                    return;
                }
                // If watchdog is expired, we stop the robot
                bool timedout = true;
                for (unsigned int i = 0; i < last_accel_time_.size(); i++)
                {
                    if ((node_->now() - last_accel_time_[i]).seconds() < watchdog_duration_)
                    {
                        timedout = false;
                        break;
                    }
                }

                if (timedout)
                {
                    RCLCPP_WARN(node_->get_logger(), "PadPluginMovement::execute: Accelerometer watchdog timedout!");
                    cmd_twist_.linear.x = 0.0;
                    cmd_twist_.angular.z = 0.0;
                    twist_pub_->publish(cmd_twist_);
                    watchdog_activated_ = true;
                    return;
                }
            }
        }

        if (buttons[button_speed_down_].isReleased())
        {
            decreaseSpeed();
        }

        else if(buttons[button_speed_up_].isReleased())
        {
            increaseSpeed();
        }

        if (buttons[button_kinematic_mode_].isReleased())
        {
            changeKinematicMode();
        }

        setSpeed(axes);
        twist_pub_->publish(cmd_twist_);
    }
}

void PadPluginMovement::decreaseSpeed()
{
    current_velocity_level_ = std::max(min_velocity_level_, current_velocity_level_ - velocity_level_step_);
    RCLCPP_INFO(node_->get_logger(), "PadPluginMovement::decreaseSpeed: velocity level = %.1f%%", current_velocity_level_ * 100.0);
}

void PadPluginMovement::increaseSpeed()
{
    current_velocity_level_ = std::min(max_velocity_level_, current_velocity_level_ + velocity_level_step_);
    RCLCPP_INFO(node_->get_logger(), "PadPluginMovement::increaseSpeed: velocity level = %.1f%%", current_velocity_level_ * 100.0);
}

void PadPluginMovement::changeKinematicMode()
{
    if (kinematic_mode_ == KinematicModes::DIFFERENTIAL)
    {
        kinematic_mode_ = KinematicModes::OMNIDIRECTIONAL;
        RCLCPP_INFO(node_->get_logger(), "PadPluginMovement::changeKinematicMode: switch mode -> from Differential to Omnidirectional");
    }
    else if (kinematic_mode_ == KinematicModes::OMNIDIRECTIONAL)
    {
        if (wheel_base_ == 0)  // not set, ackermann mode cannot be selected
        {
            kinematic_mode_ = KinematicModes::DIFFERENTIAL;
            RCLCPP_INFO(node_->get_logger(), "PadPluginMovement::changeKinematicMode: switch mode -> from Omnidirectional to Differential");
        }
        else
        {
            kinematic_mode_ = KinematicModes::ACKERMANN;
            RCLCPP_INFO(node_->get_logger(), "PadPluginMovement::changeKinematicMode: switch mode -> from Omnidirectional to Ackermann");
        }
    }
    else if (kinematic_mode_ == KinematicModes::ACKERMANN)
    {
        kinematic_mode_ = KinematicModes::DIFFERENTIAL;
        RCLCPP_INFO(node_->get_logger(), "PadPluginMovement::changeKinematicMode: switch mode -> from Ackermann to Differential");
    }
}

void PadPluginMovement::setSpeed(std::vector<Axes>& axes)
{
    if (kinematic_mode_ == KinematicModes::ACKERMANN)
    {
      cmd_twist_.linear.x = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_].getValue() * std::cos(axes[axis_angular_z_].getValue() * (M_PI / 2.0));
      cmd_twist_.angular.z = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_].getValue() *
                             std::sin(axes[axis_angular_z_].getValue() * (M_PI / 2.0)) / wheel_base_;
    }
    else
    {
      cmd_twist_.linear.x = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_].getValue();
      cmd_twist_.angular.z = current_velocity_level_ * max_angular_speed_ * axes[axis_angular_z_].getValue();
    }

    if (kinematic_mode_ == KinematicModes::OMNIDIRECTIONAL)
    {
      cmd_twist_.linear.y = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_y_].getValue();
    }
    else
    {
      cmd_twist_.linear.y = 0.0;
    }
}

void PadPluginMovement::readParams(const std::string ns)
{   
    button_deadman_ = 5;
    readParam(node_, ns + ".config.button_deadman", button_deadman_, button_deadman_, true);
    axis_linear_x_ = 1;
    readParam(node_, ns + ".config.axis_linear_x", axis_linear_x_, axis_linear_x_, true);
    axis_linear_y_ = 0;
    readParam(node_, ns + ".config.axis_linear_y", axis_linear_y_, axis_linear_y_, true);
    axis_angular_z_ = 2;
    readParam(node_, ns + ".config.axis_angular_z", axis_angular_z_, axis_angular_z_, true);
    button_kinematic_mode_ = 6;
    readParam(node_, ns + ".config.button_kinematic_mode", button_kinematic_mode_, button_kinematic_mode_, true);
    button_speed_up_ = 3;
    readParam(node_, ns + ".config.button_speed_up", button_speed_up_, button_speed_up_, true);
    button_speed_down_ = 1;
    readParam(node_, ns + ".config.button_speed_down", button_speed_down_, button_speed_down_, true);
    max_linear_speed_ = 1.5;
    readParam(node_, ns + ".max_linear_speed", max_linear_speed_, max_linear_speed_, true);
    max_angular_speed_ = 1.5;
    readParam(node_, ns + ".max_angular_speed", max_angular_speed_, max_angular_speed_, true);
    cmd_topic_vel_ = "cmd_vel";
    readParam(node_, ns + ".cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_, true);
    use_accel_watchdog_ = true;
    readParam(node_, ns + ".config.use_accel_watchdog", use_accel_watchdog_, use_accel_watchdog_, true);
    watchdog_duration_ = 0.5;
    readParam(node_, ns + ".config.watchdog_duration", watchdog_duration_, watchdog_duration_, false);

    wheel_base_ = 0;
    readParam(node_, ns + ".wheel_base", wheel_base_, wheel_base_, false);

    std::vector<int64_t> axis = {8};
    node_->declare_parameter<std::vector<int64_t>>(ns + ".config.axis_watchdog", axis);
    node_->get_parameter(ns + ".config.axis_watchdog", axis_accel_watchdog_);
}

} // namespace pad_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(pad_plugins::PadPluginMovement, pad_plugins::GenericPadPlugin);