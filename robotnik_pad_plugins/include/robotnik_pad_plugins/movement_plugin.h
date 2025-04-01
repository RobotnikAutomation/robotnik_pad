#ifndef PAD_PLUGIN_MOVEMENT_H
#define PAD_PLUGIN_MOVEMENT_H

#include <geometry_msgs/msg/twist.hpp>
#include <robotnik_pad/generic_pad_plugin.h>

namespace KinematicModes
{
    enum KinematicModes
    {
        DIFFERENTIAL = 0,
        OMNIDIRECTIONAL = 1,
        ACKERMANN = 2
    };
}

typedef KinematicModes::KinematicModes KinematicMode;

namespace pad_plugins
{
class PadPluginMovement : public GenericPadPlugin
{
public:
    PadPluginMovement();
    ~PadPluginMovement();

    virtual void initialize(const rclcpp::Node::SharedPtr& node, const std::string& plugin_ns) override;
    virtual void execute(const std::vector<Button>& buttons, std::vector<Axes>& axes) override;

protected:
    rclcpp::Node::SharedPtr node_;

    void readParams(const std::string ns);
    void decreaseSpeed(); 
    void increaseSpeed();
    void changeKinematicMode();
    void setSpeed(std::vector<Axes>& axes);

    uint8_t button_deadman_;
    uint8_t axis_linear_x_;
    uint8_t axis_linear_y_;
    uint8_t axis_angular_z_;    
    uint8_t button_kinematic_mode_;
    uint8_t button_speed_up_;
    uint8_t button_speed_down_;
    double max_linear_speed_;
    double max_angular_speed_;
    std::string cmd_topic_vel_;
    bool use_accel_watchdog_;
    double watchdog_duration_;
    double wheel_base_;

    std::vector<int64_t> axis_accel_watchdog_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

    double current_velocity_level_;
    double velocity_level_step_;
    double max_velocity_level_;
    double min_velocity_level_;

    uint8_t kinematic_mode_;
    geometry_msgs::msg::Twist cmd_twist_;
    std::vector<double> last_accel_value_;
    std::vector<rclcpp::Time> last_accel_time_;

    bool watchdog_activated_;
};
}  // namespace pad_plugins
#endif // PAD_PLUGIN_MOVEMENT_H

