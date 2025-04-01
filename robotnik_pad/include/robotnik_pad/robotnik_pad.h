#ifndef _ROBOTNIK_PAD_
#define _ROBOTNIK_PAD_

#include <math.h>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <pluginlib/class_loader.hpp>
#include <robotnik_pad/generic_pad_plugin.h>
#include <robotnik_pad/button.h>
#include <robotnik_pad/axes.h>

#define DEFAULT_THREAD_DESIRED_HZ 40.0

class RobotnikPad : public rclcpp::Node
{
public:
    RobotnikPad();
    ~RobotnikPad() {};
    void start();

    //! Reads a parameter from the param server, and shows a message if parameter is not set
    template <typename T>
    bool readParam(const std::string& name, T& value, const T& default_value,
                    bool required = false)
    {
        // Declare parameter with a default value
        this->declare_parameter<T>(name, default_value);

        // Check if the parameter exists
        if (!this->has_parameter(name))
        {
            std::stringstream ss;
            ss << default_value;
            // Handle logging based on whether the parameter is required
            if (required)
            {
                RCLCPP_ERROR(this->get_logger(), "No parameter '%s', using default value: %s", name.c_str(), ss.str().c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No parameter '%s', using default value: %s", name.c_str(), ss.str().c_str());
            }

            // Assign default value if parameter is not found
            value = default_value;
            return false;
        }

        // Get the parameter value
        this->get_parameter(name, value);
        return true;
    }

protected:
    void rosReadParams();
    void setup();
    void rosSetup();
    void readPluginsFromParams(const std::vector<std::string>& names, std::map<std::string, std::string>& plugins_definitions);
    void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
    void controlLoop();

    double desired_freq_;
    uint8_t num_of_buttons_;
    uint8_t num_of_axes_;
    std::string joy_topic_;
    double joy_timeout_;
    std::map<std::string, std::string> plugins_from_params_;

    std::vector<Button> buttons_;
    std::vector<Axes> axes_;

    pluginlib::ClassLoader<pad_plugins::GenericPadPlugin>* pad_plugins_loader_;
    std::vector<std::shared_ptr<pad_plugins::GenericPadPlugin>> plugins_;

    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Time joy_topic_last_time_received_;
};

#endif // _ROBOTNIK_PAD_