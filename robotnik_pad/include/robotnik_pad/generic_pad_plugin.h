#ifndef GENERIC_PAD_PLUGIN_H_
#define GENERIC_PAD_PLUGIN_H_

#include <robotnik_pad/button.h>
#include <robotnik_pad/axes.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <sstream>

namespace pad_plugins
{
class GenericPadPlugin
{

public:
    virtual void initialize(const rclcpp::Node::SharedPtr& node, const std::string& plugin_ns) = 0;
    virtual void execute(const std::vector<Button>& buttons, std::vector<Axes>& axes) = 0;
    virtual ~GenericPadPlugin()
    {
    }

    //! Reads a parameter from the param server, and shows a message if parameter is not set
    template <typename T>
    bool readParam(const rclcpp::Node::SharedPtr& node, const std::string& name, T& value, const T& default_value,
                    bool required = false)
    {
        // Declare parameter with a default value
        node->declare_parameter<T>(name, default_value);

        // Check if the parameter exists
        if (!node->has_parameter(name))
        {
            std::stringstream ss;
            ss << default_value;
            // Handle logging based on whether the parameter is required
            if (required)
            {
                RCLCPP_ERROR(node->get_logger(), "No parameter '%s', using default value: %s", name.c_str(), ss.str().c_str());
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "No parameter '%s', using default value: %s", name.c_str(), ss.str().c_str());
            }

            // Assign default value if parameter is not found
            value = default_value;
            return false;
        }

        // Get the parameter value
        node->get_parameter(name, value);
        return true;
    }

protected:
    GenericPadPlugin()
    {
    }

protected:
    std::vector<Button> buttons_;
    std::vector<Axes> axes_;
};
}
#endif  // GENERIC_PAD_PLUGIN_H_