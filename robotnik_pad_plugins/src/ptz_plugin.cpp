#include <robotnik_pad_plugins/ptz_plugin.h>

namespace pad_plugins
{
    PadPluginPtz::PadPluginPtz(){

    }

    PadPluginPtz::~PadPluginPtz(){

    }

    void PadPluginPtz::initialize(const ros::NodeHandle &nh, const std::string &plugin_ns){

        bool required = true;
        pnh_ = ros::NodeHandle(nh, plugin_ns);
        nh_ = nh;

        readParam(pnh_, "config/deadman", button_dead_man_, button_dead_man_, required);
        readParam(pnh_, "ptz_service_name", ptz_service_name_, ptz_service_name_, required);

        // Service client
        //set_ptz_client_ = nh_.serviceClient<robotnik_msgs::SetPTZ>(ptz_service_name_);

    }

    void PadPluginPtz::execute(const std::vector<Button> &buttons, std::vector<float> &axes){


        if (buttons[button_dead_man_].isPressed()){
           // ROS_INFO("hello");
        }

        else if (buttons[button_dead_man_].isReleased()){


        }

    }

}  // namespace pad_plugins
