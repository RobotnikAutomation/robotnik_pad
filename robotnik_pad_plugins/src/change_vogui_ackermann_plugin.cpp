#include <robotnik_pad_plugins/change_vogui_ackermann_plugin.h>

namespace pad_plugins{


    PadPluginChangeVoguiAckermann::PadPluginChangeVoguiAckermann()
    {
    }


    PadPluginChangeVoguiAckermann::~PadPluginChangeVoguiAckermann()
    {
    }


    void PadPluginChangeVoguiAckermann::initialize(const ros::NodeHandle &nh, const std::string &plugin_ns)
    {
        bool required = true;
        pnh_ = ros::NodeHandle(nh, plugin_ns);
        nh_ = ros::NodeHandle();

        readParam(pnh_, "config/deadman", button_dead_man_, button_dead_man_, required);
        readParam(pnh_, "config/button_change_kinematics", change_kinematics_, change_kinematics_, required);
        readParam(pnh_, "change_kinematics_service_name", change_kinematics_service_name_, change_kinematics_service_name_, required);

        // Service client
       change_kinematics_client_ = nh_.serviceClient<robotnik_msgs::SetString>(change_kinematics_service_name_);

    }


    void PadPluginChangeVoguiAckermann::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
    {

        if (buttons[button_dead_man_].isPressed())
        {
 
            if(buttons[change_kinematics_].isReleased()) {

                robotnik_msgs::SetString change_kin;
                change_kin.request.data = "toggle";
                change_kinematics_client_.call(change_kin);

                if (change_kin.response.ret.success){

                    ROS_INFO("PadPluginChangeVoguiAckermann::execute: Kinematics changed");
                }
                else{
                    ROS_ERROR("PadPluginChangeVoguiAckermann::execute: Failed to call the service %s or change kinematics", change_kinematics_service_name_.c_str());
                }
            }
        }
    }


}
