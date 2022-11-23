#include <robotnik_pad_plugins/poi_plugin.h>

namespace pad_plugins{


    PadPluginPoi::PadPluginPoi()
    {
    }


    PadPluginPoi::~PadPluginPoi()
    {
    }


    void PadPluginPoi::initialize(const ros::NodeHandle &nh, const std::string &plugin_ns)
    {
        bool required = true;
        pnh_ = ros::NodeHandle(nh, plugin_ns);
        nh_ = ros::NodeHandle();

        readParam(pnh_, "config/deadman", button_dead_man_, button_dead_man_, required);
        readParam(pnh_, "config/save_poi_l3", save_poi_l3_, save_poi_l3_, required);
        readParam(pnh_, "config/save_poi_r3", save_poi_r3_, save_poi_r3_, required);
        readParam(pnh_, "poi_service_name", poi_service_name_, poi_service_name_, required);

        toggle = true;
        counter = 0;

        // Service client
       save_robot_poi_client_ = nh_.serviceClient<std_srvs::Trigger>(poi_service_name_);

    }


    void PadPluginPoi::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
    {

        if (buttons[button_dead_man_].isPressed())
        {
 
            if(buttons[save_poi_l3_].isPressed() && buttons[save_poi_r3_].isPressed() && toggle == true){

                toggle = false;

                std_srvs::Trigger poi_trigger;

                if (save_robot_poi_client_.call(poi_trigger)){

                    ROS_INFO("PadPluginPoi::execute: new point saved");
                    counter++;

                }
                else{
                    ROS_ERROR("PadPluginPoi::execute: Failed to call the service %s", poi_service_name_.c_str());
                }


            }

            if(!buttons[save_poi_l3_].isPressed() && !buttons[save_poi_r3_].isPressed() && toggle == false){
                toggle = true;
            }

        }

        else {
            toggle = true;
        }

    }


}
