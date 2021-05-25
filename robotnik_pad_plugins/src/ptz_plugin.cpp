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
        readParam(pnh_, "config/home", button_home_, button_home_, required);
        readParam(pnh_, "config/vertical_arrow", button_vertical_arrow_, button_vertical_arrow_, required);
        readParam(pnh_, "config/horizontal_arrow", button_horizontal_arrow_, button_horizontal_arrow_, required);
        readParam(pnh_, "config/zoom_in", button_zoom_in_, button_zoom_in_, required);
        readParam(pnh_, "config/zoom_out", button_zoom_out_, button_zoom_out_, required);
        readParam(pnh_, "config/step_up", button_step_up_, button_step_up_, required);
        readParam(pnh_, "config/step_down", button_step_down_, button_step_down_, required);

        readParam(pnh_, "cmd_topic_ptz", cmd_topic_ptz_, cmd_topic_ptz_, required);

        readParam(pnh_, "position_increment", position_increment_, position_increment_, required);
        readParam(pnh_, "position_increment_limit", position_increment_limit_, position_increment_limit_, required);
        readParam(pnh_, "min_pan_position", min_pan_position_, min_pan_position_, required);
        readParam(pnh_, "max_pan_position", max_pan_position_, max_pan_position_, required);
        readParam(pnh_, "min_tilt_position", min_tilt_position_, min_tilt_position_, required);
        readParam(pnh_, "max_tilt_position", max_tilt_position_, max_tilt_position_, required);

        readParam(pnh_, "speed_control", speed_control_, speed_control_, required);
        readParam(pnh_, "speed_increment", speed_increment_, speed_increment_, required);
        readParam(pnh_, "speed_limit", speed_limit_, speed_limit_, required);


        ptz_pub_ = nh_.advertise<robotnik_msgs::ptz>(cmd_topic_ptz_, 10);
  
        old_cmd_ptz_ = robotnik_msgs::ptz();

        tilt_position_ = 0.0;
        pan_position_ = 0.0;
        zoom_position_ = 0.0;
        current_position_increment_= 0.1;

        tilt_speed_ = 0.0;
        pan_speed_ = 0.0;
        current_speed_ = 0.5;
        

        zoom_level_ = 50;

    }

    void PadPluginPtz::updateArrows(std::vector<float> &axes){

        /*
        * Arrow buttons are considered axes by the joy node. 
        * Therefore it is necessary to implement a bridge to use 
        * the methods of the Button class
        */

        if (axes[button_vertical_arrow_] > 0){
            up_arrow.press(1);
            down_arrow.press(0);

        }
        else if (axes[button_vertical_arrow_] < 0){
            up_arrow.press(0);
            down_arrow.press(1);

        }
        else{
            up_arrow.press(0);
            down_arrow.press(0);
        }


        if (axes[button_horizontal_arrow_] > 0){
            right_arrow.press(1);
            left_arrow.press(0);
        }
        else if (axes[button_horizontal_arrow_] < 0){
            right_arrow.press(0);
            left_arrow.press(1);
        }
        else{
            right_arrow.press(0);
            left_arrow.press(0);
        }

    }

    void PadPluginPtz::resetReleasedArrows(){

        if (up_arrow.isReleased())
            up_arrow.resetReleased();
        if (down_arrow.isReleased())
            down_arrow.resetReleased();
        if (right_arrow.isReleased())
            right_arrow.resetReleased();
        if (left_arrow.isReleased())
            left_arrow.resetReleased();
    }


    robotnik_msgs::ptz PadPluginPtz::positionControl(const std::vector<Button> &buttons){

        robotnik_msgs::ptz cmd_ptz;

        if(buttons[button_step_up_].isReleased()){
            
            current_position_increment_ = std::min(position_increment_limit_, current_position_increment_ + position_increment_);
            ROS_INFO("PadPluginPtz::positionControl: up position increment  = %.1f", current_position_increment_);
        }

        if(buttons[button_step_down_].isReleased()){
     
            current_position_increment_ = std::max( position_increment_ , current_position_increment_ - position_increment_);
            ROS_INFO("PadPluginPtz::positionControl: down position increment  = %.1f", current_position_increment_);
        }


        if (up_arrow.isReleased()){
            
            tilt_position_ = tilt_position_ + current_position_increment_;
        }

        if (down_arrow.isReleased()){
            tilt_position_ = tilt_position_ - current_position_increment_;   
        }

        if (right_arrow.isReleased()){
            pan_position_ = pan_position_ + current_position_increment_;  
        }

        if (left_arrow.isReleased()){
            pan_position_ = pan_position_ - current_position_increment_;    
        }



        if (buttons[button_zoom_in_].isReleased()){
            zoom_position_  =  zoom_position_ + (current_position_increment_*1000.0);
        }

        if (buttons[button_zoom_out_].isReleased()){
            zoom_position_  =  zoom_position_ - (current_position_increment_*1000.0);
        }


        cmd_ptz.pan = pan_position_;
        cmd_ptz.tilt = tilt_position_;
        cmd_ptz.zoom = zoom_position_;
        cmd_ptz.relative = false;
        
        return cmd_ptz;

    }

    robotnik_msgs::ptz PadPluginPtz::speedControl(const std::vector<Button> &buttons){

        robotnik_msgs::ptz cmd_ptz;

        if(buttons[button_step_up_].isReleased()){
            
            current_speed_ = std::min( speed_limit_ , current_speed_ + speed_increment_);
            ROS_INFO("PadPluginPtz::speedControl: up speed increment  = %.1f", current_speed_);
        }

        if(buttons[button_step_down_].isReleased()){
     
            current_speed_ = std::max( speed_increment_ , current_speed_ - speed_increment_);
            ROS_INFO("PadPluginPtz::speedControl: down speed increment  = %.1f", current_speed_);
        }


        if (up_arrow.isPressed()){
            tilt_speed_ = current_speed_;
        }

        if (down_arrow.isPressed()){
            tilt_speed_ = - current_speed_;     
        }

        if (right_arrow.isPressed()){
            pan_speed_ = current_speed_;   
        }

        if (left_arrow.isPressed()){
            pan_speed_ = - current_speed_;
        }


        if (!up_arrow.isPressed() && !down_arrow.isPressed()){
            tilt_speed_ = 0;
        }
        
        if (!left_arrow.isPressed() && !right_arrow.isPressed()){
            pan_speed_ = 0;
        }

        cmd_ptz.pan = pan_speed_;
        cmd_ptz.tilt = tilt_speed_;
        cmd_ptz.relative = false;

        return cmd_ptz;

    }

    void PadPluginPtz::zoomControl(const std::vector<Button> &buttons){
/*        
        if (buttons[button_zoom_in_].isReleased()){
            cmd_ptz_.zoom = cmd_ptz_.zoom + zoom_level_;
            cmd_ptz_.relative = true;      
        }

        if (buttons[button_zoom_out_].isReleased()){
            cmd_ptz_.zoom = cmd_ptz_.zoom - zoom_level_;
            cmd_ptz_.relative = true;      
        }
*/
    }


    robotnik_msgs::ptz PadPluginPtz::home(){

        robotnik_msgs::ptz cmd_ptz;

        tilt_position_ = 0.0;
        pan_position_ = 0.0;
        tilt_speed_ = 0.0;
        pan_speed_ = 0.0;

        cmd_ptz.pan = 0.0;
        cmd_ptz.tilt = 0.0;
        cmd_ptz.zoom = 0.0;
        cmd_ptz.relative = false;
        
        return cmd_ptz;
    }

    void PadPluginPtz::publishPtz(robotnik_msgs::ptz cmd_ptz){

        if (cmd_ptz != old_cmd_ptz_) {
            
            if ( cmd_ptz.relative != old_cmd_ptz_.relative && cmd_ptz.pan == 0 && cmd_ptz.tilt == 0
                 && old_cmd_ptz_.pan == 0 && old_cmd_ptz_.tilt == 0){
                ;   // Nothing to do
            }
            else{
                ptz_pub_.publish(cmd_ptz);
                old_cmd_ptz_ = cmd_ptz;
            } 
            
        }

    }


    void PadPluginPtz::execute(const std::vector<Button> &buttons, std::vector<float> &axes){

        // Revisar home y zoon en speed y pos
        // Incrementos por timeout

        robotnik_msgs::ptz cmd_ptz;

        updateArrows(axes);

        if (buttons[button_dead_man_].isPressed()){

            if (speed_control_ == false){
                cmd_ptz = positionControl(buttons);
                publishPtz(cmd_ptz);
            }
            else{
                cmd_ptz = speedControl(buttons);
                publishPtz(cmd_ptz);
            }

            if(buttons[button_home_].isReleased()){

                cmd_ptz = home();
                publishPtz(cmd_ptz);
            }

            
        }

        else if (buttons[button_dead_man_].isReleased()){

        }

        resetReleasedArrows();

    }

}  // namespace pad_plugins
