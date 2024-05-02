#include <robotnik_pad_plugins/rising_plugin.h>
#include <std_msgs/Float64.h>

namespace pad_plugins
{
PadPluginRising::PadPluginRising()
{
}

PadPluginRising::~PadPluginRising()
{
  // Stop tube extensor when pad dies
  digitalWrite(tube_enable_pin_, false);
  digitalWrite(tube_rollup_pin_, false);
  digitalWrite(tube_unroll_pin_, false);
  digitalWrite(back_tube_unroll_pin_, false);
  digitalWrite(back_tube_rollup_pin_, false);
}

void PadPluginRising::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  // Retrieve control parameters from ROS parameter server
  bool required = true;
  pnh_ = ros::NodeHandle(nh, plugin_ns);
  nh_ = ros::NodeHandle();
  button_dead_man_ = 5;
  readParam(pnh_, "config/button_deadman", button_dead_man_, button_dead_man_, required);
  button_power_on_arm_ = 8;
  readParam(pnh_, "config/button_power_on_arm", button_power_on_arm_, button_power_on_arm_, required);
  button_power_off_arm_ = 9;
  readParam(pnh_, "config/button_power_off_arm", button_power_off_arm_, button_power_off_arm_, required);
  button_kinematic_mode_ = 7;
  readParam(pnh_, "config/button_kinematic_mode", button_kinematic_mode_, button_kinematic_mode_, required);
  axis_frontflipper_ = 1;
  readParam(pnh_, "config/axis_frontflipper", axis_frontflipper_, axis_frontflipper_, required);
  axis_backflipper_ = 5;
  readParam(pnh_, "config/axis_backflipper", axis_backflipper_, axis_backflipper_, required);
  axis_linear_x_ = 1;
  readParam(pnh_, "config/axis_linear_x", axis_linear_x_, axis_linear_x_, required);
  axis_linear_y_ = 0;
  readParam(pnh_, "config/axis_linear_y", axis_linear_y_, axis_linear_y_, required);
  axis_angular_z_ = 2;
  readParam(pnh_, "config/axis_angular_z", axis_angular_z_, axis_angular_z_, required);
  button_speed_up_ = 3;
  readParam(pnh_, "config/button_speed_up", button_speed_up_, button_speed_up_, required);
  button_speed_down_ = 1;
  readParam(pnh_, "config/button_speed_down", button_speed_down_, button_speed_down_, required);
  axis_horizontal_arrow_ = 9;
  readParam(pnh_, "config/axis_record_water_data", axis_horizontal_arrow_, axis_horizontal_arrow_, required);
  axis_vertical_arrow_ = 10;
  readParam(pnh_, "config/axis_tube_extensor", axis_vertical_arrow_, axis_vertical_arrow_, required);
  max_speed_ = 1.5;
  readParam(pnh_, "max_speed", max_speed_, max_speed_, required);
  max_linear_speed_ = 1.5;
  readParam(pnh_, "max_linear_speed", max_linear_speed_, max_linear_speed_, required);
  max_angular_speed_ = 3.0;
  readParam(pnh_, "max_angular_speed", max_angular_speed_, max_angular_speed_, required);
  max_steering_angle_ = 1.57;
  readParam(pnh_, "max_steering_angle", max_steering_angle_, max_steering_angle_, required);
  cmd_topic_vel_ = "cmd_vel";
  readParam(pnh_, "cmd_topic_vel", cmd_topic_vel_, cmd_topic_vel_, required);
  wheel_base_ = 0;
  readParam(pnh_, "wheel_base", wheel_base_, wheel_base_);
  tube_enable_pin_ = 0;
  readParam(pnh_, "tube_enable_pin", tube_enable_pin_, tube_enable_pin_);
  tube_rollup_pin_ = 0;
  readParam(pnh_, "tube_rollup_pin", tube_rollup_pin_, tube_rollup_pin_);
  tube_unroll_pin_ = 0;
  readParam(pnh_, "tube_unroll_pin", tube_unroll_pin_, tube_unroll_pin_);
  back_tube_unroll_pin_ = 0;
  readParam(pnh_, "back_tube_unroll_pin", back_tube_unroll_pin_, back_tube_unroll_pin_);
  back_tube_rollup_pin_ = 0;
  readParam(pnh_, "back_tube_rollup_pin", back_tube_rollup_pin_, back_tube_rollup_pin_);
  water_record_ = "odysseus_gcs/start_stop";
  readParam(pnh_, "water_record", water_record_, water_record_, required);

  enable_web_buttons_ = false;
  readParam(pnh_, "config/enable_web_buttons", enable_web_buttons_, enable_web_buttons_, required);
  button_tube_rollup_ = 9;
  readParam(pnh_, "config/button_tube_rollup", button_tube_rollup_, button_tube_rollup_, required);
  button_tube_unroll_ = 10;
  readParam(pnh_, "config/button_tube_unroll", button_tube_unroll_, button_tube_unroll_, required);
  button_start_record_water_data_ = 9;
  readParam(pnh_, "config/button_start_record_water_data", button_start_record_water_data_, button_start_record_water_data_, required);
  button_stop_record_water_data_ = 10;
  readParam(pnh_, "config/button_stop_record_water_data", button_stop_record_water_data_, button_stop_record_water_data_, required);

  // Publishers
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 10);
  pad_status_pub_ = pnh_.advertise<robotnik_pad_msgs::MovementStatus>("status", 10);
  water_record_pub_ = nh_.advertise<std_msgs::Bool>(water_record_, 10);
  toggle_arm_pub_ = nh_.advertise<std_msgs::Bool>("/my_gen3/toggle_kinova", 1);
  	
  // RISING COMMANDS
	right_wheel_pub_ = nh_.advertise<std_msgs::Float64>("front_right_master_wheel_joint_controller/command", 1);
	left_wheel_pub_ = nh_.advertise<std_msgs::Float64>("front_left_master_wheel_joint_controller/command", 1);
  front_flipper_wheel_pub_ = nh_.advertise<std_msgs::Float64>("front_flipper_joint_controller/command", 1);
	back_flipper_wheel_pub_ = nh_.advertise<std_msgs::Float64>("back_flipper_joint_controller/command", 1);
  tube_extensor_srv_ = nh_.serviceClient<robotnik_msgs::set_digital_output>("robotnik_base_hw/set_digital_output");

  // SUBSCRIBER JOINT STATES
  joint_states_sub_ = nh_.subscribe("joint_states", 1, &PadPluginRising::jointStatesCallback, this);

  // Initialize variables
  current_velocity_level_ = 0.2; // 0.1
  velocity_level_step_ = 0.1;
  max_velocity_level_ = 1.0;
  min_velocity_level_ = 0.1;
  flipper_b_read.data = 0.0; 
  flipper_f_read.data = 0.0;
  cmd_twist_ = geometry_msgs::Twist();
  movement_status_msg_ = robotnik_pad_msgs::MovementStatus();
  tube_extensor_working = false;
}



void PadPluginRising::execute(const std::vector<Button>& buttons, std::vector<float>& axes)
{
  std_msgs::Float64 traction_l;
  std_msgs::Float64 traction_r;
  traction_l.data = 0;
  traction_r.data = 0;
  std_msgs::Float64 flipper_b;
  std_msgs::Float64 flipper_f;
  flipper_b.data = flipper_b_read.data ;
  flipper_f.data = flipper_f_read.data ;
  time_t now = time(0);
  tm *ltm = localtime(&now);
  bool is_even_seconds = (ltm->tm_sec % 3 == 0);

  if (buttons[button_dead_man_].isPressed())
  {
    // If we are pressing deadman

    if (buttons[button_speed_down_].isReleased())
    {
      current_velocity_level_ = std::max(min_velocity_level_, current_velocity_level_ - velocity_level_step_);
      ROS_INFO("PadPluginRising::execute: speed down -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }
    else if (buttons[button_speed_up_].isReleased())
    {
      current_velocity_level_ = std::min(max_velocity_level_, current_velocity_level_ + velocity_level_step_);
      ROS_INFO("PadPluginRising::execute: speed up -> velocity level = %.1f%%", current_velocity_level_ * 100.0);
    }

    if (buttons[button_power_on_arm_].isReleased())
    {
      ROS_WARN("Power on arm!!");

      std_msgs::Bool arm_msg;
      arm_msg.data = true; 
      toggle_arm_pub_.publish(arm_msg);
    }

    if (buttons[button_power_off_arm_].isReleased())
    {
      ROS_WARN("Power off arm!!");

      std_msgs::Bool arm_msg;
      arm_msg.data = false; 
      toggle_arm_pub_.publish(arm_msg);
    }

    if (buttons[button_kinematic_mode_].isPressed()){

      // If we are in flipper control mode

      if(axes[axis_backflipper_] != 0.0){
        flipper_b.data = flipper_b_read.data - axes[axis_backflipper_]*0.5;
        //back_flipper_wheel_pub_.publish(flipper_b);
      }
      if(axes[axis_frontflipper_] != 0.0){
        flipper_f.data = flipper_f_read.data + axes[axis_frontflipper_]*0.5;
        //front_flipper_wheel_pub_.publish(flipper_f);
      } 
      
    }else{

      // If we are in traction control mode 

    // ----------------------------------------------
    // ----------------------------------------------
    //        SE HA COMENTADO ESTO LO SIGUIENTE
    // ----------------------------------------------
    // ----------------------------------------------
    // cmd_twist_.linear.x = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_] * std::cos(-axes[axis_angular_z_] * (M_PI / 2.0));
    // cmd_twist_.angular.z = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_] *
    //                        std::sin(-axes[axis_angular_z_] * (M_PI / 2.0)) / 0.12;

    // ----------------------------------------------
    // ----------------------------------------------
    //           Y SE HA CAMBIADO POR ESTO
    // ----------------------------------------------
    // ----------------------------------------------
      if (abs(axes[axis_linear_x_]) > 0.05){
      cmd_twist_.linear.x = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_];
      }
      else{
        cmd_twist_.linear.x = 0.0;
      }

      if (abs(axes[axis_angular_z_]) > 0.05){
      cmd_twist_.angular.z = current_velocity_level_ * max_angular_speed_ * (axes[axis_angular_z_]*2);
      }
      else{
        cmd_twist_.angular.z = 0.0;
      }

    // ================================================================= //
    // ================ WATER SYSTEM WITH NORMAL PAD =================== //
    // ================================================================= //

    if (enable_web_buttons_ == false){

      // Tube extension system 
       ROS_WARN_THROTTLE(3, "normal pad");

        if (axes[axis_vertical_arrow_] > 0){
          
          ROS_WARN("arrow up!!!");
          if(!is_even_seconds)
          {
            digitalWrite(tube_enable_pin_, true);
            digitalWrite(tube_rollup_pin_, true);
          }else
          {
            digitalWrite(tube_enable_pin_, false);
            digitalWrite(tube_rollup_pin_, false);
          }
          digitalWrite(back_tube_unroll_pin_, true);
          digitalWrite(back_tube_rollup_pin_, true);
          tube_extensor_working = true;

        }

        else if (axes[axis_vertical_arrow_] < 0){
          
          ROS_WARN("arrow down!!!");
          digitalWrite(tube_enable_pin_, true);
          digitalWrite(tube_unroll_pin_, true);
          digitalWrite(back_tube_unroll_pin_, true);
          tube_extensor_working = true;

        }

        else{

          if (tube_extensor_working == true){
            digitalWrite(tube_enable_pin_, false);
            digitalWrite(tube_rollup_pin_, false);
            digitalWrite(back_tube_unroll_pin_, false);
            digitalWrite(back_tube_rollup_pin_, false);
            digitalWrite(tube_unroll_pin_, false);
	    digitalWrite(back_tube_unroll_pin_, false);
            tube_extensor_working = false;
          }

        }

      // Water record

        if (axes[axis_horizontal_arrow_] > 0){
          
          ROS_WARN("Start record!!");

          std_msgs::Bool record_msg;
          record_msg.data = true; 
          water_record_pub_.publish(record_msg);

        }

        if (axes[axis_horizontal_arrow_] < 0){
          
          ROS_WARN("Stop record!!");

          std_msgs::Bool record_msg;
          record_msg.data = false; 
          water_record_pub_.publish(record_msg);

        }

      }

    // ================================================================= //
    // ================ WATER SYSTEM WITH WEB PAD =================== //
    // ================================================================= //


    if (enable_web_buttons_ == true){

      // Tube extension system 
      ROS_WARN_THROTTLE(3, "web pad");

        if (buttons[button_tube_rollup_].isPressed()){
          
          ROS_WARN("arrow up!!!");

          if(!is_even_seconds)
          {
            digitalWrite(tube_enable_pin_, true);
            digitalWrite(tube_rollup_pin_, true);
          }else
          {
            digitalWrite(tube_enable_pin_, false);
            digitalWrite(tube_rollup_pin_, false);
          }
          digitalWrite(back_tube_unroll_pin_, true);
          digitalWrite(back_tube_rollup_pin_, true);
          tube_extensor_working = true;

        }

        else if (buttons[button_tube_unroll_].isPressed()){
          
          ROS_WARN("arrow down!!!");
          digitalWrite(tube_enable_pin_, true);
          digitalWrite(tube_unroll_pin_, true);
	  digitalWrite(back_tube_unroll_pin_, true);
          tube_extensor_working = true;

        }

        else{

          if (tube_extensor_working == true){
            digitalWrite(tube_enable_pin_, false);
            digitalWrite(tube_rollup_pin_, false);
            digitalWrite(tube_unroll_pin_, false);
            digitalWrite(back_tube_unroll_pin_, false);
            digitalWrite(back_tube_rollup_pin_, false);
            digitalWrite(back_tube_unroll_pin_, false);
            tube_extensor_working = false;
          }

        }

      // Water record

        if (buttons[button_start_record_water_data_].isPressed()){
          
          ROS_WARN("Start record!!");

          std_msgs::Bool record_msg;
          record_msg.data = true; 
          water_record_pub_.publish(record_msg);

        }

        if (buttons[button_stop_record_water_data_].isPressed()){
          
          ROS_WARN("Stop record!!");

          std_msgs::Bool record_msg;
          record_msg.data = false; 
          water_record_pub_.publish(record_msg);

        }

      }




    }


    // Allow rising to traction while moving flippers
    twist_pub_.publish(cmd_twist_);
    back_flipper_wheel_pub_.publish(flipper_b);
    front_flipper_wheel_pub_.publish(flipper_f);

  }
  else if (buttons[button_dead_man_].isReleased() )
  {
    // No deadman
    cmd_twist_.linear.x = 0.0;
    cmd_twist_.angular.z = 0.0;

    // AÑADIR PUBLICAR EN LA POSICIÓN ACTUAL?

    back_flipper_wheel_pub_.publish(flipper_b_read);
    front_flipper_wheel_pub_.publish(flipper_f_read);
    
    twist_pub_.publish(cmd_twist_);

    if (tube_extensor_working == true){
      digitalWrite(tube_enable_pin_, false);
      digitalWrite(tube_rollup_pin_, false);
      digitalWrite(tube_unroll_pin_, false);
      digitalWrite(back_tube_unroll_pin_, false);
      digitalWrite(back_tube_unroll_pin_, false);
      digitalWrite(back_tube_rollup_pin_, false);
      tube_extensor_working = false;
    }
    
  }

  movement_status_msg_.velocity_level = current_velocity_level_ * 100;
  pad_status_pub_.publish(movement_status_msg_);

}
void PadPluginRising::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{    
    // Make sure joint states topic we are reading is from robot wheel joints before storing it
    std::string joint = "robot_back_flipper_joint";
    if (msg->name[0] == joint){
      flipper_b_read.data = -msg->position[0];
      flipper_f_read.data =  msg->position[1];
    }
}


void PadPluginRising::digitalWrite(int pin, bool state){
  
      robotnik_msgs::set_digital_output srv;

      srv.request.output = pin;
      srv.request.value = state;
      tube_extensor_srv_.call(srv);

}

}  // namespace pad_plugins
