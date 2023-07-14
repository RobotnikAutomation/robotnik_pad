#include <robotnik_pad_plugins/rising_plugin.h>
#include <std_msgs/Float64.h>

namespace pad_plugins
{
PadPluginRising::PadPluginRising()
{
}

PadPluginRising::~PadPluginRising()
{
}

void PadPluginRising::initialize(const ros::NodeHandle& nh, const std::string& plugin_ns)
{
  bool required = true;
  pnh_ = ros::NodeHandle(nh, plugin_ns);
  nh_ = ros::NodeHandle();
  button_dead_man_ = 5;
  flipper_b_read.data = 0.0; 
  flipper_f_read.data = 0.0;
  readParam(pnh_, "config/button_deadman", button_dead_man_, button_dead_man_, required);
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

  // Publishers
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 10);
  	// RISING COMMANDS
	right_wheel_pub_ = nh_.advertise<std_msgs::Float64>("front_right_master_wheel_joint_controller/command", 1);
	left_wheel_pub_ = nh_.advertise<std_msgs::Float64>("front_left_master_wheel_joint_controller/command", 1);
  front_flipper_wheel_pub_ = nh_.advertise<std_msgs::Float64>("front_flipper_joint_controller/command", 1);
	back_flipper_wheel_pub_ = nh_.advertise<std_msgs::Float64>("back_flipper_joint_controller/command", 1);
  test_pub_ = nh_.advertise<sensor_msgs::JointState>("/test", 1);
  // SUBSCRIBER JOINT STATES
  joint_states_sub_ = nh_.subscribe("joint_states", 1, &PadPluginRising::jointStatesCallback, this);


  pad_status_pub_ = pnh_.advertise<robotnik_pad_msgs::MovementStatus>("status", 10);

  // initialize variables
  current_velocity_level_ = 0.1;
  velocity_level_step_ = 0.1;
  max_velocity_level_ = 1.0;
  min_velocity_level_ = 0.1;
  cmd_twist_ = geometry_msgs::Twist();
  movement_status_msg_ = robotnik_pad_msgs::MovementStatus();
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

  //flipper_f.data = msg_test.position[1]; // + float(axes[1])*float(0.500000000000000000000);
  //ifront_flipper_wheel_pub_.publish(flipper_f_read);

  if (buttons[button_dead_man_].isPressed())
  {
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

    if (buttons[7].isPressed()){
    
   
      if(axes[axis_backflipper_] != 0.0){
        flipper_b.data = flipper_b_read.data - axes[axis_backflipper_]*0.5;
        //flipper_b.data = -msg_test.position[0] - axes[5]*0.5;
        //flipper_b.data = msg_test.position[0] ;
        back_flipper_wheel_pub_.publish(flipper_b);
      }
      if(axes[axis_frontflipper_] != 0.0){
        flipper_f.data = flipper_f_read.data + axes[axis_frontflipper_]*0.5;
        //flipper_f.data = msg_test.position[1] + axes[1]*0.5;
        //flipper_f.data = msg_test.position[1];
        front_flipper_wheel_pub_.publish(flipper_f);
      } //|| axes[1] !=0.0 ){
      //ROS_INFO("Variable f_read: %f", flipper_f_read.data);
      //flipper_b.data = flipper_b_read.data - axes[5]*0.5;
      //flipper_f.data = flipper_f_read.data + axes[1]*0.5;
      //ROS_INFO("Variable b: %f", flipper_b.data);
      //ROS_INFO("MOVING FLIPPERS");
      //back_flipper_wheel_pub_.publish(flipper_b);
      //front_flipper_wheel_pub_.publish(flipper_f);
      


    }else{

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
    cmd_twist_.linear.x = current_velocity_level_ * max_linear_speed_ * axes[axis_linear_x_];
    cmd_twist_.angular.z = current_velocity_level_ * max_angular_speed_ * (axes[axis_angular_z_]*2);

   //traction_l.data = axes[axis_backflipper_]*30 + axes[axis_angular_z_]*20;
   //traction_r.data = axes[axis_backflipper_]*-30 + axes[axis_angular_z_]*20;
     traction_l.data = axes[5]*30 + axes[2]*20;
     traction_r.data = axes[5]*-30 + axes[2]*20;
    //ROS_INFO("axis_backflipper_: %f", axis_backflipper_);
    //ROS_INFO("dentro");
    right_wheel_pub_.publish(traction_r);
    left_wheel_pub_.publish(traction_l);
  
    }

      // back_flipper_wheel_pub_.publish(flipper_b);
     // front_flipper_wheel_pub_.publish(flipper_f);
      twist_pub_.publish(cmd_twist_);
  }
  else if (buttons[button_dead_man_].isReleased() )
  {
    //ROS_INFO("fuera");
    cmd_twist_.linear.x = 0.0;
    cmd_twist_.angular.z = 0.0;
    traction_l.data = 0.0;
    traction_r.data = 0.0;

    //back_flipper_wheel_pub_.publish(flipper_b);
    //front_flipper_wheel_pub_.publish(flipper_f);
    
    twist_pub_.publish(cmd_twist_);
    right_wheel_pub_.publish(traction_r);
    left_wheel_pub_.publish(traction_l);
    
    
  }

  //back_flipper_wheel_pub_.publish(flipper_b);
   // front_flipper_wheel_pub_.publish(flipper_f);
  movement_status_msg_.velocity_level = current_velocity_level_ * 100;
  pad_status_pub_.publish(movement_status_msg_);
}
void PadPluginRising::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {    
    std::string joint = "robot_back_flipper_joint";
    if (msg->name[0] == joint){
    flipper_b_read.data = -msg->position[0];
    //ROS_INFO("Variable b_read en subs: %f", flipper_b_read.data);
    //ROS_INFO("Variable f_read en subs: %f", flipper_f_read.data);
    flipper_f_read.data = msg->position[1];
    //msg_test = *msg;
    //test_pub_ .publish(msg_test);
    }

    //sleep(200);
 }
}  // namespace pad_plugins

