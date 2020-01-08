
#ifndef _ROBOTNIK_PAD_
#define _ROBOTNIK_PAD_

#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>

#include <rcomponent/rcomponent.h>
#include <robotnik_pad/button.h>

#define DEFAULT_NUM_OF_BUTTONS 16
#define DEFAULT_NUM_OF_AXES 8
#define DEFAULT_AXIS_LINEAR_X 1
#define DEFAULT_AXIS_LINEAR_Y 0
#define DEFAULT_AXIS_ANGULAR 2
#define DEFAULT_SCALE_LINEAR 1.5
#define DEFAULT_SCALE_ANGULAR 1.5
class RobotnikPad : public rcomponent::RComponent
{
public:
  RobotnikPad(ros::NodeHandle h);
  virtual ~RobotnikPad();

  enum KinematicMode
  {
    Differential = 0,
    Omnidirectional = 1,
    Ackermann = 2
  };

protected:
  /* RComponent stuff */

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  //! Shutdowns all the ROS' stuff
  virtual int rosShutdown();
  //! Reads data a publish several info into different topics
  virtual void rosPublish();
  //! Reads params from params server
  virtual void rosReadParams();
  //! Actions performed on init state
  virtual void initState();
  //! Actions performed on standby state
  virtual void standbyState();
  //! Actions performed on ready state
  virtual void readyState();
  //! Actions performed on the emergency state
  virtual void emergencyState();
  //! Actions performed on Failure state
  virtual void failureState();

  //! joy callback
  void joyCb(const sensor_msgs::Joy::ConstPtr& joy);
  void fillTwistMsg(double linear_x, double linear_y, double angular_z);
  void fillAckermannMsg(double linear_x, double angle);

public:
protected:
  /* ROS stuff */

  // Publishers
  ros::Publisher twist_pub_;
  ros::Publisher ackermann_pub_;
  ros::Publisher pad_data_pub_;

  // Subscribers
  ros::Subscriber joy_sub_;

  // JOYSTICK
  //! Current number of buttons of the joystick
  int num_of_buttons_;
  int num_of_axes_;
  std::string pad_type_;
  int axis_linear_x_, axis_linear_y_, axis_angular_z_;
  double a_scale_, l_scale_;
  std::string cmd_twist_topic_vel_, cmd_ackermann_topic_vel_;
  int dead_man_button_, speed_up_button_, speed_down_button_;

  // KINEMATIC MODE
  int button_kinematic_mode_, kinematic_mode_;

  //! Vector to save the axis values
  std::vector<float> axes_;
  //! Vector to save and control the axis values
  std::vector<Button> buttons_;

};

#endif  // _ROBOTNIK_PAD_