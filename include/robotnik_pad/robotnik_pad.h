
#ifndef _ROBOTNIK_PAD_
#define _ROBOTNIK_PAD_

#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>

#include <rcomponent/rcomponent.h>
#include <robotnik_pad/button.h>

class RobotnikPad : public rcomponent::RComponent
{
public:
  RobotnikPad(ros::NodeHandle h);
  virtual ~RobotnikPad();

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

public:
protected:
  /* ROS stuff */

  // Publishers
  ros::Publisher twist_pub_;
  ros::Publisher ackermann_pub_;
  ros::Publisher pad_data_pub_;

  // Subscribers
  ros::Subscriber joy_sub_;
};

#endif  // _ROBOTNIK_PAD_