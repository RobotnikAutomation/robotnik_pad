#include <robotnik_pad/robotnik_pad.h>

RobotnikPad::RobotnikPad(ros::NodeHandle h) : rcomponent::RComponent(h)
{
  component_name.assign(pnh_.getNamespace());
  rosReadParams();
}

RobotnikPad::~RobotnikPad()
{
}

int RobotnikPad::rosSetup()
{
  RComponent::rosSetup();

  // Publishers
  if (kinematic_mode_ == Ackermann)
  {
    ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>(cmd_ackermann_topic_vel_, 10);
  }
  else
  {
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_twist_topic_vel_, 10);
  }

  pad_status_pub_ = pnh_.advertise<robotnik_msgs::PadStatus>("pad_status", 10);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RobotnikPad::joyCb, this);
  addTopicsHealth(&joy_sub_, "joy", 5);
}

int RobotnikPad::rosShutdown()
{
  RComponent::rosShutdown();
}

void RobotnikPad::rosReadParams()
{
  bool required = true;
  bool not_required = false;

  // JOYSTICK PAD TYPE
  readParam(pnh_, "num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS, required);
  readParam(pnh_, "num_of_axes", num_of_axes_, DEFAULT_NUM_OF_AXES, required);
  readParam(pnh_, "pad_type", pad_type_, "ps4", required);

  readParam(pnh_, "axis_linear_x", axis_linear_x_, DEFAULT_AXIS_LINEAR_X, required);
  readParam(pnh_, "axis_linear_y", axis_linear_y_, DEFAULT_AXIS_LINEAR_Y, required);
  readParam(pnh_, "axis_angular_z", axis_angular_z_, DEFAULT_AXIS_ANGULAR, required);
  readParam(pnh_, "scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR, required);
  readParam(pnh_, "scale_linear", l_scale_, DEFAULT_SCALE_LINEAR, required);
  readParam(pnh_, "cmd_twist_topic_vel", cmd_twist_topic_vel_, cmd_twist_topic_vel_, required);
  readParam(pnh_, "cmd_ackermann_topic_vel", cmd_ackermann_topic_vel_, cmd_ackermann_topic_vel_, required);
  readParam(pnh_, "button_dead_man", dead_man_button_, dead_man_button_, required);
  readParam(pnh_, "button_speed_up", speed_up_button_, speed_up_button_, required);
  readParam(pnh_, "button_speed_down", speed_down_button_, speed_down_button_, required);

  // KINEMATIC MODE
  readParam(pnh_, "kinematic_mode", kinematic_mode_, kinematic_mode_, required);
  readParam(pnh_, "button_kinematic_mode", button_kinematic_mode_, button_kinematic_mode_, required);
}

void RobotnikPad::rosPublish()
{
  RComponent::rosPublish();

  if (getState() == robotnik_msgs::State::READY_STATE)
  {
    // Publish velocity only when deadman button is pressed
    if (buttons_[dead_man_button_].isPressed())
    {
      if (kinematic_mode_ == Ackermann)
      {
        ackermann_pub_.publish(cmd_ackermann_sent_);
      }
      else
      {
        twist_pub_.publish(cmd_twist_sent_);
      }
    }
  }
}

void RobotnikPad::initState()
{
  // Initialize the buttons and axes
  for (int i = 0; i < num_of_buttons_; i++)
  {
    Button b;
    buttons_.push_back(b);
  }

  for (int i = 0; i < num_of_axes_; i++)
  {
    axes_.push_back(0.0);
  }

  // By default the velocity it's 10% of the maximum one
  current_vel_ = 0.1;
  cmd_twist_sent_ = geometry_msgs::Twist();
  cmd_ackermann_sent_ = ackermann_msgs::AckermannDrive();

  switchToState(robotnik_msgs::State::STANDBY_STATE);
}

void RobotnikPad::standbyState()
{
  if (checkTopicsHealth() == false)
  {
    switchToState(robotnik_msgs::State::EMERGENCY_STATE);
  }
  else
  {
    switchToState(robotnik_msgs::State::READY_STATE);
  }
}

void RobotnikPad::readyState()
{
}

void RobotnikPad::emergencyState()
{
  if (checkTopicsHealth() == true)
  {
    switchToState(robotnik_msgs::State::STANDBY_STATE);
  }
}

void RobotnikPad::failureState()
{
}

void RobotnikPad::joyCb(const sensor_msgs::Joy::ConstPtr& joy)
{
  // Fill in the axes and buttons arrays
  for (int i = 0; i < joy->axes.size(); i++)
  {
    axes_[i] = joy->axes[i];
  }
  for (int i = 0; i < joy->buttons.size(); i++)
  {
    buttons_[i].press(joy->buttons[i]);
  }
  tickTopicsHealth("joy");
}

void RobotnikPad::fillTwistMsg(double linear_x, double linear_y, double angular_z)
{
  cmd_twist_sent_.linear.x = linear_x;
  if (kinematic_mode_ == Omnidirectional)
    cmd_twist_sent_.linear.y = linear_y;

  cmd_twist_sent_.angular.z = angular_z;
}

void RobotnikPad::fillAckermannMsg(double linear_x, double angle)
{
  cmd_ackermann_sent_.speed = linear_x;
  cmd_ackermann_sent_.steering_angle = angle;
}