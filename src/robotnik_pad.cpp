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
}

int RobotnikPad::rosShutdown()
{
  RComponent::rosShutdown();
}

void RobotnikPad::rosReadParams()
{
}

void RobotnikPad::rosPublish()
{
}

void RobotnikPad::initState()
{
}

void RobotnikPad::standbyState()
{
}

void RobotnikPad::readyState()
{
}

void RobotnikPad::emergencyState()
{
}

void RobotnikPad::failureState()
{
}

void RobotnikPad::joyCb(const sensor_msgs::Joy::ConstPtr& joy)
{
}