#include "robotnik_pad/robotnik_pad.h"

#include <cstdio>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotnikPad>();
  node->start();
  rclcpp::spin(node);
  return 0;
}