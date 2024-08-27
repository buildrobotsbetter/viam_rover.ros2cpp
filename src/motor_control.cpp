#include <viam_rover/MotorControlNode.hpp>

#include <cstdio>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControlNode>());
  rclcpp::shutdown();

  return 0;
}
