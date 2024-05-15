#include "lab7/ur3e_mover.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<UR3eMover>("ur_manipulator"));
  rclcpp::shutdown();

  return 0;
}
