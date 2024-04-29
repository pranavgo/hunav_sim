#include "hunav_agent_manager/extended_bt_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::BTnodeExt>());

  rclcpp::shutdown();
  return 0;
}
