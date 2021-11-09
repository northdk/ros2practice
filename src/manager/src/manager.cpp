#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/tank_active.hpp"

void sub_tank_running(const protocol::msg::TankActive::SharedPtr msg) {
  printf("manager get msg: %d\n", msg->activity);
}

int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;
  printf("hello world manager package\n");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("manager");

  auto sub = node->create_subscription<protocol::msg::TankActive>("tank_running", rclcpp::SystemDefaultsQoS(), sub_tank_running);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
