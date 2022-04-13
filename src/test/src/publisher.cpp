#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include "rclcpp/node.hpp"
#include "protocol/msg/test.hpp"

int main(int argc, char ** argv)
{
  std::cout << "hello, publisher!" << std::endl;
  rclcpp::init(argc, argv);
  auto node_ptr = rclcpp::Node::make_shared("test_publisher");
  auto pub_ptr = node_ptr->create_publisher<protocol::msg::Test>("test_topic", 10);
  
  protocol::msg::Test test_msg;
  while (rclcpp::ok())
  {
    test_msg.a++;
    // test_msg.b = 1;
    test_msg.c = std::string("hello, topic!");
    pub_ptr->publish(test_msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  rclcpp::shutdown();
}