#include <cstdio>
#include <chrono>
#include <unistd.h> 
#include <sys/syscall.h> 
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/tank_active.hpp"
#include "protocol/srv/probe.hpp"

void sub_tank_running(const protocol::msg::TankActive::SharedPtr msg) {
  printf("manager get msg: %d, tid: %ld, thread id: %ld\n", msg->activity, syscall(SYS_gettid), std::this_thread::get_id());
}

int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;
  printf("hello world manager package\n");
  rclcpp::init(argc, argv);
  auto node_ptr = rclcpp::Node::make_shared("manager");

  auto sub = node_ptr->create_subscription<protocol::msg::TankActive>("tank_running", rclcpp::SystemDefaultsQoS(), sub_tank_running);

  auto client_ptr = node_ptr->create_client<protocol::srv::Probe>("tank_calling");
  auto request_ptr = std::make_shared<protocol::srv::Probe::Request>();
  while(! client_ptr->wait_for_service(std::chrono::seconds(1))) {
    if(! rclcpp::ok()) {
      fprintf(stderr, "manager got error with rcl not ok!\n");
      return 0;
    }
    fprintf(stdout, "manager waiting for service ready...\n");
  }
  while(rclcpp::ok()) {
    fprintf(stdout, "manager client running...\n");
    auto result = client_ptr->async_send_request(request_ptr);
    if(rclcpp::spin_until_future_complete(node_ptr, result) == rclcpp::FutureReturnCode::SUCCESS) {
      fprintf(stdout, "manager get service result: distance: %d, landform: %d. tid: %ld, thread id: %ld\n",
       result.get()->distance, result.get()->landform, syscall(SYS_gettid), std::this_thread::get_id());
    } else {
      fprintf(stderr, "manager call service failed!\n");
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  } 
  
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}