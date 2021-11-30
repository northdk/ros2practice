#include <cstdio>
#include <chrono>
#include <unistd.h> 
#include <sys/syscall.h> 
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/tank_active.hpp"
#include "protocol/srv/probe.hpp"
#include "utils/backtrace.h"

void make_segment_fault() {
  char* a = nullptr;
  *a = 'a';
}

/**
 * @brief 
 * 
 * @param msg 
 */
void sub_tank_running(const protocol::msg::TankActive::SharedPtr msg) {
  printf("manager get msg: %d, tid: %ld\n", msg->activity, syscall(SYS_gettid));
  if((msg->activity % 10) == 0) {
    printf("manager subscriber will make a segment fault\n!");
    fflush(stdout);
    make_segment_fault();
  }
  fflush(stdout);
}

int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;
  signal(SIGSEGV, signal_handler);
  signal(SIGABRT, signal_handler);
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
    fflush(stdout);
    std::this_thread::sleep_for(std::chrono::seconds(2));
  } 
  
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
