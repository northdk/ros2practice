#include <cstdio>
#include <chrono>
#include <unistd.h> 
#include <sys/syscall.h> 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "protocol/msg/tank_active.hpp"
#include "protocol/srv/probe.hpp"
#include "protocol/action/contact.hpp"
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
    if(msg->activity %5 == 0) {
      printf("manager subscriber will make a segment fault\n!");
      fflush(stdout);
      make_segment_fault();
    }
  fflush(stdout);
}

class Manager : public rclcpp::Node
{
public:
  using Contact = protocol::action::Contact;
  using GoalHandleContact = rclcpp_action::ServerGoalHandle<Contact>;

  explicit Manager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("manager", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Contact>(
      this,
      "contact",
      std::bind(&Manager::handle_goal, this, _1, _2),
      std::bind(&Manager::handle_cancel, this, _1),
      std::bind(&Manager::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Contact>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Contact::Goal> goal)
  {
    printf("Received goal request with order %s", goal->cmd.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleContact> goal_handle)
  {
    printf("manager Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleContact> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Manager::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleContact> goal_handle)
  {
    printf("manager executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Contact::Feedback>();
    auto & status = feedback->status;
    
    auto result = std::make_shared<Contact::Result>();

    for (int i = 1; (i < 10) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->last = true;
        goal_handle->canceled(result);
        printf("Goal canceled");
        return;
      }
      // Update sequence
      status = std::string("aha-") + std::to_string(i);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      printf("manager Publish feedback: %s\n", status.c_str());
      
      loop_rate.sleep();
    }
    printf("manager for executing is ok\n");
    // Check if goal is done
    if (true) {
      printf("manager for executing is ok\n");
      result->last = true;
      printf("manager for executing is ok\n");
      goal_handle->succeed(result);
      printf(" manager Goal succeeded");
    }
    printf("manager for executing is ok\n");
    fflush(stdout);
  }

};  // class Manager

int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;
  // signal(SIGSEGV, signal_handler);
  // signal(SIGABRT, signal_handler);
  printf("hello, manager package\n");
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
    fflush(stdout);
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

  // auto manager = std::make_shared<Manager>();
  // rclcpp::spin(manager);
  // rclcpp::shutdown();

  return 0;
}
