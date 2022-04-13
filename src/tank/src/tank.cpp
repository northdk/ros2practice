/**
 * @file tank.cpp
 * @author North.D.K (north24@qq.com) (dk_snow on github.com)
 * @brief All tanks running in this module
 * @version 1.0
 * @date 2022-01-08
 * 
 * @copyright See the LICENSE file.
 * 
 */
#include <cstdio>
#include <chrono>
#include <thread>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <pluginlib/class_loader.hpp>
#include "tank/tank_base.hpp"
#include "protocol/msg/tank_active.hpp"
#include "protocol/srv/probe.hpp"
#include "protocol/action/contact.hpp"
#include "utils/backtrace.h"

using Contact = protocol::action::Contact;
using GoalHandleContact = rclcpp_action::ClientGoalHandle<Contact>;

class Tankt : public rclcpp::Node
{
using tankmsg = protocol::msg::TankActive;
using tanksrv = protocol::srv::Probe;
public:
  Tankt() : Node("try") {
    m_pPublisher = this->create_publisher<protocol::msg::TankActive>("tank_running", 10);
    m_pService = this->create_service<protocol::srv::Probe>("tank_calling",
      std::bind(&Tankt::ServiceFunc, this, std::placeholders::_1, std::placeholders::_2));
  }

  void RunOnce() {
    while(true) {
      auto msg = protocol::msg::TankActive();
      msg.activity = this->distance++;
      printf("tank publish once: %d\n", msg.activity);
      fflush(stdout);
      m_pPublisher->publish(msg);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

private:
  rclcpp::Publisher<protocol::msg::TankActive>::SharedPtr m_pPublisher;
  rclcpp::Service<protocol::srv::Probe>::SharedPtr m_pService;
  uint8_t distance{0};

  void ServiceFunc(const std::shared_ptr<tanksrv::Request> request, std::shared_ptr<tanksrv::Response> response) {
    (void) request;
    printf("tank service on call once\n");
    fflush(stdout);
    response->distance = 1;
    response->landform = 1;
  }

};

void TestTankPublish(const std::shared_ptr<Tankt> tank) {
  // std::shared_ptr<Tank> tank = std::make_shared<Tank>();

  while(true) {
    tank->RunOnce();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void ClientResponse(std::shared_future<GoalHandleContact::SharedPtr> future) {
  auto goal_handle = future.get();
    if (!goal_handle) {
      printf("tank Goal was rejected by server");
    } else {
      printf("tank Goal accepted by server, waiting for result");
    }
}

void ClientFeedback(GoalHandleContact::SharedPtr,
    const std::shared_ptr<const Contact::Feedback> feedback) 
  {
    printf("tank get feedback: %s\n", feedback->status.c_str());
    fflush(stdout);
  }

void ClientResult(const GoalHandleContact::WrappedResult & result) {
  printf("tank get result code: %d\n", result.code);
  printf("tank get result last: %d\n", result.result->last);
  // rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;
  // signal(SIGSEGV, signal_handler);
  // signal(SIGABRT, signal_handler);
  printf("hello, tank package!\n");

  // int counter = 0;
  // while(true) {
  //   printf("hello %d\n", counter++);
  //   std::cout << "hello counter: " << counter++ << std::endl << std::flush;
  //   fprintf(stdout, "hello tank: %d\n", counter++);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //   usleep(1000000);
  // }
  // while(true);
  rclcpp::init(argc, argv);

#ifdef old
  std::shared_ptr<Tankt> tank = std::make_shared<Tankt>();
  std::thread t = std::thread(TestTankPublish, tank);
  t.detach();

  pluginlib::ClassLoader<Tank> tank_loader("tank", "Tank");
  // fprintf(stdout, "step 1 here\n");
  std::shared_ptr<Tank> tank_ptr = tank_loader.createSharedInstance("tank_a");
  // fprintf(stdout, "step 2 here\n");
  std::string a("99a");
  tank_ptr->init(a);
  tank_ptr->action();
  

  rclcpp::spin(tank);
  rclcpp::shutdown();
#endif
  std::shared_ptr<Tankt> tank = std::make_shared<Tankt>();
  auto action_client = rclcpp_action::create_client<Contact>(
    tank, "contact"
  );
  action_client->wait_for_action_server(std::chrono::seconds(1));
  auto goal_msg = Contact::Goal();
  goal_msg.cmd = std::string("hello");
  auto send_goal_options = rclcpp_action::Client<Contact>::SendGoalOptions();
  send_goal_options.goal_response_callback = ClientResponse;
  send_goal_options.feedback_callback = ClientFeedback;
  send_goal_options.result_callback = ClientResult;
  action_client->async_send_goal(goal_msg, send_goal_options);

    // rclcpp::spin(tank);

  // std::this_thread::sleep_for(std::chrono::seconds(10));
  tank->RunOnce();
  rclcpp::spin(tank);
  rclcpp::shutdown();
  return 0;
}

