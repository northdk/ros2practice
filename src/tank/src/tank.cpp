#include <cstdio>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/tank_active.hpp"
#include "protocol/srv/probe.hpp"

class Tank : public rclcpp::Node
{
using tankmsg = protocol::msg::TankActive;
using tanksrv = protocol::srv::Probe;
public:
  Tank() : Node("try") {
    m_pPublisher = this->create_publisher<protocol::msg::TankActive>("tank_running", 10);
    m_pService = this->create_service<protocol::srv::Probe>("tank_calling", std::bind(&Tank::ServiceFunc, this, std::placeholders::_1, std::placeholders::_2));
  }

  void RunOnce() {
    while(true) {
      auto msg = protocol::msg::TankActive();
      msg.activity = this->distance++;
      printf("tank publish once: %d\n", msg.activity);
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
    response->distance = 1;
    response->landform = 1;
  }

};

void TestTankPublish() {
  std::shared_ptr<Tank> tank = std::make_shared<Tank>();

  while(true) {
    tank->RunOnce();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;

  printf("hello world tank package\n");

  rclcpp::init(argc, argv);
  std::shared_ptr<Tank> tank = std::make_shared<Tank>();

  std::thread t = std::thread(TestTankPublish);
  t.detach();
  rclcpp::spin(tank);
  rclcpp::shutdown();

  return 0;
}

