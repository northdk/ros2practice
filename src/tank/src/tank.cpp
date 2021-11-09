#include <cstdio>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/tank_active.hpp"

class Tank : public rclcpp::Node
{
public:
  Tank() : Node("try") {
    m_pPublisher = this->create_publisher<protocol::msg::TankActive>("tank_running", 10);
  }

  void RunOnce() {
    auto msg = protocol::msg::TankActive();
    msg.activity = this->distance++;
    printf("tank publish once: %d\n", msg.activity);
    m_pPublisher->publish(msg);
  }

private:
  rclcpp::Publisher<protocol::msg::TankActive>::SharedPtr m_pPublisher;
  uint8_t distance{0};
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

  while(true) {
    tank->RunOnce();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  rclcpp::spin(tank);
  rclcpp::shutdown();

  return 0;
}

