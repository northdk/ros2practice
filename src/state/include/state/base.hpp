/**
 * @file base.hpp
 * @author dk_snow( north24@qq.com )
 * @brief StateMachine base, will work in dynamic map.
 * @version 1.0
 * @date 2022-01-08
 * 
 * @copyright See the LICENSE file.
 * 
 */
#ifndef STATE__BASE_HPP_
#define STATE__BASE_HPP_
#include <iostream>
#include <string>
#include <map>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "protocol/msg/state.hpp"

class StateBase
{
// using function_t = void(StateBase::*)();
public:
    StateBase(const std::string& name) : name_(name) {
        std::cout << "state: " << name_ <<  " has been constructed\n";
        state_func_map_.insert(std::make_pair(static_cast<int8_t>(5), std::bind(&StateBase::OnConfig, this)));
        state_func_map_.insert(std::make_pair(static_cast<int8_t>(10), std::bind(&StateBase::OnShoot, this)));
        std::cout << "size: " << state_func_map_.size() << std::endl;
    }
    ~StateBase() {}

    void GetName(std::string& name) {
        name = name_;
    }

    int8_t GetState() {
        return state_;
    }

public:
    void RegisterStateCheck(rclcpp::Node* node_ptr) {
        if(node_ptr == nullptr) {
            std::cerr << "state: " << name_ << "register state check failed!\n";
        }
        pub_ = node_ptr->create_publisher<protocol::msg::State>("state_topic", 10);
        sub_ = node_ptr->create_subscription<protocol::msg::State>("state_topic", rclcpp::SystemDefaultsQoS(),
                std::bind(&StateBase::StateCheck, this, std::placeholders::_1));
    }

    void SetState(int8_t state) {
        if(pub_ == nullptr) {
            std::cerr << "state: " << name_ << "cannot set state with null pub\n";
        }
        auto msg = protocol::msg::State();
        msg.state = state;
        std::cout << name_ << " set state: " << (int)state << std::endl;
        pub_->publish(msg);
    }

    void StateCheck(const protocol::msg::State::SharedPtr msg) {
        std::cout << name_ << " received msg: " << (int)msg->state << std::endl;
        state_ = (int8_t)msg->state;
        if( state_func_map_.find(state_) != state_func_map_.end()) {
            std::cout << name_ << " has got state:" << state_ << " for transition\n";
            state_func_map_.find(state_)->second;
        }
        std::cout << name_ << " now state is: " << (int)state_ << std::endl;
    }

public:
    void OnConfig() {
        std::cout << name_ << " on config\n";
    }

    void OnShoot() {
        std::cout << name_ << " on shoot\n";
    }

private:
    std::string name_{""};
    int8_t state_{-1};
    rclcpp::Publisher<protocol::msg::State>::SharedPtr pub_{nullptr};
    rclcpp::Subscription<protocol::msg::State>::SharedPtr sub_{nullptr};

    std::map<int8_t, std::function<void()>> state_func_map_;
};

#endif  // STATE__BASE_HPP_