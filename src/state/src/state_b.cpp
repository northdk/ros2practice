/**
 * @file state_b.cpp
 * @author north24@qq.com
 * @brief 
 * @version 0.1
 * @date 2022-01-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <chrono>
#include "state/base.hpp"
using namespace std;
using namespace rclcpp;

class StateB : public rclcpp::Node,
    public StateBase
{
public:
    StateB(const string& name) : StateBase(name), Node(name){

    }
    ~StateB() {
        cout << "state b desdroyed\n";
    }

    void Init() {
        RegisterStateCheck(this);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto state_b = make_shared<StateB>(string("StateBBB"));
    state_b->Init();

    rclcpp::spin(state_b);
    return 0;    
}