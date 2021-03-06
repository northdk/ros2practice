/**
 * @file base.hpp
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

class StateA : public rclcpp::Node,
    public StateBase
{
public:
    StateA(const string& name) : StateBase(name), Node(name){

    }
    ~StateA() {
        cout << "state a desdroyed\n";
    }

    void Init() {
        RegisterStateCheck(this);
    }
    
    void GetNameA() {
        string name;
        GetName(name);
        cout << "state a get name: " << name << endl;
    }

    void timer_callback() {
        static int8_t state;
        this->SetState(state);
        state++;
    }

    void Run() {
        int8_t state = 0;
        cout << "here\n";
        t = this->create_wall_timer(chrono::seconds(1),
            [this, &state](){
                cout << "stats: " << state << endl;
                this->SetState(state);
                state++;
            });

        // t = this->create_wall_timer(chrono::seconds(1),
        //     std::bind(&StateA::timer_callback, this));

        cout << "here\n";
        // thread t([this, &state](){
        //     while (true)
        //     {
        //         this->SetState(state);
        //         this_thread::sleep_for(chrono::seconds(1));
        //         state++;
        //     }
        // });
        
        // t.detach();
    }
private:
    rclcpp::TimerBase::SharedPtr t;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto state_a = make_shared<StateA>(string("StateAAA"));
    state_a->Init();
    state_a->GetNameA();


    state_a->Run();
    rclcpp::spin(state_a);
    
    return 0;
}