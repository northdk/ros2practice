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
#include <vector>
#include <algorithm>
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

void test_func() {
    vector<int> vec;
    vec.emplace_back(1);
    vec.emplace_back(2);
    vec.emplace_back(3);
    vec.emplace_back(4);
    vec.emplace_back(5);
    int num = 0;
    auto result = all_of(vec.cbegin(), vec.cend(),
        [](const int& n) -> bool {
            if(n % 2 != 0) {
                cout << "n: " << n << endl;
                return true;
            }
            else {
                cout << "nnn: " << n << endl;
                return false;
            }
        }
    );
    cout << result << endl;
}

int main(int argc, char** argv)
{
    // rclcpp::init(argc, argv);
    
    // auto state_b = make_shared<StateB>(string("StateBBB"));
    // state_b->Init();

    // rclcpp::spin(state_b);
    auto last = chrono::system_clock::now();
    while (true)
    {
        this_thread::sleep_for(chrono::milliseconds(500));
        auto ck = chrono::system_clock::now();
        cout << chrono::duration_cast<chrono::milliseconds>(ck.time_since_epoch()).count() << endl;
        cout << chrono::duration_cast<chrono::milliseconds>(ck.time_since_epoch() - last.time_since_epoch()).count() << endl;
        last = ck;
    }
    
    

    return 0;    
}