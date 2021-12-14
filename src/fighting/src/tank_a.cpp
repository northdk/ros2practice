#include <tank/tank_base.hpp>
#include <utils/backtrace.h>

class tank_a : public Tank
{
private:
    int position {0};
    std::string _name;
public:
    void init(const std::string& name) override {
        signal(SIGSEGV, signal_handler);
        signal(SIGABRT, signal_handler);
        _name = name;
        printf("tank 99a init\n");
    }
    void action() override {
        printf("tank: %s action counter: %d\n", _name.c_str(), position++);
        // if(position % 10 == 0 && position != 0) {
        //     fprintf(stderr, "tank %s will throw a segmetation fault!\n", _name.c_str());
        //     char * a = nullptr;
        //     *a = 'a';
        // }
    }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tank_a, Tank)