#include <tank/tank_base.hpp>

class tank_99a : public Tank
{
private:
    int position {0};
public:
    void init() {
        printf("tank 99a init\n");
    }
    void action() {
        printf("tank 99a action");
    }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tank_99a, Tank)