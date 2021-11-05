colcon build --packages-select polygon_base polygon_plugins
source install/setup.bash
ros2 run polygon_base area_node
