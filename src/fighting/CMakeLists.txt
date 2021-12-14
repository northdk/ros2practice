cmake_minimum_required(VERSION 3.8)
project(fighting)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tank REQUIRED)
find_package(protocol REQUIRED)
find_package(pluginlib REQUIRED)
find_package(utils REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(fighting src/tank_a.cpp)

ament_target_dependencies(fighting
  rclcpp
  "pluginlib"
  tank
  utils
  protocol)

pluginlib_export_plugin_description_file(tank plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS fighting
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  fighting
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_package()
