cmake_minimum_required(VERSION 3.8)
project(manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action)
find_package(protocol REQUIRED)
find_package(utils REQUIRED)

set(excutable_name manager)
option(ACTION "Build with Action Code" ON)


if(utils_FOUND)
  message(STATUS "utils was found")
endif()

add_executable(manager src/manager.cpp)

# if(ACTION)
#   target_compile_definitions(${executable_name} PRIVATE "action")
# endif()

# add_executable(dockertest src/dockertest.cpp)

ament_target_dependencies(manager
  rclcpp
  rclcpp_action
  protocol
  utils)

# target_include_directories(manager PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
target_compile_features(manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS manager
  DESTINATION lib/${PROJECT_NAME})


# install(TARGETS dockertest
#   DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

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

# ament_export_dependencies(utils)
ament_package()
