cmake_minimum_required(VERSION 3.20)
project(stretch_ee_vel_ctrl_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RelativeMove.msg"
)
ament_export_dependencies(rosidl_default_runtime)

# Keyboard teleop publishes RelativeMove, so it links against this package's own typesupport
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(keyboard_teleop rclcpp::rclcpp "${cpp_typesupport_target}")
install(TARGETS keyboard_teleop
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
