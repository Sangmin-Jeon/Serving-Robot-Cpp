cmake_minimum_required(VERSION 3.8)
project(serving_robot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}  # Qt5 Widgets 헤더 파일 경로 추가
)

add_executable(kitchen_monitor src/kitchen_monitor.cpp)
add_executable(table_order src/table_order.cpp)

ament_target_dependencies(kitchen_monitor rclcpp std_msgs std_srvs)
ament_target_dependencies(table_order rclcpp std_msgs std_srvs)

target_link_libraries(kitchen_monitor Qt5::Widgets)
target_link_libraries(table_order Qt5::Widgets)

install(TARGETS
  kitchen_monitor
  table_order
  DESTINATION lib/${PROJECT_NAME}
)

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
