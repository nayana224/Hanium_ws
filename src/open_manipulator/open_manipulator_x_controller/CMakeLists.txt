cmake_minimum_required(VERSION 3.8)
project(open_manipulator_x_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(controller_node src/open_manipulator_x_controller.cpp)

ament_target_dependencies(controller_node
  rclcpp
  std_msgs
  dynamixel_sdk
  sensor_msgs
)

target_link_libraries(controller_node
  yaml-cpp
)

install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

