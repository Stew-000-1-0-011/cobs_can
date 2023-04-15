cmake_minimum_required(VERSION 3.8)
project(cobs_can)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic-errors -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

set(msg_files
  "msg/Frame.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

include_directories(include
  "/home/stew/stew_programming/Library"  #自分の環境でCRSLibtmpを入れたディレクトリのパスに変更すること
)

add_library(bridge SHARED
  src/cobs_can_bridge.cpp
)
rclcpp_components_register_nodes(bridge "cobs_can_bridge::CobsCanBridge")
ament_target_dependencies(bridge
  rclcpp
  rclcpp_components
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(bridge "${cpp_typesupport_target}")

install(TARGETS
  bridge
  DESTINATION lib)

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
