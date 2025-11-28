cmake_minimum_required(VERSION 3.8)
project(cartelo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(control_msgs REQUIRED)

generate_parameter_library(
  pose_teleoperation_parameters
  "config/pose_teleoperation.yaml"
)

generate_parameter_library(
  gripper_teleoperation_parameters
  "config/gripper_teleoperation.yaml"
)

add_library(pose_teleoperation_component SHARED
  src/pose_teleoperation.cpp
  src/joystick_handler.cpp
)
target_include_directories(pose_teleoperation_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(pose_teleoperation_component PUBLIC c_std_99 cxx_std_17)
target_link_libraries(pose_teleoperation_component
  pose_teleoperation_parameters
)

add_library(gripper_teleoperation_component SHARED
  src/gripper_teleoperation.cpp
  src/joystick_handler.cpp
)
target_include_directories(gripper_teleoperation_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(gripper_teleoperation_component PUBLIC c_std_99 cxx_std_17)
target_link_libraries(gripper_teleoperation_component
  gripper_teleoperation_parameters
)

ament_target_dependencies(pose_teleoperation_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  controller_manager_msgs
  control_msgs
)

rclcpp_components_register_node(pose_teleoperation_component PLUGIN "cartelo::PoseTeleoperation" EXECUTABLE pose_teleoperation_node)

ament_target_dependencies(gripper_teleoperation_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  control_msgs
  rclcpp_action
)
rclcpp_components_register_node(gripper_teleoperation_component PLUGIN "cartelo::GripperTeleoperation" EXECUTABLE gripper_teleoperation_node)

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

install(TARGETS pose_teleoperation_component gripper_teleoperation_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
