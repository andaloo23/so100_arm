cmake_minimum_required(VERSION 3.8)
project(so100_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# SCServo Library
add_library(scservo_lib SHARED
  include/SCServo_Linux/SCS.cpp
  include/SCServo_Linux/SCSCL.cpp
  include/SCServo_Linux/SCSerial.cpp
  include/SCServo_Linux/SMS_STS.cpp
  include/SCServo_Linux/SMSBL.cpp
  include/SCServo_Linux/SMSCL.cpp
)

target_include_directories(scservo_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Hardware Interface
add_library(so100_arm_interface SHARED
  src/so100_arm_interface.cpp
)

target_include_directories(so100_arm_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(so100_arm_interface scservo_lib yaml-cpp)

ament_target_dependencies(so100_arm_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
)

# Install
install(TARGETS so100_arm_interface scservo_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/rviz_moveit_visualizer.py
  scripts/test_joint_states.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Plugin export
pluginlib_export_plugin_description_file(hardware_interface so100_arm.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
