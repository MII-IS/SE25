cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_example_7 LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager 4.0.0 REQUIRED)

set_compiler_options()
export_windows_symbols()

# 1. TU NODO BAILARÍN
add_executable(move_robot_node src/robot_bailarin.cpp)
ament_target_dependencies(move_robot_node rclcpp sensor_msgs)
install(TARGETS move_robot_node DESTINATION lib/${PROJECT_NAME})

# 2. NUEVA LIBRERÍA DE LÓGICA (MOCK)
add_library(path_logic src/path_manager.cpp)
target_include_directories(path_logic PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

# 3. Dependencias originales
set(DEPS pluginlib hardware_interface kdl_parser trajectory_msgs controller_interface realtime_tools)
foreach(Dependency IN ITEMS ${DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

# 4. TESTING
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Añadimos el test
  ament_add_gtest(test_req_f_006 test/test_path_manager.cpp)
  target_link_libraries(test_req_f_006 path_logic)
endif()

ament_package()
