cmake_minimum_required(VERSION 3.5)
project(loading_motor_dt)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(MATLAB_DIR "/usr/local/MATLAB/R2021a")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)
find_package(digital_twin_msgs REQUIRED)
# uncomment if you have MATLAB installed.
# find_package(Matlab REQUIRED)

add_executable(current_simulator src/current_simulator/current_simulator.cpp
	                          src/current_simulator/parse_dewetron.cpp
	                          src/current_simulator/parse_dewetron.h)
	                          
add_executable(efficiency_map src/efficiency/efficiency_map.cpp)
add_executable(winding_error_checker src/winding_error_checker/winding_error_checker.cpp)
add_executable(torque_calculator src/torque_calculator/torque_calculator.cpp)
add_executable(angular_converter src/angular_converter/angular_converter.cpp)
add_executable(power_calculator src/power_calculator/power_calculator.cpp)

ament_target_dependencies(current_simulator rclcpp std_msgs digital_twin_msgs)
ament_target_dependencies(winding_error_checker rclcpp std_msgs digital_twin_msgs)
ament_target_dependencies(torque_calculator rclcpp std_msgs digital_twin_msgs message_filters)
ament_target_dependencies(efficiency_map rclcpp std_msgs digital_twin_msgs)
ament_target_dependencies(angular_converter rclcpp std_msgs digital_twin_msgs)
ament_target_dependencies(power_calculator rclcpp std_msgs digital_twin_msgs)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    current_simulator
    efficiency_map
    winding_error_checker
    torque_calculator
    angular_converter
    power_calculator
    DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/winding_error_checker/winding_error_checker_py/scripts/winding_error_checker.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

#if(UNIX)
#    message(STATUS "Linking UNIX C++")
#    target_link_libraries(windingErrorChecker ${MATLAB_DIR}/extern/bin/glnxa64/libMatlabEngine.so)
#    target_link_libraries(windingErrorChecker ${MATLAB_DIR}/extern/bin/glnxa64/libMatlabDataArray.so)
#    find_package(Threads)
#    target_link_libraries(windingErrorChecker ${CMAKE_THREAD_LIBS_INIT})
# endif(UNIX)

ament_package()