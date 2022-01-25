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
find_package(digital_twin_msgs REQUIRED)
# Find Matlab
find_package(Matlab REQUIRED)

add_executable(currentVoltageFlow src/data_processor/currentVoltageFlow.cpp
	                          src/data_processor/ParseDewetron.cpp
	                          src/data_processor/ParseDewetron.h)
	                          
add_executable(efficiencyMap src/efficiency/EfficiencyMap.cpp)
add_executable(windingErrorChecker src/winding_error_checker/winding_error_checker.cpp)
add_executable(torqueCalculator src/torque_calculator/TorqueCalculator.cpp)
add_executable(angularConverter src/angular_converter/AngularConverter.cpp)
add_executable(powerCalculator src/power_calculator/PowerCalculator.cpp)

ament_target_dependencies(currentVoltageFlow rclcpp std_msgs digital_twin_msgs)
ament_target_dependencies(windingErrorChecker rclcpp std_msgs digital_twin_msgs)
ament_target_dependencies(torqueCalculator rclcpp std_msgs digital_twin_msgs)
ament_target_dependencies(efficiencyMap rclcpp std_msgs)
ament_target_dependencies(angularConverter rclcpp std_msgs digital_twin_msgs)
ament_target_dependencies(powerCalculator rclcpp std_msgs digital_twin_msgs)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    currentVoltageFlow
    efficiencyMap
    windingErrorChecker
    torqueCalculator
    angularConverter
    powerCalculator
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
