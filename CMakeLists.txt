cmake_minimum_required(VERSION 3.8)
project(node_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
# find_package(ament_cmake_python REQUIRED)
# find_package(rclpy REQUIRED)


function(build_executable exec_name)
  add_executable(${exec_name} src/${exec_name}.cpp)
  target_include_directories(${exec_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${exec_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(
    ${exec_name}
    "rclcpp"
    "std_msgs"
    "example_interfaces"
  )
  install(TARGETS ${exec_name}
    DESTINATION lib/${PROJECT_NAME})
endfunction()


ament_python_install_package(${PROJECT_NAME})
  
  
# build_executable(events_listener)
# build_executable(graph_listener)
install(PROGRAMS
  scripts/graph_main.py
  DESTINATION lib/${PROJECT_NAME}
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()
ament_package()

