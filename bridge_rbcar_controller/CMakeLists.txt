cmake_minimum_required(VERSION 3.8)
project(bridge_rbcar_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_subdirectory(thirdparty/json-3.11.3)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(bridge_rbcar_controller src/bridge_rbcar_controller.cpp)
ament_target_dependencies(bridge_rbcar_controller rclcpp ackermann_msgs sensor_msgs)
target_include_directories(bridge_rbcar_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/json-3.11.3/single_include
)
target_compile_features(bridge_rbcar_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS bridge_rbcar_controller
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
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
