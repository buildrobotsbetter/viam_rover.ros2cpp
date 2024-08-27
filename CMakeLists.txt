cmake_minimum_required(VERSION 3.8)

project(viam_rover)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# build non-ROS third party libraries
add_subdirectory(third_party)

include(${CMAKE_CURRENT_LIST_DIR}/sources.cmake)

add_executable(motor_control ${SOURCES})
target_include_directories(motor_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(motor_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  motor_control
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
)

# depend on non-ROS third party libraries
target_link_libraries(motor_control
  libgpio
)

install(TARGETS motor_control
  DESTINATION lib/${PROJECT_NAME})

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
