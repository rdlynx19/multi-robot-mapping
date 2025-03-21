cmake_minimum_required(VERSION 3.8)
project(go1_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(ros2_unitree_legged_msgs REQUIRED)

include_directories(
  include/go1_exploration
  # ${ros2_unitree_legged_msgs_INCLUDE_DIRS}
  # ${CMAKE_SOURCE_DIR}/../unitree_ros2/unitree_legged_real/unitree_legged_sdk-master/include
)

# # install(
# #   DIRECTORY ${CMAKE_SOURCE_DIR}/../unitree_ros2/unitree_legged_real/unitree_legged_sdk-master/lib/
# #   DESTINATION lib
# # )

# link_directories(${CMAKE_SOURCE_DIR}/../unitree_ros2/unitree_legged_real/unitree_legged_sdk-master/lib)



# install(
#   DIRECTORY ${CMAKE_SOURCE_DIR}/../unitree_ros2/unitree_legged_real/unitree_legged_sdk-master/include/unitree_legged_sdk
#   DESTINATION include
# )

install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

add_executable(frontierExploration src/frontier_exploration.cpp)
target_link_libraries(frontierExploration ${EXTRA_LIBS})
ament_target_dependencies(frontierExploration rclcpp nav_msgs geometry_msgs tf2_ros visualization_msgs)


install(TARGETS 
  frontierExploration
  DESTINATION lib/${PROJECT_NAME}
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
