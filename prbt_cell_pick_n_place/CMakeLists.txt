cmake_minimum_required(VERSION 3.8)
project(prbt_cell_pick_n_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(rosparam_shortcuts REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# find_package(schunk_command_interface REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(depencies
  rclcpp
  rclcpp_action
  geometry_msgs
  # rosparam_shortcuts
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  # schunk_command_interface
  Boost
  yaml_cpp_vendor
)

# add_executable(pick_n_place_node src/pick_n_place_node.cpp)
# target_include_directories(pick_n_place_node PUBLIC include)
# # target_compile_features(pick_n_place_node PUBLIC c_std_99 cxx_std_17)
# ament_target_dependencies(pick_n_place_node ${depencies})

# install(TARGETS pick_n_place_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}
# )

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
