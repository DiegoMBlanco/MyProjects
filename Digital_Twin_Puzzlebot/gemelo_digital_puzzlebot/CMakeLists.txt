cmake_minimum_required(VERSION 3.8)
project(gemelo_digital_puzzlebot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies "rclcpp" "geometry_msgs" "nav_msgs" "tf2" "tf2_ros" "std_msgs")

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster ${dependencies})

add_executable(ground_truth_broadcaster src/ground_truth_broadcaster.cpp)
ament_target_dependencies(ground_truth_broadcaster ${dependencies})

install(TARGETS
  tf_broadcaster
  ground_truth_broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  urdf 
  meshes 
  rviz 
  worlds 
  src 
  scripts
  DESTINATION share/${PROJECT_NAME}/
)


# Install Python executables #They must start with the shebag tag
install(PROGRAMS
  scripts/line_detector.py
  scripts/line_follower.py
  scripts/obstacle_detector.py
  scripts/intersection.py
  DESTINATION lib/${PROJECT_NAME} )

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
