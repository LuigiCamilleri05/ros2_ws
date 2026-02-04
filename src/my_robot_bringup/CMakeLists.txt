cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Install Python scripts as executables
install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/navigation_fsm.py
  scripts/slam_activator.py
  scripts/metrics_collector.py
  scripts/scan_timestamp_fixer.py
  scripts/camera_pointcloud_processor.py
  scripts/dynamic_obstacles.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
