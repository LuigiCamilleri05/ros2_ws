cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_metrics_collector test/test_metrics_collector.py)
  ament_add_pytest_test(test_navigation_fsm test/test_navigation_fsm.py)
endif()

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

install(DIRECTORY launch config worlds maps urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
