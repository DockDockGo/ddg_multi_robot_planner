cmake_minimum_required(VERSION 3.8)
project(ddg_multi_robot_planner)

set(lib_name ${PROJECT_NAME}_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

include_directories(
  include
)

set(dependencies
  rclcpp
  nav_msgs
  nav2_msgs
  nav2_costmap_2d
  )

add_executable(${lib_name}  src/multi_robot_planner.cpp)

# add_library(${lib_name} SHARED
#         src/multi_robot_planner.cpp)

ament_target_dependencies(${lib_name}
  ${dependencies}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${lib_name}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${lib_name})
ament_export_dependencies(${dependencies})

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