cmake_minimum_required(VERSION 3.8)
project(chord_regulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

include_directories(include
${EIGEN3_INCLUDE_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


set(dependencies
rclcpp
geometry_msgs
visualization_msgs
Eigen3)

# reg
add_library(chord_regulator src/ChordRegulator.cpp include/chord_regulator/ChordRegulator.h)
ament_target_dependencies(chord_regulator
    ${dependencies})

install(TARGETS chord_regulator
    EXPORT chord_regulator
    LIBRARY DESTINATION lib/chord_regulator
    ARCHIVE DESTINATION lib/chord_regulator
    RUNTIME DESTINATION lib/chord_regulator
    INCLUDES DESTINATION include/chord_regulator
    )

add_executable(controller src/controller_deb.cpp include/chord_regulator/controller_deb.h)
ament_target_dependencies(controller ${dependencies})
target_link_libraries(controller chord_regulator)

install(TARGETS controller
  EXPORT controller
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include/${PROJECT_NAME}/
  )


ament_package()
