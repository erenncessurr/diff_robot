cmake_minimum_required(VERSION 3.8)
project(diff_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

pkg_check_modules(LibSerial REQUIRED libserial)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)

add_library(diff_robot_hardware SHARED
  hardware/diff_robot.cpp
)

target_link_libraries(diff_robot_hardware
  PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${LibSerial_LIBRARIES} 
)

target_include_directories(diff_robot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
  ${LibSerial_INCLUDE_DIRS} 
)

install(TARGETS diff_robot_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY description
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY hardware/include/ 
    DESTINATION include
  )

install(FILES
  robot_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  hardware_interface
  robot_plugin.xml
)

ament_package()
