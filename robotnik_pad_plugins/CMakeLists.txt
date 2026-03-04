cmake_minimum_required(VERSION 3.8)
project(robotnik_pad_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotnik_pad REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotnik_navigation_msgs REQUIRED)
find_package(action_msgs REQUIRED)

include_directories(${robotnik_pad_INCLUDE_DIRS})

# Find all source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create the shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Link dependencies and export plugins
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  geometry_msgs
  robotnik_pad
  rclcpp_action
  robotnik_navigation_msgs
  action_msgs
)

#target_include_directories(${PROJECT_NAME} PRIVATE
  #include
#)

target_include_directories(robotnik_pad_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${robotnik_pad_INCLUDE_DIRS}
)

pluginlib_export_plugin_description_file(robotnik_pad robotnik_pad_pluginlib.xml)

# Install targets and other files
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
