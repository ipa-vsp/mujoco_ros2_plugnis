cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_limits REQUIRED)
find_package(urdf REQUIRED)
find_package(glfw3 REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  hardware_interface
  pluginlib
  controller_manager
  joint_limits
  urdf
  glfw3
  control_toolbox
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(mujoco QUIET)
if(mujoco_FOUND)
  message(STATUS "Mujoco build from source has been found")
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
elseif(DEFINED ENV{MUJOCO_DIR})
  message(STATUS "Mujoco build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_DIR} instead.")
  find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Failed to find binary in $ENV{MUJOCO_DIR}")
  endif()
  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
else()
  message(FATAL_ERROR "Failed to find mujoco with find_package.
  Either build and install mujoco from source or set the MUJOCO_DIR environment variable to tell CMake where to find the binary install. ")
endif()


## COMPILE
add_library(mujoco_system_plugins SHARED src/mujoco_system.cpp)
target_compile_features(mujoco_system_plugins PUBLIC cxx_std_17)
target_include_directories(mujoco_system_plugins PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/>
  ${MUJOCO_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(
  mujoco_system_plugins PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS mujoco_system_plugins
  EXPORT export_mujoco_system_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export hardware plugins
pluginlib_export_plugin_description_file(mujoco_ros2_control mujoco_system_plugins.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/
)

add_executable(mujoco_ros2_control
  src/mujoco_ros2_control_node.cpp
  src/mujoco_ros2_control.cpp
)
ament_target_dependencies(mujoco_ros2_control ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(mujoco_ros2_control ${MUJOCO_LIB} glfw)
target_include_directories(mujoco_ros2_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MUJOCO_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  mujoco_ros2_control
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


## EXPORTS
ament_export_targets(export_mujoco_system_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
