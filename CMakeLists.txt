cmake_minimum_required(VERSION 3.8)
project(gz_control_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros_ign_bridge REQUIRED)

# Fortress
if("$ENV{GZ_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")
# Edifice (default)
else()
  find_package(ignition-transport10 REQUIRED)
  set(GZ_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Edifice")
endif()

add_library(${PROJECT_NAME} SHARED
  src/gz_hw.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  ros_ign_bridge)

target_link_libraries(${PROJECT_NAME}
  ignition-transport${GZ_TRANSPORT_VER}::core)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface gz_control_hw.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
