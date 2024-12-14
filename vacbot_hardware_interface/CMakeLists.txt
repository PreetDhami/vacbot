cmake_minimum_required(VERSION 3.8)
project(vacbot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
)


# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(
  vacbot_hardware_interface
  SHARED
  src/vacbot_hardware_interface.cpp
)


target_compile_features(vacbot_hardware_interface PUBLIC cxx_std_17)
target_include_directories(vacbot_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/vacbot_hardware_interface>
)

ament_target_dependencies(
  vacbot_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(vacbot_hardware_interface PUBLIC -lwiringPi)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface vacbot_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/vacbot_hardware_interface
)

install(TARGETS vacbot_hardware_interface
  EXPORT export_vacbot_hardware_interface
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


ament_export_targets(export_vacbot_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
