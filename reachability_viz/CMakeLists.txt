cmake_minimum_required(VERSION 3.5)
project(reachability_viz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

set(DEPS
pluginlib
rviz_common
reachability_msgs
rviz_rendering
rviz_default_plugins
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/reachability_viz/reachability_display.hpp
)

add_library(reachability_display src/reachability_display.cpp ${MOC_FILES})
target_include_directories(reachability_display PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(reachability_display
  ${DEPS}
)
install(TARGETS reachability_display
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY icons
        DESTINATION "share/${PROJECT_NAME}"
)

install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
