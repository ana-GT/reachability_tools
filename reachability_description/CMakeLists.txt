cmake_minimum_required(VERSION 3.16)
project(reachability_description)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

set(DEPS 
  ament_cmake
  rclcpp
  trac_ik_lib
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/reachability_description.cpp)
ament_target_dependencies( ${PROJECT_NAME} ${DEPS})

add_executable(reachability_generation_node src/reachability_generation_node.cpp)
target_link_libraries(reachability_generation_node ${PROJECT_NAME})
ament_target_dependencies(reachability_generation_node ${DEPS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} reachability_generation_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_libraries(
    ${PROJECT_NAME}
)
#ament_export_targets(
#  export_${PROJECT_NAME}
#)

ament_export_dependencies(
  ${DEPS}
  )

ament_export_include_directories(
  include
)


ament_package()
