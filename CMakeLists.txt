cmake_minimum_required(VERSION 3.12)
project(surface_raster_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(surface_raster_planner_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(VTK REQUIRED COMPONENTS CommonCore CommonDataModel FiltersGeneral)

# ROS-independent core: PCA frame, serpentine, ray casting, poses and jumps
add_library(raster_planner_core SHARED
  src/raster_planner.cpp
)
target_compile_features(raster_planner_core PUBLIC cxx_std_17)
target_include_directories(raster_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(raster_planner_core PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(raster_planner_core
  ${PCL_COMMON_LIBRARIES}
  Eigen3::Eigen
  VTK::CommonCore
  VTK::CommonDataModel
  VTK::FiltersGeneral
)

add_executable(raster_planner_node
  src/raster_planner_node.cpp
  src/raster_planner_main.cpp
)
target_include_directories(raster_planner_node PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(raster_planner_node PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(raster_planner_node
  raster_planner_core
  ${PCL_LIBRARIES}
  ${surface_raster_planner_msgs_TARGETS}
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2_eigen::tf2_eigen
)

install(TARGETS raster_planner_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS raster_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/make_test_mesh.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_raster_planner test/test_raster_planner.cpp)
  target_link_libraries(test_raster_planner raster_planner_core)
endif()

ament_package()
