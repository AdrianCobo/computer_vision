cmake_minimum_required(VERSION 3.8)
project(computer_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)
find_package(image_geometry REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(dependencies
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  OpenCV
  PCL
  pcl_conversions
  image_geometry
  depth_image_proc
  Eigen3
  message_filters
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)

# OpenCV and PCL node
include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/CVSubscriber.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(cv_program src/main.cpp)
ament_target_dependencies(cv_program ${dependencies})
target_link_libraries(cv_program ${PROJECT_NAME})

add_executable(pclsync src/main_pclsync.cpp)
ament_target_dependencies(pclsync ${dependencies})
target_link_libraries(pclsync ${PROJECT_NAME})

add_executable(pclsynctimer src/main_pclsynctimer.cpp)
ament_target_dependencies(pclsynctimer ${dependencies})
target_link_libraries(pclsynctimer ${PROJECT_NAME})

add_executable(pclsyncIcp src/main_pclsyncIcp.cpp)
ament_target_dependencies(pclsyncIcp ${dependencies})
target_link_libraries(pclsyncIcp ${PROJECT_NAME})

add_executable(pclsyncIcppcl src/main_pclsyncIcppcl.cpp)
ament_target_dependencies(pclsyncIcppcl ${dependencies})
target_link_libraries(pclsyncIcppcl ${PROJECT_NAME})

## depht2pcl node
add_executable(depth2pcl src/computer_vision/Depth2pcl.cpp)
target_include_directories(depth2pcl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(depth2pcl PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(depth2pcl ${dependencies})

target_link_libraries(depth2pcl ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  cv_program
  depth2pcl
  pclsync
  pclsyncIcp
  pclsyncIcppcl
  pclsynctimer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
