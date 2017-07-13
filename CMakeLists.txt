cmake_minimum_required(VERSION 2.8.3)
project(karto_ros)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    karto_scan_matcher
    rosconsole
    roscpp
    sensor_msgs
    sparse_bundle_adjustment
    tf
    visualization_msgs
)
find_package(Eigen3 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package()

add_executable(karto_ros src/karto_ros.cpp)
target_link_libraries(karto_ros ${catkin_LIBRARIES})

install(TARGETS karto_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
