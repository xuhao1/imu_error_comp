cmake_minimum_required(VERSION 2.8.3)
project(imu_error_comp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  message_filters
)

find_package(Boost REQUIRED COMPONENTS system thread signals)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

add_library(imu_error_comp src/imu_error_comp.cpp)
target_link_libraries(imu_error_comp ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_LIBRARIES})

add_executable(imu_error_comp_node src/imu_error_comp_node.cpp)
target_link_libraries(imu_error_comp_node ${catkin_LIBRARIES} imu_error_comp)
