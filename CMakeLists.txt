cmake_minimum_required(VERSION 3.1)
project(terrain_characterizer)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  genmsg
  sensor_msgs
  cv_bridge
  roscpp
  std_msgs
  image_transport
)

find_package(OpenCV)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(realsenseNode
               src/realsenseNode.cpp
              )
target_link_libraries(realsenseNode ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(realsenseNode ${catkin_EXPORTED_TARGETS})

add_executable(pointCloudNode
               src/pointCloudNode.cpp
              )
target_link_libraries(pointCloudNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointCloudNode ${catkin_EXPORTED_TARGETS})

