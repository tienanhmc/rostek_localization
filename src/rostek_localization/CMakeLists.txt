cmake_minimum_required(VERSION 3.5)
project(rostek_localization)
set(CMAKE_CXX_STANDARD 11)
# set(BUILD_SHARED_LIBS ON)
set(PACKAGE_DEPENDENCIES
  cartographer_ros_msgs
  geometry_msgs
  message_runtime
  nav_msgs
  pcl_conversions
  rosbag
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  urdf
  tf
  visualization_msgs
  roscpp
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  actionlib 
  actionlib_msgs
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
)


find_package(Boost REQUIRED iostreams system serialization filesystem thread)
# find_package(ZeroMQ REQUIRED)
find_package(Protobuf REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cartographer REQUIRED)

add_definitions(${EIGEN3_DEFINITIONS})

# set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)

include_directories(include
        external
        # build/msgs
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        )
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs  
)
# add_executable(cartographer_localization src/robot_localization.cpp)
# add_dependencies(cartographer_localization protobuf_compilation)


# target_link_libraries(cartographer_localization PUBLIC
#         protobuf::libprotobuf
#         ${ZeroMQ_LIBRARY}
#         ${Boost_LIBRARIES}
#         Eigen3::Eigen
#         yaml-cpp
#         cartographer
#         ${catkin_LIBRARIES}
#         )


add_executable(test11 src/test.cpp)
target_link_libraries(test11 PUBLIC
        protobuf::libprotobuf
        ${ZeroMQ_LIBRARY}
        ${Boost_LIBRARIES}
        Eigen3::Eigen
        yaml-cpp
        cartographer
        ${catkin_LIBRARIES}
        )


# add_executable(fastt src/fast.cpp)
# target_link_libraries(fastt PUBLIC
#         protobuf::libprotobuf
#         ${ZeroMQ_LIBRARY}
#         ${Boost_LIBRARIES}
#         Eigen3::Eigen
#         yaml-cpp
#         cartographer
#         ${catkin_LIBRARIES}
#         )