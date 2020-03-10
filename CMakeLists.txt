cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME tello_controller)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake

#add_definitions(-std=c++0x)
#add_definitions(-std=c++0x)
#add_definitions(-std=c++03)
add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


set(DRIVER_TELLO_SOURCE_DIR
	src/source)
	
set(DRIVER_TELLO_INCLUDE_DIR
	src/include
	)

set(DRIVER_TELLO_HEADER_FILES
        src/include/socket_tello.h
        src/include/tello_camera_interface.h
        src/include/tello_command_interface.h
        src/include/tello_state_interface.h
    )

set(DRIVER_TELLO_SOURCE_FILES
        src/source/socket_tello.cpp
        src/source/socket_tello_camera.cpp
        src/source/socket_tello_command.cpp
        src/source/socket_tello_state.cpp
    )

set(msg_files
  "msg/command.msg"
)



find_package(catkin REQUIRED
                COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs mavros tf_conversions cv_bridge message_generation)

add_message_files(
  FILES
  "command.msg"
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#       INCLUDE_DIRS ${OKTODRIVER_INCLUDE_DIR}
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs gazebo_msgs mavros tf_conversions message_runtime
  )


include_directories(${DRIVER_TELLO_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

add_library(tello_controller ${DRIVER_TELLO_SOURCE_FILES} ${DRIVER_TELLO_HEADER_FILES})
add_dependencies(tello_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_controller ${catkin_LIBRARIES} avcodec avutil swscale)

add_executable(tello_camera_interface  src/source/tello_camera_interface.cpp)
add_dependencies(tello_camera_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_camera_interface tello_controller)
target_link_libraries(tello_camera_interface ${catkin_LIBRARIES})

add_executable(tello_command_interface  src/source/tello_command_interface.cpp)
add_dependencies(tello_command_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_command_interface tello_controller)
target_link_libraries(tello_command_interface ${catkin_LIBRARIES})

add_executable(tello_state_interface  src/source/tello_state_interface.cpp)
add_dependencies(tello_state_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(tello_state_interface tello_controller)
target_link_libraries(tello_state_interface ${catkin_LIBRARIES})

set_target_properties(tello_controller PROPERTIES LINKER_LANGUAGE CXX)