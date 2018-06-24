cmake_minimum_required(VERSION 2.8.3)
project(rospix)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  std_msgs
  dynamic_reconfigure
  message_generation
)

add_message_files(FILES
  Status.msg
  Image.msg
)

add_service_files(FILES
  BatchExposure.srv
  Exposure.srv
  SetInt.srv
  SetDouble.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
	INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime # image_transport roscpp rospy std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Store in CMAKE_DEB_HOST_ARCH var the current build architecture
execute_process(COMMAND
  dpkg-architecture
    -qDEB_HOST_ARCH
  OUTPUT_VARIABLE
    CMAKE_DEB_HOST_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Choose the timepix library based on the system architectur
if(${CMAKE_DEB_HOST_ARCH} MATCHES "armhf")
  set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/armhf/libtpx.so)
elseif(${CMAKE_DEB_HOST_ARCH} MATCHES "i386")
  set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/i386/libtpx.so)
else(${CMAKE_DEB_HOST_ARCH} MATCHES "x64")
  set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/x64/libtpx.so)
else()
  MESSAGE(FATAL_ERROR "Library for timepix has not been selected, check CMakeLists.txt.")
endif()

# the main app
add_executable(rospix src/rospix.cpp src/timepix_handler.cpp)
add_dependencies(rospix ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rospix
  ${catkin_LIBRARIES}
  ${EXTRALIB_BIN}
)

# list devices
add_executable(list_devices src/list_devices.cpp)
add_dependencies(list_devices ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(list_devices
  ${catkin_LIBRARIES}
  ${EXTRALIB_BIN}
)

install(TARGETS
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.h"
		PATTERN ".svn" EXCLUDE
)

install(DIRECTORY doc
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
