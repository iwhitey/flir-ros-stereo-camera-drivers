cmake_minimum_required(VERSION 2.8.3)
project(stereo_camera_driver)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" CACHE PATH "Project Include Directory")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		message_generation
		cv_bridge
		image_transport
		sensor_msgs
		dynamic_reconfigure
		)


find_package(Spinnaker REQUIRED)
message("spinnaker lib :   " ${Spinnaker_LIBRARIES})
find_package(OpenCV REQUIRED)
find_package(LibUnwind REQUIRED)

find_package(Boost REQUIRED)
if(Boost_FOUND)
	find_package ( Boost COMPONENTS filesystem system serialization REQUIRED )
	set(Boost_GENERAL ${Boost_LIBRARIES})
elseif(NOT Boost_FOUND)
	message("Boost not found!")
endif()

# configure a header file to pass some of the CMake settings
# to the source code
configure_file (
		"${PROJECT_INCLUDE_DIR}/stereo_camera_driver/spinnaker_configure.h.in"
		"${PROJECT_INCLUDE_DIR}/stereo_camera_driver/spinnaker_configure.h"
)

add_message_files(
		FILES
		SpinnakerImageNames.msg
)

generate_dynamic_reconfigure_options(
		cfg/spinnaker_cam.cfg

)

generate_messages(
		DEPENDENCIES
		std_msgs
)

catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS roscpp std_msgs message_runtime
		DEPENDS OpenCV LibUnwind
)

include_directories(
		${PROJECT_INCLUDE_DIR}
		${catkin_INCLUDE_DIRS}
		SYSTEM ${Spinnaker_INCLUDE_DIRS}
		SYSTEM ${OpenCV_INCLUDE_DIRS}
		SYSTEM ${Boost_INCLUDE_DIR}
		${LibUnwind_INCLUDE_DIRS}
)

link_directories( ${Spinnaker_LIBRARIES} )

set (LIBS ${LibUnwind_LIBRARIES} ${Spinnaker_LIBRARIES} ${OpenCV_LIBS} ${Boost_GENERAL})

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

add_library (stereolib SHARED
		src/camera.cpp)
add_dependencies(stereolib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(stereolib ${LIBS} ${catkin_LIBRARIES})




add_executable(stereo_GigE_node src/stereo_node.cpp
								src/StereoGigE.cpp
								src/camera.cpp
								src/pi_controller.cpp
								)
add_dependencies(stereo_GigE_node stereolib ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries (stereo_GigE_node stereolib ${LIBS} ${catkin_LIBRARIES} yaml-cpp)

