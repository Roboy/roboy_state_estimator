cmake_minimum_required(VERSION 2.8.3)
project(roboy_state_estimator)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS
		roscpp
		roboy_middleware_msgs
		common_utilities
		geometry_msgs
		moveit_msgs
		sensor_msgs
		cv_bridge
		std_msgs
		image_transport
		tf
		tf_conversions
		)
catkin_package(CATKIN_DEPENDS message_runtime roboy_middleware_msgs common_utilities INCLUDE_DIRS include)

# Eigen
find_package(PkgConfig REQUIRED )
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#==============================================================================
# Libraries
#==============================================================================
#add_executable(roboy_state_estimator include/roboy_state_estimator/roboyStateEstimator.hpp src/roboyStateEstimator.cpp)
#add_dependencies(roboy_state_estimator
#	roboy_middleware_msgs_generate_messages_cpp
#)
#target_link_libraries(roboy_state_estimator ${catkin_LIBRARIES})
#==============================================================================
# Installation
#==============================================================================
install(TARGETS

		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY
        scripts/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
        )
