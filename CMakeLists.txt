cmake_minimum_required(VERSION 2.8.3)
project(ar_sys_prep)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
    	std_msgs 
	message_generation
	image_transport
	cv_bridge
        camera_info_manager
)


find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src/)
include_directories(${PROJECT_SOURCE_DIR}/include/)

SET(SOURCES 
	${PROJECT_SOURCE_DIR}/src/img_filter_node.cpp
	${PROJECT_SOURCE_DIR}/src/img_filter.cpp
   )
SET(HEADERS 
	${PROJECT_SOURCE_DIR}/include/ar_sys_prep/img_filter_node.h
   )

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp message_runtime
  LIBRARIES
  roscpp
  image_transport
  cv_bridge
  cmake_modules
  camera_info_manager
)

add_executable(img_filter ${SOURCES} ${HEADERS})
add_dependencies(img_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(img_filter ${OpenCV_LIBS} ${catkin_LIBRARIES})
