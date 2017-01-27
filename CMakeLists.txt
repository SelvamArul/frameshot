cmake_minimum_required(VERSION 2.8)
project(frameshot)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	image_transport
	pcl_ros
	cv_bridge
	camera_calibration_parsers
)
find_package(Boost REQUIRED COMPONENTS
	thread
	filesystem
)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(EIGEN3_INCLUDE_DIR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -std=c++11")

catkin_package(
	CATKIN_DEPENDS pcl_ros
)
add_executable(frameshot
		src/frameshot.cpp
)
target_link_libraries(frameshot
	${catkin_LIBRARIES} ${PCL_LIBRARIES}  libvtkCommon.so libvtkFiltering.so
)

add_executable(pcd2png
	src/pcd2png.cpp
)
target_link_libraries(pcd2png
	${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBS} libvtkCommon.so libvtkFiltering.so
)

add_executable(tts
	src/table_top_segmentation.cpp
)
target_link_libraries(tts
	${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBS} libvtkCommon.so libvtkFiltering.so
)

add_executable(gamma_tuner src/gamma_tuner.cpp)
target_link_libraries(gamma_tuner  ${OpenCV_LIBS} ${catkin_LIBRARIES})
