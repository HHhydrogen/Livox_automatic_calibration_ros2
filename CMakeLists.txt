cmake_minimum_required(VERSION 3.8)

project(livox_automatic_calibration_ros2)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(${PCL_DEFINITIONS})
include_directories(
	include
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

file(GLOB SRCS0 src/mapping/*.cpp)
file(GLOB SRCS1 src/calibration/*.cpp)
file(GLOB SRCS2 src/ransac/*.cpp)

add_executable(mapping ${SRCS0})
target_link_libraries(mapping ${PCL_LIBRARIES})
ament_target_dependencies(mapping ament_index_cpp)

add_executable(calibration ${SRCS1})
target_link_libraries(calibration ${PCL_LIBRARIES})
ament_target_dependencies(calibration ament_index_cpp)

add_executable(fitline ${SRCS2})
target_link_libraries(fitline ${PCL_LIBRARIES})
ament_target_dependencies(fitline ament_index_cpp)

install(TARGETS
	mapping
	calibration
	fitline
	DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
	run.sh
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
	data
	pic
	DESTINATION share/${PROJECT_NAME}
)

install(FILES
	README.md
	readme-CN.md
	LICENSE
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
