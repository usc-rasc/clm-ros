project(CLM-ROS)
cmake_minimum_required(VERSION 3.2.2)

find_package (OpenCV REQUIRED)

file (GLOB SRCS src/*cpp)

add_executable(clm-ros ${SRCS})

include_directories(/usr/include/boost)
			
target_link_libraries(clm-ros CLM dlib boost_filesystem boost_system ${OpenCV_LIBS} lapack)

add_definitions(-pthread -std=c++11)

target_link_libraries(clm-ros ${OpenCV_LIBS} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

