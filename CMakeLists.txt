project(CLM-ROS)
cmake_minimum_required(VERSION 2.8.12.2)

find_package (OpenCV REQUIRED)

file (GLOB SRCS src/*cpp)

add_executable(clm-ros ${SRCS})

include_directories(/usr/include/boost /CLM-framework-master/lib/local/CLM/include /CLM-framework-master/lib/3rdParty/dlib/include) 

set(CLM_BUILD /CLM-framework-master/build)

link_directories(${CLM_BUILD}/lib/local/CLM ${CLM_BUILD}/lib/3rdParty/dlib)

find_library(CLM_LIBRARY NAMES CLM PATHS ${CLM_BUILD}/lib/local/CLM)

find_library(DLIB_LIBRARY NAMES dlib PATHS ${CLM_BUILD}/lib/3rdParty/dlib)
			
target_link_libraries(clm-ros ${CLM_LIBRARY} ${DLIB_LIBRARY} boost_filesystem boost_system ${OpenCV_LIBS} lapack pthread tbb)

add_definitions(-pthread -std=c++11)

target_link_libraries(clm-ros ${OpenCV_LIBS} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

