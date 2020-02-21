cmake_minimum_required(VERSION 3.5)

project(m2p_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(M2P_test_analysis)
file(GLOB_RECURSE SRCS M2P_test_analysis/*.cpp M2P_test_analysis/*.c)


add_executable(m2p_test ${SRCS})

target_link_libraries(m2p_test ${OpenCV_LIBS} )
