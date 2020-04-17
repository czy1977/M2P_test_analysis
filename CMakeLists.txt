cmake_minimum_required(VERSION 3.5)

project(m2p_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system filesystem )

include_directories(M2P_test_analysis)
file(GLOB SRCS M2P_test_analysis/*.cpp M2P_test_analysis/*.c)

message(WARNING ${SRCS})

list(REMOVE_ITEM SRCS ${CMAKE_CURRENT_SOURCE_DIR}/cmdline/test.cpp)
list(REMOVE_ITEM SRCS ${CMAKE_CURRENT_SOURCE_DIR}/cmdline/test2.cpp)

message(WARNING ${SRCS})

add_executable(m2p_test ${SRCS})

target_link_libraries(m2p_test ${OpenCV_LIBS} -lpthread  -lboost_filesystem -lboost_system)
