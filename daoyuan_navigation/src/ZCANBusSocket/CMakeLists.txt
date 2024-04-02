cmake_minimum_required(VERSION 3.0)
project(ZCANBusSocket)

set(CMAKE_CXX_STANDARD 11)

# set(USE_PEAK true)

# set(USE_KVASER true)

# set(USE_ZLG true)

# set(USE_ZLG2 true)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(_COMPILER_ARCHITECTURE x64)
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()
set(CAN_FILE CANHandler.cpp)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Socket)
add_definitions(-DUSE_SOCKET)
list(APPEND CAN_FILE CANSocket.cpp)
include_directories((${Socket_INCLUDE_DIR}))
link_libraries(${Socket_LIBS})

if(CMAKE_HOST_UNIX)
  link_libraries(pthread)
endif(CMAKE_HOST_UNIX)

set(CMAKE_DEBUG_POSTFIX _d)
add_library(${PROJECT_NAME} ${CAN_FILE})
