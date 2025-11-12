cmake_minimum_required(VERSION 3.8)
project(SyncRead CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")

# collect driver source files
set(DRIVER_SRCS
    SCS.cpp
    SCSCL.cpp
    SCSerial.cpp
    SMS_STS.cpp
    SMSCL.cpp
)

add_library(SCServo STATIC ${DRIVER_SRCS})

target_include_directories(SCServo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(sync_read sync_read.cpp)

target_link_libraries(sync_read PRIVATE SCServo)
