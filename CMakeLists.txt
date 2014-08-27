cmake_minimum_required(VERSION 2.8)
PROJECT(INDI_QHYCCD_Linux C CXX)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")
SET(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -std=c++0x -g -O0" )
SET(CMAKE_CXX_FLAGS_DEBUG "-Werror" )

find_package(INDI REQUIRED)
find_package(CFITSIO REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})
include_directories( ${CFITSIO_INCLUDE_DIR})
include_directories("/usr/local/include/libqhyccd")
include_directories("/usr/local/include/libindi")

set(indi_qhyccd_linux_SRCS ${CMAKE_SOURCE_DIR}/src/indi_qhyccd_linux.cpp)

add_executable(indi_qhyccd_linux_ccd ${indi_qhyccd_linux_SRCS})

target_link_libraries(indi_qhyccd_linux_ccd
        ${INDI_DRIVER_LIBRARIES}
		${FITSIO_LIBRARIES}
        qhyccd
)
