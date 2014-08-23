cmake_minimum_required(VERSION 2.8)
PROJECT(INDI_QHYCCD_Linux C CXX)

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")

find_package(INDI REQUIRED)

set(indi_qhyccd_linux_SRCS ${CMAKE_SOURCE_DIR}/src/indi_qhyccd_linux.cpp)

add_executable(indi_qhyccd_linux_ccd ${indi_qhyccd_linux_SRCS})

target_link_libraries(indi_qhyccd_linux_ccd
        ${INDI_DRIVER_LIBRARIES}
        qhyccd
)
