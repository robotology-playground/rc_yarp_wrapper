# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Phuong Nguyen
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(rc_yarp_wrapper)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include(YarpIDL)

#file(GLOB source *.cpp)
#file(GLOB header *.h)

set (include_files	${CMAKE_CURRENT_SOURCE_DIR}/rcYarpWrapper.h)


set (source_files 	${CMAKE_CURRENT_SOURCE_DIR}/rcYarpWrapper.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/runModule.cpp
#                        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
                        )
include_directories(${OpenCV_INCLUDE_DIRS}
					#${RC_GENICAM_API_DIRS}
					#${RC_GENICAM_API_LIBRARIES}
					#${RC_GENICAM_API_INCLUDES}
					/home/pnguyen/.local/include
					/home/pnguyen/.local/include/rc_genicam_api/genicam
					)
include_directories(${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})
#include_directories(${main_project_path}/lib/include)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

set(idl_files    rcYarpWrapper.thrift)

yarp_add_idl(IDL_GEN_FILES rcYarpWrapper.thrift)

source_group("IDL Files" FILES ${idl_files})

add_definitions(-D_USE_MATH_DEFINES)

add_executable(rc_yarp_wrapper ${source} ${header} ${source_files} ${RC_GENICAM_API_DIR} ${include_files} ${IDL_GEN_FILES} ${idl_files}) #${lib_files})# ${source_files} ${include_files} )

target_link_libraries(rc_yarp_wrapper ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${RC_GENICAM_API_LIBRARIES} ${RC_GENICAM_API_DIR})

install(TARGETS rc_yarp_wrapper DESTINATION bin)

