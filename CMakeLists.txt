# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Nguyen Dong Hai Phuong <phuong.nguyen@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME rc_yarp_wrapper)
project(${PROJECTNAME})
#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/lib/cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

aux_source_directory(. SRC_LIST)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
find_package(OpenCV REQUIRED)
find_package(RC_GENICAM_API REQUIRED)

message(STATUS ${RC_GENICAM_API_DIR})

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

add_definitions(${YARP_DEFINES})   # add yarp definitions - this contains also -D_REENTRANT
include(YarpInstallationHelpers)   # pick up yarp's cmake scripts

set(main_project_path ${CMAKE_CURRENT_SOURCE_DIR})



#add_subdirectory(modules)

#add_subdirectory(app)
add_subdirectory(test)
add_subdirectory(rc_yarp_wrapper)


icubcontrib_add_uninstall_target()

