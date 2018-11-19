cmake_minimum_required(VERSION 3.10.0)

project(airsim_filter)

find_package(Threads REQUIRED)
# message("AirSim in: ${AIRSIM_ROOT}")
message("RPC in: ${RPC_LIB_INCLUDES}")
#message(STATUS "Boost directory is: ${Boost_INCLUDE_DIR}")
#IMPORTANT: include Thrift(depends on Boost) before Boost, as CMake includes in reverse
include_directories(${OpenCV_INCLUDE_DIRS},
                    ${THRIFT_INCLUDE_DIRS},
                    ${AIRSIM_ROOT}/AirLib/deps/eigen3 ,
                    ${AIRSIM_ROOT}/AirLib/include ,
                    ${RPC_LIB_INCLUDES} ,
                    ${AIRSIM_ROOT}/MavLinkCom/include ,
                    ${AIRSIM_ROOT}/MavLinkCom/common_utils ,
                    ${Boost_INCLUDE_DIR})  # make Boost last include!


set (HEADERS            stdafx.h
						AirSimFilter.h )

set (SOURCES
                        AirSimFilter.cpp )

adtf_add_filter(${PROJECT_NAME}
                ${HEADERS}
                ${SOURCES}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOGEN_TARGET_DEPENDS $(HEADERS))

target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} ${THRIFT_LINK_LIBS} ${CMAKE_THREAD_LIBS_INIT} /opt/AirSim2/AirLib/lib/libAirLib.a /opt/AirSim2/AirLib/deps/rpclib/lib/librpc.a)


# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
    DEPENDENT_PLUGINS
        "adtf_xsystem"
 )

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
