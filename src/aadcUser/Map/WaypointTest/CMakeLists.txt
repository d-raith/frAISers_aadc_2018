cmake_minimum_required(VERSION 3.10.0)

project(waypoint_test_filter)

find_package(Threads REQUIRED)

set(MAPLIB_DIR ${CMAKE_SOURCE_DIR}/Map/MapCore/)
include_directories(${MAPLIB_DIR})

#IMPORTANT: include Thrift(depends on Boost) before Boost, as CMake includes in reverse
include_directories(${OpenCV_INCLUDE_DIRS},
                    ${Boost_INCLUDE_DIR})  # make Boost last include!


set (HEADERS            stdafx.h
						WaypointTestFilter.h )

set (SOURCES
                        WaypointTestFilter.cpp )

adtf_add_filter(${PROJECT_NAME}
                ${HEADERS}
                ${SOURCES}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOGEN_TARGET_DEPENDS $(HEADERS))
link_directories(${CMAKE_BINARY_DIR}/Map/MapCore)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC globalmap ${OpenCV_LIBS} ${Boost_LIBRARIES} ${THRIFT_LINK_LIBS} ${CMAKE_THREAD_LIBS_INIT})


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
