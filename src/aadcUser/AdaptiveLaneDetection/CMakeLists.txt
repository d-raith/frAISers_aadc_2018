cmake_minimum_required(VERSION 3.10.0)



project(adaptive_lane_detection)


#
# set include directories
#
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(/usr/include/eigen3/)
include_directories(/opt/qpOASES/include)


set (SOURCEFILES    	stdafx.h
						AdaptiveLaneDetection.h
						AdaptiveLaneDetection.cpp
                        adtf_log_macros.h
                        camera_transformations.h
                        camera_transformations.cpp
                        customtypes.h
                        line_points_detection.cpp
                        line_points_detection.h
                        nonpin_types.h
                        xml_helper.hpp
						subproblem.h
						subproblem.cpp
						lane_finder.h
                        lane_finder.cpp
                        ../PinClasses/LaneListPin.cpp
                        ../PinClasses/LaneListPin.h
						../PinClasses/PointListPin.h
						../PinClasses/PointListPin.cpp
                        ../PinClasses/OpencvFramePin.h
                        ../PinClasses/OpencvFramePin.cpp
						../PinClasses/AsyncSampleBuffer.h)

adtf_add_filter(${PROJECT_NAME}
				${SOURCEFILES}
)


# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

#install dlls for opencv to binary folder
if(WIN32)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}d.dll" DESTINATION ${CMAKE_INSTALL_BINARY}/debug CONFIGURATIONS Debug)
    install(FILES "${OpenCV_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin/opencv_world${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}.dll" DESTINATION ${CMAKE_INSTALL_BINARY} CONFIGURATIONS RelWithDebInfo Release)
endif(WIN32)

adtf_create_plugindescription(
    TARGET
        ${PROJECT_NAME}
    PLUGIN_SUBDIR
        "bin"
 )


#
# Set library binaries for linking
#
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC /opt/qpOASES/bin/libqpOASES.a /opt/qpOASES/src/LAPACKReplacement.o /opt/qpOASES/src/BLASReplacement.o)


#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
