cmake_minimum_required(VERSION 3.10.0)

project(obstacle_fusion)


set(SOURCEFILES stdafx.h
        ObstacleFusionFilter.h
        ObstacleFusionFilter.cpp
        HitCount.h
        HitCount.cpp
        UltrasonicHitCount.h
        UltrasonicHitCount.cpp
        LaserSegBoolHitCount.h
        LaserSegBoolHitCount.cpp
        ../PinClasses/DetectionInfoPin.h
        ../PinClasses/DetectionInfoPin.cpp
        ../PinClasses/BoolSignalValuePin.h
        ../PinClasses/BoolSignalValuePin.cpp
        ../PinClasses/LaserSegPin.h
        ../PinClasses/LaserSegPin.cpp)

adtf_add_filter(${PROJECT_NAME}
        ${SOURCEFILES}
        )

# Specify where it should be installed to
adtf_install_target(${PROJECT_NAME} bin)

adtf_create_plugindescription(
        TARGET
        ${PROJECT_NAME}
        PLUGIN_SUBDIR
        "bin"
)

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
