cmake_minimum_required(VERSION 3.10.0)

project(state_machine)


set(LIBMAP_DIR ${CMAKE_SOURCE_DIR}/Map/MapCore/)
include_directories(${LIBMAP_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})

set(SOURCEFILES
        ICarCtrl.h
        StateMachineCtrl.h
        StateMachineFilter.h
        IControlOutput.h
        Point.h
        IPlanner.h
        PrintUtils.h
        Astar.h
        stdafx.h
        GridMap.h
        GridMap.cpp
        Astar.cpp
        StateMachineCtrl.cpp
        StateMachineFilter.cpp
        ObstacleProcessor.h
        ObstacleProcessor.cpp
        LocalMap.h
        LocalMap.cpp
        customtypes.h
        Lane.h
        BaseTask.h
        BaseTask.cpp
        PlanningTask.cpp
        PlanningTask.h
        WaypointTask.h
        WaypointTask.cpp
        LaneFollowingTask.h
        LaneFollowingTask.cpp
        LaneFollowingPerceptionTask.h
        LaneFollowingPerceptionTask.cpp
        DijkstraGoalpointExtractor.h
        DijkstraGoalpointExtractor.cpp
        LaneTracker.h
        LaneTracker.cpp
        SubGoalTask.h
        SubGoalTask.cpp
        CarModel.h
        CarModel.cpp
        TurnRightTask.h
        TurnRightTask.cpp
        TurnLeftTask.h
        TurnLeftTask.cpp
        RoadSignInfo.h
        RoadSignInfo.cpp
        ParkingTask.h
        ParkingTask.cpp
        ParkOutTask.h
        ParkOutTask.cpp
        PurePursuit.h
        PurePursuit.cpp
        DriveStraightTask.h
        DriveStraightTask.cpp
        ReverseDrivingTask.h
        ReverseDrivingTask.cpp
        OvertakingTask.h
        OvertakingTask.cpp
        MergeTask.h
        MergeTask.cpp
        #OmplPlanner.h
        #OmplPlanner.cpp
        BackgroundPlanner.h
        BackgroundPlanner.cpp
        JuryComm.h
        JuryComm.cpp
        TaskFactory.h
        Maneuver.h
        Maneuver.cpp
        ISensorListeners.h
        EmergencyBrakeOverride.h
        EnvironmentState.h
        EnvironmentState.cpp
        StVO.h
        StVO.cpp
        ../PinClasses/LaneListPin.cpp
        ../PinClasses/LaneListPin.h
        ../PinClasses/PointListPin.h
        ../PinClasses/PointListPin.cpp
        ../PinClasses/RoadSignDataPin.h
        ../PinClasses/RoadSignDataPin.cpp
        ../PinClasses/LightCtrlPin.h
        ../PinClasses/LightCtrlPin.cpp
        ../PinClasses/LaserScannerPin.h
        ../PinClasses/LaserScannerPin.cpp
        ../PinClasses/OpencvFramePin.h
        ../PinClasses/OpencvFramePin.cpp
        ../PinClasses/VideoOutPin.h
        ../PinClasses/VideoOutPin.cpp
        ../PinClasses/AsyncSampleBuffer.h
        ../PinClasses/AsyncSampleBuffer.cpp
        ../PinClasses/JuryStructPin.h
        ../PinClasses/JuryStructPin.cpp
        ../PinClasses/ManeuverListPin.h
        ../PinClasses/ManeuverListPin.cpp
        ../PinClasses/DriverStructPin.h
        ../PinClasses/DriverStructPin.cpp
        ../PinClasses/RoadSignsMapPin.h
        ../PinClasses/RoadSignsMapPin.cpp
        ../PinClasses/LaserSegPin.h
        ../PinClasses/LaserSegPin.cpp
        ../PinClasses/DetectionInfoPin.h
        ../PinClasses/DetectionInfoPin.cpp
        )

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
#target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ompl)
link_directories(${CMAKE_BINARY_DIR}/Map/MapCore)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC globalmap)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ompl)


#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
