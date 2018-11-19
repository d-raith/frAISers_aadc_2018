cmake_minimum_required(VERSION 3.10.0)

set(MAPLIB_DIR ${CMAKE_SOURCE_DIR}/Map/MapCore)
message(STATUS "Compiling Map library at: ${MAPLIB_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR},
                    ${OpenCV_INCLUDE_DIRS},
                    ${THRIFT_INCLUDE_DIRS},
                    ${Boost_INCLUDE_DIR}
                    )

add_library(globalmap SHARED ${MAPLIB_DIR}/Map.cpp
                             ${MAPLIB_DIR}/MapObjects.h
                             ${MAPLIB_DIR}/MapObjects.cpp
                             ${MAPLIB_DIR}/IGlobalMap.h
                             ../MapThrift/gen-cpp/map_data_structure_constants.h
                             ../MapThrift/gen-cpp/map_data_structure_types.h
                             ../MapThrift/gen-cpp/map_data_structure_constants.cpp
                             ../MapThrift/gen-cpp/map_data_structure_types.cpp
                             ../../PinClasses/StopWatch.h)

# set_property(TARGET globalmap PROPERTY POSITION_INDEPENDENT_CODE ON)  # important!

set_target_properties(
    globalmap PROPERTIES CXX_STANDARD 11  # important!
)

target_link_libraries(globalmap PUBLIC ${OpenCV_LIBS} ${THRIFT_LINK_LIBS} ${Boost_LIBRARIES})