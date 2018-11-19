cmake_minimum_required(VERSION 3.10.0)

project(nn_thrift_filter)

# find_package(Threads REQUIRED)
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}/opt/libtorch/")
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}/opt/cudNN/")
# find_package(Torch REQUIRED)

#message(STATUS "Boost directory is: ${Boost_INCLUDE_DIR}")
#IMPORTANT: include Thrift(depends on Boost) before Boost, as CMake includes in reverse
include_directories(${OpenCV_INCLUDE_DIRS},
                    # ${TORCH_INCLUDES},
                    ${THRIFT_INCLUDE_DIRS},
                    ${Boost_INCLUDE_DIR})


set (HEADERS            stdafx.h
                        ../NNThrift/gen-cpp/nn_thrift_constants.h
                        ../NNThrift/gen-cpp/nn_thrift_types.h
                        ../NNThrift/gen-cpp/NNComm.h
						NNThriftFilter.h )

set (SOURCES
                        ../NNThrift/gen-cpp/nn_thrift_constants.cpp
                        ../NNThrift/gen-cpp/nn_thrift_types.cpp
                        ../NNThrift/gen-cpp/NNComm.cpp
                        NNThriftFilter.cpp )

adtf_add_filter(${PROJECT_NAME}
                ${HEADERS}
                ${SOURCES}
)

set_property(TARGET ${PROJECT_NAME} PROPERTY AUTOGEN_TARGET_DEPENDS $(HEADERS))

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} ${THRIFT_LINK_LIBS})
# target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${TORCH_LIBRARIES})

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
    DEPENDENT_PLUGINS
        "adtf_xsystem"
 )

#this is to add a folder within the ADTF solution
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER user)
set_property(TARGET ${PROJECT_NAME}_pdgen PROPERTY FOLDER user)
