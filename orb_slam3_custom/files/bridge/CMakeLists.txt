cmake_minimum_required(VERSION 3.5)
project(airsim_stereo_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# AirSim root
if(NOT DEFINED ENV{AIRSIM_ROOT})
    message(FATAL_ERROR "AIRSIM_ROOT environment variable not set")
endif()

set(AIRSIM_ROOT $ENV{AIRSIM_ROOT})

include_directories(
    ${AIRSIM_ROOT}/AirLib/include
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/include
    ${AIRSIM_ROOT}/AirLib/deps/MavLinkCom/include
    ${PROJECT_SOURCE_DIR}/..
    ${PROJECT_SOURCE_DIR}/../include
)

link_directories(
    ${AIRSIM_ROOT}/AirLib/lib
    ${AIRSIM_ROOT}/AirLib/deps/rpclib/lib
    ${AIRSIM_ROOT}/AirLib/deps/MavLinkCom/lib
)

add_executable(airsim_stereo_bridge
    airsim_stereo_bridge.cc
)
target_compile_features(airsim_stereo_bridge PRIVATE cxx_std_17)
target_link_libraries(airsim_stereo_bridge
    AirLib
    rpc
    MavLinkCom
    ORB_SLAM3
    pthread
)
