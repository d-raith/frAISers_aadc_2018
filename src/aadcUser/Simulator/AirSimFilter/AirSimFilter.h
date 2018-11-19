
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
 * AUTHOR: Fabien Jenne
 *
 * AirSim filter implementation
 *
******************************************************************************/

#pragma once

#include <math.h>
#include <iostream>                             // AirSim
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <thread>                               // NOLINT
#include <atomic>
#include <future>
#include <mutex>                                // NOLINT
#include <chrono>                               // NOLINT / AirSim
#include <opencv2/opencv.hpp>

#include "stdafx.h"

// AirSim
#include "/opt/AirSim2/AirLib/include/common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif  // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
// end AirSim


using namespace cv;                             // NOLINT
using namespace msr::airlib;                    // NOLINT

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;
// typedef common_utils::FileSystem FileSystem;

#define CID_AIRSIM_FILTER "airsim_simulator.communication.adtf.cid"
#define LABEL_AIRSIM_FILTER "AirSim Simulation Filter"


class AirSimFilter : public adtf::filter::ant::cConfigurableFilter {
 public:
    AirSimFilter();

    ~AirSimFilter();

    ADTF_CLASS_ID_NAME(AirSimFilter, CID_AIRSIM_FILTER, LABEL_AIRSIM_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IKernel),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));

 private:
    //  ADTF
    adtf::base::property_variable<tBool> m_propEnableConsoleOutput = tTrue;
    object_ptr<adtf::services::IReferenceClock> m_pClock;
    tTimeStamp tmTimeOfLastActivation;
    std::atomic<tTimeStamp> tmTimeOfLastImage;
    // tTimeStamp tmTimeOfLastImage;
    tTimeStamp tmTimeOfLastPositionUpdate;

    adtf::base::property_variable<cString> m_propAirsimServerIp = cString("10.8.105.148");
    adtf::base::property_variable<cString> m_propAirsimImageId = cString("0");
    adtf::base::property_variable<tBool> m_propLimitPositionRefreshRate = tTrue;
    adtf::base::property_variable<tInt64> m_propPositionRefreshRate = 50;  // in 1/s
    adtf::base::property_variable<tBool> m_propUseMetricInput = tTrue;
    adtf::base::property_variable<tBool> m_propUseCompressedImageTransfer = tFalse;
    adtf::base::property_variable<tBool> m_propEnableFPSLogging = tFalse;
    adtf::base::property_variable<tBool> m_propEnableControlLogging = tFalse;
    adtf::base::property_variable<tBool> m_propEnablePositionLogging = tFalse;

    double control_value_speed;
    double control_value_steering;
    // AirSim
    msr::airlib::CarRpcLibClient* _airsim_client = NULL;

    // AirSim
    std::string airsim_server_ip = "10.8.105.148";
    uint16_t airsim_server_port = 41451;
    float airsim_connection_timeout = 60;
    bool airsim_client_connected = false;

    // Media descriptions
    //   tPosition
    cPinWriter m_oWriterPosition;
    /*! Media Descriptions. */
    struct tPositionId {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionId;
    /*! The tPosition data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    //   Speed/Steering
    cPinReader m_oReaderSpeed;
    cPinReader m_oReaderSteering;
    struct tSignalValueId {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;
    /*! The tSignaValue data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    //   Images
    cPinWriter m_oWriterImage;
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    // Threading
    std::thread airsim_image_thread;
    std::atomic<bool> image_thread_running;

 public:
    // implement cFilterBase (implements cFilterLevelmachine)
    tResult Init(adtf::streaming::ant::cFilterBase::tInitStage eStage);
    tResult Start();
    tResult Stop();
    tResult Shutdown(adtf::streaming::ant::cFilterBase::tInitStage eStage);
    tResult RunTrigger(tTimeStamp tmTimeofActivation);

    // AirSim client
    bool createAirSimClient();

 protected:
    // Map
    tResult setCarPoseFrom_tPosition();
    // ADTF
    tResult writeImageToPin(const cv::Mat& outputImage);
    tResult sendPositionStruct(
        const tTimeStamp& timeOfWriting, const tFloat32& f32X, const tFloat32& f32Y,
        const tFloat32& f32Radius, const tFloat32& f32Heading, const tFloat32& f32Speed);
    tResult readControlInputPins();

    // AirSim
    cv::Mat getAirSimImages();
    void writeNewestAirSimImage(tTimeStamp tmTimeOfImageUpdate);
    tResult writeNewestAirSimPosition();
    bool sendControlsToAirSim(double speed, double steering);
    std::vector<double> getAirSimVehiclePose();
    double yawFromQuaternion(const auto& q);
};
