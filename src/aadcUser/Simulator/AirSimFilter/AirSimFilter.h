
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#pragma once

#include <math.h>
#include <iostream>          
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <thread>              
#include <atomic>
#include <future>
#include <mutex>                         
#include <chrono>                          
#include <opencv2/opencv.hpp>

#include "stdafx.h"
#include "/opt/AirSim2/AirLib/include/common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON
#include "vehicles/car/api/CarRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"


using namespace cv;                         
using namespace msr::airlib;           

typedef ImageCaptureBase::ImageRequest ImageRequest;
typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageType ImageType;

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
    msr::airlib::CarRpcLibClient* _airsim_client = NULL;

    std::string airsim_server_ip = "10.8.105.148";
    uint16_t airsim_server_port = 41451;
    float airsim_connection_timeout = 60;
    bool airsim_client_connected = false;

    cPinWriter m_oWriterPosition;
    struct tPositionId {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionId;
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;
    cPinReader m_oReaderSpeed;
    cPinReader m_oReaderSteering;
    struct tSignalValueId {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;
    cPinWriter m_oWriterImage;
    adtf::streaming::tStreamImageFormat m_sImageFormat;
    std::thread airsim_image_thread;
    std::atomic<bool> image_thread_running;

 public:
    tResult Init(adtf::streaming::ant::cFilterBase::tInitStage eStage);
    tResult Start();
    tResult Stop();
    tResult Shutdown(adtf::streaming::ant::cFilterBase::tInitStage eStage);
    tResult RunTrigger(tTimeStamp tmTimeofActivation);
    bool createAirSimClient();

 protected:
    tResult setCarPoseFrom_tPosition();
    tResult writeImageToPin(const cv::Mat& outputImage);
    tResult sendPositionStruct(
        const tTimeStamp& timeOfWriting, const tFloat32& f32X, const tFloat32& f32Y,
        const tFloat32& f32Radius, const tFloat32& f32Heading, const tFloat32& f32Speed);
    tResult readControlInputPins();
    cv::Mat getAirSimImages();
    void writeNewestAirSimImage(tTimeStamp tmTimeOfImageUpdate);
    tResult writeNewestAirSimPosition();
    bool sendControlsToAirSim(double speed, double steering);
    std::vector<double> getAirSimVehiclePose();
    double yawFromQuaternion(const auto& q);
};
