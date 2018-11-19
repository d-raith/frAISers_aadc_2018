
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#pragma once
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>
#include <thrift/stdcxx.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <thread>                               
#include <mutex>                                
#include <opencv2/opencv.hpp>

#include "stdafx.h"
#include "../MapCore/Map.h"
#include "../MapThrift/gen-cpp/MapComm.h"
#include "../../VoiceControl/VoiceControlThrift/gen-cpp/ControlComm.h"


using namespace ::apache::thrift;               
using namespace ::apache::thrift::protocol;     
using namespace ::apache::thrift::transport;    
using namespace ::apache::thrift::server;       
using namespace map_thrift;                     
using namespace control_thrift;                 
using namespace cv;                             


#define CID_THRIFT_FILTER "universal_thrift.communication.adtf.cid"
#define LABEL_THRIFT_FILTER "Universal Thrift Filter"


class ThriftFilter : public adtf::filter::ant::cConfigurableFilter {
 public:
    ThriftFilter();

    ~ThriftFilter();

    ADTF_CLASS_ID_NAME(ThriftFilter, CID_THRIFT_FILTER, LABEL_THRIFT_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IKernel),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));

 private:
    property_variable<tFloat32> m_propPositionUpdateInterval = 200;
    property_variable<cString> m_propThriftRemoteServerIp = cString("192.168.168.127");
    property_variable<int> m_propThriftRemoteServerPort = 9091;
    property_variable<int> m_propThriftServerPort = 9090;
    property_variable<tBool> m_propLoadMapFromFile = tFalse;
    property_variable<tBool> m_propSaveMapFromServer = tFalse;
    property_variable<tBool> m_propUpdateMapFromThrift = tTrue;
    property_variable<cString> m_propMapFilePath = cString("/home/aadc/AADC/map_files/maps_car/");
    property_variable<tBool> m_propSendMapUpdates = tFalse;
    property_variable<tBool> m_propSendPositionUpdates = tTrue;
    property_variable<tBool> m_propEnablePositionLogging = tFalse;
    property_variable<tBool> m_propEnableMapLogging = tTrue;
    property_variable<tBool> m_propSendOutImages = tFalse;
    property_variable<tBool> m_propDisableThriftCommunication = tFalse;
    property_variable<int> m_propThriftControlServerPort = 9095;
    property_variable<tBool> m_propEnableThriftControlServer = tFalse;
    property_variable<int> m_propCarId = 26;

    runnable<> m_oRunnable;
    object_ptr<adtf::services::IReferenceClock> m_pClock;
    tTimeStamp tmTimeOfLastActivation;

 protected:
    Map* _map = nullptr;
    mapobjects::Car car_object;
    bool internalMapInitialized;

 private:
    std::string file_extension = ".aismap";
    stdcxx::shared_ptr<TTransport> _thrift_transport_client;
    MapCommClient* _thrift_car_client = NULL;
    bool is_connected_to_thrift_server = false;
    std::string thrift_remote_server = "192.168.168.127";
    int thrift_remote_server_port = 9091;
    int thrift_server_port = 9090;
    std::thread thread_thrift_map_server;
    std::thread thread_thrift_control_server;
    static TSimpleServer* _thrift_car_server;
    static TSimpleServer* _thrift_control_server;
    cPinReader m_oReaderPosition;
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
    cPinReader m_oReaderImage;
    adtf::streaming::tStreamImageFormat m_sImageFormat;


 public:
    tResult Init(adtf::streaming::ant::cFilterBase::tInitStage eStage);
    tResult Start();
    tResult Stop();
    tResult Shutdown(adtf::streaming::ant::cFilterBase::tInitStage eStage);

    tResult RunTrigger(tTimeStamp tmTimeofActivation);
    bool connectToServer();
    bool printMapMessageStats(const map_thrift::MapMessage& mm);
    std::string getMessageOpName(const map_thrift::MessageOp::type& m_type);
    static void startMapServer();
    void stopMapServer();

    static void startControlServer();
    void stopControlServer();

 protected:
    map_thrift::MapMessage returnMap(Map* map);
    map_thrift::CarMessage createCarMessagePoseUpdate(const map_thrift::Pose& c_pose);
    map_thrift::CarMessage createCarMessagePoseDelete();

    map_thrift::Pose createThriftCarPose(const mapobjects::Car& co);
    std::vector<map_thrift::PointViz> createVisualizationPointList();
    void printCarPose(const map_thrift::Pose& cp);
    void sendCarMessage(const map_thrift::CarMessage& cm);
    void sendMapMessage(const map_thrift::MapMessage& mm);
    void sendThriftImage(const map_thrift::ThriftImage& image);

    void writeMapToFile(const map_thrift::MapMessage& map_message);
    void initializeMap();
    bool deleteMap();
    tResult setCarPoseFrom_tPosition();
    void printMapStats(Map* map);
    void writeNewestCostMapImage();
    cv::Mat readImageInputPin();
    map_thrift::ThriftImage createThriftImage(const cv::Mat& image_in);

 public:
    class MapCommHandler : virtual public MapCommIf {
     private:
        ThriftFilter* _thrift_filter;
     public:
        explicit MapCommHandler(ThriftFilter* m_s) : _thrift_filter(m_s) {}
        void ping();

        bool sendMapMessage(const MapMessage& map_complete);
        bool sendCarPose(const CarMessage& car_pose);
        bool sendThriftImage(const ThriftImage& image);
    };

 public:
    class ControlCommHandler : virtual public ControlCommIf {
     private:
        ThriftFilter* _thrift_filter;
     public:
        explicit ControlCommHandler(ThriftFilter* c_s) : _thrift_filter(c_s) {}
        void ping();

        CarStatus::type sendCommand(const ControlMessage& command);
        CarStatus::type getCarStatus();
        double getRouteCosts(const ControlMessage& command);
    };
};
