
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#pragma once
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>
#include <thrift/stdcxx.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <utility>
#include <mutex>                           
#include "stdafx.h"
#include "../NNThrift/gen-cpp/NNComm.h"

using namespace ::apache::thrift;             
using namespace ::apache::thrift::protocol;   
using namespace ::apache::thrift::transport; 
using namespace nn_thrift;                    
using namespace cv;                      

#define CID_NN_THRIFT_FILTER "NN.thrift.adtf.cid"
#define LABEL_NN_THRIFT_FILTER "Neural Network Thrift Filter"


class NNThriftFilter : public adtf::filter::ant::cConfigurableFilter {
 public:
    NNThriftFilter();

    ~NNThriftFilter();

    ADTF_CLASS_ID_NAME(NNThriftFilter, CID_NN_THRIFT_FILTER, LABEL_NN_THRIFT_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IKernel),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));

 private:
    property_variable<tBool> m_propEnableConsoleOutput = tTrue;
    property_variable<bool> launch_nn_server = false;
    property_variable<bool> run_network_locally = false;
    object_ptr<adtf::services::IReferenceClock> m_pClock;
    runnable<> m_oRunnable;
    tTimeStamp tmTimeOfLastActivation;
    cPinReader m_oReaderImage;
    cPinWriter m_oWriterImage;
    adtf::streaming::tStreamImageFormat m_sImageFormat;
    stdcxx::shared_ptr<TTransport> _thrift_transport_client;
    NNCommClient* _thrift_nn_client = NULL;
    bool is_connected_to_nn_server = false;
    std::string thrift_nn_server = "127.0.0.1";
    int thrift_nn_server_port = 9093;
public:
    tResult Init(tInitStage eStage);
    tResult Start();
    tResult Stop();
    tResult Shutdown(tInitStage eStage);
    tResult RunTrigger(tTimeStamp tmTimeofActivation);
    bool connectToNNServer();

private:
    nn_thrift::NNImage getNNOutput(const nn_thrift::NNImage& img);
    tResult runNNPipeline();
    tResult readInputPin(cv::Mat* image);
    tResult writeOutputPin(const Mat& outputImage);
    nn_thrift::NNImage createThriftNNImage(const cv::Mat& image_in);
    cv::Mat createCVMatImage(const nn_thrift::NNImage& image_in);
};
