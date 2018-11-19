
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
 * AUTHOR: Fabien Jenne
 *
 * Thrift filter implementation
 *
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
#include <mutex>                                // NOLINT

// #include <torch/script.h> // One-stop header.
// #include <torch/torch.h>

#include "stdafx.h"
#include "../NNThrift/gen-cpp/NNComm.h"

using namespace ::apache::thrift;               // NOLINT
using namespace ::apache::thrift::protocol;     // NOLINT
using namespace ::apache::thrift::transport;    // NOLINT
using namespace nn_thrift;                      // NOLINT
using namespace cv;                             // NOLINT

#define CID_NN_THRIFT_FILTER "NN.thrift.adtf.cid"
#define LABEL_NN_THRIFT_FILTER "Neural Network Thrift Filter"


class NNThriftFilter : public adtf::filter::ant::cConfigurableFilter {

    // class seg_network {
    //  public:
    //     seg_network();
    //     std::pair<cv::Mat, cv::Mat> infer(torch::Tensor tensor_image);
    //     torch::Tensor preprocess(cv::Mat rgb);
    //  private:
    //     bool _use_cuda;
    //     torch::Device _device;
    //     std::shared_ptr<torch::jit::script::Module> _module;
    //     cv::Scalar _dataset_mean;
    // };

 public:
    NNThriftFilter();

    ~NNThriftFilter();

    ADTF_CLASS_ID_NAME(NNThriftFilter, CID_NN_THRIFT_FILTER, LABEL_NN_THRIFT_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IKernel),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));

 private:
    //  ADTF
    property_variable<tBool> m_propEnableConsoleOutput = tTrue;
    property_variable<bool> launch_nn_server = false;
    property_variable<bool> run_network_locally = false;
    object_ptr<adtf::services::IReferenceClock> m_pClock;
    runnable<> m_oRunnable;
    tTimeStamp tmTimeOfLastActivation;

    // Pins
    cPinReader m_oReaderImage;
    cPinWriter m_oWriterImage;

    // Media description
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    // Thrift
    stdcxx::shared_ptr<TTransport> _thrift_transport_client;  // does not have to be deleted
    NNCommClient* _thrift_nn_client = NULL;
    bool is_connected_to_nn_server = false;
    std::string thrift_nn_server = "127.0.0.1";  // "192.168.168.104";  // 192.168.167.148, 10.8.105.255
    int thrift_nn_server_port = 9093;

    // Media descriptions

public:
    // implement cFilterBase (implements cFilterLevelmachine)
    tResult Init(tInitStage eStage);
    tResult Start();
    tResult Stop();
    tResult Shutdown(tInitStage eStage);
    tResult RunTrigger(tTimeStamp tmTimeofActivation);

    // Thrift client
    bool connectToNNServer();

private:
    // Thrift
    //   Client
    nn_thrift::NNImage getNNOutput(const nn_thrift::NNImage& img);

    // filter
    tResult runNNPipeline();
    tResult readInputPin(cv::Mat* image);
    tResult writeOutputPin(const Mat& outputImage);
    nn_thrift::NNImage createThriftNNImage(const cv::Mat& image_in);
    cv::Mat createCVMatImage(const nn_thrift::NNImage& image_in);
};
