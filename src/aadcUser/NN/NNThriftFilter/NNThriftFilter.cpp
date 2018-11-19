
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
 * AUTHOR: Fabien Jenne
 * 
 * Neural Network Thrift filter implementation
 * 
******************************************************************************/

#include "NNThriftFilter.h"
#include "ADTF3_OpenCV_helper.h"
#include "../../PinClasses/StopWatch.h"

#include <stdlib.h>  // to start PredictionServer
#include <thread>


void launchPredictionServer() {
    LOG_INFO("Launch prediction server.");
    // blocking
    int exit_code =
        system("/home/aadc/AADC/src/aadcUser/NN/PredictionServer/lauch_PredictionServer.sh");
    LOG_INFO("Launch prediction server exited with exit_code %i", exit_code);
    std::cout << "Launch prediction server exited with exit_code " << exit_code << std::endl;
}

// ____________________________________________________________________________
/* ADTF filter config */

ADTF_PLUGIN(LABEL_NN_THRIFT_FILTER, NNThriftFilter);

// ____________________________________________________________________________
/* implement the class */

NNThriftFilter::NNThriftFilter() : \
    m_oRunnable([this](tTimeStamp tmTime) -> tResult {return RunTrigger(tmTime); }) {
    RegisterPropertyVariable("eneableConsoleOutput", m_propEnableConsoleOutput);
    RegisterPropertyVariable("launch_nn_server [bool]", launch_nn_server);
    RegisterPropertyVariable("Run network locally [bool]", run_network_locally);
}

NNThriftFilter::~NNThriftFilter() {
    LOG_INFO("NNThriftFilter: filter destructor called");
    // delete map;
    // delete _thrift_car_server;
    // delete _thrift_nn_client;
}

/* this function is triggered by the Runnable */
tResult NNThriftFilter::RunTrigger(tTimeStamp tmTimeOfActivation) {
    if (!is_connected_to_nn_server) {
        LOG_WARNING("NNThriftFilter: not connected to NN server!");
        is_connected_to_nn_server = connectToNNServer();
        if (!is_connected_to_nn_server) RETURN_NOERROR;
    }
    // LOG_INFO("NNThriftFilter: sending NNImage");

    // vStopWatch watch(true, false);

    tResult status =  runNNPipeline();

    // watch.print_measurement("runNNPipeline");
    return status;
}

// ____________________________________________________________________________
/* implement cFilterLevelmachine */

tResult NNThriftFilter::Init(tInitStage eStage) {
    RETURN_IF_FAILED(cFilterBase::Init(eStage));
    if (eStage == tInitStage::StageFirst) {
        // clock
        _runtime->GetObject(m_pClock);
        if (static_cast<bool>(launch_nn_server)) {
            std::thread launchThread(launchPredictionServer);
            launchThread.detach();
        }
        // register all pins
        //   set stream type
        m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
        const adtf::ucom::object_ptr<IStreamType> pType = \
            adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
        set_stream_type_image_format(*pType, m_sImageFormat);
        //   input
        RETURN_IF_FAILED(create_pin(*this, m_oReaderImage, "image_in", pType));
        //   output
        RETURN_IF_FAILED(create_pin(*this, m_oWriterImage, "image_out", pType));

        //   link input image size to output
        m_oReaderImage.SetAcceptTypeCallback(
            [this](const adtf::ucom::ant::iobject_ptr< \
            const adtf::streaming::ant::IStreamType>& pType) -> tResult {
                return ChangeType(m_oReaderImage, m_sImageFormat, *pType.Get(), m_oWriterImage);
            });


        //   register the runner
        RETURN_IF_FAILED(RegisterRunner("nn_thriftfilter_runner", m_oRunnable));
        //   link the runner to the input pin
        RETURN_IF_FAILED(create_inner_pipe(*this, { "image_in", "nn_thriftfilter_runner" }));

        if (!run_network_locally) {
            // create the Thrift client
            stdcxx::shared_ptr<TTransport> socket(
                new TSocket(thrift_nn_server, thrift_nn_server_port));
            _thrift_transport_client = stdcxx::shared_ptr<TTransport>(
                new TBufferedTransport(socket));
            stdcxx::shared_ptr<TProtocol> protocol(new TBinaryProtocol(_thrift_transport_client));
            _thrift_nn_client = new NNCommClient(protocol);
            LOG_INFO(cString::Format(
                "NNThriftFilter: expecting NNServer on ip: %s | port: %d",
                thrift_nn_server.c_str(), thrift_nn_server_port));
        } else {
            LOG_INFO("NNThriftFilter: running network inline as cpp");
        }
    }

    RETURN_NOERROR;
}
tResult NNThriftFilter::Start() {
    if (!run_network_locally) {
        is_connected_to_nn_server = connectToNNServer();
    }
    RETURN_NOERROR;
}
tResult NNThriftFilter::Stop() {
    if (!run_network_locally) {
        try {
            _thrift_transport_client->close();
            LOG_INFO("NNThriftFilter: connection closed");
            is_connected_to_nn_server = false;
        } catch (TException& tx) {
            LOG_WARNING(cString::Format("NNThriftFilter: ERROR: %s", tx.what()));
        }
    }
    RETURN_NOERROR;
}
tResult NNThriftFilter::Shutdown(tInitStage eStage) {
    LOG_INFO("NNThriftFilter: ShutdownStage");
    RETURN_NOERROR;
}

// ____________________________________________________________________________
/* NNThriftFilter: functions */

/* function which implements the whole filter loop */
tResult NNThriftFilter::runNNPipeline() {
    cv::Mat image_input = cv::Mat();
    tResult status = readInputPin(&image_input);
    if (!IS_OK(status)) {
        // LOG_INFO("Could not read input pin.");
        RETURN_ERROR(status);
    }


    // get the network output
    cv::Mat image_output;
    if (!run_network_locally) {
        // LOG_INFO("createThriftImage");
        nn_thrift::NNImage image_in_thrift = createThriftNNImage(image_input);
        // LOG_INFO("getNNOutput");
        nn_thrift::NNImage image_out_thrift = getNNOutput(image_in_thrift);
        // LOG_INFO("createCVMat");
        image_output = createCVMatImage(image_out_thrift);
    } else {
        LOG_INFO("running network locally");
    }


    status = writeOutputPin(image_output);
    if (!IS_OK(status)) {
        // LOG_INFO("Could not write to output pin.");
        RETURN_ERROR(status);
    }
    RETURN_NOERROR;
}

/* function to read input images from the streaming graph */
tResult NNThriftFilter::readInputPin(cv::Mat* image) {
    object_ptr<const ISample> pReadSample;
    bool found = false;
    while (IS_OK(m_oReaderImage.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        // lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            // create an opencv matrix from the media sample buffer
            // LOG_INFO("m_sImageFormat.m_ui32Width=%i", m_sImageFormat.m_ui32Width);
            // LOG_INFO("m_sImageFormat.m_ui32Height=%i", m_sImageFormat.m_ui32Height);

            *image = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                 CV_8UC3,
                                 const_cast<unsigned char*>(
                                     static_cast<const unsigned char*>(pReadBuffer->GetPtr())));
            found = true;
        }
    }
    if (!found) {
        RETURN_ERROR(ERR_IO_PENDING);
    } else {
        RETURN_NOERROR;
    }
}

/* function to write output images to the streaming graph */
tResult NNThriftFilter::writeOutputPin(const Mat& outputImage) {
    if (!outputImage.empty()) {
        // update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize()
                                    != m_sImageFormat.m_szMaxByteSize) {
            setTypeFromMat(m_oWriterImage, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriterImage, outputImage, m_pClock->GetStreamTime());
    }
    RETURN_NOERROR;
}

// ____________________________________________________________________________
/* NNThriftFilter: Thrift client functions */

/* function to request a NN prediction on the NN server */
nn_thrift::NNImage NNThriftFilter::getNNOutput(const nn_thrift::NNImage& image_input) {
    nn_thrift::NNImage prediction = nn_thrift::NNImage();
    try {
        _thrift_nn_client->getNNPrediction(prediction, image_input);
    } catch (TException& tx) {
        LOG_WARNING(cString::Format("NNThriftFilter: ERROR: %s", tx.what()));
    }
    return prediction;
}

/* function to create a nn_thrift::NNImage from cv::Mat */
nn_thrift::NNImage NNThriftFilter::createThriftNNImage(const cv::Mat& image_in) {
    CV_Assert(image_in.depth() == CV_8U);
    int rows = image_in.rows;
    int cols = image_in.cols;
    int n_channels = image_in.channels();

    nn_thrift::NNImage image_thrift = nn_thrift::NNImage();
    image_thrift.width = cols;
    image_thrift.height = rows;
    image_thrift.channels = n_channels;

    std::string image_bytes = std::string();
    if (n_channels == 1) {
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                image_bytes += (image_in.at<char>(y, x));
            }
        }
    } else if (n_channels > 1) {
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                for (int c = 0; c < n_channels; c++) {
                    image_bytes += static_cast<char>(image_in.at<Vec3b>(y, x).val[c]);
                }
            }
        }
    } else {
        LOG_ERROR("NNThriftFilter: number of thrift channels 0 or negative.");
    }
    LOG_DUMP(cString::Format(
        "input size h: %d | w: %d", image_in.size().height, image_in.size().width));
    image_thrift.bytes = image_bytes;

    return image_thrift;
}

/* function to create a cv::Mat from nn_thrift::NNImage */
cv::Mat NNThriftFilter::createCVMatImage(const nn_thrift::NNImage& image_in) {
    cv::Mat image_cv = cv::Mat(cv::Size(image_in.width, image_in.height), CV_8UC3);
    const std::string& image_bytes = image_in.bytes;
    if (image_in.channels == 1) {
        for (int y = 0; y < image_in.height; y++) {
            for (int x = 0; x < image_in.width; x++) {
                image_cv.at<unsigned char>(y, x) = \
                    static_cast<unsigned char>(image_bytes[(y * image_in.width) + x]);
            }
        }
    } else if (image_in.channels == 2) {
        for (int y = 0; y < image_in.height; y++) {
            for (int x = 0; x < image_in.width; x++) {
                image_cv.at<Vec3b>(y, x).val[0] = static_cast<unsigned char>(
                    image_bytes[(y * image_in.width * (image_in.channels)) + (x * (image_in.channels)) + 0]);
                unsigned char zero = 0;
                image_cv.at<Vec3b>(y, x).val[1] = zero;
                image_cv.at<Vec3b>(y, x).val[2] = static_cast<unsigned char>(
                    image_bytes[(y * image_in.width * (image_in.channels)) + (x * (image_in.channels)) + 1]);
            }
        }
    } else if (image_in.channels == 3) {
        for (int y = 0; y < image_in.height; y++) {
            for (int x = 0; x < image_in.width; x++) {
                for (int c = 0; c < image_in.channels; c++) {
                    image_cv.at<Vec3b>(y, x).val[c] = \
                        static_cast<unsigned char>(
                            image_bytes[(y * image_in.width * image_in.channels) \
                                + (x * image_in.channels) + c]);
                }
            }
        }
    }
    else {
        LOG_ERROR("NNThriftFilter: number of thrift channels 0 or negative or greater 3.");
    }
    LOG_DUMP(cString::Format(
        "thrift size h: %d | w: %d", image_in.height, image_in.width));
    LOG_DUMP(cString::Format(
        "output size h: %d | w: %d", image_cv.size().height, image_cv.size().width));
    return image_cv;
}
// ____________________________________________________________________________
/* Thrift maintanance */

/* function to check if NNThriftFilter is connected to the remote server */
bool NNThriftFilter::connectToNNServer() {
    if (!is_connected_to_nn_server) {
        try {
            _thrift_transport_client->open();
            LOG_INFO(cString::Format(
                "NNThriftFilter: connecting to NN server on ip: %s | port: %d",
                thrift_nn_server.c_str(), thrift_nn_server_port));
            bool is_connected = _thrift_nn_client->ping();
            is_connected_to_nn_server = is_connected;
            LOG_INFO("NNThriftFilter: connected!");
        } catch (TException& tx) {
            LOG_WARNING(cString::Format("NNThriftFilter: ERROR: %s", tx.what()));
            is_connected_to_nn_server = false;
        }
    } else {
        try {
            bool is_connected = _thrift_nn_client->ping();
            is_connected_to_nn_server = is_connected;
            LOG_INFO("NNThriftFilter: ping successful!");
        } catch (TException& tx) {
            LOG_WARNING(cString::Format("NNThriftFilter: ERROR: %s", tx.what()));
            is_connected_to_nn_server = false;
        }
    }
    return is_connected_to_nn_server;
}


// NNThriftFilter::seg_network::seg_network():_use_cuda(false), _device(torch::kCPU)
// {
//     std::string model_path = "/home/vertensj/code/raiscar_map/segnet/cpp_segnet_model.pt";

//     torch::DeviceType device_type;
//     if (torch::cuda::is_available()) {
//         std::cout << "CUDA available! Inference on GPU" << std::endl;
//         device_type = torch::kCUDA;
//         _use_cuda = true;
//     } else {
//         std::cout << "Inference on CPU" << std::endl;
//         device_type = torch::kCPU;
//     }
//     //_device.set_index(device_type);
//     _device = torch::Device(device_type);


//     // Deserialize the ScriptModule from a file using torch::jit::load().
//     _module = torch::jit::load(model_path);
//     if (_use_cuda)
//         _module->to(_device);

//     assert(_module != nullptr);
//     std::cout << "loaded network\n";
//     float _dataset_mean[3];
//     _dataset_mean[0] = 123.68173826f/255.0f;
//     _dataset_mean[0] = 124.31097714f/255.0f;
//     _dataset_mean[0] = 119.26626476f/255.0f;
// }

// torch::Tensor NNThriftFilter::seg_network::preprocess(cv::Mat rgb) {
//     cv::resize(rgb, rgb, cv::Size(640, 88));
//     // cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
//     unsigned char tensor_mem[1*3*640*88];
//     int width = rgb.cols;
//     int height = rgb.rows;
//     int step = rgb.step;
//     int total_channel_num = width*height;
//     for (int y=0; y < height; y++) {
//         for (int x=0; x < width; x++) {
//             tensor_mem[0*total_channel_num + y* width+ x] =
//                 rgb.data[y*step+x*3+0] - _dataset_mean[0];
//             tensor_mem[1*total_channel_num + y* width+ x] =
//                 rgb.data[y*step+x*3+1] - _dataset_mean[1];
//             tensor_mem[2*total_channel_num + y* width+ x] =
//                 rgb.data[y*step+x*3+2] - _dataset_mean[2];
//         }
//     }

//     at::Tensor tensor_image = torch::from_blob(
//         tensor_mem, {1, 3, rgb.rows, rgb.cols}, at::kByte).to(at::kFloat) / 255.0;

//     return tensor_image;
// }

// std::pair<cv::Mat, cv::Mat> NNThriftFilter::seg_network::infer(torch::Tensor tensor_image) {
//     std::vector<torch::jit::IValue> inputs;
//     if (_use_cuda) {
//         inputs.push_back(tensor_image.to(_device));
//     } else {
//         inputs.push_back(tensor_image.to(_device));
//     }

//     auto outputs = _module->forward(inputs).toTuple();
//     auto segmentation = outputs->elements()[0].toTensor();
//     auto regression = outputs->elements()[1].toTensor();

//     //std::cout << "Segmentation size: " << segmentation.size(0) << "/" << segmentation.size(1)
//         // << "/" << segmentation.size(2) << "/" << segmentation.size(3) << std::endl;
//     //std::cout << "Regression size: " << regression.size(0) << "/" << regression.size(1) 
//         // << "/" << regression.size(2) << "/" << regression.size(3) << std::endl;

//     cv::Size sizes;
//     sizes.height = 88;
//     sizes.width = 640;

//     cv::Mat output_reg(sizes, CV_32FC1, regression.cpu().data<float>());
//     output_reg.convertTo(output_reg, CV_8UC1);

//     at::Tensor seg_argmax = at::argmax(segmentation, 1).to(at::kByte);
//     cv::Mat seg_argmax_cv(sizes, CV_8UC1, seg_argmax.cpu().data<unsigned char>());

//     //    cv::Mat img_color;
//     //    cv::applyColorMap(output_reg, img_color, cv::COLORMAP_JET);

//     inputs.clear();

//     return std::pair<cv::Mat, cv::Mat>(output_reg.clone(), seg_argmax_cv.clone());
// }
