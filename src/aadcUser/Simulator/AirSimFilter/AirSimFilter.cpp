
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#include "AirSimFilter.h"
#include "ADTF3_OpenCV_helper.h"
ADTF_PLUGIN(LABEL_AIRSIM_FILTER, AirSimFilter);

AirSimFilter::AirSimFilter() {
    RegisterPropertyVariable("Limit position update rate", m_propLimitPositionRefreshRate);
    RegisterPropertyVariable("Max position update rate, Hz", m_propPositionRefreshRate);
    RegisterPropertyVariable("Use metric speed input", m_propUseMetricInput);
    RegisterPropertyVariable("Airsim server IP", m_propAirsimServerIp);
    RegisterPropertyVariable("Use compressed image transfer", m_propUseCompressedImageTransfer);
    RegisterPropertyVariable("Airsim 'ImageReqest' camera id", m_propAirsimImageId);
    RegisterPropertyVariable("Enable image fps logging", m_propEnableFPSLogging);
    RegisterPropertyVariable("Motor command logging", m_propEnableControlLogging);
    RegisterPropertyVariable("Position logging", m_propEnablePositionLogging);

    image_thread_running = false;

    control_value_speed = 0;
    control_value_steering = 0;
}

AirSimFilter::~AirSimFilter() {
    LOG_INFO("AirSimFilter: filter destructor called");
}

/* this function is triggered by the Runnable */
tResult AirSimFilter::RunTrigger(tTimeStamp tmTimeOfActivation) {
    if (airsim_client_connected) {
        readControlInputPins();
        sendControlsToAirSim(control_value_speed, control_value_steering);
        if (!image_thread_running.load()) {
            try {
                image_thread_running.store(true);
                airsim_image_thread = std::thread(
                    &AirSimFilter::writeNewestAirSimImage, this, tmTimeOfActivation);
                airsim_image_thread.detach();
            } catch (std::future_error& e) {
                LOG_WARNING("AirSimFilter: caught future_error. trying to proceed");
                image_thread_running.store(false);
            }
        }
        if (m_propLimitPositionRefreshRate) {
            if ((tmTimeOfActivation - tmTimeOfLastPositionUpdate) > \
                static_cast<tTimeStamp>(1e6 / m_propPositionRefreshRate)) {
                writeNewestAirSimPosition();
                // reset the timestamp
                tmTimeOfLastPositionUpdate = m_pClock->GetStreamTime();
            }
        } else {
            writeNewestAirSimPosition();
        }
    }
    RETURN_NOERROR;
}
tResult AirSimFilter::Init(adtf::streaming::ant::cFilterBase::tInitStage eStage) {
    RETURN_IF_FAILED(cFilterBase::Init(eStage));
    if (eStage == adtf::streaming::ant::cFilterBase::tInitStage::StageFirst) {
        // clock
        _runtime->GetObject(m_pClock);
        object_ptr<IStreamType> pTypeSignalData;
        if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
            "tSignalValue", pTypeSignalData, m_SignalValueSampleFactory)) {
            adtf_ddl::access_element::find_index(
                m_SignalValueSampleFactory, "ui32ArduinoTimestamp", m_ddlSignalValueId.timeStamp);
            adtf_ddl::access_element::find_index(
                m_SignalValueSampleFactory, "f32Value", m_ddlSignalValueId.value);
        }
        RETURN_IF_FAILED(create_pin(*this, m_oReaderSpeed, "speed_in", pTypeSignalData));
        RETURN_IF_FAILED(create_pin(*this, m_oReaderSteering, "steering_in", pTypeSignalData));
        object_ptr<IStreamType> pTypePositionData;
        if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
            "tPosition", pTypePositionData, m_PositionSampleFactory)) {
            adtf_ddl::access_element::find_index(m_PositionSampleFactory,
                cString("f32x"), m_ddlPositionId.x);
            adtf_ddl::access_element::find_index(m_PositionSampleFactory,
                cString("f32y"), m_ddlPositionId.y);
            adtf_ddl::access_element::find_index(m_PositionSampleFactory,
                cString("f32radius"), m_ddlPositionId.radius);
            adtf_ddl::access_element::find_index(m_PositionSampleFactory,
                cString("f32speed"), m_ddlPositionId.speed);
            adtf_ddl::access_element::find_index(m_PositionSampleFactory,
                cString("f32heading"), m_ddlPositionId.heading);
            LOG_DUMP("AirSimFilter: Mediadescription for tPosition found!");
        } else {
            LOG_INFO("No mediadescription for tPosition found!");
        }
        RETURN_IF_FAILED(create_pin(*this, m_oWriterPosition, "position_out", pTypePositionData));
        m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
        const adtf::ucom::object_ptr<IStreamType> pType = \
            adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
        set_stream_type_image_format(*pType, m_sImageFormat);
        RETURN_IF_FAILED(create_pin(*this, m_oWriterImage, "image_out", pType));
        adtf::ucom::object_ptr<adtf::streaming::IRunner> timerRunner = \
            adtf::ucom::make_object_ptr<adtf::streaming::cRunner>(
                "airsimfilter_runner", [&](tTimeStamp tmTime) -> tResult {
                    return RunTrigger(tmTime);
                });
        RETURN_IF_FAILED(RegisterRunner(timerRunner));

    } else if (eStage == adtf::streaming::ant::cFilterBase::tInitStage::StagePreConnect) {
        airsim_client_connected = createAirSimClient();
        getAirSimVehiclePose();
    }

    RETURN_NOERROR;
}
tResult AirSimFilter::Start() {
    tmTimeOfLastActivation = m_pClock->GetStreamTime();
    tmTimeOfLastPositionUpdate = tmTimeOfLastActivation;
    tmTimeOfLastImage.store(tmTimeOfLastActivation);
    RETURN_NOERROR;
}
tResult AirSimFilter::Stop() {
    RETURN_NOERROR;
}
tResult AirSimFilter::Shutdown(adtf::streaming::ant::cFilterBase::tInitStage eStage) {
    if (_airsim_client) {
        _airsim_client = NULL;
        LOG_INFO("AirSimFilter: deleted AirSim client object");
    }
    LOG_INFO("AirSimFilter: ShutdownStage");
    RETURN_NOERROR;
}
bool AirSimFilter::createAirSimClient() {
    _airsim_client = new msr::airlib::CarRpcLibClient(
        std::string(static_cast<cString>(m_propAirsimServerIp)),
        airsim_server_port,
        airsim_connection_timeout);
    LOG_INFO(cString::Format(
        "AirSimClient: Trying to connect to AirSim server at %s on port %d",
        airsim_server_ip.c_str(), airsim_server_port));

    try {
        _airsim_client->confirmConnection();
        LOG_INFO("AirSimClient: connected to AirSim server!");
        return true;
    } catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." <<
            std::endl << msg << std::endl; std::cin.get();  // NOLINT
        LOG_WARNING("AirSimClient: Connection to AirSim server failed");
    }
    return false;
}
cv::Mat AirSimFilter::getAirSimImages() {
    cv::Mat img;
    try {
        vector<ImageRequest> request = {
            ImageRequest(
                std::string(static_cast<cString>(m_propAirsimImageId)),
                ImageType::Scene,
                false,
                m_propUseCompressedImageTransfer)
        };
        const vector<ImageResponse>& response = _airsim_client->simGetImages(request);
        if (m_propUseCompressedImageTransfer) {
            img = cv::imdecode(Mat(response[0].image_data_uint8), CV_LOAD_IMAGE_COLOR);
            cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
        } else {
            img = cv::Mat(response[0].height, response[0].width, CV_8UC3);
            for (int y = 0; y < response[0].height; y++) {
                for (int x = 0; x < response[0].width; x++) {
                    // rgba
                    img.at<Vec3b>(y, x) = cv::Vec3b(
                        response[0].image_data_uint8[y*response[0].width*4 + x*4 + 0],
                        response[0].image_data_uint8[y*response[0].width*4 + x*4 + 1],
                        response[0].image_data_uint8[y*response[0].width*4 + x*4 + 2]);
                }
            }
        }
        return img;
    } catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." <<
            std::endl << msg << std::endl; std::cin.get();  // NOLINT
        LOG_WARNING("AirSimClient: Couldn't get image from AirSim server!");
        airsim_client_connected = false;
    }
    return img;
}
std::vector<double> AirSimFilter::getAirSimVehiclePose() {
    std::vector<double> car_pose = std::vector<double>(5, -1.0);
    try {
        auto car_state = _airsim_client->getCarState();
        auto pose = car_state.kinematics_estimated.pose;
        car_pose[0] = pose.position[0];
        car_pose[1] = -pose.position[1];
        car_pose[2] = pose.position[2];
        car_pose[3] = -yawFromQuaternion(pose.orientation);
        car_pose[4] = car_state.speed;
    } catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." <<
            std::endl << msg << std::endl; std::cin.get();  // NOLINT
        LOG_WARNING("AirSimClient: Couldn't get pose from AirSim server!");
        airsim_client_connected = false;
    }
    if (m_propEnablePositionLogging) {
        LOG_INFO(cString::Format("airsim pose: %f %f %f %f %f", \
            car_pose[0], car_pose[1], car_pose[2], car_pose[3], car_pose[4]));
    }
    return car_pose;
}
double AirSimFilter::yawFromQuaternion(const auto& q) {
    return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
bool AirSimFilter::sendControlsToAirSim(double speed, double steering) {
    try {
        _airsim_client->enableApiControl(true);
        CarApiBase::CarControls controls;
        controls.throttle = speed / 100;
        controls.steering = steering / 100;
        if (speed < 0) {
            controls.manual_gear = -1;
            controls.is_manual_gear = true;
        } else {
            controls.manual_gear = 0;
            controls.is_manual_gear = false;
        }
        if (m_propEnableControlLogging) {
            if (speed < 0) std::cout << "REVERSING" << std::endl;
            std::cout << cString::Format("car controls: %f %f | airsim controls %f %f",
            speed, steering, controls.throttle, controls.steering) << std::endl;
        }

        _airsim_client->setCarControls(controls);
        return true;
    } catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." <<
            std::endl << msg << std::endl; std::cin.get();  // NOLINT
        LOG_WARNING("AirSimClient: Couldn't send control commands to AirSim server");
        airsim_client_connected = false;
    }
    return false;
}
void AirSimFilter::writeNewestAirSimImage(tTimeStamp tmTimeOfImageUpdate) {
    try {
        cv::Mat image = getAirSimImages();
        if (!IS_OK(writeImageToPin(image))) {
            image_thread_running.store(false);
            LOG_WARNING("AirSimClient: error writing image to pin.");
            return;
        }
        if (m_propEnableFPSLogging) {
            float fps = 1 / (static_cast<float>(
                tmTimeOfImageUpdate - tmTimeOfLastImage.load()) * 1e-6);
            LOG_INFO(cString::Format("airsim image fps: %f", fps));
        }
        tmTimeOfLastImage.store(tmTimeOfImageUpdate);
        image_thread_running.store(false);
        return;
    } catch (std::future_error& e) {
                LOG_WARNING("AirSimFilter: caught future_error. trying to proceed");
                image_thread_running.store(false);
    }
}
tResult AirSimFilter::writeNewestAirSimPosition() {
    std::vector<double> simulator_pose = getAirSimVehiclePose();
    return sendPositionStruct(
        m_pClock->GetStreamTime(),
        simulator_pose[0],
        simulator_pose[1],
        1.0,
        simulator_pose[3],
        simulator_pose[4]);
}
tResult AirSimFilter::writeImageToPin(const Mat& outputImage) {
    if (!outputImage.empty()) {
        if (outputImage.total() * outputImage.elemSize()
                                    != m_sImageFormat.m_szMaxByteSize) {
            setTypeFromMat(m_oWriterImage, outputImage);
        }
        writeMatToPin(m_oWriterImage, outputImage, m_pClock->GetStreamTime());
    }
    RETURN_NOERROR;
}
tResult AirSimFilter::sendPositionStruct(
    const tTimeStamp &timeOfWriting, const tFloat32 &f32X, const tFloat32 &f32Y,
    const tFloat32 &f32Radius, const tFloat32 &f32Heading, const tFloat32 &f32Speed) {
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, timeOfWriting));

    auto oCodec = m_PositionSampleFactory.MakeCodecFor(pSample);

    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionId.x, f32X));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionId.y, f32Y));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionId.radius, f32Radius));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionId.speed, f32Speed));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPositionId.heading, f32Heading));

    if (m_propEnablePositionLogging) {
        LOG_INFO(cString::Format("tPosition struct: %f %f %f %f %f", \
            f32X, f32Y, f32Radius, f32Speed, f32Heading));
    }

    m_oWriterPosition << pSample << flush << trigger;
    RETURN_NOERROR;
}
tResult AirSimFilter::readControlInputPins() {
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oReaderSpeed.GetLastSample(pReadSample))) {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);
        control_value_speed = static_cast<double>(adtf_ddl::access_element::get_value(
            oDecoder, m_ddlSignalValueId.value));
        if (m_propUseMetricInput) {
            control_value_speed = (control_value_speed / 16) * 100;
        }
    }
    if (IS_OK(m_oReaderSteering.GetLastSample(pReadSample))) {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);
        control_value_steering = static_cast<double>(adtf_ddl::access_element::get_value(
            oDecoder, m_ddlSignalValueId.value));
    }
    RETURN_NOERROR;
}
