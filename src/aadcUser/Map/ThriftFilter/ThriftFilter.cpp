
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#include "ThriftFilter.h"
#include "ADTF3_OpenCV_helper.h"


ADTF_PLUGIN(LABEL_THRIFT_FILTER, ThriftFilter);

TSimpleServer* ThriftFilter::_thrift_car_server = NULL;
TSimpleServer* ThriftFilter::_thrift_control_server = NULL;
std::mutex map_access_mutex;

ThriftFilter::ThriftFilter() : \
    m_oRunnable([this](tTimeStamp tmTime) -> tResult {return RunTrigger(tmTime); }) {
    internalMapInitialized = false;

    RegisterPropertyVariable("Send position updates to remote server", m_propSendPositionUpdates);
    RegisterPropertyVariable("Position update rate, Hz", m_propPositionUpdateInterval);
    RegisterPropertyVariable("Internal Thrift map server port", m_propThriftServerPort);
    RegisterPropertyVariable("Internal Thrift control server port", m_propThriftControlServerPort);
    RegisterPropertyVariable("Enable Thrift control server", m_propEnableThriftControlServer);
    RegisterPropertyVariable("Load initial map from file", m_propLoadMapFromFile);
    RegisterPropertyVariable("Save incoming map to file", m_propSaveMapFromServer);
    RegisterPropertyVariable("Map absolute file path (without extension)", m_propMapFilePath);
    RegisterPropertyVariable("Incoming map updates via Thrift", m_propUpdateMapFromThrift);
    RegisterPropertyVariable("Remote Thrift server ip", m_propThriftRemoteServerIp);
    RegisterPropertyVariable("Remote Thrift server port", m_propThriftRemoteServerPort);
    RegisterPropertyVariable("Send map updates to remote server", m_propSendMapUpdates);
    RegisterPropertyVariable("Position logging", m_propEnablePositionLogging);
    RegisterPropertyVariable("Map logging", m_propEnableMapLogging);
    RegisterPropertyVariable("Send out costmaps to remote server", m_propSendOutImages);
    RegisterPropertyVariable("Disable all thrift communication (override)",
        m_propDisableThriftCommunication);
    RegisterPropertyVariable("Send out costmaps to remote server", m_propSendOutImages);
    RegisterPropertyVariable("Car Id:", m_propCarId);
}

ThriftFilter::~ThriftFilter() {
    LOG_INFO("ThriftFilter: filter destructor called");
}

tResult ThriftFilter::RunTrigger(tTimeStamp tmTimeOfActivation) {
    if (m_propSendPositionUpdates && !m_propDisableThriftCommunication) {
        if ((tmTimeOfActivation - tmTimeOfLastActivation) > \
            1.0 / (static_cast<tFloat32>(m_propPositionUpdateInterval) * 1e6)) {
            setCarPoseFrom_tPosition();
            map_thrift::CarMessage c_m = createCarMessagePoseUpdate(
                createThriftCarPose(car_object));
            if (m_propEnablePositionLogging) printCarPose(c_m.pose);
            sendCarMessage(c_m);
            tmTimeOfLastActivation = m_pClock->GetStreamTime();
        }
    }
    if (m_propSendMapUpdates && !m_propDisableThriftCommunication) {
        _map = Map::getInstance();
        MapStatus map_status = _map->getMapStatus();
        if (map_status == MapStatus::LOADED_FROM_FILE ||
            map_status == MapStatus::UPDATED_FROM_REMOTE) {
            if (_map->isMapChanged()) {
                std::cout << \
                    "ThriftClient: sending internal map update to remote thrift server" \
                    << std::endl;
                map_thrift::MapMessage m_m = returnMap(_map);
                if (m_propEnableMapLogging) printMapMessageStats(m_m);
                sendMapMessage(m_m);
                _map->setMapUpdated();
            }
        }
    }
    if (m_propSendOutImages && !m_propDisableThriftCommunication) {
        writeNewestCostMapImage();
    }
    RETURN_NOERROR;
}
tResult ThriftFilter::Init(adtf::streaming::ant::cFilterBase::tInitStage eStage) {
    RETURN_IF_FAILED(cFilterBase::Init(eStage));
    if (eStage == adtf::streaming::ant::cFilterBase::tInitStage::StageFirst) {
        RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
        tmTimeOfLastActivation = m_pClock->GetStreamTime();
        adtf::ucom::object_ptr<adtf::streaming::IRunner> timerRunner = \
            adtf::ucom::make_object_ptr<adtf::streaming::cRunner>(
                "thriftfilter_runner", [&](tTimeStamp tmTime) -> tResult {
                    return RunTrigger(tmTime);
                });
        RETURN_IF_FAILED(RegisterRunner(timerRunner));
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
            LOG_INFO("ThriftFilter: Mediadescription for tPosition found!");
        } else {
            LOG_INFO("No mediadescription for tPosition found!");
        }
        RETURN_IF_FAILED(create_pin(*this, m_oReaderPosition, "position_in", pTypePositionData));
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
        m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
        const adtf::ucom::object_ptr<IStreamType> pType = \
            adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
        set_stream_type_image_format(*pType, m_sImageFormat);
        RETURN_IF_FAILED(create_pin(*this, m_oReaderImage, "costmap_image_in", pType));
        RETURN_IF_FAILED(create_pin(*this, m_oWriterImage, "image_out", pType));

    } else if (eStage == adtf::streaming::ant::cFilterBase::tInitStage::StagePreConnect) {
        if (!m_propDisableThriftCommunication) {
            // create the Thrift map server
            ::apache::thrift::stdcxx::shared_ptr<MapCommHandler> handler(
                new MapCommHandler(this));
            ::apache::thrift::stdcxx::shared_ptr<TProcessor> processor(
                new MapCommProcessor(handler));
            ::apache::thrift::stdcxx::shared_ptr<TServerTransport> serverTransport(
                new TServerSocket(static_cast<int>(m_propThriftServerPort)));
            ::apache::thrift::stdcxx::shared_ptr<TTransportFactory> transportFactory(
                new TBufferedTransportFactory());
            ::apache::thrift::stdcxx::shared_ptr<TProtocolFactory> protocolFactory(
                new TBinaryProtocolFactory());
            _thrift_car_server = new TSimpleServer(
                processor, serverTransport, transportFactory, protocolFactory);
            LOG_INFO("ThriftFilter: created Thrift MapServer on port: %d",
                static_cast<int>(m_propThriftServerPort));
            connectToServer();

            if (m_propEnableThriftControlServer) {
                ::apache::thrift::stdcxx::shared_ptr<ControlCommHandler> handler(
                    new ControlCommHandler(this));
                ::apache::thrift::stdcxx::shared_ptr<TProcessor> processor(
                    new ControlCommProcessor(handler));
                ::apache::thrift::stdcxx::shared_ptr<TServerTransport> serverTransport(
                    new TServerSocket(static_cast<int>(m_propThriftControlServerPort)));
                ::apache::thrift::stdcxx::shared_ptr<TTransportFactory> transportFactory(
                    new TBufferedTransportFactory());
                ::apache::thrift::stdcxx::shared_ptr<TProtocolFactory> protocolFactory(
                    new TBinaryProtocolFactory());
                _thrift_control_server = new TSimpleServer(
                    processor, serverTransport, transportFactory, protocolFactory);
                LOG_INFO("ThriftFilter: created Thrift ControlServer on port: %d",
                    static_cast<int>(m_propThriftControlServerPort));
            }
        }
        ThriftFilter::initializeMap();
        car_object = mapobjects::Car();
        if (m_propLoadMapFromFile) {
            _map = Map::getInstance();
            std::string filename = std::string(static_cast<cString>(m_propMapFilePath));
            filename += file_extension;
            LOG_INFO(cString::Format(
                "ThriftFilter: loading initial map from file. path: %s", filename.c_str()));
            ::apache::thrift::stdcxx::shared_ptr<TFileTransport> transport(
                new TFileTransport(filename));
            ::apache::thrift::stdcxx::shared_ptr<TProtocol> protocol(
                new TBinaryProtocol(transport));

            map_thrift::MapMessage map_file;
            map_file.read(protocol.get());

            _map->loadMapFromFile(map_file);
        }
    }

    RETURN_NOERROR;
}
tResult ThriftFilter::Start() {
    if (!m_propDisableThriftCommunication) {
        std::ostringstream ss;
        ss << std::this_thread::get_id();
        std::string idstr = ss.str();
        LOG_INFO(cString::Format("ThriftFilter: filter thread: %s", idstr.c_str()));

        thread_thrift_map_server = std::thread(startMapServer);
        LOG_INFO("ThriftFilter: map server serving...");

        thread_thrift_control_server = std::thread(startControlServer);
        LOG_INFO("ThriftFilter: control server serving...");
    }
    RETURN_NOERROR;
}
tResult ThriftFilter::Stop() {
    if (!m_propDisableThriftCommunication) {
        stopMapServer();

        if (m_propEnableThriftControlServer) {
            stopControlServer();
        }
    }
    RETURN_NOERROR;
}
tResult ThriftFilter::Shutdown(adtf::streaming::ant::cFilterBase::tInitStage eStage) {
    if (_thrift_car_server) {
        _thrift_car_server = nullptr;
        LOG_INFO("ThriftFilter: deleted Thrift map server object");
    }
    if (_thrift_control_server) {
        _thrift_control_server = nullptr;
        LOG_INFO("ThriftFilter: deleted Thrift control server object");
    }
    if (_map) {
        _map->deleteInternalMap();
        _map = nullptr;
        LOG_INFO("ThriftFilter: deleted the internal map");
    }
    LOG_INFO("ThriftFilter: ShutdownStage");
    RETURN_NOERROR;
}
map_thrift::CarMessage ThriftFilter::createCarMessagePoseUpdate(
    const map_thrift::Pose& c_pose) {
    map_thrift::CarMessage cm = map_thrift::CarMessage();
    cm.op = map_thrift::MessageOp::type::POSE_UPDATE;
    cm.pose = c_pose;
    cm.points_viz = _map->getVisualizationPoints();
    cm.n_points_viz = cm.points_viz.size();
    return cm;
}
map_thrift::CarMessage ThriftFilter::createCarMessagePoseDelete() {
    map_thrift::CarMessage cm = map_thrift::CarMessage();
    cm.op = map_thrift::MessageOp::type::POSE_DELETE;
    map_thrift::Pose c_pose = map_thrift::Pose();
    c_pose.position.x = 0.0;
    c_pose.position.y = 0.0;
    c_pose.position.z = 0.0;
    c_pose.orientation = 0.0;
    cm.pose = c_pose;
    return cm;
}
map_thrift::MapMessage ThriftFilter::returnMap(Map* map) {
    return _map->createMapMessageAddAll();
}
bool ThriftFilter::printMapMessageStats(const map_thrift::MapMessage& mm) {
    try {
        int n = mm.container.map_parts.size();
        LOG_INFO(cString::Format("MapMessageStats: Type: %d | MapParts: %d",
           static_cast<int>(mm.op), n));
        for (int i = 0; i < n; ++i) {
            LOG_INFO(cString::Format(
                "MapMessageStats: MapPart: %d | Lanes: %d | LaneGroups: %d | LaneMarkings: %d\n",
                i,
                mm.container.map_parts[i].lanes.size(),
                mm.container.map_parts[i].lane_groups.size(),
                mm.container.map_parts[i].lane_markings.size()));
        }
        return true;
    } catch (...) {
        LOG_WARNING("MapMessageStats: could not print map message stats!");
        return false;
    }
}
std::string ThriftFilter::getMessageOpName(const map_thrift::MessageOp::type& m_type) {
    int t = static_cast<int>(m_type);
    std::string str;
    switch (t) {
        case 1:
            str = "ADD";
            break;
        case 2:
            str = "UPDATE_WHOLE";
            break;
        case 3:
            str = "UPDATE_PART";
            break;
        case 4:
            str = "DELETE";
            break;
        case 5:
            str = "POSE_UPDATE";
            break;
        case 6:
            str = "POSE_DELETE";
            break;
        default:
            str = "UNKNOWN TYPE";
            break;
    }
    return str;
}
void ThriftFilter::printMapStats(Map* map) {
    std::vector<int> stats = _map->getMapStats();
    LOG_INFO(cString::Format("MapStorage: %d Lanes | %d LaneGroups | %d LaneMarkings",
        stats[0], stats[1], stats[2]));
}
tResult ThriftFilter::setCarPoseFrom_tPosition() {
    object_ptr<const ISample> pTypePositionData;
    tFloat32 c_x = 0;
    tFloat32 c_y = 0;
    tFloat32 c_heading = 0;
    if (IS_OK(m_oReaderPosition.GetLastSample(pTypePositionData))) {
        auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pTypePositionData);
        RETURN_IF_FAILED(oDecoder.IsValid());
        c_x = access_element::get_value(oDecoder, m_ddlPositionId.x);
        c_y = access_element::get_value(oDecoder, m_ddlPositionId.y);
        c_heading = access_element::get_value(oDecoder, m_ddlPositionId.heading);
    } else {
        std::cout << ("ThriftClient: could not read tPosition") << std::endl;
    }
    car_object.setPose(c_x * 100., c_y * 100., 0.0, c_heading);
    LOG_DUMP(cString::Format("ThriftClient: car position updated. x: %f \n| y: %f | t: %f",
        c_x, c_y, c_heading));
    RETURN_NOERROR;
}
void ThriftFilter::sendCarMessage(const map_thrift::CarMessage& cm) {
    if (!is_connected_to_thrift_server) {
        connectToServer();
    }
    if (is_connected_to_thrift_server) {
        try {
            _thrift_car_client->sendCarPose(cm);
        } catch (TException& tx) {
            LOG_WARNING("ThriftFilter: Could not send CarPose. Catching exception");
            std::cout << tx.what() << std::endl;
        }
    }
}
void ThriftFilter::sendMapMessage(const map_thrift::MapMessage& mm) {
    if (!is_connected_to_thrift_server) {
        connectToServer();
    }
    if (is_connected_to_thrift_server) {
        try {
            _thrift_car_client->sendMapMessage(mm);
        } catch (TException& tx) {
            LOG_WARNING("ThriftFilter: Could not send MapMessage. Catching exception");
            std::cout << tx.what() << std::endl;
        }
    }
}
void ThriftFilter::sendThriftImage(const map_thrift::ThriftImage& image) {
    if (!is_connected_to_thrift_server) {
        connectToServer();
    }
    if (is_connected_to_thrift_server) {
        try {
            _thrift_car_client->sendThriftImage(image);
        } catch (TException& tx) {
            LOG_WARNING("ThriftFilter: Could not send ThriftImage. Catching exception");
            std::cout << tx.what() << std::endl;
        }
    }
}
map_thrift::Pose ThriftFilter::createThriftCarPose(const mapobjects::Car& co) {
    map_thrift::Pose to = map_thrift::Pose();
    map_thrift::Point3D p = map_thrift::Point3D();
    p.x = co.getX();
    p.y = co.getY();
    p.z = co.getZ();
    to.position = p;
    to.orientation = co.getT();
    return to;
}
std::vector<map_thrift::PointViz> createVisualizationPointList() {
    std::vector<map_thrift::PointViz> points_viz = std::vector<map_thrift::PointViz>();
    return points_viz;
}
void ThriftFilter::printCarPose(const map_thrift::Pose& c_p) {
    LOG_INFO(cString::Format("CarMessage pose x/y/z: %f|%f|%f",
        c_p.position.x, c_p.position.y, c_p.position.z));
}
void ThriftFilter::startMapServer() {
    if (_thrift_car_server) {
        std::ostringstream ss;
        ss << std::this_thread::get_id();
        std::string idstr = ss.str();
        LOG_INFO(cString::Format("ThriftFilter: map server thread: %s", idstr.c_str()));

        try {
            _thrift_car_server->serve();
            LOG_INFO("ThriftFilter: Thrift map server returned from serve()");
        } catch (...) {
            LOG_ERROR("ThriftFilter: something went wrong trying to serve");
        }

    } else {
        LOG_ERROR("ThriftFilter: TSimpleServer is null_ptr");
    }
}
void ThriftFilter::stopMapServer() {
    if (_thrift_car_server) {
        LOG_INFO("ThriftFilter: trying to stop the thrift map server...");
        try {
            _thrift_car_server->stop();
            LOG_INFO("ThriftFilter: map server stopped serving!");
            thread_thrift_map_server.join();
            LOG_INFO("ThriftFilter: joined map serverThread!");
        } catch (...) {
            LOG_ERROR("ThriftFilter: something went wrong trying to stop the map server");
        }
    } else {
        LOG_ERROR("ThriftFilter: TSimpleServer is null_ptr");
        thread_thrift_map_server.join();
    }
}
void ThriftFilter::startControlServer() {
    if (_thrift_control_server) {
        std::ostringstream ss;
        ss << std::this_thread::get_id();
        std::string idstr = ss.str();
        LOG_INFO(cString::Format("ThriftFilter: control server thread: %s", idstr.c_str()));

        try {
            _thrift_control_server->serve();
            LOG_INFO("ThriftFilter: Thrift control server returned from serve()");
        } catch (...) {
            LOG_ERROR("ThriftFilter: something went wrong trying to serve");
        }

    } else {
        LOG_ERROR("ThriftFilter: TSimpleServer is null_ptr");
    }
}
void ThriftFilter::stopControlServer() {
    if (_thrift_control_server) {
        LOG_INFO("ThriftFilter: trying to stop the thrift control server...");
        try {
            _thrift_control_server->stop();
            LOG_INFO("ThriftFilter: control server stopped serving!");
            thread_thrift_control_server.join();
            LOG_INFO("ThriftFilter: joined control serverThread!");
        } catch (...) {
            LOG_ERROR("ThriftFilter: something went wrong trying to stop the control server");
        }
    } else {
        LOG_ERROR("ThriftFilter: TSimpleServer is null_ptr");
        thread_thrift_control_server.join();
    }
}
bool ThriftFilter::connectToServer() {
    if (!is_connected_to_thrift_server) {
        stdcxx::shared_ptr<TTransport> socket(
            new TSocket(
                static_cast<std::string>(static_cast<cString>(m_propThriftRemoteServerIp)),
                static_cast<int>(thrift_remote_server_port)));
        _thrift_transport_client = stdcxx::shared_ptr<TTransport>(new TBufferedTransport(socket));
        stdcxx::shared_ptr<TProtocol> protocol(new TBinaryProtocol(_thrift_transport_client));
        _thrift_car_client = new MapCommClient(protocol);

        try {
            _thrift_transport_client->open();
            _thrift_car_client->ping();
            LOG_INFO("ThriftClientCar: sent PING to remote server");
            is_connected_to_thrift_server = true;
        } catch (TException& tx) {
            LOG_WARNING("ThriftClientCar: Couldn't connect to remote Thrift server");
            std::cout << cString::Format(
                "ThriftClientCar: ERROR: %s", tx.what()) << std::endl << std::flush;
            is_connected_to_thrift_server = false;
        }
    } else {
        try {
            _thrift_car_client->ping();
            LOG_INFO("ThriftClientCar: ping to remote server successful");
            is_connected_to_thrift_server = true;
        } catch (TException& tx) {
            LOG_WARNING("ThriftClientCar: Couldn't connect to remote Thrift server");
            std::cout << cString::Format(
                "ThriftClientCar: ERROR: %s", tx.what()) << std::endl << std::flush;
            is_connected_to_thrift_server = false;
        }
    }
    return is_connected_to_thrift_server;
}
void ThriftFilter::initializeMap() {
    _map = Map::getInstance();
    internalMapInitialized = true;
    LOG_INFO("MapServer: initialized the internal map");
}

bool ThriftFilter::deleteMap() {
    _map = Map::getInstance();
    LOG_WARNING("MapFilter: > deleting internal map");
    bool is_deleted = _map->deleteInternalMap();
    std::vector<int> stats = _map->getMapStats();
    LOG_INFO(cString::Format("MapStorage: %d Lanes | %d LaneGroups | %d LaneMarkings",
            stats[0], stats[1], stats[2]));
    return is_deleted;
}
    LOG_INFO("MapServer: received PING from remote");
}
bool ThriftFilter::MapCommHandler::sendMapMessage(const MapMessage& map_complete) {
    LOG_INFO("MapServer: receiving MapMessage from remote");
    Map* _map = Map::getInstance();
    _map->handleMapMessage(map_complete);
    if (_thrift_filter->m_propSaveMapFromServer) {
        _thrift_filter->writeMapToFile(map_complete);
    }
    return true;
}
bool ThriftFilter::MapCommHandler::sendCarPose(const CarMessage& car_pose) {
    LOG_INFO("ThriftServer: receiving CarMessage from remote. Ignoring!");
    return false;
}
bool ThriftFilter::MapCommHandler::sendThriftImage(const ThriftImage& image) {
    LOG_INFO("ThriftServer: receiving ThriftImage from remote. Ignoring!");
    return true;
}
void ThriftFilter::ControlCommHandler::ping() {
    LOG_INFO("ControlServer: received PING from remote");
}
CarStatus::type ThriftFilter::ControlCommHandler::sendCommand(const ControlMessage& command) {
    LOG_INFO("ControlServer: receiving control request from remote ControlClient");
    return control_thrift::CarStatus::type::UNINITIALIZED;
}

control_thrift::CarStatus::type ThriftFilter::ControlCommHandler::getCarStatus() {
    return control_thrift::CarStatus::type::UNINITIALIZED;
}

double ThriftFilter::ControlCommHandler::getRouteCosts(const ControlMessage& command) {
    if (command.op == control_thrift::MessageOp::COMPUTE_COSTS) {
        return _thrift_filter->_map->getRouteCosts(
            _thrift_filter->car_object.getPose(),
            *(mapobjects::Pose(command.p.x, command.p.y, command.p.z).convertFromADTF()));
    }
    return -1;
}
void ThriftFilter::writeNewestCostMapImage() {
    cv::Mat costmap_image = readImageInputPin();
    map_thrift::ThriftImage costmap_thrift_image = createThriftImage(costmap_image);
    sendThriftImage(costmap_thrift_image);
    return;
}
cv::Mat ThriftFilter::readImageInputPin() {
    cv::Mat image = cv::Mat();
    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oReaderImage.GetLastSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            image = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                CV_8UC3,
                const_cast<unsigned char*>(
                    static_cast<const unsigned char*>(pReadBuffer->GetPtr())));
        }
    }
    return image;
}
map_thrift::ThriftImage ThriftFilter::createThriftImage(const cv::Mat& image_in) {
    CV_Assert(image_in.depth() == CV_8U);
    int rows = image_in.rows;
    int cols = image_in.cols;
    int n_channels = image_in.channels();

    map_thrift::ThriftImage image_thrift = map_thrift::ThriftImage();
    image_thrift.width = cols;
    image_thrift.height = rows;
    image_thrift.channels = n_channels;

    std::string image_bytes = std::string();
    switch (n_channels) {
        case 1: {
            for (int y = 0; y < rows; y++) {
                for (int x = 0; x < cols; x++) {
                    image_bytes += (image_in.at<char>(y, x));
                }
            }
            break;
        }
        case 3: {
            for (int y = 0; y < rows; y++) {
                for (int x = 0; x < cols; x++) {
                    for (int c = 0; c < n_channels; c++) {
                        image_bytes += static_cast<char>(image_in.at<Vec3b>(y, x).val[c]);
                    }
                }
            }
            break;
        }
        default:
            LOG_ERROR(cString::Format(
                "ThriftFilter: wrong number of image input channels: %d", n_channels));
            break;
    }
    LOG_DUMP(cString::Format(
        "input size h: %d | w: %d", image_in.size().height, image_in.size().width));
    image_thrift.bytes = image_bytes;

    map_thrift::Pose current_pose = map_thrift::Pose();
    current_pose.position = map_thrift::Point3D();
    current_pose.position.x = car_object.getX();
    current_pose.position.y = car_object.getY();
    current_pose.position.z = car_object.getZ();
    current_pose.orientation = car_object.getT();
    image_thrift.pose = current_pose;

    return image_thrift;
}
void ThriftFilter::writeMapToFile(const map_thrift::MapMessage& map_message) {
    if (map_message.op == map_thrift::MessageOp::ADD ||
        map_message.op == map_thrift::MessageOp::UPDATE_WHOLE) {
        std::string filename = std::string(static_cast<cString>(m_propMapFilePath));
        filename += file_extension;
        LOG_INFO(cString::Format(
            "ThriftFilter: writing received map to file. path: %s",
            filename.c_str()));
        ::apache::thrift::stdcxx::shared_ptr<TFileTransport> transport(
            new TFileTransport(filename));
        ::apache::thrift::stdcxx::shared_ptr<TProtocol> protocol(
            new TBinaryProtocol(transport));

        map_message.write(protocol.get());
    } else {
        LOG_INFO("ThriftFilter: not writing mapfile. wrong MessageOp.");
    }
}
