
/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must
display the following acknowledgement: ?This product includes software developed
by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.

**********************************************************************/

#include "StateMachineFilter.h"
#include "OvertakingTask.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
                                    "StateMachine", cStateMachine,
                                    adtf::filter::pin_trigger({"carpos_in"}));

/** Filter wrapper class for StateMachineCtrl. Responsible for reading, writing and propagating pin
 * data. Supplies tasks to the StateMachineCtrl which should be executed (provided by jury or
 * other predecessors. **/

cStateMachine::cStateMachine() {
    registerCarPositionInputPin();
    registerSpeedOutPin();
    registerCurvatureOutPin();

    read_pin_lane_waypoints.registerPin(PointListPin::RW::READ,
                                        this,
                                        &(this->m_pClock),
                                        "points");

    read_pin_road_sign_data.registerPin(RoadSignDataPin::RW::READ,
                                        this,
                                        &(this->m_pClock),
                                        "roadSigns");

    read_pin_road_signs_map.registerPin(RoadSignsMapPin::RW::READ,
                                        this,
                                        &(this->m_pClock),
                                        "road_sign_map");

    read_pin_lidar.registerPin(this,
                               &(this->m_pClock),
                               "ls_data");

    write_pins_light_ctrl.registerPin(this, &(this->m_pClock));

    // video_output_local_map.registerPin(VideoOutPin::RW::WRITE,
    //                                    this,
    //                                    &(this->m_pClock),
    //                                    "video_local_map",
    //                                    300,
    //                                    300,
    //                                    3);

    network_in.registerPin(VideoOutPin::RW::READ,
                           this,
                           &(this->m_pClock),
                           "network_in",
                           300,
                           200,
                           3);


    birdseye_in.registerPin(VideoOutPin::RW::READ,
                            this,
                            &(this->m_pClock),
                            "birdseye_in",
                            300,
                            200,
                            3);

    jury_comm->registerPins(this, &m_pClock);

    read_pin_ls_seg_info.registerPin(LaserSegPin::RW::READ, this,
                            &(this->m_pClock),
                            "ls_seg_in");
    read_pin_obs_data.registerPin(DetectionInfoPin::RW::READ, this,
                            &(this->m_pClock),
                            "seg_obs_data");

    RegisterPropertyVariable("control loop trigger distance (m)", m_delta_car_pos_trigger);
    RegisterPropertyVariable("Enforce competition behaviour", m_propEnforceCompBehaviour);
    RegisterPropertyVariable("Emergency Brake distance (m)", m_em_brake_distance_m);
    RegisterPropertyVariable("default road signs file", m_roadSignFile);
    RegisterPropertyVariable("Maximum marker detection distance (m)", m_marker_det_distance_max);

    // local map output pin
    object_ptr<IStreamType> pType_local_map = make_object_ptr<cStreamType>(stream_meta_type_image());
    set_property(*pType_local_map, stream_meta_type_image::FormatName, ADTF_IMAGE_FORMAT(RGB_24));
    set_property(*pType_local_map, stream_meta_type_image::PixelWidth, local_map_output_width);
    set_property(*pType_local_map, stream_meta_type_image::PixelHeight, local_map_output_height);
    set_property(*pType_local_map, stream_meta_type_image::MaxByteSize, local_map_output_max_bytesize);
    Register(local_map_writer, "video_local_map", pType_local_map);

    object_ptr<IStreamType> pType_planner = make_object_ptr<cStreamType>(stream_meta_type_image());
    set_property(*pType_planner, stream_meta_type_image::FormatName, ADTF_IMAGE_FORMAT(RGB_24));
    set_property(*pType_planner, stream_meta_type_image::PixelWidth, planner_output_width);
    set_property(*pType_planner, stream_meta_type_image::PixelHeight, local_map_output_height);
    set_property(*pType_planner, stream_meta_type_image::MaxByteSize, planner_output_max_bytesize);
    Register(planner_writer, "planner_costmap", pType_planner);

}

void addTask(deque<shared_ptr<BaseTask>> *list, BaseTask *task) {
    list->emplace_back(shared_ptr<BaseTask>(task));
}

template<class Task>
void addTask(deque<shared_ptr<BaseTask>> *list, shared_ptr<Task> task) {
    list->emplace_back(task);
}


void cStateMachine::setupTestingTasks() {

    //Values are now set in each task directly
    //float wp_prox_m = 0.35;
    //float goal_prox_m = 0.50;

    std::shared_ptr<OvertakingTask> task0 =
        std::make_shared<OvertakingTask>(0);
    addTask(&tasks, task0);

    LOG_INFO("Num tasks: %d", tasks.size());
}


void cStateMachine::setupChildReferences() {
    stateMachineCtrl->setCarModel(car);
    obstacle_processor.setCarModel(car);
}


void cStateMachine::parseDefaultRoadSignsFile() {
    cFilename fileRoadSign = m_roadSignFile;
    adtf::services::ant::adtf_resolve_macros(fileRoadSign);
    if (cFileSystem::Exists(fileRoadSign)) {
        cDOM oDOMFromFile;
        oDOMFromFile.Load(fileRoadSign);
        vector<RoadSign> signs;
        vector<ParkingSpace> parking;
        RoadSignsMapPin::parseRoadSignFile(oDOMFromFile, &signs, &parking);
        LOG_INFO("%d signs and %d parking spaces loaded from default road signs file", signs.size(),
                 parking.size());
        RoadSignInfo::init(signs, parking, m_marker_det_distance_max);
    } else {
        LOG_ERROR("Unable to load default road signs, file does not exist");
    }


}

tResult cStateMachine::Configure() {
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    setupChildReferences();
    em_brake_override.reset(new EmergencyBrakeOverride(m_em_brake_distance_m));
    em_brake_override->setCarModel(car);
    stateMachineCtrl->setEmergencyBrakeCtrl(em_brake_override);
    stateMachineCtrl->configure();




    parseDefaultRoadSignsFile();
    if (m_propEnforceCompBehaviour) {
        // do not preinitialize tasks when in competition mode
        RETURN_NOERROR;
    }

    setupTestingTasks();
    advanceToNextTask(nullptr);

    RETURN_NOERROR;
}

/** Process data retrieved by pins **/
tResult cStateMachine::Process(tTimeStamp tmTimeOfTrigger) {
    StopWatch watch;
    StopWatch process_time_watch;

    jury_comm->checkUpdates();
    watch.print_measurement("Jury", 5);
    watch.reset();

    vector<RoadSign> roadSigns;
    vector<ParkingSpace> parkingSpace;
    if (IS_OK(read_pin_road_signs_map.readData(&roadSigns, &parkingSpace))) {
        RoadSignInfo::init(roadSigns, parkingSpace, m_marker_det_distance_max);
        LOG_INFO("received road signs data: %d signs / %d parking lots", roadSigns.size(),
                 parkingSpace.size());
        for (auto &s : roadSigns){
            LOG_INFO("Sign: id: %d, x: %f, y: %f, init: %d", s.u16Id, s.f32X, s.f32Y, s.bInit);
        }
    }
    watch.print_measurement("Road sign map", 25);
    watch.reset();


    if ((!start_cmd_received && m_propEnforceCompBehaviour) || job_done) {
        if (log_watch.didSecondsPass(2)) {
            LOG_INFO("Waiting for start cmd");
            log_watch.reset();
        }

        transmitSpeedSignal(0);
        RETURN_NOERROR;
    }


    object_ptr<const ISample> pReadCarPosSample;
    if (IS_OK(m_oReaderCarPosition.GetLastSample(pReadCarPosSample))) {
        readCarPosition(pReadCarPosSample);
    }
    watch.print_measurement("CarPosUpdate", 10);
    watch.reset();


    std::vector<tPolarCoordiante> scan;
    if (IS_OK(read_pin_lidar.readData(&scan))) {      // (changed to GetNextSample 31.10.2018, 08.11.)
        processLaserScannerData(&scan);
    }
    watch.print_measurement("LS data", 25);
    watch.reset();


    if (!car->isInitialized()) {
        if (log_watch.didSecondsPass(2)) {
            transmitSpeedSignal(Speed::STOP);
            LOG_INFO("Waiting for valid position...");
            log_watch.reset();
            // lights
            setIndicatorLeftEnabled(false);
            setIndicatorRightEnabled(false);
            setHazardLightsEnabled(false);
            setBrakeLightsEnabled(false);
            setHeadLightsEnabled(false);
            setReverseLightsEnabled(false);

            int id = 0;
            if (!tasks.empty()) {
                id = tasks.front()->getId();
            }
            jury_comm->onCarStartup(id);
        }
        RETURN_NOERROR;
    }


    // not needed any more, we plan inside the state machine package
    // vector<tPoint> laneWaypoints;
    // if (IS_OK(read_pin_lane_waypoints.readData(&laneWaypoints)) && !laneWaypoints.empty()) {   // (changed to GetNextSample 31.10.2018, 08.11.)
    //     processLaneWaypoints(&laneWaypoints);
    // }
    // watch.print_measurement("Lane waypoint data", 10);
    // watch.reset();


    cv::Mat network_data;
    if (IS_OK(network_in.readData(&network_data))) {     // (changed to GetNextSample 31.10.2018, 08.11.)
        if (!network_data.empty()) {
            stateMachineCtrl->onPerceptionDataAvailable(&network_data);
        }
    }
    watch.print_measurement("Network data", 15);
    watch.reset();


    RoadSignDetection roadSignData;
    while(IS_OK(read_pin_road_sign_data.readData(&roadSignData))) {
        stateMachineCtrl->onRoadSignDetected(&roadSignData.id);
    }
    // no markers detected
    if(roadSignData.id == -1){
        stateMachineCtrl->onRoadSignDetected(nullptr);
    }

    watch.print_measurement("Road sign detection", 25);
    watch.reset();

    cv::Mat birdseye_data;
    if (IS_OK(birdseye_in.readData(&birdseye_data))) {   // (changed to GetNextSample 31.10.2018, 08.11.)
        if (!birdseye_data.empty()) {
            stateMachineCtrl->setBirdsEyeImage(birdseye_data);
        }
    }
    watch.print_measurement("Birdseye data", 20);
    watch.reset();


    tDetectionInfo det_info;
    bool data_set = false;
    while (IS_OK(read_pin_obs_data.readData(&det_info))) {
        // empty buffer, signal at least one read was successful
        data_set = true;
    }
    vector<tLaserSegStruct> ls_seg_data;
    while (IS_OK(read_pin_ls_seg_info.readData(&ls_seg_data))) {
        if (data_set) {
            stateMachineCtrl->onUpdateDetectionInfo(det_info, &ls_seg_data);
        }
        ls_seg_data.clear();
    }


    watch.print_measurement("Update LaserSeg detection info", 5);
    watch.reset();


    stateMachineCtrl->run();
    watch.print_measurement("StatemachineCtrl run", 25);
    watch.reset();


    // local map output
    if (local_map_buffer.hasDataAvailable()) {
        object_ptr<ISample> pWriteSample;
        cv::Mat outputImage = *local_map_buffer.getDataPtr();
        if (!outputImage.empty()) {
            if (IS_OK(alloc_sample(pWriteSample, m_pClock->GetStreamTime()))) {
                object_ptr_locked<ISampleBuffer> pWriteBuffer;
                if (IS_OK(pWriteSample->WriteLock(
                    pWriteBuffer, outputImage.cols * outputImage.rows * outputImage.channels()))) {
                    pWriteBuffer->Write(
                        adtf_memory_buffer<void, tSize>(reinterpret_cast<void*>(outputImage.data),
                        outputImage.cols * outputImage.rows * outputImage.channels()));
                }
            }
            local_map_writer << pWriteSample << flush << trigger;
        }
        // video_output_local_map.writeData(*local_map_buffer.getDataPtr());
        local_map_buffer.setDataAvailable(false);
    }

    // planner output
    if (planner_buffer.hasDataAvailable()) {
        object_ptr<ISample> pWriteSample;
        cv::Mat outputImage = *planner_buffer.getDataPtr();
        if (!outputImage.empty()) {
            if (IS_OK(alloc_sample(pWriteSample, m_pClock->GetStreamTime()))) {
                object_ptr_locked<ISampleBuffer> pWriteBuffer;
                if (IS_OK(pWriteSample->WriteLock(
                    pWriteBuffer, outputImage.cols * outputImage.rows * outputImage.channels()))) {
                    pWriteBuffer->Write(
                        adtf_memory_buffer<void, tSize>(reinterpret_cast<void*>(outputImage.data),
                        outputImage.cols * outputImage.rows * outputImage.channels()));
                }
            }
            planner_writer << pWriteSample << flush << trigger;
        }
        planner_buffer.setDataAvailable(false);
    }

    watch.print_measurement("Write local map", 25);
    watch.reset();


    process_time_watch.print_measurement("StateMachineFilter::Process", 25);

    RETURN_NOERROR;
}





/** Control / callback logic **/

bool cStateMachine::advanceToNextTask(BaseTask *prevTask) {

    setIndicatorLeftEnabled(false);
    setIndicatorRightEnabled(false);
    setHazardLightsEnabled(false);
    //setHeadLightsEnabled(false);
    setReverseLightsEnabled(false);

    LOG_INFO("Advance to next task, task list size: %d ", tasks.size());
    if (tasks.empty()) {
        setHazardLightsEnabled(true);
        transmitSpeedSignal(Speed::STOP);
        if (prevTask) {
            jury_comm->onCarFinishedExecution(prevTask);
        } else {
            jury_comm->onCarFinishedExecution(0);
        }

        start_cmd_received = false;
        LOG_INFO("tasks empty");
        job_done = true;
        return true;
    }
    if (prevTask) {
        tasks.pop_front();
        if (tasks.empty()) {
            return advanceToNextTask(prevTask);
        }
    }


    LOG_INFO(cString::Format("Current task id: %d", tasks.front()->getId()));
    if (tasks.front()->doReportToJury()) {
        jury_comm->onCarStartup(tasks.front().get());
    }
    stateMachineCtrl->setTask(tasks.front());
    return true;
}

bool cStateMachine::taskFinished(BaseTask *task) {
    return advanceToNextTask(task);
}


bool cStateMachine::taskFailed(BaseTask *task) {
    jury_comm->onManeuverError(task);
    return advanceToNextTask(task);
}

bool cStateMachine::taskStarted(BaseTask *task) {
    if (task && !task->doReportToJury()) {
        return true;
    }
    return IS_OK(jury_comm->onManeuverRunning(task));
}

vector<cStateMachine::Sector> cStateMachine::getSectorList() {
    vector<Sector> sector_list;
    man_sector_lookup.clear();
    for (auto &aadc_sector : man_list_jury) {
        LOG_INFO("Adding sector: %d", aadc_sector.id);
        cout << "Adding sector: " << aadc_sector.id << endl;
        Sector sector;
        sector.sector_id = aadc_sector.id;
        for (auto &aadc_man : aadc_sector.sector) {
            cout << "Adding task: " << aadc_man.id << endl;
            BaseTask::Type type = TaskFactory::convertAADCType(aadc_man.action);
            LOG_INFO("aadc_man.extra %d ", aadc_man.extra);
            TaskFactory::makeTask(aadc_man.id, type, aadc_man.extra, &sector.maneuvers);
            man_sector_lookup.insert({aadc_man.id, aadc_sector.id});
        }

        sector_list.emplace_back(sector);
    }
    LOG_INFO("%d sectors received", sector_list.size());
    for (auto sector : sector_list) {
        LOG_INFO("num tasks %d: ", sector.maneuvers.size());
    }
    tasks_from_jury = true;
    return sector_list;
}

int cStateMachine::getSectorFromTaskId(int task_id) {
    try {
        return man_sector_lookup.at(task_id);
    } catch (std::out_of_range& err) {
        if (tasks_from_jury) {
            LOG_WARNING("StateMachineFilter: could not find sector to task");
        }
        return -1;
    }
}

void cStateMachine::onGetReady(int manId) {
    jury_comm->onCarStartup(manId);
    tasks.clear();
    vector<Sector> sector_list = getSectorList();
    LOG_INFO("Looking for task %d in %d sectors", manId, sector_list.size());
    if (sector_list.empty()) {
        jury_comm->onManeuverError(manId);
        LOG_INFO("Sector list is empty!");
        return;
    }
    bool task_found = false;
    for (auto sector : sector_list) {
        cout << "Sector: " << sector.sector_id << endl;
        for (auto &task : sector.maneuvers) {

            cout << "Checking Task: " << task->getId() << endl;
            if (task->getId() == manId) {
                cout << "found task with id " << manId << endl;
                task_found = true;
                addTask(&tasks, task);
            } else if (task_found) {
                cout << "Adding Task: " << task->getId() << endl;
                addTask(&tasks, task);
            }
        }
    }


    LOG_INFO("%d tasks added, sector found: %d", tasks.size(), task_found);
    if (!task_found) {
        jury_comm->onManeuverError(manId);
        LOG_ERROR("ManId not found in task list");
        return;
    }

    if (!tasks.empty()) {
        advanceToNextTask(nullptr);
        jury_comm->onCarReady(manId);
    }
}

void cStateMachine::onStartManeuver(int manId) {
    if (tasks.empty()) {
        LOG_ERROR("Attempt to start maneuver but task list was empty");
        jury_comm->onManeuverError(manId);
        return;
    }
    if (tasks.front()->getId() != manId) {
        LOG_ERROR("Attempt to start maneuver but first task id didnt match "
                 "desired start task id");
        jury_comm->onManeuverError(manId);
        return;
    }

    start_cmd_received = true;
    job_done = false;
    setHeadLightsEnabled(true);
}

void cStateMachine::onStop(int manId) {
    start_cmd_received = false;
    stateMachineCtrl->reset();
    tasks.clear();
    advanceToNextTask(nullptr);
}


void cStateMachine::onManeuverListAvailable(aadc::jury::maneuverList sectorList) {
    man_list_jury = sectorList;
    LOG_INFO("Maneuver list received");
}

/** Pin processing**/


tResult cStateMachine::readCarPosition(object_ptr<const ISample> sample) {
    auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*sample);
    RETURN_IF_FAILED(oDecoder.IsValid());

    tPosition carPos;

    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &carPos.f32x));
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &carPos.f32y));
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &carPos.f32heading));

    DeltaPosition delta = car->updatePosition(carPos);

    if (fabs(delta.dx) + fabs(delta.dy) >= m_delta_car_pos_trigger || true) {

        //std::lock_guard<std::mutex> lock(mutex_jury_comm);
        stateMachineCtrl->onCarPositionUpdate(delta);
    }

    RETURN_NOERROR;
}


vector<Lane> cStateMachine::processLaneData(vector<tLaneElement> *data) {

    // deprecated, remove
    vector<Lane> lanes = vector<Lane>();
    for (tLaneElement elem : (*data)) {
        if (elem.coeff0 != 0 && elem.coeff1 != 0 && elem.coeff2 != 0) {
            lanes.emplace_back(Lane(elem));
        }
    }
    return lanes;
}


void cStateMachine::processLaneWaypoints(vector<tPoint> *input) {

    vector<Point> result;
    // result.reserve(input->size());

    // for (auto &elem : *input) {

    //     // care for ordering here (currently far-> front)
    //     result.emplace_back(Point::Local(-elem.f32y, elem.f32x, car->getRearAxis().getZ()));
    //     //LOG_INFO("Perception (%f, %f)", elem.f32x, elem.f32y);
    // }

    stateMachineCtrl->onLanePointDataAvailable(&result);

}


void cStateMachine::processLaserScannerData(const std::vector<tPolarCoordiante>* input) {
    /*
     * remove unnecessary processing of data which is not used currently
    vector<Obstacle> output;
    output.reserve(input->size());
    obstacle_processor.processLsData(input, &output);
     */
    std::vector<Point> global_cartesian_ls_points;
    stateMachineCtrl->laserscannerPolarToGlobalCartesian(*input, &global_cartesian_ls_points);
    stateMachineCtrl->onLaserScannerUpdate(*input, global_cartesian_ls_points);

    Point steer_anchor = car->getFrontAxis();

    if (!tasks.empty()) {
        steer_anchor = tasks.front()->getSteeringAnchor();
    }
    em_brake_override->onLaserScannerUpdate(*input,
                                            global_cartesian_ls_points,
                                            steer_anchor,
                                            car);
}


void cStateMachine::processLanePointData(const vector<tPoint> &data, std::vector<Point> *result) {

}

/** Pin registering and writing **/



bool cStateMachine::isLocalMapBufferEmpty() {
    if (local_map_buffer.hasDataAvailable()) {
        return false;
    }
    return true;
}

bool cStateMachine::transmitLocalMap(cv::Mat *localMapRgb) {
    if(local_map_buffer.hasDataAvailable()) {
        return false;
    }
    local_map_buffer.setData(*localMapRgb);
    return true;
}

bool cStateMachine::isPlannerBufferEmpty() {
    if (planner_buffer.hasDataAvailable()) {
        return false;
    }
    return true;
}

bool cStateMachine::transmitPlanner(cv::Mat *plannerRgb) {
    if(planner_buffer.hasDataAvailable()) {
        return false;
    }
    planner_buffer.setData(*plannerRgb);
    return true;
}


bool cStateMachine::transmitCurvature(float curvature) {
    return IS_OK(writeCurvature(curvature));
}

tResult cStateMachine::writeCurvature(float curv_out) {
    object_ptr<ISample> pWriteSample;
    tTimeStamp timeOfSignal = m_pClock->GetStreamTime();

    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, curv_out));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, timeOfSignal));
    }
    //LOG_INFO("write curvature %f", curv_out);
    m_oWriterCurvature << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}


bool cStateMachine::transmitSpeedSignal(float speed) {
    return cStateMachine::writeSpeedSignal(speed) == ERR_NOERROR;
}

tResult cStateMachine::writeSpeedSignal(float speed) {
    object_ptr<ISample> pWriteSample;
    tTimeStamp timeOfSignal = m_pClock->GetStreamTime();

    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, speed));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp, timeOfSignal));
    }
    m_oWriterSpeed << pWriteSample << flush << trigger;

    //LOG_INFO("write speed: %f", speed);
    RETURN_NOERROR;
}

void cStateMachine::registerCarPositionInputPin() {
    object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition",
                                                                                       pTypePositionData,
                                                                                       m_PositionSampleFactory)) {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius",
                                             m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed",
                                             m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading",
                                             m_ddlPositionIndex.heading);
    } else {
        LOG_WARNING("No mediadescription for tPosition found!");
    }
    Register(m_oReaderCarPosition, "carpos_in", pTypePositionData);
}


void cStateMachine::registerSpeedOutPin() {
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service
                     ("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
                                              cString("ui32ArduinoTimestamp"),
                                              m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
                                              cString("f32Value"), m_ddlSignalValueId.value));
    } else {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oWriterSpeed, "speed_out", pTypeSignalValue);
}

void cStateMachine::registerCurvatureOutPin() {
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service
                     ("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
                                              cString("ui32ArduinoTimestamp"),
                                              m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
                                              cString("f32Value"), m_ddlSignalValueId.value));
    } else {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oWriterCurvature, "curvature", pTypeSignalValue);
}


bool cStateMachine::setIndicatorRightEnabled(bool enable) {
    bool success = true;
    int repeat_count = 0;
    while (repeat_count < LIGHT_ACTIVATION_REPEAT) {
        success &= write_pins_light_ctrl.setIndicatorRightEnabled(enable);
        repeat_count++;
    }
    return success;
}

bool cStateMachine::setIndicatorLeftEnabled(bool enable) {
    bool success = true;
    int repeat_count = 0;
    while (repeat_count < LIGHT_ACTIVATION_REPEAT) {
        success &= write_pins_light_ctrl.setIndicatorLeftEnabled(enable);
        repeat_count++;
    }
    return success;
}

bool cStateMachine::setHazardLightsEnabled(bool enable) {
    bool success = true;
    int repeat_count = 0;
    while (repeat_count < LIGHT_ACTIVATION_REPEAT) {
        success &= write_pins_light_ctrl.setHazardLightsEnabled(enable);
        repeat_count++;
    }
    return success;
}

bool cStateMachine::setBrakeLightsEnabled(bool enable) {
    bool success = true;
    int repeat_count = 0;
    while (repeat_count < LIGHT_ACTIVATION_REPEAT) {
        success &= write_pins_light_ctrl.setBrakeLightsEnabled(enable);
        repeat_count++;
    }
    return success;
}

bool cStateMachine::setHeadLightsEnabled(bool enable) {
    bool success = true;
    int repeat_count = 0;
    while (repeat_count < LIGHT_ACTIVATION_REPEAT) {
        success &= write_pins_light_ctrl.setHeadLightsEnabled(enable);
        repeat_count++;
    }
    return success;
}

bool cStateMachine::setReverseLightsEnabled(bool enable) {
    bool success = true;
    int repeat_count = 0;
    while (repeat_count < LIGHT_ACTIVATION_REPEAT) {
        success &= write_pins_light_ctrl.setReverseLightsEnabled(enable);
        repeat_count++;
    }
    return success;
}

bool cStateMachine::requestOvertake(BaseTask *task) {
    advanceToNextTask(task);
    vector<std::shared_ptr<BaseTask>> new_task;
    TaskFactory::makeTask(task->getId(), task->getType(), task->getExtra(), &new_task);
    tasks.emplace_front(new_task[0]);
    tasks.emplace_front(make_shared<OvertakingTask>(task->getId()));
    return advanceToNextTask(nullptr);
}


