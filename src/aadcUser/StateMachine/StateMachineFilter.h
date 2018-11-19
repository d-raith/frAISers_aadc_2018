
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

#ifndef AADC_USER_STATEMACHINE_FILTER_H
#define AADC_USER_STATEMACHINE_FILTER_H

#include <vector>
#include "IControlOutput.h"
#include "Point.h"
#include "StateMachineCtrl.h"
#include "stdafx.h"
#include "GridMap.h"
#include "customtypes.h"
#include "Lane.h"
#include "deque"
#include <map>
#include "ObstacleProcessor.h"

#include "../PinClasses/PointListPin.h"
#include "../PinClasses/LaneListPin.h"
#include "../PinClasses/RoadSignDataPin.h"
#include "../PinClasses/LightCtrlPin.h"
#include "../PinClasses/LaserScannerPin.h"
#include "../PinClasses/OpencvFramePin.h"
#include "../PinClasses/VideoOutPin.h"
#include "../PinClasses/RoadSignsMapPin.h"
#include "../PinClasses/LaserSegPin.h"
#include "../PinClasses/DetectionInfoPin.h"
#include "Map.h"
#include "JuryComm.h"
#include "PlanningTask.h"
#include "WaypointTask.h"
#include "ReverseDrivingTask.h"
#include "TaskFactory.h"
#include "../PinClasses/Threadpool.h"
#include "../PinClasses/AsyncSampleBuffer.h"
#include "CarModel.h"
#include "ISensorListeners.h"
#include "LaneFollowingTask.h"
#include "LaneFollowingPerceptionTask.h"
#include "TurnRightTask.h"
#include "RoadSignInfo.h"
#include "EmergencyBrakeOverride.h"


//*************************************************************************************************
#define CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER \
  "state_machine.filter.user.aadc.cid"

using namespace adtf_util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

using namespace fraisers::models;

class cStateMachine : public cTriggerFunction, public IControlOutput, public ILightCtrl,
        public JuryCommandListener {
private:

    static constexpr int LIGHT_ACTIVATION_REPEAT = 25;

    struct Sector {
        int sector_id;
        vector<shared_ptr<BaseTask>> maneuvers;
    };

    std::shared_ptr<CarModel> car = CarModel::init();
    IGlobalMap *map = Map::getInstance();

    deque<shared_ptr<BaseTask>> tasks;
    ObstacleProcessor obstacle_processor = ObstacleProcessor(false, 0, 1200, 335, 25);

    std::unique_ptr<StateMachineCtrl> stateMachineCtrl = std::make_unique<StateMachineCtrl>(this,
            map, this);
    std::unique_ptr<JuryCommunication> jury_comm= std::make_unique<JuryCommunication>(this);

    std::shared_ptr<EmergencyBrakeOverride> em_brake_override;


    aadc::jury::maneuverList man_list_jury;

    std::unordered_map<int, int> man_sector_lookup = std::unordered_map<int, int>();
    bool tasks_from_jury = false;

    std::atomic<bool> start_cmd_received = {false};

    AsyncSampleBuffer<cv::Mat> local_map_buffer;
    AsyncSampleBuffer<cv::Mat> planner_buffer;

    bool job_done = false;

    StopWatch log_watch;


    struct tSignalValueId {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    struct tPositionId {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;

    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

    adtf::mediadescription::cSampleCodecFactory m_PointSampleFactory;

    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;


    property_variable<tFloat32> m_delta_car_pos_trigger = tFloat32(0.01);
    property_variable<tFloat32> m_marker_det_distance_max = tFloat32(1.0);
    property_variable<tFloat32> m_em_brake_distance_m = tFloat32(0.3);
    adtf::base::property_variable<cFilename> m_roadSignFile = cFilename("roadSigns.xml");
    property_variable<tBool>   m_propEnforceCompBehaviour = true;

    cPinWriter m_oWriterSpeed;
    cPinWriter m_oWriterCurvature;;
    cPinReader m_oReaderCarPosition;

    VideoOutPin video_output_local_map;
    VideoOutPin network_in;
    VideoOutPin birdseye_in;

    object_ptr<adtf::services::IReferenceClock> m_pClock;

    PointListPin read_pin_lane_waypoints;
    RoadSignDataPin read_pin_road_sign_data;
    LightCtrlPin write_pins_light_ctrl;
    LaserScannerPin read_pin_lidar;
    LaserSegPin read_pin_ls_seg_info;
    DetectionInfoPin read_pin_obs_data;



    RoadSignsMapPin read_pin_road_signs_map;

    // size of the local map output (incl. debug output)
    tInt local_map_output_width = 400;
    tInt local_map_output_height = 300;
    tInt local_map_output_channels = 3;
    tInt local_map_output_max_bytesize = local_map_output_width *
        local_map_output_height * local_map_output_channels;
    cSampleWriter local_map_writer;
    adtf::streaming::tStreamImageFormat m_sImageFormat;
    adtf::ucom::object_ptr<IStreamType> m_Type;

    tInt planner_output_width = 300;
    tInt planner_output_max_bytesize = planner_output_width *
        local_map_output_height * local_map_output_channels;

    cSampleWriter planner_writer;

    void registerSpeedOutPin();

    void registerCarPositionInputPin();

    void registerCurvatureOutPin();



    void setupChildReferences();

    vector<Sector> getSectorList();

    tResult readCarPosition(object_ptr<const ISample> sample);

    void processLaserScannerData(const vector<tPolarCoordiante>* input);

    void processLaneWaypoints(vector<tPoint> *input);

    vector<Lane> processLaneData(std::vector<tLaneElement> *data);

    void processLanePointData(const std::vector<tPoint> &data, std::vector<fraisers::models::Point>
    *result);

    tResult writeSteerSignal(fraisers::models::Point point);

    tResult writeSpeedSignal(float speed);

    tResult writeCurvature(float curv_out);

    void onGetReady(int manId) override;

    void onStartManeuver(int manId) override;

    void onStop(int manId) override;

    void onManeuverListAvailable(aadc::jury::maneuverList sectorList) override;

    void setupTestingTasks();


    void parseDefaultRoadSignsFile();



public:
    /*! Default constructor. */
    cStateMachine();

    /*! Destructor. */
    ~cStateMachine() override {
    };


    bool transmitCurvature(float curvature) override;

    bool transmitSpeedSignal(float speed) override;

    bool requestOvertake(BaseTask *task) override;

    bool isLocalMapBufferEmpty() override;

    bool transmitLocalMap(cv::Mat *localMapRgb) override;

    bool isPlannerBufferEmpty() override;

    bool transmitPlanner(cv::Mat *plannerRgb) override;

    int getSectorFromTaskId(int task_id) override;

    bool taskFinished(BaseTask *task) override;

    bool taskFailed(BaseTask *task) override;

    bool taskStarted(BaseTask *task) override;

    bool advanceToNextTask(BaseTask *task);

    bool setIndicatorRightEnabled(bool enable) override;

    bool setIndicatorLeftEnabled(bool enable) override;

    bool setHeadLightsEnabled(bool enable) override;

    bool setReverseLightsEnabled(bool enable) override;

    bool setHazardLightsEnabled(bool enable) override;

    bool setBrakeLightsEnabled(bool enable) override;

    /**
     * Overwrites the Configure
     * This is to Read Properties prepare your Trigger Function
     */
    tResult Configure() override;

    /**
     * Overwrites the Process
     * You need to implement the Reading and Writing of Samples within this
     * function MIND: Do Reading until the Readers queues are empty or use the
     * IPinReader::GetLastSample() This FUnction will be called if the Run() of
     * the TriggerFunction was called.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;
};
#endif //AADC_USER_STATEMACHINE_FILTER_H
//*************************************************************************************************
