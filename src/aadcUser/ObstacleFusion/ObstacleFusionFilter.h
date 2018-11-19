
/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once

//*************************************************************************************************
#define CID_OBSTACLE_FUSION_FILTER "obstacle_fusion.filter.user.aadc.cid"
#include <vector>
#include "../PinClasses/LaserSegPin.h"
#include "../PinClasses/BoolSignalValuePin.h"
#include "../PinClasses/DetectionInfoPin.h"
#include "../PinClasses/StopWatch.h"
#include "UltrasonicHitCount.h"
#include "LaserSegBoolHitCount.h"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class ObstacleFusionFilter : public cTriggerFunction {


    /*BoolSignalValuePin read_pin_person;
    BoolSignalValuePin read_pin_car_left;
    BoolSignalValuePin read_pin_car_right;
    BoolSignalValuePin read_pin_car_front;*/


    /*! A signal value identifier. */
    struct tSignalValueId
    {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;

    /*! A ddl ultrasonic structure index. */
    struct
    {
        tSignalValueId SideLeft;
        tSignalValueId SideRight;
        tSignalValueId RearLeft;
        tSignalValueId RearCenter;
        tSignalValueId RearRight;

    } m_ddlUltrasonicStructIndex;
    adtf::mediadescription::cSampleCodecFactory m_USDataSampleFactory;

    /*! The input ultrasonic unit */
    cPinReader       m_oInputUltrasonicUnit;


    LaserSegPin ls_seg_input;
    DetectionInfoPin obs_data_out;
    LaserSegPin ls_seg_out;
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    std::shared_ptr<UltrasonicHitCount> us_hit_counts;
    std::shared_ptr<LaserSegBoolHitCount> laser_seg_bool_hit_counts;


    void registerUsPin();
    tResult readUltraSonics(tUltrasonicStruct *us_data);


    void processLsSegInput(vector<tLaserSegStruct> *data);

    void processUsData(tUltrasonicStruct *us_data);


    property_variable<tFloat32> us_threshold_hit = tFloat32(20);
    property_variable<tInt32> hit_count_reset_us = tInt32(1000);
    property_variable<tInt32> hit_count_reset_ls_seg_bool = tInt32(100);

    property_variable<tInt32> th_car_left = tInt32(-15);
    property_variable<tInt32> th_car_right = tInt32(15);
    property_variable<tFloat32> obstacle_distance = tFloat32(2.0);

    property_variable<tInt32> class_car_id = tInt32(4);
    property_variable<tInt32> class_person_id = tInt32(20);
    property_variable<tInt32> class_child_id = tInt32(13);





 public:
    /*! Default constructor. */
    ObstacleFusionFilter();

    /*! Destructor. */
    ~ObstacleFusionFilter() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult transmitSpeed(tSignalValue speedSignal);

    void checkEmergencyBreak(const std::vector<tPolarCoordiante>& scan);
};


//*************************************************************************************************
