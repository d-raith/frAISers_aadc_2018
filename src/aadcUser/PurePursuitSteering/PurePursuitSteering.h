
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
#include <vector>
#include "stdafx.h"
#include "deque"


//*************************************************************************************************
#define CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER "pure_pursuit_steering.filter.user.aadc.cid"

using namespace adtf_util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

class cPurePursuitSteering : public cTriggerFunction {
 private:


    /*! Reader of an InPin. */
    cPinReader m_oReaderCalibratedValue;
    /*! Media Descriptions. */

    struct tPointId {
        tSize x;
        tSize y;
       } m_ddlPointId;

   struct tPositionId
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;
    adtf::mediadescription::cSampleCodecFactory m_PointSampleFactory;


    /*! Reader of an InPin. */
    cPinReader m_oReaderWaypoint;

    cPinReader m_oReaderCarPosition;

    struct tSignalValueId {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;


    /*! The template data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    /*! Writer to an OutPin. */
    cPinWriter m_oWriterSteer;

    cPinWriter m_oWriterRequestCalibration;

    property_variable<tFloat32> m_propxStart = tFloat32(0.0);
    property_variable<tFloat32> m_propyStart = tFloat32(0.0);
    property_variable<tFloat32> m_propxGoal = tFloat32(3.0);
    property_variable<tFloat32> m_propyGoal = tFloat32(3.0);

    property_variable<tFloat32> m_propScaleKp = tFloat32(1.9);
    property_variable<tFloat32> m_propScaleKd = tFloat32(0.8);
    property_variable<tFloat32> m_propScaleKi = tFloat32(0.0);

    tPoint lookahead_point_;
    tPosition carPosition;

    deque<float> steerHistory = deque<float>(5, 0.0);
    float lastSteer = 0;
    float lastX = 0;
    float errorSum = 0;
    tTimeStamp lastUpdate = 0;

    float lookahead_dist = 1; //lookahead dist in m
    float wheel_base = 0.6; // wheel base in m

    void ConvertToLocal(tPoint wp, tPosition pos);

    float CalculateCurvature(tPoint wp, tPosition pos);

    void TransmitProportionalSteering(tPoint wp, tPosition pos, tTimeStamp tmTimeOfTrigger);

 public:
    /*! Default constructor. */
    cPurePursuitSteering();

    /*! Destructor. */
    virtual ~cPurePursuitSteering() = default;

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

    tResult TransmitCurvature(tTimeStamp tmTimeOfTrigger);

    tResult TransmitSteer(tSignalValue steerSignal);
};


//*************************************************************************************************
