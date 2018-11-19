
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

#include "PurePursuitSteering.h"
#include <math.h>
#include <iostream>

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER, "PurePursuitSteering",
    cPurePursuitSteering,
    adtf::filter::pin_trigger({"carPosition_in"}));

tFloat32 WHEELBASE = 1;
tFloat32 ANCHOR_XOFFSET = 1;

#define SCALE_GLOBAL 100;

cPurePursuitSteering::cPurePursuitSteering() {
  // DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE
  // aadc.description

  object_ptr<IStreamType> pTypePointValue;
  if
    IS_OK(adtf::mediadescription::ant::
              create_adtf_default_stream_type_from_service(
                  "tPoint", pTypePointValue, m_PointSampleFactory)) {
      (adtf_ddl::access_element::find_index(m_PointSampleFactory,
                                            cString("f32x"), m_ddlPointId.x));
      (adtf_ddl::access_element::find_index(m_PointSampleFactory,
                                            cString("f32y"), m_ddlPointId.y));
    } else {
    LOG_INFO("No mediadescription for tPoint found!");
  }
  Register(m_oReaderWaypoint, "waypoint_in", pTypePointValue);

 object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }
  Register(m_oReaderCarPosition, "carPosition_in", pTypePositionData);

  object_ptr<IStreamType> pTypeSignalValue;
  if
    IS_OK(
        adtf::mediadescription::ant::
            create_adtf_default_stream_type_from_service(
                "tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)) {
      (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
                                            cString("ui32ArduinoTimestamp"),
                                            m_ddlSignalValueId.timeStamp));
      (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
                                            cString("f32Value"),
                                            m_ddlSignalValueId.value));
    } else {
    LOG_INFO("No mediadescription for tSignalValue found!");
  }

  Register(m_oReaderCalibratedValue, "calibratedValue", pTypeSignalValue);
  Register(m_oWriterSteer, "steerout", pTypeSignalValue);
  Register(m_oWriterRequestCalibration, "calibrationOut", pTypeSignalValue);

  RegisterPropertyVariable("car position x  (m)", m_propxStart);
  RegisterPropertyVariable("car position y  (m)", m_propyStart);
  RegisterPropertyVariable("goal position x (m)", m_propxGoal);
  RegisterPropertyVariable("goal position y (m)", m_propyGoal);
  RegisterPropertyVariable("kP scale (%)", m_propScaleKp);
  RegisterPropertyVariable("kD scale (%)", m_propScaleKd);
  RegisterPropertyVariable("kI scale (%)", m_propScaleKi);
}

// implement the Configure function to read ALL Properties
tResult cPurePursuitSteering::Configure() {
  // Testing values configurable via properties, will be overriden
  // once input at m_oReaderPoint is received
  lookahead_point_.f32x = m_propxGoal;
  lookahead_point_.f32y = m_propyGoal;

  RETURN_NOERROR;
}


tPosition toGlobal(tPosition carPosRear, float local_x, float local_y, float x_offset = 0,
                                 float y_offset = 0, float scale = 1) {
                                     float heading = carPosRear.f32heading;
                float x = local_x * scale * sin(heading)
                          + local_y * scale * cos(heading) + carPosRear.f32x;

                float y = -local_x * scale * cos(heading)
                          + local_y * scale * sin(heading) + carPosRear.f32y;

                tPosition result;
                result.f32x = x + x_offset * cos(heading);
                result.f32y = y + y_offset * sin(heading);
                result.f32heading = heading;
                return result;
            }



tResult cPurePursuitSteering::Process(tTimeStamp tmTimeOfTrigger) {
  object_ptr<const ISample> pReadWaypoint;

  if (IS_OK(m_oReaderWaypoint.GetLastSample(pReadWaypoint))) {
      auto oDecoder = m_PointSampleFactory.MakeDecoderFor(*pReadWaypoint);

      RETURN_IF_FAILED(oDecoder.IsValid());

      tPoint waypoint;

      RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPointId.x, &waypoint.f32x));
      RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPointId.y, &waypoint.f32y));

      lookahead_point_.f32x = fabs(waypoint.f32x);
      lookahead_point_.f32y = fabs(waypoint.f32y);
      ConvertToLocal(lookahead_point_, carPosition);
      //TransmitProportionalSteering(lookahead_point_, carPosition, tmTimeOfTrigger);
  }

  object_ptr<const ISample> pReadCarPos;

  if (IS_OK(m_oReaderCarPosition.GetLastSample(pReadCarPos))) {
      auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadCarPos);

      RETURN_IF_FAILED(oDecoder.IsValid());
        tPosition carPosRear;
      RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &carPosRear.f32x));
      RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &carPosRear.f32y));
      RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &carPosRear.f32heading));

        carPosition = toGlobal(carPosRear, 0, 0.36);

        //LOG_INFO("Pos: %f %f", carPosition.f32x, carPosition.f32y);
  }

  TransmitCurvature(tmTimeOfTrigger);

  object_ptr<const ISample> pReadCalibratedValue;
  if (IS_OK(m_oReaderCalibratedValue.GetLastSample(pReadCalibratedValue))) {
      auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadCalibratedValue);

        RETURN_IF_FAILED(oDecoder.IsValid());

         tSignalValue steerSignal;

        // retrieve the values (using convenience methods that return a variant)
        // RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value,
        //   &speedSignal.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timeStamp,
            &steerSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value,
            &steerSignal.f32Value));

        //LOG_INFO(cString::Format("SteerSignalValue: %f", tFloat32(steerSignal.f32Value)));
        /*
        float delta_steer = fabs(steerSignal.f32Value - lastSteer);

        if (steerSignal.f32Value > 80) {
            steerSignal.f32Value = 80;
        } else if (steerSignal.f32Value < -80) {
            steerSignal.f32Value = -80;
        }

        if (delta_steer > 70) {
            steerSignal.f32Value = lastSteer;
         } else if (delta_steer > 30) {
            steerSignal.f32Value = lastSteer+(steerSignal.f32Value - lastSteer)/2;
        }
        */
       //steerSignal.f32Value *= 2;
       TransmitSteer(steerSignal);
  }

  RETURN_NOERROR;
}

float cPurePursuitSteering::CalculateCurvature(tPoint wp, tPosition pos){

     double alpha = atan2(wp.f32y - pos.f32y*10, wp.f32x - pos.f32x*10) - pos.f32heading;
    //velocity
    //if v < 0:  # back
    //    alpha = M_PI - alpha

    //Lf = k * state.v + Lfc

    double curvature = atan2(2.0 * wheel_base * sin(alpha) / lookahead_dist, 1.0);
    //LOG_INFO(cString::Format("WP: x: %f y: %f",wp.f32x, wp.f32y));
    //LOG_INFO(cString::Format("WP: x: %f y: %f, carpos: x: %f y: %f | l_dist: %f base: %f", wp.f32x, wp.f32y, pos.f32x*10, pos.f32y*10, lookahead_dist, wheel_base));
    return static_cast<float>(curvature);
}


void cPurePursuitSteering::TransmitProportionalSteering(tPoint wp, tPosition pos, tTimeStamp tmTimeOfTrigger) {

    tSignalValue steerSignal;
    //float kp = 1.9;
    //float kd = 0.8;
    //float ki = 0;


    //LOG_INFO(cString::Format("triggertime: %d | lasttrigger: %d", tmTimeOfTrigger, lastUpdate));
    //float dt = (tFloat64)(tmTimeOfTrigger - lastUpdate)*1e-6;

    //(kp*(74.603*(wp.f32x*wp.f32x)-184.83)*wp.f32x);

    //LOG_INFO(cString::Format("time of trigger: %d", tmTimeOfTrigger));
    //LOG_INFO(cString::Format("last update: %d", lastUpdate));
    float dt = 0.015;
    float error = lastX-wp.f32x;
    float delta_magic = m_propScaleKd*(error)/dt;
    float steer = -1*(m_propScaleKp*100*wp.f32x);
    errorSum -= error*dt;

    steerSignal.f32Value = steer - delta_magic - m_propScaleKi*errorSum*1e3;
    //LOG_INFO(cString::Format("steer: %f | delta: %f | steerSignal: %f | ki*errorSum: %f", steer, delta_magic, steerSignal.f32Value, m_propScaleKi*errorSum));
    //LOG_INFO(cString::Format("wp.x: %f, steerout  ", wp.f32x,steerSignal.f32Value));
    if (steerSignal.f32Value > 100) {
        steerSignal.f32Value = 100;
    } else if (steerSignal.f32Value < -100) {
        steerSignal.f32Value = -100;
    }

    if (wp.f32y < 0) {
        steerSignal.f32Value = static_cast<tFloat32>(2.3*steerSignal.f32Value);
    }

    TransmitSteer(steerSignal);
    lastX = wp.f32x;
    lastUpdate = tmTimeOfTrigger;
}


void cPurePursuitSteering::ConvertToLocal(tPoint wp, tPosition pos){

    //w*cos((pi/2) -x)+t*sin((pi/2) -x)-p*cos((pi/2) -x)-r*sin((pi/2) -x)
//(M_PI/2)-
float shifted_heading_ = pos.f32heading;
wp.f32x /=SCALE_GLOBAL;
wp.f32y /=SCALE_GLOBAL;


float local_x_ = -1*(-wp.f32x * sin(shifted_heading_)+wp.f32y*cos(shifted_heading_)
                    +pos.f32x*sin(shifted_heading_)-pos.f32y*cos(shifted_heading_));

float local_y_ =  wp.f32x * cos(shifted_heading_)+wp.f32y*sin(shifted_heading_)
                    -pos.f32x*cos(shifted_heading_)-pos.f32y*sin(shifted_heading_);

//LOG_INFO(cString::Format("PPSCarPos X: %f Y: %f", pos.f32x, pos.f32y));
//LOG_INFO(cString::Format("PPSWP Recvd X: %f Y: %f", wp.f32x, wp.f32y));
//LOG_INFO(cString::Format("PPSWP Local X: %f Y: %f", local_x_, local_y_));

//cout << shifted_heading_ << ", " << sin(shifted_heading_)<< ", "  << cos(shifted_heading_)<< ", "   << wp.f32x << wp.f32y;

//cout<< "local xy: " << local_x_ << ", " <<local_y_ <<std::endl;
lookahead_point_.f32x = local_x_;
lookahead_point_.f32y = local_y_;
}



tResult cPurePursuitSteering::TransmitCurvature(tTimeStamp timeOfTrigger) {
  tSignalValue curvature;
  curvature.ui32ArduinoTimestamp = timeOfTrigger;

  float delta_lookahead_x_ = lookahead_point_.f32x;
  float delta_lookahead_y_ = lookahead_point_.f32y;
  if(lookahead_point_.f32y < 0){
      lookahead_point_.f32y = -lookahead_point_.f32y;
  }

  //cout << "PPS Local WP: " << delta_lookahead_x_ << " | "<< delta_lookahead_y_ << std::endl;

  int rescale = 1;


  tFloat32 curvature_out =
      (delta_lookahead_x_/rescale) / (0.5 * (pow(delta_lookahead_x_/rescale, 2.0) +
                                      pow(delta_lookahead_y_/rescale, 2.0)));


  //curvature_out = fmod(curvature_out, 2.0);
  curvature.f32Value = curvature_out;

  //curvature.f32Value = CalculateCurvature(lookahead_point_, carPosition);



  object_ptr<ISample> pWriteSamplest;
  RETURN_IF_FAILED(alloc_sample(pWriteSamplest)) {
    auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSamplest);
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, curvature.f32Value));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp,
                                            curvature.ui32ArduinoTimestamp));
  }


  //LOG_INFO(cString::Format("Curvature: %f X: %f Y: %f", tFloat32(curvature.f32Value), delta_lookahead_x_, delta_lookahead_y_));
  m_oWriterRequestCalibration << pWriteSamplest << flush << trigger;

  RETURN_NOERROR;
}

tResult cPurePursuitSteering::TransmitSteer(tSignalValue steerSignal) {
  object_ptr<ISample> pWriteSamplest;

  RETURN_IF_FAILED(alloc_sample(pWriteSamplest)) {
    auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSamplest);
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, -1*steerSignal.f32Value));
    RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timeStamp,
                                            steerSignal.ui32ArduinoTimestamp));
  }
  m_oWriterSteer << pWriteSamplest << flush << trigger;
  lastSteer = steerSignal.f32Value;
  RETURN_NOERROR;
}
