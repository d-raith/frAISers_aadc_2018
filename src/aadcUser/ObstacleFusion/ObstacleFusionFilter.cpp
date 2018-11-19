
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

#include "ObstacleFusionFilter.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_OBSTACLE_FUSION_FILTER,
                                    "ObstacleFusionFilter",
                                    ObstacleFusionFilter,
                                    adtf::filter::timer_trigger(200000));


ObstacleFusionFilter::ObstacleFusionFilter() {
    ls_seg_input.registerPin(LaserSegPin::READ, this,
                             &(this->m_pClock),
                             "ls_seg_in");

    /*read_pin_person.registerPin(BoolSignalValuePin::RW::READ,
                                this,
                                &(this->m_pClock),
                                "det_person");
    read_pin_car_left.registerPin(BoolSignalValuePin::RW::READ,
                                  this,
                                  &(this->m_pClock),
                                  "det_car_left");
    read_pin_car_right.registerPin(BoolSignalValuePin::RW::READ,
                                   this,
                                   &(this->m_pClock),
                                   "det_car_right");
    read_pin_car_front.registerPin(BoolSignalValuePin::RW::READ,
                                   this,
                                   &(this->m_pClock),
                                   "det_car_front");
*/

    obs_data_out.registerPin(DetectionInfoPin::RW::WRITE, this,
                             &(this->m_pClock),
                             "obs_data_out");

    ls_seg_out.registerPin(LaserSegPin::WRITE, this,
                           &(this->m_pClock),
                           "ls_seg_out");

    registerUsPin();

    RegisterPropertyVariable("left_seperator [-deg]", th_car_left);
    RegisterPropertyVariable("right_seperator [deg]", th_car_right);
    RegisterPropertyVariable("class identifier car", class_car_id);
    RegisterPropertyVariable("class identifier person", class_person_id);
    RegisterPropertyVariable("class identifier child", class_child_id);
    RegisterPropertyVariable("obstacle_distance [m]", obstacle_distance);
    
    RegisterPropertyVariable("ultrasonic obstacle hit threshold (m, <=)", us_threshold_hit);
    RegisterPropertyVariable("ultrasonic hitcount reset threshold (#)", hit_count_reset_us);
    RegisterPropertyVariable("segmentation hitcount reset threshold (#)", hit_count_reset_ls_seg_bool);
}


// implement the Configure function to read ALL Properties
tResult ObstacleFusionFilter::Configure() {
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    us_hit_counts = make_shared<UltrasonicHitCount>(
            static_cast<float>(us_threshold_hit),
            static_cast<int>(hit_count_reset_us));

    laser_seg_bool_hit_counts = make_shared<LaserSegBoolHitCount>(
            static_cast<int>(hit_count_reset_ls_seg_bool));


    RETURN_NOERROR;
}

tResult ObstacleFusionFilter::Process(tTimeStamp tmTimeOfTrigger) {
    StopWatch watch = StopWatch(true);
    vector<tLaserSegStruct> laser_seg_data;
    bool data_processed = false;
    while (IS_OK(ls_seg_input.readData(&laser_seg_data))) {
        processLsSegInput(&laser_seg_data);
        data_processed = true;
    }
    if(!data_processed) {
        processLsSegInput(&laser_seg_data);
    }

    watch.print_measurement("ls time", 5);
    watch.reset();

    tUltrasonicStruct us_data;

    while (IS_OK(readUltraSonics(&us_data))) {
        //LOG_INFO("us left: %f", us_data.tSideLeft.f32Value);
        processUsData(&us_data);
    }

    watch.print_measurement("us time", 5);
    watch.reset();


    ls_seg_out.writeData(laser_seg_data);


    tDetectionInfo obs_data;

    laser_seg_bool_hit_counts->applyTo(&obs_data);
    us_hit_counts->applyTo(&obs_data);
    //LOG_INFO("write: %f %f %f %f %f", obs_data.f32pCarLeft, obs_data.f32pCarCenter, obs_data.f32pCarRight, obs_data.f32pPerson, obs_data.f32pChild);
    obs_data_out.writeData(obs_data);

    RETURN_NOERROR;
}

void ObstacleFusionFilter::processLsSegInput(vector<tLaserSegStruct> *data) {
    bool car_left_found = false;
    bool car_center_found = false;
    bool car_right_found = false;
    bool person_found = false;
    bool child_found = false;

    for (auto &ls_obs : *data) {
        //LOG_INFO("angle: %f, dist: %f, class: %d, w: %d, h: %d", ls_obs.f32Angle, ls_obs
        //        .f32Distance, ls_obs.i16Class, ls_obs.i32Width, ls_obs.i32Height);

        float angle = ls_obs.f32Angle;
        float distance = ls_obs.f32Distance;
        if (distance <= static_cast<tFloat32>(obstacle_distance)) {
            if (ls_obs.i16Class == class_car_id) {
                if (angle < static_cast<float>((360 + th_car_left))
                        && angle > 270.0) {
                    car_left_found = true;
                } else if (angle >= static_cast<float>((360 + th_car_left))
                        || angle < static_cast<float>(th_car_right)) {
                    car_center_found = true;
                } else if (angle >= static_cast<float>(th_car_right)
                        && angle <= 90.0) {
                    car_right_found = true;
                }
            } else if (ls_obs.i16Class == class_person_id) {
                person_found = true;

            } else if (ls_obs.i16Class == class_child_id) {
                child_found = true;
            }
        }
    }

    laser_seg_bool_hit_counts->car_left.update(car_left_found);
    laser_seg_bool_hit_counts->car_center.update(car_center_found);
    laser_seg_bool_hit_counts->car_right.update(car_right_found);
    laser_seg_bool_hit_counts->person_detected.update(person_found);
    laser_seg_bool_hit_counts->child_detected.update(child_found);

}

void ObstacleFusionFilter::registerUsPin() {
    //the us struct
    object_ptr<IStreamType> pTypeUSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
            "tUltrasonicStruct", pTypeUSData, m_USDataSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideLeft") +
                                                                     cString(".ui32ArduinoTimestamp"),
                                              m_ddlUltrasonicStructIndex.SideLeft.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory,
                                              cString("tSideLeft") + cString(".f32Value"),
                                              m_ddlUltrasonicStructIndex.SideLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tSideRight") +
                                                                     cString(".ui32ArduinoTimestamp"),
                                              m_ddlUltrasonicStructIndex.SideRight.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory,
                                              cString("tSideRight") + cString(".f32Value"),
                                              m_ddlUltrasonicStructIndex.SideRight.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearLeft") +
                                                                     cString(".ui32ArduinoTimestamp"),
                                              m_ddlUltrasonicStructIndex.RearLeft.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory,
                                              cString("tRearLeft") + cString(".f32Value"),
                                              m_ddlUltrasonicStructIndex.RearLeft.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearCenter") +
                                                                     cString(".ui32ArduinoTimestamp"),
                                              m_ddlUltrasonicStructIndex.RearCenter.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory,
                                              cString("tRearCenter") + cString(".f32Value"),
                                              m_ddlUltrasonicStructIndex.RearCenter.value));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory, cString("tRearRight") +
                                                                     cString(".ui32ArduinoTimestamp"),
                                              m_ddlUltrasonicStructIndex.RearRight.timeStamp));
        (adtf_ddl::access_element::find_index(m_USDataSampleFactory,
                                              cString("tRearRight") + cString(".f32Value"),
                                              m_ddlUltrasonicStructIndex.RearRight.value));
    } else {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    Register(m_oInputUltrasonicUnit, "us_struct", pTypeUSData);
}

tResult ObstacleFusionFilter::readUltraSonics(tUltrasonicStruct *us_data) {
    object_ptr<const ISample> pSampleFromUS;

    RETURN_IF_FAILED(m_oInputUltrasonicUnit.GetNextSample(pSampleFromUS)) {
        auto oDecoderUS = m_USDataSampleFactory.MakeDecoderFor(*pSampleFromUS);

        RETURN_IF_FAILED(oDecoderUS.IsValid());

        //we do not need the timestamps here

        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideLeft.value,
                                                    &(us_data->tSideLeft.f32Value)));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.SideRight.value,
                                                    &(us_data->tSideRight.f32Value)));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearLeft.value,
                                                    &(us_data->tRearLeft.f32Value)));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearCenter.value,
                                                    &(us_data->tRearCenter.f32Value)));
        RETURN_IF_FAILED(oDecoderUS.GetElementValue(m_ddlUltrasonicStructIndex.RearRight.value,
                                                    &(us_data->tRearRight.f32Value)));

    }

    RETURN_NOERROR;
}

void ObstacleFusionFilter::processUsData(tUltrasonicStruct *us_data) {
    us_hit_counts->updateFromStruct(*us_data);
}
