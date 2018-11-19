
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

#include "camera_transformations.h"  // CameraTransformations
#include "DijkstraGoalpointExtractor.h"
#include "../PinClasses/PointListPin.h"

#include "stdafx.h"

//*************************************************************************************************
#define CID_BIRDSEYETRANSFORM_DATA_TRIGGERED_FILTER "birdseye_transform.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;


/*! the main class of the open cv template. */
class BirdseyeTransform : public cTriggerFunction {
 private:
    // Pins
    /*! Reader of an InPin. */
    cPinReader m_oReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;
    /* Pin for the detected line points */
    PointListPin m_pointlistpin;
    /* pin to transmit bool indicating if an intersection is in reach */
    cPinWriter m_intersection_pin;
    /*! A bool signal value identifier. */
    struct tBoolSignalValueId {
        tSize ui32ArduinoTimestamp;
        tSize bValue;
    } m_ddlBoolSignalValueId;
    /*! The signal value sample factory */
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory;


    // Stream Formats
        /*! The input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    property_variable<cFilename> birdseyeTransFile = cFilename(cString(
        "/home/aadc/AADC/configuration_files/birdseyeTransFile.xml"));
    property_variable<cString> sensorName = cString("camera basler");

    // control pin quick implementation
    struct tSignalValueId {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    cPinReader m_oReaderControlSignal;
    cPinWriter m_oWriterControlSignal;
    
    property_variable<bool> extract_goalpoints = true;
    property_variable<bool> video_output = true;
    property_variable<bool> goalpoint_visualization = true;
    property_variable<int> segmentation_channel = 0;
    property_variable<int> cost_channel = 2;
    property_variable<int> goal_radius = 75;
    property_variable<unsigned int> start_position_y = 13;
    property_variable<unsigned int> select_each_xth_waypoint = 10;
    property_variable<unsigned int> intersection_region = 10;
    property_variable<float> intersection_threshold = 0.75;

    property_variable<float> scaling_factor = 0.5;
    property_variable<int> crop_offset_width = 20;
    property_variable<int> crop_offset_height = 222;
    property_variable<int> crop_width = 600;
    property_variable<int> crop_height = 88;
    property_variable<bool> enable_cropping = false;
    property_variable<bool> enable_scaling = false;

    property_variable<unsigned int> distance_to_intersection = 10;
    property_variable<int> point_shift_x = 0;


    CameraTransformations birdseye_transformation;
    DijkstraGoalpointExtractor goalpoint_extractor;

    void processImage(const cv::Mat& inputImage, cv::Mat* outputImage);

 public:
    /*! Default constructor. */
    BirdseyeTransform();


    /*! Destructor. */
    virtual ~BirdseyeTransform() = default;

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

    int readControlId();
    tResult writeControlId(const int& control_id);
};


//*************************************************************************************************
