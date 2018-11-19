
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
#pragma once  // same effect as header guard

#include <vector>
using std::vector;

#include "nonpin_types.h"  // Lane
#include "customtypes.h"  // tPoint
#include "camera_transformations.h"  // CameraTransformations
#include "line_points_detection.h"  // LinePointsDetection
#pragma push_macro("__FUNC__")  // store the adtf version of the makro
#undef __FUNC__  // avoid redefinition of adtf makro
#include "lane_finder.h"  // LaneFinder
#pragma pop_macro("__FUNC__")  // restore the makro to the adtf version
#include "stdafx.h"  // adtf, cv
using cv::Mat;
using cv::Scalar;
// using cv::Point2i;  // 2D integer point
using cv::Point2f;  // 2D float point
using adtf::ucom::object_ptr;
using adtf::filter::ant::cTriggerFunction;
using adtf::filter::ant::cPinReader;
using adtf::filter::ant::cPinWriter;
using adtf::base::property_variable;
using adtf_util::cFilename;
using adtf_util::cString;

#include "../PinClasses/PointListPin.h"
#include "../PinClasses/LaneListPin.h"
#include "../PinClasses/OpencvFramePin.h"
#include "../PinClasses/Threadpool.h"
#include "../PinClasses/AsyncSampleBuffer.h"


//*************************************************************************************************
#define CID_ADAPTIVELANEDETECTION_DATA_TRIGGERED_FILTER "adaptive_lane_detection.filter.user.aadc.cid"
/*
 * AdaptiveLaneDetection is the filter class to attempt to port the LaneDetection module from last year
 * to ADTF 3. The class
 * Author: Ben Wilhelm
 * Created: 07.06.2018
 * Based on: OpenCVTemplate (Audi),
 *           frAIburg_LaneDetection (Jan Bechtold, Felix Plum)
 */
class cAdaptiveLaneDetection : public cTriggerFunction {
 private:
    /*
     * Video pins
     */
    OpencvFramePin video_input_pin;
    OpencvFramePin video_input_dummy_pin;
    OpencvFramePin video_output_pin_lines;
    OpencvFramePin video_output_pin_lanes;

    //WorkerThread workerThread;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*
     * Other pins
     */
    PointListPin write_pin_pointlist;

    LaneListPin write_pin_lanelist;


    /*
     * Property variables registered with ADTF.
     * (They are automatically updated after they are registered)
     */
    property_variable<float> roi_offset_x = 0.2;
    property_variable<float> roi_offset_y = 0.1;
    property_variable<float> roi_width = 1.7;
    property_variable<float> roi_height = 1.1;
    property_variable<float> detection_distance = 0.04;
    property_variable<float> min_line_width = 0.02;
    property_variable<float> max_line_width = 0.06;
    property_variable<int> min_line_contrast = 50;
    property_variable<int> throttling_factor = 10;
    property_variable<bool> video_debug_enabled = true;
    property_variable<cFilename> birdseyeTransFile = cFilename(cString(
        "/home/aadc/AADC/configuration_files/birdseyeTransFile.xml"));
    property_variable<cString> sensorName = cString("camera basler");


    /* Variable to count calls to Process method */
    int64 videoFrameCounter = 0;
    /* Objetct to calculate image transformation to birdseye view */
    CameraTransformations transform_helper_;
    /*! Object for line point detection */
    LinePointsDetection points_detector_;
    /* Object for lane detection, given candidate points */
    LaneFinder lane_finder_;

    std::shared_ptr<ThreadPool> thread_pool;

    AsyncSampleBuffer<cv::Mat> video_out_buffer;



    /*!
     * Function to process the video frame.
     * Detects line points and visualizes them in outputImage.
     */
    void processVideo(const Mat& grayInputImage,
                      const Mat& inputImage,
                      Mat* outputImage_lines,
                      Mat* outputImage_lanes);

    /*! Function to visualize detected points in the outputImage */
    void visualizeDetections(vector<vector<tPoint>> detected_lines_v,
                             vector<vector<tPoint>> detected_lines_h,
                             vector<Lane> vert_lanes,
                             vector<Lane> horiz_lanes,
                             Mat* outputImage_linesm,
                             Mat* outputImage_lanes);

    /*! Function to visualize a set of lines.
        The color will be altered per line by mixing in green. */
    void visualizeLines(vector<vector<tPoint>> lines, Mat* outputImage, Scalar reference_color);

    /*! Function to transmit points on the output pin */
    tResult transmitPoints(vector<tPoint> points);

    /*! Function to transmit detected lanes on the output pin */
    tResult transmitLanes(const vector<Lane> &vert_lanes, const vector<Lane> &horiz_lanes);

    /*! Function to sample lanes for visualization. */
    void sampleLanes(std::vector<Lane>& vert_lanes,
                     const std::vector<Lane>& horiz_lanes,
                     std::vector<std::vector<tPoint> >* sampled_lanes);

    /*! Function to set up the LinePointsDetection object. */
    void setupLinePointsDetection();

    void ProcessVideoAsync(const Mat &inputImage);


 public:
    /*! Default constructor. */
    cAdaptiveLaneDetection();

    /*! Destructor. */
    ~cAdaptiveLaneDetection();
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
};


//*************************************************************************************************
