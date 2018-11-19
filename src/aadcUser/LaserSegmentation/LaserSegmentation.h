
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
#include <math.h>
#include "stdafx.h"
#include "../PinClasses/LaserSegPin.h"
#include "../PinClasses/StopWatch.h"
#include "../PinClasses/Threadpool.h"
#include "../PinClasses/VideoOutPin.h"


//*************************************************************************************************
#define CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER "laser_segmentation.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

class cLaserSegmentation : public cTriggerFunction {
 private:

    // Threading

    unique_ptr<ThreadPool> thread_pool = make_unique<ThreadPool>(1);

    // Pins

    LaserSegPin laser_seg_output;

    /*! Reader of an InPin. */
    cPinReader m_oReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;

    // Stream Formats
        /*! The input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! Media Descriptions. */
    struct tSignalValueId {
        tSize timeStamp;
        tSize value;
    } m_ddlSignalValueId;


    /*! The template data sample factory */
    adtf::mediadescription::cSampleCodecFactory m_SignalValueSampleFactory;

    struct ddlLaserScannerDataId {
        tSize size;
        tSize scanArray;
    }  m_ddlLSDataId;

    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;


    /*! A ddl laser scanner data identifier. */
    struct ddlLaserSegmentationId
    {
        tSize size;
        tSize scanArray;
        tSize classArray;
    } m_ddlLSegDataId;

    /*! The ls structure sample factory */
    adtf::mediadescription::cSampleCodecFactory m_LSegStructSampleFactory;

    // Reader pin for laser scanner
    cPinReader m_oInputLaserScanner;


    /*! A bool signal value identifier. *//*
    struct tBoolSignalValueId {
        tSize ui32ArduinoTimestamp;
        tSize bValue;
    };

    *//* carfoundpin left *//*
    cPinWriter m_carfound_pin_left;
    tBoolSignalValueId m_ddlBoolSignalValueId_car_left;
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory_car_left;
    *//* carfoundpin middle *//*
    cPinWriter m_carfound_pin_middle;
    tBoolSignalValueId m_ddlBoolSignalValueId_car_middle;
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory_car_middle;
    *//* carfoundpin right *//*
    cPinWriter m_carfound_pin_right;
    tBoolSignalValueId m_ddlBoolSignalValueId_car_right;
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory_car_right;

    *//* personfoundpin left *//*
    cPinWriter m_personfound_pin;
    tBoolSignalValueId m_ddlBoolSignalValueId_person;
    adtf::mediadescription::cSampleCodecFactory m_BoolSignalValueSampleFactory_person;*/


    tFloat32 focal_length_x;
    tFloat32 focal_length_y;
    tFloat32 intrinsic_off_x;
    tFloat32 intrinsic_off_y;

    // properties for translation in extrinsic matrix
    // property_variable<tFloat32> m_propX_Translation = tFloat32(0.0);
    // property_variable<tFloat32> m_propY_Translation = tFloat32(0.17);
    // property_variable<tFloat32> m_propZ_Translation = tFloat32(0.16);

    // properties for max and min scanning radius
    // property_variable<tFloat32> m_propMaxScanRange = tFloat32(2.0);
    // property_variable<tFloat32> m_propMinScanRange = tFloat32(0.2);

    property_variable<int> car_number = 26;
    property_variable<float> scaling = 0.5;
    property_variable<int> CropWidthOffset = 20;
    property_variable<int> CropHeightOffset = 222;

    property_variable<int> segmentation_channel = 0;
    property_variable<int> scanpoint_roi = 10;
    property_variable<bool> scanpoint_roi_debug = false;
    property_variable<tFloat32> background_threshold = 0.8;
    property_variable<tFloat32> road_threshold = 0.7;
    property_variable<tFloat32> car_threshold = 0.6;
    property_variable<tFloat32> intersection_threshold = 0.7;
    property_variable<tFloat32> person_threshold = 0.5;
    property_variable<int> count_thresh_car = 2;
    property_variable<int> count_thresh_person = 1;
    property_variable<int> left_seperator = -10;
    property_variable<int> right_seperator = 10;
    property_variable<tFloat32> scan_max_range_car = 2.0;
    property_variable<tFloat32> scan_max_range_person = 1.0;
    property_variable<bool> visualize_objects = false;

    property_variable<bool> enable_debug_output = false;

    property_variable<bool> dataset_generation_enable = false;
    property_variable<bool> dataset_generation_sirenon = false;

    // new video pin
    tInt new_video_width = 64;
    tInt new_video_height = 64;
    tInt new_video_channels = 1;
    tInt new_video_max_bytesize = new_video_channels * new_video_width * new_video_width;
    cSampleWriter new_video_writer;

    VideoOutPin raw_video_in;


    void TransformAndBackproject(std::vector<tPolarCoordiante>& scan,
                                std::vector<std::vector<tFloat32>>* scanCoordList_bp);
    void drawProjectedPoints(const Mat& inputImage,
                            std::vector<std::vector<tFloat32>> scanCoordList_bp,
                            const vector<tPolarCoordiante>& scan,
                            Mat* outputImage);

    void getPixelValueFromScan(const Mat& inputImage,
                            std::vector<std::vector<tFloat32>> scanCoordList_bp,
                            const vector<tPolarCoordiante>& scan,
                            Mat* outputImage);

    vector<int> getSegPerScanpoint(const cv::Mat& inputImage,
                                std::vector<tPolarCoordiante>& scan,
                                vector<vector<tFloat32>> scanCoordList_bp,
                                cv::Mat* outputImage);

    void makeStatisticAndSend(vector<int> seg_list,
                            vector<vector<int>> detection_list,
                            vector<tPolarCoordiante> scan);

    vector<vector<int>> highLevelDetection(const cv::Mat& rawImage,
                                        const cv::Mat& outputImage,
                                        std::vector<tPolarCoordiante>& scan,
                                        vector<vector<tFloat32>> scanCoordList_bp,
                                        const std::vector<int>& classification,
                                        cv::Mat* outputImage2);

    void prepForSirenDetection(const cv::Mat& outputImage,
                            Rect car_roi);

    tResult doThreadedProcess();

 public:
    /*! Default constructor. */
    cLaserSegmentation();

    /*! Destructor. */
    ~cLaserSegmentation();

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
