
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

#include "BirdseyeTransform.h"

#include "ADTF3_OpenCV_helper.h"
#include "ADTF3_helper.h"

#include "../PinClasses/StopWatch.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_BIRDSEYETRANSFORM_DATA_TRIGGERED_FILTER,
                                    "Birdseye Transform",
                                    BirdseyeTransform,
                                    adtf::filter::pin_trigger({ "input" }));



BirdseyeTransform::BirdseyeTransform() {
    // create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType =
        adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    // Register input pin
    Register(m_oReader, "input", pType);
    // Register output pin
    Register(m_oWriter, "output", pType);
    // register PointListPin
    m_pointlistpin.registerPin(PointListPin::RW::WRITE,
                               this,
                               &m_pClock,
                               "PointList");

    // register callback for type changes
    m_oReader.SetAcceptTypeCallback(
        [this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType)
        -> tResult {
            return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
        });

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
            "tBoolSignalValue", pTypeBoolSignalValue, m_BoolSignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory,
            cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(m_BoolSignalValueSampleFactory,
            cString("bValue"), m_ddlBoolSignalValueId.bValue));
    } else {
        LOG_INFO("KeyboardRemote: No mediadescription for tBoolSignalValue found!");
    }
    Register(m_intersection_pin, "intersectionfound", pTypeBoolSignalValue);
    
    // float pin quick implementation
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
        "tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    } else {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }
    Register(m_oReaderControlSignal, "control_signal_in", pTypeSignalValue);
    Register(m_oWriterControlSignal, "control_signal_out", pTypeSignalValue);


    RegisterPropertyVariable("birdseyeTransFile", birdseyeTransFile);
    RegisterPropertyVariable("sensorName", sensorName);
    RegisterPropertyVariable("extract_goalpoints [bool]", extract_goalpoints);
    RegisterPropertyVariable("video_output [bool]", video_output);
    RegisterPropertyVariable("goalpoint_visualization [bool]", goalpoint_visualization);
    RegisterPropertyVariable("segmentation_channel [0, 1, 2]", segmentation_channel);
    RegisterPropertyVariable("cost_channel [0, 1, 2]", cost_channel);
    RegisterPropertyVariable("goal_radius [pix]", goal_radius);
    RegisterPropertyVariable("start_position_y [pix]", start_position_y);
    RegisterPropertyVariable("select_each_xth_waypoint [uint]", select_each_xth_waypoint);
    RegisterPropertyVariable("intersection_region [pix]", intersection_region);
    RegisterPropertyVariable("intersection_threshold [float]", intersection_threshold);
    RegisterPropertyVariable("distance_to_intersection [uint]", distance_to_intersection);
    RegisterPropertyVariable("point_shift_x [int]", point_shift_x);

    RegisterPropertyVariable("scaling_factor [float]", scaling_factor);
    RegisterPropertyVariable("crop_offset_width [pix]", crop_offset_width);
    RegisterPropertyVariable("crop_offset_height [pix]", crop_offset_height);
    RegisterPropertyVariable("crop_width [pix]", crop_width);
    RegisterPropertyVariable("crop_height [pix]", crop_height);
    RegisterPropertyVariable("enable_cropping [bool]", enable_cropping);
    RegisterPropertyVariable("enable_scaling [bool]", enable_scaling);
}

tResult BirdseyeTransform::Configure() {
    // get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    this->birdseye_transformation =
        CameraTransformations(string(cString(birdseyeTransFile)),
                              string(cString(sensorName)));
    this->goalpoint_extractor =
        DijkstraGoalpointExtractor(static_cast<int>(segmentation_channel),
                                   static_cast<int>(cost_channel),
                                   static_cast<int>(goal_radius),
                                   static_cast<unsigned int>(start_position_y),
                                   static_cast<unsigned int>(select_each_xth_waypoint),
                                   static_cast<unsigned int>(intersection_region),
                                   static_cast<float>(intersection_threshold),
                                   static_cast<unsigned int>(distance_to_intersection),
                                   static_cast<int>(point_shift_x));
    RETURN_NOERROR;
}

/* read control ids */
int BirdseyeTransform::readControlId() {
    object_ptr<const ISample> pSampleControl;
    if (IS_OK(m_oReaderControlSignal.GetNextSample(pSampleControl))) {
        tFloat32 f32Value = 0.0;

        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pSampleControl);

        oDecoder.IsValid();
        if IS_OK(oDecoder.GetElementValue(m_ddlSignalValueId.value, &f32Value)) {
            return static_cast<int>(f32Value);
        }
    }
    return -1;  // error state
}

/* send out control ids */
tResult BirdseyeTransform::writeControlId(const int& control_id) {
    RETURN_IF_FAILED(transmitSignalValue(
        m_oWriterControlSignal, m_pClock->GetStreamTime(),
        m_SignalValueSampleFactory,
        m_ddlSignalValueId.timeStamp, 0,
        m_ddlSignalValueId.value, static_cast<tFloat32>(control_id)));
    RETURN_NOERROR;
}

tResult BirdseyeTransform::Process(tTimeStamp tmTimeOfTrigger) {
    object_ptr<const ISample> pReadSample;
    Mat outputImage;

    while (IS_OK(m_oReader.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        // lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            // create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                 CV_8UC3,
                                 const_cast<unsigned char*>
                                    (static_cast<const unsigned char*>(pReadBuffer->GetPtr())) );

            // Do the image processing and copy to destination image buffer
            processImage(inputImage, &outputImage);
        }
    }


//    cv::line(outputImage, cv::Point(outputImage.size().width / 2, 0), cv::Point(outputImage.size().width / 2, outputImage.size().height - 1), cv::Scalar(255, 255, 255));

    // Write processed Image to Output Pin
    if (static_cast<bool>(video_output) == true) {
        if (!outputImage.empty()) {
            // update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize) {
                setTypeFromMat(m_oWriter, outputImage);
            }
            // write to pin
            writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
        }
    }

    RETURN_NOERROR;
}


void BirdseyeTransform::processImage(const cv::Mat& inputImage, cv::Mat* outputImage) {
    *outputImage = inputImage;
    if (static_cast<bool>(enable_scaling) == true) {
        cv::resize(*outputImage,
                    *outputImage,
                    cv::Size(),
                    static_cast<float>(scaling_factor),
                    static_cast<float>(scaling_factor),
                    cv::INTER_LINEAR);
    }
    if (static_cast<bool>(enable_cropping) == true) {
        cv::Rect crop_region = cv::Rect(static_cast<int>(crop_offset_width),
                                        static_cast<int>(crop_offset_height),
                                        static_cast<int>(crop_width),
                                        static_cast<int>(crop_height));
        *outputImage = (*outputImage)(crop_region);
    }

    birdseye_transformation.PixelToStreetImage(*outputImage, outputImage);
    if (static_cast<bool>(extract_goalpoints) == true) {
        vector<cv::Point> waypoints;
        bool intersection_found = goalpoint_extractor.getGoalPoints(outputImage,
                                        &waypoints,
                                        static_cast<bool>(goalpoint_visualization),
                                        readControlId());
        vector<cv::Point2f> transformed;
        birdseye_transformation.StreetImageToStreet(waypoints, &transformed);
        
        vector<tPoint> result;
        for (auto &wp : transformed) {
            tPoint pt;
            pt.x = wp.x;
            pt.y = wp.y;
            result.emplace_back(pt);
        }
        m_pointlistpin.writeData(result);
        transmitBoolSignalValue(
            m_intersection_pin, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory,
            m_ddlBoolSignalValueId.ui32ArduinoTimestamp, 0,
            m_ddlBoolSignalValueId.bValue, intersection_found);
    }
}
