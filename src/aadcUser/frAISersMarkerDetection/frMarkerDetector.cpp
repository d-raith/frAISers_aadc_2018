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

#include "stdafx.h"
#include "frMarkerDetector.h"
#include "ADTF3_OpenCV_helper.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CMARKERDETECTOR_DATA_TRIGGERED_FILTER,
                                    "frAISers Marker Detector",
                                    cMarkerDetector,
                                    adtf::filter::pin_trigger({"input"}));





cMarkerDetector::~cMarkerDetector() {
    thread_pool->shutdown();
    thread_pool.reset();
}

cMarkerDetector::cMarkerDetector() {
    //Register Properties
    RegisterPropertyVariable("Calibration File", m_calibFile);
    RegisterPropertyVariable("Detector Parameter File", m_fileDetectorParameter);
    RegisterPropertyVariable("Marker Size [m]", m_f32MarkerSize);
    RegisterPropertyVariable("Marker Roi::x", m_MarkerX);
    RegisterPropertyVariable("Marker Roi::y", m_MarkerY);
    RegisterPropertyVariable("Marker Roi::width", m_MarkerWidth);
    RegisterPropertyVariable("Marker Roi::height", m_MarkerHeight);
    RegisterPropertyVariable("Show Roi", m_ShowMarkerRoi);
    RegisterPropertyVariable("Marker Roi::height", m_MarkerHeight);

    thread_pool = std::make_shared<ThreadPool>(1);



    //create and set inital input format type
    m_sInputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(
            stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sInputFormat);


    //register input pin
    Register(m_oReader, "input", pType);
    //register output pin
    Register(m_oImagePinWriter, "output", pType);

    //get the media description
    object_ptr<IStreamType> pTypePose;
    if IS_FAILED(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
            "tRoadSignExt",
            pTypePose, m_outputPoseSampleFactory)) {
        LOG_ERROR("Could not load media description for output pin road_sign_ext");
    } else {
        adtf_ddl::access_element::find_array_index(m_outputPoseSampleFactory, "af32TVec",
                                                   m_ddlRoadSignExtIds.tVec);
        adtf_ddl::access_element::find_array_index(m_outputPoseSampleFactory, "af32RVec",
                                                   m_ddlRoadSignExtIds.rVec);
        adtf_ddl::access_element::find_index(m_outputPoseSampleFactory, "i16Identifier",
                                             m_ddlRoadSignExtIds.id);
    }

    //register output pose pin
    Register(m_oPosePinWriter, "road_sign_ext", pTypePose);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback(
            [this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType> &pType) -> tResult {
                return ChangeType(m_oReader, m_sInputFormat, *pType.Get(), m_oImagePinWriter);
            });

    LOG_INFO("frAISers Maker Detector loaded");
}


tResult cMarkerDetector::Configure() {
    // Read Camera Calibration File
    cFilename fileCalibration = m_calibFile;
    adtf::services::ant::adtf_resolve_macros(fileCalibration);
    //check if calibration file with camera paramters exits
    if (fileCalibration.IsEmpty()) {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
    }
    if (!(cFileSystem::Exists(fileCalibration))) {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
    } else {
        // read the calibration file with camera paramters exits and save to member variable
        readCameraParameters(fileCalibration.GetPtr(), m_Intrinsics, m_Distorsion);
        m_bCamaraParamsLoaded = true;
    }


    //Aruco
    //create the detector params
    cFilename fileDetectorParameter = m_fileDetectorParameter;
    m_detectorParams = aruco::DetectorParameters::create();
    adtf::services::ant::adtf_resolve_macros(fileDetectorParameter);
    if (!(readDetectorParameters(fileDetectorParameter.GetPtr(), m_detectorParams))) {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter file not valid");
    }

    //set marker dictionary
    m_Dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    thread_pool->init();




    RETURN_NOERROR;
}


void cMarkerDetector::processMarkerDetectionAsync(tTimeStamp tmTimeOfTrigger, const Mat
&m_inputImage) {

    Mat outputImage;
    DetectionResult result;

    // the results from aruco detection
    vector<vector<Point2f> > corners, rejected;


    //Check for ROI settings and create ROI appropriately


    // doing the detection of markers in image

    cv::Mat intrinsics = m_Intrinsics.clone();

    m_MarkerRoiUsed = checkRoi();

    if (m_MarkerRoiUsed) {
        intrinsics.at<double>(0, 2)-= static_cast<tFloat32>(m_MarkerX);
        intrinsics.at<double>(1, 2)-= static_cast<tFloat32>(m_MarkerY);
    }



    if (m_MarkerRoiUsed) {
        //cv::Mat roiImage = m_inputImage(m_MarkerRoi);
        cv::Mat roiImage (m_inputImage, m_MarkerRoi);

        aruco::detectMarkers(roiImage, m_Dictionary, corners, result.ids, m_detectorParams,
                             rejected);
    } else {
        aruco::detectMarkers(m_inputImage, m_Dictionary, corners, result.ids, m_detectorParams,
                             rejected);
    }


    bool debug_draw = true;

    // draw detections
    //outputImage = m_inputImage.clone();
    outputImage = m_inputImage.clone();
    if (debug_draw) {

        if (m_ShowMarkerRoi) {
            rectangle(outputImage, m_MarkerRoi, Scalar(255), 10, 8, 0);
        }

        aruco::drawDetectedMarkers(outputImage, corners, result.ids);
    }


    // if we have the camera pararmeter available we calculate the pose
    if (m_bCamaraParamsLoaded && !result.ids.empty())
        aruco::estimatePoseSingleMarkers(corners, m_f32MarkerSize, intrinsics, m_Distorsion,
                                         result.rvecs,
                                         result.tvecs);

    if (m_bCamaraParamsLoaded && !result.ids.empty() && debug_draw) {
        for (unsigned int i = 0; i < result.ids.size(); i++) {
            aruco::drawAxis(outputImage, intrinsics, m_Distorsion, result.rvecs[i], result.tvecs[i],
                            m_f32MarkerSize * 0.5f);
        }
    }





    //write WITH markers to output pin
    if (!outputImage.empty() && debug_draw) {
        video_out_buffer.setData(outputImage);
    }
    if (!result.rvecs.empty() && !result.tvecs.empty() && !result.ids.empty()) {

        result_buffer.setData(result);


    }
}


tResult cMarkerDetector::Process(tTimeStamp tmTimeOfTrigger) {
    object_ptr<const ISample> pReadSample;

    if (IS_OK(m_oReader.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            //create a opencv matrix from the media sample buffer
            auto func = std::bind(&cMarkerDetector::processMarkerDetectionAsync, this,
                                  tmTimeOfTrigger,
                                  Mat(cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                                      CV_8UC3, (uchar *) pReadBuffer->GetPtr()).clone()
            );

            if(!thread_pool->hasJobs()){
                thread_pool->submit(func);
            }


        }

    }


    if(result_buffer.hasDataAvailable()){
        DetectionResult result = *result_buffer.getDataPtr();
        result_buffer.setDataAvailable(false);
        int n = 0;
        for (auto id : result.ids) {
            //create write buffer
            object_ptr<ISample> pWriteSample;

            if (IS_OK(alloc_sample(pWriteSample))) {
                auto oCodec = m_outputPoseSampleFactory.MakeCodecFor(pWriteSample);

                // get pointer to rotation vector in sample
                tFloat32 rVecFl32[3] = {static_cast<tFloat32>(result.rvecs[n][0]),
                                        static_cast<tFloat32>(result.rvecs[n][1]),
                                        static_cast<tFloat32>(result.rvecs[n][2])};
                auto *rVecSample = static_cast<tFloat32 *>(oCodec.GetElementAddress(
                        m_ddlRoadSignExtIds.rVec));
                memcpy(rVecSample, rVecFl32, sizeof(rVecFl32));

                // get pointer to translation vector in sample
                tFloat32 tVecFl32[3] = {static_cast<tFloat32>(result.tvecs[n][0]),
                                        static_cast<tFloat32>(result.tvecs[n][1]),
                                        static_cast<tFloat32>(result.tvecs[n][2])};
                auto *tVecSample = static_cast<tFloat32 *>(oCodec.GetElementAddress(
                        m_ddlRoadSignExtIds.tVec));
                memcpy(tVecSample, tVecFl32, sizeof(tVecFl32));

                if (IS_OK(oCodec.SetElementValue(m_ddlRoadSignExtIds.id, id))) {
                    // the sample buffer lock is released in the destructor of oCodec
                    m_oPosePinWriter << pWriteSample << flush << trigger;
                }
            }
            n++;
        }
    }


    if(video_out_buffer.hasDataAvailable()){

        cv::Mat outputImage = video_out_buffer.getDataPtr()->clone();
        video_out_buffer.setDataAvailable(false);
        //update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sInputFormat.m_szMaxByteSize) {
            setTypeFromMat(m_oImagePinWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oImagePinWriter, outputImage, m_pClock->GetStreamTime());
        video_out_buffer.setDataAvailable(false);
    }

    RETURN_NOERROR;
}

tBool cMarkerDetector::checkRoi() {
    tBool res = tTrue;

    // if width or heigt are not set ignore the roi
    if (static_cast<tFloat32>(m_MarkerWidth) == 0 ||
        static_cast<tFloat32>(m_MarkerHeight) == 0)
        return tFalse;

    //check if we are within the boundaries of the image
    res &= (static_cast<tFloat32>(m_MarkerX) + static_cast<tFloat32>(m_MarkerWidth)) <
           m_sInputFormat.m_ui32Width;
    res &= (static_cast<tFloat32>(m_MarkerY) + static_cast<tFloat32>(m_MarkerHeight)) <
           m_sInputFormat.m_ui32Height;


    if (res) {
        //create the rectangle
        m_MarkerRoi = cv::Rect2f(static_cast<tFloat32>(m_MarkerX), static_cast<tFloat32>(m_MarkerY),
                                 static_cast<tFloat32>(m_MarkerWidth),
                                 static_cast<tFloat32>(m_MarkerHeight));
    }

    return res;
}