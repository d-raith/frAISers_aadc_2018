
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

#include "LaserSegmentation.h"

#include "ADTF3_OpenCV_helper.h"
#include "ADTF3_helper.h"

#include <math.h>


// parameters  --------------
tFloat32 camera_translation_x = 0.03;
tFloat32 camera_translation_y = 0.17;
tFloat32 camera_translation_z = 0.16;
// intrinsic matrix
// car 26(distorted) // car 27(distorted)
// tFloat32 focal_length_x = 584.2917310493169; // 585.5644470303048;
// tFloat32 focal_length_y = 583.7685601949312; // 584.8964075434083;
// tFloat32 intrinsic_off_x = 641.3844958728575; // 630.5897331003082;
// tFloat32 intrinsic_off_y = 481.43623388107613; // 477.7496476463654;
// car_26 (after undistort)
tFloat32 car_26_focal_length_x = 297.84613;
tFloat32 car_26_focal_length_y = 297.57944;
tFloat32 car_26_intrinsic_off_x = 642.7352;
tFloat32 car_26_intrinsic_off_y = 481.57596;
// car 27 (after undistort)
tFloat32 car_27_focal_length_x = 293.8884;
tFloat32 car_27_focal_length_y = 293.5531;
tFloat32 car_27_intrinsic_off_x = 621.8029;
tFloat32 car_27_intrinsic_off_y = 477.5697;
//----------------------------

// classes in segmentation image
const tUInt8 BACKGROUND = 0;
const tUInt8 ROAD = 1;
const tUInt8 CAR = 4;
const tUInt8 INTERSECTION = 9;
const tUInt8 PERSON = 10;

// classes for transmitting (TM)
const int TM_BACKGROUND = 0;
const Scalar background_color(255, 255, 255);
const int TM_ROAD = 1;
const Scalar road_color(0, 0, 255);
const int TM_CAR = 4;
const Scalar car_color(255, 0, 0);
const int TM_INTERSECTION = 9;
const Scalar intersection_color(0, 255, 0);
const int TM_PERSON = 10;
const Scalar person_color(255, 255, 0);
const int TM_NOT_ON_IMAGE = 11;
const int TM_UNCLEAR = 12;
const int TM_CHILD = 13;
const int TM_ADULT = 20;
const Scalar unclear_color(0, 255, 255);

// global variable for dataset naming
int dataset_filenumber = 0;

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "LaserSegmentation",
    cLaserSegmentation,
    adtf::filter::pin_trigger({"input"}));


cLaserSegmentation::~cLaserSegmentation() {
    thread_pool->shutdown();
}

cLaserSegmentation::cLaserSegmentation() {
    // create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType =
        adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    // Register input pin
    Register(m_oReader, "input", pType);
    // Register output pin
    Register(m_oWriter, "output", pType);

    laser_seg_output.registerPin(LaserSegPin::RW::WRITE, this, &(this->m_pClock), "laser_seg_out");

    // register callback for type changes
    m_oReader.SetAcceptTypeCallback(
        [this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType)
        -> tResult {
            return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
        });


    // DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service
        ("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
            cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
            cString("f32Value"), m_ddlSignalValueId.value));
    } else {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    object_ptr<IStreamType> pTypeLSData;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pTypeLSData, m_LSStructSampleFactory))
    {
        LOG_INFO("Found mediadescription for LaserScannerData:");
        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory, "ui32Size", m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory, "tScanArray", m_ddlLSDataId.scanArray));
    }
    else {
        LOG_INFO("No mediadescription for tLaserScannerData found!");
    }
    Register(m_oInputLaserScanner, "laser_scanner" , pTypeLSData);

    RegisterPropertyVariable("car_number [int]", car_number);
    RegisterPropertyVariable("scaling [float]", scaling);
    RegisterPropertyVariable("CropWidthOffset [pix]", CropWidthOffset);
    RegisterPropertyVariable("CropHeightOffset [pix]", CropHeightOffset);


    RegisterPropertyVariable("segmentation_channel [0, 1 or 2]", segmentation_channel);
    RegisterPropertyVariable("scanpoint_roi [int]", scanpoint_roi);
    RegisterPropertyVariable("scanpoint_roi_debug [bool]", scanpoint_roi_debug);
    RegisterPropertyVariable("background_threshold [float]", background_threshold);
    RegisterPropertyVariable("road_threshold [float]", road_threshold);
    RegisterPropertyVariable("car_threshold [float]", car_threshold);
    RegisterPropertyVariable("intersection_threshold [float]", intersection_threshold);
    RegisterPropertyVariable("person_threshold [float]", person_threshold);
    RegisterPropertyVariable("scan_max_range(car) [float]", scan_max_range_car);
    RegisterPropertyVariable("scan_max_range(person) [float]", scan_max_range_person);
    RegisterPropertyVariable("visualize_objects [bool]", visualize_objects);

    RegisterPropertyVariable("left_seperator [-deg]", left_seperator);
    RegisterPropertyVariable("right_seperator [deg]", right_seperator);
    RegisterPropertyVariable("enable debug output pin", enable_debug_output);

    RegisterPropertyVariable("dataset_generation_enable", dataset_generation_enable);
    RegisterPropertyVariable("dataset_generation_sirenon", dataset_generation_sirenon);

    // car (cropped) output pin
    object_ptr<IStreamType> pTypeNewVideo = make_object_ptr<cStreamType>(stream_meta_type_image());
    set_property(*pTypeNewVideo, stream_meta_type_image::FormatName, ADTF_IMAGE_FORMAT(RGB_24));
    set_property(*pTypeNewVideo, stream_meta_type_image::PixelWidth, new_video_width);
    set_property(*pTypeNewVideo, stream_meta_type_image::PixelHeight, new_video_height);
    set_property(*pTypeNewVideo, stream_meta_type_image::MaxByteSize, new_video_max_bytesize);
    Register(new_video_writer, "detection_out", pTypeNewVideo);
    raw_video_in.registerPin(VideoOutPin::RW::READ, this, &m_pClock, "raw_video_in", 600, 88, 3);
}


// implement the Configure function to read ALL Properties
tResult cLaserSegmentation::Configure() {
    // get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    if (static_cast<int>(car_number) == 26) {
        focal_length_x = car_26_focal_length_x;
        focal_length_y = car_26_focal_length_y;
        intrinsic_off_x = car_26_intrinsic_off_x;
        intrinsic_off_y = car_26_intrinsic_off_y;
    } else if (static_cast<int>(car_number) == 27) {
        focal_length_x = car_27_focal_length_x;
        focal_length_y = car_27_focal_length_y;
        intrinsic_off_x = car_27_intrinsic_off_x;
        intrinsic_off_y = car_27_intrinsic_off_y;
    } else {
        LOG_ERROR("LaserSegmentation: Invalid car number.");
    }

    thread_pool->init();

    RETURN_NOERROR;
}





tResult cLaserSegmentation::Process(tTimeStamp tmTimeOfTrigger) {

    if (!thread_pool->hasJobs()) {
        auto func = std::bind(&cLaserSegmentation::doThreadedProcess, this);
        thread_pool->submit(func);
    }

    RETURN_NOERROR;
}





tResult cLaserSegmentation::doThreadedProcess() {
    StopWatch watch;
    std::vector<std::vector<tFloat32>> scanCoordList_bp;
    object_ptr<const ISample> pReadSampleLS;
    std::vector<tPolarCoordiante> scan;

    //read value from laser scanner
    if (IS_OK(m_oInputLaserScanner.GetLastSample(pReadSampleLS))) {
        auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pReadSampleLS);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tSize numOfScanPoints = 0;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints));

        const tPolarCoordiante* pCoordinates =
                reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(m_ddlLSDataId.scanArray));

        tPolarCoordiante scanPoint;
        for (tSize i = 0; i < numOfScanPoints; ++i) {
            scanPoint.f32Radius = pCoordinates[i].f32Radius * 0.001;
            scanPoint.f32Angle = pCoordinates[i].f32Angle;
            scan.push_back(scanPoint);
        }
        // transform and backproject laser points
        TransformAndBackproject(scan, &scanCoordList_bp);
    }

    // read raw video
    cv::Mat raw_video;
    if(!IS_OK(raw_video_in.readData(&raw_video))) {
        LOG_WARNING("IS_OK(raw_video_in.readData(&raw_video)) == false");
    }

    // read segmentation video input and process
    vector<int> seg_list;
    object_ptr<const ISample> pReadSample;
    Mat outputImage;
    vector<vector<int>> detection_list;
    Mat outputImage2;
    while (IS_OK(m_oReader.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        // lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            // create a opencv matrix from the media sample buffer
            try {
                Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                    CV_8UC3,
                                    const_cast<unsigned char*>
                                    (static_cast<const unsigned char*>(pReadBuffer->GetPtr())) );

                // get pixel value from backprojected scan, and cluster it to object
                // getPixelValueFromScan(inputImage, scanCoordList_bp, scan, &outputImage);
                outputImage = inputImage;
                seg_list = getSegPerScanpoint(inputImage, scan, scanCoordList_bp, &outputImage);
                outputImage2 = outputImage;
                detection_list = highLevelDetection(raw_video, outputImage, scan, scanCoordList_bp, seg_list, &outputImage2);
                // makeStatisticAndSend(seg_list, detection_list, scan);
            } catch (cv::Exception ex) {
                LOG_ERROR("caught cv exception in laser seg proc");
                std::cout << "caught cv exception in laser seg proc:" << ex.what() << std::endl;
            }
        }
    }

    cv::addWeighted(outputImage, 0.5, raw_video, 0.5, 0, outputImage);

    // Write processed Image to Output Pin
    if (!outputImage.empty() && enable_debug_output) {
        // update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize) {
            setTypeFromMat(m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }

    /*
    // write LaserSegmentation results // This is done using LaserSegPin
    int LASER_SCANNER_COUNT = 360;
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, m_pClock->GetStreamTime())) {
        {
            auto oCodec = m_LSegStructSampleFactory.MakeCodecFor(pSample);
            oCodec.SetElementValue(m_ddlLSegDataId.size, scan.size());
            tPolarCoordiante* pCoordinates = reinterpret_cast<tPolarCoordiante*>(oCodec.GetElementAddress(m_ddlLSegDataId.scanArray));
            tUInt8* classes_pointer = reinterpret_cast<tUInt8*>(oCodec.GetElementAddress(m_ddlLSegDataId.classArray));
            // init array with zeros
            memset(pCoordinates, 0, LASER_SCANNER_COUNT * sizeof(tPolarCoordiante));
            memset(classes_pointer, 0, LASER_SCANNER_COUNT * sizeof(tUInt8));
            tUInt32 nIdx = 0;
            for (const tPolarCoordiante& pPolarCoordinate : scan) {
                pCoordinates[nIdx].f32Angle = pPolarCoordinate.f32Angle;
                pCoordinates[nIdx++].f32Radius = pPolarCoordinate.f32Radius;
            }
            nIdx = 0;
            for (const int& tm_class : seg_list) {
                classes_pointer[nIdx++] = static_cast<tUInt8>(tm_class);
            }
        }
    }
    m_OutputLaserSegmentation << pSample << flush << trigger;
     */

    watch.print_measurement("LaserSeg::doThreadedProcess", 20);
    RETURN_NOERROR;
}



void cLaserSegmentation::TransformAndBackproject(std::vector<tPolarCoordiante>& scan,
                                                    std::vector<std::vector<tFloat32>>* scanCoordList_bp){
    // extrinsic matrix (in meter)
    // assumption : no rotation
    tFloat32 extrinsic[4][4] = {
        {1, 0, 0, camera_translation_x},
        {0, 1, 0, camera_translation_y},
        {0, 0, 1, camera_translation_z},
        {0, 0, 0, 1}
    };

    tFloat32 skew = 0.0;
    tFloat32 intrinsic[3][4] = {
        {focal_length_x,    skew,            intrinsic_off_x,  0},
        {0,                 focal_length_y,  intrinsic_off_y,  0},
        {0,                 0,               1,                0}
    };

    // convert from polar to cartesian, convert to homogenous notation
    std::vector<std::vector<tFloat32>> scanCoordList;
    for (auto element : scan) {
        tFloat32 pz = element.f32Radius * cos(M_PI * element.f32Angle / 180);
        tFloat32 px = element.f32Radius * sin(M_PI * element.f32Angle / 180);
        std::vector<tFloat32> scanCoord = {px, 0.0, pz, 1.0}; //homogenous coord

        // multiply extrinsic matrix with each coordinate
        std::vector<tFloat32> scanCoord_ext;
        scanCoord_ext.resize(4,0);
        for (int i = 0; i < 4; i++){
            for (int j = 0; j < 4; j++){
                scanCoord_ext[i] += (extrinsic[i][j] * scanCoord[j]);
            }
        }

        tFloat32 last_el = scanCoord_ext[3];
        scanCoord_ext[0] = scanCoord_ext[0]/last_el;
        scanCoord_ext[1] = scanCoord_ext[1]/last_el;
        scanCoord_ext[2] = scanCoord_ext[2]/last_el;

        //for (auto el : scanCoord_ext){
        //    el = el / last_el;
        //}
        scanCoordList.push_back(scanCoord_ext);

    }

    // backprojection, multiply intrinsic matrix with each transformed coordinate
    for (auto coord_tr : scanCoordList) {
        std::vector<tFloat32> scanCoord_bp;
        scanCoord_bp.resize(4,0);
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 4; j++){
                scanCoord_bp[i] += (intrinsic[i][j] * coord_tr[j]);
            }
        }

        scanCoord_bp[0] = scanCoord_bp[0] / scanCoord_bp[2];
        scanCoord_bp[1] = scanCoord_bp[1] / scanCoord_bp[2];
        scanCoord_bp[2] = 1.0;

        scanCoord_bp[0] = scanCoord_bp[0] * static_cast<float>(scaling) - static_cast<float>(CropWidthOffset);
        scanCoord_bp[1] = scanCoord_bp[1] * static_cast<float>(scaling) - static_cast<float>(CropHeightOffset);

        scanCoordList_bp->push_back(scanCoord_bp);
    }
}



vector<int> cLaserSegmentation::getSegPerScanpoint(const cv::Mat& inputImage,
                               std::vector<tPolarCoordiante>& scan,
                               vector<vector<tFloat32>> scanCoordList_bp,
                               cv::Mat* outputImage) {
    vector<int> classification;
    try {
        cv::Mat channels[3];
        cv::split(inputImage, channels);
        cv::Mat segmentation_map = channels[static_cast<int>(segmentation_channel)];

        for (unsigned int i = 0; i < scanCoordList_bp.size(); i++) {
            // homogenious coordinate to standard 2D coordinate
            vector<tFloat32> scanCoord_bp = scanCoordList_bp[i];
            int projected_point_x = static_cast<int>(scanCoord_bp[0] / scanCoord_bp[2]);
            int projected_point_y = static_cast<int>(scanCoord_bp[1] / scanCoord_bp[2]);

            if (projected_point_x < 0
                    || projected_point_x >= inputImage.size().width
                    || projected_point_y < 0
                    || projected_point_y >= inputImage.size().height) {
                classification.push_back(TM_NOT_ON_IMAGE);
            } else {
                // calculate class via histogram
                // so first calculate the histogram over the region
                int num_background_pixels = 0;
                int num_road_pixels = 0;
                int num_car_pixels = 0;
                int num_intersection_pixels = 0;
                int num_person_pixels = 0;

                int roi_offset_x = max(projected_point_x - static_cast<int>(scanpoint_roi) / 2, 0);
                int roi_offset_y = max(projected_point_y - (static_cast<int>(scanpoint_roi * 2) / 2), 0);
                int roi_widht = min(static_cast<int>(scanpoint_roi), segmentation_map.size().width - roi_offset_x);
                int roi_height = min(static_cast<int>(scanpoint_roi), segmentation_map.size().height - roi_offset_y);
                cv::Mat roi = segmentation_map(cv::Rect(roi_offset_x,
                                                        roi_offset_y,
                                                        roi_widht,
                                                        roi_height));

                for (int k = 0; k < roi.size().height; k++) {
                    for (int l = 0; l < roi.size().width; l++) {
                        tUInt8 seg_pix = roi.at<tUInt8>(cv::Point(l, k));
                        if (seg_pix == BACKGROUND) {
                            num_background_pixels++;
                        } else if (seg_pix == ROAD) {
                            num_road_pixels++;
                        } else if (seg_pix == CAR) {
                            num_car_pixels++;
                        } else if (seg_pix == INTERSECTION) {
                            num_intersection_pixels++;
                        } else if (seg_pix == PERSON) {
                            num_person_pixels++;
                        }
                    }
                }
                tFloat32 roi_num_pix = static_cast<tFloat32>(roi.size().width * roi.size().height);
                tFloat32 background_ratio = static_cast<tFloat32>(num_background_pixels) / roi_num_pix;
                tFloat32 road_ratio = static_cast<tFloat32>(num_road_pixels) / roi_num_pix;
                tFloat32 car_ratio = static_cast<tFloat32>(num_car_pixels) / roi_num_pix;
                tFloat32 intersection_ratio = static_cast<tFloat32>(num_intersection_pixels) / roi_num_pix;
                tFloat32 person_ratio = static_cast<tFloat32>(num_person_pixels) / roi_num_pix;
                // check if scan point is still within max range
                tFloat32 scan_range = scan[i].f32Radius;

                // classify the scan point using ratios, the order of the checks matters!
                if ( scan_range > static_cast<tFloat32>(scan_max_range_car) ) {
                    classification.push_back(TM_UNCLEAR);
                } else if (car_ratio > static_cast<tFloat32>(car_threshold)
                    && scan_range <= scan_max_range_car) {
                    classification.push_back(TM_CAR);
                } else if (person_ratio > static_cast<tFloat32>(person_threshold)
                    && scan_range <= scan_max_range_person) {
                    classification.push_back(TM_PERSON);
                } else if (road_ratio > static_cast<tFloat32>(road_threshold)
                    && scan_range <= scan_max_range_car) {
                    classification.push_back(TM_ROAD);
                } else if (intersection_ratio > static_cast<tFloat32>(intersection_threshold)
                    && scan_range <= scan_max_range_car) {
                    classification.push_back(TM_INTERSECTION);
                } else if (background_ratio > static_cast<tFloat32>(background_threshold)
                    && scan_range <= scan_max_range_car) {
                    classification.push_back(TM_BACKGROUND);
                } else {
                    classification.push_back(TM_UNCLEAR);
                }
            }  // end get class via histogram
        }  // end for each point in scan

        // visualize result
        for (unsigned int i = 0; i < scanCoordList_bp.size(); i ++) {
            vector<tFloat32> scanCoord_bp = scanCoordList_bp[i];
            int x = static_cast<int>(scanCoord_bp[0] / scanCoord_bp[2]);
            int y = static_cast<int>(scanCoord_bp[1] / scanCoord_bp[2]);
            if (x < 0
                    || x >= inputImage.size().width
                    || y < 0
                    || y >= inputImage.size().height) {
                // no problemo just skip drawing
            } else {
                int mclass = classification[i];
                if (mclass == TM_BACKGROUND) {
                    cv::rectangle(*outputImage,
                                cv::Point(max(x - 1, 0), max(y - 1, 0)),
                                cv::Point(min(x + 1, outputImage->size().width), min(y + 1, outputImage->size().height)),
                                background_color,
                                cv::FILLED);
                    // outputImage->at<Vec3b>(cv::Point(x, y))[0] = background_color[0];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[1] = background_color[1];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[2] = background_color[2];
                } else if (mclass == TM_ROAD) {
                    cv::rectangle(*outputImage,
                                cv::Point(max(x - 1, 0), max(y - 1, 0)),
                                cv::Point(min(x + 1, outputImage->size().width), min(y + 1, outputImage->size().height)),
                                road_color,
                                cv::FILLED);
                    // outputImage->at<Vec3b>(cv::Point(x, y))[0] = road_color[0];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[1] = road_color[1];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[2] = road_color[2];
                } else if (mclass == TM_CAR) {
                    cv::rectangle(*outputImage,
                                cv::Point(max(x - 1, 0), max(y - 1, 0)),
                                cv::Point(min(x + 1, outputImage->size().width), min(y + 1, outputImage->size().height)),
                                car_color,
                                cv::FILLED);
                    // outputImage->at<Vec3b>(cv::Point(x, y))[0] = car_color[0];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[1] = car_color[1];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[2] = car_color[2];
                } else if (mclass == TM_INTERSECTION) {
                    cv::rectangle(*outputImage,
                                cv::Point(max(x - 1, 0), max(y - 1, 0)),
                                cv::Point(min(x + 1, outputImage->size().width), min(y + 1, outputImage->size().height)),
                                intersection_color,
                                cv::FILLED);
                    // outputImage->at<Vec3b>(cv::Point(x, y))[0] = intersection_color[0];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[1] = intersection_color[1];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[2] = intersection_color[2];
                } else if (mclass == TM_PERSON) {
                    cv::rectangle(*outputImage,
                                cv::Point(max(x - 1, 0), max(y - 1, 0)),
                                cv::Point(min(x + 1, outputImage->size().width), min(y + 1, outputImage->size().height)),
                                person_color,
                                cv::FILLED);
                    // outputImage->at<Vec3b>(cv::Point(x, y))[0] = person_color[0];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[1] = person_color[1];
                    // outputImage->at<Vec3b>(cv::Point(x, y))[2] = person_color[2];
                } else if (mclass == TM_UNCLEAR) {
                    cv::rectangle(*outputImage,
                                cv::Point(max(x - 1, 0), max(y - 1, 0)),
                                cv::Point(min(x + 1, outputImage->size().width), min(y + 1, outputImage->size().height)),
                                unclear_color,
                                cv::FILLED);
                    // outputImage->at<Vec3b>(M_PI * element.f32Angle / 180= unclear_color[0];
                    // outputImage->at<Vec3b>(M_PI * element.f32Angle / 180= unclear_color[1];
                    // outputImage->at<Vec3b>(M_PI * element.f32Angle / 180= unclear_color[2];
                }
                // draw roi for debugging
                if (scanpoint_roi_debug) {
                    int draw_roi_offset_x = max(x - static_cast<int>(scanpoint_roi) / 2, 0);
                    int draw_roi_offset_y = max(y - (static_cast<int>(scanpoint_roi * 2) / 2), 0);
                    int draw_roi_widht = min(static_cast<int>(scanpoint_roi), segmentation_map.size().width - draw_roi_offset_x);
                    int draw_roi_height = min(static_cast<int>(scanpoint_roi), segmentation_map.size().height - draw_roi_offset_y);

                    if (mclass != TM_UNCLEAR) {
                        cv::Point pt1(draw_roi_offset_x, draw_roi_offset_y);
                        cv::Point pt2(draw_roi_offset_x + draw_roi_widht, draw_roi_offset_y + draw_roi_height);
                        cv::rectangle(*outputImage,
                                    pt1,
                                    pt2,
                                    cv::Scalar(255, 255, 255),
                                    1);
                    }
                }
            }
        }
        if (visualize_objects) {
            for (int i = 0; i != outputImage->size().height; i++) {
                for (int j = 0; j != outputImage->size().width; j++) {
                    if (segmentation_map.at<tUInt8>(cv::Point(j, i)) == PERSON) {
                        outputImage->at<Vec3b>(cv::Point(j, i))[0] = person_color[0];
                        outputImage->at<Vec3b>(cv::Point(j, i))[1] = person_color[1];
                        outputImage->at<Vec3b>(cv::Point(j, i))[2] = person_color[2];
                    }

                    if (segmentation_map.at<tUInt8>(cv::Point(j, i)) == CAR) {
                        outputImage->at<Vec3b>(cv::Point(j, i))[0] = car_color[0];
                        outputImage->at<Vec3b>(cv::Point(j, i))[1] = car_color[1];
                        outputImage->at<Vec3b>(cv::Point(j, i))[2] = car_color[2];
                    }
                }
            }
        }


        return classification;
    } catch (cv::Exception ex) {
        LOG_ERROR("caught cv exception in getSegPerScanpoint");
        std::cout << "caught cv exception in getSegPerScanpoint:" << ex.what() << std::endl;
        return classification;
    }
}  // end getSegPerScanpoint(..)



void cLaserSegmentation::makeStatisticAndSend(vector<int> seg_list, vector<vector<int>> detection_list, vector<tPolarCoordiante> scan) {
    int car_count_left = 0;
    int car_count_middle = 0;
    int car_count_right = 0;
    int person_count = 0;
    for (unsigned int i = 0; i < seg_list.size(); i++) {
        if (seg_list[i] == TM_CAR) {
            if (scan[i].f32Angle < static_cast<tFloat32>((360 + static_cast<int>(left_seperator)))
                            && scan[i].f32Angle > 270.0) {
                car_count_left++;
            } else if (scan[i].f32Angle >= static_cast<tFloat32>((360 + static_cast<int>(left_seperator)))
                            || scan[i].f32Angle < static_cast<tFloat32>(static_cast<int>(right_seperator))) {
                car_count_middle++;
            } else if (scan[i].f32Angle >= static_cast<tFloat32>(static_cast<int>(right_seperator))
                            && scan[i].f32Angle <= 90.0)  {
                car_count_right++;
            }
        } else if (seg_list[i] == TM_PERSON) {
            person_count++;
        }
    }
    // done in obstacle filter
    /*if (car_count_left >= static_cast<int>(count_thresh_car)) {
        transmitBoolSignalValue(
            m_carfound_pin_left, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory_car_left,
            m_ddlBoolSignalValueId_car_left.ui32ArduinoTimestamp, 0,
            m_ddlBoolSignalValueId_car_left.bValue, true);
    } else {
        transmitBoolSignalValue(
            m_carfound_pin_left, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory_car_left,
            m_ddlBoolSignalValueId_car_left.ui32ArduinoTimestamp, 0,
            m_ddlBoolSignalValueId_car_left.bValue, false);
    }
    if (car_count_middle >= static_cast<int>(count_thresh_car)) {
        transmitBoolSignalValue(
            m_carfound_pin_middle, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory_car_middle,
            m_ddlBoolSignalValueId_car_middle.ui32ArduinoTimestamp, 0,
            m_ddlBoolSignalValueId_car_middle.bValue, true);
    } else {
        transmitBoolSignalValue(
            m_carfound_pin_middle, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory_car_middle,
            m_ddlBoolSignalValueId_car_middle.ui32ArduinoTimestamp, 0,
            m_ddlBoolSignalValueId_car_middle.bValue, false);
    }
    if (car_count_right >= static_cast<int>(count_thresh_car)) {
        transmitBoolSignalValue(
            m_carfound_pin_right, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory_car_right,
            m_ddlBoolSignalValueId_car_right.ui32ArduinoTimestamp, 0,
            m_ddlBoolSignalValueId_car_right.bValue, true);
    } else {
        transmitBoolSignalValue(
            m_carfound_pin_right, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory_car_right,
            m_ddlBoolSignalValueId_car_right.ui32ArduinoTimestamp, 0,
            m_ddlBoolSignalValueId_car_right.bValue, false);
    }

    if (person_count > static_cast<int>(count_thresh_person)) {
        transmitBoolSignalValue(
            m_personfound_pin, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory_person,
            m_ddlBoolSignalValueId_person.ui32ArduinoTimestamp, 0,
            m_ddlBoolSignalValueId_person.bValue, true);
    } else {
        transmitBoolSignalValue(
            m_personfound_pin, m_pClock->GetStreamTime(), m_BoolSignalValueSampleFactory_person,
            m_ddlBoolSignalValueId_person.ui32ArduinoTimestamp, 0,
            m_ddlBoolSignalValueId_person.bValue, false);
    }*/
}

vector<vector<int>> cLaserSegmentation::highLevelDetection(const cv::Mat& rawImage,
                                                    const cv::Mat& outputImage,
                                                    std::vector<tPolarCoordiante>& scan,
                                                    vector<vector<tFloat32>> scanCoordList_bp,
                                                    const std::vector<int>& classification,
                                                    cv::Mat* outputImage2) {
    std::vector<std::vector<int>> object_prop_list;
    try {
        // return values
        vector<int> return_label;
        vector<tFloat32> return_radius;
        vector<tFloat32> return_angle;
        vector<int> return_roi_width;
        vector<int> return_roi_height;
        // simple clustering,
        // groups reading with same label as one object
        // and get its bottom-center point
        // (because laser readings is on below)
        std::vector<int> similarScanIndex_templist;
        std::vector<int> similarScanLabel_templist;
        std::vector<int> object_label;
        std::vector<int> object_index;

        int previousScanLabel = 15;
        tFloat32 previousScanDist = 0;
        for (unsigned int i = 0; i< classification.size(); i++) {
            //tFloat32 currentDist = scan[i].f32Radius;
            //tFloat32 distDiff = abs(currentDist - previousScanDist);
            // initialize
            if (i == 0) {
                similarScanIndex_templist.push_back(i);
                similarScanLabel_templist.push_back(classification[i]);
                previousScanLabel = classification[i];
                previousScanDist = scan[i].f32Radius;
                continue;
            }
            if ((classification[i] == previousScanLabel))  {
                // if next scan is same and the dist diff is not high,
                // add scan point to temp list
                similarScanIndex_templist.push_back(i);
                similarScanLabel_templist.push_back(classification[i]);
                previousScanLabel = classification[i];
                previousScanDist = scan[i].f32Radius;
            } else {
                // if not
                // get the temp length
                int temp_size =  similarScanIndex_templist.size();
                // get the middle index
                int temp_mid = floor(temp_size/2);
                // push mid as one object to the result, if road or person
                if (similarScanLabel_templist[temp_mid] == TM_CAR
                    || similarScanLabel_templist[temp_mid] == TM_PERSON) {
                    // object_label.push_back(similarScanLabel_templist[temp_mid]);
                    // object_index.push_back(similarScanIndex_templist[temp_mid]);
                    std::vector<int> object_prop;
                    object_prop.push_back(similarScanLabel_templist[temp_mid]);
                    object_prop.push_back(similarScanIndex_templist[temp_mid]);
                    if ( !object_prop_list.empty() ) {
                        // of last object is same with current object
                        // and its angle differences is not too high,
                        // merge
                        int last_obj_lab = object_prop_list.back()[0];
                        int curr_obj_lab = object_prop[0];
                        if ( last_obj_lab == curr_obj_lab ) {
                            int last_obj_idx = object_prop_list.back()[1];
                            int curr_obj_idx = object_prop[1];
                            float last_obj_ang = (scan[last_obj_idx].f32Angle) * M_PI / 180;
                            float curr_obj_ang = (scan[curr_obj_idx].f32Angle) * M_PI / 180;
                            if ( last_obj_ang > 4.69 ) {
                                last_obj_ang = last_obj_ang - (2*M_PI);
                            }
                            if ( curr_obj_ang > 4.69 ) {
                                curr_obj_ang = curr_obj_ang - (2*M_PI);
                            }
                            float max_ang_diff = 0;
                            if ( last_obj_lab == TM_CAR ) {
                                max_ang_diff = 0.78;  // ~45 degree
                            } else if ( last_obj_lab == TM_PERSON ) {
                                max_ang_diff = 0.34;  // ~20 degree
                            }
                            if ( abs(last_obj_ang - curr_obj_ang) < max_ang_diff ) {
                                int merge_mid_index = floor((last_obj_idx + curr_obj_idx)/2);
                                int merge_mid_label = classification[merge_mid_index];
                                if ( merge_mid_label == TM_CAR
                                    || merge_mid_label == TM_PERSON) {
                                    object_prop_list.back()[1] = merge_mid_index;
                                }
                            } else {
                                object_prop_list.push_back(object_prop);
                            }

                        } else {
                            object_prop_list.push_back(object_prop);
                        }
                    } else {
                        object_prop_list.push_back(object_prop);
                    }
                }
                // clear the temp list
                similarScanLabel_templist.clear();
                similarScanIndex_templist.clear();
                // create new cluster/object
                similarScanIndex_templist.push_back(i);
                similarScanLabel_templist.push_back(classification[i]);
                previousScanLabel = classification[i];
                previousScanDist = scan[i].f32Radius;
            }
        }  // end clustering

        // do only for detected object
        int detected_car_amt = 0;
        int detected_person_amt = 0;
        // loop for all detected object
        for (unsigned int i = 0; i < object_prop_list.size(); i++) {
            int obj_label = object_prop_list[i][0];
            int obj_index = object_prop_list[i][1];
            vector<tFloat32> obj_scanCoord_bp = scanCoordList_bp[obj_index];
            int obj_x_img = static_cast<int>(obj_scanCoord_bp[0] / obj_scanCoord_bp[2]);
            int obj_y_img = static_cast<int>(obj_scanCoord_bp[1] / obj_scanCoord_bp[2]);
            tFloat32 obj_rad = scan[obj_index].f32Radius;
            tFloat32 obj_ang = scan[obj_index].f32Angle;
            std::stringstream obj_name;
            // flood fill
            Rect obj_roi;
            Scalar obj_color2;
            if (obj_label == TM_CAR) {
                obj_color2 = Scalar(200, 0, 0);
            } else if (obj_label == TM_PERSON) {
                obj_color2 = Scalar(200, 200, 0);
            }
            floodFill(*outputImage2,
                        Point(obj_x_img, obj_y_img),
                        obj_color2,
                        &obj_roi,
                        Scalar(0, 0, 0),
                        Scalar(255, 255, 0));
            // approximate its height in meter
            int obj_roi_x = obj_roi.x;
            int obj_roi_y = obj_roi.y;
            int obj_roi_width_px = obj_roi.width;
            int obj_roi_height_px = obj_roi.height;
            const tFloat32 obj_height_m = 0.12; // 0.12 for child, 0.3 for adult
            const tFloat32 px_scale = 250.0; // from experiment, height(px)/height(m), 250 for person
            // tFloat32 person_rad_px = obj_rad * px_scale;
            tFloat32 obj_roi_height_m = obj_roi_height_px / px_scale;
            cv::rectangle(*outputImage2,
                            obj_roi,
                            cv::Scalar(255, 255, 255),
                            1);
            std::stringstream obj_roi_prop;
            obj_roi_prop << "Hpx:" << obj_roi_height_px << "|Hm:" << obj_roi_height_m;
            std::string obj_roi_prop_str = obj_roi_prop.str();
            putText(*outputImage2,               // image to draw on
                    obj_roi_prop_str,               // string to print
                    Point(obj_roi_x, obj_roi_y + obj_roi_height_px),  // string drawing origin
                    FONT_HERSHEY_SIMPLEX,       // font type
                    0.3,                        // font scale
                    Scalar(255, 255, 255),        // string color
                    1,                          // string thickness
                    8,                          // line type
                    false);
            // treatment per label
            if (obj_label == TM_CAR) {
                obj_name << "CAR";
                detected_car_amt++;
                prepForSirenDetection(rawImage, obj_roi);
            } else if (obj_label == TM_PERSON) {
                if (obj_roi_height_m < 0.18) {
                    obj_name << "PERSON, CHILD";
                    obj_label = TM_CHILD;
                } else {
                    obj_name << "PERSON, ADULT";
                    obj_label = TM_ADULT;
                }
                detected_person_amt++;
            }

            // return everything needed here
            return_label.push_back(obj_label);
            return_radius.push_back(obj_rad);
            return_angle.push_back(obj_ang);
            return_roi_width.push_back(obj_roi_width_px);
            return_roi_height.push_back(obj_roi_height_px);

            // print middle point of the object
            circle(*outputImage2,                    // image to draw on
                    Point(obj_x_img, obj_y_img),    // point to draw
                    5,                             // circle radius
                    Scalar(255, 255, 255),          // circle color
                    1,                     // circle thickness (negative: filled circle)
                    8,                              // line type
                    0);                             // shift

            // print object name
            obj_name << " radius: " << obj_rad << " angle: " << obj_ang;
            // obj_name << " x: " << obj_x_laser << " y: " << obj_y_laser;
            std::string obj_name_str = obj_name.str();
            putText(*outputImage2,               // image to draw on
                    obj_name_str,               // string to print
                    Point(obj_x_img, obj_y_img),  // string drawing origin
                    FONT_HERSHEY_SIMPLEX,       // font type
                    0.3,                        // font scale
                    Scalar(255, 255, 255),        // string color
                    1,                          // string thickness
                    8,                          // line type
                    false);                     // false = top-left

        }  // end drawing detected objects


        vector<tLaserSegStruct> output_data;
        for (uint i =0; i< return_label.size(); i++) {
            tLaserSegStruct data;
            data.f32Angle = static_cast<tFloat32>(return_angle[i]);
            data.f32Distance = static_cast<tFloat32>(return_radius[i]);
            data.i16Class = static_cast<tInt16>(return_label[i]);
            data.i32Width = static_cast<tInt32>(return_roi_width[i]);
            data.i32Height = static_cast<tInt32>(return_roi_height[i]);
            output_data.emplace_back(data);
        }
        if (!output_data.empty()) {
            laser_seg_output.writeData(output_data);
        }




        // print stats
        std::stringstream stats_print;
        stats_print << "CAR : " << detected_car_amt << " | ";
        stats_print << "PERSON : " << detected_person_amt;
        std::string stats_print2 = stats_print.str();
        putText(*outputImage2,               // image to draw on
                stats_print2,               // string to print
                Point(20, 20),               // string drawing origin
                FONT_HERSHEY_SIMPLEX,       // font type
                0.3,                        // font scale
                Scalar(255, 255, 255),        // string color
                1,                          // string thickness
                8,                          // line type
                false);                     // false = top-left

        return object_prop_list;

    } catch (cv::Exception ex) {
        LOG_ERROR("caught cv exception in highLevelDetection");
        std::cout << "caught cv exception in highLevelDetection:" << ex.what() << std::endl;
        return object_prop_list;
    }
} // end highLevelDetection(..)

void cLaserSegmentation::prepForSirenDetection(const cv::Mat& rawImage,
                    Rect car_roi) {
    try {
        int defined_size = 64;                    
        // PART 1 - resize the bounding box
        int roi_padding = 2; // per side
        // if height(after padding) more than raw image height
        // do not apply padding
        if ((car_roi.height + (roi_padding*2)) > rawImage.size().height){
            roi_padding = 0;
        }
        int car_roi_new_x, car_roi_new_y, car_roi_new_width, car_roi_new_height;
        // check the differences between width and height
        int width_diff = max(car_roi.height - car_roi.width, 0);
        int height_diff = max(car_roi.width - car_roi.height, 0);
        // make bounding box have the same ratio (1:1)
        car_roi_new_width = car_roi.width + width_diff + (roi_padding*2);
        car_roi_new_height = car_roi.height + height_diff + (roi_padding*2);
        car_roi_new_x = max(car_roi.x - ((width_diff/2) + roi_padding), 0);
        car_roi_new_y = max(car_roi.y - ((height_diff/2) + roi_padding), 0);

        // PART 2 - crop raw image using defined bbox
        // define new bbox
        Rect car_roi_new(car_roi_new_x,
                        car_roi_new_y,
                        car_roi_new_width,
                        car_roi_new_height);
        // crop 
        Mat cropped_rawImage = rawImage(car_roi_new);

        // PART 3 - resize to 64x64
        Mat out_rawImage = cropped_rawImage.clone();
        resize(cropped_rawImage, out_rawImage, Size(defined_size, defined_size));

        // PART 4 - save it to image (for dataset) or as output (for network)
        string sirenStat;
        if (dataset_generation_enable) {
            std::stringstream out_rawImage_filename;
            if (dataset_generation_sirenon){
                sirenStat = "sirenon";
            } else {
                sirenStat = "sirenoff";
            }
            string save_dir = "/home/aadc/Documents/dataset_siren/"; // TODO
            out_rawImage_filename << save_dir << sirenStat << "_";
            out_rawImage_filename << setw(4) << setfill('0') << dataset_filenumber << ".png";
            std::string out_rawImage_filename2 = out_rawImage_filename.str();
            cv::Mat out_rawImage_bgr;
            cv::cvtColor(out_rawImage, out_rawImage_bgr, cv::COLOR_RGB2BGR);
            imwrite(out_rawImage_filename2, out_rawImage_bgr);
            dataset_filenumber++;
        }

        // PART 5 - write its label and props to screen
        // and draw to screen
        // std::stringstream stats_print;
        // stats_print << dataset_filenumber << "|" << sirenStat;
        // std::string stats_print2 = stats_print.str();
        // putText(out_rawImage,               // image to draw on
        //         stats_print2,               // string to print
        //         Point(0, 0),               // string drawing origin
        //         FONT_HERSHEY_SIMPLEX,       // font type
        //         0.5,                        // font scale
        //         Scalar(255, 255, 255),        // string color
        //         1,                          // string thickness
        //         8,                          // line type
        //         false);                     // false = top-left
        // send to output pin
        // write out pin here
        //  create write buffer
        object_ptr<ISample> pWriteSample;

        if (IS_OK(alloc_sample(pWriteSample, m_pClock->GetStreamTime())))
        {
            object_ptr_locked<ISampleBuffer> pWriteBuffer;
            if (IS_OK(pWriteSample->WriteLock(pWriteBuffer, out_rawImage.cols * out_rawImage.rows * out_rawImage.channels())))
            {
                pWriteBuffer->Write(adtf_memory_buffer<void, tSize>((void*) out_rawImage.data,
                    out_rawImage.cols * out_rawImage.rows * out_rawImage.channels()));
            }
        }

        new_video_writer << pWriteSample << flush << trigger;
    } catch (cv::Exception ex) {
        LOG_ERROR("caught cv exception in prepForSirenDetection");
        std::cout << "caught cv exception in prepForSirenDetection:" << ex.what() << std::endl;
    }
} // end sirenDetection(..)





/*

    // logic section -----------------------
    // TODO : detect car is on the left or right
    // create left box
    // create right box
    // create front box (opposite lane)


    // p1-p2
    // |   |
    // p4-p3

    // z+
    // |__x+

    // x+__
    //     |
    //     z+

    // px, py, pz, 1
    // init

    int newImage_w = 200;
    int newImage_h = 200;
    Mat newImage = Mat::zeros( Size( newImage_w, newImage_h ), CV_8UC1 );


    // define points for contours in cartesian (laser space)
    // left box
    std::vector<std::vector<tFloat32>> left_p(4);
    left_p = {
        {20.0, 0, 100.0, 1},
        {70.0, 0, 100.0, 1},
        {70.0, 0, 70.0, 1},
        {20.0, 0, 70.0, 1}
    };

    std::vector<std::vector<tFloat32>> right_p(4);
    right_p = {
        {-40.0, 0, 100.0, 1},
        {-90.0, 0, 100.0, 1},
        {-90.0, 0, 70.0, 1},
        {-40.0, 0, 70.0, 1}
    };

    std::vector<std::vector<tFloat32>> front_p(4);
    front_p = {
        {20.0, 0, 100.0, 1},
        {60.0, 0, 100.0, 1},
        {60.0, 0, 150.0, 1},
        {20.0, 0, 100.0, 1}
    };

    // convert to Point
    std::vector<Point2f> left_vert;
    std::vector<Point2f> right_vert;
    std::vector<Point2f> front_vert;
    for (int jj = 0; jj < left_p.size(); jj++){
        // left
        tFloat32 l_x = (newImage_w/2) - (left_p[jj][0]/left_p[jj][3]);
        tFloat32 l_y = (left_p[jj][2]/left_p[jj][3]);
        left_vert.push_back(Point(l_x, l_y));
        // right
        tFloat32 r_x = (newImage_w/2) - (right_p[jj][0]/right_p[jj][3]);
        tFloat32 r_y = (right_p[jj][2]/right_p[jj][3]);
        right_vert.push_back(Point(r_x, r_y));
        // front
        tFloat32 f_x = (newImage_w/2) - (front_p[jj][0]/front_p[jj][3]);
        tFloat32 f_y = (front_p[jj][2]/front_p[jj][3]);
        front_vert.push_back(Point(f_x, f_y));

    }
    // draw points to square
    for( int j = 0; j < left_vert.size(); j++ ){
        line( newImage, left_vert[j],  left_vert[(j+1)%left_vert.size()], Scalar( 255 ), 3, 8 );
    }
    for( int j = 0; j < right_vert.size(); j++ ){
        line( newImage, right_vert[j],  right_vert[(j+1)%right_vert.size()], Scalar( 255 ), 3, 8 );
    }
    for( int j = 0; j < front_vert.size(); j++ ){
        line( newImage, front_vert[j],  front_vert[(j+1)%front_vert.size()], Scalar( 255 ), 3, 8 );
    }
    Mat newImage_copy = newImage.clone();

    // convert to contour
    vector<vector<Point2i>> contours;
    vector<Vec4i> hierarchy;
    findContours( newImage_copy, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);


    Mat newImage_drawing = Mat::zeros( newImage.size(), CV_8UC3 );
    // draw contours
    drawContours( newImage_drawing, contours, -1, Scalar(255,255,255), 1, 8, hierarchy );

    std::cout << "scan size: " << scan.size() << std::endl;

    // get the scan value ready to newImage
    std::vector<Point2i> scanPoint;
    for (int kk = 0; kk< scan.size(); kk++){
        // convert from polar to cartesian
        int scan_raw_x = static_cast<int>((newImage_w/2) - (scan[kk].f32Radius * cos(M_PI * scan[kk].f32Angle / 180)))*100;
        int scan_raw_y = static_cast<int>(scan[kk].f32Radius * sin(M_PI * scan[kk].f32Angle / 180))*100;

        if (scan_raw_x < 0
                || scan_raw_x > newImage_drawing.size().width-1
                || scan_raw_y < 0
                || scan_raw_y > newImage_drawing.size().height-1){
            continue;
        }

        else {
            // make it compatible with newImage
            Point2i scan_raw_point = Point2i( scan_raw_x , scan_raw_y );
            std::cout << "p: " << scan_raw_x << "   " << scan_raw_y << std::endl;
            // push it to scan_raw_point
            scanPoint.push_back(scan_raw_point);
            // POINT POLYGON TEST
            int pp_test = 0;
            std::cout << "contours size: " << contours.size() << std::endl;
            for (int i = 0; i < contours.size(); i++) {
                int pp_test = pointPolygonTest(contours[i], scan_raw_point, false );
                std::cout << "pp_test :" << pp_test << std::endl;
            }

            // draw scan value to newImage
            circle(newImage_drawing,                        // image to draw on
                    scan_raw_point,                     // point to draw
                    5,                                 // circle radius
                    Scalar(255, floor(255*pp_test), 255), // circle color
                    -1,                                  // circle thickness (negative: filled circle)
                    8,                                  // line type
                    0);                                 // shift

        }



    }

    // imwrite("/home/aadc/Desktop/test1.png", newImage_drawing);

    // write out pin here
    //   create write buffer
        object_ptr<ISample> pWriteSample;

        if (IS_OK(alloc_sample(pWriteSample, m_pClock->GetStreamTime())))
        {
            object_ptr_locked<ISampleBuffer> pWriteBuffer;
            if (IS_OK(pWriteSample->WriteLock(pWriteBuffer, newImage_drawing.cols * newImage_drawing.rows * newImage_drawing.channels())))
            {
                pWriteBuffer->Write(adtf_memory_buffer<void, tSize>((void*) newImage_drawing.data,
                    newImage_drawing.cols * newImage_drawing.rows * newImage_drawing.channels()));
            }
        }

        new_video_writer << pWriteSample << flush << trigger;

    // return hl_detection;

-------------------------

//
    // visualize
    //
    if (visualize == true) {
        // visualize junctions (optional also car and person)
        for (int i = 0; i != image->size().height; i++) {
            for (int j = 0; j != image->size().width; j++) {
                if (segmentation_map.at<tUInt8>(cv::Point(j, i)) == INTERSECTION) {
                    image->at<Vec3b>(cv::Point(j, i))[0] = intersection_color[0];
                    image->at<Vec3b>(cv::Point(j, i))[1] = intersection_color[1];
                    image->at<Vec3b>(cv::Point(j, i))[2] = intersection_color[2];
                }
            }
        }



void cLaserSegmentation::drawProjectedPoints(const Mat& inputImage,
                                            std::vector<std::vector<tFloat32>> scanCoordList_bp,
                                            const std::vector<tPolarCoordiante>& scan,
                                            Mat* outputImage) {
    *outputImage = inputImage;

    std::cout << "AddProjectedPoints" << std::endl;
    for (unsigned int i = 0; i < scanCoordList_bp.size(); i++) {
        vector<tFloat32> imageCoordinate = scanCoordList_bp[i];
        tPolarCoordiante polarCoordinate = scan[i];
        std::cout << imageCoordinate[0] << std::endl;
        std::cout << imageCoordinate[1] << std::endl;
        circle(*outputImage,            // image to draw on
            Point(imageCoordinate[0] / imageCoordinate[2], imageCoordinate[1] / imageCoordinate[2]),   // point to draw
            10,                         // circle radius
            Scalar(min(int(polarCoordinate.f32Radius / 3.0 * 255), 255), 0, 0),      // circle color
            1,                          // circle thickness (negative: filled circle)
            8,                          // line type
            0);                         // shift
    }
}




void cLaserSegmentation::getPixelValueFromScan(const Mat& inputImage,
                                            std::vector<std::vector<tFloat32>> scanCoordList_bp,
                                            const std::vector<tPolarCoordiante>& scan,
                                            Mat* outputImage) {
    *outputImage = inputImage;

    // returns vector contains tuples of label value and its distance
    std::vector<std::vector<tFloat32>> scanLabDistAng_List; // List of Label, Distance, Angle
    std::vector<std::vector<tFloat32>> scanLabDistAng_ClusteredList; // clustered into objects

    std::cout << "initialized" << std::endl;
    LOG_INFO("initialized");

    // convert pixel value to label value
    for (unsigned int i = 0; i < scanCoordList_bp.size(); i++) {
        std::vector<tFloat32> scanLabDistAng; // Label, Distance, Angle, + x & y
        scanLabDistAng.resize(5,0); // originally 3

        vector<tFloat32> imageCoordinate = scanCoordList_bp[i];
        tPolarCoordiante polarCoordinate = scan[i];
        // std::cout << "coord size" << imageCoordinate.size() << std::endl;

        // std::cout << "input image size : " << inputImage.size() << std::endl;
        // LOG_INFO("input image size : ", inputImage.size());

        tFloat32 image_w = inputImage.size().width;
        tFloat32 image_h = inputImage.size().height;

        // normalize the scan point
        tFloat32 scan_x = floor(imageCoordinate[0]/imageCoordinate[3]);
        tFloat32 scan_y = floor(imageCoordinate[1]/imageCoordinate[3]);

        // only allow projected coordinate that still within image size to avoid segfault
        if ((scan_x < image_w) && (scan_y < image_h)){

            // draw circle for projected scan points
            circle(*outputImage,                    // image to draw on
                Point(scan_x, scan_y),   // point to draw
                10,                                 // circle radius
                Scalar(min(int(polarCoordinate.f32Radius / 3.0 * 255), 255), 0, 0),      // circle color
                2,                                  // circle thickness (negative: filled circle)
                8,                                  // line type
                0);                                 // shift


            // get pixel value
            Vec3b pxvalue = inputImage.at<Vec3b>(Point(scan_x,scan_y));
            tFloat32 pxvalue_r = pxvalue.val[0];
            tFloat32 pxvalue_g = pxvalue.val[1];
            tFloat32 pxvalue_b = pxvalue.val[2];


            // show the color of pixel as sequence of colored rectangles
            tFloat32 img_height = outputImage->size().height;
            tFloat32 img_width = outputImage->size().width;
            tFloat32 rect_width = (img_width/scanCoordList_bp.size())-1;
            rectangle(*outputImage,
                    Point(floor(i*rect_width),floor(0.95*img_height-1)),            // p1
                    Point(floor(rect_width + (i*rect_width)),floor(img_height-1)),  // p2
                    Scalar(pxvalue_r, pxvalue_g, pxvalue_b),                        // color, from pixel value
                    -1                                                              // filled rectangle
                    );


            // ignore every label that more than defined detection radius
            // default = 2 meters to 0.2 meters
            tFloat32 detectionRange_max = tFloat32(m_propMaxScanRange); // 2.0;
            tFloat32 detectionRange_min = tFloat32(m_propMinScanRange); // 0.2;
            // std::cout << "bp size :" << scanCoordList_bp.size() << std::endl;
            // std::cout << "scan size :"<< scan.size() << std::endl;

            // if still in scan range, detect its label
            if ((polarCoordinate.f32Radius <= detectionRange_max) && (polarCoordinate.f32Radius >= detectionRange_min)){
                // TODO : analyze ROI around scan point
                // sampling area
                tInt sampling_x = scan_x;
                tInt sampling_y = scan_y - 5;
                tInt sampling_w = 1;
                tInt sampling_h = 5;
                // ROI
                Mat sampling_ROI (inputImage, Rect(sampling_x, sampling_y, sampling_w, sampling_h));

                // do the histogram analysis
                std::vector<Mat> bgr_planes;
                split(sampling_ROI, bgr_planes);
                tInt histSize = 256;
                tFloat32 range = {0, 256};
                const tFloat32* histRange = { range };
                bool uniform = true;
                bool accumulate = false;
                Mat b_hist, g_hist, r_hist;
                calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, uniform, accumulate);
                calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, uniform, accumulate);
                calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, uniform, accumulate);

                std::cout << "b_hist : " << b_hist :: std::endl;
                std::cout << "b_hist : " << b_hist :: std::endl;
                std::cout << "b_hist : " << b_hist :: std::endl;

                // do the histogram analysis #2
                int imgCount = 1;
                int dims = 3;
                const int sizes[] = {256,256,256};
                const int channels[] = {0,1,2};
                float r_Range[] = {0,256};
                float g_Range[] = {0,256};
                float b_Range[] = {0,256};
                const float *ranges[] = {r_Range,g_Range,b_Range};
                Mat mask = Mat();
                Mat all_hist;
                calcHist(&sampling_ROI, imgCount, channels, mask, all_hist, dims, sizes, ranges);

                std::cout << "all_hist : " << all_hist :: std::endl;

                // if yellow = person
                if (pxvalue_g == 255 && pxvalue_r == 255){
                    scanLabDistAng[0] = 5.0; // ID for person
                }
                // if red = car
                else if (pxvalue_r == 255){
                    scanLabDistAng[0] = 4.0; // ID for car
                }
                else {
                    scanLabDistAng[0] = 0.0; // ID for non-object
                }
            }
            // if not in scan range, just give it label = 0
            else {
                scanLabDistAng[0] = 0.0;
            }

            // set x & y. if valid fill it with projected point
            scanLabDistAng[3] = scan_x;
            scanLabDistAng[4] = scan_y;

        } else {
            scanLabDistAng[0] = 0.0;
            scanLabDistAng[3] = 0.0;
            scanLabDistAng[4] = 0.0;
        }
        // set radius and angle
        scanLabDistAng[1] = polarCoordinate.f32Radius;
        scanLabDistAng[2] = polarCoordinate.f32Angle;

        // push to list
        scanLabDistAng_List.push_back(scanLabDistAng);

    }

    std::cout << "scan point color converted to label" << std::endl;
    LOG_INFO("scan point color converted to label");


    // simple clustering,
    // groups reading with same label as one object
    // and get its bottom-center point
    // (because laser readings is on below)
    std::vector<std::vector<tFloat32>> similarScan_templist;
    tFloat32 previousScanLabel = 0.0;
    for (unsigned int i = 0; i< scanLabDistAng_List.size(); i++){
        //initialize
        if (i == 0){
            similarScan_templist.push_back(scanLabDistAng_List[i]);
            previousScanLabel = scanLabDistAng_List[i][0];
            continue;
        }
        if (scanLabDistAng_List[i][0] == previousScanLabel){
            // if next scan is same, add scan point to temp list
            similarScan_templist.push_back(scanLabDistAng_List[i]);
            previousScanLabel = scanLabDistAng_List[i][0];
        } else {
            // if not
            // get the temp length
            tInt temp_size =  similarScan_templist.size();
            // std::cout << "similarscantemp size : " << temp_size << std::endl;
            // LOG_INFO("similarscantemp size : ", temp_size);
            // get the middle index
            tInt temp_mid = floor(temp_size/2);
            // push mid as one object to the result
            scanLabDistAng_ClusteredList.push_back(similarScan_templist[temp_mid]);
            // clear the temp list
            similarScan_templist.clear();
            // create new cluster/object
            similarScan_templist.push_back(scanLabDistAng_List[i]);
            previousScanLabel = scanLabDistAng_List[i][0];
        }
    }

    std::cout << "label clustered" << std::endl;
    LOG_INFO("label clustered");

    std::cout << "valid object detected : " << scanLabDistAng_ClusteredList.size() << std::endl;
    LOG_INFO("valid object detected : " , scanLabDistAng_ClusteredList.size());

    for (auto res : scanLabDistAng_ClusteredList){
        std::cout << "L:" << res[0] << "|D:" << res[1] << "|A:" << res[2] << std::endl;
        LOG_INFO("L:%f|D:%f|A:%f", res[0], res[1], res[2]);
        // print middle point of the object
        circle(*outputImage,                    // image to draw on
                Point(res[3], res[4]),   // point to draw
                20,                                 // circle radius
                Scalar(255, 255, 255),      // circle color
                -1,                                  // circle thickness (negative: filled circle)
                8,                                  // line type
                0);                                 // shift

    }

    // TODO : detect child/adult
    // using ratio between distance to person with its height


}




*/