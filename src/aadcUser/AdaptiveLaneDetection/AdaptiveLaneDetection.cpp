
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
#include "AdaptiveLaneDetection.h"
#include "../PinClasses/StopWatch.h"
using adtf::ucom::object_ptr_shared_locked;
using adtf::streaming::ant::IStreamType;
using adtf::streaming::ant::cStreamType;
using adtf::streaming::ant::stream_meta_type_image;
using adtf::streaming::ant::ISample;
using adtf::streaming::ant::ISampleBuffer;

using cv::Size;
using cv::Point2i;  // 2D integer point




ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_ADAPTIVELANEDETECTION_DATA_TRIGGERED_FILTER,
                                    "Adaptive Lane Detection",
                                    cAdaptiveLaneDetection,
                                    adtf::filter::pin_trigger({"input"}));


cAdaptiveLaneDetection::~cAdaptiveLaneDetection(){
    thread_pool->shutdown();
    thread_pool.reset();
}

cAdaptiveLaneDetection::cAdaptiveLaneDetection() {
    /*
     * Register video in/out pins
     */
    this->video_input_pin.registerPin(OpencvFramePin::RW::READ,
                                      this,
                                      &(this->m_pClock),
                                      "input");
    this->video_input_dummy_pin.registerPin(OpencvFramePin::RW::READ,
                                    this,
                                    &(this->m_pClock),
                                    "input_dummy");
    this->video_output_pin_lines.registerPin(OpencvFramePin::RW::WRITE,
                                      this,
                                      &(this->m_pClock),
                                      "lines_image");
    this->video_output_pin_lanes.registerPin(OpencvFramePin::RW::WRITE,
                                      this,
                                      &(this->m_pClock),
                                      "lanes_image");
    vector<OpencvFramePin*> outputPins;
    outputPins.push_back(&(this->video_output_pin_lines));
    outputPins.push_back(&(this->video_output_pin_lanes));
    this->video_input_pin.linkTypes(outputPins);

    /*
     * Register other pins
     */
    write_pin_pointlist.registerPin(PointListPin::RW::WRITE,
                                    this,
                                    &(this->m_pClock),
                                    "points");

    write_pin_lanelist.registerPin(LaneListPin::RW::WRITE,
                                this,
                                &(this->m_pClock),
                                "lanes");


    /*
     * Register property variables
     */
    // they are automatically updated after they are registered
    RegisterPropertyVariable("roi_offset_x [m]", roi_offset_x);
    RegisterPropertyVariable("roi_offset_y [m]", roi_offset_y);
    RegisterPropertyVariable("roi_width [m]", roi_width);
    RegisterPropertyVariable("roi_height [m]", roi_height);
    RegisterPropertyVariable("detection_distance [m]", detection_distance);
    RegisterPropertyVariable("min_line_width [m]", min_line_width);
    RegisterPropertyVariable("max_line_width [m]", max_line_width);
    RegisterPropertyVariable("min_line_contrast [int]", min_line_contrast);
    RegisterPropertyVariable("birdseyeTransFile", birdseyeTransFile);
    RegisterPropertyVariable("sensorName", sensorName);
    RegisterPropertyVariable("throttling factor [mod]", throttling_factor);
    RegisterPropertyVariable("video debug enabled [bool]", video_debug_enabled);

    thread_pool = std::make_shared<ThreadPool>(1);
}


tResult cAdaptiveLaneDetection::Configure() {
    // get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    setupLinePointsDetection();
    thread_pool->init();
    RETURN_NOERROR;
}





void cAdaptiveLaneDetection::ProcessVideoAsync(const Mat &inputImage) {
    try{
    Mat outputImage_lines;
    Mat outputImage_lanes;
    Mat grayInputImage;


    StopWatch watch;
    // convert to grayscale image
    if(inputImage.empty()){
        return;
    }
    cv::cvtColor(inputImage, grayInputImage, CV_RGB2GRAY);
    watch.print_measurement("LaneDet: cvtColor", 20);
    watch.reset();
    // Do the image processing and copy to destination image buffer
    // Canny(inputImage, outputImage, 100, 200);  // Detect Edges
    try{
        processVideo(grayInputImage, inputImage, &outputImage_lines, &outputImage_lanes);
        watch.print_measurement("LaneDet: processVideo", 60);
    } catch(...){
        LOG_WARNING("CV exception caught when processing lane information");
        return;
    }


    // write images to pins
    if( video_debug_enabled){
        video_out_buffer.setData(outputImage_lanes);
        this->video_output_pin_lines.writeData(outputImage_lines);
        //this->video_output_pin_lanes.writeData(outputImage_lanes);
    }
    } catch(...){

    }

}

tResult cAdaptiveLaneDetection::Process(tTimeStamp tmTimeOfTrigger) {

    StopWatch watch;
    object_ptr<const ISample> pReadSample;
    Mat inputImage;
    // read the input pin
    this->video_input_pin.readData(&inputImage);
    if (inputImage.empty()) {
        RETURN_NOERROR;
    }


    //ProcessVideoAsync(inputImage);
    //cAdaptiveLaneDetection::ProcessVideoAsync(inputImage);
    auto func = std::bind(&cAdaptiveLaneDetection::ProcessVideoAsync, this, inputImage);
    if(!thread_pool->hasJobs()) {
        thread_pool->submit(func);
    }


    if(video_out_buffer.hasDataAvailable()){
        video_output_pin_lanes.writeData(*video_out_buffer.getDataPtr());
        video_out_buffer.setDataAvailable(false);
    }

    watch.print_measurement("LaneDet: Process", 10);

    RETURN_NOERROR;
}




void cAdaptiveLaneDetection::processVideo(const cv::Mat& grayInputImage,
                                          const cv::Mat& inputImage,
                                          cv::Mat* outputImage_lines,
                                          cv::Mat* outputImage_lanes) {
    this->videoFrameCounter++;
    if (inputImage.empty()) {
        LOG_ERROR("LaneDetection Image empty");
        return;
    }
   // auto start = std::chrono::high_resolution_clock::now();
    // apply the warping into birds eye view to the inputImage
    this->transform_helper_.PixelToStreetImage(grayInputImage, outputImage_lines);
  //  std::chrono::duration<double> process_time =  std::chrono::high_resolution_clock::now()-start;
//    LOG_INFO(cString::Format("WarpPerspective took %f s", process_time.count()));

    // detect line points
    std::vector<tInt> detection_lines;  // stores the lines in the image where we search for lanes
    std::vector<std::vector<tPoint>> detected_lines_h;  // detected horizontal line points
    std::vector<std::vector<tPoint>> detected_lines_v;  // detected vertical line points
    //start = std::chrono::high_resolution_clock::now();
    points_detector_.DetectPoints(*outputImage_lines,
                                  &detected_lines_v,
                                  &detected_lines_h,
                                  &detection_lines);
    //process_time =  std::chrono::high_resolution_clock::now()-start;
    //LOG_INFO(cString::Format("DetectPoints took %f s", process_time.count()));
//  if (detected_lines_v.empty()) return;

    // detect lanes based on line detection
    std::vector<Lane> vert_lanes;        // vertical lanes
    std::vector<Lane> vert_lanes_temp;   // vertical lanes, not intended for use
    std::vector<Lane> horiz_lanes;       // horizontal lanes
    std::vector<Lane> horiz_lanes_temp;  // horizontal lanes, not intended for use
    // call lanefinder twice to get horizontal and vertical lines separately
    //start = std::chrono::high_resolution_clock::now();
    if (!lane_finder_.getLanes(detected_lines_v, &vert_lanes, &horiz_lanes_temp)) {
      return;
    }
    /*if (!lane_finder_.getLanes(detected_lines_h, &vert_lanes_temp, &horiz_lanes)) {
//      return;
    }*/

  //  process_time =  std::chrono::high_resolution_clock::now()-start;
//    LOG_INFO(cString::Format("laneFinder::GetLanes took %f s", process_time.count()));
//  if (lane_finder_.getLanes(detected_lines_h, &vert_lanes_temp,
//                                              &horiz_lanes)) {
//      tCrossing crossing;
//      tPath corner_points;
//      if (lane_finder_.detectCrossing(vert_lanes, horiz_lanes,
//              &crossing, &corner_points)) {
//              AddCornersToMap(corner_points);
//              TransmitCrossing(crossing, pSample->GetTime());
//      }
//  }

    // logging some parameters
    //if (this->videoFrameCounter <= 1) {
        //int imgDimsAfterWarp = outputImage_lines->dims;
        // if (imgDimsAfterWarp < 2) {
        //     LOG_INFO("outputImage is not allocated, or has less than two dimensions.");
        //     return;
        // }
        /*
        Size imgSizeAfterWarp = outputImage_lines->size();
        LOG_INFO(cString::Format("Debug: imgDimsAfterWarp %i", imgDimsAfterWarp));
        LOG_INFO(cString::Format("OutputImg Width=%i Height=%i",
                                imgSizeAfterWarp.width,
                                imgSizeAfterWarp.height));
        LOG_INFO(cString::Format("Input Img Width=%i Height=%i",
                                inputImage.size().width,
                                inputImage.size().height));*/
    //}
    //start = std::chrono::high_resolution_clock::now();

    // visualize
    if (video_debug_enabled){
    this->transform_helper_.PixelToStreetImage(inputImage, outputImage_lines);
    *outputImage_lanes = outputImage_lines->clone();
    visualizeDetections(detected_lines_v,
                        detected_lines_h,
                        vert_lanes,
                        horiz_lanes,
                        outputImage_lines,
                        outputImage_lanes);
  //  process_time =  std::chrono::high_resolution_clock::now()-start;
//    LOG_INFO(cString::Format("Lane Visualization took %f s", process_time.count()));
    }
    // transmit points detected by the line detection
    vector<tPoint> all_line_points;
    for (auto itlines = detected_lines_v.begin(); itlines != detected_lines_v.end(); ++itlines) {
        for (auto itpoints = (*itlines).begin(); itpoints != (*itlines).end(); ++itpoints) {
            all_line_points.push_back(*itpoints);
        }
    }

    /*
    for (auto itlines = detected_lines_h.begin(); itlines != detected_lines_h.end(); ++itlines) {
        for (auto itpoints = (*itlines).begin(); itpoints != (*itlines).end(); ++itpoints) {
            all_line_points.push_back(*itpoints);
        }
    }*/
    //transmitPoints(all_line_points);
    transmitLanes(vert_lanes, horiz_lanes);

}

void cAdaptiveLaneDetection::visualizeDetections(vector<vector<tPoint>> detected_lines_v,
                                                 vector<vector<tPoint>> detected_lines_h,
                                                 vector<Lane> vert_lanes,
                                                 vector<Lane> horiz_lanes,
                                                 Mat* outputImage_lines,
                                                 Mat* outputImage_lanes) {
    // draw a frame around the image so one can see its extends
    int w = outputImage_lines->size().width;
    int h = outputImage_lines->size().height;
    cv::rectangle(*outputImage_lines,  // image
                  Point2i(0, 0),  // pt. one
                  Point2i(w - 1, h - 1),  // pt. 2
                  Scalar(255, 255, 255),  // color (R, G, B)
                  4);  // thinkness of the line


    // // visualize the points that determine the birdseye transform
    // vector<Point2f> sourcePoints = this->transform_helper_.getSourcePoints();
    // vector<Point2f> destinationPoints = this->transform_helper_.getDestinationPoints();
    // for (int i = 0; i < 4; i++) {
    //     cv::circle(*outputImage_lanes,            // image to draw on
    //                destinationPoints[i],    // center
    //                10,                      // radius
    //                Scalar(0, 255, 0),       // color RGB
    //                2);                      // thickness
    // }

    // visualize the region of interest of the line point detection
    int roi_offset_pix_x = points_detector_.MeterToPixel(static_cast<float>(roi_offset_x));
    int roi_offset_pix_y = points_detector_.MeterToPixel(static_cast<float>(roi_offset_y));
    int roi_pix_width = points_detector_.MeterToPixel(static_cast<float>(roi_width));
    int roi_pix_height = points_detector_.MeterToPixel(static_cast<float>(roi_height));
    cv::rectangle(*outputImage_lines,
                Point2i(roi_offset_pix_x,
                      roi_offset_pix_y),
                Point2i(roi_offset_pix_x + roi_pix_width,
                      roi_offset_pix_y + roi_pix_height),
                Scalar(255, 255, 255),
                4);

    // visualize the lines
    visualizeLines(detected_lines_v,
                   outputImage_lines,
                   Scalar(255, 0, 0));  // vert lines red-yellow
    visualizeLines(detected_lines_h,
                   outputImage_lines,
                   Scalar(0, 0, 255));  // horiz lines blue-cyan

    // visualize the lanes
    std::vector<std::vector<tPoint> > sampled_lanes;
    sampleLanes(vert_lanes, horiz_lanes, &sampled_lanes);
    visualizeLines(sampled_lanes, outputImage_lanes, Scalar(255, 0, 0));

    // logging
    /*
    LOG_INFO("--- %i ----------", this->videoFrameCounter);
    LOG_INFO("%li vert lines",
        detected_lines_v.size());
    LOG_INFO("%li horiz lines",
        detected_lines_h.size());
    LOG_INFO("%i vert lanes", vert_lanes.size());
    LOG_INFO("%i horiz lanes", horiz_lanes.size());
    for (std::vector<Lane>::iterator laneit = vert_lanes.begin();
                                     laneit < vert_lanes.end();
                                     ++laneit) {
        Lane lane = *laneit;
        LOG_INFO("  vert lane");
        LOG_INFO("    alignment=%i", lane.alignment);
        LOG_INFO("    no_of_samples=%i", lane.no_of_samples);
        LOG_INFO("    dist=%f", lane.dist);
        LOG_INFO("    angle_origin_rad=%f", lane.angle_origin_rad);
        LOG_INFO("    lane_quality=%f", lane.lane_quality);
    }

    // log the sampled lane points
    // for (auto itlines = sampled_lanes.begin(); itlines != sampled_lanes.end(); ++itlines) {
    //     for (auto itpoints = (*itlines).begin(); itpoints != (*itlines).end(); ++itpoints) {
    //         tFloat32 x = itpoints->x;
    //         tFloat32 y = itpoints->y;
    //         Point2i streetImagePoint;
    //         transform_helper_.StreetToStreetImage(x, y, &streetImagePoint);
    //         LOG_INFO("Samled-Line-Point (%f, %f)", x, y);
    //         LOG_INFO("pix-Samped-Line-Point (%i, %i)", streetImagePoint.x, streetImagePoint.y);
    //     }
    // }
    */
}


void cAdaptiveLaneDetection::visualizeLines(vector<vector<tPoint>> lines,
                                            Mat* outputImage,
                                            Scalar reference_color) {
    int i = 0;
    for (auto itlines = lines.begin(); itlines != lines.end(); ++itlines) {
        Scalar mixed_color = reference_color + Scalar(0, (255 / lines.size()) * i, 0);
        if (mixed_color[1] > 255) {
            mixed_color[1] = 255;
        }
        for (auto itpoints = (*itlines).begin(); itpoints != (*itlines).end(); ++itpoints) {
            tFloat32 x = itpoints->x;
            tFloat32 y = itpoints->y;
            Point2i streetImagePoint;
            transform_helper_.StreetToStreetImage(x, y, &streetImagePoint);
            // LOG_INFO(cString::Format("Det.Point x=%f  y=%f", x, y));
            cv::rectangle(*outputImage,
                       streetImagePoint - Point2i(5, 5),
                       streetImagePoint + Point2i(5, 5),
                       mixed_color,
                       CV_FILLED);
        }
        i++;
    }
}


tResult cAdaptiveLaneDetection::transmitPoints(vector<tPoint> points) {
    return this->write_pin_pointlist.writeData(points);
}

tResult cAdaptiveLaneDetection::transmitLanes(const vector<Lane> &vert_lanes, const vector<Lane> &horiz_lanes) {
    vector<tLaneElement> laneElements = vector<tLaneElement>(vert_lanes.size()+horiz_lanes.size());
//    LOG_INFO(cString::Format("Transmitting %d lanes", laneElements.size()));

    if(vert_lanes.empty()){
        RETURN_NOERROR;
    }

    for (Lane lane : vert_lanes) {
        tLaneElement elem;
        elem.isVertLane = 1;
        elem.dist = lane.dist;
        elem.alignment = lane.alignment;
        elem.orientation = lane.orientation;
        elem.angleOriginRad = lane.angle_origin_rad;
        elem.startx = lane.start_pnt.x;
        elem.starty = lane.start_pnt.y;
        elem.endx = lane.end_pnt.x;
        elem.endy = lane.end_pnt.y;
        if (lane.coeff.size() > 0) {
            elem.coeff0 = lane.coeff[0];
            if (lane.coeff.size() > 1) {
                elem.coeff1 = lane.coeff[1];
                if (lane.coeff.size() > 2) {
                elem.coeff2 = lane.coeff[2];
                    if (lane.coeff.size() > 3 && lane.coeff[3]!=0) {
                        LOG_INFO(cString::Format("Warning: Lane Coefficient list larger (%d) than expected: 3", lane.coeff.size()));

                    }
                }
            }
        }
        laneElements.emplace_back(elem);
    }

    for (Lane lane : horiz_lanes) {
        continue;
        tLaneElement elem;
        elem.isVertLane = 0;
        elem.dist = lane.dist;
        elem.alignment = lane.alignment;
        elem.orientation = lane.orientation;
        elem.angleOriginRad = lane.angle_origin_rad;
        elem.startx = lane.start_pnt.x;
        elem.starty = lane.start_pnt.y;
        elem.endx = lane.end_pnt.x;
        elem.endy = lane.end_pnt.y;
        if (lane.coeff.size() > 0) {
            elem.coeff0 = lane.coeff[0];
            if (lane.coeff.size() > 1) {
                elem.coeff1 = lane.coeff[1];
                if (lane.coeff.size() > 2) {
                elem.coeff2 = lane.coeff[2];
                if (lane.coeff.size() > 3 && lane.coeff[3]!=0) {
                        LOG_INFO(cString::Format("Warning: Lane Coefficient list larger (%d) than expected: 3", lane.coeff.size()));
                    }
                }
            }
        }
        laneElements.emplace_back(elem);
    }

    write_pin_lanelist.writeData(laneElements);
    RETURN_NOERROR;
}


static void extractCurrentRoadLanes(vector<Lane> *laneData, Lane **left, Lane **right) {
    for (auto &data : *laneData) {
        Lane *lane = &data;
        if (lane->start_pnt.y >= 0.02 && lane->start_pnt.y < 0.20) {
            *left = lane;
            break;
        } else if (lane->start_pnt.y < -0.0 && lane->start_pnt.y > -0.60) {
                *right = lane;
        }
    }
}

bool getLaneCenterPoint(float x_dist, tPoint* center_point,
                                     Lane const* lane_left, Lane const * lane_right) {
     float LANEWIDTH_ = 0.47;
    float car_center_dist = 0.4 * LANEWIDTH_;
    float normal_angle;
    float lane_y_at_x;
    if ((lane_left == NULL) && (lane_right == NULL)){
         return false;
    }

    /***** single lane marking given *****/
    if ((lane_left == NULL) || (lane_right == NULL)) {
        Lane const* lane_tmp;
        if (lane_left != NULL){
            lane_tmp = lane_left;
        }
        else {
            lane_tmp = lane_right;
        }
        normal_angle = atan(lane_tmp->getDerivAtX(x_dist));
        if (lane_tmp == lane_left) {  //  marking left to car
            normal_angle += -0.5 * M_PI;
            car_center_dist = 0.7 * LANEWIDTH_;
        } else {  //  marking right to car
            normal_angle += 0.5 * M_PI;
            car_center_dist = 0.4 * LANEWIDTH_;
        }

        lane_y_at_x = lane_tmp->getValAtX(x_dist);
        float x = x_dist + car_center_dist * cos(normal_angle);
        float y = lane_y_at_x + car_center_dist * sin(normal_angle);

        //cout << "xy: " << x << ", " << y <<std::endl;
        center_point->x = x;
        center_point->y = y;
        return true;
    }

     /***** two lane markings given: ******/

    float abs_y_right = fabs(lane_right->start_pnt.y);
    float abs_y_left = fabs(lane_left->start_pnt.y);

    if (abs_y_right > 0.27 && abs_y_left > 0.27) {
        if (abs_y_right > abs_y_left) {
            return getLaneCenterPoint(x_dist, center_point, lane_left, NULL);
        } else {
             return getLaneCenterPoint(x_dist, center_point, NULL, lane_right);
        }
    }

    /*
    if ((lane_right->getStartPoint().GetX() + lane_left->getStartPoint().GetX()) > 1) {
         // Making the left lane null if the distance between left and right is more than 0.5 units
         cout<<"start points too far away, setting left to null "<< lane_right->getStartPoint().GetX() + lane_left->getStartPoint().GetX() <<std::endl;
         return getLaneCenterPoint(x_dist, nullptr, lane_right);
    }
    if ((lane_right->getStartPoint().GetX() + lane_left->getStartPoint().GetX()) < -1){
        cout<<"start points too far away, setting right to null "<< lane_right->getStartPoint().GetX() + lane_left->getStartPoint().getX() <<std::endl;
        return getLaneCenterPoint(x_dist, lane_left, nullptr);
    }*/

    float dist_from_target_lane = 0.5 * LANEWIDTH_;
    float normal_angle_l = atan(lane_left->getDerivAtX(x_dist)) - 0.5 * M_PI;

    float normal_angle_r = atan(lane_right->getDerivAtX(x_dist)) + 0.5 * M_PI;

    float delta_x_l =  dist_from_target_lane * cos(normal_angle_l);
    float delta_y_l =  dist_from_target_lane * sin(normal_angle_l);

    float delta_x_r =  dist_from_target_lane * cos(normal_angle_r);
    float delta_y_r =  dist_from_target_lane * sin(normal_angle_r);

    float x = (x_dist + delta_x_l+x_dist + delta_x_r)/2;
    float y = (lane_left->getValAtX(x_dist) + delta_y_l + lane_right->getValAtX(x_dist) + delta_y_r)/2;
    center_point->x = x;
    center_point->y = y;

    return true;
}


static bool getLaneCenterPoints(std::vector<tPoint> *center_point_result, Lane
    *lane_left, Lane *lane_right, uint sample_range = 10, float scale = 1, float
                                    lane_width = 0.45) {
        //center_point_result->reserve(center_point_result->size() + sample_range);

        float sampleFrom = 0;
        if (lane_left) {
            sampleFrom = lane_left->start_pnt.x;
        }
        if (lane_right) {
            sampleFrom = fmax(sampleFrom, lane_right->start_pnt.x);
        }

        float sampleTo = 0;
        if (lane_left) {
            sampleTo = lane_left->end_pnt.x;
        }
        if (lane_right) {
            sampleTo = fmax(sampleTo, lane_right->end_pnt.x);
        }

        sampleTo *= 10;
        sampleFrom *= 15;

        for (int ix = sampleFrom; ix < sampleTo; ++ix) {
            auto x_val = static_cast<float>(ix / 10.0);
            tPoint center;

            getLaneCenterPoint(x_val, &center, lane_left,
                               lane_right);
            if (center.x != 0 && center.y != 0) {

                center_point_result->emplace_back(center);
            }
        }
        return true;
    }
void visualizeCenterLane(const std::vector<Lane>& vert_lanes,
                                         const std::vector<Lane>& horiz_lanes,
                                         std::vector<std::vector<tPoint> >* sampled_lanes) {

    Lane *left = NULL;
    Lane *right = NULL;

    std::vector<Lane> v_lanes = vert_lanes;
    extractCurrentRoadLanes(&v_lanes, &left, &right);

    vector<tPoint> center_pts;
    getLaneCenterPoints(&center_pts, left, right);


    if (!center_pts.empty()) {
        sampled_lanes->push_back(center_pts);
    }
}

void cAdaptiveLaneDetection::sampleLanes(std::vector<Lane>& vert_lanes,
                                         const std::vector<Lane>& horiz_lanes,
                                         std::vector<std::vector<tPoint> >* sampled_lanes) {
    if (!sampled_lanes) {
        printf("LaneDetection::SampleLanes failed!\n");
        return;
    }
    const float line_width_half = 0.015;
    /// horizontal lines:
    for (std::vector<Lane>::const_iterator h_lane = horiz_lanes.begin();
            h_lane < horiz_lanes.end(); ++h_lane) {
        std::vector<tPoint> temp;
        tPoint p1;
        p1.x = h_lane->start_pnt.x + line_width_half;
        p1.y = h_lane->start_pnt.y;
        temp.push_back(p1);
        tPoint p2;
        p2.x = h_lane->end_pnt.x + line_width_half;
        p2.y = h_lane->end_pnt.y;
        temp.push_back(p2);
        tPoint p3;
        p3.x = h_lane->end_pnt.x - line_width_half;
        p3.y = h_lane->end_pnt.y;
        temp.push_back(p3);
        tPoint p4;
        p4.x = h_lane->start_pnt.x - line_width_half;
        p4.y = h_lane->start_pnt.y;
        temp.push_back(p4);
        sampled_lanes->push_back(temp);
    }

    // vertical lines:
    // create lane polygon
    for (std::vector<Lane>::const_iterator lane = vert_lanes.begin();
         lane != vert_lanes.end(); ++lane) {
        int x_near = static_cast<int>(lane->start_pnt.x * 10);
        int x_far = static_cast<int>(lane->end_pnt.x * 10);
        std::vector<tPoint> temp_left;
        std::vector<tPoint> temp_right;
        for (int ix = x_near; ix < x_far; ++ix) {
            float x = ix / 10.0;
            float y_at_ix = lane_finder_.getValAtX(x, lane->coeff);
            tPoint pp1;
            pp1.x = x;
            pp1.y = y_at_ix + line_width_half;
            temp_left.push_back(pp1);
            tPoint pp2;
            pp2.x = x;
            pp2.y = y_at_ix - line_width_half;
            temp_right.push_back(pp2);
        }
        // flip temp right by 180 (last to first) and add to temp left, then
        // push back temp
        std::reverse(temp_right.begin(), temp_right.end());
        temp_left.insert(temp_left.end(), temp_right.begin(), temp_right.end());
        if (temp_left.empty()) continue;
        sampled_lanes->push_back(temp_left);
    }


    visualizeCenterLane(vert_lanes, horiz_lanes, sampled_lanes);
}




void cAdaptiveLaneDetection::setupLinePointsDetection() {
    LOG_INFO(cString::Format("roi_offset_x=%f", static_cast<float>(roi_offset_x)));
    LOG_INFO(cString::Format("roi_offset_y=%f", static_cast<float>(roi_offset_y)));
    LOG_INFO(cString::Format("roi_width=%f", static_cast<float>(roi_width)));
    LOG_INFO(cString::Format("roi_height=%f", static_cast<float>(roi_height)));
    LOG_INFO(cString::Format("detection_distance=%f", static_cast<float>(detection_distance)));
    LOG_INFO(cString::Format("min_line_width=%f", static_cast<float>(min_line_width)));
    LOG_INFO(cString::Format("max_line_width=%f", static_cast<float>(max_line_width)));
    LOG_INFO(cString::Format("min_line_contrast=%i", static_cast<int>(min_line_contrast)));

    this->transform_helper_ =
        CameraTransformations(string(cString(birdseyeTransFile)),
                              string(cString(sensorName)));
    points_detector_.SetCameraTransformationsObj(transform_helper_);

    this->points_detector_.SetRoiOffsetX(
        points_detector_.MeterToPixel(static_cast<float>(roi_offset_x)));
    this->points_detector_.SetRoiOffsetY(
        points_detector_.MeterToPixel(static_cast<float>(roi_offset_y)));
    this->points_detector_.SetRoiWidth(
        points_detector_.MeterToPixel(static_cast<float>(roi_width)));
    this->points_detector_.SetRoiHeight(
        points_detector_.MeterToPixel(static_cast<float>(roi_height)));
    this->points_detector_.SetDetectionDistance(static_cast<float>(detection_distance));
    this->points_detector_.SetMinLineWidth(static_cast<float>(min_line_width));
    this->points_detector_.SetMaxLineWidth(static_cast<float>(max_line_width));
    this->points_detector_.SetMinLineContrast(static_cast<int>(min_line_contrast));
}
