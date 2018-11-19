//
// Created by aadc on 01.09.18.
//
#include "ObstacleProcessor.h"


#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180


void partitionLsData(vector<cv::Point2f> &pts, vector<vector<cv::Point2f>> &partition_result) {


    if (pts.empty()) {
        return;
    }

    float th_distance = 10.0; //cm
    float th2 = th_distance;
    vector<int> labels;
    int n_labels = cv::partition(pts, labels, [th2](const cv::Point2f &lhs, const cv::Point2f
    &rhs) {
        //return ((lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y)) < th2;
        return (lhs.y - rhs.y) * (lhs.y - rhs.y) <= th2;
    });

    partition_result.reserve(n_labels);

    for (uint i = 0; i < partition_result.capacity(); i++) {
        partition_result.emplace_back(vector<cv::Point2f>());
    }

    for (uint i = 0; i < pts.size(); ++i) {
        vector<cv::Point2f> *label = &partition_result.at(labels[i]);
        label->emplace_back(pts[i]);
    }
}

void ObstacleProcessor::processLsData(const std::vector<tPolarCoordiante> lsData,
                                      std::vector<Obstacle> *out) {

    vector<cv::Point2f> partition_data;

    Point laser_scanner = car_model->getLaserScanner();
    for (const tPolarCoordiante &coord : lsData) {

        if (coord.f32Radius == 0 && !allow_zero_dist) {
            continue;
        }
        if (coord.f32Radius > ls_dist_min && coord.f32Radius <= ls_dist_max
            && (coord.f32Angle <= ls_angle_right || coord.f32Angle >= ls_angle_left)) {
            tPolarCoordiante pointCoord = coord;
            if (pointCoord.f32Angle >= 0 && pointCoord.f32Angle < 90) {
                pointCoord.f32Angle = 90 - pointCoord.f32Angle;
            } else {
                pointCoord.f32Angle = 450 - pointCoord.f32Angle;
            }


            float x = (pointCoord.f32Radius / 1000) *
                      cos(pointCoord.f32Angle * static_cast<float>(DEG2RAD));
            float y = (pointCoord.f32Radius / 1000) *
                      sin(pointCoord.f32Angle * static_cast<float>(DEG2RAD));

            Point lsDataPt = Point::Local(x, y, 0.0);

            lsDataPt.scaleBy(Coordinate::Type::GLOBAL);
            // add LS data offset (ls scanner is 20cm ahead of camera (center of position)
            laser_scanner.toGlobalNoCopy(lsDataPt, car_model->getHeading(), 0.0 * Coordinate::GLOBAL,
                    0);

            //partition_data.emplace_back(cv::Point2f(lsDataPt.getX(), lsDataPt.getY()));
            Obstacle obs = Obstacle(lsDataPt,
                                    Obstacle::Type::UNKNOWN, 1, 1, -1);
            out->emplace_back(obs);
        }
    }

    // LS point cloud clustering
    /*
     * cout << "Partition DATA:" << endl;
   for (auto &obs : partition_data) {
       cout << obs << endl;
   }
   cout << "END Partition DATA" << endl;


 vector<vector<cv::Point2f>> partition_result;
   partitionLsData(partition_data, partition_result);
   for (auto &result : partition_result) {
       cv::Scalar mean, stddev;
       cv::meanStdDev(result, mean, stddev);
       cv::Scalar var;
       cv::pow(stddev, 2, var);
       cout << "Partition result: " << partition_result.size() << endl;
       cout << "Mean: " << mean << endl;
       cout << "var: " << var << endl;
       Point local = Point::Local(static_cast<float>(mean[0]),
                                  static_cast<float>(mean[1]), Scalable::Unit::M);

       cv::Mat pt_mat(result.size(), 2, CV_32F, result.data());

       double min, max;
       cv::Point minLoc, maxLoc;
       cv::minMaxLoc(pt_mat.col(0), &min, &max, &minLoc, &maxLoc);
       float exp_x = fabs(max - min);

       cv::minMaxLoc(pt_mat.col(1), &min, &max, &minLoc, &maxLoc);
       float exp_y = fabs(max - min);

       Obstacle obs = Obstacle(local,
                               Obstacle::Type::Static, exp_x, exp_y);

       cout << "Computed obstacle: " << endl;
       cout << obs << endl;

       out->emplace_back(obs);
    }
*/

}


Obstacle ObstacleProcessor::processLaserSegStruct(tLaserSegStruct ls_struct) const {
    float heading = ls_struct.f32Angle;
    if (ls_struct.f32Angle >= 0 && ls_struct.f32Angle < 90) {
        ls_struct.f32Angle = 90 - ls_struct.f32Angle;
    } else {
        ls_struct.f32Angle = 450 - ls_struct.f32Angle;
    }

    float x = (ls_struct.f32Distance) *
              cos(ls_struct.f32Angle * static_cast<float>(DEG2RAD));
    float y = (ls_struct.f32Distance) *
              sin(ls_struct.f32Angle * static_cast<float>(DEG2RAD));

    Point lsDataPt = Point::Local(x, y, 0.0);

    // LOG_INFO("lspt: %f %f", lsDataPt.getX(), lsDataPt.getY());

    lsDataPt.scaleBy(Coordinate::Type::GLOBAL);
    // add LS data offset (ls scanner is 20cm ahead of camera (center of position)
    Point laser_scanner = car_model->getLaserScanner();
    laser_scanner.toGlobalNoCopy(lsDataPt, car_model->getHeading());

    return {lsDataPt,
            Obstacle::getTypeBySegmentationInfo(ls_struct.i16Class),
            static_cast<float>(ls_struct.i32Width),
            static_cast<float>(ls_struct.i32Height),
            heading};

}