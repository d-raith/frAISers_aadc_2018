//
// Created by aadc on 03.09.18.
//

#ifndef AADC_USER_LOCALMAP_H
#define AADC_USER_LOCALMAP_H

#include <vector>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <IGlobalMap.h>
#include "Point.h"
#include "Lane.h"
#include "GridMap.h"
#include "PrintUtils.h"
#include "CostMap.h"
#include "MatUtils.h"
#include "../PinClasses/StopWatch.h"
#include "CarModel.h"
#include "ISensorListeners.h"

class LocalMap : public CostMap, public ICarModelDependent, public ObstacleListener, public
        PerceptionListener {
private:
    float local_x_delta = 0;
    float local_y_delta = 0;
    float rotation_delta = 0;
    bool perception_mode_single_lane = true;
    StopWatch timing;

    float pixel_per_meter = 100;
    float pixel_per_cm = round(pixel_per_meter / 100);


    cv::Mat getGlobalMapArea(IGlobalMap &globalMap);

    void updateAndDiscretizeMovement(DeltaPosition delta, int* shift_x, int* shift_y, float* rotation);


public:
    Point recommended_goal = Point::Global(0, 0);

    int window_width = 0, window_height = 0;
    Point localCarPosition;


    // ch_road, ch_center_lane, ch_obstacle, unknown
    static constexpr int COST_UNKNOWN = 20;
    const cv::Mat cost_map = (cv::Mat_<float>(1, 3) << 8, 1, 30);
    cv::Mat distance_penalty_mask;

    bool map_initialized = false;
    cv::Mat channel_road_lanes;
    cv::Mat channel_perception;
    cv::Mat channel_obstacles;
    std::vector<Point> plainLS_obstacle_list;
    cv::Mat channel_segmentation;  // segmentation channel in birdseye. contains ids.

    cv::Mat planner_image;

    bool isInitialized();

    void init();





    void onUpdateGlobalPosition(IGlobalMap &globalMap);

    void onObstaclesDetected(const std::vector<Obstacle> *const obstacles, DataSource source) override;

    void onLaserScannerUpdate(const std::vector<tPolarCoordiante>& ls_polar,
                              const std::vector<Point>& ls_global_cartesian);

    void onPerceptionDataAvailable(const cv::Mat *const data) override;


    Point getLocalPoint(Point global);

    //TODO: Testing - remove
    void updatePnt(Point *pt);



    cv::Mat toRgbImgMatrix();

    cv::Mat toSoftMaxedCellProbImgMatrix(float proximity_radius);


    void addElements(std::vector<Point> &elements);


    void applyCostToSoftmax(cv::Mat &softmaxProb);

    float costAt(int x, int y);

    void updateHitsMisses(cv::Mat &channel, const std::vector<cv::Point2f> &hits, float hitVal);

    void updateHitsMisses(cv::Mat &channel, const std::vector<Obstacle> *const hits);

    cv::Vec3f costVectorAt(int x, int y);

    cv::Vec3f computeSoftmaxAt(int x, int y);

    void addLanes(std::vector<Lane> *lanes);

    void addLaneCenterPoints(std::vector<cv::Point2f> *center_points, float hitVal = 1);


    Point convertToGlobalFrame(const Point &localMapWaypoint);

    Point convertToLocalFrame(const Point &globalWaypoint);


    void convertToLocalFrame(cv::Point2f *globalWaypoint);

    cv::Point2i convertFromGlobalToLocalMapFrame(Point point);

    std::vector<Point> get_plainLS_obstacle_list();

    void printCostMap(const cv::Mat &channel);

    void updateMapChannel(IGlobalMap &globalMap);

    float getCost(float &heading, int &action_idx, int &cell_x, int &cell_y, bool
    useAdaptiveHeadingCost, bool isObstacleAvoidance);

    cv::Mat getLocalGoalMask(int l_x, int l_y);

    cv::Mat computeCenterLaneProbabilities();

    cv::Mat computeCostMap();




    LocalMap(float window_width_m, float window_height_m,
             Coordinate::Type resolution =
             Coordinate::Type::GLOBAL)
            :  window_width(static_cast<int>(window_width_m * resolution)),
               window_height(static_cast<int>(window_height_m * resolution)),
               localCarPosition(Point::Global(
                       static_cast<int>(ceil(window_width / 2)),
                       static_cast<int>(ceil(window_height / 6)),
                       0)) {
    }

    ~LocalMap()= default;
};

#endif //AADC_USER_LOCALMAP_H



