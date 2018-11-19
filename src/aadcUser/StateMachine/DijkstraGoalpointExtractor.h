#pragma once

#include <vector>
#include "stdafx.h"
#include <opencv2/opencv.hpp>

void getGoalPoints(cv::Mat* image);

class DijkstraGoalpointExtractor {
 private:
    int segmentation_channel = 0;
    int cost_channel = 2;
    int goal_radius = 75;
    int start_position_x = 0;
    int start_position_y = 13;
    int start_rectangle_y = 5;
    unsigned int select_each_xth_waypoint = 10;
    unsigned int intersection_region = 10;
    float intersection_threshold = 0.75;
    unsigned int distance_to_intersection = 10;
    int point_shift_x = 0;

    cv::Scalar start_color = cv::Scalar(255, 255, 255);
    cv::Scalar goal_color = cv::Scalar(120, 0, 0);
    cv::Scalar foundgoal_color = cv::Scalar(255, 0, 0);
    cv::Scalar queue_color = cv::Scalar(255, 255, 0);
    cv::Scalar path_color = cv::Scalar(0, 255, 255);
    cv::Scalar path_waypoint_color = cv::Scalar(0, 255, 255);
    cv::Scalar intersection_color = cv::Scalar(0, 255, 0);

    cv::Mat goal_map;
    std::vector<int> goal_list_x;
    std::vector<int> goal_list_y;
    std::vector<int> start_list_x;
    std::vector<int> start_list_y;

 public:
    DijkstraGoalpointExtractor();
    DijkstraGoalpointExtractor(const int segmentation_channel,
                               const int cost_channel,
                               const int goal_radius,
                               const int start_position_y,
                               const unsigned int select_each_xth_waypoint,
                               const unsigned int intersection_region,
                               const float intersection_threshold,
                               const unsigned int distance_to_intersection,
                               const int point_shift_x);

    bool getGoalPointsToGoal(const cv::Mat& _cost_map,
                             const cv::Mat& segmentation_map,
                             vector<cv::Point>* waypoints,
                             bool visualize,
                             cv::Mat* debug_image,
                             int control_id,
                             cv::Point2f local_car_position,
                             cv::Point2f goal_position,
                             bool plan_to_goal,
                             bool disable_goal_radius);

    int getGoalRadius();
};

