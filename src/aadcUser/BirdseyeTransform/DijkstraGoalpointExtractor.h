#pragma once

#include <vector>
#include "stdafx.h"
#include "../PinClasses/customtypes.h"

void getGoalPoints(cv::Mat* image);

class DijkstraGoalpointExtractor {
 private:
    int segmentation_channel = 0;
    int cost_channel = 2;
    int goal_radius = 75;
    unsigned int start_position_y = 13;
    int start_rectangle_y = 5;
    unsigned int select_each_xth_waypoint = 10;
    unsigned int intersection_region = 10;
    float intersection_threshold = 0.75;
    unsigned int distance_to_intersection = 10;
    int point_shift_x = 0;

    Scalar start_color = Scalar(255, 255, 255);
    Scalar goal_color = Scalar(120, 0, 0);
    Scalar foundgoal_color = Scalar(255, 0, 0);
    Scalar queue_color = Scalar(255, 255, 0);
    Scalar path_color = Scalar(0, 255, 255);
    Scalar path_waypoint_color = Scalar(0, 255, 255);
    Scalar intersection_color = Scalar(0, 255, 0);

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
                               const unsigned int start_position_y,
                               const unsigned int select_each_xth_waypoint,
                               const unsigned int intersection_region,
                               const float intersection_threshold,
                               const unsigned int distance_to_intersection,
                               const int point_shift_x);

    bool getGoalPoints(cv::Mat* image, vector<cv::Point>* waypoints, bool visualize, int control_id);
};

