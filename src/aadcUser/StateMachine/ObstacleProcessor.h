//
// Created by aadc on 01.09.18.
//
#pragma once
#ifndef AADC_USER_OBSTACLEPROCESSOR_H
#define AADC_USER_OBSTACLEPROCESSOR_H

#endif //AADC_USER_OBSTACLEPROCESSOR_H

#include "vector"
#include "Point.h"
#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include "CarModel.h"
#include "aadc_custom_structs_fraisers.h"

using namespace fraisers::models;

class ObstacleProcessor: public ICarModelDependent {
    ObstacleProcessor() = default;

    bool allow_zero_dist;
    int ls_dist_min, ls_dist_max;
    float ls_angle_left, ls_angle_right;


public:

    ObstacleProcessor(bool allow_zero_dist, int ls_min_dist, int ls_max_dist, float ls_angle_left,
                      float ls_angle_right)
            : allow_zero_dist(allow_zero_dist),
              ls_dist_min(ls_min_dist),
              ls_dist_max(ls_max_dist),
              ls_angle_left(ls_angle_left),
              ls_angle_right(ls_angle_right) {
    }


    void processLsData(const std::vector<tPolarCoordiante> lsData, std::vector<Obstacle> *out);


    Obstacle processLaserSegStruct(tLaserSegStruct ls_struct) const;
};
