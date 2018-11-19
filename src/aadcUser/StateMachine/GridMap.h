/*
//
// Created by aadc on 02.08.18.
//

#pragma once
#ifndef PLANNER_GRIDMAP_H
#define PLANNER_GRIDMAP_H

#endif //PLANNER_GRIDMAP_H


#include "IGlobalMap.h"
#include "Point.h"
#include "vector"

using namespace std;
using namespace fraisers::models;


class GridMap : public IGlobalMap {

private:
    int numGridRows;
    int numGridCols;


    cv::Mat grid;

    void InitRoadGrid(cv::Mat *grid);

    cv::Mat InitGlobalMap();



    vector<vector<uint>> GetRadiusVector(uint x, uint y, uint radius, float heading);

public:

    GridMap(uint rows, uint cols);

    cv::Mat getDiscretizedMapArea(int window_width, int window_height, double car_x, double car_y,
                                  double car_angle) override;

    cv::Mat getDiscretizedMapArea(int window_width, int window_height, CarPosition *carPosition);
};
*/
