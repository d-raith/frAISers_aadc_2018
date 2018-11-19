/*
//
// Created by aadc on 02.08.18.
//

#include <cmath>
#include <iostream>
#include <algorithm>
#include <cstring>
#include "GridMap.h"
#include "PrintUtils.h"
#include "opencv2/imgproc.hpp"


//TODO: Obstacle cache
//TODO: GridElement reference obstacle/type
//TODO: Heading class?

GridMap::GridMap(uint rows, uint cols) {
    numGridRows = rows;
    numGridCols = cols;

    grid = InitGlobalMap();
}


cv::Mat GridMap::InitGlobalMap() {
    grid = cv::Mat(numGridRows, numGridCols, CV_32FC1, cvScalar(0));
    InitRoadGrid(&grid);

    return grid;
};

cv::Mat
GridMap::getDiscretizedMapArea(int window_width, int window_height, CarPosition *carPosition) {
    return getDiscretizedMapArea(window_width, window_height, carPosition->getX(),
                                 carPosition->getY(), carPosition->getHeadingDegrees());
}

cv::Mat GridMap::getDiscretizedMapArea(int window_width, int window_height, double car_x, double car_y,
        double car_angle){
    cv::Mat map_roi;
    cv::Mat map_window;
    car_angle = fmod(car_angle, 360.0);
    double window_center_x = car_y + sin(car_angle * (M_PI / 180)) * (window_width / 2.0);
    double window_center_y = car_x + cos(car_angle * (M_PI / 180)) * (window_height / 2.0);

    //double window_center_x = car_y;
    //double window_center_y = car_x;
    std::cout << "\t\tWindow center x|y: " << window_center_x << "|" << window_center_y \
        << std::endl << std::flush;
    // convert angle to opencv frame of reference
    //car_angle += 90;
    cv::RotatedRect window = cv::RotatedRect(
            cv::Point2f(window_center_x, window_center_y), cv::Size2f(
                    window_width-1, window_height-1), car_angle);

    cv::Rect roi = cv::Rect(window_center_x-window_width/2, window_center_y-window_height/2,
                            window_width, window_height);
    */
/*cv::Size window_size = window.size;
    if (window.angle < -45.) {
        car_angle += 90.0;
        cv::swap(window_size.width, window_size.height);
    }*//*

    //cv::Mat rot_mat = cv::getRotationMatrix2D(window.center, car_angle, 1);
    //cv::Mat map_bb = grid(roi);
    cv::Mat init = cv::Mat(window_height, window_width, CV_32FC1);
    while (roi.x<0){
        roi.x++;
    }
    while (roi.y<0){
        roi.y++;
    }
    grid(roi).copyTo(init);
    return init;



    //cv::warpAffine(grid, map_roi, rot_mat, grid.size(), cv::WARP_FILL_OUTLIERS);
    //cv::getRectSubPix(map_bb, window_size, window.center, map_window);
    //cv::getRectSubPix(map_roi, window_size, window.center, map_window);
    */
/*cv::Rect window_roi = map_window;


    bool valid = (0 <= window_roi.x && 0 <= window_roi.width && window_roi.x + window_roi.width <=
    grid
    .cols && 0 <= window_roi.y && 0 <= window_roi.height && window_roi.y + window_roi.height <=
    grid.rows);*//*


    return map_window;
}


void
addRoad(vector<vector<uint>> *grid_ptr, uint startx, uint starty, uint width, uint length, uint stop_idx, char orientation) {
    uint line_solid = 1;
    uint line_dashed = 2;

    uint roadVal = 3;
    uint roadOppositeVal = 4;

    vector<uint> road = vector<uint>(width, roadVal);
    road[0] = line_solid;
    road[width - 1] = line_solid;
    road[width / 2] = line_dashed;
    for (uint idx = width / 2 + 1; idx < width - 1; idx++) {
        road[idx] = roadOppositeVal;
    }


    if (orientation == 'h') {
        reverse(road.begin(), road.end());
        for (uint y = 0; y < length; y++) {
            for (uint x = 0; x < min((uint)road.size(), stop_idx); x++) {
                (*grid_ptr)[startx + x][starty + y] = road[x];
            }
        }


    } else {
        for (uint x = 0; x < length; x++) {
            for (uint y = 0; y < min((uint) road.size(), stop_idx); y++) {
                (*grid_ptr)[startx + x][starty + y] = road[y];
            }
        }
    }


}



class Road {
private:
    uint line_solid = 1;
    uint line_dashed = 1;

    uint roadVal = 3;
    uint roadOppositeVal = 4;
    vector<uint> road;

    char orientation = 'v';
public:
    Road(uint width, char orientation) {
        this->orientation = orientation;
        road = vector<uint>(width, 0);
        road[0] = line_solid;
        road[width - 1] = line_solid;
        road[width / 2] = line_dashed;
        for (uint idx = width / 2 + 1; idx < width - 1; idx++) {
            //road[idx] = roadOppositeVal;
        }
    }

    vector<uint> addTo(cv::Mat *grid, vector<uint> connectionPoint, uint length) {
        return addTo(grid, connectionPoint[0], connectionPoint[1], length);
    }

    vector<uint> addTo(cv::Mat *grid, uint startx, uint starty, uint length) {
        if (orientation == 'h') {
            //reverse(road.begin(), road.end());
            for (uint y = length; y > 0; --y) {
                for (uint x = 0; x < road.size(); ++x) {
                    grid->at<float>(startx + x, starty + y) = road[road.size() - 1 - x];
                }
            }
            return {startx, starty + length};
        } else {
            for (uint x = 0; x < length; ++x) {
                for (uint y = 0; y < road.size(); y++) {
                    grid->at<float>(startx + x, starty + y) = road[y];
                }
            }
            return {startx + length, starty};
        }
    }

    Road *reverseRoad() {
        reverse(road.begin(), road.end());
        return this;
    }


};


class Curve {
private:
   */
/* vector<vector<uint>> curve_grid = {{1, 3, 3, 3, 2, 4, 4, 4, 1},
                                       {0, 1, 3, 3, 3, 2, 4, 4, 4},
                                       {0, 0, 1, 3, 3, 3, 2, 4, 4},
                                       {0, 0, 0, 1, 3, 3, 3, 2, 4},
                                       {0, 0, 0, 0, 1, 3, 3, 3, 2},
                                       {0, 0, 0, 0, 0, 1, 3, 3, 3},
                                       {0, 0, 0, 0, 0, 0, 1, 3, 3},
                                       {0, 0, 0, 0, 0, 0, 0, 1, 3},
                                       {0, 0, 0, 0, 0, 0, 0, 0, 1}};*//*


    vector<vector<uint>> curve_grid = {{1, 0, 0, 0, 1, 0, 0, 0, 1},
                                       {0, 1, 0, 0, 0, 1, 0, 0, 0},
                                       {0, 0, 1, 0, 0, 0, 1, 0, 0},
                                       {0, 0, 0, 1, 0, 0, 0, 1, 0},
                                       {0, 0, 0, 0, 1, 0, 0, 0, 1},
                                       {0, 0, 0, 0, 0, 1, 0, 0, 0},
                                       {0, 0, 0, 0, 0, 0, 1, 0, 0},
                                       {0, 0, 0, 0, 0, 0, 0, 1, 0},
                                       {0, 0, 0, 0, 0, 0, 0, 0, 1}};

    vector<vector<uint>> transpose(vector<vector<uint>> &src) {
        vector<vector<uint>> dest = vector<vector<uint>>(src.size(), vector<uint>(src[0].size()));


        for (uint i = 0; i < src.size(); ++i) {
            for (uint j = 0; j < src[i].size(); ++j) {
                dest[j][i] = src[i][j];
            }
        }
        return dest;
    }


public:
    vector<uint> addTo(cv::Mat *grid, const vector<uint> &connectionPoint) {
        return addTo(grid, connectionPoint[0], connectionPoint[1]);
    };


    vector<uint> addTo(cv::Mat *grid, uint startx, uint starty) {

        for (uint x = 0; x < curve_grid.size(); ++x) {
            for (uint y = 0; y < curve_grid[x].size(); ++y) {
                grid->at<float>(startx + x, starty + y) = curve_grid[x][y];
            }
        }

        return {startx, starty + (uint) curve_grid[0].size() - 1};
    }


    vector<vector<uint>> &get() {
        return curve_grid;
    }

    Curve *transposed() {
        curve_grid = transpose(curve_grid);
        return this;
    }

    Curve *reversed() {
        reverse(curve_grid.begin(), curve_grid.end());
        return this;
    }

    Curve *rotateAntiClock() {
        rotateMatrix(curve_grid);
        return this;
    }

    vector<vector<uint>> transposed_reversed() {

        vector<vector<uint>> _grid = transpose(curve_grid);
        reverse(_grid.begin(), _grid.end());
        return _grid;
    }

    void rotateMatrix(vector<vector<uint>> &mat) {
        uint N = (uint) mat[0].size();
        // Consider all squares one by one
        for (uint x = 0; x < N / 2; x++) {
            // Consider elements in group of 4 in
            // current square
            for (uint y = x; y < N - x - 1; y++) {
                // store current cell in temp variable
                uint temp = mat[x][y];

                // move values from right to top
                mat[x][y] = mat[y][N - 1 - x];

                // move values from bottom to right
                mat[y][N - 1 - x] = mat[N - 1 - x][N - 1 - y];

                // move values from left to bottom
                mat[N - 1 - x][N - 1 - y] = mat[N - 1 - y][x];

                // assign temp to left
                mat[N - 1 - y][x] = temp;
            }
        }
    }

};


void GridMap::InitRoadGrid(cv::Mat *grid) {
    uint road_width = 9;
    Road road = Road(road_width, 'v');
    Road hroad = Road(road_width, 'h');
    Curve curve = Curve();
    vector<uint> cp = road.addTo(grid, 8, 0, 32);
    cp = curve.addTo(grid, cp);
    cp = hroad.addTo(grid, cp, 32);
    cp = curve.rotateAntiClock()->addTo(grid, cp);

    cp = road.reverseRoad()->addTo(grid, 9, cp[1] - road_width + 1, 32);
    //cp = curve.rotateAntiClock()->addTo(grid, 0, 41);
    //cp = hroad.reverseRoad()->addTo((*grid), 0,9, 31);
    //cp = curve.rotateAntiClock()->addTo((*grid), 0,0);
}

*/
