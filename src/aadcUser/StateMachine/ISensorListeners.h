//
// Created by aadc on 29.09.18.
//

#ifndef AADC_USER_ISENSORLISTENERS_H
#define AADC_USER_ISENSORLISTENERS_H

#include "Point.h"

enum DataSource {
    LS,
    US,
    PC
};
class ObstacleListener {
public:


    virtual void onObstaclesDetected(const std::vector<fraisers::models::Obstacle> *const
            obstacles,
            DataSource source)= 0;
};


class PerceptionListener {

    virtual void onPerceptionDataAvailable(const cv::Mat *const data) = 0;
};


#endif //AADC_USER_ISENSORLISTENERS_H
