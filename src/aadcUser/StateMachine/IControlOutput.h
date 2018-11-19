//
// Created by aadc on 29.07.18.
//
#pragma once

#include "Point.h"
#include "opencv2/core/mat.hpp"
#include "BaseTask.h"

#ifndef STATEMACHINE_IFILTEROUTPUT_H
#define STATEMACHINE_IFILTEROUTPUT_H


using namespace fraisers::models;

class IControlOutput {
public:
    virtual bool transmitCurvature(float curvature) { return false; }

    virtual bool transmitSpeedSignal(float speed) { return false; }

    virtual bool isLocalMapBufferEmpty() { return false; }

    virtual bool transmitLocalMap(cv::Mat *localMapRgb) { return false; };

    virtual bool isPlannerBufferEmpty() { return false; }

    virtual bool transmitPlanner(cv::Mat *plannerRgb) { return false; };

    virtual bool taskFinished(BaseTask *task) { return false; }

    virtual bool taskFailed(BaseTask *task) { return false; }
    virtual bool taskStarted(BaseTask *task) { return false; }


    virtual bool requestOvertake(BaseTask *task) {return false;};

    virtual int getSectorFromTaskId(int task_id) {return -1;};
};
#endif //STATEMACHINE_IFILTEROUTPUT_H