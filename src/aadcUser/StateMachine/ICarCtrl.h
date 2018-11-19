//
// Created by aadc on 27.07.18.
//

#pragma once

#include "Point.h"
#include "../PinClasses/StopWatch.h"
#ifndef STATEMACHINE_ICARCTRL_H
#define STATEMACHINE_ICARCTRL_H


using namespace fraisers::models;


class Speed {
public:
    static constexpr float REV_SAFE=-0.3f;
    static constexpr float REV_SLOW=-0.4f;
    //static constexpr float BRAKE=-0.4f;
    static constexpr float STOP=0.0f;
    static constexpr float SLOW=0.9f;
    //static constexpr float FAST=1.2f;
};

class IVelocityCtrl {

public:
    virtual bool setVehicleSpeed(const float speed) {
        return false;
    }
};


class ISteerCtrl {
public:

    virtual bool setCurvature(float curv_value) {
        return false;
    }
};

class ILightCtrl {
public:
    virtual bool setIndicatorRightEnabled(bool enable) = 0;

    virtual bool setIndicatorLeftEnabled(bool enable) = 0;

    virtual bool setHazardLightsEnabled(bool enable) = 0;

    virtual bool setBrakeLightsEnabled(bool enable) = 0;

    virtual bool setHeadLightsEnabled(bool enable) = 0;

    virtual bool setReverseLightsEnabled(bool enable) = 0;
};

class IEmergencyBrakeCtrl {
    public:
    virtual void setEmergencyBrakeEnabled(bool enabled) = 0;

    virtual bool isEnabled() = 0;

    virtual StopWatch* getLastTriggerWatch() = 0;
};
#endif //STATEMACHINE_ICARCTRL_H
