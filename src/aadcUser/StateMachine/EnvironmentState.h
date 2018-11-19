//
// Created by aadc on 23.10.18.
//

#ifndef AADC_USER_ENVIRONMENTSTATE_H
#define AADC_USER_ENVIRONMENTSTATE_H

#include "../PinClasses/StopWatch.h"
#include "memory"
#include "Point.h"
#include "stdafx.h"
#include "aadc_custom_structs_fraisers.h"
#include "CarModel.h"
#include "ObstacleProcessor.h"



using namespace fraisers::models;

template<class T>
class TimedEvent {
    T elem;
    StopWatch timer;


private:


public:
    explicit TimedEvent(T data) : elem(data), timer(StopWatch()) {}

    float getElapsedMilliseconds() {
        return static_cast<float>(timer.measure());
    }

    const StopWatch &getTimer() const {
        return timer;
    }

    const T *const getElement() {
        return &elem;
    }

    void setElement(T elem) {
        this->elem = elem;
    }

    void refresh(T data) {
        setElement(data);
        timer.reset();
    }

};



class EnvironmentState : public ICarModelDependent {

    ObstacleProcessor obs_processor = ObstacleProcessor(false, 0, 5000, 0, 0);
    bool debug_output = false;
    static constexpr float p_threshold_car = 0.2;
    static constexpr float p_threshold_person = 0.2;
    static constexpr float p_threshold_us_detection = 0.8;

    static constexpr int crosswalk_keep_alive_seconds = 8;
    static constexpr int marker_keep_alive_seconds = 8;
    static constexpr int crosswalk_detection_distance = 80;  // cm 
    static constexpr int after_marker_timer = 2000; // ms
    static constexpr int after_crosswalk_distance = 170;  // cm


    bool emergency_brake_active = false;


    tDetectionInfo detection_info;
    vector<tLaserSegStruct> laser_seg_data;
    std::unique_ptr<TimedEvent<fraisers::models::Marker>> crosswalk_marker;
    std::unique_ptr<TimedEvent<fraisers::models::Marker>> junction_marker;

    int car_in_planned_path_hits = 0;
    int car_in_planned_path_misses = 0;

    float steering_curvature;


    bool isMarkerValid(const std::unique_ptr<TimedEvent<fraisers::models::Marker>> &marker, int
    age_seconds) const;

    void update_with_adaptive_region(std::vector<Obstacle> obstacles, Point steering_anchor);

    float getCurvatureSteering();

    void onCrossWalkSignDetected(Marker *marker);

    void onJunctionSignDetected(Marker *marker);

public:
    void updateSteeringValue(float curv_value);

    float getCarInPlannedPathProb() const;

    bool isJunctionMarkerValid(int age_seconds) const;

    bool getObstacles(int classId, vector<Obstacle>* obs_out) const;

    void debugLog();

    bool isCarAhead()const;

    bool isPersonDetected() const;

    bool isChildDetected() const;

    bool isCarLeft()const;

    bool isCarRight()const;

    float getProbCarAhead() const;
    float getProbCarLeft() const;
    float getProbCarRight() const;
    float getProbPerson() const;
    float getProbChild() const;
    float getProbObsLeft() const;
    float getProbObsRight() const;
    float getProbObsRear() const;



    bool isCarInPlannedPath() const;

    bool isUsObstacleLeft()const;

    bool isUsObstacleRight() const;

    bool isUsObstacleRear() const;

    const tDetectionInfo& getDetInfo() const;


    bool isCrossWalkDetected() const;

    void update(tDetectionInfo pinData,
                vector<tLaserSegStruct> *laserSegData,
                std::deque<Point> waypoints,
                Point steering_anchor);

    void setEmergencyBrakeState(bool isActive);



    bool isEmergencyBrakeActive() const;

    void onMarkerDetected(fraisers::models::Marker *marker);


    const Marker* getJunctionMarker() const;

    void setCarModel(std::shared_ptr<CarModel> &carModel) override;


    CarModel* getCarModel() const;

};


#endif //AADC_USER_ENVIRONMENTSTATE_H
