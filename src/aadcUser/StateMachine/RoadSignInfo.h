#include <utility>

#include <utility>

//
// Created by aadc on 06.10.18.
//

#ifndef AADC_USER_ROADSIGNS_H
#define AADC_USER_ROADSIGNS_H

#include "../PinClasses/RoadSignsMapPin.h"
#include "CarModel.h"
using namespace fraisers::models;

class RoadSignInfo {
    static RoadSignInfo *instance;

    vector<ParkingSpace> parkingSpace;

    float marker_det_distance_max_threshhold;

    unordered_map<int, vector<RoadSign>> road_sign_map;

    bool enable_debug = false;

    RoadSignInfo() = default;

    explicit RoadSignInfo(vector<RoadSign> signs, vector<ParkingSpace> parkingSpace, float max_marker_det_distance_m) :
    parkingSpace(parkingSpace),
    marker_det_distance_max_threshhold(max_marker_det_distance_m*Coordinate::Type::GLOBAL) {
        for (auto &sign: signs){
            road_sign_map[sign.u16Id].emplace_back(sign);
        }
    }


public:

    static RoadSignInfo *getInstance() {
        if (instance == nullptr) {
            instance = new RoadSignInfo();
        }
        return instance;
    }

    static RoadSignInfo *init(vector<RoadSign> signs, vector<ParkingSpace> parkingSpace, float max_marker_det_distance_m) {
        instance = new RoadSignInfo(std::move(signs), std::move(parkingSpace), max_marker_det_distance_m);
        return instance;
    }


    const ParkingSpace* getParkingSpace(int id);

    const RoadSign* getSign(int id, CarModel* position);


};


#endif //AADC_USER_ROADSIGNS_H
