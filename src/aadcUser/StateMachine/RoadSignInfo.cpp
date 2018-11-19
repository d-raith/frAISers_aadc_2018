//
// Created by aadc on 06.10.18.
//
#include "RoadSignInfo.h"

RoadSignInfo *RoadSignInfo::instance = nullptr;


const RoadSign *RoadSignInfo::getSign(int id, CarModel *position) {
    if (enable_debug) {
        LOG_INFO("RoadSign query: %d", id);
    }


    auto sign_list_iter = road_sign_map.find(id);

    if (sign_list_iter == road_sign_map.end()) {
        // handle init value of marker (-1)
        return nullptr;
    }

    const vector<RoadSign> &signs_list = sign_list_iter->second;

    double dist_closest = -1;
    const RoadSign *closest = nullptr;
    float orientation = position->getHeadingDegreesClamped();

    if (enable_debug) {
        LOG_INFO("orientation: %f", orientation);
    }
    for (auto &sign : signs_list) {
        if (sign.f32Direction != orientation) {
            if (enable_debug) {
                LOG_INFO("orientation mismatch: sign: id: %d, %f, orientation: %f", sign.u16Id,
                         sign.f32Direction,
                         orientation);
            }
            continue;
        }
        Point sign_loc = Point::Global(sign.f32X, sign.f32Y);
        sign_loc.scaleBy(Coordinate::Type::GLOBAL);
        if (position->toLocal(sign_loc).getY() <= 0) {
            if (enable_debug) {
                LOG_INFO("Marker candidate at %f %f is behind car, ignoring", sign_loc.getX(),
                         sign_loc.getY());
            }
            continue;
        }

        double dist = position->getRearAxis().distanceTo(sign_loc);
        if (dist > marker_det_distance_max_threshhold) {
            if (enable_debug) {
                LOG_INFO(
                        "Marker candidate at %f %f too far away dist: %f, th: %f (threshhold limit exceeded)",
                        sign_loc.getX(), sign_loc.getY(), dist, marker_det_distance_max_threshhold);
            }
            continue;
        }

        if (dist_closest == -1 || dist_closest > dist) {
            dist_closest = dist;
            closest = &sign;
            if (enable_debug) {
                LOG_INFO("Marker candidate at %f %f found as potential result", sign_loc.getX(),
                         sign_loc.getY());
            }
        }
    }
    if (enable_debug) {
        if (closest) {
            LOG_INFO("RoadSignDetResult: id: %d, x: %f, y: %f", closest->u16Id, closest->f32X,
                     closest->f32Y);
        } else {
            LOG_INFO("No plausible road sign found");
        }
    }
    return closest;
}

const ParkingSpace *RoadSignInfo::getParkingSpace(int id) {

    for (auto &pspace : parkingSpace) {
        if (pspace.u16Id == id) {
            return &pspace;
        }
    }
    return nullptr;
}



