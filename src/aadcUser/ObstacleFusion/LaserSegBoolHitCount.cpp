//
// Created by aadc on 26.10.18.
//

#include "LaserSegBoolHitCount.h"

void LaserSegBoolHitCount::applyTo(tDetectionInfo *obs_data) {
    obs_data->f32pCarCenter = car_center.getProbability();
    obs_data->f32pCarLeft = car_left.getProbability();
    obs_data->f32pCarRight = car_right.getProbability();
    obs_data->f32pPerson = person_detected.getProbability();
    obs_data->f32pChild = child_detected.getProbability();
}
