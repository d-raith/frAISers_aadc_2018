//
// Created by aadc on 26.10.18.
//

#ifndef AADC_USER_LASERSEGBOOLHITCOUNT_H
#define AADC_USER_LASERSEGBOOLHITCOUNT_H

#include <aadc_custom_structs_fraisers.h>
#include "HitCount.h"

struct LaserSegInfoStruct {
    bool person_detected = false;
    bool car_left_detected = false;
    bool car_right_detected = false;
    bool car_front_detected = false;
};


class LaserSegBoolHitCount {

public:
    HitCount car_left;
    HitCount car_center;
    HitCount car_right;
    HitCount person_detected;
    HitCount child_detected;


    explicit LaserSegBoolHitCount(int prob_reset_threshold):
                                                             car_left(HitCount(prob_reset_threshold)),
                                                             car_center(HitCount(prob_reset_threshold)),
                                                             car_right(HitCount(prob_reset_threshold)),
                                                             person_detected(HitCount
                                                             (prob_reset_threshold)),
                                                             child_detected(HitCount
                                                             (prob_reset_threshold)) {

    }


    void applyTo(tDetectionInfo* obs_data);

};


#endif //AADC_USER_LASERSEGBOOLHITCOUNT_H
