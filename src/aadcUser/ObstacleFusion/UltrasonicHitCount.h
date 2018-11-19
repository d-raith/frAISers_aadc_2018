//
// Created by aadc on 26.10.18.
//

#ifndef AADC_USER_ULTRASONICHITCOUNT_H
#define AADC_USER_ULTRASONICHITCOUNT_H

#include <aadc_custom_structs_fraisers.h>
#include "HitCount.h"
#include "aadc_structs.h"

class UltrasonicHitCount {
    float hit_threshold;

    HitCount left;
    HitCount right;
    HitCount rear_left;
    HitCount rear_right;
    HitCount rear_center;



public:


    explicit UltrasonicHitCount(float us_hit_threshold, int prob_reset_threshold): hit_threshold
    (us_hit_threshold*100),
        left(HitCount(prob_reset_threshold)),
        right(HitCount(prob_reset_threshold)),
        rear_left(HitCount(prob_reset_threshold)),
        rear_right(HitCount(prob_reset_threshold)),
        rear_center(HitCount(prob_reset_threshold)) {

    }
    bool updateFromStruct(const tUltrasonicStruct& data);

    void applyTo(tDetectionInfo* obs_data);
};


#endif //AADC_USER_ULTRASONICHITCOUNT_H
