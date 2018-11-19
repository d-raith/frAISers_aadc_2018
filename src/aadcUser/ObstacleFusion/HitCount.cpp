//
// Created by aadc on 26.10.18.
//

#include "HitCount.h"


void HitCount::update(bool isHit) {

    if (isHit) {
        hit++;
    } else {
        miss++;
    }

    // reduce counts to avoid slow response time due to high miss counts
    if (miss+hit > probability_reset_threshold) {
        hit = 0;
        miss = 1;
    }
}


float HitCount::getProbability() const {
    return 2*hit / (2*hit + miss);
}