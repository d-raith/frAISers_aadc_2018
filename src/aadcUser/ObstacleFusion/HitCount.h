//
// Created by aadc on 26.10.18.
//

#ifndef AADC_USER_HITCOUNT_H
#define AADC_USER_HITCOUNT_H


class HitCount {
    // avoid 0 division
    float miss = 1;
    float hit = 0;

    int probability_reset_threshold;

public:

    HitCount() = delete;
    explicit HitCount(int p_reset_threshold) : probability_reset_threshold(p_reset_threshold) {}

    void update(bool isHit);

    float getProbability() const;
};

#endif //AADC_USER_HITCOUNT_H
