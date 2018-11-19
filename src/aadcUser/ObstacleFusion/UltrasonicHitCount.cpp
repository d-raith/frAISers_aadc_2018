//
// Created by aadc on 26.10.18.
//

#include "UltrasonicHitCount.h"
#include "iostream"
bool UltrasonicHitCount::updateFromStruct(const tUltrasonicStruct &data) {
    left.update(
        data.tSideLeft.f32Value >= 0
    && data.tSideLeft.f32Value <= hit_threshold);

    right.update(
        data.tSideRight.f32Value >= 0
        && data.tSideRight.f32Value <= hit_threshold);

    rear_left.update(
        data.tRearLeft.f32Value >= 0
        && data.tRearLeft.f32Value <= hit_threshold);
    
    rear_center.update(
        data.tRearCenter.f32Value >= 0
        && data.tRearCenter.f32Value <= hit_threshold);
    
    rear_right.update(
        data.tRearRight.f32Value >= 0
        && data.tRearRight.f32Value <= hit_threshold);

        //    std::cout << "left: " << static_cast<float>(data.tSideLeft.f32Value) << " isHit: " <<(data.tSideLeft.f32Value <= hit_threshold) << std::endl;
    return true;
}

void UltrasonicHitCount::applyTo(tDetectionInfo *obs_data) {
    obs_data->f32pObsRear = (rear_center.getProbability() + rear_left.getProbability() + rear_right
            .getProbability()) / 3;
    obs_data->f32pObsRight = right.getProbability();
    obs_data->f32pObsLeft = left.getProbability();
}
