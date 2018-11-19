//
// Created by aadc on 29.09.18.
//

#ifndef AADC_USER_ICTRLOVERRIDE_H
#define AADC_USER_ICTRLOVERRIDE_H




class SpeedCtrlOverride {
public:
    virtual bool onOverrideSpeed(float *const speed) = 0;
};


class MaxSpeedOverride: public SpeedCtrlOverride {
public:
    float max_speed;

    MaxSpeedOverride(float max_abs_speed):max_speed(max_abs_speed){

    }

    bool onOverrideSpeed(float *const speed) override {
        if(fabs(*speed) > max_speed){
            (*speed) = Speed::STOP;
            cout << "Invalid speed signal provided, safety stop: " << speed << std::endl;
            return true;
        }
        return false;
    }
};




#endif //AADC_USER_ICTRLOVERRIDE_H
