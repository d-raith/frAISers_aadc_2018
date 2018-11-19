//
// Created by aadc on 06.10.18.
//

#ifndef AADC_USER_PUREPURSUIT_H
#define AADC_USER_PUREPURSUIT_H

#include "CarModel.h"





class PurePursuit:public ICarModelDependent {



    Point convertToLocal(const Point &anchor, Point target);

public:

    PurePursuit() = default;

    float getCurvature(const Point &anchor, Point wp, bool is_local=false);

};


#endif //AADC_USER_PUREPURSUIT_H
