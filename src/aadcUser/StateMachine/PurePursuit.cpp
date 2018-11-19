//
// Created by aadc on 06.10.18.
//

#include "PurePursuit.h"

float PurePursuit::getCurvature(const Point &anchor, Point wp, bool is_local) {

    if (!is_local) {
        wp = convertToLocal(anchor, wp);
    }
    wp.scaleBy(1.0/Coordinate::GLOBAL);

    //LOG_INFO("Local wp: %f %f", wp.getX(), wp.getY());

    float delta_lookahead_x_ = wp.getX();
    float delta_lookahead_y_ = wp.getY();
    if (wp.getY() < 0) {
        wp.setY(-wp.getY());
    }

    //curvature = -1 * static_cast<float>((delta_lookahead_x_) / (0.5 * (pow(delta_lookahead_x_, 2.0) +pow(delta_lookahead_y_, 2.0))))
    /*if (lane following){
        steering = -50.96 * curvature ;
    }else{
        steering = -64.84738*curvature + 3.581326 *curvature*curvature*curvature;
    } */


    return -1 * static_cast<float>((delta_lookahead_x_) / (0.5 * (pow(delta_lookahead_x_, 2.0) +
                                                                  pow(delta_lookahead_y_, 2.0))));
                            
}

Point PurePursuit::convertToLocal(const Point &anchor, Point target) {
    //LOG_INFO("Anchor point: %f %f", anchor.getX(), anchor.getY());
    return anchor.globalToLocal(target, car_model->getHeading());
}
