//
// Created by aadc on 27.09.18.
//

#ifndef AADC_USER_CARMODEL_H
#define AADC_USER_CARMODEL_H

#include <memory>
#include "Point.h"
#include "stdafx.h"

using namespace fraisers::models;

struct DeltaPosition: public Heading {
    DeltaPosition() {   }

    DeltaPosition(float x, float y, float p_heading) {
        dx = x;
        dy = y;
        heading = p_heading;
    }

    float dx = 0;
    float dy = 0;
};

class CarModel : public Heading {

    DeltaPosition last_delta;
    bool is_initialized = false;
    static constexpr float offset_front_axis = 0.36;
    static constexpr float offset_laserscanner = 0.45;

    static constexpr float offset_camera_rear_m = 0.295;

    static constexpr float car_width = 0.32; // car width in m
    static constexpr float car_length = 0.58; // car width in m


    float distance_rearaxis_frontaxis = 0.36;   // meter
    float distance_rearaxis_camera = 0.29;   // meter
    float distance_center_wheel = 0.13;
    float distance_frontaxis_birdseye_crop = 0.0925;
    float distance_frontaxis_front = 0.12;


    Point position_rear_axis = Point::Global(0, 0);
    Point position_front_axis = Point::Global(0, 0);
    Point position_laser_scanner = Point::Global(0, 0);

    CarModel() = default;

public:
    static std::shared_ptr<CarModel> init() {
        return std::make_shared<CarModel>(CarModel());
    }


    DeltaPosition updatePosition(tPosition pos);


    DeltaPosition getLastPositionDelta();


    const Point& getFrontAxis();

    const Point& getLaserScanner();

    vector<Point> getFrontWheels();

    Point getCameraLocation();

    Point getBirdseyeAnchor();

    const Point getRearAxis();


    cv::Point2f getFrontAxisCV();

    cv::Point2f getCameraLocationCV();

    cv::Point2f getRearAxisCV();

    cv::Point2f getLocalRearAxis(float pixel_per_meter);

    cv::Point2f getLocalFrontAxis(float pixel_per_meter);

    cv::Point2f getLocalFront(float pixel_per_meter);

    cv::Point2f getLocalCamera(float pixel_per_meter);

    cv::Point2f getLocalBeyeAnchor(float pixel_per_meter);

    cv::Point2f getLocalWheelLeft(float pixel_per_meter);

    cv::Point2f getLocalWheelRight(float pixel_per_meter);

    bool isInitialized() {
        return is_initialized;
    }


    const void toGlobalNoCopy(Point &pt, float x_offset = 0,
                              float y_offset = 0, float scale = 1) {
        float x = pt.getX() * scale * sin(getHeading())
                  + pt.getY() * scale * cos(getHeading()) + position_rear_axis.getX();

        float y = -pt.getX() * scale * cos(getHeading())
                  + pt.getY() * scale * sin(getHeading()) + position_rear_axis.getY();

        pt.setX(x + x_offset * cos(getHeading()));
        pt.setY(y + y_offset * sin(getHeading()));
    }

    const void toGlobalCV(cv::Point2f &pt, float x_offset = 0,
                          float y_offset = 0, float scale = 1) {
        float x = pt.x * scale * sin(getHeading())
                  + pt.y * scale * cos(getHeading()) + position_rear_axis.getX();

        float y = -pt.x * scale * cos(getHeading())
                  + pt.y * scale * sin(getHeading()) + position_rear_axis.getY();

        pt.x = x + x_offset * cos(getHeading());
        pt.y = y + y_offset * sin(getHeading());
    }


    const Point toGlobal(float local_x, float local_y, float x_offset = 0,
                         float y_offset = 0, float scale = 1) {
        float x = local_x * scale * sin(getHeading())
                  + local_y * scale * cos(getHeading()) + position_rear_axis.getX();

        float y = -local_x * scale * cos(getHeading())
                  + local_y * scale * sin(getHeading()) + position_rear_axis.getY();

        return Point::Global(x + x_offset * cos(getHeading()),
                             y + y_offset * sin(getHeading()),
                             position_rear_axis.getZ());
    }

    const Point toGlobal(const Point &local, float x_offset = 0, float y_offset = 0) {
        return toGlobal(local.getX(), local.getY());
    }

    Point toLocal(const Point &global) const {
        float heading = getHeading();
        float x = global.getX();
        float y = global.getY();

        float local_x_ = -1 * (-x * sin(heading) + y * cos(heading)
                               + position_rear_axis.getX() * sin(heading) - position_rear_axis
                                                                                    .getY() *
                                                                            cos(heading));

        float local_y_ = x * cos(heading) + y * sin(heading)
                         - position_rear_axis.getX() * cos(heading) - position_rear_axis.getY() *
                                                                      sin(heading);

        return Point::Local(local_x_, local_y_);
    }


    friend std::ostream &operator<<(std::ostream &os, CarModel &model) {
        os << "Car state: " << endl;
        os << "Heading: " << model.getHeading() << "\n";
        os << "Rear axis: " << model.position_rear_axis << "\n";
        os << "Front axis: " << model.getFrontAxis() << "\n";
        os << "Camera: " << model.getCameraLocation() << "\n";
        os << "Last delta: dx: " << model.last_delta.dx << " dy: " << model.last_delta.dy <<"\n";
        os << "      ##########      " << "\n";

        return os;
    }

};


class ICarModelDependent {
protected:
    std::shared_ptr<CarModel> car_model;
public:
    virtual void setCarModel(std::shared_ptr<CarModel> &carModel) {
        car_model = carModel;
    }

};


#endif //AADC_USER_CARMODEL_H
