//
// Created by aadc on 27.09.18.
//

#include "CarModel.h"

DeltaPosition CarModel::updatePosition(tPosition pos) {

    float new_x = pos.f32x * Coordinate::GLOBAL;
    float new_y = pos.f32y * Coordinate::GLOBAL;
    DeltaPosition delta;
    delta.dx = position_rear_axis.getX() - new_x;
    delta.dy = position_rear_axis.getY() - new_y;
    delta.setHeading(getHeading()-pos.f32heading);
    position_rear_axis.setX(new_x);
    position_rear_axis.setY(new_y);
    setHeading(pos.f32heading);

    position_front_axis = toGlobal(Point::Local(0.0, offset_front_axis * Coordinate::GLOBAL));
    position_laser_scanner = toGlobal(Point::Local(0.0, offset_laserscanner * Coordinate::GLOBAL));
    last_delta = delta;

    is_initialized = !position_rear_axis.isInitPoint();
    return last_delta;
}

const Point& CarModel::getFrontAxis() {
    return position_front_axis;
}

const Point& CarModel::getLaserScanner() {
    return position_laser_scanner;
}


Point CarModel::getBirdseyeAnchor() {
    return getRearAxis().toGlobal(
        Point::Local(getLocalBeyeAnchor(Coordinate::GLOBAL).x,
        getLocalBeyeAnchor(Coordinate::GLOBAL).y), getHeading());
}

vector<Point> CarModel::getFrontWheels() {
    Point wheel_left = toGlobal(Point::Local(car_width/2* Coordinate::GLOBAL, offset_front_axis * Coordinate::GLOBAL));
    Point wheel_right = toGlobal(Point::Local(-car_width/2* Coordinate::GLOBAL, offset_front_axis *
    Coordinate::GLOBAL));
    return {wheel_left, wheel_right};
}

const Point CarModel::getRearAxis() {
    return position_rear_axis;
}

DeltaPosition CarModel::getLastPositionDelta() {
    return last_delta;
}

Point CarModel::getCameraLocation() {
    return toGlobal(Point::Local(0.0, offset_camera_rear_m * Coordinate::GLOBAL));
}

cv::Point2f CarModel::getFrontAxisCV() {
    cv::Point2f cam (0.0, offset_front_axis * Coordinate::GLOBAL);
    toGlobalCV(cam);
    return cam;
}

cv::Point2f CarModel::getCameraLocationCV() {
    cv::Point2f cam (0, offset_camera_rear_m * Coordinate::GLOBAL);
    toGlobalCV(cam);
    return cam;
}

cv::Point2f CarModel::getRearAxisCV() {
    return cv::Point2f(position_rear_axis.getX(), position_rear_axis.getY());

}

cv::Point2f CarModel::getLocalRearAxis(float pixel_per_meter) {
    return cv::Point2f(0, 0);
}

cv::Point2f CarModel::getLocalFrontAxis(float pixel_per_meter) {
    return getLocalRearAxis(pixel_per_meter) + cv::Point2f(
        0, distance_rearaxis_frontaxis * pixel_per_meter);
}

cv::Point2f CarModel::getLocalFront(float pixel_per_meter) {
    return getLocalFrontAxis(pixel_per_meter) + cv::Point2f(
        0, distance_frontaxis_front * pixel_per_meter);
}

cv::Point2f CarModel::getLocalCamera(float pixel_per_meter) {
    return getLocalRearAxis(pixel_per_meter) + cv::Point2f(
        0, distance_rearaxis_camera * pixel_per_meter);
}

cv::Point2f CarModel::getLocalBeyeAnchor(float pixel_per_meter) {
    return getLocalFrontAxis(pixel_per_meter) + cv::Point2f(
        0, distance_frontaxis_birdseye_crop * pixel_per_meter);
}

cv::Point2f CarModel::getLocalWheelLeft(float pixel_per_meter) {
    return cv::Point2f(
        distance_center_wheel * pixel_per_meter, 0);
}

cv::Point2f CarModel::getLocalWheelRight(float pixel_per_meter) {
    return cv::Point2f(
        -1 * distance_center_wheel * pixel_per_meter, 0);
}
