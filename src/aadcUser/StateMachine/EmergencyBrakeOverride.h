//
// Created by aadc on 29.09.18.
//

#ifndef AADC_USER_EMERGENCYBRAKEOVERRIDE_H
#define AADC_USER_EMERGENCYBRAKEOVERRIDE_H

#include "ICtrlOverride.h"
#include "ISensorListeners.h"
#include "ICarCtrl.h"
#include "CarModel.h"
#include "../PinClasses/StopWatch.h"
class EmergencyBrakeOverride: public SpeedCtrlOverride,
        public ICarModelDependent,
        public IEmergencyBrakeCtrl {

    static constexpr float ls_angle_left =  335;
    static constexpr float ls_angle_right = 25;

    bool emergency_brake_active = false;
    bool emergency_brake_enabled = true;




    float th_distance_m;
    StopWatch last_em_trigger;
    float current_curvature_steering = 0;


    void setActive(bool em_brake_required) {
        if (emergency_brake_active != em_brake_required) {
            emergency_brake_active = em_brake_required;
            onStateChanged(emergency_brake_active);
        }
    }



    void onStateChanged(bool current) {
        if (current) {
            LOG_INFO("Emergency brake enabled");
        } else {
            LOG_INFO("Emergency brake disabled");
        }
    }


public:


    float getBreakDistanceCM() {
        return th_distance_m * 100;
    }

    bool isActive() {
        return emergency_brake_enabled && emergency_brake_active;
    }

    bool isEnabled() override {
        return emergency_brake_enabled;
    }


     void setEmergencyBrakeEnabled(bool enabled) override {
        emergency_brake_enabled = enabled;
    }

    StopWatch* getLastTriggerWatch() override {
        return &last_em_trigger;
    }
    explicit EmergencyBrakeOverride(float th_distance_m):th_distance_m
    (th_distance_m) {

    }

    bool onOverrideSpeed(float *const speed) override {

        if (*speed <= Speed::STOP) {
            // no need to enforce em brake when stopping or going reverse
            return false;
        }

        if (emergency_brake_enabled) {
            if (emergency_brake_active || !last_em_trigger.didSecondsPass(2)) {
                (*speed) = Speed::STOP;
            }
        }


        return emergency_brake_active && emergency_brake_enabled;
    }


    void updateSteeringValue(float steeringCurvature) {
        current_curvature_steering = steeringCurvature;
    }


    // /** Check for requirement of em brake here, called when pt in generic angle range & distance
    //  * threshold triggered */
    // bool onEmergencyBrakeCandidatePointDetected(tPolarCoordiante *raw_pt, Point *converted_global) {
    //     // local variables passed, do not store pointers for later processing
    //     return onEmergencyBrakeRequired(raw_pt, converted_global);
    // }


    bool onEmergencyBrakeRequired() {
        // em brake currently required, reset last trigger timer
        last_em_trigger.reset();
        setActive(true);
        return true;
    }

    void onEmergencyBrakeNotRequired() {
        setActive(false);
    }

    float getCurvatureSteering() {
        return current_curvature_steering;
    }

    float cvPoint2fDistance(cv::Point2f a, cv::Point2f b) {
        return sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
    }

    int cvPoint2iDistance(cv::Point2i a, cv::Point2i b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    void onLaserScannerUpdate(const vector<tPolarCoordiante>& ls_data,
                              vector<Point>& global_cartesian_ls_points,
                              Point steering_anchor,
                              shared_ptr<CarModel> car_model) {
        // StopWatch watch;
        float em_brake_corridor_width = 20;
        float em_brake_distance_frontaxis = getBreakDistanceCM();
        float steering_curvature = getCurvatureSteering();
        if (abs(steering_curvature) > 0.001) {
            float steering_radius = (1.0 / steering_curvature) * 100.0;
            Point local_steer_anchor = car_model->getRearAxis().toLocal(steering_anchor, car_model->getHeading());

            cv::Point steering_center = cv::Point2i(local_steer_anchor.getX(), local_steer_anchor.getY())
                                            - cv::Point2i(steering_radius, 0);

            int i = 0;
            for (const tPolarCoordiante &ls_pt : ls_data) {
                if (ls_pt.f32Radius == 0) {
                    i++;
                    continue;
                }
                Point scanpoint = global_cartesian_ls_points[i];
                Point local_scan_pt = car_model->getRearAxis().toLocal(scanpoint, car_model->getHeading());
                cv::Point2i local_scanpoint = cv::Point2i(local_scan_pt.getX(), local_scan_pt.getY());
                int dist_steeringcenter_scanpoint = cvPoint2iDistance(steering_center, local_scanpoint);
                bool inCorridor = (dist_steeringcenter_scanpoint
                                    > abs(steering_radius) - em_brake_corridor_width)
                                        && (dist_steeringcenter_scanpoint
                                            < abs(steering_radius) + em_brake_corridor_width);

                
                // if (inCorridor) {
                // LOG_INFO("scanpoint.x=%f, scanpoint.y=%f", scanpoint.getX(), scanpoint.getY());
                // LOG_INFO("inCorrdor=%i", inCorridor);
                // LOG_INFO("car_model->getFrontAxis().distanceTo(scanpoint)=%f", car_model->getFrontAxis().distanceTo(scanpoint));
                // }
                if (inCorridor
                        && car_model->getFrontAxis().distanceTo(scanpoint)
                            < em_brake_distance_frontaxis) {
                    onEmergencyBrakeRequired();
                    return;
                }
                i++;
            }
        } else {
            // LOG_INFO("debug embrake: straight case");
            // check for straight
            int i = 0;
            for (const tPolarCoordiante &ls_pt : ls_data) {
                if (ls_pt.f32Radius == 0) {
                    i++;
                    continue;
                }
                Point scanpoint = global_cartesian_ls_points[i];
                // std::cout << "scanpoint" << scanpoint << std::endl;
                Point local_scan_pt = car_model->getRearAxis().toLocal(scanpoint, car_model->getHeading());
                // std::cout << "local_scan_pt" << local_scan_pt << std::endl;
                cv::Point2i local_scanpoint = cv::Point2i(local_scan_pt.getX(), local_scan_pt.getY());
                float dist = car_model->getFrontAxis().distanceTo(scanpoint);
                // std::cout << "dist" << dist << std::endl;
                if (dist
                            < getBreakDistanceCM()
                        && local_scanpoint.x
                            > - em_brake_corridor_width
                        && local_scanpoint.x
                            <  em_brake_corridor_width) {
                    onEmergencyBrakeRequired();
                    return;
                }
                i++;
            }
        }
        onEmergencyBrakeNotRequired();
        // watch.print_measurement("em_brake_override update");

    // original version
    //     int i = 0;
    //     for (const tPolarCoordiante &ls_pt : ls_data) {

    //         if (ls_pt.f32Radius == 0) {
    //             continue;
    //         }
    //         tPolarCoordiante coord = ls_pt;
    //         coord.f32Radius /= 1000;
    //         if (coord.f32Radius < th_distance_m
    //                 && (coord.f32Angle <= ls_angle_right || coord.f32Angle >= ls_angle_left)) {

    //             Point lsDataPt = global_cartesian_ls_points[i];
    //             if (onEmergencyBrakeCandidatePointDetected()) {
    //                 return;
    //             }
    //         }
    //         i++;
    //     }
    //     onEmergencyBrakeNotRequired();
    }
};

#endif //AADC_USER_EMERGENCYBRAKEOVERRIDE_H
