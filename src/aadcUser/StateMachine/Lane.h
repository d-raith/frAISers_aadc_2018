#pragma once

#include <vector>
#include "Point.h"
#include "customtypes.h"
#include "math.h"
#include "opencv2/core/types.hpp"

using namespace fraisers::models;


class Lane {
public:
    enum orientation_type {
        vertical,
        horizontal,
        unknown
    };

    vector<float> coeff;
    orientation_type orientation = unknown;
    int alignment = 0;  // 2, 1, -1, -2 (2,1, ..  left from car
    float dist = -1;              // either vertical or lateral distance from car
    float angle_origin_rad = 0;  // angle at x=0; pos. in direction of y
    float lane_quality = 0;  // 0: don't trust if possible! 1: good fit, good length
    Point start_pnt = Point::Local(0, 0, 0);
    Point end_pnt = Point::Local(0, 0, 0);


    Lane() = default;


public:

    friend std::ostream &operator<<(std::ostream &os, Lane lane) {
        os << "[ start: " << lane.start_pnt << endl;
        os << "end: " << lane.end_pnt << endl;

        os << "coeffs (x^0, x^1, x^2): " << endl;
        for (auto &c : lane.coeff) {
            os << c << " ";
        }
        os << endl;

        os << "align: " << lane.alignment << " | " << "angle: " << lane.angle_origin_rad << endl;
        os << "dist: " << lane.dist << " | " << "orientation: " << lane.orientation << " ]" << endl;
        return os;
    }

    explicit Lane(tLaneElement element) :
            orientation(static_cast<orientation_type>((element.orientation))),
            alignment(element.alignment), dist(element.dist),
            angle_origin_rad(element.angleOriginRad),
            start_pnt(Point::Local(element.startx, element.starty)),
            end_pnt(Point::Local(element.endx, element.endy)) {
        coeff = {element.coeff0, element.coeff1, element.coeff2};
    }

    // coefficients in ascending order c+bx+ax^2
    const vector<float> &getCoefficients() {
        return coeff;
    }


    bool isRightLane() {
        return start_pnt.getY() < 0;
    }

    bool isLeftLane() {
        return start_pnt.getY() >= 0;
    }

    bool operator<(const Lane &rhs) {
        return start_pnt.getY() > rhs.start_pnt
                .getY();
    };

    bool operator>(const Lane &rhs) {
        return start_pnt.getY() < rhs.start_pnt
                .getY();
    };

    inline bool operator<=(Lane &rhs) { return !(*this > rhs); }

    float getValAtX(float x) {
        float res = 0;
        for (unsigned int i = 0; i < coeff.size(); ++i) {
            res += coeff[i] * pow(x, i);
        }
        return res;
    }


    float getDerivAtX(float x) {
        float res = 0;
        for (unsigned int i = 1; i < coeff.size(); ++i) {
            res += coeff[i] * pow(x, i - 1);
        }
        return res;
    }


    static bool getLaneCenterPoints(std::vector<Point> *center_point_result, Lane
    *lane_left, Lane *lane_right, uint sample_range = 10, bool switchCoordinates = true, float
                                    lane_width = 0.47) {
        //center_point_result->reserve(center_point_result->size() + sample_range);

        float min_sample_x = 0;
        if (lane_left) {
            min_sample_x = lane_left->start_pnt.getX();
        }
        if (lane_right) {
            min_sample_x = fmax(min_sample_x, lane_right->start_pnt.getX());
        }

        float sampleTo = 0;
        if (lane_left) {
            sampleTo = lane_left->end_pnt.getX();
        }
        if (lane_right) {
            sampleTo = fmax(sampleTo, lane_right->end_pnt.getX());
        }


    min_sample_x *= 10;
    sampleTo *= 15;

    for (float ix = min_sample_x; ix < sampleTo; ix+=0.1) {
            auto x_val = static_cast<float>(ix / 10.0);
            Point center = Point::Local(0,0);

            getLaneCenterPoint(x_val, &center, lane_left,
                               lane_right, switchCoordinates, lane_width);
            if (!center.isInitPoint()) {
                center_point_result->emplace_back(center);
            }
        }

        return true;
    }


    static bool getLaneCenterPoint(float x_dist, Point *centerPoint, Lane *lane_left, Lane
    *lane_right, bool switchCoordinates = true, float lane_width = 0.47) {
        /* This method evaluates one or two lane objects and returns waypoint for
         * given x-distance */
        double car_center_dist = 0.5 * lane_width;
        float normal_angle;
        float lane_y_at_x;
        if (!lane_left && !lane_right) {
            cout << "No lanes available for center point calculation!" << std::endl;
            return false;
        }

        /***** single lane marking given *****/
        if (!lane_left || !lane_right) {
            Lane *lane_tmp;
            if (lane_left != nullptr) {
                lane_tmp = lane_left;
            } else {
                lane_tmp = lane_right;
            }
            normal_angle = atan(lane_tmp->getDerivAtX(x_dist));
            if (lane_tmp == lane_left) {
                //  marking left to car
                normal_angle += -0.5 * M_PI;
            } else {
                //  marking right to car
                normal_angle += 0.5 * M_PI;
            }

            lane_y_at_x = lane_tmp->getValAtX(x_dist);
            double x = x_dist + car_center_dist * cos(normal_angle);
            double y = lane_y_at_x + car_center_dist * sin(normal_angle);
            if (switchCoordinates) {
                centerPoint->setX(static_cast<float>(-y));
                centerPoint->setY(static_cast<float>(x));
            } else {
                centerPoint->setX(static_cast<float>(x));
                centerPoint->setY(static_cast<float>(y));
            }

            return true;
        }

        /***** two lane markings given: ******/
        float abs_y_right = fabs(lane_right->start_pnt.getY());
        if (abs_y_right < 0.35) {
            return getLaneCenterPoint(x_dist, centerPoint, NULL, lane_right);
        }

        /*
        if ((lane_right->getStartPoint().GetX() + lane_left->getStartPoint().GetX()) > 1) {
             // Making the left lane null if the distance between left and right is more than 0.5 units
             cout<<"start points too far away, setting left to null "<< lane_right->getStartPoint().GetX() + lane_left->getStartPoint().GetX() <<std::endl;
             return getLaneCenterPoint(x_dist, nullptr, lane_right);
        }
        if ((lane_right->getStartPoint().GetX() + lane_left->getStartPoint().GetX()) < -1){
            cout<<"start points too far away, setting right to null "<< lane_right->getStartPoint().GetX() + lane_left->getStartPoint().getX() <<std::endl;
            return getLaneCenterPoint(x_dist, lane_left, nullptr);
        }*/

        double dist_from_target_lane = 0.5 * lane_width;
        double normal_angle_l = atan(lane_left->getDerivAtX(x_dist)) + M_PI;

        double normal_angle_r = atan(lane_right->getDerivAtX(x_dist)) - M_PI;

        double delta_x_l = dist_from_target_lane * cos(normal_angle_l);
        double delta_y_l = dist_from_target_lane * sin(normal_angle_l);

        double delta_x_r = dist_from_target_lane * cos(normal_angle_r);
        double delta_y_r = dist_from_target_lane * sin(normal_angle_r);

        double x = (x_dist + delta_x_l + x_dist + delta_x_r) / 2;
        double y = (lane_left->getValAtX(x_dist) + delta_y_l
                    + lane_right->getValAtX(x_dist) + delta_y_r) / 2;
        if (switchCoordinates) {
            centerPoint->setX(static_cast<float>(-y));
            centerPoint->setY(static_cast<float>(x));
        } else {
            centerPoint->setX(static_cast<float>(x));
            centerPoint->setY(static_cast<float>(y));
        }

        return true;
    }


    static bool getLaneCenterPoints(std::vector<cv::Point2f> *center_point_result, Lane
    *lane_left, Lane *lane_right, uint sample_range = 10, float scale = 1, float
                                    lane_width = 0.47) {
        //center_point_result->reserve(center_point_result->size() + sample_range);

        float sampleFrom = 0;
        if (lane_left) {
            sampleFrom = lane_left->start_pnt.getX();
        }
        if (lane_right) {
            sampleFrom = fmax(sampleFrom, lane_right->start_pnt.getX());
        }

        float sampleTo = 0;
        if (lane_left) {
            sampleTo = lane_left->end_pnt.getX();
        }
        if (lane_right) {
            sampleTo = fmax(sampleTo, lane_right->end_pnt.getX());
        }

        sampleTo *= 10;
        sampleFrom *= 15;

        for (float ix = sampleFrom; ix < sampleTo; ix+=0.1) {
            auto x_val = static_cast<float>(ix / 10.0);
            cv::Point2f center;

            getLaneCenterPoint(x_val, &center, lane_left,
                               lane_right, scale, lane_width);
            if (center.x != 0 && center.y != 0) {

                center_point_result->emplace_back(center);
            }
        }
        return true;
    }


    static bool getLaneCenterPoint(float x_dist, cv::Point2f *centerPoint, Lane *lane_left, Lane
    *lane_right, float scale = 1, float lane_width = 0.47) {
        /* This method evaluates one or two lane objects and returns waypoint for
         * given x-distance */
        double car_center_dist = 0.5 * lane_width;
        float normal_angle;
        float lane_y_at_x;
        if (!lane_left && !lane_right) {
            cout << "No lanes available for center point calculation!" << std::endl;
            return false;
        }

        /***** single lane marking given *****/
        if (!lane_left || !lane_right) {
            Lane *lane_tmp;
            if (lane_left != nullptr) {
                lane_tmp = lane_left;
            } else {
                lane_tmp = lane_right;
            }
            normal_angle = atan(lane_tmp->getDerivAtX(x_dist));
            if (lane_tmp == lane_left) {
                //  marking left to car
                normal_angle += -0.5 * M_PI;
                car_center_dist = 0.5 * lane_width;
            } else {
                //  marking right to car
                normal_angle += 0.5 * M_PI;
                car_center_dist = 0.35 * lane_width;
            }

            lane_y_at_x = lane_tmp->getValAtX(x_dist);
            double x = x_dist + car_center_dist * cos(normal_angle);
            double y = lane_y_at_x + car_center_dist * sin(normal_angle);

            centerPoint->x = static_cast<float>(-y * scale);
            centerPoint->y = static_cast<float>(x * scale);


            return true;
        }

        /***** two lane markings given: ******/
         float abs_y_right = fabs(lane_right->start_pnt.getY());
        if (abs_y_right < 0.35) {
            return getLaneCenterPoint(x_dist, centerPoint, NULL, lane_right);
        }

        /*
        if ((lane_right->getStartPoint().GetX() + lane_left->getStartPoint().GetX()) > 1) {
             // Making the left lane null if the distance between left and right is more than 0.5 units
             cout<<"start points too far away, setting left to null "<< lane_right->getStartPoint().GetX() + lane_left->getStartPoint().GetX() <<std::endl;
             return getLaneCenterPoint(x_dist, nullptr, lane_right);
        }
        if ((lane_right->getStartPoint().GetX() + lane_left->getStartPoint().GetX()) < -1){
            cout<<"start points too far away, setting right to null "<< lane_right->getStartPoint().GetX() + lane_left->getStartPoint().getX() <<std::endl;
            return getLaneCenterPoint(x_dist, lane_left, nullptr);
        }*/

        double dist_from_target_lane = 0.5 * lane_width;
        double normal_angle_l = atan(lane_left->getDerivAtX(x_dist)) + M_PI;

        double normal_angle_r = atan(lane_right->getDerivAtX(x_dist)) - M_PI;

        double delta_x_l = dist_from_target_lane * cos(normal_angle_l);
        double delta_y_l = dist_from_target_lane * sin(normal_angle_l);

        double delta_x_r = dist_from_target_lane * cos(normal_angle_r);
        double delta_y_r = dist_from_target_lane * sin(normal_angle_r);

        double x = (x_dist + delta_x_l + x_dist + delta_x_r) / 2;
        double y = (lane_left->getValAtX(x_dist) + delta_y_l
                    + lane_right->getValAtX(x_dist) + delta_y_r) / 2;
        centerPoint->x = static_cast<float>(-y * scale);
        centerPoint->y = static_cast<float>(x * scale);

        return true;
    }


static void extractCurrentRoadLanes(vector<Lane> *laneData, Lane **left, Lane **right) {
    for (auto &data : *laneData) {
        Lane *lane = &data;
        if (lane->start_pnt.getY() >= 0.02 && lane->start_pnt.getY() < 0.20) {
            *left = lane;
            break;
        } else if (lane->start_pnt.getY() < -0.10 && lane->start_pnt.getY() > -0.60) {
                *right = lane;
        }
    }
}
};
struct Road {
    Lane *left = nullptr;
    Lane *right = nullptr;

    void initFrom(Lane *lane1, Lane *lane2) {
        if (lane1 && lane2) {
            if (*lane1 < *lane2) {
                left = lane1;
                right = lane2;
            } else {
                left = lane2;
                right = lane1;
            }
        } else if (lane1) {
            if (lane1->isLeftLane()) {
                left = lane1;
            } else {
                right = lane1;
            }
        } else if (lane2) {
            if (lane2->isLeftLane()) {
                left = lane2;
            } else {
                right = lane2;
            }
        }

    }

    friend std::ostream &operator<<(std::ostream &os, Road road) {
        os << "[ Road: " << endl;
        if (road.left) {
            os << "Left: " << endl << *road.left << endl;
        }
        if (road.right) {
            os << "Right: " << endl << *road.right << endl;
        }
        os << "     ]";

        return os;
    }

};