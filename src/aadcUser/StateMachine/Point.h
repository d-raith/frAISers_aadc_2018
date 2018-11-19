//
// Created by aadc on 27.07.18.
//


#ifndef STATEMACHINE_POINT_H
#define STATEMACHINE_POINT_H

#include <math.h>
#include <iostream>
#include "ostream"
#include "opencv2/core/types.hpp"

#define UNIT_GLOBAL_COORDS Unit::CM
#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

namespace fraisers {
    namespace models {
        class Scalable {
        public:
            enum Unit {
                M = 1,
                DM = 10,
                CM = 100
            };

        protected:
            float current_scale_factor = 1;
            Unit unit = Unit::M;
        public:

            explicit Scalable(float scale) : current_scale_factor(scale) {}

            explicit Scalable(Unit scale) : current_scale_factor(scale), unit(scale) {}

            virtual void scaleBy(float scaleFactor) = 0;

            virtual float getActualScaling() const {
                return current_scale_factor;
            }

            void setUnit(Unit unit) {
                current_scale_factor = unit;
            }

            Unit getUnit() const {
                return unit;
            }

            float getScale() {
                return current_scale_factor;
            }
        };

        class Coordinate : public Scalable {
        public:
            enum Type {
                LOCAL = Unit::M,
                GLOBAL = UNIT_GLOBAL_COORDS
            };

            Type type;

            Coordinate(Type type) : Scalable(type), type(type) {

            }

            Coordinate(Type type, float scale) : Scalable(scale), type(type) {

            }

            void setType(Type type) {
                this->type = type;
            }

            Type getType() {
                return type;
            }

            float getActualScaling() const override {
                return current_scale_factor * type;
            }
        };


        class Heading {
        protected:
            float heading = 0;

            Heading() = default;

            explicit Heading(float value) : heading(value) {

            }

        public:

            float getHeading() const {
                return heading;
            }

            float getHeadingDegrees() const {
                return heading * static_cast<float>(RAD2DEG);
            }

            float getHeadingDegreesClamped() const {
                float degrees = heading * static_cast<float>(RAD2DEG);
                int sign = degrees < 0 ? -1 : 1;
                float abs_deg = fabs(degrees);

                if (abs_deg < 45) {
                    return 0;
                }

                if (abs_deg < 135) {
                    return sign * 90;
                } else if (abs_deg < 225) {
                    return 180;
                    //cases below should never be called
                } else if (abs_deg < 315) {
                    return 270;
                } else {
                    return 360;
                }

            }

            void setHeading(float heading) {
                this->heading = heading;
            }

            bool operator==(const Heading &other) const {
                return heading == other.heading;
            }

            bool operator!=(const Heading &other) const {
                return !(*this == other);
            }
        };


        class Point : public Coordinate {
        protected:
            float x_ = 0;
            float y_ = 0;
            float z_ = 0;

            explicit Point(float x, float y, float z, Coordinate::Type coord, Scalable::Unit unit) :
                    Coordinate(coord, unit), x_(x), y_(y), z_(z) {
            }

            explicit Point(float x, float y, Coordinate::Type coordType, Scalable::Unit unit)
                    : Point(x,
                            y,
                            0,
                            coordType,
                            unit) {
            }


        public:


            static Point GlobalM(float x, float y, float z) {
                return Point(x, y, z, Coordinate::GLOBAL, Unit::M);
            }

            static Point Global(float x, float y, float z) {
                return Point(x, y, z, Coordinate::GLOBAL, UNIT_GLOBAL_COORDS);
            }

            static Point Global(float x, float y) {
                return Point(x, y, 0, Coordinate::GLOBAL, UNIT_GLOBAL_COORDS);
            }

            static Point Local(float x, float y, float z) {
                return Point(x, y, z, Coordinate::LOCAL, Unit::M);
            }

            static Point Local(int x, int y, int z) {
                return Point(x, y, z, Coordinate::LOCAL, UNIT_GLOBAL_COORDS);
            }

            static Point Local(float x, float y, Unit unit) {
                return Point(x, y, 0, Coordinate::LOCAL, unit);
            }

            static Point Local(float x, float y) {
                return Point(x, y, 0, Coordinate::LOCAL, Unit::M);
            }

            static Point Local(int x, int y) {
                return Point(x, y, 0, Coordinate::LOCAL, UNIT_GLOBAL_COORDS);
            }

            ~Point() = default;

            float getX() const { return x_; }

            float getY() const { return y_; }

            float getZ() const { return z_; }

            void setX(float x) {
                x_ = x;
            }

            void setY(float y) {
                y_ = y;
            }

            void setZ(float z) {
                z_ = z;
            }

            bool isInitPoint() const {
                return x_ == 0 && y_ == 0 && z_ == 0;
            }

            float distanceTo(Point &other) const {
                return sqrt(pow(other.getX() - x_, 2) +
                            pow(other.getY() - y_, 2));
            }

            double distanceTo(const Point &other) const {
                return sqrt(pow(other.getX() - x_, 2) +
                            pow(other.getY() - y_, 2));
            }

            double distanceTo(const cv::Point2f &other) const {
                return sqrt(pow(other.x - x_, 2) +
                            pow(other.y - y_, 2));
            }

            static Point toLocal(const Point &pt) {
                return Point::Local(-pt.getY(), pt.getX(), pt.getZ());
            }

            const Point globalToLocal(const Point &local, float heading) const {

                float local_x_ = -1 * (-local.getX() * sin(heading) + local.getY() * cos(heading)
                                       + getX() * sin(heading) - getY() * cos(heading));

                float local_y_ = local.getX() * cos(heading) + local.getY() * sin(heading)
                                 - getX() * cos(heading) - getY() * sin(heading);

                return Point::Local(local_x_, local_y_);
            }

            void scaleBy(const float scaleFactor) override {
                x_ *= scaleFactor;
                y_ *= scaleFactor;
                z_ *= scaleFactor;
                current_scale_factor *= scaleFactor;
            }


            const void toGlobalNoCopy(Point &pt, float heading, float x_offset = 0,
                                      float y_offset = 0, float scale = 1) {
                float x = pt.getX() * scale * sin(heading)
                          + pt.getY() * scale * cos(heading) + getX();

                float y = -pt.getX() * scale * cos(heading)
                          + pt.getY() * scale * sin(heading) + getY();

                pt.setX(x + x_offset * cos(heading));
                pt.setY(y + y_offset * sin(heading));
            }

            const void toGlobalCV(cv::Point2f &pt, float heading, float x_offset = 0,
                                  float y_offset = 0, float scale = 1) {
                float x = pt.x * scale * sin(heading)
                          + pt.y * scale * cos(heading) + getX();

                float y = -pt.x * scale * cos(heading)
                          + pt.y * scale * sin(heading) + getY();

                pt.x = x + x_offset * cos(heading);
                pt.y = y + y_offset * sin(heading);
            }


            const Point toGlobal(float local_x, float local_y, float heading, float x_offset = 0,
                                 float y_offset = 0, float scale = 1) const {
                float x = local_x * scale * sin(heading)
                          + local_y * scale * cos(heading) + getX();

                float y = -local_x * scale * cos(heading)
                          + local_y * scale * sin(heading) + getY();

                return Point::Global(x + x_offset * cos(heading),
                                     y + y_offset * sin(heading),
                                     getZ());
            }

            const Point toGlobal(const Point &local, float heading, float x_offset = 0,
                                 float y_offset = 0) const {
                return toGlobal(local.getX(), local.getY(), heading);
            }

            Point toLocal(const Point &global, float heading) const {
                float x = global.getX();
                float y = global.getY();

                float local_x_ = -1 * (-x * sin(heading) + y * cos(heading)
                                       + getX() * sin(heading) - getY() * cos(heading));

                float local_y_ = x * cos(heading) + y * sin(heading)
                                 - getX() * cos(heading) - getY() * sin(heading);

                return Point::Local(local_x_, local_y_);
            }


            friend std::ostream &operator<<(std::ostream &os, Point point) {
                os << "[ x: " << point.x_ << ", y: " << point.y_
                   << "| sf: " << point.current_scale_factor << " | coord: " << point.type << " ]";
                //
                // << " z: "<< point.z_
                // <<" ]";
                return os;
            }

            Point &operator+=(const Point &rhs) {
                x_ += rhs.x_;
                y_ += rhs.y_;
                return *this;
            }

            // friends defined inside class body are inline and are hidden from non-ADL lookup
            Point operator+(const Point &rhs) {
                return Point(x_ + rhs.x_, y_ + rhs.y_, type, unit);
            }

            Point operator-(const Point &rhs) {
                return Point(x_ - rhs.x_, y_ - rhs.y_, type, unit);
            }

            Point operator*(const float scalar) {
                return Point(x_ * scalar, y_ * scalar, type, unit);
            }

            bool operator==(const Point &other) const {
                return x_ == other.x_ && y_ == other.y_ && z_ == other.z_;
            }

            bool operator!=(const Point &other) const {
                return !(*this == other);
            }


        };


        class Obstacle : public Point {
        public:
            enum Type {

                UNKNOWN = 0,
                CAR = 4,
                CHILD = 13,
                ADULT = 20
            };

            Obstacle(Point &point, Type type, float radius) : Point(point), type(type),
                                                              exp_x(radius), exp_y(radius) {}

            Obstacle(Point &point, Type type, float exp_x, float exp_y, float heading) : Point(point),
                                                                          type(type),
                                                                          exp_x(exp_x),
                                                                          exp_y(exp_y),
                                                                          heading(heading) {}

            Obstacle(Point &point, Type type) : Point(point), type(type) {}

            float getHeading() const {
                return heading;
            }


            cv::Rect getBoundingBox() const {
                int c_x = static_cast<int>(floor(x_ - exp_x / 2));
                int c_y = static_cast<int>(floor(y_ - exp_y / 2));
                int height = static_cast<int>(ceil(exp_x));
                int width = static_cast<int>(ceil(exp_y));
                return cv::Rect(c_x, c_y, width, height);
            }


            float getExpX() const {
                return exp_x;
            }

            float getExpY() const {
                return exp_y;
            }

            cv::Point2f getExpVector() {
                return cv::Point2f(exp_x, exp_y);
            }

            void scaleBy(const float scaleFactor) override {
                Obstacle::Point::scaleBy(scaleFactor);
                exp_x *= scaleFactor;
                exp_y *= scaleFactor;
            }

            Type getType() const {
                return type;
            }

            static Type getTypeBySegmentationInfo(int classId) {
                switch(classId){
                    case 4:
                        return CAR;
                    case 13:
                        return CHILD;
                    case 20:
                        return ADULT;
                    default:
                        return UNKNOWN;
                }
            }

            bool operator==(const Obstacle &other) const {
                return static_cast<Point>(*this) == static_cast<Point>(other)
                       && type == other.type && exp_x == other.exp_x && exp_y == other.exp_y;
            }

            bool operator!=(const Obstacle &other) const {
                return !(*this == other);
            }

            friend std::ostream &operator<<(std::ostream &os, Obstacle obs) {
                os << static_cast<Point>(obs) << std::endl;
                os << "[ rx: " << obs.exp_x << ", ry: " << obs.exp_y << ", type " << obs.type
                   << " ]";
                return os;
            }

        private:
            const Type type = UNKNOWN;
            //expansion of obstacle in x,y directions
            float exp_x = 0;
            float exp_y = 0;
            int heading = -1;
        };


        /* class CarPosition : public Point, public Heading {
         protected:
             CarPosition(float x, float y, float z, float heading, Unit scale) : Point(x, y, z,
                                                                                       Coordinate::GLOBAL,
                                                                                       scale),
                                                                                 Heading(heading) {

             }

             CarPosition(float x, float y, float heading, Unit scale) : CarPosition(x, y, 0,
                                                                                    heading,
                                                                                    scale) {

             }

         public:
             static CarPosition Global(float x, float y, float z) {
                 return CarPosition(x, y, z, Unit::M);
             }

             static CarPosition Global(float x, float y) {
                 return CarPosition(x, y, 0, Unit::M);
             }

             static CarPosition Global(int x, int y) {
                 return CarPosition(x, y, 0, Unit::DM);
             }


             ~CarPosition() = default;

             const void toGlobalNoCopy(Point &pt, float x_offset = 0,
                                       float y_offset = 0, float scale = 1) {
                 float x = pt.getX() * scale * sin(getHeading())
                           + pt.getY() * scale * cos(getHeading()) + getX();

                 float y = -pt.getX() * scale * cos(getHeading())
                           + pt.getY() * scale * sin(getHeading()) + getY();

                 pt.setX(x + x_offset * cos(getHeading()));
                 pt.setY(y + y_offset * sin(getHeading()));
             }

             const void toGlobalCV(cv::Point2f &pt, float x_offset = 0,
                                   float y_offset = 0, float scale = 1) {
                 float x = pt.x * scale * sin(getHeading())
                           + pt.y * scale * cos(getHeading()) + getX();

                 float y = -pt.x * scale * cos(getHeading())
                           + pt.y * scale * sin(getHeading()) + getY();

                 pt.x = x + x_offset * cos(getHeading());
                 pt.y = y + y_offset * sin(getHeading());
             }


             const Point toGlobal(float local_x, float local_y, float x_offset = 0,
                                  float y_offset = 0, float scale = 1) {
                 float x = local_x * scale * sin(getHeading())
                           + local_y * scale * cos(getHeading()) + getX();

                 float y = -local_x * scale * cos(getHeading())
                           + local_y * scale * sin(getHeading()) + getY();

                 return Point::Global(x + x_offset * cos(getHeading()),
                                      y + y_offset * sin(getHeading()),
                                      getZ());
             }

             const Point toGlobal(const Point &local, float x_offset = 0, float y_offset = 0) {
                 return toGlobal(local.getX(), local.getY());
             }

             Point toLocal(const Point &global) const {
                 float heading = getHeading();
                 float x = global.getX();
                 float y = global.getY();

                 float local_x_ = -1 * (-x * sin(heading) + y * cos(heading)
                                        + getX() * sin(heading) - getY() * cos(heading));

                 float local_y_ = x * cos(heading) + y * sin(heading)
                                  - getX() * cos(heading) - getY() * sin(heading);

                 return Point::Local(local_x_, local_y_);
             }
         };*/

        class Marker : public Point, Heading {
        public:


            enum MarkerType {
                CROSSING = 0,
                STOP = 1,
                PARKING_AHEAD = 2,
                RIGHT_OF_WAY = 3,
                STRAIGHT_ONLY = 4,
                GIVE_WAY = 5,
                ZEBRA = 6,
                ROUNDABOUT = 7,
                NO_OVERTAKING = 8,
                NO_ENTERING = 9,
                ONE_WAY = 11,
                CONSTRUCTION = 12,
                SPEED_LIMIT_50 = 13,
                SPEED_LIMIT_100 = 14,
                POSITIONING = 29
            };

            MarkerType marker_type;

            Marker(int id, float x, float y, float heading) : Point(x, y, 0, Coordinate::GLOBAL,
                                                                    Unit::M),
                                                              Heading(heading) {
                if (CROSSING >= 0 && id <= SPEED_LIMIT_100) {
                    marker_type = static_cast<MarkerType>(id);
                } else {
                    marker_type = POSITIONING;
                }
            }


            std::string getReadableType() const {
                switch (marker_type) {

                    case CROSSING:
                        return "CROSSING";
                    case STOP:
                        return "STOP";
                    case PARKING_AHEAD:
                        return "PARKING_AHEAD";
                    case RIGHT_OF_WAY:
                        return "RIGHT_OF_WAY";
                    case STRAIGHT_ONLY:
                        return "STRAIGHT_ONLY";
                    case GIVE_WAY:
                        return "GIVE_WAY";
                    case ZEBRA:
                        return "ZEBRA";
                    case ROUNDABOUT:
                        return "ROUNDABOUT";
                    case NO_OVERTAKING:
                        return "NO_OVERTAKING";
                    case NO_ENTERING:
                        return "NO_ENTERING";
                    case ONE_WAY:
                        return "ONE_WAY";
                    case CONSTRUCTION:
                        return "CONSTRUCTION";
                    case SPEED_LIMIT_50:
                        return "SPEED_LIMIT_50";
                    case SPEED_LIMIT_100:
                        return "SPEED_LIMIT_100";
                    case POSITIONING:
                        return "POSITIONING";
                }
                return "UNKNOWN: " + std::to_string(marker_type);
            }

            MarkerType getMarkerType() const {
                return marker_type;
            }


            bool operator==(const Marker &other) const {
                return static_cast<Point>(*this) == static_cast<Point>(other) &&
                       this->marker_type == other.marker_type && this->heading == other.heading;
            }

            bool operator!=(const Marker &other) const {
                return !(*this == other);
            }

            friend std::ostream &operator<<(std::ostream &os, Marker &marker) {


                os << "[ marker: " << std::endl;
                os << static_cast<Point>(marker) << std::endl;
                os << "Type: " << marker.getReadableType() << std::endl;
                return os;
            }
        };

    }
}
#endif //STATEMACHINE_POINT_H