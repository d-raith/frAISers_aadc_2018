
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#ifndef MAP_MAPCORE_MAPOBJECTS_H_
#define MAP_MAPCORE_MAPOBJECTS_H_
#pragma once

#include <math.h>
#include <vector>
#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>

#include <boost/uuid/uuid.hpp>                  // NOLINT
#include <boost/uuid/uuid_generators.hpp>       // NOLINT
#include <boost/uuid/uuid_io.hpp>               // NOLINT
#include <boost/lexical_cast.hpp>               // NOLINT


namespace mapobjects {
enum LaneDirection {
    RIGHT = 1,
    LEFT = 2
};

enum LaneType {
    NORMAL = 1,
    CAR_LANE = 2,
    PEDESTRIAN_LANE = 3,
};

enum LaneMarkingType {
    SOLID = 1,
    DASHED = 2,
    CENTER_SOLID = 3,
    CENTER_DASHED = 4,
};

enum LaneObjectType {
    LO_STOPLINE = 1,
    LO_SIGN = 2,
    LO_CROSSWALK = 3,
    LO_PARKING = 4,
};

enum WaypointType {
    ROAD = 1,
    JUNCTION = 2,
    STOPLINE = 3,
    PARKING = 4,
    CROSSWALK = 5,
    DEADEND = 6,
    OFFROAD = 7,
    ERROR = 8,
    OPPOSITE_ROAD = 9,
    INTERPOLATED = 10,
    JUNCTION_ROAD = 11,
    PLAN_START = 12,
    PLAN_GOAL = 13,
};

enum AADC_Maneuvers {
    NOT_SPECIFIED = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    DRIVE_STRAIGHT = 3,
    PARK_OUT_LEFT = 4,
    PARK_OUT_RIGHT = 5,
    PARALLEL_PARKING = 6,
    CROSS_PARKING = 7,
};

enum WaypointException {
    LANEPOINT_WITHOUT_JUNCTION = 1,
    MORE_THAN_3_OUTGOING_CONNECTIONS = 2,
    NO_OUTGOING_CONNECTIONS = 3,
    NO_TURN_LEFT_DETECTED = 4,
    NO_DRIVE_STRAIGHT_DETECTED = 5,
    NO_TURN_RIGHT_DETECTED = 6,
    MULTIPLE_TURN_LEFT_DETECTED = 7,
    MULTIPLE_DRIVE_STRAIGHT_DETECTED = 8,
    MULTIPLE_TURN_RIGHT_DETECTED = 9,
    POSE_POSITION_MISMATCH = 10,
    COULD_NOT_LOCALIZE = 11,
    NO_PARKING_LOTS = 12,
    NOT_AT_PARKING = 13,
    MULTIPLE_TURN_TYPES_DETECTED = 14,
    NO_OPPOSITE_LANE = 15,
};

enum PlannerException {
    START_LP_NOT_START_LANE = 1,
    GOAL_LP_NOT_GOAL_LANE = 2,
    GOAL_LP_BEHIND_START_LP = 3,
    LP_IS_NULLPTR = 4,
    PATH_INCONSISTENT = 5,
    START_LANE_IS_DEADEND = 6,
    NO_GOAL_FOUND = 7,
    NOT_IMPLEMENTED = 8,
};

class AADC_Maneuver;

std::string printWaypointException(WaypointException e);
std::string printManeuverId(AADC_Maneuver man);
class Uuid {
 private:
    std::string uuid;
 public:
    explicit Uuid(std::string id_in) : uuid(id_in) {}
    Uuid();
    ~Uuid();

    bool equals(Uuid id2);
    bool equals(const Uuid& id2) const;

    std::string getUuidValue() const;

    bool operator ==(Uuid uuid2) {
        return this->equals(uuid2);
    }
    bool operator ==(const Uuid uuid2) const {
        return this->equals(uuid2);
    }
};
class Point3D {
 private:
    double x = 0.;
    double y = 0.;
    double z = 0.;

 public:
    Point3D() : x(0.), y(0.), z(0.) {}
    Point3D(double x_in, double y_in) : x(x_in), y(y_in), z(0.) {}
    Point3D(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}

    std::vector<double> getCoords() const;
    double getX() const;
    double getY() const;
    double getZ() const;
    double computeDistance(mapobjects::Point3D);
    double computeHeadingToPoint(mapobjects::Point3D p);
};
class WaypointTypeHeader {
 public:
    WaypointType type = WaypointType::ROAD;
    double offset = 0;

 public:
    WaypointTypeHeader() {}
    WaypointTypeHeader(WaypointType type_in, double offset_in) \
        : type(type_in), offset(offset_in) {}
    explicit WaypointTypeHeader(WaypointType type_in) \
        : type(type_in) {}
};
class LaneInfo {
 public:
    bool no_overtaking = false;
    bool has_stoplines = false;
    bool has_crosswalks = false;
    bool has_parking = false;
    bool is_ground = true;
    bool is_ramp_up = false;
    bool is_ramp_down = false;
    bool is_merging = false;
    bool is_tunnel = false;

 public:
    LaneInfo() {}
    LaneInfo(bool overtake_in, bool stoplines_in, bool crosswalks_in, bool parking_in) \
        : no_overtaking(overtake_in), \
        has_stoplines(stoplines_in), has_crosswalks(crosswalks_in), has_parking(parking_in) {}
};
class LanePoint3D {
 public:
    Point3D p;
    int p_index;
    Uuid lane_uuid;
    WaypointTypeHeader lp_type = WaypointTypeHeader();
    bool isOppositeLane = false;
    LaneInfo lane_info = LaneInfo();
    bool isInterpolated = false;

 public:
    LanePoint3D() : p(Point3D()), p_index(-1),
        lane_uuid(Uuid("uninitialized")), lp_type(WaypointTypeHeader()) {}
    LanePoint3D(Point3D p_in, int p_index_in, Uuid id_in) : \
        p(p_in), p_index(p_index_in), lane_uuid(id_in), lp_type(WaypointTypeHeader()) {}
    LanePoint3D(Point3D p_in, int p_index_in, Uuid id_in, WaypointTypeHeader lp_type_in) : \
        p(p_in), p_index(p_index_in), lane_uuid(id_in), lp_type(lp_type_in) {}
    LanePoint3D(Point3D p_in, int p_index_in, Uuid id_in, WaypointType lp_type_type_in) : \
        p(p_in), p_index(p_index_in), lane_uuid(id_in), lp_type(
            WaypointTypeHeader(lp_type_type_in)) {}
};
class LaneNode {
 public:
    Uuid lane_uuid;
    Uuid parent_uuid;
    double length;
    std::vector<Uuid> outgoing_lanes;
    double g = 0;
    double h = 0;

 public:
    LaneNode(const Uuid& lane_uuid_in, const Uuid& parent_uuid_in,
        const double& g_in, const double& h_in, std::vector<Uuid> outgoing_lanes_in);
    ~LaneNode() {}

    double f() {
        return g + h;
    }
    double f() const {
        return g + h;
    }
    bool operator <(LaneNode node) {
        if (f() < node.f()) {
            return true;
        }
        return false;
    }
    bool operator >(LaneNode node) {
        if (f() > node.f()) {
            return true;
        }
        return false;
    }
    bool operator <(LaneNode node) const {
        if (f() < node.f()) {
            return true;
        }
        return false;
    }
    bool operator >(LaneNode node) const {
        if (f() > node.f()) {
            return true;
        }
        return false;
    }
    bool operator ==(const LaneNode node) {
        if (f() == node.f()) {
            return true;
        }
        return false;
    }
    bool operator ==(const LaneNode& node) const {
        if (f() == node.f()) {
            return true;
        }
        return false;
    }
    std::vector<Uuid> successors() {
        return outgoing_lanes;
    }
};
class LaneMarkingContainer {
 public:
    LaneMarkingContainer(std::vector<Uuid> left_lm, std::vector<Uuid> right_lm) : \
        left(left_lm), right(right_lm) {}

    ~LaneMarkingContainer();

 private:
    std::vector<Uuid> left;
    std::vector<Uuid> right;

 public:
    std::vector<Uuid> getLaneMarkingsLeft() const;
    std::vector<Uuid> getLaneMarkingsRight() const;
};
class MapObject {
 protected:
    Uuid id;
    explicit MapObject(Uuid id_in) : id(id_in) {}
    MapObject();

 public:
     * Must instanciate child objects */
    virtual ~MapObject() = 0;
};
class Pose {
 protected:
    double x;
    double y;
    double z;
    double t;

 public:
    Pose() : x(0), y(0), z(0), t(0) {}
    Pose(double x_in, double y_in, double z_in) : \
        x(x_in), y(y_in), z(z_in), t(0) {}
    Pose(double x_in, double y_in, double z_in, double t_in) : \
        x(x_in), y(y_in), z(z_in), t(t_in) {}
    explicit Pose(Point3D p);
    Pose(Point3D p, double t_in);
    ~Pose();

    std::vector<double> getXYZT();
    double getX() const;
    double getY() const;
    double getZ() const;
    double getT() const;
    mapobjects::Point3D getPoint3D();
    Pose getPose() const;
    Pose* convertFromADTF();
    Pose projectAlongHeading(double distance);

    void setPose(double x_in, double y_in, double z_in, double t_in);
};
class GlobalWaypoint {
 public:
    Pose pose;
    WaypointType type;
    LanePoint3D lp;
    bool isOppositeLane = false;

 public:
    GlobalWaypoint() = default;
    ~GlobalWaypoint() = default;

    explicit GlobalWaypoint(WaypointType type_in) \
        : pose(mapobjects::Pose()), type(type_in) {}
    GlobalWaypoint(double x_in, double y_in, double dir_in) \
        : pose(x_in, y_in, 0.0, dir_in), type(WaypointType::ROAD) {}
    GlobalWaypoint(double x_in, double y_in, double dir_in, WaypointType type_in) \
        : pose(x_in, y_in, 0.0, dir_in), type(type_in) {}
    GlobalWaypoint(double x_in, double y_in, double z_in, double dir_in) \
        : pose(x_in, y_in, z_in, dir_in), type(WaypointType::ROAD) {}
    GlobalWaypoint(double x_in, double y_in, double z_in, double dir_in, \
        WaypointType type_in) \
        : pose(x_in, y_in, z_in, dir_in), type(type_in) {}
    explicit GlobalWaypoint(LanePoint3D lp);
    GlobalWaypoint(LanePoint3D lp, WaypointType type_in);
    GlobalWaypoint(LanePoint3D lp, WaypointType type_in, Pose pose_in);
};
class AADC_Maneuver {
 public:
    AADC_Maneuvers man_id = AADC_Maneuvers::NOT_SPECIFIED;
    int extra = 0;

 public:
    AADC_Maneuver() {}
    explicit AADC_Maneuver(AADC_Maneuvers man_id_in): man_id(man_id_in) {}
    AADC_Maneuver(AADC_Maneuvers man_id_in, int extra_in): man_id(man_id_in), extra(extra_in) {}
};
class ManeuverRoute {
 public:
    GlobalWaypoint wp_start;
    GlobalWaypoint wp_goal;
    std::vector<AADC_Maneuver> maneuver_list;

 public:
    ManeuverRoute() = delete;
    ManeuverRoute(
        GlobalWaypoint wp_start_in,
        GlobalWaypoint wp_goal_in,
        std::vector<AADC_Maneuver> maneuver_list_in) :
            wp_start(wp_start_in), wp_goal(wp_goal_in), maneuver_list(maneuver_list_in) {}
};
class Pivot : public MapObject, public Pose {
 public:
    Pivot() : MapObject(), Pose() {}
    Pivot(Uuid id_in, Pose pose_in) : id(id_in), pose(pose_in) {}

    ~Pivot();

 private:
    Uuid id;
    Pose pose;

 public:
    mapobjects::Pose getPose() const;
};
class LaneGroup : public MapObject {
 public:
    LaneGroup(Uuid id_in, std::vector<Uuid> l_l, std::vector<Uuid> l_r) : \
        MapObject(id_in), lanes_left(l_l), lanes_right(l_r) {}

    ~LaneGroup();

 private:
    std::vector<Uuid> lanes_left;
    std::vector<Uuid> lanes_right;

 public:
    std::vector<Uuid> getLanesLeft() const;
    std::vector<Uuid> getLanesRight() const;
};
class Lane : public MapObject {
 public:
    Lane(Uuid id_in, LaneMarkingContainer lm, LaneDirection d, std::vector<Uuid> c_in,
        std::vector<Uuid> c_out, LaneType t, std::vector<Point3D> p, double w, \
        Point3D hp, bool v, int h) : \
        MapObject(id_in), lane_markings(lm), dir(d), connections_out(c_out), connections_in(c_in),
        type(t), points(p), width(w), handle_point(hp), visibility(v), height(h) {}

    ~Lane();

 private:
    LaneMarkingContainer lane_markings;
    LaneDirection dir;
    std::vector<Uuid> connections_out;
    std::vector<Uuid> connections_in;
    LaneType type;
    std::vector<Point3D> points;
    double width;
    Point3D handle_point;
    bool visibility;
    int height = 0;

 public:
    LaneType getLaneType() const;
    LaneDirection getLaneDirection() const;
    Point3D getHandlePoint() const;
    double getWidth() const;
    LaneMarkingContainer getLaneMarkingContainer() const;
    std::vector<Uuid> getConnectionsOut() const;
    std::vector<Uuid> getConnectionsIn() const;
    std::vector<Point3D> getPoints() const;
    bool getVisibility() const;
    int getHeight() const;
    double length();
};
class LaneMarking : public MapObject {
 public:
    LaneMarking(Uuid id_in, LaneMarkingType t, std::vector<Point3D> p, bool v) : \
        MapObject(id_in), type(t), points(p), visibility(v) {}

    ~LaneMarking();

 private:
    LaneMarkingType type;
    std::vector<Point3D> points;
    bool visibility;

 public:
    LaneMarkingType getLaneMarkingType() const;
    bool getVisibility() const;
    std::vector<Point3D> getPoints() const;
};
class LaneObject : public MapObject {
 public:
    LaneObject(Uuid id_in, LaneObjectType type_in, Uuid lg_id_in, \
        std::vector<Uuid> lane_ids_in, double offset_in) \
        : MapObject(id_in), type(type_in), lg_id(lg_id_in), \
        lane_ids(lane_ids_in), offset(offset_in) {}

    ~LaneObject();

 protected:
    LaneObjectType type;
    Uuid lg_id;
    std::vector<Uuid> lane_ids;
    double offset;

 public:
    LaneObjectType getLaneObjectType() const;
    Uuid getLaneGroupId() const;
    std::vector<Uuid> getLaneIds() const;
    double getOffset() const;
};
class Stopline : public LaneObject {
 public:
    Stopline(Uuid id_in, Uuid lg_id_in, \
        std::vector<Uuid> lane_ids_in, double offset_in) \
        : LaneObject(id_in, LaneObjectType::LO_STOPLINE, lg_id_in, lane_ids_in, offset_in) {}
    ~Stopline();
};
class Crosswalk : public LaneObject {
 public:
    Crosswalk(Uuid id_in, Uuid lg_id_in, \
        std::vector<Uuid> lane_ids_in, double offset_in) \
        : LaneObject(id_in, LaneObjectType::LO_CROSSWALK, lg_id_in, lane_ids_in, offset_in) {}
    ~Crosswalk();
};
class Parking : public LaneObject {
 public:
    Parking(Uuid id_in, Uuid lg_id_in, \
        std::vector<Uuid> lane_ids_in, double offset_in,
        int n_parking_lots_in, double parking_height_in, double parking_width_in, \
        double linewidth_in, std::vector<std::vector<Point3D>> outer_points_in) \
        : LaneObject(id_in, LaneObjectType::LO_PARKING, lg_id_in, lane_ids_in, offset_in), \
        n_parking_lots(n_parking_lots_in), parking_height(parking_height_in), \
        parking_width(parking_width_in), linewidth(linewidth_in), outer_points(outer_points_in) {}
    ~Parking();

 private:
    int n_parking_lots;
    double parking_height;
    double parking_width;
    double linewidth;
    std::vector<std::vector<Point3D>> outer_points;

 public:
    int getNumberOfParkingLots() const;
    double getWidth() const;
    double getHeight() const;
    double getLinewidth() const;
    std::vector<std::vector<Point3D>> getOuterPoints() const;
};
class Roadsign : public LaneObject {
 public:
    Roadsign(Uuid id_in, Uuid lg_id_in, \
        std::vector<Uuid> lane_ids_in, double offset_in, std::string sign_type_in, \
        mapobjects::Point3D position_in, double rotation_in) \
        : LaneObject(id_in, LaneObjectType::LO_SIGN, lg_id_in, lane_ids_in, offset_in), \
        sign_type(sign_type_in), position(position_in), rotation(rotation_in) {}
    ~Roadsign();

 private:
    std::string sign_type;
    Point3D position;
    double rotation;
};
class Car : public MapObject, public Pose {
 private:
 public:
    Car() : MapObject(), Pose() {}
    Car(double x_in, double y_in, double z_in, double t_in) : \
        MapObject(), Pose(x_in, y_in, z_in, t_in) {}
    Car(Uuid id_in, double x_in, double y_in, double z_in, double t_in) : \
        MapObject(id_in), Pose(x_in, y_in, z_in, t_in) {}
    ~Car();
};

}

#endif
