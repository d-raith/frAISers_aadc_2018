
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#include "MapObjects.h"

using mapobjects::MapObject;
using mapobjects::Point3D;
using mapobjects::Uuid;
using mapobjects::LaneGroup;
using mapobjects::Lane;
using mapobjects::LaneMarking;
using mapobjects::LaneMarkingContainer;
using mapobjects::LaneObject;
using mapobjects::Stopline;
using mapobjects::Crosswalk;
using mapobjects::Parking;
using mapobjects::Roadsign;
using mapobjects::Car;
using mapobjects::Pose;
using mapobjects::Pivot;
using mapobjects::GlobalWaypoint;
using mapobjects::LaneNode;
using mapobjects::AADC_Maneuvers;

MapObject::MapObject()  {
    id = Uuid();
}
MapObject::~MapObject() {}

Lane::~Lane() {}
LaneGroup::~LaneGroup() {}
LaneMarking::~LaneMarking() {}
LaneMarkingContainer::~LaneMarkingContainer() {}
LaneObject::~LaneObject() {}
Stopline::~Stopline() {}
Crosswalk::~Crosswalk() {}
Parking::~Parking() {}
Roadsign::~Roadsign() {}
Pivot::~Pivot() {}

Car::~Car() {}
Pose::~Pose() {}
std::string mapobjects::printWaypointException(WaypointException e) {
    switch (e) {
        case LANEPOINT_WITHOUT_JUNCTION: {
            return "LANEPOINT_WITHOUT_JUNCTION";
        }
        break;
        case MORE_THAN_3_OUTGOING_CONNECTIONS: {
            return "MORE_THAN_3_OUTGOING_CONNECTIONS";
        }
        break;
        case NO_OUTGOING_CONNECTIONS: {
            return "NO_OUTGOING_CONNECTIONS";
        }
        break;
        case NO_TURN_LEFT_DETECTED: {
            return "NO_TURN_LEFT_DETECTED";
        }
        break;
        case NO_DRIVE_STRAIGHT_DETECTED: {
            return "NO_DRIVE_STRAIGHT_DETECTED";
        }
        break;
        case NO_TURN_RIGHT_DETECTED: {
            return "NO_TURN_RIGHT_DETECTED";
        }
        break;
        case MULTIPLE_TURN_LEFT_DETECTED: {
            return "MULTIPLE_TURN_LEFT_DETECTED";
        }
        break;
        case MULTIPLE_DRIVE_STRAIGHT_DETECTED: {
            return "MULTIPLE_DRIVE_STRAIGHT_DETECTED";
        }
        break;
        case MULTIPLE_TURN_RIGHT_DETECTED: {
            return "MULTIPLE_TURN_RIGHT_DETECTED";
        }
        break;
        case POSE_POSITION_MISMATCH : {
            return "POSE_POSITION_MISMATCH";
        }
        break;
        case COULD_NOT_LOCALIZE : {
            return "COULD_NOT_LOCALIZE";
        }
        break;
        case NO_PARKING_LOTS : {
            return "NO_PARKING_LOTS";
        }
        break;
        case NOT_AT_PARKING : {
            return "NOT_AT_PARKING";
        }
        break;
        case MULTIPLE_TURN_TYPES_DETECTED : {
            return "MULTIPLE_TURN_TYPES_DETECTED";
        }
        break;
        case NO_OPPOSITE_LANE : {
            return "NO_OPPOSITE_LANE";
        }
        break;
        default: {
                return "error code: " + std::to_string(e);
        }
    }
}
std::string mapobjects::printManeuverId(mapobjects::AADC_Maneuver man) {
    switch (man.man_id) {
        case NOT_SPECIFIED: {
            return "NOT_SPECIFIED";
        }
        break;
        case TURN_LEFT: {
            return "TURN_LEFT";
        }
        break;
        case TURN_RIGHT: {
            return "TURN_RIGHT";
        }
        break;
        case DRIVE_STRAIGHT: {
            return "DRIVE_STRAIGHT";
        }
        break;
        case PARK_OUT_LEFT: {
            return "PARK_OUT_LEFT";
        }
        break;
        case PARK_OUT_RIGHT: {
            return "PARK_OUT_RIGHT";
        }
        break;
        case PARALLEL_PARKING: {
            return "PARALLEL_PARKING";
        }
        break;
        case CROSS_PARKING: {
            return "CROSS_PARKING";
        }
        break;
        default: {
                return "maneuver id: " + std::to_string(man.man_id);
        }
    }
}

Uuid::Uuid() {
    boost::uuids::uuid uuid_boost = boost::uuids::random_generator()();
    uuid = boost::lexical_cast<std::string>(uuid_boost);
}
Uuid::~Uuid() {}
Uuid MapObject::getUuid() const {
    return id;
}
bool Uuid::equals(Uuid id2) {
    return (uuid == id2.getUuidValue()) ? true : false;
}
bool Uuid::equals(const Uuid& id2) const {
    return (uuid == id2.getUuidValue()) ? true : false;
}
std::string Uuid::getUuidValue() const {
    return uuid;
}
GlobalWaypoint::GlobalWaypoint(mapobjects::LanePoint3D lp) {
    mapobjects::Pose pose = mapobjects::Pose(lp.p);
    this->pose = pose;
    this->type = mapobjects::WaypointType::ROAD;
    this->lp = lp;
}
GlobalWaypoint::GlobalWaypoint(mapobjects::LanePoint3D lp, mapobjects::WaypointType type_in) {
    mapobjects::Pose pose = mapobjects::Pose(lp.p);
    this->pose = pose;
    this->type = type_in;
    this->lp = lp;
}
GlobalWaypoint::GlobalWaypoint(
    mapobjects::LanePoint3D lp, mapobjects::WaypointType type_in, mapobjects::Pose pose_in) {
    this->pose = pose_in;
    this->type = type_in;
    this->lp = lp;
}
LaneNode::LaneNode(const Uuid& lane_uuid_in, const Uuid& parent_uuid_in,
    const double& g_in, const double& h_in, std::vector<Uuid> outgoing_lanes_in) {
    this->lane_uuid = lane_uuid_in;
    this->parent_uuid = parent_uuid_in;
    this->g = g_in;
    this->h = h_in;
    this->outgoing_lanes = outgoing_lanes_in;
}
std::vector<double> Point3D::getCoords() const {
    std::vector<double> p_xyz = std::vector<double>(3);
    p_xyz[0] = x;
    p_xyz[1] = y;
    p_xyz[2] = z;
    return p_xyz;
}

double Point3D::getX() const {
    return x;
}
double Point3D::getY() const {
    return y;
}
double Point3D::getZ() const {
    return z;
}
double Point3D::computeDistance(mapobjects::Point3D p) {
    std::vector<double> p_coords = p.getCoords();
    double x2 = p_coords[0];
    double y2 = p_coords[1];
    double z2 = p_coords[2];

    double distance = 1e9;
    distance = sqrt(pow((x2 - x), 2) + pow((y2 - y), 2));
    distance = sqrt(pow(distance, 2) + pow((z2 - z), 2));
    return distance;
}
double Point3D::computeHeadingToPoint(mapobjects::Point3D p) {
    std::vector<double> p_coords = p.getCoords();
    double x2 = p_coords[0];
    double y2 = p_coords[1];
    double opposite = y2 - y;
    double adjacent = x2 - x;
    return fmod(atan2(opposite, adjacent), 2 * M_PI);
}
mapobjects::Pose Pivot::getPose() const {
    return pose;
}
std::vector<mapobjects::Uuid> LaneMarkingContainer::getLaneMarkingsLeft() const {
    return left;
}
std::vector<mapobjects::Uuid> LaneMarkingContainer::getLaneMarkingsRight() const {
    return right;
}
std::vector<mapobjects::Uuid> LaneGroup::getLanesLeft() const {
    return lanes_left;
}
std::vector<mapobjects::Uuid> LaneGroup::getLanesRight() const {
    return lanes_right;
}
mapobjects::Point3D Lane::getHandlePoint() const {
    return handle_point;
}
double Lane::getWidth() const {
    return width;
}
mapobjects::LaneMarkingContainer Lane::getLaneMarkingContainer() const {
    return lane_markings;
}
mapobjects::LaneDirection Lane::getLaneDirection() const {
    return dir;
}
mapobjects::LaneType Lane::getLaneType() const {
    return type;
}
std::vector<mapobjects::Uuid> Lane::getConnectionsOut() const {
    return connections_out;
}
std::vector<mapobjects::Uuid> Lane::getConnectionsIn() const {
    return connections_in;
}
std::vector<mapobjects::Point3D> Lane::getPoints() const {
    return points;
}
bool Lane::getVisibility() const {
    return visibility;
}
int Lane::getHeight() const {
    return height;
}
double Lane::length() {
    int n_points = points.size();
    double length = 0;
    for (int i = 0; i < n_points - 1; i++) {
        length += points[i].computeDistance(points[i+1]);
    }
    return length;
}
mapobjects::LaneMarkingType LaneMarking::getLaneMarkingType() const {
    return type;
}
bool LaneMarking::getVisibility() const {
    return visibility;
}
std::vector<mapobjects::Point3D> LaneMarking::getPoints() const {
    return points;
}
Pose::Pose(Point3D p) {
    std::vector<double> coords = p.getCoords();
    this->x = coords[0];
    this->y = coords[1];
    this->z = coords[2];
    this->t = 0.0;
}
Pose::Pose(Point3D p, double t_in) {
    std::vector<double> coords = p.getCoords();
    this->x = coords[0];
    this->y = coords[1];
    this->z = coords[2];
    this->t = t_in;
}
std::vector<double> Pose::getXYZT() {
    return std::vector<double>{this->x, this->y, this->z, getT()};
}
double Pose::getX() const {
    return this->x;
}
double Pose::getY() const {
    return this->y;
}
double Pose::getZ() const {
    return this->z;
}
double Pose::getT() const {
    return fmod(this->t, 2 * M_PI);
}
mapobjects::Point3D Pose::getPoint3D() {
    return mapobjects::Point3D(this->x, this->y, this->z);
}
Pose Pose::getPose() const {
    return Pose(this->x, this->y, this->z, this->t);
}
Pose* Pose::convertFromADTF() {
    this->y *= -1;
    this->t *= -1;
    return this;
}
void Pose::setPose(double x_in, double y_in, double z_in, double t_in) {
    this->x = x_in;
    this->y = y_in;
    this->z = z_in;
    this->t = t_in;
}

Pose Pose::projectAlongHeading(double distance) {
    this->x += distance * cos(this->t);
    this->y += distance * sin(this->t);
    return Pose(this->x, this->y, this->z, this->t);
}
mapobjects::LaneObjectType LaneObject::getLaneObjectType() const {
    return type;
}
mapobjects::Uuid LaneObject::getLaneGroupId() const {
    return lg_id;
}
std::vector<mapobjects::Uuid> LaneObject::getLaneIds() const {
    return lane_ids;
}
double LaneObject::getOffset() const {
    return offset;
}
int Parking::getNumberOfParkingLots() const {
    return n_parking_lots;
}
double Parking::getWidth() const {
    return parking_width;
}
double Parking::getHeight() const {
    return parking_height;
}
double Parking::getLinewidth() const {
    return linewidth;
}
std::vector<std::vector<mapobjects::Point3D>> Parking::getOuterPoints() const {
    return outer_points;
}