
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#ifndef MAP_MAPCORE_MAP_H_
#define MAP_MAPCORE_MAP_H_

#include <math.h>
#include <vector>
#include <array>
#include <numeric>
#include <cstdint>
#include <map>
#include <mutex>                                
#include <string>
#include <stdexcept>
#include <queue>
#include <functional>

#include <opencv2/opencv.hpp>
#include "../MapThrift/gen-cpp/map_data_structure_types.h"
#include "MapObjects.h"
#include "IGlobalMap.h"

using map_thrift::MapMessage;



static const int MAP_MAX_SIZE_X = 9000;                   
static const int MAP_MAX_SIZE_Y = 7000;                   
static const int DISCRETIZATION_DEFAULT = 1;              
static const int MAX_WINDOW_LENGTH = 500;                 
static const int CENTERLINE_THICKNESS = 2;                
static const double DELOCALIZATION_THRESHOLD = 200;       
static const int GLOBAL_X_OFFSET = 4500;
static const int GLOBAL_Y_OFFSET = 4700;

static const double JUNCTION_LEFT_ANGLE = -M_PI_2;
static const double JUNCTION_LEFT_MARGIN = M_PI_4;
static const double JUNCTION_STRAIGHT_ANGLE = 0;
static const double JUNCTION_STRAIGHT_MARGIN = M_PI_4;
static const double JUNCTION_RIGHT_ANGLE = M_PI_2;
static const double JUNCTION_RIGHT_MARGIN = M_PI_4;

static const double STOPLINE_MIN_SEPARATOR = 5;           



class Map: public IGlobalMap {
 public:
    enum GeometryException {
        COLLINEAR_ERROR = 1,
        PARALLEL_ERROR = 2,
        NOT_INTERSECTING_ERROR = 3,
    };

 private:
    std::mutex map_data_access;

    mapobjects::Pivot map_pivot;
    std::vector<mapobjects::LaneMarking> lanemarking_storage;
    std::vector<mapobjects::Lane> lane_storage;
    std::vector<mapobjects::LaneGroup> lanegroup_storage;
    std::vector<mapobjects::LaneObject*> laneobject_storage;

    std::unordered_map<std::string, int> lanemarking_uuids;
    std::unordered_map<std::string, int> lane_uuids;
    std::unordered_map<std::string, int> lanegroup_uuids;
    std::unordered_map<std::string, int> laneobject_uuids;

    
    std::unordered_map<std::string, std::vector<mapobjects::Uuid>> lane_stoplines_lookup;
    std::unordered_map<std::string, std::vector<mapobjects::Uuid>> lane_crosswalks_lookup;
    std::unordered_map<std::string, std::vector<mapobjects::Uuid>> lane_parking_lookup;
    std::unordered_map<std::string, std::vector<mapobjects::Uuid>> lane_roadsign_lookup;

    std::vector<map_thrift::PointViz> visualization_points;

    cv::Mat map_discretized;
    int discretization;
    int max_padding;
    
    int x_min, x_max, y_min, y_max;
    int map_x_max, map_y_max;

    
    bool map_changed;
    bool map_discretized_changed;

 private:
    
    bool lanepoint_debug = false;
    bool lanepoint_debug2 = false;
    bool lanepoint_search_debug = false;
    bool waypoint_debug = false;
    bool junction_debug = false;
    bool drawing_debug = false;
    bool data_debug = false;
    bool discretization_debug = false;
    bool file_debug = false;
    bool geometry_debug = false;
    bool planning_debug = true;
    bool planner_debug = false;
    bool plan_generation_debug = true;

 private:
    static Map* instance;
    Map();

    struct MapCosts {
        int c_lanes = 180;
        int c_lm_solid = 1;
        int c_lm_dashed = 1;
        int c_lm_center_solid = 1;
        int c_lm_center_dashed = 1;
    } map_costs;

    const double LANEWIDTH = 46.5;  
    const double CROSSWALK_WIDTH = 20;
    MapStatus internal_map_status;

 public:
    static Map* getInstance();
    
    Map(Map const&) = delete;
    Map& operator=(Map const) = delete;

 public:
    
    MapMessage& returnMapAsMapMessage();
    MapMessage& returnMapPartAsMapMessage();

    
    bool handleMapMessage(const MapMessage& message);
    bool addMapFromRemote(const MapMessage& message);

    bool updateMapFromMapMessage(const MapMessage&);
    bool updateMapPartFromMapMessage(const MapMessage&);  
    std::vector<int> getMapStats();
    void changeDiscretizationLevel(int new_discretization);

    
    
    void addMapObject(const map_thrift::LaneMarking& mo);
    void addMapObject(const map_thrift::Lane& mo);
    void addMapObject(const map_thrift::LaneGroup& mo);
    void addMapObject(const map_thrift::LaneObjectList& mo);
    
    void addMapObjectBatch(const std::vector<map_thrift::LaneMarking>& mo_list);
    void addMapObjectBatch(const std::vector<map_thrift::Lane>& mo_list);
    void addMapObjectBatch(const std::vector<map_thrift::LaneGroup>& mo_list);

    
    void addMapObjectBatch(const std::vector<mapobjects::LaneMarking>& mo_list);
    void addMapObjectBatch(const std::vector<mapobjects::Lane>& mo_list);
    void addMapObjectBatch(const std::vector<mapobjects::LaneGroup>& mo_list);
    

    
    map_thrift::LaneMarking createThriftLaneMarking(const mapobjects::LaneMarking& mo);
    map_thrift::Lane createThriftLane(const mapobjects::Lane& mo);
    map_thrift::LaneGroup createThriftLaneGroup(const mapobjects::LaneGroup& mo);
    map_thrift::Pivot createThriftPivot();

    
    std::vector<map_thrift::LaneMarking> createThriftLaneMarkingBatch(
        const std::vector<mapobjects::LaneMarking>& mo);
    std::vector<map_thrift::Lane> createThriftLaneBatch(
        const std::vector<mapobjects::Lane>& mo);
    std::vector<map_thrift::LaneGroup> createThriftLaneGroupBatch(
        const std::vector<mapobjects::LaneGroup>& mo);

    
    map_thrift::MapContainer createMapContainerFromMapParts(
        const std::vector<map_thrift::MapPart>& m_parts);
    map_thrift::MapContainer createMapContainerFromMapParts(
        const map_thrift::MapPart& m_part);
    map_thrift::MapPart createMapPartFromMapObjects(
        const std::vector<map_thrift::LaneMarking>& v_lm,
        const std::vector<map_thrift::Lane>& v_l,
        const std::vector<map_thrift::LaneGroup>& v_lg);

    
    map_thrift::MapMessage createMapMessageAdd(const map_thrift::MapContainer& m_container);
    map_thrift::MapMessage createMapMessageUpdateWhole(const map_thrift::MapContainer& m_container);
    map_thrift::MapMessage createMapMessageUpdatePart(const map_thrift::MapContainer& m_container);
    map_thrift::MapMessage createMapMessageDelete(const map_thrift::MapContainer& m_container);
    map_thrift::MapMessage createMapMessageAddAll();
    map_thrift::MapMessage createMapMessageDeleteAll();

    void triggerMapDiscretization();

    void loadMapFromFile(map_thrift::MapMessage map_object);

 public:
    
    
    bool deleteInternalMap();
    bool isMapChanged();
    void setMapChanged();
    void setMapUpdated();
    bool isDiscretizationChanged();
    MapStatus getMapStatus() override;

    void getCarStatus();

    
    std::vector<mapobjects::GlobalWaypoint> planWaypointRoute(
        const mapobjects::Pose& pose_start,
        const mapobjects::Pose& pose_goal,
        double waypoint_distance,
        bool ignoreStoplines,
        bool ignoreCrosswalks);
    mapobjects::ManeuverRoute planManeuverRoute(
        const mapobjects::Pose& pose_start,
        const mapobjects::Pose& pose_goal);
    double getRouteCosts(
        const mapobjects::Pose& pose_start,
        const mapobjects::Pose& pose_goal
    );

 private:
    mapobjects::LanePoint3D getSearchStartLanePoint3D(const mapobjects::Pose& pose_start);
    mapobjects::LanePoint3D getSearchGoalLanePoint3D(const mapobjects::Pose& pose_start);

    std::vector<mapobjects::Uuid> planRoute(
        const mapobjects::LanePoint3D& lp_start,
        const mapobjects::LanePoint3D& lp_goal,
        double* _route_costs);

    std::vector<mapobjects::Uuid> findRoute(
        const mapobjects::LanePoint3D& lp_start,
        const mapobjects::LanePoint3D& lp_goal,
        double* _costs);
    double h_function(const mapobjects::Uuid& lane_uuid, const mapobjects::LanePoint3D& lp_goal);
    mapobjects::LaneNode getNodeFromUuid(const mapobjects::Uuid& current_node_uuid,
        const std::vector<mapobjects::LaneNode>& nodes);

    bool goalReached(const mapobjects::LaneNode& node_1, const mapobjects::Uuid& goal_uuid);

    std::vector<mapobjects::GlobalWaypoint> getRouteWaypoints(
        const std::vector<mapobjects::Uuid>& route_uuids,
        mapobjects::LanePoint3D lp_start,
        mapobjects::LanePoint3D lp_goal,
        double waypoint_distance,
        bool ignoreStoplines,
        bool ignoreCrosswalks);
    std::vector<mapobjects::GlobalWaypoint> getLaneWaypoints(
        mapobjects::Uuid lane_uuid,
        mapobjects::LanePoint3D* lp_start,
        mapobjects::LanePoint3D* lp_goal,
        bool isStart,
        bool isGoal,
        double waypoint_distance,
        bool ignoreStoplines,
        bool ignoreCrosswalks);
    std::vector<mapobjects::AADC_Maneuver> getRouteManeuvers(
        const std::vector<mapobjects::Uuid>& route_uuids);

    mapobjects::AADC_Maneuver getJunctionLaneTurnType(const mapobjects::Uuid& lane_uuid);

 public:
    
    template<
        class T,
        class Container = std::vector<T>,
        class Compare = std::less<typename Container::value_type>> class map_priority_queue : \
        public std::priority_queue<T, Container, Compare> {

     public:
        typedef typename std::priority_queue<
            T,
            Container,
            Compare>::container_type::const_iterator const_iterator;

        const_iterator find(const T&val) const {
            auto first = this->c.cbegin();
            auto last = this->c.cend();
            while (first != last) {
                if (*first == val) return first;
                ++first;
            }
            return last;
        }

        const_iterator end() {
            return this->c.cend();
        }
    };

    
    std::vector<map_thrift::PointViz> getVisualizationPoints();
    void setVisualizationPoints(std::vector<mapobjects::Pose> points);
    void clearVisualizationPoints();

    mapobjects::GlobalWaypoint getNextGlobalWaypoint(
        const mapobjects::Pose& pose,
        double waypoint_distance,
        bool ignoreStoplines,
        bool ignoreCrosswalks,
        bool ignoreParking) override;
   
    mapobjects::GlobalWaypoint getNextGlobalWaypoint(
        const mapobjects::LanePoint3D& lp,
        double waypoint_distance,
        bool ignoreStoplines,
        bool ignoreCrosswalks,
        bool ignoreParking) override;
    mapobjects::GlobalWaypoint globalWaypointSearch(
        const mapobjects::LanePoint3D& lp_start,
        double waypoint_distance,
        bool ignoreStoplines,
        bool ignoreCrosswalks,
        bool ignoreParking);
    
    
    
    

    mapobjects::GlobalWaypoint getWaypointOnOppositeLane(
        const mapobjects::GlobalWaypoint& wp, bool invertHeading) override;

    mapobjects::GlobalWaypoint getWaypointTurnLeft(mapobjects::LanePoint3D lp) override;
    mapobjects::GlobalWaypoint getWaypointDriveStraight(mapobjects::LanePoint3D lp) override;
    mapobjects::GlobalWaypoint getWaypointTurnRight(mapobjects::LanePoint3D lp) override;
    std::vector<mapobjects::GlobalWaypoint> getWaypointsTurnLeft(mapobjects::LanePoint3D lp)
    override;
    std::vector<mapobjects::GlobalWaypoint> getWaypointsDriveStraight(mapobjects::LanePoint3D lp)
    override;
    std::vector<mapobjects::GlobalWaypoint> getWaypointsTurnRight(mapobjects::LanePoint3D lp)
    override;

    std::vector<mapobjects::GlobalWaypoint> getInnerJunctionWaypoints(
        mapobjects::Uuid junction_lane_uuid);

    
    
    

    mapobjects::GlobalWaypoint getWayPointNextParking(mapobjects::LanePoint3D lp);

    mapobjects::GlobalWaypoint getWaypointParkOut(
        const mapobjects::Pose& pose,
        bool park_out_left,
        double waypoint_distance);

    cv::Mat getDiscretizedMapArea(
        int window_width, int window_height, double car_x, double car_y, double car_angle);
    
    
    cv::Mat getShiftedMapWindow(
        int window_width, int window_height, \
        double offset_x, double offset_y,
        double car_x, double car_y, \
        double car_angle);


    cv::Mat getCenteredMapWindow(
        int window_width, int window_height, double car_x, double car_y);

    cv::Mat getShiftedMapWindow(
        int window_width, int window_height, \
        double car_x, double car_y, \
        double offset_x, double offset_y);

    bool writeMapToPng(int img_padding, std::string path);

    map_thrift::MapPart createMapPartFromAllMapObjects();

 public:
    
    void drawWaypoint(mapobjects::GlobalWaypoint waypoint);
    void drawPoint3D(mapobjects::Point3D p, int thickness, int radius);
    void drawLine(mapobjects::Point3D p1, mapobjects::Point3D p2, int thickness);
    void drawLanePoint(mapobjects::LanePoint3D lanepoint, int thickness, bool enable_direction);
    void drawLanePoint(mapobjects::LanePoint3D lanepoint, int thickness,
        bool enable_direction, bool first_lp);
    void drawCar(mapobjects::Pose car_pose);

 protected:
   
    mapobjects::Point3D createMapObjectPoint3D(double x, double y, double z);
    void addToStoplinesLookup(mapobjects::Stopline* _mo);
    void addToCrosswalksLookup(mapobjects::Crosswalk* _mo);
    void addToParkingLookup(mapobjects::Parking* _mo);
    void addToRoadsignLookup(mapobjects::Roadsign* _mo);

    
    std::vector<mapobjects::Point3D> convertToMapObjectPoints(
        const std::vector<map_thrift::Point3D>& points);
    std::vector<mapobjects::Uuid> convertToMapObjectsUuids(
        const std::vector<map_thrift::Uuid>& uuids);
    mapobjects::Uuid convertToMapObjectsUuid(
        const map_thrift::Uuid& uuid);

    mapobjects::LaneMarkingType convertToMapObjectLaneMarkingType(
        const map_thrift::LaneMarkingType::type& e);
    mapobjects::LaneType convertToMapObjectLaneType(
        const map_thrift::LaneType::type& e);
    mapobjects::LaneDirection convertToMapObjectLaneDirection(
        const map_thrift::LaneDirection::type& e);
    mapobjects::LaneMarkingContainer convertToMapObjectLaneMarkingContainer(
        const map_thrift::LaneMarkingContainer& c);

    
    std::vector<map_thrift::Point3D> convertToThriftPoints(
        const std::vector<mapobjects::Point3D>& points);
    map_thrift::Point3D convertToThriftPoint(
        const mapobjects::Point3D& point);
    std::vector<map_thrift::Uuid> convertToThriftUuids(
        const std::vector<mapobjects::Uuid>& uuids);
    map_thrift::Uuid convertToThriftUuid(
        const mapobjects::Uuid& uuid);

    map_thrift::LaneMarkingType::type convertToThriftLaneMarkingType(
        const mapobjects::LaneMarkingType& e);
    map_thrift::LaneType::type convertToThriftLaneType(
        const mapobjects::LaneType& e);
    map_thrift::LaneDirection::type convertToThriftLaneDirection(
        const mapobjects::LaneDirection& e);
    map_thrift::LaneMarkingContainer convertToThriftLaneMarkingContainer(
        const mapobjects::LaneMarkingContainer& c);

    int checkIfObjectExists(const mapobjects::MapObject& mo);

 private:
    
    
    
    
    void addMapObject(mapobjects::LaneMarking mo);
    void addMapObject(mapobjects::Lane mo);
    void addMapObject(mapobjects::LaneGroup mo);
    void addMapObject(mapobjects::LaneObject* _mo);

    
    void setPivot();
    void setPivot(const map_thrift::Pivot& thrift_pivot);

 private:
    void doSetMapChanged();
    void doSetMapUpdated();
    void doSetMapStatus(MapStatus status);
    void doSetPivot(mapobjects::Pivot pivot);
    void discretizeGlobalMap();

    
    cv::Point2i getMappedPoint(mapobjects::Point3D p, int discretization);
    bool writeMapWindowToPng(const cv::Mat& window, const std::string& path);
    void updateBoundingBox(mapobjects::Point3D);

    
    std::vector<int> getDiscretizedPosition(double x, double y);
    std::vector<int> getDiscretizedPosition(double x, double y, double z);
    int getDiscretizedX(double x);
    int getDiscretizedY(double y);
    int getDiscretizedZ(double z);
    int getPaddedX(double x);
    int getPaddedY(double y);

    double getAbsoluteX(double x);
    double getAbsoluteY(double y);

    std::vector<double> getWorldPosition(int x, int y);
    std::vector<double> getWorldPosition(int x, int y, int z);
    double getWorldX(int x);
    double getWorldY(int y);
    double getWorldZ(int z);
    double getRelativeX(double x);
    double getRelativeY(double y);
    mapobjects::Point3D getRelativePoint3D(const mapobjects::Point3D& p);

    
    void drawDiscretizedLines(std::vector<mapobjects::Point3D> points,
        int line_cost, int discretization, int thickness);
    void drawLaneObjects();
    void drawRoadsign(mapobjects::Roadsign* lo);
    void drawParking(mapobjects::Parking* lo);
    void drawCrosswalk(mapobjects::Crosswalk* lo);
    void drawStopline(mapobjects::Stopline* lo);


    
    mapobjects::Uuid getLaneGroupUuidFromLaneUuid(mapobjects::Uuid lane_uuid);
    mapobjects::Lane getLaneFromLaneUuid(const mapobjects::Uuid& lane_uuid) const;
    mapobjects::Uuid getOppositeLaneUuid(mapobjects::Uuid lane_uuid);

    double getClosestLaneHeading(mapobjects::LanePoint3D lp, mapobjects::Pose pose);
    bool isEqualCarAndLanePointDirection(mapobjects::Pose pose, mapobjects::LanePoint3D lp);
    double getLanePointDirection(mapobjects::LanePoint3D lp);
    double getAngularDifferenceFromAtan2(double heading_car, double heading_lane);
    double normalizeToAtan2(double heading);
    double getDirectionalAngularDifference(double heading_car, double heading_lane);

    mapobjects::LanePoint3D getNearestLanePoint(mapobjects::Pose pose);
    mapobjects::LanePoint3D getNearestOppositeLanePoint(mapobjects::LanePoint3D lp);
    mapobjects::LanePoint3D getNextLanePoint(mapobjects::LanePoint3D lp, double min_distance);
    std::vector<mapobjects::Point3D> getLanePoints(const mapobjects::Uuid& lane_uuid) const;

    bool checkSuccessorIsJunction(const mapobjects::Lane& lane);
    bool checkLaneIsDeadend(const mapobjects::Lane& lane);
    mapobjects::WaypointTypeHeader determineLanePointType(
        const mapobjects::LanePoint3D& lp, bool is_endpoint);
    mapobjects::WaypointTypeHeader determineLanePointType(
        const mapobjects::LanePoint3D& lp_prev,
        const mapobjects::LanePoint3D& lp,
        bool is_endpoint);

    mapobjects::LanePoint3D getFirstLanePoint(const mapobjects::Uuid& lane_uuid);
    mapobjects::LanePoint3D getPreviousLanePoint(const mapobjects::LanePoint3D& lp);

    mapobjects::Uuid getNextLaneUuid(const mapobjects::Uuid& lane_uuid);
    mapobjects::Uuid getNextLaneUuid(const mapobjects::Lane& lane);
    std::vector<mapobjects::Uuid> getNextLaneUuids(const mapobjects::Uuid& lane_uuid);
    std::vector<mapobjects::Uuid> getNextLaneUuids(const mapobjects::Lane& lane);

    std::vector<mapobjects::Uuid> getRealConnectionsOut(const mapobjects::Lane& lane);
    std::vector<mapobjects::Uuid> getRealConnectionsIn(const mapobjects::Lane& lane);
    std::vector<mapobjects::Uuid> getLaneStoplines(const mapobjects::Uuid& lane_uuid);
    std::vector<mapobjects::Uuid> getLaneCrosswalks(const mapobjects::Uuid& lane_uuid);
    double getCrosswalkOffset(mapobjects::Uuid crosswalk_uuid, mapobjects::Uuid lane_uuid);
    std::vector<mapobjects::Uuid> getLaneParkings(const mapobjects::Uuid& lane_uuid);
    std::vector<mapobjects::Uuid> getAllParkingLots();
    bool containsUuid(std::vector<mapobjects::Uuid> uuids, mapobjects::Uuid uuid);
    std::vector<mapobjects::Uuid> getAllLanesFromLanegroup(mapobjects::Uuid lg_uuid);
    mapobjects::LaneInfo getLaneInfo(mapobjects::Uuid lane_uuid);

    
    double getOffsetAlongLane(const mapobjects::LanePoint3D& lp);
    double getOffsetToPoint3D(mapobjects::LanePoint3D lp, mapobjects::Point3D p);
    mapobjects::Point3D getPoint3DFromLaneOffset(
        const mapobjects::LanePoint3D& lp, const double& offset);
    mapobjects::Point3D getPoint3DFromLanePointOffset(
        const mapobjects::LanePoint3D& lp, const double& offset);
    mapobjects::Point3D getLaneSegmentIntersection(
        mapobjects::Point3D lane1_p1,
        mapobjects::Point3D lane1_p2,
        mapobjects::Point3D lane2_p1,
        mapobjects::Point3D lane2_p2);
    double crossProduct2D(double x1, double y1, double x2, double y2);

    mapobjects::LanePoint3D getInterpolatedLanePoint(
        const mapobjects::LanePoint3D& lp_next,
        const double& offset);
    mapobjects::LanePoint3D projectPoint3DToLane(
        mapobjects::Point3D p, mapobjects::Uuid lane_uuid);
};

#endif  
