
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#include "WaypointTestFilter.h"
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_WAYPOINT_TEST_FILTER, "WaypointTestFilter",
    WaypointTestFilter,
    adtf::filter::timer_trigger(100000));


WaypointTestFilter::WaypointTestFilter() {
    RegisterPropertyVariable("initial x pos (cm)", m_propX);
    RegisterPropertyVariable("initial y pos (cm)", m_propY);
    RegisterPropertyVariable("initial z pos (cm)", m_propZ);
    RegisterPropertyVariable("initial heading (rad)", m_propT);
    RegisterPropertyVariable("waypoint distance (cm)", m_propGoalDistance);
    RegisterPropertyVariable("ignore stoplines", m_propIgnoreStoplines);
    RegisterPropertyVariable("ignore crosswalks", m_propIgnoreCrosswalks);
    RegisterPropertyVariable("ignore parking", m_propIgnoreParking);
    RegisterPropertyVariable("goal x pos (cm)", m_propX_goal);
    RegisterPropertyVariable("goal y pos (cm)", m_propY_goal);
    RegisterPropertyVariable("goal z pos (cm)", m_propZ_goal);
    RegisterPropertyVariable("goal heading (rad)", m_propT_goal);
    RegisterPropertyVariable("plan route", m_propPlanRoute);


    is_finished = false;
}

WaypointTestFilter::~WaypointTestFilter() {}

tResult WaypointTestFilter::Configure() {
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


void printLaneInfo(mapobjects::LaneInfo laneinfo);

tResult WaypointTestFilter::Process(tTimeStamp tmTimeOfTrigger) {
    if (!is_finished) {
        _map = Map::getInstance();
        MapStatus map_status = _map->getMapStatus();
        if (map_status == MapStatus::UPDATED_FROM_REMOTE) {
            mapobjects::Car car_object = mapobjects::Car();
            _map->drawCar(mapobjects::Pose(m_propX, m_propY, m_propZ, m_propT));

            if (m_propPlanRoute) {
                std::vector<mapobjects::GlobalWaypoint> route_waypoints = _map->planWaypointRoute(
                    mapobjects::Pose(m_propX, m_propY, m_propZ, m_propT),
                    mapobjects::Pose(m_propX_goal, m_propY_goal, m_propZ_goal, m_propT_goal),
                    m_propGoalDistance,
                    m_propIgnoreStoplines,
                    m_propIgnoreCrosswalks);
                LOG_INFO("route planning returned %d waypoints", route_waypoints.size());
                mapobjects::Uuid first_uuid = route_waypoints[0].lp.lane_uuid;
                int i = 0;
                for (mapobjects::GlobalWaypoint wp : route_waypoints) {
                        _map->drawWaypoint(wp);
                    i++;
                }
                _map->writeMapToPng(200, "/home/aadc/Desktop/planning.png");

                mapobjects::ManeuverRoute route_mans = _map->planManeuverRoute(
                    mapobjects::Pose(m_propX, m_propY, m_propZ, m_propT),
                    mapobjects::Pose(m_propX_goal, m_propY_goal, m_propZ_goal, m_propT_goal));
                LOG_INFO("Planned wp_start: x %f y %f h %f | type %d",
                    route_mans.wp_start.pose.getX(),
                    route_mans.wp_start.pose.getY(),
                    route_mans.wp_start.pose.getT(),
                    route_mans.wp_start.type);
                LOG_INFO("Planned wp_goal: x %f y %f h %f | type %d",
                    route_mans.wp_goal.pose.getX(),
                    route_mans.wp_goal.pose.getY(),
                    route_mans.wp_goal.pose.getT(),
                    route_mans.wp_goal.type);
                for (mapobjects::AADC_Maneuver man : route_mans.maneuver_list) {
                    LOG_INFO("maneuver id: %s", mapobjects::printManeuverId(man).c_str());
                }
                is_finished = true;
            } else {
                mapobjects::GlobalWaypoint waypoint = _map->getNextGlobalWaypoint(
                    mapobjects::Pose(m_propX, m_propY, m_propZ, m_propT),
                    m_propGoalDistance,
                    m_propIgnoreStoplines,
                    m_propIgnoreCrosswalks,
                    m_propIgnoreParking);
                _map->drawWaypoint(waypoint);
                _map->writeMapToPng(200, "/home/aadc/Desktop/test.png");
                printLaneInfo(waypoint.lp.lane_info);

                waypoint = _map->getNextGlobalWaypoint(
                    waypoint.lp,
                    m_propGoalDistance,
                    m_propIgnoreStoplines,
                    m_propIgnoreCrosswalks,
                    m_propIgnoreParking);
                _map->drawWaypoint(waypoint);
                printLaneInfo(waypoint.lp.lane_info);
                waypoint = _map->getNextGlobalWaypoint(
                    waypoint.lp,
                    m_propGoalDistance,
                    m_propIgnoreStoplines,
                    m_propIgnoreCrosswalks,
                    m_propIgnoreParking);
                _map->drawWaypoint(waypoint);
                _map->writeMapToPng(200, "/home/aadc/Desktop/test.png");
                printLaneInfo(waypoint.lp.lane_info);

                if (waypoint.isOppositeLane) {
                    LOG_WARNING("car is on the opposite lane!");
                }
                if (waypoint.type == mapobjects::WaypointType::JUNCTION) {
                    LOG_INFO("waypoint is junction");
                    std::vector<mapobjects::GlobalWaypoint> waypoints = \
                        _map->getWaypointsTurnRight(waypoint.lp);
                    for (mapobjects::GlobalWaypoint wp : waypoints) {
                        _map->drawWaypoint(wp);
                    }
                    _map->writeMapToPng(200, "/home/aadc/Desktop/test.png");
                    waypoint = _map->getNextGlobalWaypoint(
                        waypoint.lp,
                        m_propGoalDistance,
                        m_propIgnoreStoplines,
                        m_propIgnoreCrosswalks,
                        m_propIgnoreParking);
                    _map->drawWaypoint(waypoint);
                    _map->writeMapToPng(200, "/home/aadc/Desktop/test.png");
                    printLaneInfo(waypoint.lp.lane_info);

                } else if (waypoint.type == mapobjects::WaypointType::ROAD) {
                    LOG_INFO("waypoint is road");
                }
                is_finished = true;
            }
        }
    }
    RETURN_NOERROR;
}


void printLaneInfo(mapobjects::LaneInfo laneinfo) {
    std::string lane_info_string = "___LaneInfo___\n";
    if (laneinfo.is_ground) {
        lane_info_string += "level 0\n";
    }
    if (!laneinfo.is_ground) {
        lane_info_string += "level 1\n";
    }
    if (laneinfo.is_ramp_up) {
        lane_info_string += "ramp up\n";
    }
    if (laneinfo.is_ramp_down) {
        lane_info_string += "ramp down\n";
    }
    if (laneinfo.is_tunnel) {
        lane_info_string += "tunnel\n";
    }
    if (laneinfo.is_merging) {
        lane_info_string += "merging\n";
    }
    if (laneinfo.no_overtaking) {
        lane_info_string += "no overtaking\n";
    }
    if (laneinfo.has_crosswalks) {
        lane_info_string += "lane has crosswalks\n";
    }
    if (laneinfo.has_stoplines) {
        lane_info_string += "lane has stoplines\n";
    }
    if (laneinfo.has_parking) {
        lane_info_string += "lane has parking\n";
    }
    std::cout << lane_info_string << std::endl;
}
