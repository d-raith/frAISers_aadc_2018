
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#include "Map.h"


Map* Map::instance = NULL;

Map::Map() {
    doSetMapStatus(MapStatus::UNINITIALIZED);
    lane_storage = std::vector<mapobjects::Lane>();
    lanegroup_storage = std::vector<mapobjects::LaneGroup>();
    lanemarking_storage = std::vector<mapobjects::LaneMarking>();

    lanemarking_uuids = std::unordered_map<std::string, int>();
    lane_uuids = std::unordered_map<std::string, int>();
    lanegroup_uuids = std::unordered_map<std::string, int>();

    lane_stoplines_lookup = std::unordered_map<std::string, std::vector<mapobjects::Uuid>>();
    lane_crosswalks_lookup = std::unordered_map<std::string, std::vector<mapobjects::Uuid>>();
    lane_parking_lookup = std::unordered_map<std::string, std::vector<mapobjects::Uuid>>();
    lane_roadsign_lookup = std::unordered_map<std::string, std::vector<mapobjects::Uuid>>();

    visualization_points = std::vector<map_thrift::PointViz>();

    map_discretized = cv::Mat();
    discretization = DISCRETIZATION_DEFAULT;
    max_padding = MAX_WINDOW_LENGTH;
    map_x_max = MAP_MAX_SIZE_X;
    map_y_max = MAP_MAX_SIZE_Y;
    x_min = map_x_max;
    x_max = 0;
    y_min = map_y_max;
    y_max = 0;

    map_changed = false;
    map_discretized_changed = true;
    doSetPivot(mapobjects::Pivot());
    discretizeGlobalMap();
}

Map* Map::getInstance() {
    if (instance == 0) {
        instance = new Map();
    }

    return instance;
}

bool Map::deleteInternalMap() {
    std::lock_guard<std::mutex> guard(map_data_access);

    std::vector<mapobjects::LaneMarking>().swap(lanemarking_storage);
    std::vector<mapobjects::Lane>().swap(lane_storage);
    std::vector<mapobjects::LaneGroup>().swap(lanegroup_storage);
    for (std::vector<mapobjects::LaneObject*>::iterator it = laneobject_storage.begin();
        it != laneobject_storage.end(); ++it) {
        delete *it;
    }
    std::vector<mapobjects::LaneObject*>().swap(laneobject_storage);

    lane_uuids.clear();
    lanegroup_uuids.clear();
    lanemarking_uuids.clear();
    laneobject_uuids.clear();
    lane_stoplines_lookup.clear();
    lane_crosswalks_lookup.clear();
    lane_parking_lookup.clear();
    lane_roadsign_lookup.clear();

    doSetMapChanged();
    doSetPivot(mapobjects::Pivot());

    if (lanemarking_storage.size() == 0 && lane_storage.size() == 0 \
        && lanegroup_storage.size() == 0 && laneobject_storage.size() == 0) {
            return true;
    }
    return false;
}
bool Map::handleMapMessage(const MapMessage& message) {
    map_thrift::MessageOp::type op = message.op;
    switch (op) {
        case map_thrift::MessageOp::type::ADD:
            if (data_debug) std::cout << "Map: Thrift MapMessage ADD received. adding." \
                << std::endl;
            addMapFromRemote(message);
            break;
        case map_thrift::MessageOp::type::UPDATE_WHOLE:
            if (data_debug) std::cout << "Map: Thrift MapMessage UPDATE_WHOLE received. updating." \
                << std::endl;
            addMapFromRemote(message);
            break;
        case map_thrift::MessageOp::type::UPDATE_PART:
            if (data_debug) std::cout << "Map: Thrift MapMessage UPDATE_PART received. updating." \
                << std::endl;
            addMapFromRemote(message);
            break;
        case map_thrift::MessageOp::type::DELETE:
            if (data_debug) std::cout << "Map: Thrift MapMessage DELETE received. deleting map." \
                << std::endl;
            deleteInternalMap();
            break;
        case map_thrift::MessageOp::type::POSE_UPDATE:
            std::cout << "Map: Thrift MapMessage POSE_UPDATE received. invalid use." << std::endl;
            break;
        case map_thrift::MessageOp::type::POSE_DELETE:
            std::cout << "Map: Thrift MapMessage POSE_DELETE received. invalid use." << std::endl;
            break;
    }
    return true;
}
bool Map::addMapFromRemote(const MapMessage& mm) {
    const map_thrift::MapContainer& map_container = mm.container;
    int n_parts = map_container.map_parts.size();
    if (data_debug) printf("Map: %d MapParts received", static_cast<int>(n_parts));
    for (int i = 0; i < n_parts; ++i) {
        const map_thrift::MapPart& map_part = map_container.map_parts[i];
        if (data_debug) printf("Map: > %d Lanes | %d LaneGroups | %d LaneMarkings to be added",
            static_cast<int>(map_part.lanes.size()),
            static_cast<int>(map_part.lane_groups.size()),
            static_cast<int>(map_part.lane_markings.size()));
        if (map_part.lanes.size() != 0) {
            if (data_debug) std::cout << "Map: >> adding Lanes to Map" << std::endl;
            addMapObjectBatch(map_part.lanes);
        }
        if (map_part.lane_groups.size() != 0) {
            if (data_debug) std::cout << "Map: >> adding LaneGroups to Map" << std::endl;
            addMapObjectBatch(map_part.lane_groups);
        }
        if (map_part.lane_markings.size() != 0) {
            if (data_debug) std::cout << "Map: >> adding LaneMarkings to Map" << std::endl;
            addMapObjectBatch(map_part.lane_markings);
        }
        if (data_debug) std::cout << "Map: >> adding LaneObjects to Map" << std::endl;
        addMapObject(map_part.lane_objects);
    }
    if (data_debug) std::cout << "MapFilter: Map from remote added!" << std::endl;
    std::vector<int> stats = getMapStats();
    if (data_debug) printf("MapStorage: %d Lanes | %d LaneGroups | %d LaneMarkings\n",
            stats[0], stats[1], stats[2]);
    setPivot(map_container.pivot);

    triggerMapDiscretization();
    doSetMapStatus(MapStatus::UPDATED_FROM_REMOTE);
    return true;
}
void Map::addMapObject(const map_thrift::LaneMarking& mo) {
    mapobjects::LaneMarking lm = mapobjects::LaneMarking(
        mapobjects::Uuid(mo.id.uuid),
        convertToMapObjectLaneMarkingType(mo.type),
        convertToMapObjectPoints(mo.points),
        mo.visibility);
    addMapObject(lm);
}
void Map::addMapObject(const map_thrift::Lane& mo) {
    mapobjects::Lane l = mapobjects::Lane(
        mapobjects::Uuid(mo.id.uuid),
        convertToMapObjectLaneMarkingContainer(mo.lane_markings),
        convertToMapObjectLaneDirection(mo.dir),
        convertToMapObjectsUuids(mo.incoming_connections),
        convertToMapObjectsUuids(mo.outgoing_connections),
        convertToMapObjectLaneType(mo.type),
        convertToMapObjectPoints(mo.points),
        mo.width,
        createMapObjectPoint3D(mo.handle_point.x, mo.handle_point.y, mo.handle_point.z),
        mo.visibility,
        mo.height);
    addMapObject(l);
}
void Map::addMapObject(const map_thrift::LaneGroup& mo) {
    mapobjects::LaneGroup lg = mapobjects::LaneGroup(
        mapobjects::Uuid(mo.id.uuid),
        convertToMapObjectsUuids(mo.lanes_left),
        convertToMapObjectsUuids(mo.lanes_right));
    addMapObject(lg);
}
void Map::addMapObject(const map_thrift::LaneObjectList& mo) {
    int n_stoplines = mo.stoplines.size();
    for (int i = 0; i < n_stoplines; i++) {
        mapobjects::Stopline* _lo_stopline = new mapobjects::Stopline(
            convertToMapObjectsUuid(mo.stoplines[i].lane_object.id),
            convertToMapObjectsUuid(mo.stoplines[i].lane_object.lg_id),
            convertToMapObjectsUuids(mo.stoplines[i].lane_object.l_ids),
            mo.stoplines[i].lane_object.offset);
        addMapObject(_lo_stopline);
    }
    int n_crosswalks = mo.crosswalks.size();
    for (int i = 0; i < n_crosswalks; i++) {
        mapobjects::Crosswalk* _lo_crosswalk = new mapobjects::Crosswalk(
            convertToMapObjectsUuid(mo.crosswalks[i].lane_object.id),
            convertToMapObjectsUuid(mo.crosswalks[i].lane_object.lg_id),
            convertToMapObjectsUuids(mo.crosswalks[i].lane_object.l_ids),
            mo.crosswalks[i].lane_object.offset);
        addMapObject(_lo_crosswalk);
    }
    int n_parking_lots = mo.parking_lots.size();
    for (int i = 0; i < n_parking_lots; i++) {
        std::vector<std::vector<mapobjects::Point3D>> outer_points_list \
            = std::vector<std::vector<mapobjects::Point3D>>();
        for (int j = 0; j < n_parking_lots; j++) {
            outer_points_list.push_back(convertToMapObjectPoints(
                mo.parking_lots[i].outer_points[j]));
        }
        mapobjects::Parking* _lo_parking = new mapobjects::Parking(
            convertToMapObjectsUuid(mo.parking_lots[i].lane_object.id),
            convertToMapObjectsUuid(mo.parking_lots[i].lane_object.lg_id),
            convertToMapObjectsUuids(mo.parking_lots[i].lane_object.l_ids),
            mo.parking_lots[i].lane_object.offset,
            mo.parking_lots[i].number_of_lots,
            mo.parking_lots[i].parking_height,
            mo.parking_lots[i].parking_width,
            mo.parking_lots[i].line_width,
            outer_points_list);
        addMapObject(_lo_parking);
    }
    int n_roadsigns = mo.roadsigns.size();
    for (int i = 0; i < n_roadsigns; i++) {
        mapobjects::Roadsign* _lo_roadsign = new mapobjects::Roadsign(
            convertToMapObjectsUuid(mo.roadsigns[i].lane_object.id),
            convertToMapObjectsUuid(mo.roadsigns[i].lane_object.lg_id),
            convertToMapObjectsUuids(mo.roadsigns[i].lane_object.l_ids),
            0,
            mo.roadsigns[i].sign_type,
            createMapObjectPoint3D(
                mo.roadsigns[i].position.x, mo.roadsigns[i].position.y, mo.roadsigns[i].position.z),
            mo.roadsigns[i].rotation);
        addMapObject(_lo_roadsign);
    }
}
void Map::addMapObjectBatch(const std::vector<map_thrift::LaneMarking>& mo_list) {
    int n = mo_list.size();
    for (int i = 0; i < n; ++i) {
        addMapObject(mo_list[i]);
    }
}
void Map::addMapObjectBatch(const std::vector<map_thrift::Lane>& mo_list) {
    int n = mo_list.size();
    for (int i = 0; i < n; ++i) {
        addMapObject(mo_list[i]);
    }
}
void Map::addMapObjectBatch(const std::vector<map_thrift::LaneGroup>& mo_list) {
    int n = mo_list.size();
    for (int i = 0; i < n; ++i) {
        addMapObject(mo_list[i]);
    }
}
int Map::checkIfObjectExists(const mapobjects::MapObject& mo) {
    std::unordered_map<std::string, int>::iterator it;
    std::string uuid = (mo.getUuid()).getUuidValue();
    int index = -1;
    it = lane_uuids.find(uuid);
    if (it != lane_uuids.end()) {
        if (index == -1) {
            index = it->second;
        } else {
            index = -2;
        }
    }
    it = lanemarking_uuids.find(uuid);
    if (it != lanemarking_uuids.end()) {
        if (index == -1) {
            index = it->second;
        } else {
            index = -2;
        }
    }
    it = lanegroup_uuids.find(uuid);
    if (it != lanegroup_uuids.end()) {
        if (index == -1) {
            index = it->second;
        } else {
            index = -2;
        }
    }
    it = laneobject_uuids.find(uuid);
    if (it != laneobject_uuids.end()) {
        if (index == -1) {
            index = it->second;
        } else {
            index = -2;
        }
    }
    return index;
}
void Map::addMapObject(mapobjects::LaneMarking mo) {
    std::lock_guard<std::mutex> guard(map_data_access);
    int mo_found = checkIfObjectExists(mo);
    if (mo_found == -1) {
        try {
            lanemarking_storage.push_back(mo);
            lanemarking_uuids.insert(
            {(mo.getUuid()).getUuidValue(), (static_cast<int>(lanemarking_storage.size()) - 1)});
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!\n" << std::endl << std::flush;
        }
    } else if (mo_found > -1) {
        try {
            lanemarking_storage[mo_found] = mo;
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!\n" << std::endl << std::flush;
        }
    }
    doSetMapChanged();
}
void Map::addMapObject(mapobjects::Lane mo) {
    std::lock_guard<std::mutex> guard(map_data_access);
    int mo_found = checkIfObjectExists(mo);
    if (mo_found == -1) {
        try {
            lane_storage.push_back(mo);
            lane_uuids.insert(
                {(mo.getUuid()).getUuidValue(), (static_cast<int>(lane_storage.size()) - 1)});
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!\n" << std::endl << std::flush;
        }
    } else if (mo_found > -1) {
        try {
            lane_storage[mo_found] = mo;
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!\n" << std::endl << std::flush;
        }
    }
    doSetMapChanged();
}
void Map::addMapObject(mapobjects::LaneGroup mo) {
    std::lock_guard<std::mutex> guard(map_data_access);

    int mo_found = checkIfObjectExists(mo);
    if (mo_found == -1) {
        try {
            lanegroup_storage.push_back(mo);
            lanegroup_uuids.insert(
                {(mo.getUuid()).getUuidValue(), (static_cast<int>(lanegroup_storage.size()) - 1)});
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!\n" << std::endl << std::flush;
        }
    } else if (mo_found > -1) {
        try {
            lanegroup_storage[mo_found] = mo;
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!\n" << std::endl << std::flush;
        }
    }
    doSetMapChanged();
}
void Map::addMapObject(mapobjects::LaneObject* _mo) {
    std::lock_guard<std::mutex> guard(map_data_access);

    int mo_found = checkIfObjectExists(*_mo);
    if (mo_found == -1) {
        try {
            laneobject_storage.push_back(_mo);
            laneobject_uuids.insert(
                {(_mo->getUuid()).getUuidValue(),
                (static_cast<int>(laneobject_storage.size()) - 1)});
            // add to the lookup tables
            if (_mo->getLaneObjectType() == mapobjects::LaneObjectType::LO_STOPLINE) {
                addToStoplinesLookup(dynamic_cast<mapobjects::Stopline*>(_mo));
            } else if (_mo->getLaneObjectType() == mapobjects::LaneObjectType::LO_CROSSWALK) {
                addToCrosswalksLookup(dynamic_cast<mapobjects::Crosswalk*>(_mo));
            } else if (_mo->getLaneObjectType() == mapobjects::LaneObjectType::LO_PARKING) {
                addToParkingLookup(dynamic_cast<mapobjects::Parking*>(_mo));
            } else if (_mo->getLaneObjectType() == mapobjects::LaneObjectType::LO_SIGN) {
                addToRoadsignLookup(dynamic_cast<mapobjects::Roadsign*>(_mo));
            }
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!\n" << std::endl << std::flush;
        }
    } else if (mo_found > -1) {
        try {
            laneobject_storage[mo_found] = _mo;
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!\n" << std::endl << std::flush;
        }
    }
    doSetMapChanged();
}
void Map::addMapObjectBatch(const std::vector<mapobjects::LaneMarking>& mo_list) {
    int n = mo_list.size();
    for (int i = 0; i < n; ++i) {
        addMapObject(mo_list[i]);
    }
}
void Map::addMapObjectBatch(const std::vector<mapobjects::Lane>& mo_list) {
    int n = mo_list.size();
    for (int i = 0; i < n; ++i) {
        addMapObject(mo_list[i]);
    }
}
void Map::addMapObjectBatch(const std::vector<mapobjects::LaneGroup>& mo_list) {
    int n = mo_list.size();
    for (int i = 0; i < n; ++i) {
        addMapObject(mo_list[i]);
    }
}
void Map::setPivot() {
    std::lock_guard<std::mutex> guard(map_data_access);
    doSetPivot(mapobjects::Pivot());
}
void Map::setPivot(const map_thrift::Pivot& thrift_pivot) {
    if (thrift_pivot.is_valid) {
        if (data_debug) std::cout << "Map: detected valid pivot" << std::endl;
        mapobjects::Pivot pivot = mapobjects::Pivot(
            convertToMapObjectsUuid(thrift_pivot.id),
            mapobjects::Pose(createMapObjectPoint3D(
                thrift_pivot.pose.position.x, thrift_pivot.pose.position.y,
                thrift_pivot.pose.position.z),
                thrift_pivot.pose.orientation));
        std::lock_guard<std::mutex> guard(map_data_access);
        doSetPivot(pivot);
    } else {
        std::cout << "Map: detected invalid pivot. ignoring" << std::endl;
    }
    return;
}
void Map::doSetPivot(mapobjects::Pivot pivot) {
    if (data_debug) std::cout << "Map: >> updating map pivot" << std::endl;
    map_pivot = pivot;
}
void Map::addToStoplinesLookup(mapobjects::Stopline* _mo) {
    std::vector<mapobjects::Uuid> stopline_lanes = _mo->getLaneIds();
    int n_lanes_to_process = stopline_lanes.size();
    for (int i = 0; i < n_lanes_to_process; i++) {
        std::unordered_map<std::string, std::vector<mapobjects::Uuid>>::iterator it = \
            lane_stoplines_lookup.find(stopline_lanes[i].getUuidValue());
        if (it == lane_stoplines_lookup.end()) {
            lane_stoplines_lookup.insert(
                {stopline_lanes[i].getUuidValue(), std::vector<mapobjects::Uuid>(
                    1, _mo->getUuid())});
        } else {
            std::vector<mapobjects::Uuid> stopline_uuids = it->second;
            stopline_uuids.push_back(_mo->getUuid());
            it->second = stopline_uuids;
        }
    }
}
void Map::addToCrosswalksLookup(mapobjects::Crosswalk* _mo) {
    std::vector<mapobjects::Uuid> crosswalk_lanes = getAllLanesFromLanegroup(_mo->getLaneGroupId());
    int n_lanes_to_process = crosswalk_lanes.size();
    for (int i = 0; i < n_lanes_to_process; i++) {
        std::unordered_map<std::string, std::vector<mapobjects::Uuid>>::iterator it = \
            lane_crosswalks_lookup.find(crosswalk_lanes[i].getUuidValue());
        if (it == lane_stoplines_lookup.end()) {
            lane_crosswalks_lookup.insert(
                {crosswalk_lanes[i].getUuidValue(), std::vector<mapobjects::Uuid>(
                    1, _mo->getUuid())});
        } else {
            std::vector<mapobjects::Uuid> crosswalk_uuids = it->second;
            crosswalk_uuids.push_back(_mo->getUuid());
            it->second = crosswalk_uuids;
        }
    }
}
void Map::addToParkingLookup(mapobjects::Parking* _mo) {
    std::vector<mapobjects::Uuid> parking_lanes = _mo->getLaneIds();
    int n_lanes_to_process = parking_lanes.size();
    for (int i = 0; i < n_lanes_to_process; i++) {
        std::unordered_map<std::string, std::vector<mapobjects::Uuid>>::iterator it = \
            lane_parking_lookup.find(parking_lanes[i].getUuidValue());
        if (it == lane_parking_lookup.end()) {
            lane_parking_lookup.insert(
                {parking_lanes[i].getUuidValue(), std::vector<mapobjects::Uuid>(
                    1, _mo->getUuid())});
        } else {
            std::vector<mapobjects::Uuid> parking_uuids = it->second;
            parking_uuids.push_back(_mo->getUuid());
            it->second = parking_uuids;
            std::cout << "updating parking to lookup" << std::endl;
        }
    }
}
void Map::addToRoadsignLookup(mapobjects::Roadsign* _mo) {
    std::vector<mapobjects::Uuid> stopline_lanes = _mo->getLaneIds();
    int n_lanes_to_process = stopline_lanes.size();
    for (int i = 0; i < n_lanes_to_process; i++) {
        std::unordered_map<std::string, std::vector<mapobjects::Uuid>>::iterator it = \
            lane_stoplines_lookup.find(stopline_lanes[i].getUuidValue());
        if (it == lane_stoplines_lookup.end()) {
            lane_stoplines_lookup.insert(
                {stopline_lanes[i].getUuidValue(), std::vector<mapobjects::Uuid>(
                    1, _mo->getUuid())});
        } else {
            std::vector<mapobjects::Uuid> stopline_uuids = it->second;
            stopline_uuids.push_back(_mo->getUuid());
            it->second = stopline_uuids;
        }
    }
}
map_thrift::LaneMarking Map::createThriftLaneMarking(const mapobjects::LaneMarking& mo) {
    map_thrift::LaneMarking to = map_thrift::LaneMarking();
    to.type = convertToThriftLaneMarkingType(mo.getLaneMarkingType());
    to.points = convertToThriftPoints(mo.getPoints());
    to.id = convertToThriftUuid(mo.getUuid());
    to.visibility = mo.getVisibility();
    return to;
}
map_thrift::Lane Map::createThriftLane(const mapobjects::Lane& mo) {
    map_thrift::Lane to = map_thrift::Lane();
    to.lane_markings = convertToThriftLaneMarkingContainer(mo.getLaneMarkingContainer());
    to.dir = convertToThriftLaneDirection(mo.getLaneDirection());
    to.outgoing_connections = convertToThriftUuids(mo.getConnectionsOut());
    to.incoming_connections = convertToThriftUuids(mo.getConnectionsIn());
    to.type = convertToThriftLaneType(mo.getLaneType());
    to.points = convertToThriftPoints(mo.getPoints());
    to.width = mo.getWidth();
    to.id = convertToThriftUuid(mo.getUuid());
    to.handle_point = convertToThriftPoint(mo.getHandlePoint());
    to.visibility = mo.getVisibility();
    return to;
}
map_thrift::LaneGroup Map::createThriftLaneGroup(const mapobjects::LaneGroup& mo) {
    map_thrift::LaneGroup to = map_thrift::LaneGroup();
    to.id = convertToThriftUuid(mo.getUuid());
    to.lanes_left = convertToThriftUuids(mo.getLanesLeft());
    to.lanes_right = convertToThriftUuids(mo.getLanesRight());
    return to;
}
std::vector<map_thrift::LaneMarking> Map::createThriftLaneMarkingBatch(
    const std::vector<mapobjects::LaneMarking>& mo_list) {
    int n = mo_list.size();
    std::vector<map_thrift::LaneMarking> to_list = std::vector<map_thrift::LaneMarking>(n);
    for (int i = 0; i < n; ++i) {
        to_list[i] = createThriftLaneMarking(mo_list[i]);
    }
    return to_list;
}
std::vector<map_thrift::Lane> Map::createThriftLaneBatch(
    const std::vector<mapobjects::Lane>& mo_list) {
    int n = mo_list.size();
    std::vector<map_thrift::Lane> to_list = std::vector<map_thrift::Lane>(n);
    for (int i = 0; i < n; ++i) {
        to_list[i] = createThriftLane(mo_list[i]);
    }
    return to_list;
}
std::vector<map_thrift::LaneGroup> Map::createThriftLaneGroupBatch(
    const std::vector<mapobjects::LaneGroup>& mo_list) {
    int n = mo_list.size();
    std::vector<map_thrift::LaneGroup> to_list = std::vector<map_thrift::LaneGroup>(n);
    for (int i = 0; i < n; ++i) {
        to_list[i] = createThriftLaneGroup(mo_list[i]);
    }
    return to_list;
}
map_thrift::Pivot Map::createThriftPivot() {
    map_thrift::Pivot t_pivot = map_thrift::Pivot();
    t_pivot.id = convertToThriftUuid(map_pivot.getUuid());
    map_thrift::Pose t_pivot_pose = map_thrift::Pose();
    t_pivot_pose.position = convertToThriftPoint(mapobjects::Point3D(
        map_pivot.getPose().getX(),
        map_pivot.getPose().getY(),
        map_pivot.getPose().getZ()));
    t_pivot_pose.orientation = map_pivot.getPose().getT();
    t_pivot.pose = t_pivot_pose;
    t_pivot.is_valid = true;
    return t_pivot;
}
std::vector<map_thrift::PointViz> Map::getVisualizationPoints() {
    std::lock_guard<std::mutex> guard(map_data_access);

    return visualization_points;
}
void Map::setVisualizationPoints(std::vector<mapobjects::Pose> points) {
    std::lock_guard<std::mutex> guard(map_data_access);
    int n_points = points.size();
    visualization_points = std::vector<map_thrift::PointViz>(n_points);
    for (int i = 0; i < n_points; i++) {
        map_thrift::PointViz p_viz = map_thrift::PointViz();
        p_viz.point = convertToThriftPoint(points[i].getPoint3D());
        p_viz.radius = static_cast<double>(points[i].getT()) * 10;
        visualization_points[i] = p_viz;
    }
}
void Map::clearVisualizationPoints() {
    std::lock_guard<std::mutex> guard(map_data_access);

    std::vector<map_thrift::PointViz>().swap(visualization_points);

std::vector<mapobjects::Point3D> Map::convertToMapObjectPoints(
    const std::vector<map_thrift::Point3D>& points) {
    int n = points.size();
    std::vector<mapobjects::Point3D> r = std::vector<mapobjects::Point3D>();
    r.reserve(n);
    for (int i = 0; i < n; ++i) {
        r.push_back(createMapObjectPoint3D(points[i].x, points[i].y, points[i].z));
    }
    return r;
}
std::vector<mapobjects::Uuid> Map::convertToMapObjectsUuids(
    const std::vector<map_thrift::Uuid>& uuids) {
    int n = uuids.size();
    std::vector<mapobjects::Uuid> r = std::vector<mapobjects::Uuid>();
    r.reserve(n);
    for (int i = 0; i < n; ++i) {
        r.push_back(mapobjects::Uuid(uuids[i].uuid));
    }
    return r;
}

mapobjects::Uuid Map::convertToMapObjectsUuid(
    const map_thrift::Uuid& uuid) {
    mapobjects::Uuid id = mapobjects::Uuid(uuid.uuid);
    return id;
}
mapobjects::LaneMarkingType Map::convertToMapObjectLaneMarkingType(
    const map_thrift::LaneMarkingType::type& e) {
    switch (e) {
        case map_thrift::LaneMarkingType::type::SOLID:
            return mapobjects::LaneMarkingType::SOLID;
        case map_thrift::LaneMarkingType::type::DASHED:
            return mapobjects::LaneMarkingType::DASHED;
        case map_thrift::LaneMarkingType::type::CENTER_SOLID:
            return mapobjects::LaneMarkingType::CENTER_SOLID;
        case map_thrift::LaneMarkingType::type::CENTER_DASHED:
            return mapobjects::LaneMarkingType::CENTER_DASHED;
        default:
            return mapobjects::LaneMarkingType::SOLID;
    }
}
mapobjects::LaneType Map::convertToMapObjectLaneType(
    const map_thrift::LaneType::type& e) {
    switch (e) {
        case map_thrift::LaneType::NORMAL:
            return mapobjects::LaneType::NORMAL;
        case map_thrift::LaneType::CAR_LANE:
            return mapobjects::LaneType::CAR_LANE;
        case map_thrift::LaneType::PEDESTRIAN_LANE:
            return mapobjects::LaneType::PEDESTRIAN_LANE;
        default:
            return mapobjects::LaneType::NORMAL;
    }
}
mapobjects::LaneDirection Map::convertToMapObjectLaneDirection(
    const map_thrift::LaneDirection::type& e) {
    switch (e) {
        case map_thrift::LaneDirection::RIGHT:
            return mapobjects::LaneDirection::RIGHT;
        case map_thrift::LaneDirection::LEFT:
            return mapobjects::LaneDirection::LEFT;
        default:
            return mapobjects::LaneDirection::RIGHT;
    }
}
mapobjects::LaneMarkingContainer Map::convertToMapObjectLaneMarkingContainer(
    const map_thrift::LaneMarkingContainer& c) {
    mapobjects::LaneMarkingContainer lmc = mapobjects::LaneMarkingContainer(
        convertToMapObjectsUuids(c.left),
        convertToMapObjectsUuids(c.right));
    return lmc;
}
std::vector<map_thrift::Point3D> Map::convertToThriftPoints(
    const std::vector<mapobjects::Point3D>& points) {
    int n = points.size();
    std::vector<map_thrift::Point3D> r = std::vector<map_thrift::Point3D>();
    r.reserve(n);
    for (int i = 0; i < n; ++i) {
        map_thrift::Point3D p = convertToThriftPoint(points[i]);
        r.push_back(p);
    }
    return r;
}
map_thrift::Point3D Map::convertToThriftPoint(
    const mapobjects::Point3D& point) {
    map_thrift::Point3D r = map_thrift::Point3D();
    std::vector<double> p_xyz = point.getCoords();
    r.x = p_xyz[0];
    r.y = p_xyz[1];
    r.z = p_xyz[2];
    return r;
}
std::vector<map_thrift::Uuid> Map::convertToThriftUuids(
    const std::vector<mapobjects::Uuid>& uuids) {
    int n = uuids.size();
    std::vector<map_thrift::Uuid> r = std::vector<map_thrift::Uuid>();
    r.reserve(n);
    for (int i = 0; i < n; ++i) {
        map_thrift::Uuid id = map_thrift::Uuid();
        id.uuid = uuids[i].getUuidValue();
        r.push_back(id);
    }
    return r;
}
map_thrift::Uuid Map::convertToThriftUuid(
    const mapobjects::Uuid& uuid) {
    map_thrift::Uuid id = map_thrift::Uuid();
    id.uuid = uuid.getUuidValue();
    return id;
}

map_thrift::LaneMarkingType::type Map::convertToThriftLaneMarkingType(
    const mapobjects::LaneMarkingType& e) {
    switch (e) {
        case mapobjects::LaneMarkingType::SOLID:
            return map_thrift::LaneMarkingType::type::SOLID;
        case mapobjects::LaneMarkingType::DASHED:
            return map_thrift::LaneMarkingType::type::DASHED;
        case mapobjects::LaneMarkingType::CENTER_SOLID:
            return map_thrift::LaneMarkingType::type::CENTER_SOLID;
        case mapobjects::LaneMarkingType::CENTER_DASHED:
            return map_thrift::LaneMarkingType::type::CENTER_DASHED;
        default:
            return map_thrift::LaneMarkingType::type::SOLID;
    }
}
map_thrift::LaneType::type Map::convertToThriftLaneType(
    const mapobjects::LaneType& e) {
    switch (e) {
        case mapobjects::LaneType::NORMAL:
            return map_thrift::LaneType::type::NORMAL;
        case mapobjects::LaneType::CAR_LANE:
            return map_thrift::LaneType::type::CAR_LANE;
        case mapobjects::LaneType::PEDESTRIAN_LANE:
            return map_thrift::LaneType::type::PEDESTRIAN_LANE;
        default:
            return map_thrift::LaneType::type::NORMAL;
    }
}
map_thrift::LaneDirection::type Map::convertToThriftLaneDirection(
    const mapobjects::LaneDirection& e) {
    switch (e) {
        case mapobjects::LaneDirection::RIGHT:
            return map_thrift::LaneDirection::type::RIGHT;
        case mapobjects::LaneDirection::LEFT:
            return map_thrift::LaneDirection::type::LEFT;
        default:
            return map_thrift::LaneDirection::type::RIGHT;
    }
}
map_thrift::LaneMarkingContainer Map::convertToThriftLaneMarkingContainer(
    const mapobjects::LaneMarkingContainer& lmc) {
    map_thrift::LaneMarkingContainer to = map_thrift::LaneMarkingContainer();
    to.left = convertToThriftUuids(lmc.getLaneMarkingsLeft());
    to.right = convertToThriftUuids(lmc.getLaneMarkingsRight());
    return to;
}
mapobjects::Point3D Map::createMapObjectPoint3D(double x, double y, double z) {
    mapobjects::Point3D p = mapobjects::Point3D(x, y, z);
    updateBoundingBox(p);
    return p;
}

std::vector<int> Map::getMapStats() {
    std::vector<int> stats = std::vector<int>();
    stats.push_back(lane_storage.size());
    stats.push_back(lanegroup_storage.size());
    stats.push_back(lanemarking_storage.size());
    return stats;
}
map_thrift::MapPart Map::createMapPartFromAllMapObjects() {
    std::lock_guard<std::mutex> guard(map_data_access);

    map_thrift::MapPart m_part = createMapPartFromMapObjects(
        createThriftLaneMarkingBatch(lanemarking_storage),
        createThriftLaneBatch(lane_storage),
        createThriftLaneGroupBatch(lanegroup_storage));
    return m_part;
}
map_thrift::MapContainer Map::createMapContainerFromMapParts(
    const std::vector<map_thrift::MapPart>& m_parts) {
    map_thrift::MapContainer m_container = map_thrift::MapContainer();
    m_container.map_parts = m_parts;
    m_container.pivot = createThriftPivot();
    return m_container;
}
map_thrift::MapContainer Map::createMapContainerFromMapParts(
    const map_thrift::MapPart& m_part) {
    map_thrift::MapContainer m_container = map_thrift::MapContainer();
    m_container.map_parts = std::vector<map_thrift::MapPart>{m_part};
    m_container.pivot = createThriftPivot();
    return m_container;
}
map_thrift::MapPart Map::createMapPartFromMapObjects(
    const std::vector<map_thrift::LaneMarking>& v_lm,
    const std::vector<map_thrift::Lane>& v_l,
    const std::vector<map_thrift::LaneGroup>& v_lg) {
    map_thrift::LaneObjectList lane_object_list = map_thrift::LaneObjectList();
    std::cout << "Map: currently no lane_objects are added to the MapMessage" << std::endl;
    map_thrift::MapPart m_part = map_thrift::MapPart();
    m_part.lanes = v_l;
    m_part.lane_groups = v_lg;
    m_part.lane_markings = v_lm;
    m_part.lane_objects = lane_object_list;
    return m_part;
}
map_thrift::MapMessage Map::createMapMessageAdd(const map_thrift::MapContainer& m_container) {
    map_thrift::MapMessage mm = map_thrift::MapMessage();
    mm.op = map_thrift::MessageOp::type::ADD;
    mm.container = m_container;
    return mm;
}
map_thrift::MapMessage Map::createMapMessageUpdateWhole(
    const map_thrift::MapContainer& m_container) {
    map_thrift::MapMessage mm = map_thrift::MapMessage();
    mm.op = map_thrift::MessageOp::type::UPDATE_WHOLE;
    mm.container = m_container;
    return mm;
}
map_thrift::MapMessage Map::createMapMessageUpdatePart(
    const map_thrift::MapContainer& m_container) {
    map_thrift::MapMessage mm = map_thrift::MapMessage();
    mm.op = map_thrift::MessageOp::type::UPDATE_PART;
    mm.container = m_container;
    return mm;
}
map_thrift::MapMessage Map::createMapMessageDelete(const map_thrift::MapContainer& m_container) {
    map_thrift::MapMessage mm = map_thrift::MapMessage();
    mm.op = map_thrift::MessageOp::type::DELETE;
    mm.container = m_container;
    return mm;
}
map_thrift::MapMessage Map::createMapMessageDeleteAll() {
    map_thrift::MapMessage mm = map_thrift::MapMessage();
    mm.op = map_thrift::MessageOp::type::DELETE;
    mm.container = map_thrift::MapContainer();
    return mm;
}

map_thrift::MapMessage Map::createMapMessageAddAll() {
    return createMapMessageAdd(createMapContainerFromMapParts(createMapPartFromAllMapObjects()));
}
void Map::loadMapFromFile(map_thrift::MapMessage map_object) {
    addMapFromRemote(map_object);
    doSetMapStatus(MapStatus::LOADED_FROM_FILE);
}
void Map::discretizeGlobalMap() {
    int padding = max_padding * 2;
    int thickness = CENTERLINE_THICKNESS;
    int n_elements_lanes = lane_storage.size();
    map_discretized = cv::Mat::zeros(
        cv::Size(((map_y_max + padding) * discretization),
            ((map_x_max + padding) * discretization)),
        CV_8UC1);

    if (discretization_debug) std::cout << "\t\t*** [Discretized map] ***\n\t\tSize: " \
        << map_discretized.size().width << " | " << map_discretized.size().height \
        << std::endl << std::flush;
    for (int i = 0; i < n_elements_lanes; i++) {
        drawDiscretizedLines(lane_storage[i].getPoints(), map_costs.c_lanes,
            discretization, thickness);
    }

    map_discretized_changed = false;
}

void Map::drawLaneObjects() {
    int n_elements_laneobjects = laneobject_storage.size();

    for (int i = 0; i < n_elements_laneobjects; i++) {
        mapobjects::LaneObjectType lo_type = laneobject_storage[i]->getLaneObjectType();
        switch (lo_type) {
            case mapobjects::LaneObjectType::LO_SIGN:
                break;
            case mapobjects::LaneObjectType::LO_PARKING:
                break;
            case mapobjects::LaneObjectType::LO_CROSSWALK:
                drawCrosswalk(dynamic_cast<mapobjects::Crosswalk*>(laneobject_storage[i]));
                break;
            case mapobjects::LaneObjectType::LO_STOPLINE:
                drawStopline(dynamic_cast<mapobjects::Stopline*>(laneobject_storage[i]));
                break;
        }
    }
}

void Map::drawRoadsign(mapobjects::Roadsign* lo) {
    return;
}
void Map::drawParking(mapobjects::Parking* _lo) {
    int n_parking_lots = _lo->getNumberOfParkingLots();
    int parking_thickness = _lo->getLinewidth();
    std::vector<std::vector<mapobjects::Point3D>> points_outer = _lo->getOuterPoints();
    for (int i = 0; i < n_parking_lots; i++) {
        int n_points_outer_i = points_outer[i].size();
        std::vector<cv::Point2i> parking_outer_points;
        for (int j = 0; j < n_points_outer_i; j++) {
            parking_outer_points.push_back(cv::Point2i(
                getPaddedX(points_outer[i][j].getX()),
                getPaddedY(points_outer[i][j].getY())));
        }
        const cv::Point2i* _pts = (const cv::Point2i*) cv::Mat(parking_outer_points).data;
        int npts = cv::Mat(parking_outer_points).rows;
        cv::polylines(map_discretized, &_pts, &npts, 1, true,
            cv::Scalar(255), parking_thickness, 4, 0);
    }
    return;
}
void Map::drawCrosswalk(mapobjects::Crosswalk* lo)  {
    return;
}
void Map::drawStopline(mapobjects::Stopline* lo) {
    return;
}

void Map::drawWaypoint(mapobjects::GlobalWaypoint waypoint) {
    std::vector<int> position_discretized = \
        getDiscretizedPosition(waypoint.pose.getX(), waypoint.pose.getY());
    if (drawing_debug) std::cout << "MapDraw: waypoint pos / cm (relative): " \
        << waypoint.pose.getX() << "   " << waypoint.pose.getY() << std::endl;
    if (drawing_debug) std::cout << "MapDraw: waypoint pos / px (absolute): " \
        << position_discretized[0] << "   " << position_discretized[1] << std::endl;
    double heading = waypoint.pose.getT();
    int radius = 10;
    int dir_x = position_discretized[0] + cos(heading) * (radius - 1);
    int dir_y = position_discretized[1] + sin(heading) * (radius - 1);
    cv::circle(
        map_discretized,
        cv::Point2i(
            position_discretized[0],
            position_discretized[1]),
        radius,
        cv::Scalar(255),
        -1,
        8,
        0);
    cv::line(
        map_discretized,
        cv::Point2i(
            position_discretized[0],
            position_discretized[1]),
        cv::Point2i(
            dir_x,
            dir_y),
        cv::Scalar(0),
        3,
        4,
        0);
}
void Map::drawPoint3D(mapobjects::Point3D p, int thickness, int radius) {
    cv::circle(
        map_discretized,
        cv::Point2i(
            getPaddedX(p.getX()),
            getPaddedY(p.getY())),
        radius,
        cv::Scalar(255),
        thickness,
        8,
        0);
}
void Map::drawLine(mapobjects::Point3D p1, mapobjects::Point3D p2, int thickness) {
    cv::line(
        map_discretized,
        cv::Point2i(
            getPaddedX(p1.getX()),
            getPaddedY(p1.getY())),
        cv::Point2i(
            getPaddedX(p2.getX()),
            getPaddedY(p2.getY())),
        cv::Scalar(255),
        thickness,
        8,
        0);
}
void Map::drawLanePoint(mapobjects::LanePoint3D lanepoint, int thickness, bool enable_direction) {
    drawLanePoint(lanepoint, thickness, enable_direction, false);
}
void Map::drawLanePoint(mapobjects::LanePoint3D lanepoint, int thickness,
    bool enable_direction, bool first_lp) {
    int x_discretized = getPaddedX(lanepoint.p.getX());
    int y_discretized = getPaddedY(lanepoint.p.getY());
    if (drawing_debug) std::cout << "MapDraw: lanepoint pos / cm: " \
        << lanepoint.p.getX() << "   " << lanepoint.p.getY() << std::endl;
    if (drawing_debug) std::cout << "MapDraw: lanepoint pos / px: " \
        << x_discretized << "   " << y_discretized << std::endl;
    int radius = 5;
    if (first_lp) {
        radius = 15;
    }
    cv::circle(
        map_discretized,
        cv::Point2i(
            x_discretized,
            y_discretized),
        radius,
        cv::Scalar(255),
        thickness,
        8,
        0);
    if (enable_direction) {
        double heading = getLanePointDirection(lanepoint);
        int dir_x = x_discretized + cos(heading) * (radius);
        int dir_y = y_discretized + sin(heading) * (radius);
        cv::line(
            map_discretized,
            cv::Point2i(
                x_discretized,
                y_discretized),
            cv::Point2i(
                dir_x,
                dir_y),
            cv::Scalar(255),
            2,
            4,
            0);
    }
}
void Map::drawCar(mapobjects::Pose car_pose) {
    int pos_x = getDiscretizedX(car_pose.getX());
    int pos_y = getDiscretizedY(car_pose.getY());
    int radius = 10;
    double heading = car_pose.getT();
    int dir_x = pos_x + cos(heading) * radius;
    int dir_y = pos_y + sin(heading) * radius;
    if (drawing_debug) std::cout << "MapDraw: car pos / cm (relative): " \
        << car_pose.getX() << "   " << car_pose.getY() << std::endl;
    if (drawing_debug) std::cout << "MapDraw: car pos / px (absolute): " \
        << pos_x << "   " << pos_y << std::endl;
    if (drawing_debug) std::cout << "MapDraw: car dir / px (absolute): " \
        << dir_x << "   " << dir_y << std::endl;
    cv::circle(
        map_discretized,
        cv::Point2i(
            pos_x,
            pos_y),
        radius,
        cv::Scalar(255),
        3,
        8,
        0);
    cv::line(
        map_discretized,
        cv::Point2i(
            pos_x,
            pos_y),
        cv::Point2i(
            dir_x,
            dir_y),
        cv::Scalar(255),
        3,
        4,
        0);
}
bool Map::writeMapToPng(int img_padding, std::string path) {
    std::lock_guard<std::mutex> guard(map_data_access);

    if (file_debug) {
        std::cout << "MapDraw:\t\t[Writing map to .png]" << std::endl << std::flush;
        std::cout << "\t\timg_padding: " << img_padding << std::endl << std::flush;
        std::cout << "\t\tmax_padding: " << max_padding << std::endl << std::flush;
    }
    if (img_padding > max_padding) {
        img_padding = max_padding;
        std::cout << "\t\tWARNING: img_padding exceeds max_padding" << std::endl << std::flush;
    }
    if (y_min > y_max || x_min > x_max) {
        if (file_debug) std::cout << "\t\tdiscretized map bounding box invalid. not saving." \
            << std::endl << std::flush;
        return false;
    }

    if (file_debug) {
        std::cout << "\t\ty min|max: " << y_min << " | " << y_max << std::endl << std::flush;
        std::cout << "\t\tx min|max: " << x_min << " | " << x_max << std::endl << std::flush;
    }
    try {
        cv::Mat map_cropped = map_discretized(
            cv::Range(
                std::max(0,
                    (y_min + max_padding) * discretization - img_padding),
                std::min((y_max + max_padding) * discretization + img_padding,
                    map_discretized.size().height)),
            cv::Range(
                std::max(0,
                    (x_min + max_padding) * discretization - img_padding),
                std::min((x_max + max_padding) * discretization + img_padding,
                    map_discretized.size().width)));

        cv::imwrite(path, map_cropped);
        if (file_debug) {
            std::cout << "\t\tcropped map size y|x: " << map_cropped.size().height << \
                " | " << map_cropped.size().width << std::endl << std::flush;
            std::cout << "\t\tMap saved as: " << path << std::endl << std::flush;
        }
        return true;
    }
    catch (cv::Exception& ex) {
        fprintf(stderr, "Exception exporting map as .png file: %s\n", ex.what());
        return false;
    }
}
bool Map::writeMapWindowToPng(const cv::Mat& window, const std::string& path) {
    if (file_debug) {
        std::cout << "MapDraw:\t\t[Writing map window to .png]" << std::endl << std::flush;
        std::cout << "\t\ty width|height: " << window.size().width << " | " \
            << window.size().height << std::endl << std::flush;
    }
    try {
        cv::imwrite(path, window);
        if (file_debug) std::cout << "\t\tMap window saved as: " << path << std::endl << std::flush;
        return true;
    }
    catch (cv::Exception& ex) {
        fprintf(stderr, "Exception exporting map window as .png file: %s\n", ex.what());
        return false;
    }
}

cv::Point2i Map::getMappedPoint(mapobjects::Point3D p, int discretization) {
    std::vector<double> p_coords = p.getCoords();
    int x_mapped = p_coords[0] * discretization;
    int y_mapped = p_coords[1] * discretization;
    x_mapped += (max_padding * discretization);
    y_mapped += (max_padding * discretization);
    return cv::Point2i(std::round(x_mapped), std::round(y_mapped));
}

void Map::drawDiscretizedLines(
    std::vector<mapobjects::Point3D> points,
    int line_cost, int discretization, int thickness) {
    int n_points = points.size();
        for (int j = 0; j < (n_points - 1); j++) {
            cv::line(
                map_discretized,
                getMappedPoint(points[j], discretization),
                getMappedPoint(points[j + 1], discretization),
                cv::Scalar(line_cost),
                thickness,
                8,
                0);
        }
}
cv::Mat Map::getDiscretizedMapArea(
    int window_width, int window_height, double car_x, double car_y, double car_angle) {
    std::lock_guard<std::mutex> guard(map_data_access);

    std::cout << "\t\t[Creating local map]" << std::endl << std::flush;
    std::cout << "\t\tRegion of Interest: " << window_width << "x" << window_height << \
        " at x|y|angle " << car_x << "|" << car_y << "|" << car_angle << std::endl << std::flush;
    if (map_discretized_changed) {
        discretizeGlobalMap();
    }
    if (window_height > max_padding) {
        window_height = max_padding;
        std::cout << "\t\tWARNING: window height larger max" << std::endl << std::flush;
    }
    if (window_width > max_padding) {
        window_width = max_padding;
        std::cout << "\t\tWARNING: window width larger max" << std::endl << std::flush;
    }

    cv::Mat map_roi;
    cv::Mat map_window;
    car_angle = fmod(car_angle, 360.0);
    std::vector<int> window_pos = getDiscretizedPosition(
        car_x + cos(car_angle * (M_PI / 180)) * (window_height / 2),
        car_y + sin(car_angle * (M_PI / 180)) * (window_height / 2));

    double window_center_x = static_cast<double>(window_pos[0]);
    double window_center_y = static_cast<double>(window_pos[1]);
    std::cout << "\t\tWindow center x|y: " << window_center_x << "|" << window_center_y \
        << std::endl << std::flush;
    car_angle += 90;
    cv::RotatedRect window = cv::RotatedRect(
        cv::Point2f(window_center_x, window_center_y),
        cv::Size2f(window_width, window_height), car_angle);
    cv::Rect roi = window.boundingRect();
    cv::Size window_size = window.size;
    if (window.angle < -45.) {
        car_angle += 90.0;
        cv::swap(window_size.width, window_size.height);
    }
    cv::Mat rot_mat = cv::getRotationMatrix2D(window.center, car_angle, 1);
    cv::Mat map_bb;
    try {
        map_bb = map_discretized(roi);
    } catch (cv::Exception e) {
        std::cout << "Map::getDiscretizedMapArea: detected cv roi exception";
        throw e;
    }
    cv::warpAffine(map_bb, map_roi, rot_mat, map_discretized.size(), cv::INTER_NEAREST);
    cv::getRectSubPix(map_roi, window_size, window.center, map_window);
    return map_window;
}
cv::Mat Map::getCenteredMapWindow(
    int window_width, int window_height, double car_x, double car_y) {
    return getShiftedMapWindow(window_width, window_height, car_x, car_y, 0.0, 0.0);
}
cv::Mat Map::getShiftedMapWindow(int window_width, int window_height, double car_x, double car_y, \
    double offset_x, double offset_y) {
    std::lock_guard<std::mutex> guard(map_data_access);

    if (discretization_debug) {
        std::cout << "\t\t*** [Creating discretized map window] ***" << std::endl << std::flush;
        std::cout << "\t\tRegion of Interest: " \
            << window_width << "x" << window_height << std::endl;
        std::cout << "\t\tCar position:" << car_x << " | " << car_y << std::endl << std::flush;
    }

    if (map_discretized_changed) {
        discretizeGlobalMap();
    }
    if (window_height > max_padding) {
        window_height = max_padding;
        std::cout << "\t\tWARNING: window height larger max_padding" << std::endl << std::flush;
    }
    if (window_width > max_padding) {
        window_width = max_padding;
        std::cout << "\t\tWARNING: window width larger max_padding" << std::endl << std::flush;
    }
    if (offset_y > window_height / 2) {
        offset_y = window_height / 2;
        std::cout << "\t\tWARNING: offset_y larger window_height" << std::endl << std::flush;
    }
    if (offset_x > window_width / 2) {
        offset_x = window_width / 2;
        std::cout << "\t\tWARNING: offset_x larger window_width" << std::endl << std::flush;
    }

    std::vector<int> window_pos = getDiscretizedPosition(
        car_x - window_width / 2 + offset_x,
        car_y - window_height / 2 + offset_y);
    int window_left = window_pos[0];
    int window_top = window_pos[1];
    if (discretization_debug) std::cout \
        << "\t\twindow left|top\t" << window_left << " | " << window_top << std::endl;

    cv::Rect roi = cv::Rect(
        cv::Point2i(window_left, window_top),
        cv::Size2i(window_width, window_height));

    cv::Mat map_window;
    try {
        map_window = map_discretized(roi);
    } catch (cv::Exception e) {
        std::cout << "Map::getShiftedMapWindow: detected opencv roi exception" << std::endl;
        throw e;
    }
}
cv::Mat Map::getShiftedMapWindow(
    int window_width, int window_height, \
    double offset_x, double offset_y,
    double car_x, double car_y, \
    double car_angle) {
    std::lock_guard<std::mutex> guard(map_data_access);

    if (discretization_debug) {
        std::cout << "\t\t*** [Creating rotated map window] ***" << std::endl << std::flush;
        std::cout << "\t\tRegion of Interest: "
            << window_width << "x" << window_height << std::endl;
        std::cout << "\t\tCar position:" << car_x << " | " << car_y
            << " | heading: " << car_angle << std::endl << std::flush;
    }

    if (map_discretized_changed) {
        discretizeGlobalMap();
    }
    if (window_height > max_padding) {
        window_height = max_padding;
        std::cout << "\t\tWARNING: window height larger max_padding" << std::endl << std::flush;
    }
    if (window_width > max_padding) {
        window_width = max_padding;
        std::cout << "\t\tWARNING: window width larger max_padding" << std::endl << std::flush;
    }
    if (offset_y > window_height / 2) {
        offset_y = window_height / 2;
        std::cout << "\t\tWARNING: offset_y larger window_height" << std::endl << std::flush;
    }
    if (offset_x > window_width / 2) {
        offset_x = window_width / 2;
        std::cout << "\t\tWARNING: offset_x larger window_width" << std::endl << std::flush;
    }

    cv::Mat map_roi;
    cv::Mat map_window;
    car_angle = fmod(car_angle, 360.0);
    std::vector<int> window_center = getDiscretizedPosition(
        car_x + offset_x * cos(car_angle / 180 * M_PI) - offset_y * sin(car_angle / 180 * M_PI),
        car_y + offset_x * sin(car_angle / 180 * M_PI) + offset_y * cos(car_angle / 180 * M_PI));

    double window_center_x = static_cast<double>(window_center[0]);
    double window_center_y = static_cast<double>(window_center[1]);
    if (discretization_debug) std::cout << "\t\tWindow center x|y: " << window_center_x << "|" \
        << window_center_y << std::endl << std::flush;
    car_angle += 90;
    cv::RotatedRect window = cv::RotatedRect(
        cv::Point2f(window_center_x, window_center_y),
        cv::Size2f(window_width, window_height), car_angle);
    cv::Rect roi = window.boundingRect();
    cv::Size window_size = window.size;
    if (window.angle < -45.) {
        car_angle += 90.0;
        cv::swap(window_size.width, window_size.height);
    }
    cv::Mat rot_mat = cv::getRotationMatrix2D(window.center, car_angle, 1);
    cv::Mat map_bb = map_discretized(roi);
    cv::warpAffine(map_bb, map_roi, rot_mat, map_bb.size(), cv::INTER_NEAREST);
    cv::getRectSubPix(map_roi, window_size, window.center, map_window);
    return map_window;
}

void Map::updateBoundingBox(mapobjects::Point3D p) {
    std::vector<double> p_coords = p.getCoords();
    // x
    if (p_coords[0] < x_min) {
        x_min = p_coords[0];
    } else if (p_coords[0] > x_max) {
        x_max = p_coords[0];
    }
    // y
    if (p_coords[1] < y_min) {
        y_min = p_coords[1];
    } else if (p_coords[1] > y_max) {
        y_max = p_coords[1];
    }
}

void Map::changeDiscretizationLevel(int new_discretization) {
    if (new_discretization > 0) {
        discretization = new_discretization;
        map_discretized_changed = true;
        discretizeGlobalMap();
    }
}

void Map::triggerMapDiscretization() {
    std::lock_guard<std::mutex> guard(map_data_access);

    discretizeGlobalMap();
}
std::vector<int> Map::getDiscretizedPosition(double x, double y) {
    return getDiscretizedPosition(x, y, 0.0);
}
std::vector<int> Map::getDiscretizedPosition(double x, double y, double z) {
    std::vector<int> pos_discretized = std::vector<int>(3);
    pos_discretized[0] = getDiscretizedX(x);
    pos_discretized[1] = getDiscretizedY(y);
    pos_discretized[2] = getDiscretizedZ(z);
    return pos_discretized;
}
int Map::getDiscretizedX(double x) {
    return static_cast<int>((x + max_padding + map_pivot.getPose().getX()) * discretization);
}
int Map::getDiscretizedY(double y) {
    return static_cast<int>((y + max_padding + map_pivot.getPose().getY()) * discretization);
}
int Map::getDiscretizedZ(double z) {
    return static_cast<int>(round(z));
}
int Map::getPaddedX(double x) {
    return static_cast<int>((x + max_padding) * discretization);
}
int Map::getPaddedY(double y) {
    return static_cast<int>((y + max_padding) * discretization);
}
std::vector<double> Map::getWorldPosition(int x, int y) {
    return getWorldPosition(x, y, 0);
}
std::vector<double> Map::getWorldPosition(int x, int y, int z) {
    std::vector<double> pos_discretized = std::vector<double>(3);
    pos_discretized[0] = getWorldX(x);
    pos_discretized[1] = getWorldY(y);
    pos_discretized[2] = getWorldZ(z);
    return pos_discretized;
}
double Map::getWorldX(int x) {
    return (static_cast<double>(x) / discretization) - map_pivot.getPose().getX() - max_padding;
}
double Map::getWorldY(int y) {
    return (static_cast<double>(y) / discretization) - map_pivot.getPose().getY() - max_padding;
}
double Map::getWorldZ(int z) {
    return static_cast<double>(z);
}
double Map::getRelativeX(double x) {
    return x - map_pivot.getPose().getX();
}
double Map::getRelativeY(double y) {
    return y - map_pivot.getPose().getY();
}
double Map::getAbsoluteX(double x) {
    return x + map_pivot.getPose().getX();
}
double Map::getAbsoluteY(double y) {
    return y + map_pivot.getPose().getY();
}
mapobjects::Point3D Map::getRelativePoint3D(const mapobjects::Point3D& p) {
    return mapobjects::Point3D(
        getRelativeX(p.getX()),
        getRelativeY(p.getY()),
        p.getZ());
}
void Map::setMapChanged() {
    std::lock_guard<std::mutex> guard(map_data_access);

    doSetMapChanged();
}
void Map::setMapUpdated() {
    std::lock_guard<std::mutex> guard(map_data_access);

    doSetMapUpdated();
}
bool Map::isMapChanged() {
    std::lock_guard<std::mutex> guard(map_data_access);

    return map_changed;
}
bool Map::isDiscretizationChanged() {
    std::lock_guard<std::mutex> guard(map_data_access);

    return map_discretized_changed;
}
void Map::doSetMapChanged() {
    map_changed = true;
    map_discretized_changed = true;
}
void Map::doSetMapUpdated() {
    map_changed = false;
}

void Map::doSetMapStatus(MapStatus status) {
    internal_map_status = status;
}

MapStatus Map::getMapStatus() {
    return internal_map_status;
}
mapobjects::GlobalWaypoint Map::getNextGlobalWaypoint(
    const mapobjects::Pose& pose, double waypoint_distance,
    bool ignoreStoplines, bool ignoreCrosswalks, bool ignoreParking) {
    if (waypoint_debug) std::cout << "*** MapWaypoint: searching next waypoint from pose ***" \
        << std::endl;

    mapobjects::GlobalWaypoint waypoint = mapobjects::GlobalWaypoint(
        mapobjects::WaypointType::ERROR);

    if (waypoint_debug) std::cout << "MapWaypoint: pivot pos / cm (absolute): " \
        << map_pivot.getX() << "   " << map_pivot.getY() << std::endl;
    if (waypoint_debug) std::cout << "MapWaypoint: search pos / cm (relative) " \
        << pose.getX() << "   " << pose.getY() << std::endl;
    mapobjects::Pose pose_abs = mapobjects::Pose(
        getAbsoluteX(pose.getX()), getAbsoluteY(pose.getY()), pose.getZ(), pose.getT());
    if (waypoint_debug) std::cout << "MapWaypoint: search pos / cm (absolute) " \
        << pose_abs.getX() << "   " << pose_abs.getY() << std::endl;
    std::lock_guard<std::mutex> guard(map_data_access);
1f    mapobjects::LanePoint3D lp_start = getNearestLanePoint(pose_abs);
    if (geometry_debug) drawLanePoint(lp_start, 4, false, true);

    return globalWaypointSearch(lp_start, waypoint_distance,
        ignoreStoplines, ignoreCrosswalks, ignoreParking);
}
mapobjects::LanePoint3D Map::getInterpolatedLanePoint(
    const mapobjects::LanePoint3D& lp_next, const double& offset) {
    if (lanepoint_debug) std::cout << "MapLanePoint: interpolating new point" << std::endl;

    double current_lane_offset = getOffsetAlongLane(lp_next);

    mapobjects::Point3D lp_p_interpolated = getPoint3DFromLaneOffset(
        lp_next,
        current_lane_offset + offset);
    mapobjects::LanePoint3D lp_interpolated = lp_next;
    lp_interpolated.p = lp_p_interpolated;
    if (!lp_next.isInterpolated) {
        if (lp_next.p_index > 0) {
            lp_interpolated.p_index--;
        }
    } else {
        lp_interpolated.p_index = lp_next.p_index;
    }
    lp_interpolated.isInterpolated = true;
    lp_interpolated.lp_type = determineLanePointType(lp_interpolated, false);
    lp_interpolated.lane_info = getLaneInfo(lp_interpolated.lane_uuid);
    return lp_interpolated;
}
mapobjects::GlobalWaypoint Map::getNextGlobalWaypoint(
    const mapobjects::LanePoint3D& lp, double waypoint_distance,
    bool ignoreStoplines, bool ignoreCrosswalks, bool ignoreParking) {
    if (waypoint_debug) std::cout << "*** MapWaypoint: searching next waypoint from lp ***" \
        << std::endl;
    mapobjects::GlobalWaypoint waypoint = mapobjects::GlobalWaypoint(
        mapobjects::WaypointType::ERROR);


    mapobjects::LanePoint3D lp_start = lp;
    if (geometry_debug) drawLanePoint(lp_start, 4, false, true);

    mapobjects::GlobalWaypoint wp = globalWaypointSearch(lp_start, waypoint_distance,
        ignoreStoplines, ignoreCrosswalks, ignoreParking);
    return wp;
}
mapobjects::GlobalWaypoint Map::globalWaypointSearch(
    const mapobjects::LanePoint3D& lp_start, double waypoint_distance,
    bool ignoreStoplines, bool ignoreCrosswalks, bool ignoreParking) {
    mapobjects::GlobalWaypoint waypoint = mapobjects::GlobalWaypoint(
        mapobjects::WaypointType::ERROR);
    double distance_to_waypoint = 0.0;
    mapobjects::LanePoint3D lp_waypoint = lp_start;
    mapobjects::LanePoint3D lp_waypoint_next;

    if (waypoint_debug) std::cout << "*** MapWaypoint: staring waypoint search ***" << std::endl;
    if (waypoint_debug) std::cout << "MapWaypoint: goal waypoint distance / cm: " \
        << waypoint_distance << std::endl;
    if (waypoint_debug) std::cout << "MapWaypoint: starting waypoint search" << std::endl;

    bool skip_search = false;
    if (lp_waypoint.lp_type.type == mapobjects::WaypointType::JUNCTION) {
        if (waypoint_debug) std::cout << "MapWaypoint: waypoint is junction" << std::endl;
        if (waypoint_debug) std::cout << "MapWaypoint: skipping waypoint search" << std::endl;
        lp_waypoint_next = lp_waypoint;
        waypoint.type = mapobjects::WaypointType::JUNCTION;
        skip_search = true;
    }
    while ((distance_to_waypoint < waypoint_distance) && !skip_search) {
        lp_waypoint_next = getNextLanePoint(lp_waypoint, 0);
        if (geometry_debug) drawLanePoint(lp_waypoint_next, 2, false);
        distance_to_waypoint += lp_waypoint.p.computeDistance(lp_waypoint_next.p);

        if (waypoint_debug) std::cout << "MapWaypoint: current waypoint distance / cm: " \
            << distance_to_waypoint << std::endl;
        if (distance_to_waypoint >= waypoint_distance) {
            if (waypoint_debug) std::cout \
                << "MapWaypoint: waypoint distance exceeded. Interpolating" << std::endl;
            double distance_difference = waypoint_distance - distance_to_waypoint;  // negative

            if (waypoint_debug) std::cout \
                << "MapWaypoint: interpolated distance: " \
                << distance_to_waypoint + distance_difference << std::endl;

            lp_waypoint_next = getInterpolatedLanePoint(lp_waypoint_next, distance_difference);
            skip_search = true;
        }
        if (lp_waypoint_next.lp_type.type == mapobjects::WaypointType::ERROR) {
            std::cout << "MapWaypoint: waypoint is ERROR_TYPE *****" << std::endl;
            break;
        } else if (lp_waypoint_next.lp_type.type == mapobjects::WaypointType::DEADEND) {
            if (waypoint_debug) std::cout << "MapWaypoint: waypoint is deadend" << std::endl;
            waypoint.type = mapobjects::WaypointType::DEADEND;
            break;
        } else if (lp_waypoint_next.lp_type.type == mapobjects::WaypointType::CROSSWALK) {
            if (waypoint_debug) std::cout << "MapWaypoint: waypoint is crosswalk" << std::endl;
            if (!ignoreCrosswalks) {
                waypoint.type = mapobjects::WaypointType::CROSSWALK;
                double crosswalk_offset = lp_waypoint_next.lp_type.offset;
                lp_waypoint_next = getInterpolatedLanePoint(
                    lp_waypoint_next, crosswalk_offset - getOffsetAlongLane(lp_waypoint_next));
                lp_waypoint_next.lp_type.type = mapobjects::WaypointType::CROSSWALK;

                if (waypoint_debug) std::cout << "MapWaypoint: stopping waypoint search" \
                    << std::endl;
                break;
            }
            if (waypoint_debug) std::cout << "MapWaypoint: proceeding search" << std::endl;
            waypoint.type = mapobjects::WaypointType::CROSSWALK;
        } else if (lp_waypoint_next.lp_type.type == mapobjects::WaypointType::STOPLINE) {
            if (waypoint_debug) std::cout << "MapWaypoint: waypoint is stopline" << std::endl;
            if (!ignoreStoplines) {
                waypoint.type = mapobjects::WaypointType::STOPLINE;
                double stopline_offset = lp_waypoint_next.lp_type.offset;
                lp_waypoint_next = getInterpolatedLanePoint(
                    lp_waypoint_next, stopline_offset - getOffsetAlongLane(lp_waypoint_next));
                lp_waypoint_next.lp_type.type = mapobjects::WaypointType::STOPLINE;

                if (waypoint_debug) std::cout << "MapWaypoint: stopping waypoint search" \
                    << std::endl;

                break;
            }
            if (waypoint_debug) std::cout << "MapWaypoint: proceeding search" << std::endl;
            waypoint.type = mapobjects::WaypointType::STOPLINE;
        } else if (lp_waypoint_next.lp_type.type == mapobjects::WaypointType::JUNCTION) {
            if (waypoint_debug) std::cout << "MapWaypoint: waypoint is junction" << std::endl;
            if (waypoint_debug) std::cout << "MapWaypoint: stopping waypoint search" << std::endl;
            waypoint.type = mapobjects::WaypointType::JUNCTION;
            break;
        } else if (lp_waypoint_next.lp_type.type == mapobjects::WaypointType::PARKING) {
            if (waypoint_debug) std::cout << "MapWaypoint: waypoint is parking" << std::endl;
            if (!ignoreParking) {
                waypoint.type = mapobjects::WaypointType::PARKING;
                if (waypoint_debug) std::cout << "MapWaypoint: stopping waypoint search" \
                    << std::endl;
                break;
            }
            if (waypoint_debug) std::cout << "MapWaypoint: proceeding search" << std::endl;
        } else {
            waypoint.type = mapobjects::WaypointType::ROAD;
            if (waypoint_debug) std::cout \
                << "MapWaypoint: intermediate waypoint is road" << std::endl;
        }
        lp_waypoint = lp_waypoint_next;
    }
    waypoint.pose = mapobjects::Pose(
        getRelativePoint3D(lp_waypoint_next.p),
        getLanePointDirection(lp_waypoint_next));
    waypoint.lp = lp_waypoint_next;

    if (waypoint_debug) {
        std::cout << "MapWaypoint: waypoint position (x,y) / cm:\t" \
            << (waypoint.pose.getX()) << "   "<< (waypoint.pose.getY()) << std::endl;
        std::cout << "MapWaypoint: returning waypoint" << std::endl;
    }
    if (lp_start.isOppositeLane) {
        waypoint.isOppositeLane = true;
    }

    if (waypoint_debug) std::cout << "*** MapWaypoint: returning from search ***" << std::endl;
    return waypoint;
}
mapobjects::GlobalWaypoint Map::getWaypointOnOppositeLane(
    const mapobjects::GlobalWaypoint& wp, bool invertHeading = false) {
    mapobjects::LanePoint3D lp_opposite = getNearestOppositeLanePoint(wp.lp);
    if (lp_opposite.lane_uuid.equals(wp.lp.lane_uuid)) {
        std::cout << "MapWaypoint: ERROR: getWaypointOnOppositeLane(): no opposite lane found"
            << std::endl;
        throw mapobjects::WaypointException::NO_OPPOSITE_LANE;
    }
    double lp_opposite_direction = wp.pose.getT();
    if (!invertHeading) {
        lp_opposite_direction = getLanePointDirection(lp_opposite);
    }
    return mapobjects::GlobalWaypoint(
        lp_opposite,
        mapobjects::WaypointType::OPPOSITE_ROAD,
        mapobjects::Pose(
            getRelativePoint3D(lp_opposite.p),
            lp_opposite_direction));
}
std::vector<mapobjects::GlobalWaypoint> Map::getWaypointsTurnLeft(mapobjects::LanePoint3D lp) {
    std::vector<mapobjects::GlobalWaypoint> waypoints = std::vector<mapobjects::GlobalWaypoint>();
    mapobjects::GlobalWaypoint waypoint = mapobjects::GlobalWaypoint(lp);
    if (junction_debug) std::cout \
        << "*** MapWaypoint: searching waypoint: turn left ***" << std::endl;
    std::vector<mapobjects::Uuid> lane_connections_out = \
        getRealConnectionsOut(getLaneFromLaneUuid(lp.lane_uuid));
    int n_connections_out = lane_connections_out.size();
    if (junction_debug) std::cout << "MapWaypoint: outgoing connections: " \
        << n_connections_out << std::endl;
    if (n_connections_out < 1) {
        if (junction_debug) std::cout << "MapWaypoint: no outgoing connections" << std::endl;
        throw mapobjects::WaypointException::NO_OUTGOING_CONNECTIONS;
    } else if (n_connections_out == 1) {
        if (junction_debug) std::cout << "MapWaypoint: one outgoing connection" << std::endl;
        mapobjects::LanePoint3D lp_out = getFirstLanePoint(getNextLaneUuid(lp.lane_uuid));
        waypoint.lp = lp_out;
        waypoint.type = lp_out.lp_type.type;
        waypoint.pose = mapobjects::Pose(
            getRelativePoint3D(lp_out.p),
            getLanePointDirection(lp_out));
        waypoints.push_back(waypoint);
    } else if (n_connections_out == 2) {
        if (junction_debug) std::cout << "MapWaypoint: 2 outgoing connections" << std::endl;
        double heading_junction_in = getLanePointDirection(lp);
        if (geometry_debug) drawLanePoint(lp, 2, true);
        mapobjects::LanePoint3D lp_out_1 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[0]));
        double heading_out_1 = getLanePointDirection(lp_out_1);
        bool turn_left_1_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_1, 2, true);
        mapobjects::LanePoint3D lp_out_2 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[1]));
        double heading_out_2 = getLanePointDirection(lp_out_2);
        bool turn_left_2_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_2, 2, true);
        double heading_to_out_1 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_1);
        double heading_to_out_2 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_2);

        if ((heading_to_out_1 > (JUNCTION_LEFT_ANGLE - JUNCTION_LEFT_MARGIN)) &&
            (heading_to_out_1 < (JUNCTION_LEFT_ANGLE + JUNCTION_LEFT_MARGIN))) {
            turn_left_1_valid = true;
        }
        if ((heading_to_out_2 > (JUNCTION_LEFT_ANGLE - JUNCTION_LEFT_MARGIN)) &&
            (heading_to_out_2 < (JUNCTION_LEFT_ANGLE + JUNCTION_LEFT_MARGIN))) {
            turn_left_2_valid = true;
        }

        if (junction_debug) {
            std::cout << "MapWaypoint: lp headings:\t1: " \
                << heading_junction_in << "\t2: " << heading_out_1 << "\t3: " \
                << heading_out_2 << std::endl;
            std::cout << "MapWaypoint: headings:\t1: " \
                << heading_to_out_1 << "\t2: " << heading_to_out_2 << std::endl;
            std::cout << "MapWaypoint: valid?:\t1: " \
                << turn_left_1_valid << "\t2: " << turn_left_2_valid << std::endl;
        }
        if (turn_left_1_valid && turn_left_2_valid) {
            throw mapobjects::WaypointException::MULTIPLE_TURN_LEFT_DETECTED;
        } else if (turn_left_1_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[0]);
            waypoint.lp = lp_out_1;
            waypoint.type = determineLanePointType(lp_out_1, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_1.p),
                getLanePointDirection(lp_out_1));
            waypoints.push_back(waypoint);
        } else if (turn_left_2_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[1]);
            waypoint.lp = lp_out_2;
            waypoint.type = determineLanePointType(lp_out_2, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_2.p),
                getLanePointDirection(lp_out_2));
            waypoints.push_back(waypoint);
        } else {
            throw mapobjects::WaypointException::NO_TURN_LEFT_DETECTED;
        }
    } else if (n_connections_out == 3) {
        if (junction_debug) std::cout << "MapWaypoint: 3 outgoing connections" << std::endl;
        double heading_junction_in = getLanePointDirection(lp);
        if (geometry_debug) drawLanePoint(lp, 2, true);
        mapobjects::LanePoint3D lp_out_1 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[0]));
        double heading_out_1 = getLanePointDirection(lp_out_1);
        bool turn_left_1_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_1, 2, true);
        mapobjects::LanePoint3D lp_out_2 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[1]));
        double heading_out_2 = getLanePointDirection(lp_out_2);
        bool turn_left_2_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_2, 2, true);
        mapobjects::LanePoint3D lp_out_3 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[2]));
        double heading_out_3 = getLanePointDirection(lp_out_3);
        bool turn_left_3_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_3, 2, true);
        double heading_to_out_1 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_1);
        double heading_to_out_2 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_2);
        double heading_to_out_3 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_3);

        if ((heading_to_out_1 > (JUNCTION_LEFT_ANGLE - JUNCTION_LEFT_MARGIN)) &&
            (heading_to_out_1 < (JUNCTION_LEFT_ANGLE + JUNCTION_LEFT_MARGIN))) {
            turn_left_1_valid = true;
        }
        if ((heading_to_out_2 > (JUNCTION_LEFT_ANGLE - JUNCTION_LEFT_MARGIN)) &&
            (heading_to_out_2 < (JUNCTION_LEFT_ANGLE + JUNCTION_LEFT_MARGIN))) {
            turn_left_2_valid = true;
        }
        if ((heading_to_out_3 > (JUNCTION_LEFT_ANGLE - JUNCTION_LEFT_MARGIN)) &&
            (heading_to_out_3 < (JUNCTION_LEFT_ANGLE + JUNCTION_LEFT_MARGIN))) {
            turn_left_3_valid = true;
        }

        if (junction_debug) {
            std::cout << "MapWaypoint: lp headings:\t1: " \
                << heading_junction_in << "\t2: " << heading_out_1 << "\t3: " \
                << heading_out_2 << "\t4: " \
                << heading_out_3 << std::endl;
            std::cout << "MapWaypoint: headings:\t1: " \
                << heading_to_out_1 << "\t2: " << heading_to_out_2 << "\t3: " \
                << heading_to_out_3 << std::endl;
            std::cout << "MapWaypoint: valid?:\t1: " \
                << turn_left_1_valid << "\t2: " << turn_left_2_valid << "\t3: " \
                << turn_left_3_valid << std::endl;
        }
        if ((turn_left_1_valid && turn_left_2_valid) ||
            (turn_left_1_valid && turn_left_3_valid) ||
            (turn_left_2_valid && turn_left_3_valid)) {
            throw mapobjects::WaypointException::MULTIPLE_TURN_LEFT_DETECTED;
        } else if (turn_left_1_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[0]);
            waypoint.lp = lp_out_1;
            waypoint.type = determineLanePointType(lp_out_1, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_1.p),
                getLanePointDirection(lp_out_1));
            waypoints.push_back(waypoint);
        } else if (turn_left_2_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[1]);
            waypoint.lp = lp_out_2;
            waypoint.type = determineLanePointType(lp_out_2, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_2.p),
                getLanePointDirection(lp_out_2));
            waypoints.push_back(waypoint);
        } else if (turn_left_3_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[2]);
            waypoint.lp = lp_out_3;
            waypoint.type = determineLanePointType(lp_out_3, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_3.p),
                getLanePointDirection(lp_out_3));
            waypoints.push_back(waypoint);
        } else {
            if (junction_debug) std::cout << "MapWaypoint: no turn left detected" << std::endl;
            throw mapobjects::WaypointException::NO_TURN_LEFT_DETECTED;
        }
    } else {
        if (junction_debug) std::cout \
            << "MapWaypoint: more than 3 outgoing connections" << std::endl;
        throw mapobjects::WaypointException::MORE_THAN_3_OUTGOING_CONNECTIONS;
    }
    return waypoints;
}
mapobjects::GlobalWaypoint Map::getWaypointTurnLeft(mapobjects::LanePoint3D lp) {
    return getWaypointsTurnLeft(lp).back();
}
std::vector<mapobjects::GlobalWaypoint> Map::getWaypointsTurnRight(mapobjects::LanePoint3D lp) {
    std::vector<mapobjects::GlobalWaypoint> waypoints = std::vector<mapobjects::GlobalWaypoint>();
    mapobjects::GlobalWaypoint waypoint = mapobjects::GlobalWaypoint(lp);
    if (junction_debug) std::cout \
        << "*** MapWaypoint: searching waypoint: turn right ***" << std::endl;
    std::vector<mapobjects::Uuid> lane_connections_out = \
        getRealConnectionsOut(getLaneFromLaneUuid(lp.lane_uuid));
    int n_connections_out = lane_connections_out.size();
    if (junction_debug) std::cout << "MapWaypoint: outgoing connections: " \
        << n_connections_out << std::endl;
    if (n_connections_out < 1) {
        if (junction_debug) std::cout << "MapWaypoint: no outgoing connections" << std::endl;
        throw mapobjects::WaypointException::NO_OUTGOING_CONNECTIONS;
    } else if (n_connections_out == 1) {
        if (junction_debug) std::cout << "MapWaypoint: one outgoing connection" << std::endl;
        mapobjects::LanePoint3D lp_out = getFirstLanePoint(getNextLaneUuid(lp.lane_uuid));
        waypoint.lp = lp_out;
        waypoint.type = determineLanePointType(lp_out, false).type;
        waypoint.pose = mapobjects::Pose(
            getRelativePoint3D(lp_out.p),
            getLanePointDirection(lp_out));
        waypoints.push_back(waypoint);
    } else if (n_connections_out == 2) {
        if (junction_debug) std::cout << "MapWaypoint: 2 outgoing connections" << std::endl;
        double heading_junction_in = getLanePointDirection(lp);
        if (geometry_debug) drawLanePoint(lp, 2, true);
        mapobjects::LanePoint3D lp_out_1 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[0]));
        double heading_out_1 = getLanePointDirection(lp_out_1);
        bool turn_right_1_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_1, 2, true);
        mapobjects::LanePoint3D lp_out_2 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[1]));
        double heading_out_2 = getLanePointDirection(lp_out_2);
        bool turn_right_2_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_2, 2, true);
        double heading_to_out_1 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_1);
        double heading_to_out_2 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_2);

        if ((heading_to_out_1 > (JUNCTION_RIGHT_ANGLE - JUNCTION_RIGHT_MARGIN)) &&
            (heading_to_out_1 < (JUNCTION_RIGHT_ANGLE + JUNCTION_RIGHT_MARGIN))) {
            turn_right_1_valid = true;
        }
        if ((heading_to_out_2 > (JUNCTION_RIGHT_ANGLE - JUNCTION_RIGHT_MARGIN)) &&
            (heading_to_out_2 < (JUNCTION_RIGHT_ANGLE + JUNCTION_RIGHT_MARGIN))) {
            turn_right_2_valid = true;
        }

        if (junction_debug) {
            std::cout << "MapWaypoint: lp headings:\t1: " \
                << heading_junction_in << "\t2: " << heading_out_1 << "\t3: " \
                << heading_out_2 << std::endl;
            std::cout << "MapWaypoint: headings:\t1: " \
                << heading_to_out_1 << "\t2: " << heading_to_out_2 << std::endl;
            std::cout << "MapWaypoint: valid?:\t1: " \
                << turn_right_1_valid << "\t2: " << turn_right_2_valid << std::endl;
        }
        if (turn_right_1_valid && turn_right_2_valid) {
            throw mapobjects::WaypointException::MULTIPLE_TURN_RIGHT_DETECTED;
        } else if (turn_right_1_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[0]);
            std::cout << "generated inner waypoints" << std::endl;
            waypoint.lp = lp_out_1;
            waypoint.type = determineLanePointType(lp_out_1, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_1.p),
                getLanePointDirection(lp_out_1));
            waypoints.push_back(waypoint);
        } else if (turn_right_2_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[1]);
            waypoint.lp = lp_out_2;
            waypoint.type = determineLanePointType(lp_out_2, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_2.p),
                getLanePointDirection(lp_out_2));
            waypoints.push_back(waypoint);
        } else {
            throw mapobjects::WaypointException::NO_TURN_RIGHT_DETECTED;
        }
    } else if (n_connections_out == 3) {
        if (junction_debug) std::cout << "MapWaypoint: 2 outgoing connections" << std::endl;
        double heading_junction_in = getLanePointDirection(lp);
        if (geometry_debug) drawLanePoint(lp, 2, true);
        mapobjects::LanePoint3D lp_out_1 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[0]));
        double heading_out_1 = getLanePointDirection(lp_out_1);
        bool turn_right_1_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_1, 2, true);
        mapobjects::LanePoint3D lp_out_2 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[1]));
        double heading_out_2 = getLanePointDirection(lp_out_2);
        bool turn_right_2_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_2, 2, true);
        mapobjects::LanePoint3D lp_out_3 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[2]));
        double heading_out_3 = getLanePointDirection(lp_out_3);
        bool turn_right_3_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_3, 2, true);
        double heading_to_out_1 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_1);
        double heading_to_out_2 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_2);
        double heading_to_out_3 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_3);

        if ((heading_to_out_1 > (JUNCTION_RIGHT_ANGLE - JUNCTION_RIGHT_MARGIN)) &&
            (heading_to_out_1 < (JUNCTION_RIGHT_ANGLE + JUNCTION_RIGHT_MARGIN))) {
            turn_right_1_valid = true;
        }
        if ((heading_to_out_2 > (JUNCTION_RIGHT_ANGLE - JUNCTION_RIGHT_MARGIN)) &&
            (heading_to_out_2 < (JUNCTION_RIGHT_ANGLE + JUNCTION_RIGHT_MARGIN))) {
            turn_right_2_valid = true;
        }
        if ((heading_to_out_3 > (JUNCTION_RIGHT_ANGLE - JUNCTION_RIGHT_MARGIN)) &&
            (heading_to_out_3 < (JUNCTION_RIGHT_ANGLE + JUNCTION_RIGHT_MARGIN))) {
            turn_right_3_valid = true;
        }

        if (junction_debug) {
            std::cout << "MapWaypoint: lp headings:\t1: " \
                << heading_junction_in << "\t2: " << heading_out_1 << "\t3: " \
                << heading_out_2 << "\t4: " \
                << heading_out_3 << std::endl;
            std::cout << "MapWaypoint: headings:\t1: " \
                << heading_to_out_1 << "\t2: " << heading_to_out_2 << "\t3: " \
                << heading_to_out_3 << std::endl;
            std::cout << "MapWaypoint: valid?:\t1: " \
                << turn_right_1_valid << "\t2: " << turn_right_2_valid << "\t3: " \
                << turn_right_3_valid << std::endl;
        }
        if ((turn_right_1_valid && turn_right_2_valid) ||
            (turn_right_1_valid && turn_right_3_valid) ||
            (turn_right_2_valid && turn_right_3_valid)) {
            throw mapobjects::WaypointException::MULTIPLE_TURN_RIGHT_DETECTED;
        } else if (turn_right_1_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[0]);
            waypoint.lp = lp_out_1;
            waypoint.type = determineLanePointType(lp_out_1, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_1.p),
                getLanePointDirection(lp_out_1));
            waypoints.push_back(waypoint);
        } else if (turn_right_2_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[1]);
            waypoint.lp = lp_out_2;
            waypoint.type = determineLanePointType(lp_out_2, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_2.p),
                getLanePointDirection(lp_out_2));
            waypoints.push_back(waypoint);
        } else if (turn_right_3_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[2]);
            waypoint.lp = lp_out_3;
            waypoint.type = determineLanePointType(lp_out_3, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_3.p),
                getLanePointDirection(lp_out_3));
            waypoints.push_back(waypoint);
        } else {
            if (junction_debug) std::cout << "MapWaypoint: no turn right detected" << std::endl;
            throw mapobjects::WaypointException::NO_TURN_RIGHT_DETECTED;
        }
    } else {
        if (junction_debug) std::cout \
            << "MapWaypoint: more than 3 outgoing connections" << std::endl;
        throw mapobjects::WaypointException::MORE_THAN_3_OUTGOING_CONNECTIONS;
    }
    return waypoints;
}
mapobjects::GlobalWaypoint Map::getWaypointTurnRight(mapobjects::LanePoint3D lp) {
    return getWaypointsTurnRight(lp).back();
}
std::vector<mapobjects::GlobalWaypoint> Map::getWaypointsDriveStraight(mapobjects::LanePoint3D lp) {
    std::vector<mapobjects::GlobalWaypoint> waypoints = std::vector<mapobjects::GlobalWaypoint>();
    mapobjects::GlobalWaypoint waypoint = mapobjects::GlobalWaypoint(lp);
    if (junction_debug) std::cout \
        << "*** MapWaypoint: searching waypoint: drive straight ***" << std::endl;
    std::vector<mapobjects::Uuid> lane_connections_out = \
        getRealConnectionsOut(getLaneFromLaneUuid(lp.lane_uuid));
    int n_connections_out = lane_connections_out.size();
    if (junction_debug) std::cout << "MapWaypoint: outgoing connections: " \
        << n_connections_out << std::endl;
    if (n_connections_out < 1) {
        if (junction_debug) std::cout << "MapWaypoint: no outgoing connections" << std::endl;
        throw mapobjects::WaypointException::NO_OUTGOING_CONNECTIONS;
    } else if (n_connections_out == 1) {
        if (junction_debug) std::cout << "MapWaypoint: one outgoing connection" << std::endl;
        mapobjects::LanePoint3D lp_out = getFirstLanePoint(getNextLaneUuid(lp.lane_uuid));
        waypoint.lp = lp_out;
        waypoint.type = determineLanePointType(lp_out, false).type;
        waypoint.pose = mapobjects::Pose(
            getRelativePoint3D(lp_out.p),
            getLanePointDirection(lp_out));
        waypoints.push_back(waypoint);
    } else if (n_connections_out == 2) {
        if (junction_debug) std::cout << "MapWaypoint: 2 outgoing connections" << std::endl;
        double heading_junction_in = getLanePointDirection(lp);
        if (geometry_debug) drawLanePoint(lp, 2, true);
        mapobjects::LanePoint3D lp_out_1 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[0]));
        double heading_out_1 = getLanePointDirection(lp_out_1);
        bool drive_straight_1_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_1, 2, true);
        mapobjects::LanePoint3D lp_out_2 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[1]));
        double heading_out_2 = getLanePointDirection(lp_out_2);
        bool drive_straight_2_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_2, 2, true);
        double heading_to_out_1 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_1);
        double heading_to_out_2 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_2);
        if ((heading_to_out_1 > (JUNCTION_STRAIGHT_ANGLE - JUNCTION_STRAIGHT_MARGIN)) &&
            (heading_to_out_1 < (JUNCTION_STRAIGHT_ANGLE + JUNCTION_STRAIGHT_MARGIN))) {
            drive_straight_1_valid = true;
        }
        if ((heading_to_out_2 > (JUNCTION_STRAIGHT_ANGLE - JUNCTION_STRAIGHT_MARGIN)) &&
            (heading_to_out_2 < (JUNCTION_STRAIGHT_ANGLE + JUNCTION_STRAIGHT_MARGIN))) {
            drive_straight_2_valid = true;
        }

        if (junction_debug) {
            std::cout << "MapWaypoint: lp headings:\t1: " \
                << heading_junction_in << "\t2: " << heading_out_1 << "\t3: " \
                << heading_out_2 << std::endl;
            std::cout << "MapWaypoint: headings:\t1: " \
                << heading_to_out_1 << "\t2: " << heading_to_out_2 << std::endl;
            std::cout << "MapWaypoint: valid?:\t1: " \
                << drive_straight_1_valid << "\t2: " << drive_straight_2_valid << std::endl;
        }
        if (drive_straight_1_valid && drive_straight_2_valid) {
            throw mapobjects::WaypointException::MULTIPLE_DRIVE_STRAIGHT_DETECTED;
        } else if (drive_straight_1_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[0]);
            waypoint.lp = lp_out_1;
            waypoint.type = determineLanePointType(lp_out_1, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_1.p),
                getLanePointDirection(lp_out_1));
            waypoints.push_back(waypoint);
        } else if (drive_straight_2_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[1]);
            waypoint.lp = lp_out_2;
            waypoint.type = determineLanePointType(lp_out_2, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_2.p),
                getLanePointDirection(lp_out_2));
            waypoints.push_back(waypoint);
        } else {
            throw mapobjects::WaypointException::NO_DRIVE_STRAIGHT_DETECTED;
        }
    } else if (n_connections_out == 3) {
        if (junction_debug) std::cout << "MapWaypoint: 3 outgoing connections" << std::endl;
        double heading_junction_in = getLanePointDirection(lp);
        if (geometry_debug) drawLanePoint(lp, 2, true);
        mapobjects::LanePoint3D lp_out_1 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[0]));
        double heading_out_1 = getLanePointDirection(lp_out_1);
        bool drive_straight_1_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_1, 2, true);
        mapobjects::LanePoint3D lp_out_2 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[1]));
        double heading_out_2 = getLanePointDirection(lp_out_2);
        bool drive_straight_2_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_2, 2, true);
        mapobjects::LanePoint3D lp_out_3 = getFirstLanePoint(
            getNextLaneUuid(lane_connections_out[2]));
        double heading_out_3 = getLanePointDirection(lp_out_3);
        bool drive_straight_3_valid = false;
        if (geometry_debug) drawLanePoint(lp_out_3, 2, true);
        double heading_to_out_1 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_1);
        double heading_to_out_2 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_2);
        double heading_to_out_3 = getDirectionalAngularDifference(
            heading_junction_in, heading_out_3);
        if ((heading_to_out_1 > (JUNCTION_STRAIGHT_ANGLE - JUNCTION_STRAIGHT_MARGIN)) &&
            (heading_to_out_1 < (JUNCTION_STRAIGHT_ANGLE + JUNCTION_STRAIGHT_MARGIN))) {
            drive_straight_1_valid = true;
        }
        if ((heading_to_out_2 > (JUNCTION_STRAIGHT_ANGLE - JUNCTION_STRAIGHT_MARGIN)) &&
            (heading_to_out_2 < (JUNCTION_STRAIGHT_ANGLE + JUNCTION_STRAIGHT_MARGIN))) {
            drive_straight_2_valid = true;
        }
        if ((heading_to_out_3 > (JUNCTION_STRAIGHT_ANGLE - JUNCTION_STRAIGHT_MARGIN)) &&
            (heading_to_out_3 < (JUNCTION_STRAIGHT_ANGLE + JUNCTION_STRAIGHT_MARGIN))) {
            drive_straight_3_valid = true;
        }

        if (junction_debug) {
            std::cout << "MapWaypoint: lp headings:\t1: " \
                << heading_junction_in << "\t2: " << heading_out_1 << "\t3: " \
                << heading_out_2 << "\t4: " \
                << heading_out_3 << std::endl;
            std::cout << "MapWaypoint: headings:\t1: " \
                << heading_to_out_1 << "\t2: " << heading_to_out_2 << "\t3: " \
                << heading_to_out_3 << std::endl;
            std::cout << "MapWaypoint: valid?:\t1: " \
                << drive_straight_1_valid << "\t2: " << drive_straight_2_valid << "\t3: " \
                << drive_straight_3_valid << std::endl;
        }
        if ((drive_straight_1_valid && drive_straight_2_valid) ||
            (drive_straight_1_valid && drive_straight_3_valid) ||
            (drive_straight_2_valid && drive_straight_3_valid)) {
            throw mapobjects::WaypointException::MULTIPLE_DRIVE_STRAIGHT_DETECTED;
        } else if (drive_straight_1_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[0]);
            waypoint.lp = lp_out_1;
            waypoint.type = determineLanePointType(lp_out_1, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_1.p),
                getLanePointDirection(lp_out_1));
            waypoints.push_back(waypoint);
        } else if (drive_straight_2_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[1]);
            waypoint.lp = lp_out_2;
            waypoint.type = determineLanePointType(lp_out_2, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_2.p),
                getLanePointDirection(lp_out_2));
            waypoints.push_back(waypoint);
        } else if (drive_straight_3_valid) {
            waypoints = getInnerJunctionWaypoints(lane_connections_out[2]);
            waypoint.lp = lp_out_3;
            waypoint.type = determineLanePointType(lp_out_3, false).type;
            waypoint.pose = mapobjects::Pose(
                getRelativePoint3D(lp_out_3.p),
                getLanePointDirection(lp_out_3));
            waypoints.push_back(waypoint);
        } else {
            if (junction_debug) std::cout << "MapWaypoint: no drive straight detected" << std::endl;
            throw mapobjects::WaypointException::NO_DRIVE_STRAIGHT_DETECTED;
        }
    } else {
        if (junction_debug) std::cout \
            << "MapWaypoint: more than 3 outgoing connections" << std::endl;
        throw mapobjects::WaypointException::MORE_THAN_3_OUTGOING_CONNECTIONS;
    }
    return waypoints;
}
mapobjects::GlobalWaypoint Map::getWaypointDriveStraight(mapobjects::LanePoint3D lp) {
    return getWaypointsDriveStraight(lp).back();
}
std::vector<mapobjects::GlobalWaypoint> Map::getInnerJunctionWaypoints(
    mapobjects::Uuid junction_lane_uuid) {
    std::vector<mapobjects::GlobalWaypoint> waypoints = std::vector<mapobjects::GlobalWaypoint>();
    std::vector<mapobjects::Point3D> junction_points = getLanePoints(
        junction_lane_uuid);
    int n_points_junctionlane = junction_points.size();
    for (int i = 0; i < n_points_junctionlane; i++) {
        mapobjects::LanePoint3D lp = mapobjects::LanePoint3D(
            junction_points[i], i, junction_lane_uuid);
        mapobjects::GlobalWaypoint wp_i = mapobjects::GlobalWaypoint(
            lp,
            mapobjects::WaypointType::JUNCTION_ROAD);
        wp_i.pose = mapobjects::Pose(
            getRelativePoint3D(junction_points[i]),
            getLanePointDirection(lp));
        waypoints.push_back(wp_i);
    }
    return waypoints;
}
mapobjects::GlobalWaypoint Map::getWaypointParkOut(
    const mapobjects::Pose& pose,
    bool park_out_left,
    double waypoint_distance) {
    if (waypoint_debug) std::cout << "*** MapWaypoint: searching waypoint for park out ***" \
        << std::endl;
    std::vector<mapobjects::Uuid> parking_lane_uuids = std::vector<mapobjects::Uuid>();
    for (std::unordered_map<std::string, std::vector<mapobjects::Uuid>>::const_iterator it = \
        lane_parking_lookup.begin();
        it != lane_parking_lookup.end(); it++) {
        std::vector<mapobjects::Uuid> lg_lanes_parking_lane = getAllLanesFromLanegroup(
            getLaneGroupUuidFromLaneUuid(lane_storage[lane_uuids.at(it->first)].getUuid()));
        parking_lane_uuids.insert(
            parking_lane_uuids.end(),
            lg_lanes_parking_lane.begin(),
            lg_lanes_parking_lane.end());
    }

    int n_parking_lane_uuids = parking_lane_uuids.size();
    if (n_parking_lane_uuids < 1) {
        std::cout << "MapWaypoint: no parking lots in map" << std::endl;
        throw mapobjects::WaypointException::NO_PARKING_LOTS;
    }
    mapobjects::Point3D p_1 = mapobjects::Point3D(
        getAbsoluteX(pose.getX()),
        getAbsoluteY(pose.getY()),
        pose.getZ());
    double heading_car = normalizeToAtan2(pose.getT());
    double line_segment_length = 300;  // in cm
    mapobjects::Point3D p_2 = mapobjects::Point3D(
        p_1.getX() \
            + cos(heading_car) * line_segment_length,
        p_1.getY() \
            + sin(heading_car) * line_segment_length,
        p_1.getZ());
    if (geometry_debug) {
        drawPoint3D(p_1, 4, 6);
        drawPoint3D(p_2, 3, 4);
        drawLine(p_1, p_2, 2);
    }
    std::vector<int> lane_index = std::vector<int>();
    std::vector<int> lane_segment_start_p_index = std::vector<int>();
    std::vector<mapobjects::Point3D> p_projected = std::vector<mapobjects::Point3D>();
    for (int j = 0; j < n_parking_lane_uuids; j++) {
        std::vector<mapobjects::Point3D> points_lane = getLanePoints(parking_lane_uuids[j]);
        int n_points_lane = points_lane.size();
        for (int i = 0; i < (n_points_lane - 1); i++) {
            mapobjects::Point3D p_l_1 = points_lane[i];
            mapobjects::Point3D p_l_2 = points_lane[i+1];
            try {
                p_projected.push_back(getLaneSegmentIntersection(p_1, p_2, p_l_1, p_l_2));
                lane_segment_start_p_index.push_back(i);
                lane_index.push_back(j);
                if (geometry_debug) {
                    drawPoint3D(p_l_1, 2, 5);
                    drawPoint3D(p_l_2, 2, 5);
                }
            } catch (Map::GeometryException& e) {
            }
        }
    }
    int n_matches = lane_index.size();
    double distance_lowest = 1e9;
    double distance_second_lowest = 1e9;
    int lowest_index = 0;
    int second_lowest_index = 0;
    for (int i = 0; i < n_matches; i++) {
        double distance = p_1.computeDistance(p_projected[i]);
        if (distance < distance_lowest) {
            distance_second_lowest = distance_lowest;
            distance_lowest = distance;
            second_lowest_index = lowest_index;
            lowest_index = i;
        } else {
            if (distance < distance_second_lowest) {
                distance_second_lowest = distance;
                second_lowest_index = i;
            }
        }
    }
    mapobjects::LanePoint3D lp_park_out = mapobjects::LanePoint3D();
    if (n_matches < 1) {
        std::cout << "MapWaypoint: could not localize at parking lot" << std::endl;
        throw mapobjects::WaypointException::NOT_AT_PARKING;
    } else if (n_matches == 1) {
        std::cout << "MapWaypoint: WARNING: found only one lane to park out" << std::endl;
        lp_park_out.p = p_projected[lowest_index];
        lp_park_out.p_index = lane_segment_start_p_index[lowest_index];
        lp_park_out.isInterpolated = true;
        lp_park_out.lane_uuid = parking_lane_uuids[lane_index[lowest_index]];
        lp_park_out.lane_info = getLaneInfo(lp_park_out.lane_uuid);
        bool is_endpoint = false;
        if (lp_park_out.p_index == static_cast<int>((getLanePoints(
            parking_lane_uuids[
                lane_index[
                    lowest_index]]).size() - 1))) {
            is_endpoint = true;
        }
        lp_park_out.lp_type = determineLanePointType(lp_park_out, is_endpoint);
    } else if (n_matches > 1) {
        if (park_out_left) {
            lp_park_out.p = p_projected[second_lowest_index];
            lp_park_out.p_index = lane_segment_start_p_index[second_lowest_index];
            lp_park_out.isInterpolated = true;
            lp_park_out.lane_uuid = parking_lane_uuids[lane_index[second_lowest_index]];
            lp_park_out.lane_info = getLaneInfo(lp_park_out.lane_uuid);
            bool is_endpoint = false;
            if (lp_park_out.p_index == static_cast<int>((getLanePoints(
                parking_lane_uuids[
                    lane_index[
                        second_lowest_index]]).size() - 1))) {
                is_endpoint = true;
            }
            lp_park_out.lp_type = determineLanePointType(lp_park_out, is_endpoint);
        } else {
            lp_park_out.p = p_projected[lowest_index];
            lp_park_out.p_index = lane_segment_start_p_index[lowest_index];
            lp_park_out.isInterpolated = true;
            lp_park_out.lane_uuid = parking_lane_uuids[lane_index[lowest_index]];
            lp_park_out.lane_info = getLaneInfo(lp_park_out.lane_uuid);
            bool is_endpoint = false;
            if (lp_park_out.p_index == static_cast<int>((getLanePoints(
                parking_lane_uuids[
                    lane_index[
                        lowest_index]]).size() - 1))) {
                is_endpoint = true;
            }
            lp_park_out.lp_type = determineLanePointType(lp_park_out, is_endpoint);
        }
    }
    return getNextGlobalWaypoint(lp_park_out, waypoint_distance, false, false, true);
}
std::vector<mapobjects::Uuid> Map::planRoute(
    const mapobjects::LanePoint3D& lp_start,
    const mapobjects::LanePoint3D& lp_goal,
    double* _route_costs) {
    if (planning_debug) std::cout << "*** MapPlanner: searching route from start to goal lps ***"
        << std::endl;
    std::vector<mapobjects::Uuid> route_uuids = std::vector<mapobjects::Uuid>();
    std::vector<mapobjects::GlobalWaypoint> route_waypoints = \
        std::vector<mapobjects::GlobalWaypoint>();

    *_route_costs = 0;
    if (lp_start.lane_uuid.equals(lp_goal.lane_uuid)) {
        if (plan_generation_debug) std::cout
            << "MapPlanner: start and goal lie on the same lane" << std::endl;
        if (lp_start.p_index < lp_goal.p_index) {
            route_uuids = findRoute(lp_start, lp_goal, _route_costs);
        } else if (lp_start.p_index == lp_goal.p_index) {
            std::cout << lp_start.p.getX() << "  " << lp_start.p.getY() << "  " << lp_goal.p.getX() << "  " << lp_goal.p.getY() << std::endl;
            if (lp_start.isInterpolated || lp_goal.isInterpolated) {
                std::cout <<
                    "MapPlanner: ERROR: planRoute(): lp_start/lp_goal is interpolated. "
                    << "not implemented." << std::endl;
                throw mapobjects::PlannerException::NOT_IMPLEMENTED;
            }
            route_uuids = findRoute(lp_start, lp_goal, _route_costs);

        } else {
            std::vector<mapobjects::Uuid> successor_lane_uuids = getNextLaneUuids(
                lp_start.lane_uuid);
            int n_successor_lanes = successor_lane_uuids.size();
            if (n_successor_lanes < 1) {
                std::cout <<
                    "MapPlanner: ERROR: lp_start is after lp_goal on the same lane,"
                    << "but lane is deadend" << std::endl;
                throw mapobjects::PlannerException::START_LANE_IS_DEADEND;
            }
            std::vector<std::vector<mapobjects::Uuid>> routes_uuids =
                std::vector<std::vector<mapobjects::Uuid>>(n_successor_lanes);
            std::vector<double> routes_cost = std::vector<double>(n_successor_lanes);
            double costs_min = 1e9;
            int n_costs_min = 0;

            for (int i = 0; i < n_successor_lanes; i++) {
                routes_uuids[i] = findRoute(
                    getFirstLanePoint(successor_lane_uuids[i]), lp_goal, &routes_cost[i]);
                if (routes_cost[i] < costs_min) {
                    costs_min = routes_cost[i];
                    n_costs_min = i;
                }
                if (planning_debug) {
                    std::cout << "MapPlanner: checking route variant: " << i << std::endl;
                    std::cout << "MapPlanner: > costs: " << routes_cost[i] << std::endl;
                }
            }
            if (planning_debug) {
                std::cout << "MapPlanner: taking shortest variant: " << n_costs_min << std::endl;
            }
            route_uuids = routes_uuids[n_costs_min];
            *_route_costs = costs_min;
            route_uuids.insert(route_uuids.begin(), lp_start.lane_uuid);
        }
    } else {
        route_uuids = findRoute(lp_start, lp_goal, _route_costs);
    }

    if (planning_debug) {
        int n_lanes_route = route_uuids.size();
        std::cout << "MapPlanner: found route size: " << n_lanes_route << std::endl;
        std::cout << "MapPlanner: found route cost: " << *_route_costs << std::endl;
        if (plan_generation_debug) {
            for (int i = 0; i < n_lanes_route; i++) {
                std::cout << " > lane uuid: " << route_uuids[i].getUuidValue() << std::endl;
            }
        }
    }

    return route_uuids;
}
std::vector<mapobjects::GlobalWaypoint> Map::planWaypointRoute(
    const mapobjects::Pose& pose_start,
    const mapobjects::Pose& pose_goal,
    double waypoint_distance,
    bool ignoreStoplines,
    bool ignoreCrosswalks) {
    if (planning_debug) std::cout <<
        "*** MapPlanner: searching waypoint route from start to goal pose ***"
        << std::endl;

    mapobjects::LanePoint3D lp_start = getSearchStartLanePoint3D(pose_start);
    mapobjects::LanePoint3D lp_goal = getSearchStartLanePoint3D(pose_goal);
    std::vector<mapobjects::Uuid> route_uuids = std::vector<mapobjects::Uuid>();
    std::vector<mapobjects::GlobalWaypoint> route_waypoints = \
        std::vector<mapobjects::GlobalWaypoint>();

    double route_costs = 0;
    route_uuids = planRoute(
        lp_start,
        lp_goal,
        &route_costs);
    route_waypoints = getRouteWaypoints(
        route_uuids, lp_start, lp_goal, waypoint_distance, ignoreStoplines, ignoreCrosswalks);

    return route_waypoints;
}
mapobjects::ManeuverRoute Map::planManeuverRoute(
    const mapobjects::Pose& pose_start,
    const mapobjects::Pose& pose_goal) {
    if (planning_debug) std::cout <<
        "*** MapPlanner: searching maneuver route from start to goal pose ***" << std::endl;
    std::vector<mapobjects::Uuid> route_uuids = std::vector<mapobjects::Uuid>();
    std::vector<mapobjects::GlobalWaypoint> route_waypoints = \
        std::vector<mapobjects::GlobalWaypoint>();

    mapobjects::LanePoint3D lp_start = getSearchStartLanePoint3D(pose_start);
    mapobjects::LanePoint3D lp_goal = getSearchStartLanePoint3D(pose_goal);

    double route_costs = 0;
    route_uuids = planRoute(
        lp_start,
        lp_goal,
        &route_costs);

    std::vector<mapobjects::AADC_Maneuver> maneuvers_route = getRouteManeuvers(route_uuids);
    mapobjects::GlobalWaypoint wp_start = mapobjects::GlobalWaypoint(
        lp_start,
        mapobjects::WaypointType::PLAN_START,
        mapobjects::Pose(
            getRelativePoint3D(lp_start.p),
            getLanePointDirection(lp_start)));
    mapobjects::GlobalWaypoint wp_goal = mapobjects::GlobalWaypoint(
        lp_goal,
        mapobjects::WaypointType::PLAN_GOAL,
        mapobjects::Pose(
            getRelativePoint3D(lp_goal.p),
            getLanePointDirection(lp_goal)));
    mapobjects::ManeuverRoute route_maneuvers = mapobjects::ManeuverRoute(
        wp_start, wp_goal, maneuvers_route);

    return route_maneuvers;
}

double Map::getRouteCosts(const mapobjects::Pose& pose_start, const mapobjects::Pose& pose_goal) {
    std::vector<mapobjects::Uuid> route_uuids = std::vector<mapobjects::Uuid>();

    mapobjects::LanePoint3D lp_start = getSearchStartLanePoint3D(pose_start);
    mapobjects::LanePoint3D lp_goal = getSearchStartLanePoint3D(pose_goal);

    double route_costs = 0;
    route_uuids = planRoute(
        lp_start,
        lp_goal,
        &route_costs);
    return route_costs;
}
mapobjects::LanePoint3D Map::getSearchStartLanePoint3D(const mapobjects::Pose& pose_start) {
    mapobjects::Pose pose_start_abs = mapobjects::Pose(
        getAbsoluteX(pose_start.getX()),
        getAbsoluteY(pose_start.getY()),
        pose_start.getZ(),
        pose_start.getT());
    mapobjects::LanePoint3D lp_start = getNearestLanePoint(pose_start_abs);
    if (planning_debug) drawLanePoint(lp_start, 4, false, true);
    return lp_start;
}
mapobjects::LanePoint3D Map::getSearchGoalLanePoint3D(const mapobjects::Pose& pose_goal) {
    mapobjects::Pose pose_goal_abs = mapobjects::Pose(
        getAbsoluteX(pose_goal.getX()),
        getAbsoluteY(pose_goal.getY()),
        pose_goal.getZ(),
        pose_goal.getT());
    mapobjects::LanePoint3D lp_goal = getNearestLanePoint(pose_goal_abs);
    if (planning_debug) drawLanePoint(lp_goal, 4, false, true);
    return lp_goal;
}
std::vector<mapobjects::Uuid> Map::findRoute(
    const mapobjects::LanePoint3D& lp_start,
    const mapobjects::LanePoint3D& lp_goal,
    double* _costs) {

    *_costs = 1e9;
    if (planner_debug) {
        std::cout << "MapAStar: starting search" << std::endl;
    }
    map_priority_queue<
        mapobjects::LaneNode,
        std::vector<mapobjects::LaneNode>,
        std::greater<std::vector<mapobjects::LaneNode>::value_type>> open_nodes;

    std::vector<mapobjects::LaneNode> processed_nodes = std::vector<mapobjects::LaneNode>();

    std::vector<mapobjects::Uuid> path_uuids = std::vector<mapobjects::Uuid>();
    mapobjects::Uuid goal_uuid = lp_goal.lane_uuid;
    mapobjects::LaneNode start_node = mapobjects::LaneNode(
        lp_start.lane_uuid, mapobjects::Uuid("startnode"), 0, 0,
        getRealConnectionsOut(getLaneFromLaneUuid(lp_start.lane_uuid)));
    open_nodes.push(start_node);
    if (planner_debug) std::cout << "MapAStar: > pushing start node" << std::endl;
    while (open_nodes.size() > 0) {
        mapobjects::LaneNode current_node = open_nodes.top();
        open_nodes.pop();
        processed_nodes.push_back(current_node);
        if (planner_debug) std::cout << "MapAStar: > expanding top node" << std::endl;
        if (goalReached(current_node, goal_uuid)) {
            if (planner_debug) std::cout << "MapAStar: goal reached! backtracking" << std::endl;
            *_costs = current_node.g;
            while (current_node.parent_uuid.getUuidValue() != "startnode") {
                path_uuids.push_back(current_node.lane_uuid);
                if (planner_debug) std::cout <<
                    "MapAStar: > backtracking: adding parent node to path" << std::endl;
                current_node = getNodeFromUuid(current_node.parent_uuid, processed_nodes);
            }
            if (planner_debug) std::cout <<
                "MapAStar: > backtracking: adding start node to path" << std::endl;
            path_uuids.push_back(start_node.lane_uuid);
            std::reverse(path_uuids.begin(), path_uuids.end());
            break;
        }
        for (const mapobjects::Uuid& successor_uuid : current_node.successors()) {
            if (planner_debug) std::cout << "MapAStar:  > processing successor node" << std::endl;
            bool successor_processed = false;
            int n_processed_nodes = processed_nodes.size();
            for (int i = 0; i < n_processed_nodes; i++) {
                if (processed_nodes[i].lane_uuid == successor_uuid) {
                    successor_processed = true;
                    break;
                }
            }
            if (successor_processed) {
                if (planner_debug) std::cout <<
                    "MapAStar:   > successor node already processed. continuing." << std::endl;
                continue;
            }

            double g = current_node.g + \
                getLanePoints(current_node.lane_uuid).back().computeDistance(
                    getLanePoints(successor_uuid).back());
            const mapobjects::LaneNode successor_node = mapobjects::LaneNode(
                successor_uuid, current_node.lane_uuid, g, h_function(successor_uuid, lp_goal),
                getRealConnectionsOut(getLaneFromLaneUuid(successor_uuid)));
            std::vector<mapobjects::LaneNode>::const_iterator it = open_nodes.find(successor_node);
            if (it != open_nodes.end()) {
                if (successor_node.g > it->g) {
                    continue;
                }
            }
            open_nodes.push(successor_node);
            if (planner_debug) std::cout <<
                "MapAStar:   > adding successor node to open list" << std::endl;
        }
    }

    if (path_uuids.size() < 1) {
        std::cout << "MapAStar: ERROR: could not find a path to the goal" << std::endl;
        throw mapobjects::PlannerException::NO_GOAL_FOUND;
    }
    return path_uuids;
}
std::vector<mapobjects::GlobalWaypoint> Map::getRouteWaypoints(
    const std::vector<mapobjects::Uuid>& route_uuids,
    mapobjects::LanePoint3D lp_start,
    mapobjects::LanePoint3D lp_goal,
    double waypoint_distance,
    bool ignoreStoplines,
    bool ignoreCrosswalks) {
    std::vector<mapobjects::GlobalWaypoint> waypoints = std::vector<mapobjects::GlobalWaypoint>();

    int n_lanes_route = route_uuids.size();
    if (planning_debug) {
        std::cout << "MapPlanner: generating waypoints from path"
            << std::endl;
        std::cout << "MapPlanner: n lanes on path: " << n_lanes_route
            << std::endl;
    }

    std::vector<mapobjects::Uuid> processed_lanes = std::vector<mapobjects::Uuid>();
    processed_lanes.push_back(lp_start.lane_uuid);

    if (n_lanes_route > 1) {
        waypoints.push_back(
            mapobjects::GlobalWaypoint(
                lp_start,
                mapobjects::WaypointType::PLAN_START,
                mapobjects::Pose(
                    getRelativePoint3D(lp_start.p),
                    getLanePointDirection(lp_start))));
        mapobjects::LanePoint3D lp = lp_start;
        mapobjects::LanePoint3D lp_prev = lp_start;

        if (plan_generation_debug)
            std::cout << "current lane id " << lp.lane_uuid.getUuidValue() << std::endl;

        int i = 0;
        double offset_goal = getOffsetAlongLane(lp_goal);
        double offset_current = 0;
        bool goal_reached = false;

        while (i < n_lanes_route) {
            while (lp.lp_type.type != mapobjects::WaypointType::JUNCTION) {
                mapobjects::GlobalWaypoint wp_next = getNextGlobalWaypoint(
                    lp, waypoint_distance, ignoreStoplines, ignoreCrosswalks, true);
                lp_prev = lp;
                lp = wp_next.lp;
                if (plan_generation_debug)
                    std::cout << "current lane id " << lp.lane_uuid.getUuidValue() << std::endl;
                if (lp.lane_uuid.equals(lp_goal.lane_uuid) && i > 0) {
                    if (plan_generation_debug) std::cout << "goal check" << std::endl;

                    offset_current = getOffsetAlongLane(lp);
                    if (offset_current < offset_goal) {
                        if (plan_generation_debug)
                            std::cout << "MapPlanner: offset not exceeded" << std::endl;

                        waypoints.push_back(wp_next);
                        continue;
                    } else {
                        if (plan_generation_debug)
                            std::cout << "MapPlanner: offset exceeded. adding goal" << std::endl;
                        waypoints.push_back(mapobjects::GlobalWaypoint(
                            lp_goal,
                            mapobjects::WaypointType::PLAN_GOAL,
                            mapobjects::Pose(
                                getRelativePoint3D(lp_goal.p),
                                getLanePointDirection(lp_goal))));
                        i = (n_lanes_route - 1);
                        goal_reached = true;
                        break;
                    }
                }
                if (!lp.lane_uuid.equals(route_uuids[i])) {
                    if (plan_generation_debug)
                        std::cout << "MapPlanner: lane uuid has changed" << std::endl;
                    if (i < (n_lanes_route - 1)) {
                        if (lp.lane_uuid.equals(route_uuids[i + 1])) {
                            i++;
                            processed_lanes.push_back(lp.lane_uuid);
                            if (plan_generation_debug)
                                std::cout << "MapPlanner: new lane is next lane" << std::endl;
                        } else {
                            if (plan_generation_debug)
                                std::cout << "MapPlanner: new lane is other lane" << std::endl;
                            std::vector<mapobjects::Uuid>::const_iterator it = std::find(
                                route_uuids.begin(), route_uuids.end(), lp.lane_uuid);
                            int i_new = it - route_uuids.begin();
                            if (it != route_uuids.end()) {
                                if (plan_generation_debug)
                                    std::cout << "new lane is in lane path" << std::endl;
                                if (std::find(
                                    processed_lanes.begin(), processed_lanes.end(), lp.lane_uuid)
                                    == processed_lanes.end()) {
                                    waypoints.push_back(wp_next);
                                    i = i_new;
                                    processed_lanes.push_back(lp.lane_uuid);

                                    if (plan_generation_debug) std::cout <<
                                        "MapPlanner: lane uuid in route: " << i << std::endl;

                                    continue;
                                }
                            }
                            if (plan_generation_debug)
                                std::cout << "MapPlanner: uuid not in route or already processed. "
                                << "returning goal point" << std::endl;
                            processed_lanes.push_back(lp.lane_uuid);
                            waypoints.push_back(mapobjects::GlobalWaypoint(
                                lp_goal,
                                mapobjects::WaypointType::PLAN_GOAL,
                                mapobjects::Pose(
                                    getRelativePoint3D(lp_goal.p),
                                    getLanePointDirection(lp_goal))));
                            i = n_lanes_route;
                            goal_reached = true;
                            break;
                        }
                    } else {
                        if (plan_generation_debug)
                            std::cout << "MapPlanner: goal lane exceeded. returning goal point"
                            << std::endl;
                        processed_lanes.push_back(lp.lane_uuid);
                        waypoints.push_back(mapobjects::GlobalWaypoint(
                            lp_goal,
                            mapobjects::WaypointType::PLAN_GOAL,
                            mapobjects::Pose(
                                getRelativePoint3D(lp_goal.p),
                                getLanePointDirection(lp_goal))));
                        i = n_lanes_route;
                        goal_reached = true;
                        break;
                    }
                }
                waypoints.push_back(wp_next);
            }
            if ((!(i < (n_lanes_route - 1))) || goal_reached) {
                if (plan_generation_debug)
                    std::cout << "MapPlanner: outer goal check true" << std::endl;
                break;
            }
            i++;
            if (plan_generation_debug)
                std::cout << "MapPlanner: adding junction lane waypoints" << std::endl;
            std::vector<mapobjects::GlobalWaypoint> wps = getInnerJunctionWaypoints(route_uuids[i]);
            waypoints.insert(waypoints.end(), wps.begin(), wps.end());
            i++;
            lp = getFirstLanePoint(route_uuids[i]);
        }
    } else {
        if (!lp_start.lane_uuid.equals(lp_goal.lane_uuid)) {
            std::cout <<
                "MapPlanner: ERROR: route has length 1 && lp_start and lp_goal"
                << " are on different lanes" << std::endl;
            throw mapobjects::PlannerException::PATH_INCONSISTENT;
        }
        std::vector<mapobjects::GlobalWaypoint> wps = getLaneWaypoints(
            route_uuids[0],
            &lp_start, &lp_goal,
            true, true,
            waypoint_distance, ignoreStoplines, ignoreCrosswalks);
        waypoints.insert(waypoints.end(), wps.begin(), wps.end());
        return waypoints;
    }

    if (planning_debug) std::cout << "MapPlanner: returning from waypoint generation" << std::endl;
    return waypoints;
}

std::vector<mapobjects::GlobalWaypoint> Map::getLaneWaypoints(
    mapobjects::Uuid lane_uuid,
    mapobjects::LanePoint3D* lp_start,
    mapobjects::LanePoint3D* lp_goal,
    bool isStart,
    bool isGoal,
    double waypoint_distance,
    bool ignoreStoplines,
    bool ignoreCrosswalks) {
    std::vector<mapobjects::GlobalWaypoint> waypoints = std::vector<mapobjects::GlobalWaypoint>();
    std::cout << "    getting lane waypoints" << std::endl;
    if (!isStart && !isGoal) {
        if (!getLaneFromLaneUuid(lane_uuid).getVisibility()) {
            std::vector<mapobjects::Point3D> lane_points = getLanePoints(lane_uuid);
            return getInnerJunctionWaypoints(lane_uuid);
        } else {
            mapobjects::LanePoint3D lp = mapobjects::LanePoint3D(
                getLanePoints(lane_uuid)[0], 0, lane_uuid);
            lp.lp_type = determineLanePointType(lp, false);
            lp.lane_info = getLaneInfo(lane_uuid);
            waypoints.push_back(mapobjects::GlobalWaypoint(
                    lp,
                    lp.lp_type.type,
                    mapobjects::Pose(
                        getRelativePoint3D(lp.p),
                        getLanePointDirection(lp))));
            while (lp.lp_type.type != mapobjects::WaypointType::JUNCTION) {
                mapobjects::GlobalWaypoint wp_next = getNextGlobalWaypoint(
                    lp, waypoint_distance, ignoreStoplines, ignoreCrosswalks, true);
                waypoints.push_back(wp_next);
                lp = wp_next.lp;
            }
            std::cout << "waypoints size: " << waypoints.size() << std::endl;
            return waypoints;
        }

    } else if (isStart && !isGoal) {
        if (lp_start == nullptr) {
            std::cout <<
                "MapPlanner: ERROR: getLaneWaypoints() was called with nullptr and invalid flag"
                << std::endl;
            throw mapobjects::PlannerException::LP_IS_NULLPTR;
        }
        mapobjects::LanePoint3D lp = *lp_start;
        waypoints.push_back(mapobjects::GlobalWaypoint(
            lp,
            mapobjects::WaypointType::PLAN_START,
            mapobjects::Pose(
                getRelativePoint3D(lp.p),
                getLanePointDirection(lp))));
        while (lp.lp_type.type != mapobjects::WaypointType::JUNCTION) {
            mapobjects::GlobalWaypoint wp_next = getNextGlobalWaypoint(
                lp, waypoint_distance, ignoreStoplines, ignoreCrosswalks, true);
            waypoints.push_back(wp_next);
            lp = wp_next.lp;
        }
        return waypoints;

    } else if (!isStart && isGoal) {
        if (lp_goal == nullptr) {
            std::cout <<
                "MapPlanner: ERROR: getLaneWaypoints() was called with nullptr and invalid flag"
                << std::endl;
            throw mapobjects::PlannerException::LP_IS_NULLPTR;
        }
        double offset_goal = getOffsetAlongLane(*lp_goal);
        mapobjects::LanePoint3D lp = mapobjects::LanePoint3D(
            getLanePoints(lane_uuid)[0], 0, lane_uuid);
        lp.lp_type = determineLanePointType(lp, false);
        lp.lane_info = getLaneInfo(lane_uuid);
        waypoints.push_back(
            mapobjects::GlobalWaypoint(
                lp, lp.lp_type.type, mapobjects::Pose(
                    getRelativePoint3D(lp.p),
                    getLanePointDirection(lp))));
        while (getOffsetAlongLane(lp) < (offset_goal - waypoint_distance)) {
            mapobjects::GlobalWaypoint wp_next = getNextGlobalWaypoint(
                lp, waypoint_distance, ignoreStoplines, ignoreCrosswalks, true);
            waypoints.push_back(wp_next);
            lp = wp_next.lp;
        }
        waypoints.push_back(
            mapobjects::GlobalWaypoint(
                *lp_goal,
                mapobjects::WaypointType::PLAN_GOAL,
                mapobjects::Pose(
                    getRelativePoint3D(lp_goal->p),
                    getLanePointDirection(*lp_goal))));
        return waypoints;

    } else if (isStart && isGoal) {
        double offset_start = getOffsetAlongLane(*lp_start);
        double offset_goal = getOffsetAlongLane(*lp_goal);
        if (offset_goal < offset_start) {
            std::cout <<
                "MapPlanner: ERROR: getLaneWaypoints(): goal lp is before start lp"
                << std::endl;
            throw mapobjects::PlannerException::GOAL_LP_BEHIND_START_LP;
        } else {
            mapobjects::LanePoint3D lp = *lp_start;
            waypoints.push_back(
                mapobjects::GlobalWaypoint(
                    *lp_start,
                    mapobjects::WaypointType::PLAN_START,
                    mapobjects::Pose(
                        getRelativePoint3D(lp_start->p),
                        getLanePointDirection(*lp_start))));
            while (lp.lp_type.type != mapobjects::WaypointType::JUNCTION) {
                mapobjects::GlobalWaypoint wp_next = getNextGlobalWaypoint(
                    lp, waypoint_distance, ignoreStoplines, ignoreCrosswalks, true);
                if (getOffsetAlongLane(wp_next.lp) < offset_goal) {
                    waypoints.push_back(wp_next);
                    lp = wp_next.lp;
                } else {
                    break;
                }
            }
            waypoints.push_back(
                mapobjects::GlobalWaypoint(
                    *lp_goal,
                    mapobjects::WaypointType::PLAN_GOAL,
                    mapobjects::Pose(
                        getRelativePoint3D(lp_goal->p),
                        getLanePointDirection(*lp_goal))));
            return waypoints;
        }
    }
    return waypoints;
}
std::vector<mapobjects::AADC_Maneuver> Map::getRouteManeuvers(
    const std::vector<mapobjects::Uuid>& route_uuids) {
    std::vector<mapobjects::AADC_Maneuver> man_ids = std::vector<mapobjects::AADC_Maneuver>();
    int n_lanes_route = route_uuids.size();
    for (int i = 0; i < n_lanes_route; i++) {
        if (i == 0) {
            std::vector<mapobjects::Uuid> l_connections_in = getRealConnectionsIn(
                getLaneFromLaneUuid(route_uuids[i]));
            if (static_cast<int>(l_connections_in.size()) > 0) {
                if (getRealConnectionsOut(getLaneFromLaneUuid(l_connections_in[0])).size() > 1) {
                    mapobjects::AADC_Maneuver man = getJunctionLaneTurnType((route_uuids[i]));
                    man_ids.push_back(man);
                }
            }
        }
        if (checkSuccessorIsJunction(getLaneFromLaneUuid(route_uuids[i]))) {
            try {
                if ((i + 1) < n_lanes_route) {
                    if (plan_generation_debug) std::cout << "successor is junction, id: "
                        << route_uuids[i + 1].getUuidValue() << std::endl;
                    mapobjects::AADC_Maneuver man = getJunctionLaneTurnType((route_uuids[i + 1]));
                    man_ids.push_back(man);
                }
            } catch (mapobjects::WaypointException e) {
                std::cout <<"MapPlanner: ERROR: getRouteManeuvers(): "
                << "junction turn type could not be resolved" << std::endl;
            }
        }
    }
    return man_ids;
}

mapobjects::LaneNode Map::getNodeFromUuid(const mapobjects::Uuid& current_node_uuid,
    const std::vector<mapobjects::LaneNode>& nodes) {
    int n_nodes = nodes.size();

    for (int i = 0; i < n_nodes; i++) {
        if (nodes[i].lane_uuid.equals(current_node_uuid)) {
            return nodes[i];
        }
    }
    return mapobjects::LaneNode(
        mapobjects::Uuid("error"), mapobjects::Uuid("not found"),
        0, 0, std::vector<mapobjects::Uuid>());
}
bool Map::goalReached(const mapobjects::LaneNode& node_1, const mapobjects::Uuid& goal_uuid) {
    if (node_1.lane_uuid.equals(goal_uuid)) {
        return true;
    }
    return false;
}
double Map::h_function(const mapobjects::Uuid& lane_uuid, const mapobjects::LanePoint3D& lp_goal) {
    return getLanePoints(lane_uuid).back().computeDistance(lp_goal.p);
}
mapobjects::AADC_Maneuver Map::getJunctionLaneTurnType(const mapobjects::Uuid& lane_uuid) {
    mapobjects::LanePoint3D lp_in = mapobjects::LanePoint3D(
        getLanePoints(lane_uuid)[0],
        0,
        lane_uuid,
        mapobjects::WaypointType::JUNCTION_ROAD);
    mapobjects::LanePoint3D lp_out = getFirstLanePoint(
        getNextLaneUuid(lane_uuid));
    double heading_junction_in = getLanePointDirection(lp_in);
    double heading_junction_out = getLanePointDirection(lp_out);

    double heading_in_to_out = getDirectionalAngularDifference(
            heading_junction_in, heading_junction_out);

    bool turn_left_valid = false;
    bool turn_right_valid = false;
    bool drive_straight_valid = false;

    if (geometry_debug)
        std::cout << "junction headings:\nin: "
        << heading_junction_in << "\nout: " << heading_junction_out << std::endl;

    if ((heading_in_to_out > (JUNCTION_LEFT_ANGLE - JUNCTION_LEFT_MARGIN)) &&
        (heading_in_to_out < (JUNCTION_LEFT_ANGLE + JUNCTION_LEFT_MARGIN))) {
        turn_left_valid = true;
    }
    if ((heading_in_to_out > (JUNCTION_RIGHT_ANGLE - JUNCTION_RIGHT_MARGIN)) &&
        (heading_in_to_out < (JUNCTION_RIGHT_ANGLE + JUNCTION_RIGHT_MARGIN))) {
        turn_right_valid = true;
    }
    if ((heading_in_to_out > (JUNCTION_STRAIGHT_ANGLE - JUNCTION_STRAIGHT_MARGIN)) &&
        (heading_in_to_out < (JUNCTION_STRAIGHT_ANGLE + JUNCTION_STRAIGHT_MARGIN))) {
        drive_straight_valid = true;
    }

    if (turn_left_valid) {
        if (turn_right_valid || drive_straight_valid) {
            throw mapobjects::WaypointException::MULTIPLE_TURN_TYPES_DETECTED;
        } else {
            return mapobjects::AADC_Maneuver(mapobjects::AADC_Maneuvers::TURN_LEFT);
        }
    }
    if (turn_right_valid) {
        if (turn_left_valid || drive_straight_valid) {
            throw mapobjects::WaypointException::MULTIPLE_TURN_TYPES_DETECTED;
        } else {
            return mapobjects::AADC_Maneuver(mapobjects::AADC_Maneuvers::TURN_RIGHT);
        }
    }
    if (drive_straight_valid) {
        if (turn_left_valid || turn_right_valid) {
            throw mapobjects::WaypointException::MULTIPLE_TURN_TYPES_DETECTED;
        } else {
            return mapobjects::AADC_Maneuver(mapobjects::AADC_Maneuvers::DRIVE_STRAIGHT);
        }
    }
    return mapobjects::AADC_Maneuver(mapobjects::AADC_Maneuvers::NOT_SPECIFIED);
}
mapobjects::LanePoint3D Map::getNearestLanePoint(mapobjects::Pose pose) {
    mapobjects::Point3D p_pose = mapobjects::Point3D(pose.getX(), pose.getY(), pose.getZ());

    if (lanepoint_debug) std::cout << "MapLanepoint: searching nearest waypoint..." << std::endl;
    if (lanepoint_search_debug) std::cout << "MapLanepoint: current position / cm (!absolute!)" \
        << pose.getX() << "   " << pose.getY() << std::endl;
    int n_elements_lanes = lane_storage.size();
    if (lanepoint_search_debug) std::cout << "MapLanepoint: found " \
        << n_elements_lanes << " lanes to index" << std::endl;

    mapobjects::Uuid nearest_lane_uuid;
    mapobjects::LanePoint3D lp_nearest = mapobjects::LanePoint3D();
    bool is_endpoint = false;
    bool lp_found = false;
    double distance = 1e9;
    for (int i = 0; i < n_elements_lanes; i++) {
        if (lanepoint_search_debug) std::cout << "MapLanepoint: indexing lane " << i << std::endl;
        is_endpoint = false;
        if (!lane_storage[i].getVisibility()) {
            if (lanepoint_search_debug) std::cout \
                << "MapLanepoint: lane is junction. skipping." << std::endl;
            continue;
        }
        std::vector<mapobjects::Point3D> points_lane = lane_storage[i].getPoints();
        int n_points_lane = points_lane.size();
        if (lanepoint_search_debug) std::cout << "MapLanepoint: found " \
            << n_points_lane << " points in lane" << std::endl;
        for (int j = 0; j < n_points_lane; j++) {
            if (lanepoint_search_debug) std::cout \
                << "MapLanepoint: point position / cm (absolute): " \
                << points_lane[j].getX() << "   " << points_lane[j].getY() << std::endl;
            double lp_distance = p_pose.computeDistance(points_lane[j]);
            if (lanepoint_search_debug) std::cout << "MapLanepoint: distance measure / cm: " \
                << lp_distance << std::endl;
            if (lp_distance < distance) {
                lp_found = true;
                if (lanepoint_search_debug) std::cout \
                    << "MapLanepoint: lanepoint is nearer. updating." << std::endl;
                distance = lp_distance;
                lp_nearest = mapobjects::LanePoint3D(
                    points_lane[j], j, lane_storage[i].getUuid());
                if (j == n_points_lane - 1) {
                    is_endpoint = true;
                }
            }
        }
    }
    if (!lp_found) {
        throw mapobjects::WaypointException::COULD_NOT_LOCALIZE;
    }
    if (!isEqualCarAndLanePointDirection(pose, lp_nearest)) {
        if (lanepoint_debug) std::cout << "swapping lanepoints on lanegroup" << std::endl;
        is_endpoint = false;
        mapobjects::LanePoint3D lp_opposite = getNearestOppositeLanePoint(lp_nearest);
        lp_opposite.isOppositeLane = true;
        if (lp_opposite.p_index == static_cast<int>(
            getLanePoints(lp_opposite.lane_uuid).size() - 1)) {
            is_endpoint = true;
        }
        lp_opposite.lp_type = determineLanePointType(lp_opposite, is_endpoint);
        return lp_opposite;
    }
    lp_nearest.lp_type = determineLanePointType(lp_nearest, is_endpoint);
    lp_nearest.lane_info = getLaneInfo(lp_nearest.lane_uuid);
    return lp_nearest;
}
mapobjects::Uuid Map::getLaneGroupUuidFromLaneUuid(mapobjects::Uuid lane_uuid) {
    std::string lane_id = lane_uuid.getUuidValue();
    mapobjects::Uuid lg_uuid = mapobjects::Uuid();
    int n_elements_lanegroups = lanegroup_storage.size();
    for (int i = 0; i < n_elements_lanegroups; i++) {
        std::vector<mapobjects::Uuid> lanes_l_ids = lanegroup_storage[i].getLanesLeft();
        int n_elements_lanes_l = lanes_l_ids.size();
        for (int j = 0; j < n_elements_lanes_l; j++) {
            if (lanes_l_ids[j].getUuidValue() == lane_id) {
                lg_uuid = lanegroup_storage[i].getUuid();
            }
        }
        std::vector<mapobjects::Uuid> lanes_r_ids = lanegroup_storage[i].getLanesRight();
        int n_elements_lanes_r = lanes_r_ids.size();
        for (int j = 0; j < n_elements_lanes_r; j++) {
            if (lanes_r_ids[j].getUuidValue() == lane_id) {
                lg_uuid = lanegroup_storage[i].getUuid();
            }
        }
    }
    return lg_uuid;
}
double Map::getClosestLaneHeading(mapobjects::LanePoint3D lp, mapobjects::Pose pose) {
    double heading_target = pose.getT();
    std::vector<mapobjects::Point3D> lane_points = \
        getLaneFromLaneUuid(lp.lane_uuid).getPoints();
    double heading_to_predecessor = heading_target;
    double heading_to_successor = heading_target;
    int n_points = lane_points.size();
    if (n_points > 1) {
        if (lp.p_index > 0) {
            heading_to_predecessor = lp.p.computeHeadingToPoint(lane_points[lp.p_index - 1]);
        } else {
            heading_to_predecessor = lp.p.computeHeadingToPoint(lane_points[lp.p_index + 1]) + M_PI;
        }
        if (lp.p_index < (n_points - 1)) {
            heading_to_successor = lp.p.computeHeadingToPoint(lane_points[lp.p_index + 1]);
        } else {
            heading_to_successor = lp.p.computeHeadingToPoint(lane_points[lp.p_index - 1]) - M_PI;
        }
        double heading_predecessor_diff = fabs(
            fmod(heading_target, 2 * M_PI) - fmod(heading_to_predecessor, 2 * M_PI));
        double heading_successor_diff = fabs(
            fmod(heading_target, 2 * M_PI) - fmod(heading_to_successor, 2 * M_PI));
        if (heading_predecessor_diff < heading_successor_diff) {
            return heading_to_predecessor;
        } else {
        }
    }
    return heading_target;
}
bool Map::isEqualCarAndLanePointDirection(mapobjects::Pose pose, mapobjects::LanePoint3D lp) {
    mapobjects::LanePoint3D lp_opposite = getNearestOppositeLanePoint(lp);
    double heading_car = normalizeToAtan2(pose.getT());
    double heading_lane = getLanePointDirection(lp);
    double heading_lane_opposite = getLanePointDirection(lp_opposite);

    if (lanepoint_debug) std::cout << "Direction comparison:\nheading_car:\t" << heading_car \
        << "\nheading_lp:\t" << heading_lane << "\nheading_lane_opposite:\t" \
        << heading_lane_opposite << std::endl;

    double heading_relative_to_lane = fabs(getDirectionalAngularDifference(
        heading_car, heading_lane));
    double heading_relative_to_lane_opposite = fabs(getDirectionalAngularDifference(
        heading_car, heading_lane_opposite));

    if (lanepoint_debug) std::cout << "heading to lane:\t" << heading_relative_to_lane \
        << "\nheading to opposite lane:\t" << heading_relative_to_lane_opposite << std::endl;

    if (heading_relative_to_lane_opposite < heading_relative_to_lane) {
        if (lanepoint_debug) std::cout << "heading to current lane larger than to opposite" \
            << std::endl;
        return false;
    }
    if (lanepoint_debug) std::cout << "heading to current lane smaller" << std::endl;/*  */
    return true;
}
double Map::getAngularDifferenceFromAtan2(double heading_car, double heading_lane) {
    double angular_difference = 0;
    if (heading_car >= 0) {
        if (heading_lane > 0) {
            angular_difference = fabs(heading_car - heading_lane);
        } else {
            angular_difference = heading_car - heading_lane;
            if (angular_difference > M_PI) angular_difference = (2 * M_PI) - angular_difference;
        }
    } else {
        if (heading_lane > 0) {
            angular_difference = fabs(heading_car - heading_lane);
        } else {
            angular_difference = fabs(heading_car) + heading_lane;
            if (angular_difference > M_PI) angular_difference = (2 * M_PI) - angular_difference;
        }
    }
    return angular_difference;
}
double Map::getDirectionalAngularDifference(double heading_car, double heading_lane) {
    heading_car = normalizeToAtan2(heading_car);
    heading_lane = normalizeToAtan2(heading_lane);
    double angular_difference = 0;

    if (heading_car >= 0) {
        if (heading_lane >= 0) {
            angular_difference = heading_lane - heading_car;
        } else {
            angular_difference = heading_car - heading_lane;
            if (angular_difference < M_PI) {
                angular_difference *= -1;
            } else {
                angular_difference = (2 * M_PI) - angular_difference;
            }
        }
    } else {
        if (heading_lane >= 0) {
            angular_difference = heading_lane - heading_car;
            if (angular_difference > M_PI) {
                angular_difference -= (2 * M_PI);
            }
        } else {
            angular_difference = heading_lane - heading_car;
        }
    }
    return angular_difference;
}
double Map::normalizeToAtan2(double heading) {
    heading = fmod(heading, (2 * M_PI));
    if (heading > M_PI) {
        heading -= (2 * M_PI);
    } else if (heading <= -M_PI) {
        heading += (2 * M_PI);
    }
    return heading;
}
double Map::getLanePointDirection(mapobjects::LanePoint3D lp) {
    double heading = 0;
    std::vector<mapobjects::Point3D> lane_points = \
        getLaneFromLaneUuid(lp.lane_uuid).getPoints();
    if (lp.p_index < static_cast<int>(lane_points.size() - 1)) {
        heading = lp.p.computeHeadingToPoint(lane_points[lp.p_index + 1]);
    } else {
        if (lp.p_index > 0) {
            heading = lp.p.computeHeadingToPoint(lane_points[lp.p_index - 1]) + M_PI;
        } else {
            std::cout << "WARNING: something went wrong during lanepoint direction computation" \
                << std::endl;
            heading = -1000.0;
            return heading;
        }
    }
    return normalizeToAtan2(heading);
}
mapobjects::Uuid Map::getOppositeLaneUuid(mapobjects::Uuid lane_uuid) {
    mapobjects::LaneGroup lg = lanegroup_storage[
        lanegroup_uuids.at(getLaneGroupUuidFromLaneUuid(lane_uuid).getUuidValue())];
    std::vector<mapobjects::Uuid> lanes_r = lg.getLanesRight();
    int n_lanes_l = lg.getLanesLeft().size();
    int n_lanes_r = lg.getLanesRight().size();
    bool on_right_lane = false;
    for (int i = 0; i < n_lanes_r; i++) {
        if (lanes_r[i].getUuidValue() == lane_uuid.getUuidValue()) {
            on_right_lane = true;
            break;
        }
    }
    if (on_right_lane) {
        if (n_lanes_l > 0) {
            return lg.getLanesLeft()[0];
        } else {
            return lane_uuid;
        }
    } else {
        if (n_lanes_r > 0) {
            return lg.getLanesRight()[0];
        } else {
            return lane_uuid;
        }
    }
}
mapobjects::LanePoint3D Map::getNearestOppositeLanePoint(mapobjects::LanePoint3D lp) {
    mapobjects::LanePoint3D lp_opposite = lp;
    mapobjects::Lane lane_opposite = \
        getLaneFromLaneUuid(getOppositeLaneUuid(lp.lane_uuid));

    std::vector<mapobjects::Point3D> points_opposite = lane_opposite.getPoints();
    int n_points_opposite = points_opposite.size();
    int index_nearest_point = -1;
    double distance = 1e9;
    for (int i = 0; i < n_points_opposite; i++) {
        double distance_current = lp.p.computeDistance(points_opposite[i]);
        if (distance_current < distance) {
            distance = distance_current;
            index_nearest_point = i;
        }
    }
    lp_opposite = mapobjects::LanePoint3D(
        points_opposite[index_nearest_point], index_nearest_point, lane_opposite.getUuid());
    lp_opposite.lane_info = getLaneInfo(lp_opposite.lane_uuid);
    return lp_opposite;
}
mapobjects::LanePoint3D Map::getNextLanePoint(mapobjects::LanePoint3D lp, double min_distance) {
    if (lanepoint_debug2) std::cout << "MapLanePoint: searching next lanepoint..." << std::endl;
    mapobjects::Lane lane = getLaneFromLaneUuid(lp.lane_uuid);
    std::vector<mapobjects::Point3D> lane_points = lane.getPoints();
    int n_lanepoints = lane_points.size();

    mapobjects::LanePoint3D lp_next = mapobjects::LanePoint3D(
        lp.p, lp.p_index, lp.lane_uuid);
    if (lp.p_index < (n_lanepoints - 1)) {
        lp_next.p_index++;

        if (lanepoint_debug2) std::cout << "MapLanePoint: incrementing point index:\t\t" \
            << (lp.p_index) << "-> new: " << lp_next.p_index << std::endl;

        if (!lp.isInterpolated) {
            lp_next.lp_type = determineLanePointType(lp_next, false);
        } else {
            lp_next.lp_type = determineLanePointType(lp, lp_next, false);
        }
        lp_next.p = lane_points[lp_next.p_index];
    } else {
        if (lanepoint_debug) std::cout << "MapLanePoint: reached end of lane" << std::endl;
        if (lp.lp_type.type == mapobjects::WaypointType::ROAD) {
            if (lanepoint_debug2) std::cout \
                << "MapLanePoint: getting first point on next lane" << std::endl;
            lp_next = getFirstLanePoint(getNextLaneUuid(lane));
        } else {
            if (getRealConnectionsOut(getLaneFromLaneUuid(lp_next.lane_uuid)).size() > 0) {
                lp_next.lp_type = mapobjects::WaypointTypeHeader(mapobjects::WaypointType::JUNCTION);
            } else {
                lp_next.lp_type = mapobjects::WaypointTypeHeader(mapobjects::WaypointType::DEADEND);
            }
            
        }
    }
    lp_next.lane_info = getLaneInfo(lp_next.lane_uuid);
    return lp_next;
}
std::vector<mapobjects::Point3D> Map::getLanePoints(const mapobjects::Uuid& lane_uuid) const {
    return lane_storage[lane_uuids.at(lane_uuid.getUuidValue())].getPoints();
}
mapobjects::Lane Map::getLaneFromLaneUuid(const mapobjects::Uuid& lane_uuid) const {
    return lane_storage[lane_uuids.at(lane_uuid.getUuidValue())];
}
std::vector<mapobjects::Uuid> Map::getRealConnectionsOut(const mapobjects::Lane& lane) {
    std::vector<mapobjects::Uuid> l_connections_out = lane.getConnectionsOut();
    int n_connections_out = l_connections_out.size();
    mapobjects::LaneGroup lg_current = lanegroup_storage[
        lanegroup_uuids.at(getLaneGroupUuidFromLaneUuid(lane.getUuid()).getUuidValue())];
    std::vector<mapobjects::Uuid> lg_lanes_left_ids = lg_current.getLanesLeft();
    std::vector<mapobjects::Uuid> lg_lanes_right_ids = lg_current.getLanesRight();
    std::vector<mapobjects::Uuid> lg_lanes_ids;
    lg_lanes_ids.insert(lg_lanes_ids.end(), lg_lanes_left_ids.begin(), lg_lanes_left_ids.end());
    lg_lanes_ids.insert(lg_lanes_ids.end(), lg_lanes_right_ids.begin(), lg_lanes_right_ids.end());
    int n_lg_lanes = lg_lanes_ids.size();
    std::vector<mapobjects::Uuid> l_real_connections_out = std::vector<mapobjects::Uuid>();
    for (int i = 0; i < n_connections_out; i++) {
        bool match_found = false;
        for (int j = 0; j < n_lg_lanes; j++) {
            if (l_connections_out[i].equals(lg_lanes_ids[j])) {
                match_found = true;
            }
        }
        if (!match_found) {
            l_real_connections_out.push_back(l_connections_out[i]);
        }
    }
    return l_real_connections_out;
}
std::vector<mapobjects::Uuid> Map::getRealConnectionsIn(const mapobjects::Lane& lane) {
    std::vector<mapobjects::Uuid> l_connections_in = lane.getConnectionsIn();
    int n_connections_in = l_connections_in.size();
    mapobjects::LaneGroup lg_current = lanegroup_storage[
        lanegroup_uuids.at(getLaneGroupUuidFromLaneUuid(lane.getUuid()).getUuidValue())];
    std::vector<mapobjects::Uuid> lg_lanes_left_ids = lg_current.getLanesLeft();
    std::vector<mapobjects::Uuid> lg_lanes_right_ids = lg_current.getLanesRight();
    std::vector<mapobjects::Uuid> lg_lanes_ids;
    lg_lanes_ids.insert(lg_lanes_ids.end(), lg_lanes_left_ids.begin(), lg_lanes_left_ids.end());
    lg_lanes_ids.insert(lg_lanes_ids.end(), lg_lanes_right_ids.begin(), lg_lanes_right_ids.end());
    int n_lg_lanes = lg_lanes_ids.size();
    std::vector<mapobjects::Uuid> l_real_connections_in = std::vector<mapobjects::Uuid>();
    for (int i = 0; i < n_connections_in; i++) {
        bool match_found = false;
        for (int j = 0; j < n_lg_lanes; j++) {
            if (l_connections_in[i].equals(lg_lanes_ids[j])) {
                match_found = true;
            }
        }
        if (!match_found) {
            l_real_connections_in.push_back(l_connections_in[i]);
        }
    }
    return l_real_connections_in;
}

bool Map::checkSuccessorIsJunction(const mapobjects::Lane& lane) {
    if (static_cast<int>(getRealConnectionsOut(lane).size()) > 1) {
        return true;
    }
    return false;
}

bool Map::checkLaneIsDeadend(const mapobjects::Lane& lane) {
    if (static_cast<int>(getRealConnectionsOut(lane).size()) < 1) {
        return true;
    }
    return false;
}
mapobjects::WaypointTypeHeader Map::determineLanePointType(
    const mapobjects::LanePoint3D& lp, bool is_endpoint) {
    mapobjects::Lane lane = getLaneFromLaneUuid(lp.lane_uuid);
    mapobjects::WaypointTypeHeader lp_type;

    if (lanepoint_debug) std::cout << "MapLanePoint: determining lp type" << std::endl;
    std::vector<mapobjects::Uuid> lane_stoplines = getLaneStoplines(lp.lane_uuid);
    int n_lane_stoplines = lane_stoplines.size();
    if (n_lane_stoplines > 0) {
        if (lanepoint_debug) std::cout << "checking for stoplines" << std::endl;
        for (int i = 0; i < n_lane_stoplines; i++) {
            double stopline_offset = round(laneobject_storage[
                laneobject_uuids.at(lane_stoplines[i].getUuidValue())]->getOffset() * 1000) / 1000;
            double offset_lp_prev = \
                round(getOffsetAlongLane(getPreviousLanePoint(lp)) * 1000) / 1000;
            double offset_lp_curr = \
                round(getOffsetAlongLane(lp) * 1000) / 1000;

            if (lanepoint_debug2) {
                std::cout << "offset prev: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << offset_lp_prev << std::endl;
                std::cout << "offset curr: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << offset_lp_curr << std::endl;
                std::cout << "offset stopline: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << stopline_offset << std::endl;
            }
            if (lp.p_index > 0) {
                if (stopline_offset > offset_lp_prev && stopline_offset <= offset_lp_curr) {
                    lp_type.type = mapobjects::WaypointType::STOPLINE;
                    lp_type.offset = stopline_offset;

                    if (lanepoint_debug) std::cout \
                        << "MapLanePointType: lp is stopline" << std::endl;
                    return lp_type;
                }
            } else {
                if (stopline_offset >= offset_lp_prev && stopline_offset <= offset_lp_curr) {
                    lp_type.type = mapobjects::WaypointType::STOPLINE;
                    lp_type.offset = stopline_offset;

                    if (lanepoint_debug) std::cout \
                        << "MapLanePointType: lp is stopline" << std::endl;
                    return lp_type;
                }
            }
        }
    }
    std::vector<mapobjects::Uuid> lane_crosswalks = getLaneCrosswalks(lp.lane_uuid);
    int n_lane_crosswalks = lane_crosswalks.size();
    if (n_lane_crosswalks > 0) {
        if (lanepoint_debug) std::cout << "checking for crosswalks" << std::endl;
        for (int i = 0; i < n_lane_crosswalks; i++) {
            double crosswalk_offset = round(getCrosswalkOffset(
                lane_crosswalks[i],
                lp.lane_uuid) * 1000) / 1000;
            double offset_lp_prev = \
                round(getOffsetAlongLane(getPreviousLanePoint(lp)) * 1000) / 1000;
            double offset_lp_curr = \
                round(getOffsetAlongLane(lp) * 1000) / 1000;

            if (lanepoint_debug2) {
                std::cout << "offset prev: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << offset_lp_prev << std::endl;
                std::cout << "offset curr: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << offset_lp_curr << std::endl;
                std::cout << "offset crosswalk: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << crosswalk_offset << std::endl;
            }
            if (lp.p_index > 0) {
                if (crosswalk_offset > offset_lp_prev && crosswalk_offset <= offset_lp_curr) {
                    lp_type.type = mapobjects::WaypointType::CROSSWALK;
                    lp_type.offset = crosswalk_offset;

                    if (lanepoint_debug) std::cout \
                        << "MapLanePointType: lp is crosswalk" << std::endl;
                    return lp_type;
                }
            } else {
                if (crosswalk_offset >= offset_lp_prev && crosswalk_offset <= offset_lp_curr) {
                    lp_type.type = mapobjects::WaypointType::CROSSWALK;
                    lp_type.offset = crosswalk_offset;

                    if (lanepoint_debug) std::cout \
                        << "MapLanePointType: lp is crosswalk" << std::endl;
                    return lp_type;
                }
            }
        }
    }
    if (is_endpoint || lp.p_index == static_cast<int>(lane.getPoints().size() - 1)) {
        if (lanepoint_debug) std::cout << "MapLanePointType: lanepoint marked to be endpoint" \
            << std::endl;

        std::vector<mapobjects::Uuid> connections_out = getRealConnectionsOut(lane);
        int n_connections_out = connections_out.size();
        if (n_connections_out > 1) {
            if (waypoint_debug) std::cout << "MapLanePointType: lp is junction" << std::endl;

            lp_type.type = mapobjects::WaypointType::JUNCTION;
            return lp_type;
        } else if (n_connections_out < 1) {
            if (lanepoint_debug) std::cout << "MapLanePointType: lp is deadend" << std::endl;

            lp_type.type = mapobjects::WaypointType::DEADEND;
            return lp_type;
        }
    }

    if (lanepoint_debug) std::cout << "MapLanePointType: lp is road" << std::endl;

    lp_type.type = mapobjects::WaypointType::ROAD;
    return lp_type;
}
mapobjects::WaypointTypeHeader Map::determineLanePointType(
    const mapobjects::LanePoint3D& lp_prev, const mapobjects::LanePoint3D& lp,
    bool is_endpoint) {
    mapobjects::WaypointTypeHeader lp_type;
    if (lanepoint_debug) std::cout << "determining lp type from interpolated" << std::endl;
    if (!lp_prev.lane_uuid.equals(lp.lane_uuid)) {
        std::cout << "MapDetermineLanePointType: ERROR: lanepoints have different lane uuids!" \
            << std::endl;
        lp_type.type = mapobjects::WaypointType::ERROR;
        return lp_type;
    }
    mapobjects::Lane lane = getLaneFromLaneUuid(lp.lane_uuid);
    std::vector<mapobjects::Uuid> lane_stoplines = getLaneStoplines(lp.lane_uuid);
    int n_lane_stoplines = lane_stoplines.size();
    if (n_lane_stoplines > 0) {
        if (lanepoint_debug) std::cout << "checking for stoplines" << std::endl;
        for (int i = 0; i < n_lane_stoplines; i++) {
            double stopline_offset = round(laneobject_storage[
                laneobject_uuids.at(lane_stoplines[i].getUuidValue())]->getOffset() * 1000) / 1000;
            double offset_lp_prev = \
                round(getOffsetAlongLane(lp_prev) * 1000) / 1000;
            double offset_lp_curr = \
                round(getOffsetAlongLane(lp) * 1000) / 1000;
            if (lanepoint_debug2) {
                std::cout << "offset prev (interpolated): " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << offset_lp_prev << std::endl;
                std::cout << "offset curr: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << offset_lp_curr << std::endl;
                std::cout << "offset stopline: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << stopline_offset << std::endl;
            }
            if ((stopline_offset > (offset_lp_prev + STOPLINE_MIN_SEPARATOR)) \
                && (stopline_offset <= offset_lp_curr)) {
                lp_type.type = mapobjects::WaypointType::STOPLINE;
                lp_type.offset = stopline_offset;

                if (lanepoint_debug) std::cout \
                    << "MapLanePointType: lp is stopline" << std::endl;
                return lp_type;
            }
        }
    }
    std::vector<mapobjects::Uuid> lane_crosswalks = getLaneCrosswalks(lp.lane_uuid);
    int n_lane_crosswalks = lane_crosswalks.size();
    if (n_lane_crosswalks > 0) {
        if (lanepoint_debug) std::cout << "checking for crosswalks" << std::endl;
        for (int i = 0; i < n_lane_crosswalks; i++) {
            double crosswalk_offset = round(getCrosswalkOffset(
                lane_crosswalks[i],
                lp.lane_uuid) * 1000) / 1000;
            double offset_lp_prev = \
                round(getOffsetAlongLane(lp_prev) * 1000) / 1000;
            double offset_lp_curr = \
                round(getOffsetAlongLane(lp) * 1000) / 1000;

            if (lanepoint_debug2) {
                std::cout << "offset prev (interpolated): " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << offset_lp_prev << std::endl;
                std::cout << "offset curr: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << offset_lp_curr << std::endl;
                std::cout << "offset crosswalk: " \
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1) \
                    << crosswalk_offset << std::endl;
            }
            if (lp.p_index > 0) {
                if (crosswalk_offset > offset_lp_prev && crosswalk_offset <= offset_lp_curr) {
                    lp_type.type = mapobjects::WaypointType::CROSSWALK;
                    lp_type.offset = crosswalk_offset;

                    if (lanepoint_debug) std::cout \
                        << "MapLanePointType: lp is crosswalk" << std::endl;
                    return lp_type;
                }
            } else {
                if (crosswalk_offset >= offset_lp_prev && crosswalk_offset <= offset_lp_curr) {
                    lp_type.type = mapobjects::WaypointType::CROSSWALK;
                    lp_type.offset = crosswalk_offset;

                    if (lanepoint_debug) std::cout \
                        << "MapLanePointType: lp is crosswalk" << std::endl;
                    return lp_type;
                }
            }
        }
    }
    if (is_endpoint || lp.p_index == static_cast<int>(lane.getPoints().size() - 1)) {
        if (lanepoint_debug) std::cout << "MapLanePointType: lanepoint marked to be endpoint" \
            << std::endl;

        std::vector<mapobjects::Uuid> connections_out = getRealConnectionsOut(lane);
        int n_connections_out = connections_out.size();
        if (n_connections_out > 1) {
            if (waypoint_debug) std::cout << "MapLanePointType: lp is junction" << std::endl;

            lp_type.type = mapobjects::WaypointType::JUNCTION;
            return lp_type;
        } else if (n_connections_out < 1) {
            if (lanepoint_debug) std::cout << "MapLanePointType: lp is deadend" << std::endl;

            lp_type.type = mapobjects::WaypointType::DEADEND;
            return lp_type;
        }
    }

    if (lanepoint_debug) std::cout << "MapLanePointType: lp is road" << std::endl;

    lp_type.type = mapobjects::WaypointType::ROAD;
    return lp_type;
}
mapobjects::LanePoint3D Map::getPreviousLanePoint(const mapobjects::LanePoint3D& lp) {
    std::vector<mapobjects::Point3D> points_lane = getLanePoints(lp.lane_uuid);
    if (lanepoint_debug) std::cout << "MapLanePoint: getting previous lp" << std::endl;

    if (lp.p_index > 0) {
        mapobjects::LanePoint3D lp_prev = mapobjects::LanePoint3D();
        lp_prev.lane_uuid = lp.lane_uuid;
        lp_prev.p_index = lp.p_index - 1;

        mapobjects::Point3D p_prev = points_lane[lp.p_index-1];
        if (lp.isInterpolated) {
            p_prev = points_lane[lp.p_index];
            lp_prev.p_index = lp.p_index;
        }
        lp_prev.p = p_prev;
        return lp_prev;
    } else {
        return lp;
    }
}
mapobjects::LanePoint3D Map::getFirstLanePoint(const mapobjects::Uuid& lane_uuid) {
    if (lanepoint_debug) std::cout << "MapLanePoint: getting first lanepoint on new lane" \
        << std::endl;

    std::vector<mapobjects::Point3D> lane_points = getLanePoints(lane_uuid);
    mapobjects::LanePoint3D lp = mapobjects::LanePoint3D();
    if (lane_points.size() > 0) {
        lp.p_index = 0;
        lp.lane_uuid = lane_uuid;
        lp.p = lane_points[lp.p_index];
        lp.lp_type = determineLanePointType(lp, false);
    } else {
        lp.lp_type.type = mapobjects::WaypointType::ERROR;
    }
    lp.lane_info = getLaneInfo(lp.lane_uuid);
    return lp;
}
mapobjects::Uuid Map::getNextLaneUuid(const mapobjects::Uuid& lane_uuid) {
    return getNextLaneUuids(lane_uuid)[0];
}
mapobjects::Uuid Map::getNextLaneUuid(const mapobjects::Lane& lane) {
    return getNextLaneUuids(lane)[0];
}
std::vector<mapobjects::Uuid> Map::getNextLaneUuids(const mapobjects::Uuid& lane_uuid) {
    return getRealConnectionsOut(getLaneFromLaneUuid(lane_uuid));
}
std::vector<mapobjects::Uuid> Map::getNextLaneUuids(const mapobjects::Lane& lane) {
    return getRealConnectionsOut(lane);
}
std::vector<mapobjects::Uuid> Map::getLaneStoplines(const mapobjects::Uuid& lane_uuid) {
    std::unordered_map<std::string, std::vector<mapobjects::Uuid>>::const_iterator it = \
        lane_stoplines_lookup.find(lane_uuid.getUuidValue());
    if (it != lane_stoplines_lookup.end()) {
        return it->second;
    }
    return std::vector<mapobjects::Uuid>();
}
std::vector<mapobjects::Uuid> Map::getLaneCrosswalks(const mapobjects::Uuid& lane_uuid) {
    std::unordered_map<std::string, std::vector<mapobjects::Uuid>>::const_iterator it = \
        lane_crosswalks_lookup.find(lane_uuid.getUuidValue());
    if (it != lane_crosswalks_lookup.end()) {
        return it->second;
    }
    return std::vector<mapobjects::Uuid>();
}
std::vector<mapobjects::Uuid> Map::getLaneParkings(const mapobjects::Uuid& lane_uuid) {
    std::unordered_map<std::string, std::vector<mapobjects::Uuid>>::const_iterator it = \
        lane_parking_lookup.find(lane_uuid.getUuidValue());
    if (it != lane_parking_lookup.end()) {
        return it->second;
    }
    return std::vector<mapobjects::Uuid>();
}

std::vector<mapobjects::Uuid> Map::getAllParkingLots() {
    std::vector<mapobjects::Uuid> parking_uuids = std::vector<mapobjects::Uuid>();
    for (std::unordered_map<std::string, std::vector<mapobjects::Uuid>>::const_iterator it = \
        lane_parking_lookup.begin();
        it != lane_parking_lookup.end(); it++) {
        int n_parkings = it->second.size();
        for (int i = 0; i < n_parkings; i++) {
            parking_uuids.push_back(it->second[i]);
        }
    }
    return parking_uuids;
}
bool Map::containsUuid(std::vector<mapobjects::Uuid> uuids, mapobjects::Uuid uuid) {
    int n_uuids = uuids.size();
    for (int i = 0; i < n_uuids; i++) {
        if (uuids[i].equals(uuid)) {
            return true;
        }
    }
    return false;
}
std::vector<mapobjects::Uuid> Map::getAllLanesFromLanegroup(mapobjects::Uuid lg_uuid) {
    std::vector<mapobjects::Uuid> lg_lanes_uuids = std::vector<mapobjects::Uuid>();

    std::vector<mapobjects::Uuid> lg_lanes_left = lanegroup_storage[
        lanegroup_uuids.at(lg_uuid.getUuidValue())].getLanesLeft();
    int n_lanes_left = lg_lanes_left.size();
    for (int i = 0; i < n_lanes_left; i++) {
        lg_lanes_uuids.push_back(lg_lanes_left[i]);
    }
    std::vector<mapobjects::Uuid> lg_lanes_right = lanegroup_storage[
        lanegroup_uuids.at(lg_uuid.getUuidValue())].getLanesRight();
    int n_lanes_right = lg_lanes_right.size();
    for (int i = 0; i < n_lanes_right; i++) {
        lg_lanes_uuids.push_back(lg_lanes_right[i]);
    }
    return lg_lanes_uuids;
}
mapobjects::LaneInfo Map::getLaneInfo(mapobjects::Uuid lane_uuid) {
    const mapobjects::Lane& lane = getLaneFromLaneUuid(lane_uuid);
    mapobjects::LaneInfo lane_info = mapobjects::LaneInfo();
    int height = lane.getHeight();
    switch (height) {
        case 1:
            lane_info.is_ground = false;
            break;
        case 2:
            lane_info.is_ramp_up = true;
            break;
        case 3:
            lane_info.is_ramp_down = true;
            break;
        case 4:
            lane_info.is_merging = true;
            break;
        case 5:
            lane_info.is_tunnel = true;
            break;
        default:
            break;
    }
    if (lane_crosswalks_lookup.find(lane_uuid.getUuidValue()) \
        != lane_crosswalks_lookup.end()) {
        lane_info.has_crosswalks = true;
    }
    if (lane_stoplines_lookup.find(lane_uuid.getUuidValue()) \
        != lane_stoplines_lookup.end()) {
        lane_info.has_stoplines = true;
    }
    if (lane_parking_lookup.find(lane_uuid.getUuidValue()) \
        != lane_parking_lookup.end()) {
        lane_info.has_parking = true;
    }
    mapobjects::LaneMarkingType lmt = lanemarking_storage[
            lanemarking_uuids.at(
                lane.getLaneMarkingContainer().getLaneMarkingsLeft()[
                    0].getUuidValue())].getLaneMarkingType();
    if ((lmt == mapobjects::LaneMarkingType::CENTER_SOLID) \
         || (lmt == mapobjects::LaneMarkingType::SOLID)) {
        lane_info.no_overtaking = true;
    }
    return lane_info;
}
double Map::getOffsetAlongLane(const mapobjects::LanePoint3D& lp) {
    std::vector<mapobjects::Point3D> points = getLanePoints(lp.lane_uuid);
    double offset = 0;

    for (int i = 0; i < lp.p_index; i++) {
        offset += points[i].computeDistance(points[i+1]);
    }
    if (lp.isInterpolated) {
        offset += points[lp.p_index].computeDistance(lp.p);
    }
    return offset;
}
double Map::getOffsetToPoint3D(mapobjects::LanePoint3D lp, mapobjects::Point3D p) {
    return lp.p.computeDistance(p);
}
mapobjects::Point3D Map::getPoint3DFromLaneOffset(
    const mapobjects::LanePoint3D& lp, const double& offset) {
    std::vector<mapobjects::Point3D> points_lane = getLanePoints(lp.lane_uuid);
    int n_points = points_lane.size();
    double lane_offset = 0;
    int lane_position = 0;
    for (int i = 0; i < (n_points - 1); i++) {
        lane_position = i;
        double delta_offset = points_lane[i].computeDistance(points_lane[i+1]);
        if (lane_offset + delta_offset > offset) {
            break;
        } else {
            lane_offset += points_lane[i].computeDistance(points_lane[i+1]);
        }
    }
    double lane_segment_heading = points_lane[lane_position].computeHeadingToPoint(
        points_lane[lane_position+1]);
    double offset_delta = offset - lane_offset;  // positive
    double lane_segment_length = \
        points_lane[lane_position].computeDistance(points_lane[lane_position+1]);
    double x_interpolated = \
        points_lane[lane_position].getX() + cos(lane_segment_heading) * offset_delta;
    double y_interpolated = \
        points_lane[lane_position].getY() + sin(lane_segment_heading) * offset_delta;
    double z_interpolated = 0;
    if (points_lane[lane_position].getZ() > 0 && points_lane[lane_position+1].getZ() > 0) {
        z_interpolated = points_lane[lane_position].getZ() + (
            points_lane[lane_position+1].getZ() - points_lane[lane_position].getZ()) * (
                offset_delta / lane_segment_length);
    }
    return mapobjects::Point3D(x_interpolated, y_interpolated, z_interpolated);
}
mapobjects::Point3D Map::getPoint3DFromLanePointOffset(
    const mapobjects::LanePoint3D& lp, const double& offset) {
    double lane_segment_heading = getLanePointDirection(lp);
    double x_interpolated = \
        lp.p.getX() + cos(lane_segment_heading) * offset;
    double y_interpolated = \
        lp.p.getY() + sin(lane_segment_heading) * offset;
    double z_interpolated = lp.p.getZ();
    return mapobjects::Point3D(x_interpolated, y_interpolated, z_interpolated);
}
mapobjects::Point3D Map::getLaneSegmentIntersection(
    mapobjects::Point3D lane1_p1,
    mapobjects::Point3D lane1_p2,
    mapobjects::Point3D lane2_p1,
    mapobjects::Point3D lane2_p2) {
    double p_1_x = lane1_p1.getX();
    double p_1_y = lane1_p1.getY();
    double p_2_x = lane1_p2.getX();
    double p_2_y = lane1_p2.getY();
    double r_x = p_2_x - p_1_x;
    double r_y = p_2_y - p_1_y;
    double q_1_x = lane2_p1.getX();
    double q_1_y = lane2_p1.getY();
    double q_2_x = lane2_p2.getX();
    double q_2_y = lane2_p2.getY();
    double s_x = q_2_x - q_1_x;
    double s_y = q_2_y - q_1_y;
    double num1 = crossProduct2D((q_1_x - p_1_x), (q_1_y - p_1_y), r_x, r_y);
    double num2 = crossProduct2D((q_1_x - p_1_x), (q_1_y - p_1_y), s_x, s_y);
    double den = crossProduct2D(r_x, r_y, s_x, s_y);

    double x_new = p_1_x;
    double y_new = p_1_y;

    if (num2 == 0 && den == 0) {
        if (geometry_debug) std::cout << "MapGeometry: ERROR: lanesegments collinear" << std::endl;
        throw Map::GeometryException::COLLINEAR_ERROR;
    } else if (den == 0 && num2 != 0) {
        if (geometry_debug) std::cout << "MapGeometry: ERROR: lanesegments parallel" << std::endl;
        throw Map::GeometryException::PARALLEL_ERROR;
    } else {
        double t = num1 / den;
        double u = num2 / den;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            x_new = p_1_x + u * r_x;
            y_new = p_1_y + u * r_y;
        } else {
            if (geometry_debug) std::cout \
                << "MapGeometry: ERROR: no intersection between lane segments" << std::endl;
            throw Map::GeometryException::NOT_INTERSECTING_ERROR;
        }
    }
    return mapobjects::Point3D(x_new, y_new, 0);
}
double Map::crossProduct2D(double x1, double y1, double x2, double y2) {
    return (x1*y2) - (y1*x2);
}
double Map::getCrosswalkOffset(mapobjects::Uuid crosswalk_uuid, mapobjects::Uuid lane_uuid) {
    double crosswalk_lg_center_offset = laneobject_storage[
        laneobject_uuids.at(crosswalk_uuid.getUuidValue())]->getOffset();
    mapobjects::LaneGroup crosswalk_lg = lanegroup_storage[
        lanegroup_uuids.at(
            laneobject_storage[
                laneobject_uuids.at(
                    crosswalk_uuid.getUuidValue())]->getLaneGroupId().getUuidValue())];

    if (crosswalk_lg.getLanesRight().size() > 0) {
        mapobjects::Uuid right_lane_uuid = crosswalk_lg.getLanesRight()[0];
        std::vector<mapobjects::Point3D> left_marking_points = lanemarking_storage[
            lanemarking_uuids.at(
                lane_storage[
                    lane_uuids.at(right_lane_uuid.getUuidValue())
                ].getLaneMarkingContainer().getLaneMarkingsLeft()[0].getUuidValue())
        ].getPoints();
        int n_lanemarking_points = left_marking_points.size();
        int p_index_start = 0;
        int p_index_end = 0;
        int offset_distance = 0;
        for (int i = 0; i < (n_lanemarking_points - 1); i++) {
            offset_distance += left_marking_points[i].computeDistance(left_marking_points[i+1]);
            if (round(offset_distance * 1000) / 1000 >= \
                round(crosswalk_lg_center_offset * 1000) / 1000) {
                p_index_start = i;
                p_index_end = i + 1;
                break;
            }
        }
        double point_segment_heading = left_marking_points[p_index_start].computeHeadingToPoint(
            left_marking_points[p_index_end]);
        double offset_delta = offset_distance - crosswalk_lg_center_offset;
        mapobjects::Point3D p_1 = mapobjects::Point3D(
            left_marking_points[p_index_start].getX() + cos(point_segment_heading) * offset_delta,
            left_marking_points[p_index_start].getY() + sin(point_segment_heading) * offset_delta,
            left_marking_points[p_index_start].getZ());
        point_segment_heading = normalizeToAtan2(point_segment_heading + M_PI_2);
        double line_segment_length = 100;
        mapobjects::Point3D p_2 = mapobjects::Point3D(
            p_1.getX() \
                + cos(point_segment_heading) * line_segment_length,
            p_1.getY() \
                + sin(point_segment_heading) * line_segment_length,
            p_1.getZ());
        p_1 = mapobjects::Point3D(
            p_1.getX() + cos(point_segment_heading) * -line_segment_length,
            p_1.getY() + sin(point_segment_heading) * -line_segment_length,
            p_1.getZ());
        if (geometry_debug) {
            drawPoint3D(p_1, 3, 5);
            drawPoint3D(p_2, 3, 5);
            drawLine(p_1, p_2, 2);
        }
        std::vector<mapobjects::Point3D> points_lane = getLanePoints(lane_uuid);
        int n_points_lane = points_lane.size();
        mapobjects::Point3D p_projected;
        bool intersection_found = false;
        int lane_segment_start_p_index = 0;

        for (int i = 0; i < (n_points_lane - 1); i++) {
            mapobjects::Point3D p_l_1 = points_lane[i];
            mapobjects::Point3D p_l_2 = points_lane[i+1];
            try {
                p_projected = getLaneSegmentIntersection(p_1, p_2, p_l_1, p_l_2);
                intersection_found = true;
                lane_segment_start_p_index = i;
                if (geometry_debug) {
                    drawPoint3D(p_l_1, 2, 5);
                    drawPoint3D(p_l_2, 2, 5);
                }
                break;
            } catch (Map::GeometryException& e) {
            }
        }
        if (!intersection_found) {
            std::cout << \
                "MapWaypoint: ERROR: could not compute projected Point3D for crosswalk position" \
                << std::endl;
            return 1e9;
        }
        mapobjects::LanePoint3D lp_crosswalk = mapobjects::LanePoint3D();
        lp_crosswalk.p_index = lane_segment_start_p_index;
        lp_crosswalk.lp_type = mapobjects::WaypointTypeHeader(
            mapobjects::WaypointType::INTERPOLATED);
        lp_crosswalk.isInterpolated = true;
        lp_crosswalk.lane_uuid = lane_uuid;
        lp_crosswalk.p = p_projected;
        double crosswalk_offset = getOffsetAlongLane(lp_crosswalk);

        if (geometry_debug) \
            drawPoint3D(getPoint3DFromLaneOffset(lp_crosswalk, crosswalk_offset), 4, 25);
        return crosswalk_offset;
    }
    std::cout << "MapWaypoint: ERROR: no right lane for crosswalk found" << std::endl;
    return 1e9;
}
mapobjects::LanePoint3D Map::projectPoint3DToLane(
    mapobjects::Point3D p, mapobjects::Uuid lane_uuid) {
    std::vector<mapobjects::Point3D> lane_points = getLanePoints(lane_uuid);
    int n_lane_points = lane_points.size();

    int p_index_distance_min = 0;
    double distance_min = 1e9;
    for (int i = 0; i < n_lane_points; i++) {
        double distance_to_p = p.computeDistance(lane_points[i]);
        if (distance_to_p < distance_min) {
            p_index_distance_min = i;
            distance_min = distance_to_p;
        }
    }
    bool is_endpoint = false;
    if (p_index_distance_min == (n_lane_points - 1)) {
        is_endpoint = true;
    }
    mapobjects::LanePoint3D lp = mapobjects::LanePoint3D(
        lane_points[p_index_distance_min],
        p_index_distance_min,
        lane_uuid);
    lp.lp_type = determineLanePointType(lp, is_endpoint);
    lp.lane_info = getLaneInfo(lp.lane_uuid);
    return lp;
}
