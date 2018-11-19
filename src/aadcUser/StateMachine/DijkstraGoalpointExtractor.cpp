
#include "DijkstraGoalpointExtractor.h"
#include "../PinClasses/StopWatch.h"



vector<cv::Point> TURN_LEFT_WAYPOINTS;
vector<cv::Point> TURN_STRAIGHT_WAYPOINTS;
vector<cv::Point> TURN_RIGHT_WAYPOINTS;


DijkstraGoalpointExtractor::DijkstraGoalpointExtractor() {   }

DijkstraGoalpointExtractor::DijkstraGoalpointExtractor(
                            const int segmentation_channel,
                            const int cost_channel,
                            const int goal_radius,
                            const int start_position_y,
                            const unsigned int select_each_xth_waypoint,
                            const unsigned int intersection_region,
                            const float intersection_threshold,
                            const unsigned int distance_to_intersection,
                            const int point_shift_x) {
    this->segmentation_channel = segmentation_channel;
    this->cost_channel = cost_channel;
    this->goal_radius = goal_radius;
    this->start_position_y = start_position_y;
    this->select_each_xth_waypoint = select_each_xth_waypoint;
    this->intersection_region = intersection_region;
    this->intersection_threshold = intersection_threshold;
    this->distance_to_intersection = distance_to_intersection;
    this->point_shift_x = point_shift_x;


    // --- parameters
    // left
    TURN_LEFT_WAYPOINTS.push_back(cv::Point(0, 20));
    TURN_LEFT_WAYPOINTS.push_back(cv::Point(0, 30));
    TURN_LEFT_WAYPOINTS.push_back(cv::Point(10, 40));
    TURN_LEFT_WAYPOINTS.push_back(cv::Point(20, 50));
    TURN_LEFT_WAYPOINTS.push_back(cv::Point(30, 50));
    TURN_LEFT_WAYPOINTS.push_back(cv::Point(40, 50));
    // straight
    TURN_STRAIGHT_WAYPOINTS.push_back(cv::Point(0, 20));
    TURN_STRAIGHT_WAYPOINTS.push_back(cv::Point(0, 30));
    TURN_STRAIGHT_WAYPOINTS.push_back(cv::Point(0, 40));
    TURN_STRAIGHT_WAYPOINTS.push_back(cv::Point(0, 50));
    TURN_STRAIGHT_WAYPOINTS.push_back(cv::Point(0, 60));
    // right
    TURN_RIGHT_WAYPOINTS.push_back(cv::Point(-0, 20));
    TURN_RIGHT_WAYPOINTS.push_back(cv::Point(-15, 22));
}



const tUInt8 NODIRECTION = 0;
const tUInt8 BOTTOM_LEFT = 25;
const tUInt8 BOTTOM = 50;
const tUInt8 BOTTOM_RIGHT = 75;
const tUInt8 RIGHT = 100;
const tUInt8 TOP_LEFT = 125;
const tUInt8 TOP = 150;
const tUInt8 TOP_RIGHT = 175;
const tUInt8 LEFT = 200;

const tUInt8 GOAL = 255;
const tUInt8 START = 111;
const tUInt8 NOGOAL = 0;

const tUInt8 BACKGROUND = 0;
const tUInt8 ROAD = 1;
const tUInt8 CAR = 4;
const tUInt8 INTERSECTION = 9;
const tUInt8 PERSON = 10;

const int TURN_ERROR = -1;
const int TURN_LANEFOLLOWING = 0;
const int TURN_LEFT = 1;
const int TURN_STRAIGHT = 2;
const int TURN_RIGHT = 3;


class MyNode {
 public:
    MyNode(unsigned int px, unsigned int py, tFloat32 pcost) {
        x = px;
        y = py;
        cost = pcost;
    }
    unsigned int x;
    unsigned int y;
    tFloat32 cost;
};

bool compareNode(MyNode a, MyNode b) {
    return (a.cost > b.cost);
}

bool DijkstraGoalpointExtractor::getGoalPointsToGoal(
            const cv::Mat& _cost_map,
            const cv::Mat& segmentation_map,
            vector<cv::Point>* waypoints,
            bool visualize,
            cv::Mat* debug_image,
            int control_id,
            cv::Point2f local_car_position,
            cv::Point2f goal_position,
            bool plan_to_goal,
            bool disable_goal_radius) {
    // StopWatch watch;
    //
    // extract the cost map and segmentation map from the image
    //
    // use junction as goal for now, or asign some cost to junctions
    cv::Mat cost_map;
    _cost_map.convertTo(cost_map, CV_8UC1, 255);
    cv::Mat* image = debug_image;
    cv::Mat channels[3];
    cv::split(*image, channels);
    cost_map.copyTo(channels[2]);
    cv::merge(channels, 3, *image);
    cost_map = 255 - cost_map;  // where segmentation gives low probability of lane, high cost

    bool fallback = false;
    // set the new car position to start search from
    start_position_x = static_cast<int>(local_car_position.x);
    int start_position_y_persistent = start_position_y + static_cast<int>(local_car_position.y);

    if (goal_position.y < start_position_y_persistent && plan_to_goal) {
        fallback = true;
        // LOG_WARNING("DijkstraGoalpointExtractor: goal waypoint behind car. not planning. goal: %d %d | local_car: %d %d",
        // goal_position.x, goal_position.y, start_position_x, start_position_y_persistent);
        plan_to_goal = false;
    }

    // validate goal position
    if (goal_position.x < 0 || goal_position.x > (cost_map.size().width - 1) \
        || goal_position.y < start_position_y_persistent || goal_position.y > (cost_map.size().height - 1)) {
        fallback = true;
        // LOG_WARNING("DijkstraGoalpointExtractor: goal_position out of bounds. fallback to lanefollowing");
        plan_to_goal = false;
    }

    //
    // define and set varibles
    //
    // int goal_circle_center_x = cost_map.size().width / 2;
    // int start_position_x = cost_map.size().width / 2;
    unsigned int found_goal_x = 1;
    unsigned int found_goal_y = 1;
    bool goal_found = false;
    bool intersection_found = false;

    //
    // declare the data structures
    //
    // queue that makes the position with the lowest cost available
    std::priority_queue<MyNode,
                        std::vector<MyNode>,
                        decltype(&compareNode)> prioqueue(&compareNode);
    // contains costs to reach each node
    cv::Mat result_map_cost =
        cv::Mat(cv::Size(cost_map.size().width, cost_map.size().height), CV_32F);
    result_map_cost = cv::Scalar(1000000000.0);
    // contains direction of the best path for each node towards the start
    cv::Mat result_map_predecessors =
        cv::Mat(cv::Size(cost_map.size().width, cost_map.size().height), CV_8U);
    result_map_predecessors = cv::Scalar(NODIRECTION);

    //
    // initialize the data structures
    //
    // cost to reach start position is cost of the start position itself
    // start_list_x.clear();
    // start_list_y.clear();
    goal_list_x.clear();
    goal_list_y.clear();
    // create goal_list and fill goal_map
    goal_map = cv::Mat(cv::Size(cost_map.size().width, cost_map.size().height), CV_8U);
    goal_map = NOGOAL;

    //
    //  start shapes
    //
    goal_map.at<tUInt8>(cv::Point(start_position_x, start_position_y_persistent)) = START;
    // cv::line(goal_map,
    //                 cv::Point(start_position_x - 5, start_position_y_persistent),
    //                 cv::Point(start_position_x - 4, start_position_y_persistent),
    //                 START,
    //                 1);

    //
    // goal shapes
    //
    if (plan_to_goal) {
        if (!disable_goal_radius) {
            // global subgoal radius
            cv::circle(goal_map,
                goal_position,
                goal_radius,
                GOAL,  // color
                2);  // thickness
        } else {
            // disable goal radius if approaching junction
            goal_map.at<tUInt8>(goal_position) = GOAL;
        }
    } else {
        // LOG_WARNING(
        //     "DijkstraGoalpointExtractor: no global goal provided. fallback to lanefollowing");
        // stong inclined line left, straigt line middle, no goal right
        cv::line(goal_map,
            cv::Point(0, goal_map.size().height / 2),
            cv::Point(goal_map.size().width  / 3, goal_map.size().height * 4 / 5),
            GOAL,
            1);
        cv::line(goal_map,
            cv::Point(goal_map.size().width / 3, goal_map.size().height * 4 / 5),
            cv::Point(goal_map.size().width * 3 / 4, goal_map.size().height * 4 / 5),
            GOAL,
            1);
    }

    for (int i = 0; i != goal_map.size().height; i++) {
        for (int j = 0; j != goal_map.size().width; j++) {
            if (goal_map.at<tUInt8>(cv::Point(j, i)) == GOAL) {
                goal_list_x.push_back(j);
                goal_list_y.push_back(i);
            } else if (goal_map.at<tUInt8>(cv::Point(j, i)) == START) {
                start_list_x.push_back(j);
                start_list_y.push_back(i);
            }
        }
    }
    // write start nodes to priority queue and init maps
    for (unsigned int i = 0; i < start_list_x.size(); i++) {
        int x = start_list_x[i];
        int y = start_list_y[i];
        result_map_cost.at<tFloat32>(cv::Point(x, y)) =
            static_cast<tFloat32>(cost_map.at<tUInt8>(cv::Point(x, y)));
            result_map_predecessors.at<tUInt8>(cv::Point(x, y)) = TOP;
            prioqueue.push(MyNode(static_cast<unsigned int>(x),
                                  static_cast<unsigned int>(y),
                                  static_cast<tFloat32>(
                                        cost_map.at<tUInt8>(cv::Point(x, y)))));
    }

    //
    // Dijkstra Breadth-First-Search
    //
    while (prioqueue.size() > 0) {
        MyNode element = prioqueue.top();
        prioqueue.pop();
        // check if element is goal
        if (goal_map.at<tUInt8>(cv::Point(element.x, element.y)) == GOAL) {
            found_goal_x = element.x;
            found_goal_y = element.y;
            goal_found = true;
            break;
        }
        // second goal check, see if point is on junction
        // if (segmentation_map.at<tUInt8>(cv::Point(element.x, element.y)) == INTERSECTION) {
        //     // check some region around the detected intersection point

        //         int roi_offset_x = max(static_cast<int>(element.x - intersection_region / 2), 0);
        //         int roi_offset_y = max(static_cast<int>(element.y - intersection_region / 2), 0);
        //         int roi_widht = min(static_cast<int>(intersection_region),
        //                                 segmentation_map.size().width - roi_offset_x);
        //         int roi_height =  min(static_cast<int>(intersection_region),
        //                                 segmentation_map.size().height - roi_offset_y);
        //         cv::Mat roi;
        //         try {
        //             roi = segmentation_map(cv::Rect(roi_offset_x,
        //                                                 roi_offset_y,
        //                                                 roi_widht,
        //                                                 roi_height));
        //         } catch (cv::Exception e) {
        //             LOG_ERROR("DijkstraGoalpointExtractor: roi error. rethrowing");
        //             throw e;
        //         }

        //     int intersection_count = 0;
        //     for (int i = 0; i < roi.size().height; i++) {
        //         for (int j = 0; j < roi.size().width; j++) {
        //             if (roi.at<tUInt8>(cv::Point(j, i)) == INTERSECTION) {
        //                 intersection_count++;
        //             }
        //         }
        //     }
        //     float roi_intersection_ratio = static_cast<float>(intersection_count) /
        //                         static_cast<float>(roi.size().width * roi.size().height);
        //     if (roi_intersection_ratio > intersection_threshold) {
        //         found_goal_x = element.x;
        //         found_goal_y = element.y;
        //         goal_found = true;
        //         intersection_found = true;
        //         break;
        //     }
        // }
        // expand element
        // (first add neighbours to neighbour list, than process the neightbour list)
        unsigned int neighbour_list_size = 3;
        unsigned int neighbour_list_x[3];
        unsigned int neighbour_list_y[3];
        tFloat32 new_direction_list[3];
        // std::vector<tFloat32> cost_factors;
        // bottom
        if (element.y + 1 < static_cast<unsigned int>(cost_map.size().height)) {
            neighbour_list_x[0] = element.x;
            neighbour_list_y[0] = element.y + 1;
            new_direction_list[0] = TOP;
            // cost_factors.push_back(1.0);
        }
        // right
        if (element.x + 1 < static_cast<unsigned int>(cost_map.size().width)) {
            neighbour_list_x[1] = element.x + 1;
            neighbour_list_y[1] = element.y;
            new_direction_list[1] = LEFT;
            // cost_factors.push_back(1.0);
        }
        // left
        if (element.x >= 1) {
            neighbour_list_x[2] = element.x - 1;
            neighbour_list_y[2] = element.y;
            new_direction_list[2] = RIGHT;
            // cost_factors.push_back(1.0);
        }
        // // bottom_left
        // if (element.y + 1 < static_cast<unsigned int>(cost_map.size().height)
        //     && element.x > 1) {
        //     neighbour_list_x.push_back(element.x - 1);
        //     neighbour_list_y.push_back(element.y + 1);
        //     new_direction_list.push_back(TOP_RIGHT);
        //     cost_factors.push_back(1.414);
        // }
        // // bottom_right
        // if (element.y + 1 < static_cast<unsigned int>(cost_map.size().height)
        //     && element.x + 1 < static_cast<unsigned int>(cost_map.size().width)) {
        //     neighbour_list_x.push_back(element.x + 1);
        //     neighbour_list_y.push_back(element.y + 1);
        //     new_direction_list.push_back(TOP_LEFT);
        //     cost_factors.push_back(1.85);
        // }
        for (unsigned int i = 0; i < neighbour_list_size; i++) {
            unsigned int neighbour_x = neighbour_list_x[i];
            unsigned int neighbour_y = neighbour_list_y[i];
            tFloat32 neighbour_cost = static_cast<tFloat32>(
                                    cost_map.at<tUInt8>(cv::Point(neighbour_x, neighbour_y)));
            tFloat32 new_direction = new_direction_list[i];
            // tFloat32 cost_factor = cost_factors[i];
            if (neighbour_cost > 190.0) {  // extra punish non-lane-space
                neighbour_cost = neighbour_cost + static_cast<tFloat32>(200.0);
            }
            tFloat32 new_cost = element.cost + neighbour_cost;  //  * cost_factor;
            tFloat32 old_cost = result_map_cost.at<tFloat32>(cv::Point(neighbour_x, neighbour_y));

            if (new_cost < old_cost) {
                result_map_predecessors.at<tUInt8>(cv::Point(neighbour_x, neighbour_y)) =
                    new_direction;
                result_map_cost.at<tFloat32>(cv::Point(neighbour_x, neighbour_y)) = new_cost;
                prioqueue.push(MyNode(neighbour_x, neighbour_y, new_cost));
            }
        }
    }

    //
    // extract path
    //
    vector<cv::Point> result_path;
    if (goal_found == false) {
        LOG_ERROR("Goal not found");
    } else if (found_goal_x > static_cast<unsigned int>(cost_map.size().width)
            || found_goal_y > static_cast<unsigned int>(cost_map.size().height)) {
        LOG_ERROR("Goal invalid: x=%i, y=%i", found_goal_x, found_goal_y);
        std::cout << "Goal invalid: x=" <<
            found_goal_x << ",y=" << found_goal_y << std::endl;
    } else {
        // extract path
        unsigned int vis_element_x = found_goal_x;
        unsigned int vis_element_y = found_goal_y;
        unsigned int path_length = 0;
        while (goal_map.at<tUInt8>(cv::Point(vis_element_x, vis_element_y)) != START) {
            if (result_map_predecessors.at<tUInt8>(cv::Point(vis_element_x, vis_element_y))
                    == TOP) {
                vis_element_x = vis_element_x;
                vis_element_y = vis_element_y - 1;
            } else if (result_map_predecessors.at<tUInt8>(cv::Point(vis_element_x, vis_element_y))
                    == LEFT) {
                vis_element_x = vis_element_x - 1;
                vis_element_y = vis_element_y;
            } else if (result_map_predecessors.at<tUInt8>(cv::Point(vis_element_x, vis_element_y))
                    == RIGHT) {
                vis_element_x = vis_element_x + 1;
                vis_element_y = vis_element_y;
            } else if (result_map_predecessors.at<tUInt8>(cv::Point(vis_element_x, vis_element_y))
                    == BOTTOM) {
                vis_element_x = vis_element_x;
                vis_element_y = vis_element_y + 1;
            } else if (result_map_predecessors.at<tUInt8>(cv::Point(vis_element_x, vis_element_y))
                    == TOP_RIGHT) {
                vis_element_x = vis_element_x + 1;
                vis_element_y = vis_element_y - 1;
            }  else if (result_map_predecessors.at<tUInt8>(cv::Point(vis_element_x, vis_element_y))
                    == TOP_LEFT) {
                vis_element_x = vis_element_x - 1;
                vis_element_y = vis_element_y - 1;
            } else {
                break;
            }
            result_path.push_back(cv::Point(vis_element_x, vis_element_y));
            if (path_length % select_each_xth_waypoint == 0) {
                waypoints->push_back(cv::Point(vis_element_x, vis_element_y));
            }
            path_length++;
        }
    }

    // only say intersection when we are close
    // if (result_path.size() > distance_to_intersection) {
    //     intersection_found = false;
    // }
    // if (sqrt((result_path[0].x - start_position_x)*(result_path[0].x - start_position_x)
    //     + (result_path[0].y - start_position_y_persistent)*(result_path[0].y - start_position_y_persistent))
    //     > distance_to_intersection) {
    //     intersection_found = false;
    // } //else {
    //     if (intersection_found) {
    //         LOG_WARNING("DEBUG: in intersection proximity. generating straight line");
    //     }
    // }

    //
    // apply shift in x direction to points
    //
    for (unsigned int i = 0; i < waypoints->size(); i++) {
        (*waypoints)[i] = cv::Point(
                                max(min(
                                    (*waypoints)[i].x + point_shift_x, cost_map.size().width), 0),
                                (*waypoints)[i].y);
    }

    //
    //  write corner goalpoints
    //
    int turn_type = control_id;
    if (intersection_found == true && (
            turn_type == TURN_LEFT || false)) {     // todo reactivate condition
        waypoints->clear();
        for (unsigned int i = 0; i < TURN_LEFT_WAYPOINTS.size(); i++) {
            waypoints->push_back(cv::Point(TURN_LEFT_WAYPOINTS[i].x + cost_map.size().width / 2 + start_position_x,
                                           TURN_LEFT_WAYPOINTS[i].y + start_position_y_persistent));
        }
    }
    if (intersection_found == true && (
        turn_type == TURN_STRAIGHT || false)) {     // todo reactivate condition
        waypoints->clear();
        for (unsigned int i = 0; i < TURN_STRAIGHT_WAYPOINTS.size(); i++) {
            waypoints->push_back(cv::Point(TURN_STRAIGHT_WAYPOINTS[i].x + start_position_x,
                                           TURN_STRAIGHT_WAYPOINTS[i].y + start_position_y_persistent));
        }
    }
    if (intersection_found == true && (
        turn_type == TURN_RIGHT || false)) {    // todo reactivate condition
        waypoints->clear();
        for (unsigned int i = 0; i < TURN_RIGHT_WAYPOINTS.size(); i++) {
            waypoints->push_back(cv::Point(TURN_RIGHT_WAYPOINTS[i].x + cost_map.size().width / 2 + start_position_x,
                                           TURN_RIGHT_WAYPOINTS[i].y + start_position_y_persistent));
        }
    }

    //
    // visualize
    //
    if (visualize == true) {
        // visualize junctions (optional also car and person)
        for (int i = 0; i != image->size().height; i++) {
            for (int j = 0; j != image->size().width; j++) {
                if (segmentation_map.at<tUInt8>(cv::Point(j, i)) == INTERSECTION) {
                    image->at<cv::Vec3b>(cv::Point(j, i))[0] = intersection_color[0];
                    image->at<cv::Vec3b>(cv::Point(j, i))[1] = intersection_color[1];
                    image->at<cv::Vec3b>(cv::Point(j, i))[2] = intersection_color[2];
                }
            }
        }
        // visualize goal
        for (unsigned int i = 0; i < goal_list_x.size(); i++) {
            image->at<cv::Vec3b>(cv::Point(goal_list_x[i], goal_list_y[i]))[0] = goal_color[0];
            image->at<cv::Vec3b>(cv::Point(goal_list_x[i], goal_list_y[i]))[1] = goal_color[1];
            image->at<cv::Vec3b>(cv::Point(goal_list_x[i], goal_list_y[i]))[2] = goal_color[2];
        }
        // visualize path
        for (unsigned int i = 0; i < result_path.size(); i++) {
            // image->at<Vec3b>(cv::Point(vis_element_x, vis_element_y))[0] = path_color[0];
            // image->at<Vec3b>(cv::Point(vis_element_x, vis_element_y))[1] = path_color[1];
            // image->at<Vec3b>(cv::Point(vis_element_x, vis_element_y))[2] = path_color[2];
        }
        // visualize waypoints
        for (unsigned int i = 0; i < waypoints->size(); i++) {
            image->at<cv::Vec3b>(static_cast<int>((*waypoints)[i].y),
                             static_cast<int>((*waypoints)[i].x))[0] = path_waypoint_color[0];
            image->at<cv::Vec3b>(static_cast<int>((*waypoints)[i].y),
                             static_cast<int>((*waypoints)[i].x))[1] = path_waypoint_color[1];
            image->at<cv::Vec3b>(static_cast<int>((*waypoints)[i].y),
                             static_cast<int>((*waypoints)[i].x))[2] = path_waypoint_color[2];
        }
        // visualize start
        for (unsigned int i = 0; i < start_list_x.size(); i++) {
            image->at<cv::Vec3b>(cv::Point(start_list_x[i], start_list_y[i]))[0] = start_color[0];
            image->at<cv::Vec3b>(cv::Point(start_list_x[i], start_list_y[i]))[1] = start_color[1];
            image->at<cv::Vec3b>(cv::Point(start_list_x[i], start_list_y[i]))[2] = start_color[2];
        }
        // visualize found goal
        if (goal_found == true) {
            image->at<cv::Vec3b>(cv::Point(found_goal_x, found_goal_y))[0] = foundgoal_color[0];
            image->at<cv::Vec3b>(cv::Point(found_goal_x, found_goal_y))[1] = foundgoal_color[1];
            image->at<cv::Vec3b>(cv::Point(found_goal_x, found_goal_y))[2] = foundgoal_color[2];
        }
        // visualize expansion front
        while (prioqueue.size() > 0) {
            MyNode element = prioqueue.top();
            prioqueue.pop();
            image->at<cv::Vec3b>(cv::Point(element.x, element.y))[0] = queue_color[0];
            image->at<cv::Vec3b>(cv::Point(element.x, element.y))[1] = queue_color[1];
            image->at<cv::Vec3b>(cv::Point(element.x, element.y))[2] = queue_color[2];
        }
    }

    // watch.print_measurement("dij");
    return fallback;   // return intersection_found;
}

int DijkstraGoalpointExtractor::getGoalRadius() {
    return this->goal_radius;
}
