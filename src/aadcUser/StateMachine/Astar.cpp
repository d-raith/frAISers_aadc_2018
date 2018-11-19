//
// Created by aadc on 28.07.18.
//

#include "Astar.h"


class GoalUnreachableException {
};

class EmptyOpenListException {
};


float HeadingCostFunction::getCost(float &heading, int &action_idx, int &cell_x, int &cell_y, bool
useAdaptiveHeadingCost, bool isObstacleAvoidance) {
    if (isObstacleAvoidance) {
        return action_heading_cost[4][action_idx];
    }

    if (useAdaptiveHeadingCost) {
        uint action_cost_idx;

        if (heading >= -M_PI / 4 && heading < M_PI / 4) {
            action_cost_idx = 0;
        } else if (heading > M_PI / 4 && heading <= 3 * M_PI / 4) {
            action_cost_idx = 1;
        } else if (abs(heading) > 3 * M_PI / 4) {
            action_cost_idx = 2;
        } else {
            action_cost_idx = 3;
        }
        return action_heading_cost[action_cost_idx][action_idx];
    }
    return 1;
}


void AStarPlanner::doBacktrace(vector<vector<int>> *resultPath,
                               std::unordered_map<std::pair<int, int>, Node,
                                       boost::hash<std::pair<int, int>>> &nodes, Node &next,
                               Node &current, Node &goal, Node
                               &start) {
    /* nodes.insert({{next.x, next.y}, current});
                 if (!resultPath->empty()) {
                     throw "result path not empty!";
                 }

                 Node &backtrace = nodes.at({next.x, next.y});
                 while (true) {

                     if (backtrace == start) {
                         vector<int> point = {start.x, start.y};
                         resultPath->emplace_back(point);
                         break;
                     }
                     vector<int> point = {backtrace.x, backtrace.y};
                     resultPath->emplace_back(point);
                     backtrace = nodes.at({backtrace.x, backtrace.y});
                 }*/

    if (!resultPath->empty()) {
        throw "result path not empty!";
    }

    Node &backtrace = current;

    while (true) {
        if (backtrace == start) {
            vector<int> point = {start.x, start.y};
            resultPath->emplace_back(point);
            break;
        }
        vector<int> point = {backtrace.x, backtrace.y};
        resultPath->emplace_back(point);
        backtrace = nodes.at({backtrace.x, backtrace.y});
    }
}

void AStarPlanner::debugPrint(cv::Mat *grid,
                              std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int,
                                      int>>> closed_list, deque<Node> open_list, Node &start,
                              Node &current, Node &goal) {
    for (int i = 0; i < grid->rows; i++) {
        for (int j = 0; j < grid->cols; j++) {
            auto test = grid->at<float>(i, j);
            int cropped = static_cast<int>(test);
            std::string str = (cropped >= 10 ? "" : " ") + std::to_string(cropped) + " ";

            Node val_node;
            val_node.x = i;
            val_node.y = j;

            if (start == val_node) {
                PrintUtils::printWithColor(" P ", PrintUtils::Color::GREEN);
            } else if (current == val_node) {
                PrintUtils::printWithColor(" C ", PrintUtils::Color::MAGENTA);
            } else if (goal == val_node) {
                PrintUtils::printWithColor("G ", PrintUtils::Color::CYAN);
            } else if (isClosed(closed_list, val_node)) {
                //closed
                PrintUtils::printWithColor(str, PrintUtils::Color::RED);
            } else if (isOpen(open_list, val_node)) {
                //open
                PrintUtils::printWithColor(str, PrintUtils::Color::YELLOW);
            } else {
                cout << std::fixed << std::setprecision(0) << test << " ";
            }

        }
        cout << std::endl;
    }
    cout << std::endl;
}


bool
AStarPlanner::AStarSearch(LocalMap *local_map_ptr, const vector<Move> &delta_moves_, Node start,
                          Node goal,
                          float heading, vector<vector<int>> *resultPath,
                          bool applyHeadingCost) {
    LocalMap local_map = *local_map_ptr;
    int count = 0;


    deque<Node> open_list_ = {start};
    std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int, int>>> closed_list;
    std::unordered_map<std::pair<int, int>, Node, boost::hash<std::pair<int, int>>> nodes;
    const uint num_moves = static_cast<int>(delta_moves_.size());

    // worst case cost assumption (all cells are obstacles)
    float mean_val = 1;

    while (true) {

        if (open_list_.empty()) {
            cout << "resign @ " << count << std::endl;
            throw EmptyOpenListException();
        }
        sort(open_list_.begin(), open_list_.end(), sortOpenList);


        Node current = open_list_.front();
        while (isClosed(closed_list, current)) {
            cout << "open list popped closed node " << current << endl;
            open_list_.pop_front();
            current = open_list_.front();
        }
        open_list_.pop_front();
        if (count % 50 == 0) {
            //debugPrint(&costMap, closed_list, open_list_, start, current, goal);
        }

        close(closed_list, current);


        if (max_iterations != -1 && count >= max_iterations) {
            goal = current;
            cout << "max iterations reached, goal is" << goal << std::endl;
            doBacktrace(resultPath, nodes, current, current, goal, start);
            return true;
        }


        for (uint i = 0; i < num_moves; ++i) {
            const Move *move = &(delta_moves_[i]);

            Node next = current.makeSuccessor(move);

            if (next == goal) {
                doBacktrace(resultPath, nodes, next, current, goal, start);

                return true;
            }

            if (isClosed(closed_list, next)) {
                continue;
            }


            if (next.x >= local_map.channel_perception.rows ||
                next.y >= local_map.channel_perception.cols
                || next.x < 0 ||
                next.y < 0) {
                continue;
            }
            nodes.insert({{next.x, next.y}, current});

            next.g = current.g + cost(heading, i, next.x, next.y,
                                      (applyHeadingCost && count < heading_cost_distance), false);


            next.h = heuristicEstimate(start, goal, current, mean_val);
            next.f = next.g + heuristic_weight * next.h;

            Node *existing = isOpen(open_list_, next);

            if (existing) {
                if (next.g > existing->g) {
                    continue;
                } else {
                    // update node in open list
                    *existing = next;
                }
            } else {
                open(open_list_, next);
            }
        }

        count++;


    }

}


void AStarPlanner::adjustPosition(LocalMap *grid_ptr, Point *startPnt, float heading, Node *
target_ptr) {
    Node position = *target_ptr;
    position.x = static_cast<int>(ceil(startPnt->getX()));
    position.y = static_cast<int>(ceil(startPnt->getY()));

    int shift_x = static_cast<int>(round(cos(heading)));
    int shift_y = static_cast<int>(round(sin(heading)));


    int shift_y_;
    if (position.y > 1) {
        shift_y_ = shift_y >= 0 ? 1 : -1;
    } else {
        shift_y_ = 1;
    }
    int shift_x_;
    if (position.x > 1) {
        shift_x_ = shift_x >= 0 ? 1 : -1;
    } else {
        shift_x_ = 1;
    }


    if (position.x <= 0 && position.y >= 0) {
        position.x = 0;
        position.y = shift_y_ + position.y;
    }
    if (position.y <= 0 && position.x >= 0) {
        position.y = 0;
        position.x = shift_x_ + position.x;
    }

    if (position.x < 0 && position.y < 0) {
        position.x = 0;
        position.y = 0;
    }

    int x = position.x;
    int y = position.y;

    int rows = grid_ptr->channel_obstacles.rows;
    int cols = grid_ptr->channel_obstacles.cols;

    if (x >= rows) {
        x = rows - 1;
    }
    if (y >= cols) {
        y = cols - 1;
    }


    while (isOccupied(grid_ptr, x, y)) {
        if (x + shift_x_ < rows && !isOccupied(grid_ptr, x + shift_x_, y)) {
            x = x + shift_x_;
        } else if (y + shift_y_ < cols && !isOccupied(grid_ptr, x, y + shift_y_)) {
            y = y + shift_y_;
        } else {
            x = x + shift_x_;
            y = y + shift_y_;
        }

        if (x + 1 >= rows || y + 1 >= cols - 1) {
            cout << "Error finding adjusted position " << x << ", " << y << "shifts: " << shift_x
                 << ", " << shift_y;
            break;
        }
    }


    target_ptr->x = min(rows - 1, x);
    target_ptr->y = min(y, cols - 1);
}


void AStarPlanner::getPlan(LocalMap &local_map, PlanRequest *request_ptr, bool
applyHeadingCost) {

    PlanRequest &request = *request_ptr;

    if(request.status != PlanRequest::Status::PENDING){
        throw std::runtime_error("PlanRequest indicates already solved request");
    }

    float heading = request.heading;

    LocalMap *grid_ptr = &local_map;

    Node position = Node();

    adjustPosition(grid_ptr, &request.start, heading, &position);

    Node goal = Node();

    adjustPosition(grid_ptr, &request.goal.point, heading, &goal);


    if (isOccupied(grid_ptr, goal)) {
        cout << "Goal not reachable, adjusting" << std::endl;
        adjustPosition(grid_ptr, &request.goal.point, heading, &goal);
        if (isOccupied(grid_ptr, goal)) {
            request.status = PlanRequest::Status::FAILED;
            cout << "ERROR: Goal still not reachable, aborting" << std::endl;
            throw GoalUnreachableException();
        }
    }

    vector<vector<int>> resultPath;

    try {
        AStarSearch(grid_ptr, move_set, position, goal, heading, &resultPath,
                    applyHeadingCost);
    } catch (EmptyOpenListException e) {
        request.status = PlanRequest::Status::FAILED;
        cout << "ERROR: No more nodes to explore, aborting" << std::endl;

        throw GoalUnreachableException();

    }

    reverse(resultPath.begin(), resultPath.end());

    vector<vector<float>> smoothWp = smoothCourse(resultPath, weight_data, weight_smooth);


    request.result.reserve(smoothWp.size());


    for (auto elem : smoothWp) {
        Point p = Point::Global(elem[0], elem[1]);
        request.result.emplace_back(p);
    }
    request.result.emplace_back(request.goal.point);

    request.status = PlanRequest::Status::SOLVED;
}

double
AStarPlanner::heuristicEstimate(Node &position, Node &goal, Node &cell, float estMean) {
    return (sqrt(pow(goal.x - cell.x, 2) + pow(goal.y - cell.y, 2))) * estMean;

    //performs worse than euclidean dist
    float D = move_step*estMean;
    //float D = cost_function.getCellCostMean();
    float D2 = 1.414*estMean;
    //float D2 = sqrt(D * 2);
    int dx = abs(cell.x - goal.x);
    int dy = abs(cell.y - goal.y);
    return (D * (dx + dy) + (D2 - 2 * D) * min(dx, dy));
}


float AStarPlanner::cost(float heading, int action_idx, int cell_x, int cell_y,
                         bool useAdaptiveHeadingCost,
                         bool isObstacleAvoidance) {

    return heading_cost_function.getCost(heading, action_idx, cell_x, cell_y,
                                                     useAdaptiveHeadingCost, isObstacleAvoidance) *
           cost_function.getCost
                   (heading,
                    action_idx, cell_x, cell_y,
                    useAdaptiveHeadingCost,
                    isObstacleAvoidance);
}

vector<vector<float>> AStarPlanner::smoothCourse(const vector<vector<int>> &course,
                                                 float weight_data,
                                                 float weight_smooth) {
    vector<vector<float>> newpath;
    newpath.reserve(course.size());


    for (auto elem : course) {
        vector<float> point = {(float) elem[0], (float) elem[1]};
        newpath.emplace_back(point);
    }

    float tolerance = 0.00001;
    float change = tolerance;

    while (change >= tolerance) {
        change = 0;

        for (uint i = 1; i < newpath.size() - 1; i++) {
            for (uint j = 0; j < newpath[i].size(); j++) {
                auto aux = newpath[i][j];
                newpath[i][j] = newpath[i][j] + weight_data * (course[i][j] - newpath[i][j]);
                newpath[i][j] = static_cast<float>(newpath[i][j] + weight_smooth *
                                                                   (course[i - 1][j] +
                                                                    newpath[i + 1][j] -
                                                                    (2.0 * newpath[i][j])));
                change += abs(aux - newpath[i][j]);
            }
        }
    }
    return newpath;
}


bool AStarPlanner::isOccupied(LocalMap *map, Node node) {
    return isOccupied(map, node.x, node.y);
}

bool AStarPlanner::isOccupied(LocalMap *map, int x, int y) {
    return false;
    return map->channel_obstacles.at<float>(x, y) >= 0.7;
}

bool
AStarPlanner::isClosed(std::unordered_set<pair<int, int>, boost::hash<pair<int, int>>> &closed_list,
                       AStarPlanner::Node &node) {
    return closed_list.find({node.x, node.y}) != closed_list.end();
}

AStarPlanner::Node *
AStarPlanner::isOpen(deque<AStarPlanner::Node> &open_list, AStarPlanner::Node &node) {
    deque<AStarPlanner::Node, std::allocator<AStarPlanner::Node>>::iterator existingNode;
    existingNode = std::find(open_list.begin(), open_list.end(), node);
    if (existingNode != open_list.end()) {
        return &(*existingNode);
    }
    return nullptr;
}

void
AStarPlanner::close(std::unordered_set<pair<int, int>, boost::hash<pair<int, int>>> &closed_list,
                    AStarPlanner::Node &node) {
    closed_list.insert({node.x, node.y});
}

void AStarPlanner::open(deque<AStarPlanner::Node> &open_list, AStarPlanner::Node &node) {
    open_list.emplace_back(node);
}

