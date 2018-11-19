//
// Created by aadc on 28.07.18.
//

#ifndef PLANNER_ASTAR_H
#define PLANNER_ASTAR_H
#include <vector>
#include "IPlanner.h"
#include "string"
#include <algorithm>
#include <cstdio>
#include <deque>
#include <iostream>
#include <math.h>
#include "chrono"
#include "PrintUtils.h"
#include "utility"
#include "functional"
#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <unordered_set>
#include "CostMap.h"


using namespace std;

class HeadingCostFunction : public CostMap {
    // unkown, lane solid, lane dashed, road, road(opposite lane), obstacle(moving), obstacle(static)
    const vector<int> cell_type_cost_ = {10, 8, 3, 1, 4, 100, 100};

    //heading 0, 1.57, 3.14, -1.57
    // u, ul, l,  dl, d, dr, r, ur
    vector<vector<int>> action_heading_cost = {{100, 100, 100, 40,  1,   100, 100, 100},
                                               {100, 100, 100, 100, 100, 20,  1,   19},
                                               {1,   19, 100, 100, 100, 100, 100, 20},
                                               {100, 20,  1,   19, 100, 100, 100, 100},
                                               //obstacle avoidance
                                               {100, 100, 100, 100, 100, 0,   100, 100}};


 /*   vector<vector<int>> action_heading_cost = {{10,  10, 10, 4,  1,  4,  10, 10},
                                               {10,  10, 10, 10, 10, 4,  1,  4},
                                               {1,   4,  10, 10, 10, 10, 10, 4},
                                               {100, 4,  1,  4,  10, 10, 10, 10},
                                               {10,  10, 10, 10, 10, 4,  1,  4}};*/

public:
    float
    getCost(float &heading, int &action_idx, int &cell_x, int &cell_y, bool useAdaptiveHeadingCost,
            bool isObstacleAvoidance) override;

};

class AStarPlanner : public IPlanner {
private:


    class Move {
    public:
        enum Direction {
            UP,
            UP_LEFT,
            LEFT,
            DOWN_LEFT,
            DOWN,
            DOWN_RIGHT,
            RIGHT,
            UP_RIGHT
        };
        const int dx = 0;
        const int dy = 0;
        const Direction type;

        Move(int dx, int dy, Direction type) : dx(dx), dy(dy), type(type) {
        }
    };

    float heuristic_weight = 1.0;
    int max_iterations = -1;
    int heading_cost_distance = 0;
    CostMap &cost_function;
    HeadingCostFunction heading_cost_function;

    struct Node {
        double f = 0;
        float g = 0;
        double h = 0;
        int x = 0;
        int y = 0;

        bool operator==(const Node &other) {
            return x == other.x && y == other.y;
        }

        Node makeSuccessor(const Move *delta_action) {
            Node n;
            n.x = x + (*delta_action).dx;
            n.y = y + (*delta_action).dy;
            return n;
        }

        friend ostream &operator<<(ostream &os, Node node) {
            os << "[ x: " << node.x << " y: " << node.y << " f: " << node.f << " g: " << node.g
               << " h: " << node.h
               << " ]";
            return os;
        }

    };

    static bool sortOpenList(const Node &a, const Node &b) {
        if (a.f != b.f) {
            return a.f < b.f;
        }

        if (a.h != b.h) {
            return a.h < b.h;
        }

        return a.g < b.g;
    }


    static float distance(Node a, Node b) {
        return sqrt(pow(b.x - a.x, 2) +
                    pow(b.y - a.y, 2));
    }

    bool isClosed(std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int,
            int>>> &closed_list, Node &node);

    Node *isOpen(deque<Node> &open_list, Node &node);

    void close(std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int,
            int>>> &closed_list, Node &node);

    void open(deque<Node> &open_list, Node &node);


    void
    debugPrint(cv::Mat *grid, std::unordered_set<std::pair<int, int>, boost::hash<std::pair<int,
            int>>> closed_list, deque<Node> open_list, Node &start, Node &current, Node &goal);

    void doBacktrace(vector<vector<int>> *resultPath, std::unordered_map<std::pair<int, int>, Node,
            boost::hash<std::pair<int, int>>> &nodes, Node &next, Node &current, Node &goal,
                     Node &start);

    /* vector<vector<int>> action_heading_cost = {{10, 10, 5,  2,  1, 2, 5, 10},
                                                {5,  10, 10, 10, 5, 2, 1, 2},
                                                {1,  2,  2,  2,  2, 2, 2, 2},
                                                {2,  1,  2,  2,  2, 2, 2, 2}};*/
    const int move_step = 1;

    const vector<Move> move_set = {Move(-move_step, 0, Move::Direction::UP),
                                   Move(-move_step, -move_step, Move::Direction::UP_LEFT),
                                   Move(0, -move_step, Move::Direction::LEFT),
                                   Move(move_step, -move_step, Move::Direction::DOWN_LEFT),
                                   Move(move_step, 0, Move::Direction::DOWN),
                                   Move(move_step, move_step, Move::Direction::DOWN_RIGHT),
                                   Move(0, move_step, Move::Direction::RIGHT),
                                   Move(-move_step, move_step, Move::Direction::UP_RIGHT)};

    float cost(float heading, int action_idx, int cell_x, int cell_y, bool useAdaptiveHeadingCost,
               bool isObstacleAvoidance);

    vector<vector<float>> smoothCourse(const vector<vector<int>> &course,
                                       float weight_data, float weight_smooth);

    bool isOccupied(LocalMap *map, int x, int y);

    bool isOccupied(LocalMap *map, Node node);


    void adjustPosition(LocalMap *map, Point *startPnt, float heading, Node *
    target_ptr);

    double heuristicEstimate(Node &position, Node &goal, Node &current, float estMean);

    bool
    AStarSearch(LocalMap *local_map_ptr, const vector<Move> &delta_moves_, Node start, Node goal,
                float heading,
                vector<vector<int>> *resultPath, bool applyHeadingCost);

    float weight_data = 0.1;
    float weight_smooth = 0.5;


public:

    explicit AStarPlanner(float astar_weight, CostMap &costFunction, float
    force_heading_cost_dist = 0.5, const int step_length
    = 1, int max_iter = -1) :
            heuristic_weight(astar_weight), max_iterations(max_iter),
            heading_cost_distance
                    (static_cast<int>(ceil(force_heading_cost_dist * Coordinate::GLOBAL))),
            cost_function(costFunction),
            move_step(step_length) {

    }

    virtual ~AStarPlanner() = default;

    void getPlan(LocalMap &local_map, PlanRequest *request,
                          bool applyHeadingCost) override;
};

#endif  // PLANNER_ASTAR_H