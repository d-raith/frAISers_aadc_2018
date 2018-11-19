//
// Created by aadc on 07.09.18.
//

#ifndef AADC_USER_COSTMAP_H
#define AADC_USER_COSTMAP_H


class CostMap {
public:
    virtual float getCost(float &heading, int &action_idx, int &cell_x, int &cell_y, bool
    useAdaptiveHeadingCost, bool isObstacleAvoidance) = 0;
};

#endif //AADC_USER_COSTMAP_H
