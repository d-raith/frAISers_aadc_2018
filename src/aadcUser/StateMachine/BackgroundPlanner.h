#include <memory>

//
// Created by aadc on 18.09.18.
//

#ifndef AADC_USER_BACKGROUNDPLANNER_H
#define AADC_USER_BACKGROUNDPLANNER_H
#include "../PinClasses/Threadpool.h"
#include "Point.h"
#include "LocalMap.h"
#include "vector"
#include "IPlanner.h"


class NoMoreRequestsException: public exception{

};

class BackgroundPlanner {
private:
    std::mutex wp_list_lock;
    std::unique_ptr<ThreadPool> pool;
    vector<PlanRequest> processedRequests;
    IPlanner *planner;
    shared_ptr<LocalMap> local_map;
    fraisers::models::Point *goal;
    std::atomic<bool> shutdown = {false};
    std::atomic<int> request_count = {0};

    void processPlanRequest(PlanRequest &request);

public:

    void setPlanner(IPlanner *planner);

    void setLocalMap(shared_ptr<LocalMap> &local_map);

    PlanRequest getLastRequest();


    void requestPlan(PlanRequest request);


    bool hasOpenRequests(){
        return request_count>0;
    }

    int openRequestCount(){
        return request_count;
    }

    bool solutionAvailable();


    void start();

    void stop();


    BackgroundPlanner();

    ~BackgroundPlanner();


};

#endif //AADC_USER_BACKGROUNDPLANNER_H
