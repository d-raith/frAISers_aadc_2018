

#include "BackgroundPlanner.h"


void BackgroundPlanner::setPlanner(IPlanner *planner) {
    this->planner = planner;
}

void BackgroundPlanner::setLocalMap(shared_ptr<LocalMap> &local_map) {
    this->local_map = local_map;
}

void BackgroundPlanner::processPlanRequest(PlanRequest &request) {
    StopWatch watch;

    planner->getPlan(*request.local_map, &request, false);
    watch.print_measurement("BackgroundPlanner::processPlanRequest");

    std::lock_guard<std::mutex> lock(wp_list_lock);
    processedRequests.emplace_back(request);
    request_count--;

}


void BackgroundPlanner::requestPlan(PlanRequest request) {

    cout << "Plan requested " << endl;
    cout << "Start:" << request.start << endl;
    cout << "Goal: " << request.goal.point<< endl;

    auto func = std::bind(&BackgroundPlanner::processPlanRequest, this, request);
    pool->submit(func);
    request_count++;
}


PlanRequest BackgroundPlanner::getLastRequest() {
    std::lock_guard<std::mutex> lock(wp_list_lock);
    if (processedRequests.empty()) {
        throw NoMoreRequestsException();
    }

    PlanRequest result =  processedRequests.back();
    processedRequests.pop_back();
    return result;
}

bool BackgroundPlanner::solutionAvailable() {
    std::lock_guard<std::mutex> lock(wp_list_lock);
    return static_cast<bool>(processedRequests.size());

};

void BackgroundPlanner::start() {

}

void BackgroundPlanner::stop() {
    shutdown = true;
}


BackgroundPlanner::BackgroundPlanner() {
    pool = std::make_unique<ThreadPool>(1);
    // Initialize pool
    pool->init();
}


BackgroundPlanner::~BackgroundPlanner() {
    stop();
    pool->shutdown();
    pool.reset();

}


