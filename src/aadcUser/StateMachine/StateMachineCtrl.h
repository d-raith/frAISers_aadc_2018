#ifndef AADC_USER_STATEMACHINE_CTRL_H
#define AADC_USER_STATEMACHINE_CTRL_H
#include <vector>
#include <iostream>
#include "ICarCtrl.h"
#include "IPlanner.h"
#include "IControlOutput.h"
#include "BaseTask.h"
#include "IGlobalMap.h"
#include "Astar.h"
#include "cmath"
#include "PrintUtils.h"
#include "Lane.h"
#include "Map.h"
#include "MatUtils.h"
#include "OmplPlanner.h"
#include "../PinClasses/Threadpool.h"
#include "CarModel.h"
#include "ISensorListeners.h"
#include "ICtrlOverride.h"
#include "EmergencyBrakeOverride.h"
#include "RoadSignInfo.h"
#include "EnvironmentState.h"


class StateMachineCtrl : public IVelocityCtrl,
                         public ISteerCtrl, public ICarModelDependent, public ObstacleListener,
                         public PerceptionListener {
private:

    StateMachineCtrl() = default;
    StateMachineCtrl(const StateMachineCtrl&) = delete;
    StateMachineCtrl& operator=(const StateMachineCtrl&) = delete;

    shared_ptr<BaseTask> current_task = nullptr;
    IPlanner *planner;
    IGlobalMap *map = nullptr;
    shared_ptr<LocalMap> local_map;

    IControlOutput *filter_output;
    ILightCtrl *light_ctrl;

    bool task_changed;
    float current_speed = Speed::STOP;
    int update_count = 0;
    std::shared_ptr<ThreadPool> pool = nullptr;

    StopWatch local_map_debug_timer;
    void writeDebugOutputLocalMap(const deque<Point>& waypoints, CarModel car);
    std::mutex birdseye_data_mutex;
    cv::Mat birdseye_data;

    std::shared_ptr<EmergencyBrakeOverride> em_brake_override;
    std::unique_ptr<MaxSpeedOverride> max_speed_override;
    shared_ptr<EnvironmentState> environment_state = make_shared<EnvironmentState>();

public:


    static constexpr float MAX_SPEED = 4.0;

    // colors for drawing in local map
    const cv::Scalar CAR_COLOR = cv::Scalar(0, 255, 0);
    cv::Vec3b LOCALMAP_WAYPOINT_COLOR = cv::Vec3b(255, 165, 0);
    cv::Vec3b INTERSECTION_COLOR = cv::Vec3b(0, 100, 100);
    explicit StateMachineCtrl(IControlOutput *output, IGlobalMap *map, ILightCtrl
    *lightCtrl);

    ~StateMachineCtrl();


    void configure();

    void run();

    void reset();

    void setTask(shared_ptr<BaseTask> &task);

    bool setVehicleSpeed(float speed) override;

    void onObstaclesDetected(const std::vector<Obstacle> *const obstacles, DataSource source) override;

    void onLaserScannerUpdate(const std::vector<tPolarCoordiante>& ls_polar,
                                  const std::vector<Point>& ls_global_cartesian);

    void onPerceptionDataAvailable(const cv::Mat *const data) override;

    void onCarPositionUpdate(DeltaPosition position_update);

    void onUpdateDetectionInfo(tDetectionInfo obstacle_info, vector<tLaserSegStruct> *laserSegData);

    void onLanePointDataAvailable(std::vector<Point> *input);

    void onLaneDataAvailable(std::vector<Lane> &input);

    void onRoadSignDetected(int *sign_id);

    void laserscannerPolarToGlobalCartesian(const std::vector<tPolarCoordiante>& input,
                                            std::vector<Point>* output);

    void setBirdsEyeImage(const cv::Mat &be_img);


    void setCarModel(std::shared_ptr<CarModel> &carModel) override;

    bool setCurvature(float curv_value) override;

    void setEmergencyBrakeCtrl(shared_ptr<EmergencyBrakeOverride> em_brake_ctrl) {
        this->em_brake_override = em_brake_ctrl;
    }

};

#endif //AADC_USER_STATEMACHINE_CTRL_H