
#include "LocalMap.h"
#include "../Map/MapCore/Map.h"


void LocalMap::init() {
    this->channel_road_lanes = cv::Mat(window_height, window_width, CV_32FC2,
                                       cv::Scalar(0, 1));
    this->channel_perception = cv::Mat(window_height, window_width, CV_32F,
                                      cv::Scalar(0));
    this->channel_obstacles = cv::Mat(window_height, window_width, CV_32FC2,
                                       cv::Scalar(0, 1));
    this->channel_segmentation = cv::Mat(window_height, window_width, CV_8UC1, cv::Scalar(0));

    local_x_delta = 0;
    local_y_delta = 0;
    rotation_delta = 0;

    cv::Mat mask(channel_road_lanes.size(), CV_8U, cv::Scalar(1));
    mask.at<uchar>(static_cast<int>(localCarPosition.getX()),
                   static_cast<int>(localCarPosition.getY())) = 0;

    cv::distanceTransform(mask, distance_penalty_mask, cv::DIST_L2, cv::DIST_MASK_3);


    cv::normalize(distance_penalty_mask, distance_penalty_mask, 0, 1, cv::NORM_MINMAX);
    distance_penalty_mask = 1 - distance_penalty_mask;
    map_initialized = true;
}

void LocalMap::addElements(std::vector<Point> &elements) {

}

bool LocalMap::isInitialized() {
    return map_initialized;
}


float LocalMap::getCost(float &heading, int &action_idx, int &cell_x, int &cell_y,
                        bool useAdaptiveHeadingCost, bool isObstacleAvoidance) {
    return costAt(cell_x, cell_y);
}


void softmax(cv::Mat &vals) {

    cv::exp(vals, vals);
    cv::Scalar sum = cv::sum(vals);
    vals = vals / sum[0];
}


void LocalMap::applyCostToSoftmax(cv::Mat &softmaxProb) {
    softmaxProb = (1 - softmaxProb).mul(cost_map);
}

int watchcount = 0;

cv::Vec3f LocalMap::computeSoftmaxAt(int x, int y) {
    cv::Vec2f hit_miss_road = channel_road_lanes.at<cv::Vec2f>(x, y);
    //cv::Vec2f hit_miss_perception = channel_perception.at<cv::Vec2f>(x, y);
    cv::Vec2f hit_miss_obstacle = channel_obstacles.at<cv::Vec2f>(x, y);

    float p_road = hit_miss_road[0] / (hit_miss_road[0] + hit_miss_road[1]);
    //float p_perc = hit_miss_perception[0] / (hit_miss_perception[0] + hit_miss_perception[1]);
    float p_perc = channel_perception.at<float>(x, y);
    float p_obs = hit_miss_obstacle[0] / (hit_miss_obstacle[0] + hit_miss_obstacle[1]);

    // cv::Mat channel_vals = (cv::Mat_<float>(1, 3) << p_road, p_obs, p_perc);
    // softmax(channel_vals);  // ~50ms
    // cv::Point min_idx, max_idx;
    // double min_val, max_val;
    // cv::minMaxLoc(channel_vals, &min_val, &max_val, &min_idx, &max_idx);
    // if (max_val < 0.34) {
    //     return cv::Vec3f(0, 0, 0);
    // }
    // cv::Mat channel_vals = ... and from softmax(channel_vals) to here makes together ~70ms

    // cv::Vec3f result = cv::Vec3f((float *) channel_vals.data);
    cv::Vec3f result = cv::Vec3f(p_road, p_obs, p_perc);
    return result;
}


cv::Vec3f LocalMap::costVectorAt(int x, int y) {

    cv::Vec2f hit_miss_road = channel_road_lanes.at<cv::Vec2f>(x, y);
    cv::Vec2f hit_miss_perception = channel_perception.at<cv::Vec2f>(x, y);
    cv::Vec2f hit_miss_obstacle = channel_obstacles.at<cv::Vec2f>(x, y);

    float p_road = hit_miss_road[0] / (hit_miss_road[0] + hit_miss_road[1]);
    float p_perc = hit_miss_perception[0] / (hit_miss_perception[0] + hit_miss_perception[1]);
    float p_obs = hit_miss_obstacle[0] / (hit_miss_obstacle[0] + hit_miss_obstacle[1]);


    cv::Mat channel_vals = (cv::Mat_<float>(1, 3) << p_road, p_perc, p_obs);
    softmax(channel_vals);

    cv::Point min_idx, max_idx;
    double min_val, max_val;
    cv::minMaxLoc(channel_vals, &min_val, &max_val, &min_idx, &max_idx);

    if (max_val < 0.34) {
        return cv::Vec3f(0, 0, 0);
    }

    applyCostToSoftmax(channel_vals);
    cv::Vec3f cost((float *) channel_vals.data);
    return cost;
}

float LocalMap::costAt(int x, int y) {

    cv::Vec2f hit_miss_road = channel_road_lanes.at<cv::Vec2f>(x, y);
    float hit_miss_perception = channel_perception.at<float>(x, y);
    cv::Vec2f hit_miss_obstacle = channel_obstacles.at<cv::Vec2f>(x, y);

/*    if (hit_miss_road[1] > 100 || hit_miss_road[0] > 100) {
        channel_road_lanes /= 100;
    }

    if (hit_miss_perception[1] > 100 || hit_miss_perception[0] > 100) {
        channel_perception /= 100;
    }

    if (hit_miss_obstacle[1] > 1000 || hit_miss_obstacle[0] > 1000) {
        channel_obstacles /= 1000;
    }*/


    cv::Mat channel_vals = (cv::Mat_<float>(1, 3) <<
                                                  hit_miss_road[0] /
                                                  (hit_miss_road[0] + hit_miss_road[1]),
            //hit_miss_perception[0] / (hit_miss_perception[0] + hit_miss_perception[1]),
            hit_miss_perception,
            hit_miss_obstacle[0] / (hit_miss_obstacle[0] + hit_miss_obstacle[1]));

    softmax(channel_vals);


    cv::Point min_idx, max_idx;
    double min_val, max_val;
    cv::minMaxLoc(channel_vals, &min_val, &max_val, &min_idx, &max_idx);

    if (max_val < 0.34) {
        return COST_UNKNOWN;
    }


    applyCostToSoftmax(channel_vals);


    float cost = channel_vals.at<float>(max_idx);

    if (max_idx == cv::Point(2, 0) && false) {
        cout << "Obstacle at cell:" << x << ", " << y << endl;
        cout << "Channel vals:" << endl;
        cout << channel_vals << endl;
        cout << "return cost: " << cost << endl;
        cout << "----------------" << endl;
        //return -1;
    }
    return cost;
}


// DEPRACTED. do not use for local map where car heading is fixed
Point LocalMap::convertToGlobalFrame(const Point &localMapWaypoint) {
    LOG_WARNING("Function convertToGlobalFrame not up to date with local map.");
    float x = localMapWaypoint.getX() - localCarPosition.getX() + car_model->getRearAxis().getX();

    float y = localMapWaypoint.getY() - localCarPosition.getY() + car_model->getRearAxis().getY();
    Point global = Point::Global(x, y);
    global.setType(car_model->getRearAxis().type);
    return global;
}
// DEPRACTED. do not use for local map where car heading is fixed
Point LocalMap::convertToLocalFrame(const Point &globalWaypoint) {
    LOG_WARNING("Function convertToLocalFrame not up to date with local map.");
    float x = globalWaypoint.getX() - car_model->getRearAxis().getX() + localCarPosition.getX();

    float y = globalWaypoint.getY() - car_model->getRearAxis().getY() + localCarPosition.getY();
    return Point::Local(x, y, car_model->getRearAxis().getUnit());
}
// DEPRACTED. do not use for local map where car heading is fixed
void LocalMap::convertToLocalFrame(cv::Point2f *globalWaypoint) {
    LOG_WARNING("Function convertToLocalFrame not up to date with local map.");
    // float x = globalWaypoint->x - car_model->getFrontAxis().getX() + localCarPosition.getX();
    float x = globalWaypoint->x - car_model->getRearAxis().getX() + localCarPosition.getX();

    // float y = globalWaypoint->y - car_model->getFrontAxis().getY() + localCarPosition.getY();
    float y = globalWaypoint->y - car_model->getRearAxis().getY() + localCarPosition.getY();
    globalWaypoint->x = x;
    globalWaypoint->y = y;

}
Point LocalMap::getLocalPoint(Point global) {
    return car_model->getRearAxis().toLocal(global, car_model->getHeading());
}
void LocalMap::updatePnt(Point *pt) {
        pt->setX(-pt->getX());
        pt->setX(pt->getX()+localCarPosition.getX());
        pt->setY(pt->getY()+localCarPosition.getY());
}

cv::Point2i LocalMap::convertFromGlobalToLocalMapFrame(Point point) {
    if (car_model) {
        point = getLocalPoint(point);
        updatePnt(&point);
        return cv::Point2i(static_cast<int>(point.getX()), static_cast<int>(point.getY()));
    } else {
        LOG_WARNING("car_model null in convertFromGlobalToLocalMapFrame. Returning (0, 0) as dummy output.");
        return cv::Point2i(0, 0);
    }
}


void translateImg(const cv::Mat &img, int offsetx, int offsety) {
    //switch from car frame (x -> rowidx, y->colidx) to opencv frame (y->rowidx, x-> colidx)
    cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
    cv::warpAffine(img, img, trans_mat, img.size(), CV_INTER_LINEAR, cv::BORDER_CONSTANT,
                   cv::Scalar(0, 1));
}

void rotateImg(cv::Mat &img, float rotation, cv::Point2f center) {
    // switch from car frame (x -> rowidx, y->colidx) to opencv frame (y->rowidx, x-> colidx)  ?????
    double scale = 1;
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, rotation, scale);
    cv::warpAffine(img, img, rotation_matrix, img.size(), CV_INTER_LINEAR, cv::BORDER_CONSTANT,
                   cv::Scalar(0, 1));
}



void LocalMap::updateHitsMisses(cv::Mat &channel, const std::vector<cv::Point2f> &hits, float
hitVal) {
    cv::Mat updateMask = cv::Mat(channel.size(), CV_32FC2, cv::Scalar(0, 1));

    for (auto &center : hits) {
        int x = static_cast<int>(ceil(center.x));
        int y = static_cast<int>(ceil(center.y));


        if (!MatUtils::inBoundaries(updateMask, x, y)) {
            //cout << "Not in boundaries: " << x << " " << y << endl;
            continue;
        }

        updateMask.at<cv::Vec2f>(x, y)[0] += hitVal;
        updateMask.at<cv::Vec2f>(x, y)[1] = 0;
        updateMask.mul(distance_penalty_mask);

    }

    channel += updateMask;

    // cv::add(channel, cv::Scalar(1), channel, updateMask, CV_32FC2);
}

void LocalMap::updateHitsMisses(cv::Mat &channel, const std::vector<Obstacle> *const obstacles) {
    cv::Mat updateMask = cv::Mat(channel.size(), CV_32FC2, cv::Scalar(0, 1));
    for (auto &obs : *obstacles) {
        Point local = convertToLocalFrame(obs);

        int x = static_cast<int>(ceil(local.getX()));
        int y = static_cast<int>(ceil(local.getY()));

        if (MatUtils::inBoundaries(updateMask, x, y)) {
            updateMask.at<cv::Vec2f>(x, y)[0] = 1;
            updateMask.at<cv::Vec2f>(x, y)[1] = 0;
        }

    }

    //cv::GaussianBlur(updateMask, updateMask, cv::Size(25, 25), 5, 5);

    channel += updateMask;
}


void LocalMap::onPerceptionDataAvailable(const cv::Mat *const data) {
    try {
        std::vector<cv::Mat> perception_channels;
        cv::split(*data, perception_channels);
        cv::Mat regr_update = perception_channels[2];
        cv::Mat segm_update = perception_channels[0];
        // perception_channels[0].copyTo(channel_segmentation);

        //
        //  update regression a.k.a channel_perception, runnning average
        //
        // get the birdseye anchor
        float pixel_per_meter = 100;
        cv::Point2f local_car_position = cv::Point2f(localCarPosition.getX(),
                                                    localCarPosition.getY());
        cv::Point2f rear_axis = local_car_position;
        cv::Point2f beye_anchor_local = rear_axis + car_model->getLocalBeyeAnchor(pixel_per_meter);


        int src_roi_offset_width = 0;
        int src_roi_offset_height = 36;
        int src_roi_width = regr_update.size().width;
        int src_roi_height = regr_update.size().height - src_roi_offset_height;

        int target_roi_offset_width = beye_anchor_local.x - regr_update.size().width / 2;
        int target_roi_offset_height = beye_anchor_local.y + src_roi_offset_height;
        int target_roi_width = regr_update.size().width;
        int target_roi_height = regr_update.size().height - src_roi_offset_height;
        // make sure that roi is not out of bound!
        cv::Rect target_roi = cv::Rect(target_roi_offset_width,
                                        target_roi_offset_height,
                                        target_roi_width,
                                        target_roi_height);

        cv::Rect src_roi = cv::Rect(src_roi_offset_width,
                                    src_roi_offset_height,
                                    src_roi_width,
                                    src_roi_height);

        // integrate the costs
        cv::Mat regr_update_float = cv::Mat(cv::Size(src_roi_width, src_roi_height), CV_32FC1);
        regr_update(src_roi).convertTo(regr_update_float,
                                        CV_32F,
                                        1 / 255.0);
        cv::Mat updated_perception_window = ((channel_perception(target_roi)
                                             + regr_update_float)
                                             / 2);
        updated_perception_window.copyTo(channel_perception(target_roi));
        //  regr_update.convertTo(channel_perception(target_roi), CV_32F, 1/255.0);

        //
        // update segmentation channel
        //
        segm_update(src_roi).copyTo(channel_segmentation(target_roi));
    } catch (cv::Exception ex) {
        LOG_ERROR("caught cv exception in onPerceptionDataAvailable");
        fprintf(stderr, "caught cv exception in onPerceptionDataAvailable: %s\n", ex.what());
    }
}

void LocalMap::onObstaclesDetected(const std::vector<Obstacle> *const obstacles, DataSource
source) {
    // updateHitsMisses(channel_obstacles, obstacles);
}

void LocalMap::onLaserScannerUpdate(const std::vector<tPolarCoordiante>& ls_polar,
                                    const std::vector<Point>& ls_global_cartesian) {
    plainLS_obstacle_list = ls_global_cartesian;
}

std::vector<Point> LocalMap::get_plainLS_obstacle_list() {
    return plainLS_obstacle_list;
}


void LocalMap::addLaneCenterPoints(std::vector<cv::Point2f> *center_points, float hitVal) {


    cv::Mat updateMask = cv::Mat(channel_perception.size(), CV_32FC2, cv::Scalar(0, 1));


    updateHitsMisses(channel_perception, *center_points, hitVal);
}


void interpolate(const cv::Mat &channel, vector<cv::Point2f> *center_points, float connect_th =
50) {
    uint pts_size = static_cast<uint>(center_points->size() - 1);


    for (uint idx = 0; idx < pts_size; idx += 1) {
        cv::Point2f curr_center = (*center_points)[idx];
        cv::Point2f next_center = (*center_points)[idx + 1];

        if (sqrt(pow(next_center.x - curr_center.x, 2) +
                 pow(next_center.y - curr_center.y, 2)) > connect_th) {
            continue;
        }

        cv::LineIterator it(channel, curr_center, next_center, 8);
        for (int i = 0; i < it.count; i++, ++it) {
            cv::Point interpolated_pt = it.pos();
            center_points->emplace_back(interpolated_pt);
        }
    }
}


void
computeRecommendedGoal(std::vector<cv::Point2f> *center_points, Point *goal, float threshold = 0.05,
                       float max_local_dist = 0.6) {
    cv::Point2f sign_pt = center_points->front();

    for (auto &pt : *center_points) {

        if (fabs(fabs(sign_pt.x) - fabs(pt.x)) > threshold || pt.y > max_local_dist) {
            if (pt.y > max_local_dist) {
                goal->setX(pt.x);
                goal->setY(pt.y);
                return;
            }
        }
    }
}

void LocalMap::addLanes(std::vector<Lane> *lanes) {
    timing.beginMeasure();

    if (lanes->empty()) {
        return;
    }

    std::vector<cv::Point2f> center_points;

    //only use current detected center lane
    if (perception_mode_single_lane) {
        Lane *left = nullptr, *right = nullptr;

        Lane::extractCurrentRoadLanes(lanes, &left, &right);
        Lane::getLaneCenterPoints(&center_points, left, right, 10);

    } else {
        Lane *left = nullptr;

        for (uint lane_idx = 0; lane_idx < lanes->size(); lane_idx++) {
            Road road;
            if (lane_idx + 1 < lanes->size()) {
                left = &(*lanes)[lane_idx + 1];
            }
            Lane *right = &(*lanes)[lane_idx];

            road.initFrom(left, right);


            Lane::getLaneCenterPoints(&center_points, road.left, road.right);

            Lane::getLaneCenterPoints(&center_points, road.left, nullptr);


            Lane::getLaneCenterPoints(&center_points, road.right, nullptr);


            Lane::getLaneCenterPoints(&center_points, nullptr, road.left);

            Lane::getLaneCenterPoints(&center_points, nullptr, road.right);


        }
    }

    if (center_points.empty()) {
        return;
    }

    Point camera_loc = car_model->getCameraLocation();
    Point new_recomm = Point::Local(0, 0);
    computeRecommendedGoal(&center_points, &new_recomm, 0.09, 0.6);
    if (!new_recomm.isInitPoint()) {
        new_recomm.scaleBy(Coordinate::Type::GLOBAL);
        //recommended_goal = camera_loc.toGlobal(new_recomm, car_model->getHeading());
    }
    for (auto &elem : center_points) {
        elem *= Coordinate::Type::GLOBAL;
        camera_loc.toGlobalCV(elem, car_model->getHeading());
        convertToLocalFrame(&elem);

        //cout << "center pt: " << elem <<endl;
    }
    interpolate(channel_perception, &center_points);
    //addLaneCenterPoints(&center_points);
}


void LocalMap::updateMapChannel(IGlobalMap &globalMap) {
    if(!globalMap.isInitialized()) {
        return;
    }
    cv::Mat globalMapArea = getGlobalMapArea(globalMap);

    cv::Mat hit_miss(globalMapArea.size(), CV_32FC2, cv::Scalar(0, 1));
    std::vector<cv::Point2i> locationsNz;
    cv::findNonZero(globalMapArea, locationsNz);

    for (auto &hit : locationsNz) {
        auto &val = hit_miss.at<cv::Vec2f>(hit);
        val[0] = 1;
        val[1] = 0;
    }

    channel_road_lanes += 1*hit_miss;
    channel_road_lanes /= 2;
}

void LocalMap::updateAndDiscretizeMovement(DeltaPosition delta, int* shift_x, int* shift_y, float* rotation) {

    // delta is old - new
    float local_x_delta_step = -1 * sin(car_model->getHeading()) * delta.dx
                            +cos(car_model->getHeading()) * delta.dy;
    float local_y_delta_step = -1 * cos(car_model->getHeading()) * delta.dx
                            -1 * sin(car_model->getHeading()) * delta.dy;
    float rotation_delta_step = -delta.getHeading();

    // x  (1 pix threshold)
    local_x_delta += local_x_delta_step;
    if (local_x_delta >= 5) {
        (*shift_x) = static_cast<int>(floor(local_x_delta));
        local_x_delta -= (*shift_x);
    }
    if (local_x_delta <= -5) {
        (*shift_x) = static_cast<int>(ceil(local_x_delta));
        local_x_delta -= (*shift_x);
    }
    // y  (1 pix threshold)
    local_y_delta += local_y_delta_step;
    if (local_y_delta >= 5) {
        (*shift_y) = static_cast<int>(floor(local_y_delta));
        local_y_delta -= (*shift_y);
    }
    if (local_y_delta <= -5) {
        (*shift_y) = static_cast<int>(ceil(local_y_delta));
        local_y_delta -= (*shift_y);
    }
    // roation  (+/- 1 degree threshold, no discretization)
    rotation_delta += rotation_delta_step;
    if (rotation_delta >= (2 * static_cast<float>(DEG2RAD))) {
        (*rotation) = rotation_delta;
        rotation_delta = 0;
    }
    if (rotation_delta <= (-2 * static_cast<float>(DEG2RAD))) {
        (*rotation) = rotation_delta;
        rotation_delta = 0;
    }
}


void LocalMap::onUpdateGlobalPosition(IGlobalMap &globalMap) {
    // StopWatch watch(true, true);
    try {
        DeltaPosition delta = car_model->getLastPositionDelta();
        int shift_x = 0;
        int shift_y = 0;
        float rotation = 0;
        updateAndDiscretizeMovement(delta, &shift_x, &shift_y, &rotation);

        if (shift_x != 0 || shift_y != 0) {
            // LOG_INFO("shift, x %i, y %i", shift_x, shift_y);
            // shift the map inverse to the driving direction
            // translateImg(channel_road_lanes, -shift_x, -shift_y);
            // translateImg(channel_obstacles, -shift_x, -shift_y);
            translateImg(channel_perception, -shift_x, -shift_y);
            // translateImg(channel_segmentation, -shift_x, -shift_y);
        }
        if (rotation != 0) {
            // LOG_INFO("rotate %f", rotation * RAD2DEG);
            // rotate the map inverse to the rotation of the car
            cv::Point2f center = cv::Point2f(localCarPosition.getX(), localCarPosition.getY());
            // rotateImg(channel_road_lanes, -rotation, center);
            // rotateImg(channel_obstacles, -rotation, center);
            rotateImg(channel_perception, -rotation * RAD2DEG, center);
            // rotateImg(channel_segmentation, -rotation * RAD2DEG, center);
        }
        // updateMapChannel(globalMap);

        // // cv::GaussianBlur(channel_road_lanes, channel_road_lanes, cv::Size(3, 3), 3, 3);
        // watch.print_measurement("upd pos");
    } catch (cv::Exception ex) {
        LOG_ERROR("caught cv exception in onUpdateGlobalPosition");
        fprintf(stderr, "caught cv exception in onUpdateGlobalPosition: %s\n", ex.what());
    }
}


cv::Mat LocalMap::getGlobalMapArea(IGlobalMap &globalMap) {

    // cv::Mat area_fullsize = cv::Mat(channel_road_lanes.size(), CV_8UC1, cv::Scalar(0));
    float car_x = car_model->getFrontAxis().getX();
    float car_y = -car_model->getFrontAxis().getY();
    // cv::Rect roi = cv::Rect(0, localCarPosition.getY(),
    //     window_width, window_height - localCarPosition.getY());

    cv::Mat area = Map::getInstance()->getCenteredMapWindow(window_width, window_height,
                                                            car_x, car_y).clone();
    // cv::Mat area = Map::getInstance()->getDiscretizedMapArea(
    //     window_width, window_height - localCarPosition.getY(),
    //     car_x, car_y, -1*car_model->getHeadingDegrees()).clone();
    // area.copyTo(area_fullsize(roi));
    MatUtils::rotateImg(&area, car_model->getHeadingDegrees(),
        Point::Global(area.cols / 2, area.rows / 2));
    translateImg(area, 0,
        (area.size().height - (localCarPosition.getY() + car_model->getLocalFront(pixel_per_meter).y)));
    return area;
}

void LocalMap::printCostMap(const cv::Mat &channel) {
    for (int i = 0; i < channel.rows; i++) {
        for (int j = 0; j < channel.cols; j++) {
            auto val = costAt(i, j);
            std::cout << std::fixed << std::setprecision(1) << val << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}


cv::Mat LocalMap::toRgbImgMatrix() {
    cv::Mat target = cv::Mat(channel_road_lanes.size(), CV_8UC3);
    cv::Mat img_channel_road_lanes, img_channel_center_lanes,
            img_channel_obstacles;


    channel_road_lanes.convertTo(img_channel_road_lanes, CV_8U, 255, 0);
    channel_perception.convertTo(img_channel_center_lanes, CV_8U, 255, 0);
    channel_obstacles.convertTo(img_channel_obstacles, CV_8U, 255, 0);

    std::vector<cv::Mat> channels = {img_channel_obstacles,
                                     img_channel_center_lanes,
                                     img_channel_road_lanes
    };
    cv::merge(channels, target);

    return target;
}

cv::Mat LocalMap::toSoftMaxedCellProbImgMatrix(float proximity_radius) {
    cv::Mat target = cv::Mat(channel_perception.size(), CV_32FC3);

    for (int i = 0; i < target.rows; i++) {
        for (int j = 0; j < target.cols; j++) {
            target.at<cv::Vec3f>(i, j) = computeSoftmaxAt(i, j);
        }
    }

    target.convertTo(target, CV_8UC3, 255);
    // cv::Mat out;
    // channel_perception.convertTo(out, CV_8UC3);   // made problems!
    // return out;
    return target;
}

/*
LocalMap::SubGoal LocalMap::getLocalGoal() {

    return SubGoal();

    timing.beginMeasure();
    float heading = localCarPosition.getHeading();

    int lookahead_x = static_cast<int>(ceil(localCarPosition.getX() + (channel_perception.rows /
            2) * cos(heading)));
    int lookahead_y = static_cast<int>(ceil(localCarPosition.getY() + (channel_perception.cols /
            2) * sin(heading)));
    cv::Mat log_probs = computeCenterLaneProbabilities();
    //cv::Mat dist_bonus = getLocalGoalMask(lookahead_x, lookahead_y);

    //cv::log(dist_bonus, dist_bonus);
    //log_probs = log_probs+dist_bonus;



    double min, max;

    cv::Rect roi(static_cast<int>(localCarPosition.getX()), static_cast<int>
    (localCarPosition.getY()), lookahead_x, lookahead_y);
    cout << "lookaheads : " << roi <<endl;


    roi = roi & cv::Rect(0,0, log_probs.cols, log_probs.rows);
    cv::Mat roi_mat = log_probs(roi);

    cv::Point min_loc, max_loc;


    cv::minMaxLoc(roi_mat, &min, &max, &min_loc, &max_loc);
    SubGoal goal;
    goal.point = Point::Global(max_loc.y + roi.x, max_loc.x + roi.y);
    goal.val = static_cast<float>(max);
    cout << "goal" << goal.point <<endl;
    timing.print_measurement("computing local goal");
    return goal;
}*/

cv::Mat LocalMap::getLocalGoalMask(int l_x, int l_y) {

    cv::Mat mask(channel_road_lanes.size(), CV_8U, cv::Scalar(1));
    cv::Point2f center(0, 10);
    localCarPosition.toGlobalCV(center, car_model->getHeading());


    auto it = cv::LineIterator(mask, center,
                               cv::Point(l_x, l_y));

    for (int i = 0; i < it.count; i++, ++it) {
        cv::Point pt = it.pos();
        mask.at<uchar>(pt.x, pt.y) = 0;
    }

    cv::Mat distMat;
    cv::distanceTransform(mask, distMat, cv::DIST_L1, cv::DIST_MASK_3);


    cv::normalize(distMat, distMat, 0, 0.6, cv::NORM_MINMAX);
    distMat = 1 - distMat;
    return distMat;
}

cv::Mat LocalMap::computeCenterLaneProbabilities() {
    vector<cv::Mat> channels;
    cv::split(channel_perception, channels);
    cv::Mat p_perception;
    cv::log((channels[0] + 1) / ((channels[0] + 1) + (channels[1])), p_perception);

    cv::split(channel_road_lanes, channels);
    cv::Mat p_road_log;
    cv::log((channels[0] + 1) / ((channels[0] + 1) + (channels[1])), p_road_log);
    cv::Mat added_probs = p_perception + p_road_log;
    added_probs.convertTo(added_probs, CV_32F);
    return added_probs;
}

cv::Mat LocalMap::computeCostMap() {
    cv::Size size = channel_road_lanes.size();
    cv::Mat cost = cv::Mat(size, CV_32F);
    for (int i = 0; i < cost.rows; i++) {
        for (int j = 0; j < cost.cols; j++) {
            cost.at<float>(i, j) = costAt(i, j);
        }
    }
    return cost;
}
