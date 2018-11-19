//
// Created by aadc on 23.09.18.
//

#ifndef AADC_USER_TASKFACTORY_H
#define AADC_USER_TASKFACTORY_H
#pragma once
#include "BaseTask.h"
#include "stdafx.h"
#include "PlanningTask.h"
#include "WaypointTask.h"
#include "ReverseDrivingTask.h"
#include "ParkingTask.h"
#include "ParkOutTask.h"
#include "LaneFollowingTask.h"
#include "TurnRightTask.h"
#include "LaneFollowingPerceptionTask.h"
#include "DriveStraightTask.h"
#include "MergeTask.h"
#include "TurnLeftTask.h"
#include "Point.h"


class TaskFactory {
public:
    static BaseTask::Type convertAADCType(aadc::jury::maneuver man) {
        switch (man) {
            case aadc::jury::manuever_undefined:
                return BaseTask::Type::undefined;
            case aadc::jury::maneuver_left:
                return BaseTask::Type::turn_left;
            case aadc::jury::maneuver_right:
                return BaseTask::Type::turn_right;
            case aadc::jury::maneuver_straight:
                return BaseTask::Type::straight;
            case aadc::jury::maneuver_parallel_parking:
                return BaseTask::Type::parallel_parking;
            case aadc::jury::maneuver_cross_parking:
                return BaseTask::Type::cross_parking;
            case aadc::jury::maneuver_pull_out_left:
                return BaseTask::Type::pull_out_left;
            case aadc::jury::maneuver_pull_out_right:
                return BaseTask::Type::pull_out_right;
            case aadc::jury::maneuver_merge_left:
                return BaseTask::Type::merge_left;
            case aadc::jury::maneuver_merge_right:
                return BaseTask::Type::merge_right;
            default:
                throw std::runtime_error("invalid maneuver");
        }
    }



    static void generatePath(int extra, vector<Point> *front_move_out, vector<Point> *rev_move_out) {
        const ParkingSpace *pspace = RoadSignInfo::getInstance()->getParkingSpace(extra);

        if (!pspace) {
            LOG_INFO("Unable to obtain parking lot with id: %d", extra);
            return;
        }

        int park_id = extra;

        float park_y = pspace->f32Y;
        float park_x = pspace->f32X;
        float park_heading = pspace->f32Direction;
        float heading = 3.1428 * (park_heading/180);
        park_x*=100;
        park_y*=100;

        LOG_INFO("Parking Heading is: %f", heading);

        if (heading > -0.785398 && heading < 0.785398) {
            for (int j = park_y + 25; j >= park_y + 10; j = j - 1) {
                front_move_out->emplace_back(Point::Global(park_x + 25, j));
            }
            front_move_out->emplace_back(Point::Global(park_x + 75, park_y - 25));
            front_move_out->emplace_back(Point::Global(park_x + 75, park_y - 40));
            rev_move_out->emplace_back(Point::Global(park_x+25,park_y-25));

            for (int d = park_x-30 ; d >= park_x - 70; d = d - 1) {
                rev_move_out->emplace_back(Point::Global(d,park_y));
        }
        }

        

        else if (fabs(heading) > 2.356) {
            for (int j = park_y - 25; j <= park_y - 10; j = j + 1) {
                front_move_out->emplace_back(Point::Global(park_x - 25, j));
            }
            front_move_out->emplace_back(Point::Global(park_x - 75, park_y + 25));
            front_move_out->emplace_back(Point::Global(park_x - 75, park_y + 40));
            rev_move_out->emplace_back(Point::Global(park_x-25,park_y+25));

            for (int d = park_x+30 ; d <= park_x + 70; d = d + 1) {
                rev_move_out->emplace_back(Point::Global(d,park_y));
        }

        }

        else if (heading > 0.785398 && heading < 2.356) {
            for (int j = park_x - 25; j <= park_x - 10; j = j + 1) {
                front_move_out->emplace_back(Point::Global(j, park_y +25));
            }
            front_move_out->emplace_back(Point::Global(park_x + 25, park_y + 75));
            front_move_out->emplace_back(Point::Global(park_x + 40, park_y + 75));
            rev_move_out->emplace_back(Point::Global(park_x+25,park_y+25));

            for (int d = park_y-30 ; d >= park_y -70; d = d - 1) {
                rev_move_out->emplace_back(Point::Global(park_x,d));
        }

        }
        else  {
            for (int j = park_x + 25; j >= park_x + 10; j = j - 1) {
                front_move_out->emplace_back(Point::Global(j, park_y -25));
            }
            front_move_out->emplace_back(Point::Global(park_x - 25, park_y - 75));
            front_move_out->emplace_back(Point::Global(park_x - 40, park_y - 75));
            rev_move_out->emplace_back(Point::Global(park_x-25,park_y-25));

            for (int d = park_y+30 ; d <= park_y +70; d = d + 1) {
                rev_move_out->emplace_back(Point::Global(park_x,d));
        }

        }



        for (auto &p: *front_move_out){
            LOG_INFO("Front move out: %f %f", p.getX(), p.getY());
        }

         for (auto &p: *rev_move_out){
            LOG_INFO("Front move out: %f %f", p.getX(), p.getY());
        }


    }


   static void makeTask(int id, BaseTask::Type type, int extra,
                                  vector<std::shared_ptr<BaseTask>> *out) {
        switch (type) {
            case BaseTask::Type::undefined:
                throw std::runtime_error("No idea how to handle undefined task");
            case BaseTask::Type::cross_parking:{
                const ParkingSpace *pspace = RoadSignInfo::getInstance()->getParkingSpace(extra);

                if (!pspace) {
                    LOG_INFO("Unable to obtain parking lot with id: %d", extra);
                    return;
                }

                int park_id = extra;

                float park_y = pspace->f32Y;
                float park_x = pspace->f32X;
                float park_heading = pspace->f32Direction;
                float heading = 3.1428 * (park_heading/180);
                park_x*=100;
                park_y*=100;
                if (fabs(heading) > 2.356) {
                    auto follow = std::make_shared<LaneFollowingPerceptionTask>(id, true, Point::Global(park_x-25, park_y-25));
                    out->emplace_back(follow);
                    
                }
                else if (heading > -0.785398 && heading < 0.785398){
                    auto follow = std::make_shared<LaneFollowingPerceptionTask>(id, true, Point::Global(park_x+25, park_y+25));
                    out->emplace_back(follow);
                    

                }
                else if(heading > 0.785398 && heading < 2.356){
                    auto follow = std::make_shared<LaneFollowingPerceptionTask>(id, true, Point::Global(park_x-25, park_y+25));
                    out->emplace_back(follow);

                }

                else{
                    auto follow = std::make_shared<LaneFollowingPerceptionTask>(id, true, Point::Global(park_x+25, park_y-25));
                    out->emplace_back(follow);

                }
                
                //follow->addMarkerTrigger(Marker::MarkerType::PARKING_AHEAD, 0.3);
                
                //TODO: add lane follow?!
                vector<Point> rev_move;
                vector<Point> front_move;
                generatePath(extra, &front_move, &rev_move);
                out->emplace_back(std::make_shared<WaypointTask>(id, front_move));
                out->emplace_back(std::make_shared<ReverseDrivingTask>(id, rev_move));
                //auto wp_driving = std::make_shared<ParkingTask>(id, extra);
                break;
            }




            case BaseTask::Type::pull_out_right:
                out->emplace_back(std::make_shared<ParkOutTask>(id, type));
                break;

            case BaseTask::Type::pull_out_left:
                out->emplace_back(std::make_shared<ParkOutTask>(id, type));
                break;

            case BaseTask::Type::turn_right:
                out->emplace_back(std::make_shared<LaneFollowingPerceptionTask>(id, false));
                out->emplace_back(std::make_shared<TurnRightTask>(id));
                LOG_INFO("Add turning task");
                break;

            case BaseTask::Type::turn_left:
                out->emplace_back(std::make_shared<LaneFollowingPerceptionTask>(id, false));
                out->emplace_back(std::make_shared<TurnLeftTask>(id));
                break;

            case BaseTask::Type::straight:
                out->emplace_back(std::make_shared<LaneFollowingPerceptionTask>(id, false));
                out->emplace_back(std::make_shared<DriveStraightTask>(id));
                break;

            case BaseTask::Type::merge_left:
                out->emplace_back(std::make_shared<MergeTask>(id, BaseTask::Type::merge_left));
                break;


            default:
                LOG_INFO("Unhandled maneuver: %d, placeholder lane follow is added", type);
                out->emplace_back(std::make_shared<LaneFollowingPerceptionTask>(id, false));


        }



    }

};

#endif //AADC_USER_TASKFACTORY_H
