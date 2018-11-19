
/******************************************************************************
 * Audi Autonomous Driving Cup 2018
 * Team frAIsers
******************************************************************************/

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include "stdafx.h"
#include "../MapCore/Map.h"
#include "IGlobalMap.h"

#define CID_WAYPOINT_TEST_FILTER "waypointtest.user.adtf.cid"
#define LABEL_WAYPOINT_TEST_FILTER "Waypoint Test Filter"

class WaypointTestFilter : public cTriggerFunction {
 public:
    WaypointTestFilter();
    virtual ~WaypointTestFilter();

    ADTF_CLASS_ID_NAME(WaypointTestFilter, CID_WAYPOINT_TEST_FILTER, LABEL_WAYPOINT_TEST_FILTER);
    ADTF_CLASS_DEPENDENCIES(REQUIRE_INTERFACE(adtf::services::IKernel),
                            REQUIRE_INTERFACE(adtf::services::IReferenceClock));


 private:
    property_variable<tBool> m_propStub = tTrue;
    property_variable<tFloat64> m_propX = 0;
    property_variable<tFloat64> m_propY = 0;
    property_variable<tFloat64> m_propZ = 0;
    property_variable<tFloat64> m_propT = 0;
    property_variable<tFloat64> m_propGoalDistance = 20;
    property_variable<tBool> m_propIgnoreStoplines = tTrue;
    property_variable<tBool> m_propIgnoreParking = tTrue;
    property_variable<tBool> m_propIgnoreCrosswalks = tTrue;
    property_variable<tBool> m_propPlanRoute = tFalse;
    property_variable<tFloat64> m_propX_goal = 0;
    property_variable<tFloat64> m_propY_goal = 0;
    property_variable<tFloat64> m_propZ_goal = 0;
    property_variable<tFloat64> m_propT_goal = 0;

    tUInt8 test;

    object_ptr<adtf::services::IReferenceClock> m_pClock;

    bool is_finished;
    Map* _map = nullptr;

 public:
    tResult Configure() override;
    tResult Process(tTimeStamp tmTimeOfTrigger) override;
};
