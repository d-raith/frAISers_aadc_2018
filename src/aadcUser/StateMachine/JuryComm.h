//
// Created by aadc on 23.09.18.
//

#ifndef AADC_USER_JURYCOMM_H
#define AADC_USER_JURYCOMM_H
#pragma once
#include "stdafx.h"
#include "BaseTask.h"
#include <a_utils/core/a_utils_core.h>
#include "../PinClasses/ManeuverListPin.h"
#include "../PinClasses/JuryStructPin.h"
#include "../PinClasses/DriverStructPin.h"

using namespace adtf::base;
using namespace adtf::filter;
using namespace adtf::mediadescription;
using namespace adtf::util;



#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages
class JuryCommandListener {
public:
    virtual void onGetReady(int manId) = 0;
    virtual void onStartManeuver(int manId) = 0;
    virtual void onStop(int manId) = 0;
    virtual void onManeuverListAvailable(aadc::jury::maneuverList sectorList) = 0;
};

class JuryCommunication {

    /*! The property TCP port */

    property_variable<tBool>    m_propEnableConsoleOutput = true;

    /*! The reference clock */
    adtf::ucom::object_ptr<adtf::services::IReferenceClock> m_pClock;


    ManeuverListPin maneuverListPin;
    DriverStructPin driverStructPin;
    JuryStructPin juryStructPin;

    /*! this is the list with all the loaded sections from the maneuver list*/


    JuryCommandListener *jury_listener;


    void handleJuryCommand(tJuryStruct *juryStruct);

    tResult loadManeuverList(const cString &man_str);
public:
    /*! Default constructor. */
    JuryCommunication(JuryCommandListener *cmd_listener);
    /*! Destructor. */
    ~JuryCommunication();


    tResult registerPins(cTriggerFunction* filter, const object_ptr<adtf::services::IReferenceClock>* p_clock);


    tResult checkUpdates();


    tResult onTimer();

    tResult loadManeuverList();

    tResult onGlobalStateReady();
    tResult onGlobalStateComplete();

    tResult onCarReady(BaseTask *man);
    tResult onCarStartup(BaseTask *man);
    tResult onManeuverRunning(BaseTask* man);
    tResult onManeuverError(BaseTask* man);
    tResult onCarFinishedExecution(BaseTask *man);

    tResult onCarReady(int manId);
    tResult onCarStartup(int manId);
    tResult onManeuverRunning(int manId);
    tResult onManeuverError(int manId);
    tResult onCarFinishedExecution(int manId);


    tResult sendState(aadc::jury::stateCar stateID, tInt16 i16ManeuverEntry);

};

#endif //AADC_USER_JURYCOMM_H
