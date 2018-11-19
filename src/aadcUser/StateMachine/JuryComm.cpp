//
// Created by aadc on 23.09.18.
//

/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/
//otherwise cDOM will cause a deprecated warning, however there is no alternative yet
#define A_UTILS_NO_DEPRECATED_WARNING

#include "JuryComm.h"

using namespace aadc::jury;

JuryCommunication::JuryCommunication(JuryCommandListener *cmd_listener) : jury_listener
                                                                                  (cmd_listener) {

}

JuryCommunication::~JuryCommunication() {

};


tResult JuryCommunication::registerPins(cTriggerFunction *filter, const
object_ptr<adtf::services::IReferenceClock> *p_clock) {

    this->m_pClock = *p_clock;
    driverStructPin.registerPin(DriverStructPin::RW::WRITE, filter, p_clock,
                                "jc_driver_struct_out");

    juryStructPin.registerPin(JuryStructPin::RW::READ, filter, p_clock,
                              "jc_jury_struct_in");
    maneuverListPin.registerPin(ManeuverListPin::RW::READ, filter, p_clock,
                                "jc_man_list_in");


    return ERR_NOERROR;
}


tResult JuryCommunication::checkUpdates() {

    tJuryStruct jury_cmd;
    if (IS_OK(juryStructPin.readData(&jury_cmd))) {
        handleJuryCommand(&jury_cmd);
        LOG_INFO("Jury command received");
    }

    cString man_data_str;
    if (IS_OK(maneuverListPin.readData(&man_data_str))) {
        if (!IS_OK(loadManeuverList(man_data_str))) {
            LOG_INFO("maneuver list received");
            CONSOLE_LOG_INFO("Unable to parse maneuver list");
        };
    }

    RETURN_NOERROR;
}


void JuryCommunication::handleJuryCommand(tJuryStruct *juryStruct) {
    tInt8 i8ActionID = juryStruct->i16ActionID;
    tInt16 i16entry = juryStruct->i16ManeuverEntry;


    switch (i8ActionID) {
        case action_getready:
            CONSOLE_LOG_INFO(
                    cString::Format("Driver Module: Received Request Ready with maneuver ID %d",
                                    i16entry));
            jury_listener->onGetReady(static_cast<int>(i16entry));
            break;
        case action_start:
            CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Run with maneuver ID %d",
                                             i16entry));
            jury_listener->onStartManeuver(static_cast<int>(i16entry));
            break;
        case action_stop:
            CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d",
                                             i16entry));
            jury_listener->onStop(static_cast<int>(i16entry));
            break;
        default:
            CONSOLE_LOG_INFO(cString::Format("Driver Module: Received Stop with maneuver ID %d",
                                             i16entry));
    }
}


tResult JuryCommunication::loadManeuverList(const cString &man_str) {

    // create dom from string received from pin
    cDOM oDOM;
    oDOM.FromString(man_str);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;
    aadc::jury::maneuverList m_sectorList;

    //read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems))) {
        m_sectorList.clear();
        //iterate through sectors
        for (auto &oSectorElem : oSectorElems) {
            //if sector found
            tSector sector;
            sector.id = oSectorElem->GetAttributeUInt32("id");

            if (IS_OK(oSectorElem->FindNodes("AADC-Maneuver", oManeuverElems))) {
                //iterate through maneuvers
                for (auto &oManeuverElem : oManeuverElems) {
                    tManeuver man;
                    man.id = oManeuverElem->GetAttributeUInt32("id");
                    man.action = maneuverFromString(
                            oManeuverElem->GetAttribute("action").GetPtr());
                    man.extra = oManeuverElem->GetAttributeUInt32("extra");
                    sector.sector.push_back(man);

                }
            }

            m_sectorList.push_back(sector);


        }
    }
    for (auto sec: m_sectorList){
        for (auto man : sec.sector){
            LOG_INFO("Sec: %d, Man: %d, %d, extra: %d", sec.id, man.id, man.action, man.extra);
        }

    }
    if (!m_sectorList.empty()) {
        LOG_INFO("DriverFilter: Loaded Maneuver file successfully.");
        jury_listener->onManeuverListAvailable(m_sectorList);

    } else {
        LOG_ERROR("DriverFilter: no valid Maneuver Data found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}

tResult JuryCommunication::sendState(stateCar stateID, tInt16 i16ManeuverEntry) {


    tDriverStruct driverStruct;
    driverStruct.i16StateID = stateID;
    driverStruct.i16ManeuverEntry = i16ManeuverEntry;

    if (m_propEnableConsoleOutput) {
        switch (stateID) {
            case statecar_ready:
                LOG_INFO(cString::Format("Driver Module: Send state: READY, Maneuver ID %d",
                                         i16ManeuverEntry));
                break;
            case statecar_running:
                LOG_INFO(cString::Format("Driver Module: Send state: RUNNING, Maneuver ID %d",
                                         i16ManeuverEntry));
                break;
            case statecar_complete:
                LOG_INFO(cString::Format("Driver Module: Send state: COMPLETE, Maneuver ID %d",
                                         i16ManeuverEntry));
                break;
            case statecar_error:
                LOG_INFO(cString::Format("Driver Module: Send state: ERROR, Maneuver ID %d",
                                         i16ManeuverEntry));
                break;
            case statecar_startup:
                LOG_INFO(cString::Format("Driver Module: Send state: STARTUP, Maneuver ID %d",
                                         i16ManeuverEntry));
                break;
        }
    }


    if(!IS_OK(driverStructPin.writeData(&driverStruct))) {
        LOG_INFO("Unable to send driver struct!");
    }
    LOG_INFO("Sent state to jury successfully: %d for maneuver %d", stateID, i16ManeuverEntry);


    RETURN_NOERROR;
}

/**maneuver / task state transmission*/
tResult JuryCommunication::onManeuverRunning(BaseTask *man) {
    return onManeuverRunning(man->getId());
}

tResult JuryCommunication::onManeuverRunning(int manId) {
    return sendState(stateCar::statecar_running, manId);
}


tResult JuryCommunication::onManeuverError(BaseTask *man) {
    return onManeuverError(man->getId());
}

tResult JuryCommunication::onManeuverError(int manId) {
    return sendState(stateCar::statecar_error, manId);
}

/**Global car state transmission*/

tResult JuryCommunication::onCarReady(BaseTask *man) {
    return onCarReady(man->getId());
}

tResult JuryCommunication::onCarReady(int manId) {
    return sendState(stateCar::statecar_ready, manId);
}

tResult JuryCommunication::onCarStartup(BaseTask *man) {
    return onCarStartup(man->getId());
}

tResult JuryCommunication::onCarStartup(int manId) {
    return sendState(stateCar::statecar_startup, manId);
}

tResult JuryCommunication::onCarFinishedExecution(BaseTask *man) {
    return onCarFinishedExecution(man->getId());
}

tResult JuryCommunication::onCarFinishedExecution(int manId) {
    return sendState(stateCar::statecar_complete, manId);
}

