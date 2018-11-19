
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

#include "LightCtrlPin.h"
#include "ADTF3_helper.h"


LightCtrlPin::LightCtrlPin() {   }

tResult LightCtrlPin::registerPin(cTriggerFunction* p_filter,
                               const object_ptr<adtf::services::IReferenceClock>* p_clock) {
    if (this->was_registered == true) {
        LOG_ERROR("Trying to register pin twice.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }
    this->filter = p_filter;
    this->m_pClock = p_clock;

    object_ptr<IStreamType> pTypeBoolSignalValue;
    if (ERR_NOERROR ==
        adtf::mediadescription::ant::
        create_adtf_default_stream_type_from_service("tBoolSignalValue",
                                                     pTypeBoolSignalValue,
                                                     m_inputBoolSignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(
            m_inputBoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlBoolSignalValueId.ui32ArduinoTimestamp));
        (adtf_ddl::access_element::find_index(
            m_inputBoolSignalValueSampleFactory, cString("bValue"), m_ddlBoolSignalValueId.bValue));
    } else {
        LOG_INFO("No mediadescription for tRoadSignData found!");
    }

    this->filter->Register(m_oWriterHeadlights, "lights_head", pTypeBoolSignalValue);
    this->filter->Register(m_oWriterIndicatorLeft, "lights_ind_left", pTypeBoolSignalValue);
    this->filter->Register(m_oWriterIndicatorRight, "lights_ind_right", pTypeBoolSignalValue);
    this->filter->Register(m_oWriterBrakeLight, "lights_brake", pTypeBoolSignalValue);
    this->filter->Register(m_oWriterHazardLights, "lights_hazard", pTypeBoolSignalValue);
    this->filter->Register(m_oWriterReverseLights, "lights_reverse", pTypeBoolSignalValue);

    this->was_registered = true;
    RETURN_NOERROR;
}

tResult LightCtrlPin::writeData(cPinWriter *writer, const tBoolSignalValue &data) {
    if (!this->was_registered) {
        LOG_ERROR("Trying to write but not registered or not configured to write.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }
    return transmitBoolSignalValue(*writer, (*(this->m_pClock))->GetStreamTime(),
        m_inputBoolSignalValueSampleFactory, m_ddlBoolSignalValueId.ui32ArduinoTimestamp,
        0, m_ddlBoolSignalValueId.bValue, data.bValue);
}

bool LightCtrlPin::setIndicatorRightEnabled(bool enable) {
    //LOG_INFO("Ind right enabled: %d", enable);
    tBoolSignalValue data;
    data.ui32ArduinoTimestamp = (*(this->m_pClock))->GetStreamTime();
    //data.ui32ArduinoTimestamp = 0;
    data.bValue = enable;
    if (debug_log_enabled) {
        LOG_INFO("Enable ind right %d", enable);
    }
    return IS_OK(writeData(&m_oWriterIndicatorRight, data));
}

bool LightCtrlPin::setIndicatorLeftEnabled(bool enable) {
    //LOG_INFO("Ind left enabled: %d", enable);
    tBoolSignalValue data;
    data.ui32ArduinoTimestamp = (*(this->m_pClock))->GetStreamTime();
    //data.ui32ArduinoTimestamp = 0;
    data.bValue = enable;
    if (debug_log_enabled) {
        LOG_INFO("Enable ind left %d", enable);
    }
    return IS_OK(writeData(&m_oWriterIndicatorLeft, data));
}

bool LightCtrlPin::setHazardLightsEnabled(bool enable) {
    tBoolSignalValue data;
    data.ui32ArduinoTimestamp = (*(this->m_pClock))->GetStreamTime();
    //data.ui32ArduinoTimestamp = 0;
    data.bValue = enable;
    if (debug_log_enabled) {
        LOG_INFO("Enable Hazard lights %d", enable);
    }
    return IS_OK(writeData(&m_oWriterHazardLights, data));
}
bool LightCtrlPin::setBrakeLightsEnabled(bool enable) {
    //LOG_INFO("Brake light enabled: %d", enable);
    tBoolSignalValue data;
    data.ui32ArduinoTimestamp = (*(this->m_pClock))->GetStreamTime();
    //data.ui32ArduinoTimestamp = 0;
    data.bValue = enable;
    if (debug_log_enabled) {
        LOG_INFO("Enable brake lights %d", enable);
    }
    
    return IS_OK(writeData(&m_oWriterBrakeLight, data));
}

bool LightCtrlPin::setHeadLightsEnabled(bool enable) {
    tBoolSignalValue data;
    data.ui32ArduinoTimestamp = (*(this->m_pClock))->GetStreamTime();
    //data.ui32ArduinoTimestamp = 0;
    data.bValue = enable;
    if (debug_log_enabled) {
        LOG_INFO("Enable headlights %d", enable);
    }
    return IS_OK(writeData(&m_oWriterHeadlights, data));
}

bool LightCtrlPin::setReverseLightsEnabled(bool enable){
    tBoolSignalValue data;
    data.ui32ArduinoTimestamp = (*(this->m_pClock))->GetStreamTime();
    //data.ui32ArduinoTimestamp = 0;
    data.bValue = enable;
    if (debug_log_enabled) {
        LOG_INFO("Enable reverse lights %d", enable);
    }
    return IS_OK(writeData(&m_oWriterReverseLights, data));
}
