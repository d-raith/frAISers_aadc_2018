
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

#include "DriverStructPin.h"
#include "iostream"


DriverStructPin::DriverStructPin() = default;

tResult DriverStructPin::registerPin(RW p_rw,
                                     cTriggerFunction* p_filter,
                                     const object_ptr<adtf::services::IReferenceClock>* p_clock,
                                     const tChar* pin_name) {
    if (this->was_registered) {
        LOG_ERROR("Trying to register pin twice.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }
    this->rw = p_rw;
    this->filter = p_filter;
    this->m_pClock = p_clock;

    if(!m_pClock){
        throw runtime_error("pclock not defined");
    }

    object_ptr<IStreamType> pTypeDriverStruct;
    if (ERR_NOERROR ==
        adtf::mediadescription::ant::
        create_adtf_default_stream_type_from_service("tDriverStruct",
                                                     pTypeDriverStruct,
                                                     m_driverStructSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16StateID"), m_ddlDriverStructId.stateId));
        (adtf_ddl::access_element::find_index(m_driverStructSampleFactory, cString("i16ManeuverEntry"), m_ddlDriverStructId.maneuverEntry));

    } else {
        LOG_INFO("No mediadescription for tJuryStruct found!");
    }

    switch (rw) {
        case RW::READ: {
            this->filter->Register(m_oReader, pin_name, pTypeDriverStruct);
            break;
        }
        case RW::WRITE: {
            this->filter->Register(m_oWriter, pin_name, pTypeDriverStruct);
            break;
        }
        default: LOG_ERROR("Unknown enum value.");
    }

    this->was_registered = true;
    RETURN_NOERROR;
}


tResult DriverStructPin::writeData(tDriverStruct *driverStruct) {

    if (!this->was_registered || this->rw == RW::READ) {
        LOG_ERROR("Trying to write but not registered or not configured to write.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    object_ptr<ISample> pWriteSample;

    tResult r = alloc_sample(pWriteSample, (*(this->m_pClock))->GetStreamTime());

    RETURN_IF_FAILED(r) {
        auto oCodec = m_driverStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.IsValid());
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.stateId, driverStruct->i16StateID));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDriverStructId.maneuverEntry,
                driverStruct->i16ManeuverEntry));
    }

    m_oWriter << pWriteSample << flush << trigger;

    RETURN_NOERROR;
}

tResult DriverStructPin::readData(tDriverStruct *data) {
    if (!this->was_registered || this->rw == RW::WRITE) {
        LOG_ERROR("Trying to read but not registered or not configured to read.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }


    object_ptr<const ISample> pReadSample;
    tResult result = m_oReader.GetNextSample(pReadSample);
    if (!IS_OK(result)) {
        return result;
    }
    auto oDecoder = m_driverStructSampleFactory.MakeDecoderFor(*pReadSample);
    RETURN_IF_FAILED(oDecoder.IsValid());
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.stateId, &data->i16StateID));
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDriverStructId.maneuverEntry, &data->i16ManeuverEntry));
    RETURN_NOERROR;
}

