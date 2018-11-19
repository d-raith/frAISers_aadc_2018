
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

#include "ManeuverListPin.h"



ManeuverListPin::ManeuverListPin() = default;

tResult ManeuverListPin::registerPin(RW p_rw,
                                     cTriggerFunction *p_filter,
                                     const object_ptr<adtf::services::IReferenceClock> *p_clock,
                                     const tChar *pin_name) {
    if (this->was_registered) {
        LOG_ERROR("Trying to register pin twice.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }
    this->rw = p_rw;
    this->filter = p_filter;
    this->m_pClock = p_clock;


    switch (rw) {
        case RW::READ: {
            object_ptr<IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<cStreamType>(
                    stream_meta_type_anonymous());
            this->filter->Register(m_oReader, pin_name, pTypeDefault);
            break;
        }
        default:
            LOG_ERROR("Unknown enum value.");
    }

    this->was_registered = true;
    RETURN_NOERROR;
}


tResult ManeuverListPin::readData(cString *man_list_str) {
    if (!this->was_registered) {
        LOG_ERROR("Trying to read but not registered or not configured to read.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }


    object_ptr<const ISample> pSampleAnonymous;
    while (IS_OK(m_oReader.GetNextSample(pSampleAnonymous))) {

        std::vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (!data.empty()) {//maneuverlist
            man_list_str->Set(data.data(), data.size());
            RETURN_NOERROR;
        }
    }

    RETURN_ERROR(ERR_NOT_READY);
}

