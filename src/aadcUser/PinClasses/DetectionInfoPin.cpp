
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

#include "DetectionInfoPin.h"
#include "iostream"


tResult DetectionInfoPin::registerPin(RW p_rw,
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

    object_ptr<IStreamType> pTypeObsData;
    if (ERR_NOERROR ==
        adtf::mediadescription::ant::
        create_adtf_default_stream_type_from_service("tDetectionInfo",
                                                     pTypeObsData,
                                                     m_ObsDataFactory)) {
        (adtf_ddl::access_element::find_index(m_ObsDataFactory,
                                              cString("f32pCarLeft"),
                                              m_ddlDetectionInfoId.f32pCarLeft));
        (adtf_ddl::access_element::find_index(m_ObsDataFactory,
                                              cString("f32pCarRight"),
                                              m_ddlDetectionInfoId.f32pCarRight));
        (adtf_ddl::access_element::find_index(m_ObsDataFactory,
                                              cString("f32pCarCenter"),
                                              m_ddlDetectionInfoId.f32pCarCenter));
        (adtf_ddl::access_element::find_index(m_ObsDataFactory,
                                              cString("f32pPerson"),
                                              m_ddlDetectionInfoId.f32pPerson));
        (adtf_ddl::access_element::find_index(m_ObsDataFactory,
                                              cString("f32pChild"),
                                              m_ddlDetectionInfoId.f32pChild));
        (adtf_ddl::access_element::find_index(m_ObsDataFactory,
                                              cString("f32pObsLeft"),
                                              m_ddlDetectionInfoId.f32pObsLeft));
        (adtf_ddl::access_element::find_index(m_ObsDataFactory,
                                              cString("f32pObsRight"),
                                              m_ddlDetectionInfoId.f32pObsRight));
        (adtf_ddl::access_element::find_index(m_ObsDataFactory,
                                              cString("f32pObsRear"),
                                              m_ddlDetectionInfoId.f32pObsRear));
    } else {
        LOG_INFO("No mediadescription for tDetectionInfo found!");
    }

    switch (rw) {
        case RW::READ: {
            this->filter->Register(m_oReader, pin_name, pTypeObsData);
            break;
        }
        case RW::WRITE: {
            this->filter->Register(m_oWriter, pin_name, pTypeObsData);
            break;
        }
        default: LOG_ERROR("Unknown enum value.");
    }

    this->was_registered = true;
    RETURN_NOERROR;
}


tResult DetectionInfoPin::writeData(const tDetectionInfo& data) {
    if (!this->was_registered || this->rw == RW::READ) {
        LOG_ERROR("Trying to write but not registered or not configured to write.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    object_ptr<ISample> pWriteSample;

    tResult r = alloc_sample(pWriteSample, (*(this->m_pClock))->GetStreamTime());
    RETURN_IF_FAILED(r) {
        auto oCodec = m_ObsDataFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.IsValid());

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDetectionInfoId.f32pCarLeft,
                                                data.f32pCarLeft));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDetectionInfoId.f32pCarRight,
                                                data.f32pCarRight));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDetectionInfoId.f32pCarCenter,
                                                data.f32pCarCenter));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDetectionInfoId.f32pObsLeft,
                                                data.f32pObsLeft));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDetectionInfoId.f32pObsRight,
                                                data.f32pObsRight));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDetectionInfoId.f32pObsRear,
                                                data.f32pObsRear));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDetectionInfoId.f32pPerson,
                                                data.f32pPerson));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlDetectionInfoId.f32pChild,
                                                data.f32pChild));

        /*auto *list = reinterpret_cast<tLaserSegStruct*>
                            (oCodec.GetElementAddress(m_ddlDetectionInfoId.list));

        memset(list, 0, MAX_NO_POINTS * sizeof(tLaserSegStruct));
        int i = 0;
        for (auto iterator = obstacles.begin(); iterator < obstacles.end(); iterator++) {
            if (i >= MAX_NO_POINTS) {
                break;
            }
            tLaserSegStruct ls_struct = *iterator;
            list[i].f32Angle = ls_struct.f32Angle;
            list[i].f32Distance = ls_struct.f32Distance;
            list[i].i16Class = ls_struct.i16Class;
            list[i].i32Height = ls_struct.i32Height;
            list[i].i32Width = ls_struct.i32Width;
            i++;
        }*/
    }

    m_oWriter << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}


tResult DetectionInfoPin::readData(tDetectionInfo *data) {
    if (!this->was_registered || this->rw == RW::WRITE) {
        LOG_ERROR("Trying to read but not registered or not configured to read.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    object_ptr<const ISample> pReadSamplePoints;
    RETURN_IF_FAILED(m_oReader.GetNextSample(pReadSamplePoints)) {
        auto oDecoder = m_ObsDataFactory.MakeDecoderFor(*pReadSamplePoints);

        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDetectionInfoId.f32pCarLeft,
                                                  &(data->f32pCarLeft)));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDetectionInfoId.f32pCarRight,
                                                  &(data->f32pCarRight)));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDetectionInfoId.f32pCarCenter,
                                                  &(data->f32pCarCenter)));
         RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDetectionInfoId.f32pPerson,
                                                &(data->f32pPerson)));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDetectionInfoId.f32pChild,
                                                  &(data->f32pChild)));

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDetectionInfoId.f32pObsLeft,
                                                  &(data->f32pObsLeft)));

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDetectionInfoId.f32pObsRight,
                                                  &(data->f32pObsRight)));

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlDetectionInfoId.f32pObsRear,
                                                  &(data->f32pObsRear)));

       
    }

    RETURN_NOERROR;
}

