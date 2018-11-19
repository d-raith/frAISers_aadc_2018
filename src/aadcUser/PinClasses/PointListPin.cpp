
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

#include "PointListPin.h"


PointListPin::PointListPin() {   }

tResult PointListPin::registerPin(RW p_rw,
                               cTriggerFunction* p_filter,
                               const object_ptr<adtf::services::IReferenceClock>* p_clock,
                               const tChar* pin_name) {
    if (this->was_registered == true) {
        LOG_ERROR("Trying to register pin twice.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }
    this->rw = p_rw;
    this->filter = p_filter;
    this->m_pClock = p_clock;

    object_ptr<IStreamType> pTypePointListData;
    if (ERR_NOERROR ==
        adtf::mediadescription::ant::
        create_adtf_default_stream_type_from_service("tPointList",
                                                     pTypePointListData,
                                                     m_PointListStructSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_PointListStructSampleFactory,
                                              cString("ui32Size"),
                                              m_ddlPointListDataId.size));
        (adtf_ddl::access_element::find_array_index(m_PointListStructSampleFactory,
                                                    cString("list"),
                                                    m_ddlPointListDataId.list));
    } else {
        LOG_INFO("No mediadescription for tPointList found!");
    }

    switch (rw) {
        case RW::READ: {
            this->filter->Register(m_oReader_point_list, pin_name, pTypePointListData);
            break;
        }
        case RW::WRITE: {
            this->filter->Register(m_oWriter_point_list, pin_name, pTypePointListData);
            break;
        }
        default: LOG_ERROR("Unknown enum value.");
    }

    this->was_registered = true;
    RETURN_NOERROR;
}


tResult PointListPin::writeData(const vector<tPoint>& data) {
    const int MAX_NO_POINTS = 200;

    if (!this->was_registered || this->rw == RW::READ) {
        LOG_ERROR("Trying to write but not registered or not configured to write.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    unsigned int listsize = data.size();
    if (listsize > MAX_NO_POINTS) {
        LOG_INFO("Number of points to transmit (%i) exceeds max (%i)",
                 data.size(), MAX_NO_POINTS);
        listsize = MAX_NO_POINTS;
    }

    object_ptr<ISample> pWriteSample;

    tResult r = alloc_sample(pWriteSample, (*(this->m_pClock))->GetStreamTime());
    RETURN_IF_FAILED(r) {
        auto oCodec = m_PointListStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlPointListDataId.size,
                                                listsize));

        tPoint* list = reinterpret_cast<tPoint*>
                            (oCodec.GetElementAddress(m_ddlPointListDataId.list));
        memset(list, 0, MAX_NO_POINTS * sizeof(tPoint));
        int i = 0;
        for (auto iterator = data.begin(); iterator < data.end(); iterator++) {
            if (i >= MAX_NO_POINTS) {
                break;
            }
            list[i].x = (*iterator).x;
            list[i].y = (*iterator).y;
            i++;
        }
    }

    m_oWriter_point_list << pWriteSample << flush << trigger;

    RETURN_NOERROR;
}


tResult PointListPin::readData(vector<tPoint>* data) {
    if (!this->was_registered || this->rw == RW::WRITE) {
        LOG_ERROR("Trying to read but not registered or not configured to read.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    object_ptr<const ISample> pReadSamplePoints;
    bool sample_found = false;
    while (IS_OK(m_oReader_point_list.GetNextSample(pReadSamplePoints))) {
        data->clear();
        auto oDecoder = m_PointListStructSampleFactory.MakeDecoderFor(*pReadSamplePoints);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tSize listsize = 0;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPointListDataId.size, &listsize));
        const tPoint* pPoints =
            reinterpret_cast<const tPoint*>(oDecoder.GetElementAddress(
                m_ddlPointListDataId.list));

        tPoint point;

        for (tSize i = 0; i < listsize; ++i) {
            point.x = pPoints[i].x;
            point.y = pPoints[i].y;
            data->push_back(point);
        }
        sample_found = true;
    }
    if (sample_found == false) {
        RETURN_ERROR(ERR_EMPTY);
    } else {
        RETURN_NOERROR;
    }
}
