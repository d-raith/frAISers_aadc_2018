﻿
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

#include "LaneListPin.h"


LaneListPin::LaneListPin() {   }

tResult LaneListPin::registerPin(RW p_rw,
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

    object_ptr<IStreamType> pTypeLaneListData;
    if (ERR_NOERROR ==
        adtf::mediadescription::ant::
        create_adtf_default_stream_type_from_service("tLaneList",
                                                     pTypeLaneListData,
                                                     m_LaneListStructSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_LaneListStructSampleFactory,
                                              cString("ui32Size"),
                                              m_ddlLaneListDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LaneListStructSampleFactory,
                                                    cString("list"),
                                                    m_ddlLaneListDataId.list));
    } else {
        LOG_INFO("No mediadescription for tPointList found!");
    }

    switch (rw) {
        case RW::READ: {
            this->filter->Register(m_oReader_lane_list, pin_name, pTypeLaneListData);
            break;
        }
        case RW::WRITE: {
            this->filter->Register(m_oWriter_lane_list, pin_name, pTypeLaneListData);
            break;
        }
        default: LOG_ERROR("Unknown enum value.");
    }

    this->was_registered = true;
    RETURN_NOERROR;
}


tResult LaneListPin::writeData(const vector<tLaneElement>& data) {
    const int MAX_NO_POINTS = 10;

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
        auto oCodec = m_LaneListStructSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlLaneListDataId.size,
                                                listsize));

        tLaneElement* list = reinterpret_cast<tLaneElement*>
                            (oCodec.GetElementAddress(m_ddlLaneListDataId.list));
        memset(list, 0, MAX_NO_POINTS * sizeof(tLaneElement));
        int i = 0;
        for (auto iterator = data.begin(); iterator < data.end(); iterator++) {
            if (i >= MAX_NO_POINTS) {
                break;
            }
            list[i].alignment = (*iterator).alignment;
            list[i].dist = (*iterator).dist;
            list[i].orientation = (*iterator).orientation;
            list[i].isVertLane = (*iterator).isVertLane;
            list[i].angleOriginRad = (*iterator).angleOriginRad;

            list[i].coeff0 = (*iterator).coeff0;
            list[i].coeff1 = (*iterator).coeff1;
            list[i].coeff2 = (*iterator).coeff2;


            list[i].startx = (*iterator).startx;
            list[i].starty = (*iterator).starty;
            list[i].endx = (*iterator).endx;
            list[i].endy = (*iterator).endy;

            i++;
        }
    }

    m_oWriter_lane_list << pWriteSample; //<< flush << trigger;

    RETURN_NOERROR;
}


tResult LaneListPin::readData(vector<tLaneElement>* data) {
    if (!this->was_registered || this->rw == RW::WRITE) {
        LOG_ERROR("Trying to read but not registered or not configured to read.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    object_ptr<const ISample> pReadSamplePoints;
    if (IS_OK(m_oReader_lane_list.GetLastSample(pReadSamplePoints))) {
        auto oDecoder = m_LaneListStructSampleFactory.MakeDecoderFor(*pReadSamplePoints);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tSize listsize = 0;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLaneListDataId.size, &listsize));

        const tLaneElement* pLanes =
            reinterpret_cast<const tLaneElement*>(oDecoder.GetElementAddress(
                m_ddlLaneListDataId.list));

        tLaneElement lane;

        for (tSize i = 0; i < listsize; ++i) {
            lane.alignment = pLanes[i].alignment;
            lane.dist = pLanes[i].dist;
            lane.orientation = pLanes[i].orientation;
            lane.isVertLane = pLanes[i].isVertLane;
            lane.angleOriginRad = pLanes[i].angleOriginRad;

            lane.coeff0 = pLanes[i].coeff0;
            lane.coeff1 = pLanes[i].coeff1;
            lane.coeff2 = pLanes[i].coeff2;


            lane.startx = pLanes[i].startx;
            lane.starty = pLanes[i].starty;
            lane.endx = pLanes[i].endx;
            lane.endy = pLanes[i].endy;
            data->push_back(lane);
        }
    }

    RETURN_NOERROR;
}

