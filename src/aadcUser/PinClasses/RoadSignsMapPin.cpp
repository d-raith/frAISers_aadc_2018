
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

#include "RoadSignsMapPin.h"


#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

tResult RoadSignsMapPin::registerPin(RW p_rw,
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

tResult RoadSignsMapPin::processRoadSignFile(const ISample& sample,
                                             vector<RoadSign> *roadSigns,
                                             vector<ParkingSpace> *parking)
{
    adtf::ucom::ant::object_ptr_shared_locked<const adtf::streaming::ant::ISampleBuffer> pSampleBuffer;
    RETURN_IF_FAILED(sample.Lock(pSampleBuffer));

    adtf_util::cString roadSignFileString;
    roadSignFileString.SetBuffer(pSampleBuffer->GetSize());

    memcpy(roadSignFileString.GetBuffer(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());

    LOG_INFO("Parsing road sign file");
    cDOM oDOM;
    RETURN_IF_FAILED(oDOM.FromString(roadSignFileString));
    RETURN_IF_FAILED(parseRoadSignFile(oDOM, roadSigns, parking));

    LOG_INFO(cString::Format("Received road sign file from pin with %d signs and %d parking "
                             "spaces", roadSigns->size(), parking->size()));
    RETURN_NOERROR;
}



tResult RoadSignsMapPin::parseRoadSignFile(cDOM& oDOM,  vector<RoadSign> *roadSigns,
        vector<ParkingSpace> *parking, bool convertDirectionToRadian)
{

    cDOMElementRefList oElems;

    if (IS_OK(oDOM.FindNodes("configuration/roadSign", oElems)))
    {
        for (auto &oElem : oElems) {
            RoadSign item;
            item.u16Id = tUInt16(oElem->GetAttribute("id", "0").AsUInt32());
            item.f32X = tFloat32(oElem->GetAttribute("x", "0").AsFloat64());
            item.f32Y = tFloat32(oElem->GetAttribute("y", "0").AsFloat64());
            item.f32Radius = tFloat32(oElem->GetAttribute("radius", "0").AsFloat64());
            item.f32Direction = tFloat32(oElem->GetAttribute("direction", "0").AsFloat64());

            //item.bInit = ((*itElem)->GetAttribute("init","0").AsInt32());
            item.bInit = oElem->GetAttribute("init", "0").AsInt32() != 0;


            item.u16Cnt = 0;
            item.u32ticks = adtf_util::cHighResTimer::GetTime();
            if (convertDirectionToRadian) {
                item.f32Direction *= static_cast<tFloat32>(DEG2RAD); // convert to radians
            }



            roadSigns->push_back(item);

        }

    }

    if (IS_OK(oDOM.FindNodes("configuration/parkingSpace", oElems)))
    {
        for (auto &oElem : oElems) {
            ParkingSpace item;
            item.u16Id = tUInt16(oElem->GetAttribute("id", "0").AsInt32());
            item.f32X = tFloat32(oElem->GetAttribute("x", "0").AsFloat64());
            item.f32Y = tFloat32(oElem->GetAttribute("y", "0").AsFloat64());
            item.f32Direction = tFloat32(oElem->GetAttribute("direction", "0").AsFloat64());

            //item.f32Direction *= static_cast<tFloat32>(DEG2RAD); // convert to radians


            parking->push_back(item);

        }

    }

    RETURN_NOERROR;
}


tResult RoadSignsMapPin::readData(vector<RoadSign> *roadSigns,
                                  vector<ParkingSpace> *parking) {
    if (!this->was_registered) {
        LOG_ERROR("Trying to read but not registered or not configured to read.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }


    object_ptr<const ISample> pSampleAnonymous;
    while (IS_OK(m_oReader.GetNextSample(pSampleAnonymous))) {
        RETURN_IF_FAILED(processRoadSignFile(*pSampleAnonymous, roadSigns, parking)){
            LOG_INFO("Road signs parsed successfully");
            RETURN_NOERROR;
        };
    }

    RETURN_ERROR(ERR_NOT_READY);
}

