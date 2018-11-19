
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

#include "LaserScannerPin.h"


tResult LaserScannerPin::registerPin(cTriggerFunction *p_filter,
                                     const object_ptr<adtf::services::IReferenceClock> *p_clock,
                                     const tChar *pin_name) {
    if (this->was_registered) {
        LOG_ERROR("Trying to register pin twice.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }
    this->filter = p_filter;
    this->m_pClock = p_clock;

    object_ptr<IStreamType> pTypeLSData;
    if (ERR_NOERROR ==
        adtf::mediadescription::ant::
        create_adtf_default_stream_type_from_service("tLaserScannerData",
                                                     pTypeLSData,
                                                     m_LSStructSampleFactory)) {
        //// find the indexes of the element for faster access in the process method
        LOG_INFO("Found mediadescription for tLaserScannerData!");
        // get all the member indices
        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory,
                                              "ui32Size",
                                              m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory,
                                                    "tScanArray",
                                                    m_ddlLSDataId.scanArray));
    } else {
        LOG_INFO("No mediadescription for tLaserScannerData found!");
    }

    this->filter->Register(m_oReader, pin_name, pTypeLSData);

    this->was_registered = true;
    RETURN_NOERROR;
}


tResult LaserScannerPin::readData(vector<tPolarCoordiante> *data) {
    if (!this->was_registered) {
        LOG_ERROR("Trying to read but not registered or not configured to read.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    object_ptr<const ISample> pReadSampleLS;
    bool new_sample_found = false;
    while (IS_OK(m_oReader.GetNextSample(pReadSampleLS))) {
        data->clear();
        auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pReadSampleLS);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tSize numOfScanPoints = 0;
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints));

        const auto *pCoordinates =
                reinterpret_cast<const tPolarCoordiante *>(oDecoder.GetElementAddress(
                        m_ddlLSDataId.scanArray));

        tPolarCoordiante scanPoint;

        for (tSize i = 0; i < numOfScanPoints; ++i) {
            scanPoint.f32Radius = pCoordinates[i].f32Radius;
            scanPoint.f32Angle = pCoordinates[i].f32Angle;
            data->emplace_back(scanPoint);
        }
        new_sample_found = true;
    }
    if (new_sample_found == false) {
        RETURN_ERROR(ERR_EMPTY);
    } else {
        RETURN_NOERROR;
    }
}
