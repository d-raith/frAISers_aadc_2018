
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

#include "TestFilter.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_TEMPLATEFILTER_DATA_TRIGGERED_FILTER,
    "TestFilter",
    cTestFilter,
    adtf::filter::pin_trigger({"input"}));


cTestFilter::cTestFilter() {
    // DO NOT FORGET TO LOAD MEDIA DESCRIPTION SERVICE IN ADTF3 AND CHOOSE aadc.description
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
        "tSignalValue",
        pTypeSignalValue,
        m_SignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, 
                                            cString("ui32ArduinoTimestamp"),
                                            m_ddlSignalValueId.timestamp));
        
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory,
                                             cString("f32Value"),
                                             m_ddlSignalValueId.value));
    } else {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }

    Register(m_oReaderSpeed, "speed" , pTypeSignalValue);
    Register(m_oWriter, "output", pTypeSignalValue);

    object_ptr<IStreamType> pTypeLSData;
    if (ERR_NOERROR == adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScanneerData", pTypeLSData, m_LSStructSampleFactory))
    {
        //// find the indexes of the element for faster access in the process method
        LOG_INFO("Found mediadescription for tLaserScannerData");
        // get all the number indices
        (adtf_ddl::access_element::find_index(m_LSStructSampleFactory,
                                            "ui32Size",
                                            m_ddlLSDataId.size));
        (adtf_ddl::access_element::find_array_index(m_LSStructSampleFactory,
                                            "tScanArray",
                                            m_ddlLSDataId.scanArray));
    } else {
        LOG_WARNING("No mediadescription for tLaserScannerData found!");
    }

    Register(m_oInputLaserScanner, "laser_scanner", pTypeLSData);

    RegisterPropertyVariable("Field of view min angle [deg]", m_propMinFoVAngle);
    RegisterPropertyVariable("Field of view max angle [deg]", m_propMaxFoVAngle);
    RegisterPropertyVariable("Min distance to obstacle [mm]", m_propMinObstacleDistance);
}

// implement the Configure function to read ALL Properties
tResult cTestFilter::Configure() {
    RETURN_NOERROR;
}

tResult cTestFilter::Process(tTimeStamp tmTimeOfTrigger) {
    object_ptr<const ISample> pReadSample;

    tSignalValue speedSignal;

    if (IS_OK(m_oReaderSpeed.GetLastSample(pReadSample))) {
        auto oDecoder = m_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);

        RETURN_IF_FAILED(oDecoder.IsValid());

        // retrieve the values (using convenience methods that return a variant)
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.value, &speedSignal.f32Value));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlSignalValueId.timestamp, &speedSignal.ui32ArduinoTimestamp));

        // do the processing
        if (m_doEmergencyBreak) {
            speedSignal.f32Value = 0;
        }

        RETURN_IF_FAILED(transmitSpeed(speedSignal));
    }

    object_ptr<const ISample> pReadSampleLS;
        if(IS_OK(m_oInputLaserScanner.GetLastSample(pReadSampleLS))) {
            auto oDecoder = m_LSStructSampleFactory.MakeDecoderFor(*pReadSampleLS);

            RETURN_IF_FAILED(oDecoder.IsValid());
            tSize numOfScanPoints = 0;
            RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlLSDataId.size, &numOfScanPoints));

            const tPolarCoordiante* pCoordinates=
                reinterpret_cast<const tPolarCoordiante*>(oDecoder.GetElementAddress(
                    m_ddlLSDataId.scanArray));

            std::vector<tPolarCoordiante> scan;
            tPolarCoordiante scanPoint;

            for(tSize i = 0; i < numOfScanPoints; ++i) {
                scanPoint.f32Radius = pCoordinates[i].f32Radius;
                scanPoint.f32Angle = pCoordinates[i].f32Angle;
                scan.push_back(scanPoint);
            }

            // init with some max value
            CheckEmergencyBreak(scan);
        }

    RETURN_NOERROR;
}


void cTestFilter::CheckEmergencyBreak(std::vector<tPolarCoordiante> scan) {
    tPolarCoordiante closestObstacle;
    closestObstacle.f32Angle = 0.6;
    closestObstacle.f32Radius = 9999.9;

    // check for obstacle
    for (auto element : scan) {
        // as laserscanner values start from 270 --> 0 (straight) --> 90
        if (element.f32Angle >= tFloat32(m_propMinFoVAngle) || element.f32Angle <= tFloat32(m_propMaxFoVAngle)) {
            // zero radius is actually invalid measurement
            if (element.f32Radius != 0.0 && closestObstacle.f32Radius > element.f32Radius) {
                closestObstacle = element;
            }
        }
    }

    // check min distance
    m_doEmergencyBreak = closestObstacle.f32Radius < tFloat32(m_propMinObstacleDistance);

    CONSOLE_LOG_INFO(cString::Format(
        "%s obstacle found, closest is at %f deg with %f mm dist, FoV (max/min (%f/%f)",
        (m_doEmergencyBreak?" ":"no"),
        closestObstacle.f32Angle,
        closestObstacle.f32Radius,
        tFloat32(m_propMaxFoVAngle),
        tFloat32(m_propMinFoVAngle)));
}

tResult cTestFilter::TransmitSpeed(tSignalValue speedSignal) {
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = m_SignalValueSampleFactory.MakeCodecFor(pWriteSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.timestamp, speedSignal.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlSignalValueId.value, speedSignal.f32Value));
    }

    m_oWriter << pWriteSample << trigger;
    RETURN_NOERROR; 
}