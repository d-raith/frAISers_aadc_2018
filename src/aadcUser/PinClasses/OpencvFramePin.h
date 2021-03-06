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
#pragma once

#include <vector>
using std::vector;

#include "stdafx.h"
#include "customtypes.h"
#include "opencv2/core/mat.hpp"


//*************************************************************************************************

using namespace adtf_util;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;



/*! Pin class for transmitting and receiving a list of tPoints from customtypes.h
    One object can be configured to read only or to write only, not both at a time. */
class OpencvFramePin {
 public:
    enum RW {
        READ,
        WRITE
    };

 private:
    /*! indicate if this pin instance is for reading or writing */
    RW rw;

    /*! Pointer to the clock of the filter. */
    const object_ptr<adtf::services::IReferenceClock>* m_pClock;

    /*! Pointer to the filter object that wants to register the pin. */
    cTriggerFunction* filter;

    /* Indicates if the pin was already registered */
    bool was_registered = false;


    /*! Reader of an InPin. */
    cPinReader m_oReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;

    /*! Video Stream input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;
    adtf::ucom::object_ptr<IStreamType> m_Type;

    /*! output-OpencvFramePins of which the type is linked to this type */
    vector<OpencvFramePin*> linked_output_pins;

 public:
    /*! Default constructor. */
    OpencvFramePin();

    /*! Function to register a pin.
        Parameter p_rw: indicates weather to register the pin for reading or for writing.
        Parameter filter: Pointer to the filter object that wants to register the pin.
        Parameter p_Clock: Pointer to the clock of the filter object. The responsibility
                            for registering the clock remains at the filter!
        Parameter pin_name: Name for registering the pin.
        Return: True if successfull, False if not successfull. */
    tResult registerPin(RW p_rw,
                     cTriggerFunction* filter,
                     const object_ptr<adtf::services::IReferenceClock>* p_clock,
                     const tChar* pin_name);

    /*! Function to link the stream type of one or more write-OpencvFramePin
        to the stream type of this OpencvFramePin.
        This function must only be called on an input pin.
        This function must be called after the the pins are registered. */
    tResult linkTypes(vector<OpencvFramePin*> outputPins);

    /* Function to write data to the pin.
       Pin must have been registed for writing before. */
    tResult writeData(const cv::Mat& outputImage);
    tResult writeData(const cv::Mat& outputImage, tStreamImageFormatName
    imageFormat);


    /* Function to read data from the pin.
       Pin must have been registered for reading before. */
    tResult readData(cv::Mat* image);
};


//*************************************************************************************************
