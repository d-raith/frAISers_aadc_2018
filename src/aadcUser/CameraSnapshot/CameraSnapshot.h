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
/*
Author: Ben Wilhelm
based on: OpenCVTemplate, frAIburg_CameraSnapshot
*/
#include "stdafx.h"

#pragma once

//*************************************************************************************************
#define CID_CCAMERASNAPSHOT_DATA_TRIGGERED_FILTER "camera_snapshot.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;


/*! the main class of the open cv template. */
class cCameraSnapshot : public cTriggerFunction {
 private:
    // Pins
    /*! Reader of an InPin. */
    cPinReader m_oReader;
    /*! Writer to an OutPin. */
    cPinWriter m_oWriter;

    // Stream Formats
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_sImageFormat;
    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /* The number of video frames per snapshot (e.g. 10 to capture every 10th frame) */
    property_variable<int> snapshotInterval = 10;
    /* Path to folder, where all snapshots are stored.
       For each ADTF execution a subfolder is generated. */
    property_variable<cFilepath> snapshotFolder = cFilepath(cString("/home/aadc/AADC/capturing/"));


    /* Processes a single video frame */
    void processVideoFrame(cv::Mat inputImage, cv::Mat* outputImage);
    /* Function to set up storage path for the snapshots */
    tResult ComposeStoragePath();
    /* Function to store a snapshot */
    void storeSnapshot(const Mat& inputImage);

    /* Count the number of video frames seen so far. */
    int64 videoFrameCounter = 0;
    /* Variable to count the calls to the function Configure() */
    int noConfigureCalls = 0;
    /* Path to folder, where the snapshots of this execution are stored */
    cString storage_path_;
    /* variable to store the state of this filter */
    enum cCameraSnapshotState {
        /* filter is starting up, NOT ready yet! */
        STARTUP,
        /* ready to do work */
        READY,
        /* failed to set up */
        FAIL
    };
    cCameraSnapshotState state = STARTUP;

 public:
    /*! Default constructor. */
    cCameraSnapshot();

    /*! Destructor. */
    virtual ~cCameraSnapshot() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;
};


//*************************************************************************************************
