
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
#include "CameraSnapshot.h"

#include "ADTF3_OpenCV_helper.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CCAMERASNAPSHOT_DATA_TRIGGERED_FILTER,
                                    "Camera Snapshot",
                                    cCameraSnapshot,
                                    adtf::filter::pin_trigger({ "input" }));

cCameraSnapshot::cCameraSnapshot() {
    // create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType =
        adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    // Register input pin
    Register(m_oReader, "input", pType);
    // Register output pin
    Register(m_oWriter, "output", pType);

    // register callback for type changes
    m_oReader.SetAcceptTypeCallback(
        [this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType)
        -> tResult {
            return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
        });

    RegisterPropertyVariable("snapshot interval [#]", snapshotInterval);
    RegisterPropertyVariable("snapshot folder", snapshotFolder);
}

tResult cCameraSnapshot::Configure() {
    // count calls to this function
    noConfigureCalls++;

    // get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    if (noConfigureCalls == 1) {
        tResult tresult = ComposeStoragePath();
        if (tresult.GetErrorCode() == ERR_NOERROR) {
            this->state = READY;
        } else {
            this->state = FAIL;
            LOG_INFO("CameraSnapshot failed to set up storage folder.");
        }
    }

    RETURN_NOERROR;
}

tResult cCameraSnapshot::Process(tTimeStamp tmTimeOfTrigger) {
    object_ptr<const ISample> pReadSample;
    Mat outputImage;

    while (IS_OK(m_oReader.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        // lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            // create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                 CV_8UC3,
                                 const_cast<unsigned char*>
                                    (static_cast<const unsigned char*>(pReadBuffer->GetPtr())) );

            // Do the image processing and copy to destination image buffer
            processVideoFrame(inputImage, &outputImage);
        }
    }

    // Write processed Image to Output Pin
    if (!outputImage.empty()) {
        // update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize) {
            setTypeFromMat(m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }

    RETURN_NOERROR;
}

void cCameraSnapshot::processVideoFrame(cv::Mat inputImage, cv::Mat* outputImage) {
    // count frames
    videoFrameCounter++;
    // copy input to output image
    *outputImage = inputImage;
    // draw center line
    // line(*outputImage,
    //      Point(outputImage->size().width) / 2,
    //      Point(outputImage->size().width / 2, outputImage->size().height),
    //      Scalar(255, 255, 255),
    //      1);

    // store snapshot if conditions indicate to do so
    if (this->state == FAIL || this->state == STARTUP) {
        // not ready to store snapshots
        return;
    } else {
        if (videoFrameCounter % snapshotInterval == 0) {
            storeSnapshot(inputImage);
            // mark the output frame
            cv::rectangle(*outputImage,
                Point(0, 0),
                outputImage->size(),
                Scalar(0, 255, 0),
                50);
        }
    }
}

string GetTimeStampString() {
    time_t t = time(NULL);
    struct tm* timestruct = localtime(&t);
    char buffer[120];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H:%M:%S", timestruct);
    return std::string(buffer);
}

tResult cCameraSnapshot::ComposeStoragePath() {
    this->storage_path_ = snapshotFolder;
    if (!storage_path_.EndsWith("/")) {
        storage_path_ = storage_path_ + "/";
    }
    storage_path_ = storage_path_ + GetTimeStampString().c_str() + "/";
    // create the directory
    LOG_INFO("Debug: StorageDirectory " + storage_path_);
    const int error = mkdir(storage_path_,  // storage_path_.GetPtr()
                            S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);  // flags
    if (error == -1) {
        LOG_ERROR("Error creating directory!\n");
        RETURN_ERROR(ERR_IO_INCOMPLETE);
    }
    RETURN_NOERROR;
}

void cCameraSnapshot::storeSnapshot(const Mat& inputImage) {
    cString path = this->storage_path_ + cString::Format("img%d.tiff", this->videoFrameCounter);
    std::string stdpath(path.GetPtr());
    if (inputImage.type() == CV_8UC3) {
        cv::cvtColor(inputImage, inputImage, CV_RGB2BGR);
    }
    cv::imwrite(stdpath.c_str(), inputImage);
}
