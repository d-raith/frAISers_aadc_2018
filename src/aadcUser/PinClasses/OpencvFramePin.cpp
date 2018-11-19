#include <utility>


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

#include "OpencvFramePin.h"
#include "ADTF3_OpenCV_helper.h"

OpencvFramePin::OpencvFramePin() {   }

tResult OpencvFramePin::registerPin(RW p_rw,
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


    this->m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    this->m_Type = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*m_Type, m_sImageFormat);


    switch (rw) {
        case RW::READ: {
            this->filter->Register(m_oReader, pin_name, m_Type);
            break;
        }
        case RW::WRITE: {
            this->filter->Register(m_oWriter, pin_name, m_Type);
            break;
        }
        default: LOG_ERROR("Unknown enum value.");
    }

    this->was_registered = true;
    RETURN_NOERROR;
}

tResult OpencvFramePin::linkTypes(vector<OpencvFramePin*> outputPins) {
    if (this->was_registered == false) {
        LOG_ERROR("Trying to link types, but this pin was not registered yet.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }
    if (this->rw == RW::WRITE) {
        LOG_ERROR("Trying to link ouput-stream-type to non-input-stream-type");
        RETURN_ERROR(ERR_INVALID_STATE);
    }
    for (auto iterator = outputPins.begin(); iterator < outputPins.end(); iterator++) {
        if ((*iterator)->rw != RW::WRITE) {
            LOG_ERROR("Trying to link non-output-stream-type to input-stream-type");
            RETURN_ERROR(ERR_INVALID_STATE);
        }
    }
    this->linked_output_pins = outputPins;

    m_oReader.SetAcceptTypeCallback(
        [this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType> &pType)
            -> tResult {
            for (unsigned int i = 0; i != linked_output_pins.size(); i++) {
                ChangeType(m_oReader,
                           m_sImageFormat,
                           *(this->m_Type).Get(),
                           linked_output_pins[i]->m_oWriter);
            }
            RETURN_NOERROR;
        });
    RETURN_NOERROR;
}

tResult OpencvFramePin::writeData(const cv::Mat& outputImage, tStreamImageFormatName imageFormat) {
    if (!this->was_registered || this->rw == RW::READ) {
        LOG_ERROR("Trying to write but not registered or not configured to write.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    // write outputImage_lines to pin
    if (!outputImage.empty()) {
        this->m_sImageFormat.m_strFormatName = std::move(imageFormat);
        this->m_Type = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
        set_stream_type_image_format(*m_Type, m_sImageFormat);
        // update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize()
            != m_sImageFormat.m_szMaxByteSize) {
            setTypeFromMat(this->m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(this->m_oWriter, outputImage, (*m_pClock)->GetStreamTime());
    }
    RETURN_NOERROR;
}

tResult OpencvFramePin::writeData(const cv::Mat& outputImage) {
    if (!this->was_registered || this->rw == RW::READ) {
        LOG_ERROR("Trying to write but not registered or not configured to write.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    // write outputImage_lines to pin
    if (!outputImage.empty()) {
        // update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize()
                                    != m_sImageFormat.m_szMaxByteSize) {
            setTypeFromMat(this->m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(this->m_oWriter, outputImage, (*m_pClock)->GetStreamTime());
    }
    RETURN_NOERROR;
}


tResult OpencvFramePin::readData(cv::Mat* image) {
    if (!this->was_registered || this->rw == RW::WRITE) {
        LOG_ERROR("Trying to read but not registered or not configured to read.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }
    
    object_ptr<const ISample> pReadSample;
    RETURN_IF_FAILED(m_oReader.GetLastSample(pReadSample)){
        // create an opencv matrix from the media sample buffer 
         object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
    
        RETURN_IF_FAILED(pReadSample->Lock(pReadBuffer)){
            LOG_INFO("Read image");
            *image = cv::Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                 CV_8UC3,
                                 const_cast<unsigned char*>
                                    (static_cast<const unsigned char*>(pReadBuffer->GetPtr())) );


        LOG_INFO("image read successfully %f %f", image->size().width, image->size().height);
        }
    }
    
  /*  while (IS_OK(m_oReader.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        // lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            // create an opencv matrix from the media sample buffer
            *image = cv::Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                 CV_8UC3,
                                 const_cast<unsigned char *>
                                    (static_cast<const unsigned char *>(pReadBuffer->GetPtr())));
        }
    }*/
    RETURN_NOERROR;
}
