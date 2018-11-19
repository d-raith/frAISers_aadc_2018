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

#include "VideoOutPin.h"
// #include "ADTF3_OpenCV_helper.h"

VideoOutPin::VideoOutPin() {   }

tResult VideoOutPin::registerPin(RW p_rw,
                               cTriggerFunction* p_filter,
                               const object_ptr<adtf::services::IReferenceClock>* p_clock,
                               const tChar* pin_name,
                               tInt mat_width, tInt mat_height, tInt mat_channels) {
    if (this->was_registered == true) {
        LOG_ERROR("Trying to register pin twice.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    this->m_bStreamTypeChanged = tFalse;
    this->filter = p_filter;
    this->m_pClock = p_clock;
    this->rw = p_rw;
    this->mat_width = mat_width;
    this->mat_height = mat_height;
    this->mat_channels = mat_channels;
    this->mat_max_bytesize = mat_width * mat_height * mat_channels;

    if (this->rw == RW::WRITE) {
        // local map output pin
        object_ptr<IStreamType> pType = make_object_ptr<cStreamType>(stream_meta_type_image());
        set_property(*pType, stream_meta_type_image::FormatName, ADTF_IMAGE_FORMAT(RGB_24));
        set_property(*pType, stream_meta_type_image::PixelWidth, mat_width);
        set_property(*pType, stream_meta_type_image::PixelHeight, mat_height);
        set_property(*pType, stream_meta_type_image::MaxByteSize, this->mat_max_bytesize);

        this->m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
        this->m_Type = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
        set_stream_type_image_format(*m_Type, m_sImageFormat);

        this->filter->Register(m_oWriter, pin_name, pType);

    } else if (this->rw == RW::READ) {
        m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);   // was RGB_32
        m_sImageFormat.m_ui32Width = mat_width;
        m_sImageFormat.m_ui32Height = mat_height;
        m_sImageFormat.m_szMaxByteSize = 0;
        m_sImageFormat.m_ui8DataEndianess = PLATFORM_BYTEORDER;
        adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
        set_stream_type_image_format(*pType, m_sImageFormat);

        RETURN_IF_FAILED(ChangeType(*pType.Get()));

        // RETURN_IF_FAILED(adtf::filter::filter_create_pin(*this, m_oImageReader, "video", pType));   // TODO use pin_name  // TODO use instead of Register
        this->filter->Register(m_oReader, pin_name, pType);

        m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
                                            {
                                                return ChangeType(*pType.Get());
                                            });
    } else {
        LOG_ERROR("VideoOutPin unknown pin type");
    }

    this->was_registered = true;
    RETURN_NOERROR;
}

tResult VideoOutPin::writeData(const cv::Mat& outputImage) {
    if (!this->was_registered || this->rw != RW::WRITE) {
        LOG_ERROR("Trying to write but not registered or not configured to write.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    object_ptr<ISample> pWriteSample;

    if (!outputImage.empty()) {
        LOG_INFO("image not empty");
        if (IS_OK(alloc_sample(pWriteSample, (*m_pClock)->GetStreamTime()))) {
            object_ptr_locked<ISampleBuffer> pWriteBuffer;
            if (IS_OK(pWriteSample->WriteLock(
                pWriteBuffer, outputImage.cols * outputImage.rows * outputImage.channels()))) {
                pWriteBuffer->Write(
                    adtf_memory_buffer<void, tSize>(reinterpret_cast<void*>(outputImage.data),
                    outputImage.cols * outputImage.rows * outputImage.channels()));
            }
        }
        local_map_writer << pWriteSample << flush << trigger;
    }
    RETURN_NOERROR;
}

tResult VideoOutPin::readData(cv::Mat* outputImage) {
    if (!this->was_registered || this->rw != RW::READ) {
        LOG_ERROR("Trying to read but not registered or not configured to read.");
        RETURN_ERROR(ERR_INVALID_STATE);
    }

    if (m_bStreamTypeChanged) {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pType;
        m_oReader >> pType;
        adtf::streaming::get_stream_type_image_format(m_sImageFormat, *pType);
        m_bStreamTypeChanged = tFalse;
    }

    // reading code from here is not from demo_qt_video_display
    object_ptr<const ISample> pReadSample;
    bool sample_found = false;
    while (IS_OK(m_oReader.GetNextSample(pReadSample))) {
        // create an opencv matrix from the media sample buffer
         object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;

        RETURN_IF_FAILED(pReadSample->Lock(pReadBuffer)) {
            *outputImage = cv::Mat(cv::Size(m_sImageFormat.m_ui32Width,
                                   m_sImageFormat.m_ui32Height),
                                   CV_8UC3,
                                   const_cast<unsigned char*>
                                      (static_cast<const unsigned char*>(pReadBuffer->GetPtr())) );
            sample_found = true;
        }
    }
    if (sample_found == false) {
        RETURN_ERROR(ERR_EMPTY);
    } else {
        RETURN_NOERROR;
    }
}



tResult VideoOutPin::ChangeType(const adtf::streaming::ant::IStreamType& oType) {
    // LOG_INFO("Change type called.");
    if (oType == adtf::streaming::stream_meta_type_image()) {
        m_bStreamTypeChanged = tTrue;
    }
    else {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }
    RETURN_NOERROR;
}
