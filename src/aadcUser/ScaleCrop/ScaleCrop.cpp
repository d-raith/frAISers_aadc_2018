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

#include "stdafx.h"
#include "ScaleCrop.h"
#include "ADTF3_OpenCV_helper.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_SCALECROP_DATA_TRIGGERED_FILTER,
                                    "Scale Crop",
                                    ScaleCrop,
                                    adtf::filter::pin_trigger({ "input" }));

ScaleCrop::ScaleCrop()
{

    //create and set inital input format type
    m_sImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    const adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sImageFormat);

    //Register input pin
    Register(m_oReader, "input", pType);
    //Register output pin
    Register(m_oWriter, "output", pType);

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sImageFormat, *pType.Get(), m_oWriter);
    });

    RegisterPropertyVariable("scaling_factor [float]", scaling_factor);
    RegisterPropertyVariable("crop_offset_width [pix]", crop_offset_width);
    RegisterPropertyVariable("crop_offset_height [pix]", crop_offset_height);
    RegisterPropertyVariable("crop_width [pix]", crop_width);
    RegisterPropertyVariable("crop_height [pix]", crop_height);
    RegisterPropertyVariable("enable_cropping [bool]", enable_cropping);
    RegisterPropertyVariable("enable_scaling [bool]", enable_scaling);
}

tResult ScaleCrop::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    
    RETURN_NOERROR;
}

tResult ScaleCrop::Process(tTimeStamp tmTimeOfTrigger)
{
    object_ptr<const ISample> pReadSample;
    Mat outputImage;

    while (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage = Mat(cv::Size(m_sImageFormat.m_ui32Width, m_sImageFormat.m_ui32Height),
                                   CV_8UC3, const_cast<unsigned char*>(static_cast<const unsigned char*>(pReadBuffer->GetPtr())));

            //Do the image processing and copy to destination image buffer
            // Canny(inputImage, outputImage, 100, 200);// Detect Edges
            cv::Mat resized_image = inputImage;
            if (static_cast<bool>(enable_scaling) == true) {
                cv::resize(resized_image,
                            resized_image,
                            cv::Size(),
                            static_cast<float>(scaling_factor),
                            static_cast<float>(scaling_factor),
                            cv::INTER_LINEAR);
            }
            if (static_cast<bool>(enable_cropping) == true) {
                cv::Rect crop_region = cv::Rect(static_cast<int>(crop_offset_width),
                                                static_cast<int>(crop_offset_height),
                                                static_cast<int>(crop_width),
                                                static_cast<int>(crop_height));
                resized_image(crop_region).copyTo(outputImage);
            } else {
                outputImage = resized_image;
            }
        }
    }

    //Write processed Image to Output Pin
    if (!outputImage.empty())
    {
        // LOG_INFO("scalecrop: w=%i h=%i", outputImage.size().width, outputImage.size().height);
        //update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sImageFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oWriter, outputImage, m_pClock->GetStreamTime());
    }
    
    RETURN_NOERROR;
}