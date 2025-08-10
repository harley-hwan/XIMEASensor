#include "pch.h"
#include "XIMEACameraImpl.h"
#include <xiApi.h>  // Only included in implementation file
#include <map>
#include <cstring>

namespace Camera {

    // Private implementation class
    class XIMEACameraPrivate {
    public:
        // Parameter mapping tables
        std::map<ParamType, const char*> paramToXiString;
        std::map<Camera::ReturnCode, XI_RETURN> returnCodeMapping;
        std::map<XI_RETURN, Camera::ReturnCode> xiReturnMapping;

        XIMEACameraPrivate() {
            InitializeMappings();
        }

        void InitializeMappings() {
            // Parameter mappings
            paramToXiString[ParamType::DEVICE_NAME] = XI_PRM_DEVICE_NAME;
            paramToXiString[ParamType::DEVICE_SN] = XI_PRM_DEVICE_SN;
            paramToXiString[ParamType::IMAGE_DATA_FORMAT] = XI_PRM_IMAGE_DATA_FORMAT;
            paramToXiString[ParamType::OUTPUT_DATA_BIT_DEPTH] = XI_PRM_OUTPUT_DATA_BIT_DEPTH;
            paramToXiString[ParamType::SENSOR_DATA_BIT_DEPTH] = XI_PRM_SENSOR_DATA_BIT_DEPTH;
            paramToXiString[ParamType::WIDTH] = XI_PRM_WIDTH;
            paramToXiString[ParamType::HEIGHT] = XI_PRM_HEIGHT;
            paramToXiString[ParamType::OFFSET_X] = XI_PRM_OFFSET_X;
            paramToXiString[ParamType::OFFSET_Y] = XI_PRM_OFFSET_Y;
            paramToXiString[ParamType::EXPOSURE] = XI_PRM_EXPOSURE;
            paramToXiString[ParamType::GAIN] = XI_PRM_GAIN;
            paramToXiString[ParamType::FRAMERATE] = XI_PRM_FRAMERATE;
            paramToXiString[ParamType::ACQ_TIMING_MODE] = XI_PRM_ACQ_TIMING_MODE;
            paramToXiString[ParamType::BUFFER_POLICY] = XI_PRM_BUFFER_POLICY;
            paramToXiString[ParamType::AUTO_BANDWIDTH_CALCULATION] = XI_PRM_AUTO_BANDWIDTH_CALCULATION;
            paramToXiString[ParamType::TRG_SOURCE] = XI_PRM_TRG_SOURCE;
            paramToXiString[ParamType::DOWNSAMPLING] = XI_PRM_DOWNSAMPLING;
            paramToXiString[ParamType::DOWNSAMPLING_TYPE] = XI_PRM_DOWNSAMPLING_TYPE;
            paramToXiString[ParamType::SENSOR_TAPS] = XI_PRM_SENSOR_TAPS;
            paramToXiString[ParamType::BUFFERS_QUEUE_SIZE] = XI_PRM_BUFFERS_QUEUE_SIZE;
            paramToXiString[ParamType::RECENT_FRAME] = XI_PRM_RECENT_FRAME;
            paramToXiString[ParamType::GAMMAY] = XI_PRM_GAMMAY;
            paramToXiString[ParamType::SHARPNESS] = XI_PRM_SHARPNESS;
            paramToXiString[ParamType::HDR] = XI_PRM_HDR;
            paramToXiString[ParamType::AUTO_WB] = XI_PRM_AUTO_WB;
            paramToXiString[ParamType::MANUAL_WB] = XI_PRM_MANUAL_WB;
            paramToXiString[ParamType::SENS_DEFECTS_CORR] = XI_PRM_SENS_DEFECTS_CORR;
            paramToXiString[ParamType::COLOR_FILTER_ARRAY] = XI_PRM_COLOR_FILTER_ARRAY;
            paramToXiString[ParamType::TRANSPORT_PIXEL_FORMAT] = XI_PRM_TRANSPORT_PIXEL_FORMAT;

            // Return code mappings
            xiReturnMapping[XI_OK] = Camera::ReturnCode::OK;
            xiReturnMapping[XI_INVALID_HANDLE] = Camera::ReturnCode::INVALID_HANDLE;
            xiReturnMapping[XI_READREG] = Camera::ReturnCode::READREG;
            xiReturnMapping[XI_WRITEREG] = Camera::ReturnCode::WRITEREG;
            xiReturnMapping[XI_FREE_RESOURCES] = Camera::ReturnCode::FREE_RESOURCES;
            xiReturnMapping[XI_FREE_CHANNEL] = Camera::ReturnCode::FREE_CHANNEL;
            xiReturnMapping[XI_FREE_BANDWIDTH] = Camera::ReturnCode::FREE_BANDWIDTH;
            xiReturnMapping[XI_TIMEOUT] = Camera::ReturnCode::TIMEOUT;
            xiReturnMapping[XI_INVALID_ARG] = Camera::ReturnCode::INVALID_ARG;
            xiReturnMapping[XI_NOT_SUPPORTED] = Camera::ReturnCode::NOT_SUPPORTED;
            xiReturnMapping[XI_MEMORY_ALLOCATION] = Camera::ReturnCode::MEMORY_ALLOCATION;
            xiReturnMapping[XI_DEVICE_NOT_READY] = Camera::ReturnCode::DEVICE_NOT_READY;
            xiReturnMapping[XI_NO_DEVICES_FOUND] = Camera::ReturnCode::NO_DEVICES_FOUND;
            xiReturnMapping[XI_ACQUISITION_STOPED] = Camera::ReturnCode::ACQUISITION_STOPED;
            xiReturnMapping[XI_WRONG_PARAM_VALUE] = Camera::ReturnCode::WRONG_PARAM_VALUE;
            // ... add all other mappings
        }

        Camera::ReturnCode ConvertXiReturn(XI_RETURN xiRet) {
            auto it = xiReturnMapping.find(xiRet);
            if (it != xiReturnMapping.end()) {
                return it->second;
            }
            return Camera::ReturnCode::UNKNOWN_PARAM;
        }

        XI_RETURN ConvertToXiReturn(Camera::ReturnCode code) {
            for (const auto& pair : xiReturnMapping) {
                if (pair.second == code) {
                    return pair.first;
                }
            }
            return XI_UNKNOWN_PARAM;
        }

        const char* GetXiParamString(ParamType param) {
            auto it = paramToXiString.find(param);
            if (it != paramToXiString.end()) {
                return it->second;
            }
            return nullptr;
        }

        Camera::ImageFormat ConvertXiFormat(XI_IMG_FORMAT xiFormat) {
            switch (xiFormat) {
            case XI_MONO8: return Camera::ImageFormat::MONO8;
            case XI_MONO16: return Camera::ImageFormat::MONO16;
            case XI_RGB24: return Camera::ImageFormat::RGB24;
            case XI_RGB32: return Camera::ImageFormat::RGB32;
            case XI_RAW8: return Camera::ImageFormat::RAW8;
            case XI_RAW16: return Camera::ImageFormat::RAW16;
            case XI_FRM_TRANSPORT_DATA: return Camera::ImageFormat::FRM_TRANSPORT_DATA;
            default: return Camera::ImageFormat::MONO8;
            }
        }

        XI_IMG_FORMAT ConvertToXiFormat(Camera::ImageFormat format) {
            switch (format) {
            case Camera::ImageFormat::MONO8: return XI_MONO8;
            case Camera::ImageFormat::MONO16: return XI_MONO16;
            case Camera::ImageFormat::RGB24: return XI_RGB24;
            case Camera::ImageFormat::RGB32: return XI_RGB32;
            case Camera::ImageFormat::RAW8: return XI_RAW8;
            case Camera::ImageFormat::RAW16: return XI_RAW16;
            case Camera::ImageFormat::FRM_TRANSPORT_DATA: return XI_FRM_TRANSPORT_DATA;
            default: return XI_MONO8;
            }
        }
    };

    // XIMEACameraImpl implementation
    XIMEACameraImpl::XIMEACameraImpl() : d(std::make_unique<XIMEACameraPrivate>()) {
    }

    XIMEACameraImpl::~XIMEACameraImpl() = default;

    ReturnCode XIMEACameraImpl::OpenDevice(int deviceIndex, void** handle) {
        HANDLE xiHandle;
        XI_RETURN stat = xiOpenDevice(deviceIndex, &xiHandle);
        if (stat == XI_OK) {
            *handle = xiHandle;
        }
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::CloseDevice(void* handle) {
        XI_RETURN stat = xiCloseDevice(handle);
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::GetNumberDevices(uint32_t* count) {
        DWORD deviceCount = 0;
        xiGetNumberDevices(&deviceCount);
        *count = deviceCount;
        return ReturnCode::OK;
    }

    ReturnCode XIMEACameraImpl::GetDeviceInfoString(int deviceIndex, ParamType param, char* value, size_t size) {
        const char* xiParam = d->GetXiParamString(param);
        if (!xiParam) {
            return ReturnCode::UNKNOWN_PARAM;
        }

        XI_RETURN stat = xiGetDeviceInfoString(deviceIndex, xiParam, value, static_cast<DWORD>(size));
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::StartAcquisition(void* handle) {
        XI_RETURN stat = xiStartAcquisition(handle);
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::StopAcquisition(void* handle) {
        XI_RETURN stat = xiStopAcquisition(handle);
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::GetImage(void* handle, uint32_t timeout_ms, ImageData* image) {
        XI_IMG xiImage;
        memset(&xiImage, 0, sizeof(xiImage));
        xiImage.size = sizeof(XI_IMG);

        XI_RETURN stat = xiGetImage(handle, timeout_ms, &xiImage);

        if (stat == XI_OK) {
            // Convert XI_IMG to ImageData
            image->bp = xiImage.bp;
            image->bp_size = xiImage.bp_size;
            image->frm = d->ConvertXiFormat(xiImage.frm);
            image->width = xiImage.width;
            image->height = xiImage.height;
            image->nframe = xiImage.nframe;
            image->tsSec = xiImage.tsSec;
            image->tsUSec = xiImage.tsUSec;
            image->GPI_level = xiImage.GPI_level;
            image->black_level = xiImage.black_level;
            image->padding_x = xiImage.padding_x;
            image->AbsoluteOffsetX = xiImage.AbsoluteOffsetX;
            image->AbsoluteOffsetY = xiImage.AbsoluteOffsetY;
            image->transport_frm = xiImage.transport_frm;
            image->img_desc = xiImage.img_desc;
            image->DownsamplingX = xiImage.DownsamplingX;
            image->DownsamplingY = xiImage.DownsamplingY;
            image->flags = xiImage.flags;
            image->exposure_time_us = xiImage.exposure_time_us;
            image->gain_db = xiImage.gain_db;
            image->acq_nframe = xiImage.acq_nframe;
            image->image_user_data = xiImage.image_user_data;
            memcpy(image->exposure_sub_times_us, xiImage.exposure_sub_times_us,
                sizeof(image->exposure_sub_times_us));
        }

        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::SetParamInt(void* handle, ParamType param, int value) {
        const char* xiParam = d->GetXiParamString(param);
        if (!xiParam) {
            return ReturnCode::UNKNOWN_PARAM;
        }

        XI_RETURN stat = xiSetParamInt(handle, xiParam, value);
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::GetParamInt(void* handle, ParamType param, int* value) {
        const char* xiParam = d->GetXiParamString(param);
        if (!xiParam) {
            return ReturnCode::UNKNOWN_PARAM;
        }

        XI_RETURN stat = xiGetParamInt(handle, xiParam, value);
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::SetParamFloat(void* handle, ParamType param, float value) {
        const char* xiParam = d->GetXiParamString(param);
        if (!xiParam) {
            return ReturnCode::UNKNOWN_PARAM;
        }

        XI_RETURN stat = xiSetParamFloat(handle, xiParam, value);
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::GetParamFloat(void* handle, ParamType param, float* value) {
        const char* xiParam = d->GetXiParamString(param);
        if (!xiParam) {
            return ReturnCode::UNKNOWN_PARAM;
        }

        XI_RETURN stat = xiGetParamFloat(handle, xiParam, value);
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::SetParamString(void* handle, ParamType param, const char* value) {
        const char* xiParam = d->GetXiParamString(param);
        if (!xiParam) {
            return ReturnCode::UNKNOWN_PARAM;
        }

        XI_RETURN stat = xiSetParamString(handle, xiParam, const_cast<char*>(value),
            static_cast<DWORD>(strlen(value) + 1));
        return d->ConvertXiReturn(stat);
    }

    ReturnCode XIMEACameraImpl::GetParamString(void* handle, ParamType param, char* value, size_t size) {
        const char* xiParam = d->GetXiParamString(param);
        if (!xiParam) {
            return ReturnCode::UNKNOWN_PARAM;
        }

        XI_RETURN stat = xiGetParamString(handle, xiParam, value, static_cast<DWORD>(size));
        return d->ConvertXiReturn(stat);
    }

    std::string XIMEACameraImpl::GetErrorString(ReturnCode error) {
        XI_RETURN xiError = d->ConvertToXiReturn(error);

        switch (xiError) {
        case XI_OK: return "Success";
        case XI_INVALID_HANDLE: return "Invalid handle";
        case XI_READREG: return "Register read error";
        case XI_WRITEREG: return "Register write error";
        case XI_FREE_RESOURCES: return "Free resources error";
        case XI_FREE_CHANNEL: return "Free channel error";
        case XI_FREE_BANDWIDTH: return "Free bandwidth error";
        case XI_READBLK: return "Read block error";
        case XI_WRITEBLK: return "Write block error";
        case XI_NO_IMAGE: return "No image";
        case XI_TIMEOUT: return "Timeout";
        case XI_INVALID_ARG: return "Invalid argument";
        case XI_NOT_SUPPORTED: return "Not supported";
        case XI_MEMORY_ALLOCATION: return "Memory allocation error";
        case XI_DEVICE_NOT_READY: return "Device not ready";
        case XI_ACQUISITION_STOPED: return "Acquisition stopped";
        case XI_WRONG_PARAM_VALUE: return "Wrong parameter value";
        case XI_NO_DEVICES_FOUND: return "No devices found";
            // ... add all other error strings
        default: return "Unknown error (" + std::to_string(static_cast<int>(error)) + ")";
        }
    }

} // namespace Camera