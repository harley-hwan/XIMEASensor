#include "pch.h"
#include "HikVisionCameraImpl.h"
#include "Logger.h"
#include <sstream>
#include <cstring>

namespace Camera {

    HikVisionCameraImpl::HikVisionCameraImpl() {
        memset(&m_deviceList, 0, sizeof(m_deviceList));
    }

    HikVisionCameraImpl::~HikVisionCameraImpl() {
        // Cleanup any open handles
    }

    ReturnCode HikVisionCameraImpl::OpenDevice(int deviceIndex, void** handle) {
        if (!handle) return ReturnCode::INVALID_ARG;

        // Enumerate devices if needed
        uint32_t count = 0;
        ReturnCode ret = GetNumberDevices(&count);
        if (ret != ReturnCode::OK) return ret;

        if (deviceIndex >= static_cast<int>(count)) {
            return ReturnCode::INVALID_ARG;
        }

        std::lock_guard<std::mutex> lock(m_deviceListMutex);

        MV_CC_DEVICE_INFO* pDeviceInfo = m_deviceList.pDeviceInfo[deviceIndex];
        if (!pDeviceInfo) return ReturnCode::INVALID_ARG;

        // Create handle structure
        auto* hikHandle = new HikHandle();
        hikHandle->deviceInfo = *pDeviceInfo;
        hikHandle->isAcquiring = false;
        hikHandle->lastFrameNumber = 0;

        // Create HikVision handle
        int mvRet = MV_CC_CreateHandle(&hikHandle->deviceHandle, pDeviceInfo);
        if (mvRet != MV_OK) {
            delete hikHandle;
            LOG_ERROR("Failed to create HikVision handle: " + std::to_string(mvRet));
            return ConvertMvError(mvRet);
        }

        // Open device
        mvRet = MV_CC_OpenDevice(hikHandle->deviceHandle);
        if (mvRet != MV_OK) {
            MV_CC_DestroyHandle(hikHandle->deviceHandle);
            delete hikHandle;
            LOG_ERROR("Failed to open HikVision device: " + std::to_string(mvRet));
            return ConvertMvError(mvRet);
        }

        // Configure basic parameters
        MV_CC_SetEnumValue(hikHandle->deviceHandle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        MV_CC_SetEnumValue(hikHandle->deviceHandle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        MV_CC_SetIntValue(hikHandle->deviceHandle, "GevSCPSPacketSize", 1500);

        *handle = hikHandle;
        LOG_INFO("HikVision camera opened successfully");
        return ReturnCode::OK;
    }

    ReturnCode HikVisionCameraImpl::CloseDevice(void* handle) {
        if (!handle) return ReturnCode::INVALID_ARG;

        auto* hikHandle = static_cast<HikHandle*>(handle);

        if (hikHandle->isAcquiring) {
            StopAcquisition(handle);
        }

        MV_CC_CloseDevice(hikHandle->deviceHandle);
        MV_CC_DestroyHandle(hikHandle->deviceHandle);

        delete hikHandle;
        return ReturnCode::OK;
    }

    ReturnCode HikVisionCameraImpl::GetNumberDevices(uint32_t* count) {
        if (!count) return ReturnCode::INVALID_ARG;

        // Check cache
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastEnumTime).count();

        if (elapsed > ENUM_CACHE_MS) {
            std::lock_guard<std::mutex> lock(m_deviceListMutex);

            memset(&m_deviceList, 0, sizeof(m_deviceList));
            int mvRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_deviceList);
            if (mvRet != MV_OK) {
                LOG_ERROR("Failed to enumerate HikVision devices");
                return ConvertMvError(mvRet);
            }

            m_lastEnumTime = now;
        }

        *count = m_deviceList.nDeviceNum;
        return ReturnCode::OK;
    }

    ReturnCode HikVisionCameraImpl::GetDeviceInfoString(int deviceIndex, ParamType param,
        char* value, size_t size) {
        if (!value || size == 0) return ReturnCode::INVALID_ARG;

        uint32_t count = 0;
        ReturnCode ret = GetNumberDevices(&count);
        if (ret != ReturnCode::OK) return ret;

        if (deviceIndex >= static_cast<int>(count)) {
            return ReturnCode::INVALID_ARG;
        }

        std::lock_guard<std::mutex> lock(m_deviceListMutex);
        MV_CC_DEVICE_INFO* pDeviceInfo = m_deviceList.pDeviceInfo[deviceIndex];
        if (!pDeviceInfo) return ReturnCode::INVALID_ARG;

        std::string result;

        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            auto& gigEInfo = pDeviceInfo->SpecialInfo.stGigEInfo;

            switch (param) {
            case ParamType::DEVICE_NAME:
                result = reinterpret_cast<const char*>(gigEInfo.chModelName);
                break;
            case ParamType::DEVICE_SN:
                result = reinterpret_cast<const char*>(gigEInfo.chSerialNumber);
                break;
            default:
                return ReturnCode::NOT_SUPPORTED_PARAM;
            }
        }
        else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            auto& usbInfo = pDeviceInfo->SpecialInfo.stUsb3VInfo;

            switch (param) {
            case ParamType::DEVICE_NAME:
                result = reinterpret_cast<const char*>(usbInfo.chModelName);
                break;
            case ParamType::DEVICE_SN:
                result = reinterpret_cast<const char*>(usbInfo.chSerialNumber);
                break;
            default:
                return ReturnCode::NOT_SUPPORTED_PARAM;
            }
        }

        strncpy_s(value, size, result.c_str(), _TRUNCATE);
        return ReturnCode::OK;
    }

    ReturnCode HikVisionCameraImpl::StartAcquisition(void* handle) {
        if (!handle) return ReturnCode::INVALID_ARG;

        auto* hikHandle = static_cast<HikHandle*>(handle);

        if (hikHandle->isAcquiring) {
            return ReturnCode::ACQUISITION_ALREADY_UP;
        }

        // Register callback
        int mvRet = MV_CC_RegisterImageCallBackEx(hikHandle->deviceHandle,
            ImageCallbackEx, hikHandle);
        if (mvRet != MV_OK) {
            LOG_ERROR("Failed to register callback");
            return ConvertMvError(mvRet);
        }

        // Start grabbing
        mvRet = MV_CC_StartGrabbing(hikHandle->deviceHandle);
        if (mvRet != MV_OK) {
            LOG_ERROR("Failed to start grabbing");
            return ConvertMvError(mvRet);
        }

        hikHandle->isAcquiring = true;
        hikHandle->lastFrameTime = std::chrono::steady_clock::now();
        return ReturnCode::OK;
    }

    ReturnCode HikVisionCameraImpl::StopAcquisition(void* handle) {
        if (!handle) return ReturnCode::INVALID_ARG;

        auto* hikHandle = static_cast<HikHandle*>(handle);

        if (!hikHandle->isAcquiring) {
            return ReturnCode::ACQUISITION_STOPED;
        }

        MV_CC_StopGrabbing(hikHandle->deviceHandle);
        hikHandle->isAcquiring = false;

        // Clear queue
        {
            std::lock_guard<std::mutex> lock(hikHandle->queueMutex);
            while (!hikHandle->imageQueue.empty()) {
                hikHandle->imageQueue.pop();
            }
        }

        return ReturnCode::OK;
    }

    ReturnCode HikVisionCameraImpl::GetImage(void* handle, uint32_t timeout_ms, ImageData* image) {
        if (!handle || !image) return ReturnCode::INVALID_ARG;

        auto* hikHandle = static_cast<HikHandle*>(handle);

        if (!hikHandle->isAcquiring) {
            return ReturnCode::ACQUISITION_STOPED;
        }

        std::unique_lock<std::mutex> lock(hikHandle->queueMutex);

        if (hikHandle->queueCV.wait_for(lock, std::chrono::milliseconds(timeout_ms),
            [hikHandle] { return !hikHandle->imageQueue.empty(); })) {

            auto imageData = std::move(hikHandle->imageQueue.front());
            hikHandle->imageQueue.pop();

            *image = *imageData;
            return ReturnCode::OK;
        }

        return ReturnCode::TIMEOUT;
    }

    void __stdcall HikVisionCameraImpl::ImageCallbackEx(unsigned char* pData,
        MV_FRAME_OUT_INFO_EX* pFrameInfo,
        void* pUser) {
        // pUser contains the HikHandle pointer, not the impl pointer
        auto* handle = static_cast<HikHandle*>(pUser);
        if (!handle || !pData || !pFrameInfo) {
            return;
        }

        // Process directly without needing 'this'
        auto imageData = std::make_unique<ImageData>();

        // Fill ImageData structure
        imageData->size = sizeof(ImageData);
        imageData->bp_size = pFrameInfo->nFrameLen;
        imageData->bp = new unsigned char[imageData->bp_size];
        memcpy(imageData->bp, pData, imageData->bp_size);

        imageData->width = pFrameInfo->nWidth;
        imageData->height = pFrameInfo->nHeight;
        imageData->nframe = static_cast<uint32_t>(pFrameInfo->nFrameNum);
        imageData->exposure_time_us = static_cast<uint32_t>(pFrameInfo->fExposureTime);
        imageData->gain_db = pFrameInfo->fGain;

        // Convert pixel format
        switch (pFrameInfo->enPixelType) {
        case PixelType_Gvsp_Mono8:
            imageData->frm = Camera::ImageFormat::MONO8;
            break;
        case PixelType_Gvsp_Mono16:
            imageData->frm = Camera::ImageFormat::MONO16;
            break;
        default:
            imageData->frm = Camera::ImageFormat::MONO8;
            break;
        }

        // Calculate timestamp
        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration - seconds);
        imageData->tsSec = static_cast<uint32_t>(seconds.count());
        imageData->tsUSec = static_cast<uint32_t>(microseconds.count());

        // Frame drop detection
        uint64_t currentFrame = pFrameInfo->nFrameNum;
        uint64_t lastFrame = handle->lastFrameNumber.exchange(currentFrame);
        if (lastFrame != 0 && currentFrame > lastFrame + 1) {
            LOG_WARNING("Frame drop detected: " + std::to_string(currentFrame - lastFrame - 1) + " frames");
        }

        // Add to queue
        {
            std::lock_guard<std::mutex> lock(handle->queueMutex);

            // Limit queue size
            while (handle->imageQueue.size() >= 5) {
                auto& front = handle->imageQueue.front();
                if (front && front->bp) {
                    delete[] static_cast<unsigned char*>(front->bp);
                    front->bp = nullptr;
                }
                handle->imageQueue.pop();
            }

            handle->imageQueue.push(std::move(imageData));
            handle->queueCV.notify_one();
        }
    }

    void HikVisionCameraImpl::ProcessImageCallback(HikHandle* handle,
        unsigned char* pData,
        MV_FRAME_OUT_INFO_EX* pFrameInfo) {
        auto imageData = std::make_unique<ImageData>();

        // Fill ImageData structure
        imageData->bp_size = pFrameInfo->nFrameLen;
        imageData->bp = new unsigned char[imageData->bp_size];
        memcpy(imageData->bp, pData, imageData->bp_size);

        imageData->width = pFrameInfo->nWidth;
        imageData->height = pFrameInfo->nHeight;
        imageData->frm = ConvertFromMvPixelFormat(pFrameInfo->enPixelType);
        imageData->nframe = static_cast<uint32_t>(pFrameInfo->nFrameNum);
        imageData->exposure_time_us = static_cast<uint32_t>(pFrameInfo->fExposureTime);
        imageData->gain_db = pFrameInfo->fGain;

        // Calculate timestamp
        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration - seconds);
        imageData->tsSec = static_cast<uint32_t>(seconds.count());
        imageData->tsUSec = static_cast<uint32_t>(microseconds.count());

        // Frame rate calculation
        auto frameTime = now;
        auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(
            frameTime - handle->lastFrameTime).count();
        if (timeDiff > 0) {
            float fps = 1000000.0f / timeDiff;
            // Store FPS in black_level field (as a workaround)
            imageData->black_level = static_cast<uint32_t>(fps * 100); // Store as fixed point
        }
        handle->lastFrameTime = frameTime;

        // Add to queue
        {
            std::lock_guard<std::mutex> lock(handle->queueMutex);

            // Limit queue size
            while (handle->imageQueue.size() >= 5) {
                handle->imageQueue.pop();
            }

            handle->imageQueue.push(std::move(imageData));
            handle->queueCV.notify_one();
        }
    }

    ReturnCode HikVisionCameraImpl::SetParamInt(void* handle, ParamType param, int value) {
        if (!handle) return ReturnCode::INVALID_ARG;

        auto* hikHandle = static_cast<HikHandle*>(handle);
        int mvRet = MV_OK;

        switch (param) {
        case ParamType::WIDTH:
            mvRet = MV_CC_SetIntValue(hikHandle->deviceHandle, "Width", value);
            break;
        case ParamType::HEIGHT:
            mvRet = MV_CC_SetIntValue(hikHandle->deviceHandle, "Height", value);
            break;
        case ParamType::OFFSET_X:
            mvRet = MV_CC_SetIntValue(hikHandle->deviceHandle, "OffsetX", value);
            break;
        case ParamType::OFFSET_Y:
            mvRet = MV_CC_SetIntValue(hikHandle->deviceHandle, "OffsetY", value);
            break;
        case ParamType::EXPOSURE:
            mvRet = MV_CC_SetFloatValue(hikHandle->deviceHandle, "ExposureTime",
                static_cast<float>(value));
            break;
        case ParamType::IMAGE_DATA_FORMAT:
            mvRet = MV_CC_SetEnumValue(hikHandle->deviceHandle, "PixelFormat",
                ConvertToMvPixelFormat(static_cast<ImageFormat>(value)));
            break;
        case ParamType::TRG_SOURCE:
            if (value == static_cast<int>(TriggerSource::OFF)) {
                mvRet = MV_CC_SetEnumValue(hikHandle->deviceHandle, "TriggerMode", MV_TRIGGER_MODE_OFF);
            }
            else {
                mvRet = MV_CC_SetEnumValue(hikHandle->deviceHandle, "TriggerMode", MV_TRIGGER_MODE_ON);
                if (mvRet == MV_OK && value == static_cast<int>(TriggerSource::SOFTWARE)) {
                    mvRet = MV_CC_SetEnumValue(hikHandle->deviceHandle, "TriggerSource",
                        MV_TRIGGER_SOURCE_SOFTWARE);
                }
            }
            break;
        default:
            return ReturnCode::NOT_SUPPORTED_PARAM;
        }

        return ConvertMvError(mvRet);
    }

    ReturnCode HikVisionCameraImpl::GetParamInt(void* handle, ParamType param, int* value) {
        if (!handle || !value) return ReturnCode::INVALID_ARG;

        auto* hikHandle = static_cast<HikHandle*>(handle);
        MVCC_INTVALUE stIntValue = { 0 };
        int mvRet = MV_OK;

        switch (param) {
        case ParamType::WIDTH:
            mvRet = MV_CC_GetIntValue(hikHandle->deviceHandle, "Width", &stIntValue);
            *value = static_cast<int>(stIntValue.nCurValue);
            break;
        case ParamType::HEIGHT:
            mvRet = MV_CC_GetIntValue(hikHandle->deviceHandle, "Height", &stIntValue);
            *value = static_cast<int>(stIntValue.nCurValue);
            break;
        case ParamType::EXPOSURE:
        {
            MVCC_FLOATVALUE stFloatValue = { 0 };
            mvRet = MV_CC_GetFloatValue(hikHandle->deviceHandle, "ExposureTime", &stFloatValue);
            *value = static_cast<int>(stFloatValue.fCurValue);
        }
        break;
        default:
            return ReturnCode::NOT_SUPPORTED_PARAM;
        }

        return ConvertMvError(mvRet);
    }

    ReturnCode HikVisionCameraImpl::SetParamFloat(void* handle, ParamType param, float value) {
        if (!handle) return ReturnCode::INVALID_ARG;

        auto* hikHandle = static_cast<HikHandle*>(handle);
        int mvRet = MV_OK;

        switch (param) {
        case ParamType::GAIN:
            mvRet = MV_CC_SetFloatValue(hikHandle->deviceHandle, "Gain", value);
            break;
        case ParamType::FRAMERATE:
            mvRet = MV_CC_SetFloatValue(hikHandle->deviceHandle, "AcquisitionFrameRate", value);
            break;
        default:
            return ReturnCode::NOT_SUPPORTED_PARAM;
        }

        return ConvertMvError(mvRet);
    }

    ReturnCode HikVisionCameraImpl::GetParamFloat(void* handle, ParamType param, float* value) {
        if (!handle || !value) return ReturnCode::INVALID_ARG;

        auto* hikHandle = static_cast<HikHandle*>(handle);
        MVCC_FLOATVALUE stFloatValue = { 0 };
        int mvRet = MV_OK;

        switch (param) {
        case ParamType::GAIN:
            mvRet = MV_CC_GetFloatValue(hikHandle->deviceHandle, "Gain", &stFloatValue);
            *value = stFloatValue.fCurValue;
            break;
        case ParamType::FRAMERATE:
            mvRet = MV_CC_GetFloatValue(hikHandle->deviceHandle, "AcquisitionFrameRate", &stFloatValue);
            *value = stFloatValue.fCurValue;
            break;
        default:
            return ReturnCode::NOT_SUPPORTED_PARAM;
        }

        return ConvertMvError(mvRet);
    }

    ReturnCode HikVisionCameraImpl::ConvertMvError(int mvError) {
        switch (mvError) {
        case MV_OK:
            return ReturnCode::OK;
        case MV_E_HANDLE:
            return ReturnCode::INVALID_HANDLE;
        case MV_E_SUPPORT:
            return ReturnCode::NOT_SUPPORTED;
        case MV_E_BUFOVER:
            return ReturnCode::BUFFER_TOO_SMALL;
        case MV_E_NODATA:
            return ReturnCode::NO_IMAGE;
        case MV_E_RESOURCE:
            return ReturnCode::FREE_RESOURCES;
        case MV_E_PARAMETER:
            return ReturnCode::INVALID_ARG;
        }
    }

    ImageFormat HikVisionCameraImpl::ConvertFromMvPixelFormat(MvGvspPixelType mvFormat) {
        switch (mvFormat) {
        case PixelType_Gvsp_Mono8:
            return ImageFormat::MONO8;
        case PixelType_Gvsp_Mono16:
            return ImageFormat::MONO16;
        case PixelType_Gvsp_RGB8_Packed:
            return ImageFormat::RGB24;
        case PixelType_Gvsp_BGR8_Packed:
            return ImageFormat::RGB24; // Convert BGR to RGB if needed
        default:
            return ImageFormat::MONO8;
        }
    }

    MvGvspPixelType HikVisionCameraImpl::ConvertToMvPixelFormat(ImageFormat format) {
        switch (format) {
        case ImageFormat::MONO8:
            return PixelType_Gvsp_Mono8;
        case ImageFormat::MONO16:
            return PixelType_Gvsp_Mono16;
        case ImageFormat::RGB24:
            return PixelType_Gvsp_RGB8_Packed;
        default:
            return PixelType_Gvsp_Mono8;
        }
    }

    std::string HikVisionCameraImpl::GetErrorString(ReturnCode error) {
        switch (error) {
        case ReturnCode::OK:
            return "Success";
        case ReturnCode::INVALID_HANDLE:
            return "Invalid handle";
        case ReturnCode::TIMEOUT:
            return "Timeout";
        case ReturnCode::INVALID_ARG:
            return "Invalid argument";
        case ReturnCode::NOT_SUPPORTED:
            return "Not supported";
        case ReturnCode::NO_IMAGE:
            return "No image available";
        case ReturnCode::ACQUISITION_ALREADY_UP:
            return "Acquisition already started";
        case ReturnCode::ACQUISITION_STOPED:
            return "Acquisition stopped";
        default:
            return "Unknown error";
        }
    }

    // SetParamString and GetParamString implementations...
    ReturnCode HikVisionCameraImpl::SetParamString(void* handle, ParamType param, const char* value) {
        // HikVision doesn't use string parameters in our implementation
        return ReturnCode::NOT_SUPPORTED_PARAM;
    }

    ReturnCode HikVisionCameraImpl::GetParamString(void* handle, ParamType param, char* value, size_t size) {
        // Implement if needed
        return ReturnCode::NOT_SUPPORTED_PARAM;
    }

} // namespace Camera