#include "pch.h"
#include "HikVisionCameraImpl.h"
#include "Logger.h"
#include <sstream>
#include <cstring>

namespace Camera {

    HikVisionCameraImpl::HikVisionCameraImpl()
        : m_deviceHandle(nullptr)
        , m_connected(false)
        , m_acquiring(false)
        , m_lastFrameNumber(0)
        , m_callbackRegistered(false) {
        memset(&m_deviceInfo, 0, sizeof(m_deviceInfo));
    }

    HikVisionCameraImpl::~HikVisionCameraImpl() {
        // 소멸자에서 안전한 정리
        if (m_acquiring.load()) {
            StopAcquisition(m_deviceHandle);
        }
        if (m_connected.load()) {
            CloseDevice(m_deviceHandle);
        }
    }

    ReturnCode HikVisionCameraImpl::OpenDevice(int deviceIndex, void** handle) {
        if (!handle) return ReturnCode::INVALID_ARG;
        *handle = nullptr;

        // 이미 연결된 경우
        if (m_connected.load()) {
            LOG_WARNING("Device already connected");
            return ReturnCode::INVALID_HANDLE;
        }

        // 디바이스 목록 가져오기
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet) {
            SetError("Failed to enumerate devices");
            return ReturnCode::NO_DEVICES_FOUND;
        }

        if (deviceIndex >= stDeviceList.nDeviceNum) {
            SetError("Invalid device index");
            return ReturnCode::INVALID_ARG;
        }

        // 디바이스 정보 저장
        m_deviceInfo = *(stDeviceList.pDeviceInfo[deviceIndex]);

        // 핸들 생성
        nRet = MV_CC_CreateHandle(&m_deviceHandle, &m_deviceInfo);
        if (MV_OK != nRet) {
            SetError("Failed to create device handle");
            return ConvertMvError(nRet);
        }

        // 디바이스 열기
        nRet = MV_CC_OpenDevice(m_deviceHandle);
        if (MV_OK != nRet) {
            MV_CC_DestroyHandle(m_deviceHandle);
            m_deviceHandle = nullptr;
            SetError("Failed to open device");
            return ConvertMvError(nRet);
        }

        // 디바이스 설정
        if (!ConfigureDevice()) {
            MV_CC_CloseDevice(m_deviceHandle);
            MV_CC_DestroyHandle(m_deviceHandle);
            m_deviceHandle = nullptr;
            return ReturnCode::DEVICE_NOT_READY;
        }

        m_connected = true;
        *handle = m_deviceHandle;
        LOG_INFO("HikVision device opened successfully");

        return ReturnCode::OK;
    }

    ReturnCode HikVisionCameraImpl::CloseDevice(void* handle) {
        if (!handle || handle != m_deviceHandle) {
            return ReturnCode::INVALID_HANDLE;
        }

        // 이미 닫혀있는 경우
        if (!m_connected.load()) {
            LOG_WARNING("Device already closed");
            return ReturnCode::OK;
        }

        LOG_INFO("Closing HikVision device...");

        // 1. 획득 중이면 먼저 중지
        if (m_acquiring.load()) {
            LOG_INFO("Stopping acquisition before closing device");
            StopAcquisition(handle);

            // 획득이 완전히 중지될 때까지 대기
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 2. 콜백 해제
        if (m_callbackRegistered) {
            MV_CC_RegisterImageCallBackEx(m_deviceHandle, nullptr, nullptr);
            m_callbackRegistered = false;
            LOG_INFO("Image callback unregistered");
        }

        // 3. 이미지 큐 정리
        ClearImageQueue();

        // 4. 디바이스 닫기
        int nRet = MV_CC_CloseDevice(m_deviceHandle);
        if (MV_OK != nRet) {
            LOG_ERROR("Failed to close device: " + std::to_string(nRet));
        }

        // 5. 핸들 해제
        nRet = MV_CC_DestroyHandle(m_deviceHandle);
        if (MV_OK != nRet) {
            LOG_ERROR("Failed to destroy handle: " + std::to_string(nRet));
        }

        // 6. 상태 초기화
        m_deviceHandle = nullptr;
        m_connected = false;
        m_acquiring = false;
        m_lastFrameNumber = 0;

        LOG_INFO("HikVision device closed successfully");
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
        if (!handle || handle != m_deviceHandle || !m_connected.load()) {
            return ReturnCode::INVALID_HANDLE;
        }

        if (m_acquiring.load()) {
            LOG_WARNING("Already acquiring");
            return ReturnCode::ACQUISITION_ALREADY_UP;
        }

        // 콜백 등록
        int nRet = MV_CC_RegisterImageCallBackEx(m_deviceHandle, ImageCallbackEx, this);
        if (MV_OK != nRet) {
            SetError("Failed to register callback");
            return ConvertMvError(nRet);
        }
        m_callbackRegistered = true;

        // 획득 시작
        nRet = MV_CC_StartGrabbing(m_deviceHandle);
        if (MV_OK != nRet) {
            MV_CC_RegisterImageCallBackEx(m_deviceHandle, nullptr, nullptr);
            m_callbackRegistered = false;
            SetError("Failed to start grabbing");
            return ConvertMvError(nRet);
        }

        m_acquiring = true;
        m_lastFrameNumber = 0;
        LOG_INFO("HikVision acquisition started");

        return ReturnCode::OK;
    }

    ReturnCode HikVisionCameraImpl::StopAcquisition(void* handle) {
        if (!handle || handle != m_deviceHandle) {
            return ReturnCode::INVALID_HANDLE;
        }

        if (!m_acquiring.load()) {
            LOG_WARNING("Not acquiring");
            return ReturnCode::OK;
        }

        LOG_INFO("Stopping HikVision acquisition...");

        // 1. 획득 중지 플래그 설정
        m_acquiring = false;

        // 2. 이미지 획득 중지
        int nRet = MV_CC_StopGrabbing(m_deviceHandle);
        if (MV_OK != nRet) {
            LOG_ERROR("Failed to stop grabbing: " + std::to_string(nRet));
        }

        // 3. 잠시 대기하여 진행 중인 콜백이 완료되도록 함
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 4. 콜백 해제
        if (m_callbackRegistered) {
            MV_CC_RegisterImageCallBackEx(m_deviceHandle, nullptr, nullptr);
            m_callbackRegistered = false;
        }

        // 5. 버퍼 정리
        nRet = MV_CC_ClearImageBuffer(m_deviceHandle);
        if (MV_OK != nRet) {
            LOG_WARNING("Failed to clear image buffer: " + std::to_string(nRet));
        }

        // 6. 큐가 비워질 때까지 대기
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_queueCV.wait_for(lock, std::chrono::milliseconds(200),
                [this] { return m_imageQueue.empty(); });
        }

        // 7. 남은 이미지 큐 강제 정리
        ClearImageQueue();

        // 8. 트리거 모드 해제 (연속 모드로 복원)
        MV_CC_SetEnumValue(m_deviceHandle, "TriggerMode", MV_TRIGGER_MODE_OFF);

        LOG_INFO("HikVision acquisition stopped successfully");
        return ReturnCode::OK;
    }

    void HikVisionCameraImpl::ClearImageQueue() {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        while (!m_imageQueue.empty()) {
            m_imageQueue.pop();
        }
        LOG_DEBUG("Image queue cleared");
    }

    bool HikVisionCameraImpl::ConfigureDevice() {
        int nRet;

        // 트리거 모드 끄기
        nRet = MV_CC_SetEnumValue(m_deviceHandle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        if (MV_OK != nRet) {
            LOG_WARNING("Failed to set trigger mode off: " + std::to_string(nRet));
        }

        // 연속 획득 모드 설정
        nRet = MV_CC_SetEnumValue(m_deviceHandle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        if (MV_OK != nRet) {
            LOG_WARNING("Failed to set continuous acquisition mode: " + std::to_string(nRet));
        }

        // 픽셀 포맷 설정 (Mono8)
        nRet = MV_CC_SetEnumValue(m_deviceHandle, "PixelFormat", PixelType_Gvsp_Mono8);
        if (MV_OK != nRet) {
            LOG_WARNING("Failed to set pixel format: " + std::to_string(nRet));
        }

        // GigE 카메라인 경우 패킷 크기 설정
        if (m_deviceInfo.nTLayerType == MV_GIGE_DEVICE) {
            // 패킷 크기 최적화
            int nPacketSize = 0;
            nRet = MV_CC_GetOptimalPacketSize(m_deviceHandle);
            if (MV_OK == nRet) {
                nRet = MV_CC_SetIntValue(m_deviceHandle, "GevSCPSPacketSize", nPacketSize);
                if (MV_OK != nRet) {
                    LOG_WARNING("Failed to set optimal packet size: " + std::to_string(nRet));
                }
            }

            // 하트비트 타임아웃 설정
            nRet = MV_CC_SetIntValue(m_deviceHandle, "GevHeartbeatTimeout", 3000);
            if (MV_OK != nRet) {
                LOG_WARNING("Failed to set heartbeat timeout: " + std::to_string(nRet));
            }
        }

        return true;
    }


    ReturnCode HikVisionCameraImpl::GetImage(void* handle, uint32_t timeout_ms, ImageData* image) {
        if (!handle || handle != m_deviceHandle || !image) {
            return ReturnCode::INVALID_ARG;
        }

        if (!m_acquiring.load()) {
            return ReturnCode::ACQUISITION_STOPED;
        }

        std::unique_lock<std::mutex> lock(m_queueMutex);
        if (m_queueCV.wait_for(lock, std::chrono::milliseconds(timeout_ms),
            [this] { return !m_imageQueue.empty() || !m_acquiring.load(); })) {

            if (!m_acquiring.load()) {
                return ReturnCode::ACQUISITION_STOPED;
            }

            if (!m_imageQueue.empty()) {
                *image = m_imageQueue.front();
                m_imageQueue.pop();
                return ReturnCode::OK;
            }
        }

        return ReturnCode::TIMEOUT;
    }

    void __stdcall HikVisionCameraImpl::ImageCallbackEx(unsigned char* pData,
        MV_FRAME_OUT_INFO_EX* pFrameInfo,
        void* pUser) {
        auto* pThis = static_cast<HikVisionCameraImpl*>(pUser);
        if (pThis && pThis->m_acquiring.load()) {
            pThis->ProcessImage(pData, pFrameInfo);
        }
    }

    void HikVisionCameraImpl::ProcessImage(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo) {
        if (!pData || !pFrameInfo) return;

        // 프레임 드롭 감지
        uint64_t currentFrame = pFrameInfo->nFrameNum;
        uint64_t expectedFrame = m_lastFrameNumber.load() + 1;
        if (m_lastFrameNumber != 0 && currentFrame > expectedFrame) {
            LOG_WARNING("Frame drop detected: " +
                std::to_string(currentFrame - expectedFrame) + " frames lost");
        }
        m_lastFrameNumber = currentFrame;

        // ImageData 생성
        ImageData image;
        image.size = sizeof(ImageData);
        image.bp = pData;
        image.bp_size = pFrameInfo->nFrameLen;
        image.width = pFrameInfo->nWidth;
        image.height = pFrameInfo->nHeight;
        image.nframe = pFrameInfo->nFrameNum;
        image.tsSec = static_cast<uint32_t>(pFrameInfo->nDevTimeStampHigh);
        image.tsUSec = pFrameInfo->nDevTimeStampLow / 1000;
        image.exposure_time_us = static_cast<uint32_t>(pFrameInfo->fExposureTime);
        image.gain_db = pFrameInfo->fGain;

        // 픽셀 포맷 변환 추가
        image.frm = ConvertFromMvPixelFormat(pFrameInfo->enPixelType);

        // 큐에 추가
        {
            std::lock_guard<std::mutex> lock(m_queueMutex);

            // 큐 크기 제한
            while (m_imageQueue.size() >= MAX_QUEUE_SIZE) {
                m_imageQueue.pop();
            }

            m_imageQueue.push(image);
            m_queueCV.notify_one();
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
        // handle 검증 수정
        if (!handle || handle != m_deviceHandle || !m_connected.load()) {
            return ReturnCode::INVALID_HANDLE;
        }

        int mvRet = MV_OK;

        switch (param) {
        case ParamType::WIDTH:
            mvRet = MV_CC_SetIntValue(m_deviceHandle, "Width", value);
            break;
        case ParamType::HEIGHT:
            mvRet = MV_CC_SetIntValue(m_deviceHandle, "Height", value);
            break;
        case ParamType::OFFSET_X:
            mvRet = MV_CC_SetIntValue(m_deviceHandle, "OffsetX", value);
            break;
        case ParamType::OFFSET_Y:
            mvRet = MV_CC_SetIntValue(m_deviceHandle, "OffsetY", value);
            break;
        case ParamType::EXPOSURE:
            mvRet = MV_CC_SetFloatValue(m_deviceHandle, "ExposureTime",
                static_cast<float>(value));
            break;
        case ParamType::IMAGE_DATA_FORMAT:
            mvRet = MV_CC_SetEnumValue(m_deviceHandle, "PixelFormat",
                ConvertToMvPixelFormat(static_cast<ImageFormat>(value)));
            break;
        case ParamType::TRG_SOURCE:
            if (value == static_cast<int>(TriggerSource::OFF)) {
                mvRet = MV_CC_SetEnumValue(m_deviceHandle, "TriggerMode", MV_TRIGGER_MODE_OFF);
            }
            else {
                mvRet = MV_CC_SetEnumValue(m_deviceHandle, "TriggerMode", MV_TRIGGER_MODE_ON);
                if (mvRet == MV_OK && value == static_cast<int>(TriggerSource::SOFTWARE)) {
                    mvRet = MV_CC_SetEnumValue(m_deviceHandle, "TriggerSource",
                        MV_TRIGGER_SOURCE_SOFTWARE);
                }
            }
            break;
        case ParamType::OUTPUT_DATA_BIT_DEPTH:
        case ParamType::SENSOR_DATA_BIT_DEPTH:
        case ParamType::ACQ_TIMING_MODE:
        case ParamType::BUFFER_POLICY:
        case ParamType::AUTO_BANDWIDTH_CALCULATION:
        case ParamType::DOWNSAMPLING:
        case ParamType::DOWNSAMPLING_TYPE:
        case ParamType::SENSOR_TAPS:
        case ParamType::BUFFERS_QUEUE_SIZE:
        case ParamType::RECENT_FRAME:
        case ParamType::HDR:
        case ParamType::AUTO_WB:
        case ParamType::MANUAL_WB:
        case ParamType::SENS_DEFECTS_CORR:
        case ParamType::COLOR_FILTER_ARRAY:
        case ParamType::TRANSPORT_PIXEL_FORMAT:
            // HikVision에서 지원하지 않는 파라미터들
            LOG_WARNING("Parameter not supported by HikVision: " + std::to_string(static_cast<int>(param)));
            return ReturnCode::NOT_SUPPORTED_PARAM;
        default:
            return ReturnCode::NOT_SUPPORTED_PARAM;
        }

        return ConvertMvError(mvRet);  // 수정됨
    }

    ReturnCode HikVisionCameraImpl::GetParamInt(void* handle, ParamType param, int* value) {
        if (!handle || handle != m_deviceHandle || !value || !m_connected.load()) {
            return ReturnCode::INVALID_HANDLE;
        }

        MVCC_INTVALUE stIntValue = { 0 };
        int mvRet = MV_OK;

        switch (param) {
        case ParamType::WIDTH:
            mvRet = MV_CC_GetIntValue(m_deviceHandle, "Width", &stIntValue);
            *value = static_cast<int>(stIntValue.nCurValue);
            break;
        case ParamType::HEIGHT:
            mvRet = MV_CC_GetIntValue(m_deviceHandle, "Height", &stIntValue);
            *value = static_cast<int>(stIntValue.nCurValue);
            break;
        case ParamType::EXPOSURE:
        {
            MVCC_FLOATVALUE stFloatValue = { 0 };
            mvRet = MV_CC_GetFloatValue(m_deviceHandle, "ExposureTime", &stFloatValue);
            *value = static_cast<int>(stFloatValue.fCurValue);
        }
        break;
        default:
            return ReturnCode::NOT_SUPPORTED_PARAM;
        }

        return ConvertMvError(mvRet);  // 수정됨
    }

    ReturnCode HikVisionCameraImpl::SetParamFloat(void* handle, ParamType param, float value) {
        if (!handle || handle != m_deviceHandle || !m_connected.load()) {
            return ReturnCode::INVALID_HANDLE;
        }

        int mvRet = MV_OK;

        switch (param) {
        case ParamType::GAIN:
            mvRet = MV_CC_SetFloatValue(m_deviceHandle, "Gain", value);
            break;
        case ParamType::FRAMERATE:
            mvRet = MV_CC_SetFloatValue(m_deviceHandle, "AcquisitionFrameRate", value);
            break;
        case ParamType::GAMMA:
            mvRet = MV_CC_SetFloatValue(m_deviceHandle, "Gamma", value);
            break;
        case ParamType::SHARPNESS:
            // HikVision에서 지원하지 않는 파라미터
            LOG_WARNING("Parameter not supported by HikVision: " + std::to_string(static_cast<int>(param)));
            return ReturnCode::NOT_SUPPORTED_PARAM;
        default:
            return ReturnCode::NOT_SUPPORTED_PARAM;
        }

        return ConvertMvError(mvRet);  // 수정됨
    }

    ReturnCode HikVisionCameraImpl::GetParamFloat(void* handle, ParamType param, float* value) {
        if (!handle || handle != m_deviceHandle || !value || !m_connected.load()) {
            return ReturnCode::INVALID_HANDLE;
        }

        MVCC_FLOATVALUE stFloatValue = { 0 };
        int mvRet = MV_OK;

        switch (param) {
        case ParamType::GAIN:
            mvRet = MV_CC_GetFloatValue(m_deviceHandle, "Gain", &stFloatValue);
            *value = stFloatValue.fCurValue;
            break;
        case ParamType::FRAMERATE:
            mvRet = MV_CC_GetFloatValue(m_deviceHandle, "AcquisitionFrameRate", &stFloatValue);
            *value = stFloatValue.fCurValue;
            break;
        case ParamType::GAMMA:
            mvRet = MV_CC_GetFloatValue(m_deviceHandle, "Gamma", &stFloatValue);
            *value = stFloatValue.fCurValue;
            break;
        default:
            return ReturnCode::NOT_SUPPORTED_PARAM;
        }

        return ConvertMvError(mvRet);  // 수정됨
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
        default:
            return ReturnCode::CANT_PROCESS;
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

    ReturnCode HikVisionCameraImpl::SetParamString(void* handle, ParamType param, const char* value) {
        // HikVision doesn't use string parameters in our implementation
        return ReturnCode::NOT_SUPPORTED_PARAM;
    }

    ReturnCode HikVisionCameraImpl::GetParamString(void* handle, ParamType param, char* value, size_t size) {
        // Implement if needed
        return ReturnCode::NOT_SUPPORTED_PARAM;
    }

    void HikVisionCameraImpl::SetError(const std::string& error) const {
        std::lock_guard<std::mutex> lock(m_errorMutex);
        m_lastError = error;
        LOG_ERROR("HikVisionCamera: " + error);
    }


} // namespace Camera