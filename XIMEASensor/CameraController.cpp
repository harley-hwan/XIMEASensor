#include "pch.h"
#include "CameraController.h"
#include "Logger.h"
#include <sstream>
#include <algorithm>

namespace XimeaSensor {

    // 카메라 스펙 상수
    constexpr uint32_t PYTHON1300_WIDTH = 1280;
    constexpr uint32_t PYTHON1300_HEIGHT = 1024;
    constexpr uint32_t PYTHON1300_MAX_FPS = 210;
    constexpr float DEFAULT_EXPOSURE_US = 4000.0f;
    constexpr float DEFAULT_GAIN_DB = 0.0f;
    constexpr size_t MAX_FRAME_SIZE = PYTHON1300_WIDTH * PYTHON1300_HEIGHT;

    std::unique_ptr<CameraController> CameraController::s_instance = nullptr;
    std::mutex CameraController::s_instanceMutex;

    CameraController::CameraController()
        : m_xiHandle(nullptr)
        , m_frameCallback(nullptr)
        , m_isCapturing(false)
        , m_shouldStop(false)
        , m_writeIndex(0)
        , m_readIndex(1) {

        // 카메라 정보 초기화
        memset(&m_cameraInfo, 0, sizeof(m_cameraInfo));
        strcpy_s(m_cameraInfo.modelName, "MQ013MG-ON");
        m_cameraInfo.maxWidth = PYTHON1300_WIDTH;
        m_cameraInfo.maxHeight = PYTHON1300_HEIGHT;
        m_cameraInfo.maxFps = PYTHON1300_MAX_FPS;
        m_cameraInfo.isColorCamera = false;

        // 기본 설정
        m_settings.exposureTime = DEFAULT_EXPOSURE_US;
        m_settings.gain = DEFAULT_GAIN_DB;
        m_settings.targetFps = PYTHON1300_MAX_FPS;
        m_settings.roi = { 0, 0, PYTHON1300_WIDTH, PYTHON1300_HEIGHT };
        m_settings.autoExposure = false;
        m_settings.autoGain = false;

        // 버퍼 초기화
        for (size_t i = 0; i < BUFFER_COUNT; ++i) {
            m_frameBuffers[i].data = std::make_unique<uint8_t[]>(MAX_FRAME_SIZE);
            m_frameBuffers[i].ready = false;
        }

        // 통계 초기화
        ResetPerformanceStats();

        LOG_INFO("CameraController initialized");
    }

    CameraController::~CameraController() {
        if (m_isCapturing) {
            StopCapture();
        }
        if (m_xiHandle) {
            CloseCamera();
        }
        LOG_INFO("CameraController destroyed");
    }

    CameraController& CameraController::GetInstance() {
        std::lock_guard<std::mutex> lock(s_instanceMutex);
        if (!s_instance) {
            s_instance.reset(new CameraController());
        }
        return *s_instance;
    }

    ErrorCode CameraController::OpenCamera(int deviceIndex) {
        std::lock_guard<std::mutex> lock(m_callbackMutex);

        if (m_xiHandle) {
            LOG_WARNING("Camera already open");
            return ErrorCode::CameraAlreadyOpen;
        }

        LOG_INFO("Opening camera at index " + std::to_string(deviceIndex));

        XI_RETURN status = xiOpenDevice(deviceIndex, &m_xiHandle);
        if (status != XI_OK) {
            LOG_ERROR("Failed to open camera: " + GetXiErrorString(status));
            return ErrorCode::CameraOpenFailed;
        }

        // 시리얼 번호 읽기
        xiGetParamString(m_xiHandle, XI_PRM_DEVICE_SN, m_cameraInfo.serialNumber, sizeof(m_cameraInfo.serialNumber));
        LOG_INFO("Camera serial number: " + std::string(m_cameraInfo.serialNumber));

        // 카메라 초기 설정
        ErrorCode result = InitializeCamera();
        if (result != ErrorCode::Success) {
            xiCloseDevice(m_xiHandle);
            m_xiHandle = nullptr;
            return result;
        }

        return ErrorCode::Success;
    }

    ErrorCode CameraController::CloseCamera() {
        if (!m_xiHandle) {
            return ErrorCode::CameraNotOpen;
        }

        LOG_INFO("Closing camera");

        if (m_isCapturing) {
            StopCapture();
        }

        xiCloseDevice(m_xiHandle);
        m_xiHandle = nullptr;

        return ErrorCode::Success;
    }

    ErrorCode CameraController::InitializeCamera() {
        // 픽셀 포맷 설정 (8비트 모노크롬)
        XI_RETURN status = xiSetParamInt(m_xiHandle, XI_PRM_IMAGE_DATA_FORMAT, XI_MONO8);
        if (status != XI_OK) {
            LOG_ERROR("Failed to set pixel format: " + GetXiErrorString(status));
            return ErrorCode::UnknownError;
        }

        // 기본 설정 적용
        return ConfigureCamera();
    }

    ErrorCode CameraController::ConfigureCamera() {
        // ROI 설정
        XI_RETURN status = xiSetParamInt(m_xiHandle, XI_PRM_WIDTH, m_settings.roi.width);
        status = xiSetParamInt(m_xiHandle, XI_PRM_HEIGHT, m_settings.roi.height);
        status = xiSetParamInt(m_xiHandle, XI_PRM_OFFSET_X, m_settings.roi.offsetX);
        status = xiSetParamInt(m_xiHandle, XI_PRM_OFFSET_Y, m_settings.roi.offsetY);

        // 노출 시간 설정
        status = xiSetParamInt(m_xiHandle, XI_PRM_EXPOSURE, static_cast<int>(m_settings.exposureTime));

        // 게인 설정
        status = xiSetParamFloat(m_xiHandle, XI_PRM_GAIN, m_settings.gain);

        // 프레임레이트 제한 해제
        status = xiSetParamInt(m_xiHandle, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FREE_RUN);

        // 버퍼 정책 설정 (최신 프레임 우선)
        status = xiSetParamInt(m_xiHandle, XI_PRM_BUFFER_POLICY, XI_BP_UNSAFE);

        // 자동 대역폭 계산
        status = xiSetParamInt(m_xiHandle, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_ON);

        LOG_INFO("Camera configured successfully");
        return ErrorCode::Success;
    }

    ErrorCode CameraController::StartCapture() {
        if (!m_xiHandle) {
            return ErrorCode::CameraNotOpen;
        }

        if (m_isCapturing) {
            LOG_WARNING("Capture already running");
            return ErrorCode::Success;
        }

        {
            std::lock_guard<std::mutex> lock(m_callbackMutex);
            if (!m_frameCallback) {
                LOG_ERROR("Frame callback not set");
                return ErrorCode::CallbackNotSet;
            }
        }

        LOG_INFO("Starting capture");

        XI_RETURN status = xiStartAcquisition(m_xiHandle);
        if (status != XI_OK) {
            LOG_ERROR("Failed to start acquisition: " + GetXiErrorString(status));
            return ErrorCode::AcquisitionStartFailed;
        }

        m_shouldStop = false;
        m_isCapturing = true;

        // 캡처 스레드 시작
        m_captureThread = std::make_unique<std::thread>(&CameraController::CaptureThreadFunc, this);

        // 스레드 우선순위 설정
        SetThreadPriority(m_captureThread->native_handle(), THREAD_PRIORITY_TIME_CRITICAL);

        LOG_INFO("Capture started successfully");
        return ErrorCode::Success;
    }

    ErrorCode CameraController::StopCapture() {
        if (!m_xiHandle || !m_isCapturing) {
            return ErrorCode::Success;
        }

        LOG_INFO("Stopping capture");

        m_shouldStop = true;

        if (m_captureThread && m_captureThread->joinable()) {
            m_captureThread->join();
            m_captureThread.reset();
        }

        XI_RETURN status = xiStopAcquisition(m_xiHandle);
        if (status != XI_OK) {
            LOG_ERROR("Failed to stop acquisition: " + GetXiErrorString(status));
        }

        m_isCapturing = false;
        ResetBuffers();

        LOG_INFO("Capture stopped");
        return ErrorCode::Success;
    }

    void CameraController::CaptureThreadFunc() {
        XI_IMG xiImage;
        memset(&xiImage, 0, sizeof(xiImage));
        xiImage.size = sizeof(XI_IMG);

        auto lastStatsTime = std::chrono::steady_clock::now();
        uint64_t localFrameCount = 0;

        // CPU 코어 고정 (성능 최적화)
        SetThreadAffinityMask(GetCurrentThread(), 1);

        while (!m_shouldStop) {
            XI_RETURN status = xiGetImage(m_xiHandle, 100, &xiImage);

            if (status == XI_OK) {
                localFrameCount++;

                // 쓰기 버퍼 선택
                size_t writeIdx = m_writeIndex.load();
                FrameBuffer& writeBuffer = m_frameBuffers[writeIdx];

                // Try-lock으로 블로킹 방지
                if (writeBuffer.mutex.try_lock()) {
                    if (!writeBuffer.ready) {
                        // 프레임 데이터 복사
                        memcpy(writeBuffer.data.get(), xiImage.bp, xiImage.width * xiImage.height);

                        // 프레임 정보 설정
                        writeBuffer.info.frameNumber = xiImage.nframe;
                        writeBuffer.info.timestamp = xiImage.tsSec * 1000000ULL + xiImage.tsUSec;
                        writeBuffer.info.width = xiImage.width;
                        writeBuffer.info.height = xiImage.height;
                        writeBuffer.info.stride = xiImage.width;
                        writeBuffer.info.bytesPerPixel = 1;
                        writeBuffer.info.exposureTime = m_settings.exposureTime / 1000.0f; // ms
                        writeBuffer.info.gain = m_settings.gain;

                        writeBuffer.ready = true;
                        writeBuffer.mutex.unlock();

                        // 다음 쓰기 버퍼로 이동
                        m_writeIndex = (writeIdx + 1) % BUFFER_COUNT;

                        // 콜백 호출을 위해 읽기 시도
                        for (size_t i = 0; i < BUFFER_COUNT; ++i) {
                            size_t readIdx = (m_readIndex + i) % BUFFER_COUNT;
                            FrameBuffer& readBuffer = m_frameBuffers[readIdx];

                            if (readBuffer.mutex.try_lock()) {
                                if (readBuffer.ready) {
                                    // 콜백 호출
                                    {
                                        std::lock_guard<std::mutex> lock(m_callbackMutex);
                                        if (m_frameCallback) {
                                            m_frameCallback->OnFrameReady(readBuffer.data.get(), readBuffer.info);
                                        }
                                    }

                                    readBuffer.ready = false;
                                    m_stats.totalFrames++;
                                }
                                readBuffer.mutex.unlock();
                                break;
                            }
                        }
                    }
                    else {
                        writeBuffer.mutex.unlock();
                        m_stats.droppedFrames++;
                    }
                }
                else {
                    m_stats.droppedFrames++;
                }

                // 주기적인 로그 (1초마다)
                auto now = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastStatsTime);
                if (elapsed.count() >= 1) {
                    LOG_DEBUG("Capture thread: " + std::to_string(localFrameCount) + " frames in last second");
                    localFrameCount = 0;
                    lastStatsTime = now;
                }

            }
            else if (status == XI_TIMEOUT) {
                // 타임아웃은 정상
                continue;
            }
            else {
                // 에러 처리
                m_stats.errorCount++;
                std::string errorMsg = "Frame capture error: " + GetXiErrorString(status);
                LOG_ERROR(errorMsg);

                {
                    std::lock_guard<std::mutex> lock(m_callbackMutex);
                    if (m_frameCallback) {
                        m_frameCallback->OnError(ErrorCode::FrameCaptureFailed, errorMsg.c_str());
                    }
                }
            }

            // CPU 사용률 최적화
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        LOG_INFO("Capture thread exiting");
    }

    void CameraController::ResetBuffers() {
        for (size_t i = 0; i < BUFFER_COUNT; ++i) {
            std::lock_guard<std::mutex> lock(m_frameBuffers[i].mutex);
            m_frameBuffers[i].ready = false;
        }
        m_writeIndex = 0;
        m_readIndex = 1;
    }

    ErrorCode CameraController::SetExposureTime(float microseconds) {
        if (!m_xiHandle) return ErrorCode::CameraNotOpen;

        // 범위 검증
        if (microseconds < 10.0f) microseconds = 10.0f;
        if (microseconds > 1000000.0f) microseconds = 1000000.0f;

        XI_RETURN status = xiSetParamInt(m_xiHandle, XI_PRM_EXPOSURE, static_cast<int>(microseconds));
        if (status != XI_OK) {
            LOG_ERROR("Failed to set exposure: " + GetXiErrorString(status));
            return ErrorCode::InvalidParameter;
        }

        m_settings.exposureTime = microseconds;
        LOG_INFO("Exposure set to " + std::to_string(microseconds) + " us");
        return ErrorCode::Success;
    }

    ErrorCode CameraController::SetGain(float gainDb) {
        if (!m_xiHandle) return ErrorCode::CameraNotOpen;

        // 범위 검증
        if (gainDb < 0.0f) gainDb = 0.0f;
        if (gainDb > 24.0f) gainDb = 24.0f;

        XI_RETURN status = xiSetParamFloat(m_xiHandle, XI_PRM_GAIN, gainDb);
        if (status != XI_OK) {
            LOG_ERROR("Failed to set gain: " + GetXiErrorString(status));
            return ErrorCode::InvalidParameter;
        }

        m_settings.gain = gainDb;
        LOG_INFO("Gain set to " + std::to_string(gainDb) + " dB");
        return ErrorCode::Success;
    }

    ErrorCode CameraController::SetROI(const ROI& roi) {
        if (!m_xiHandle) return ErrorCode::CameraNotOpen;

        ErrorCode result = ValidateROI(roi);
        if (result != ErrorCode::Success) return result;

        // 캡처 중이면 중지
        bool wasCapturing = m_isCapturing;
        if (wasCapturing) {
            StopCapture();
        }

        // ROI 설정
        xiSetParamInt(m_xiHandle, XI_PRM_OFFSET_X, roi.offsetX);
        xiSetParamInt(m_xiHandle, XI_PRM_OFFSET_Y, roi.offsetY);
        xiSetParamInt(m_xiHandle, XI_PRM_WIDTH, roi.width);
        xiSetParamInt(m_xiHandle, XI_PRM_HEIGHT, roi.height);

        m_settings.roi = roi;

        // 캡처 재시작
        if (wasCapturing) {
            StartCapture();
        }

        LOG_INFO("ROI set to: " + std::to_string(roi.width) + "x" + std::to_string(roi.height) +
            " at (" + std::to_string(roi.offsetX) + "," + std::to_string(roi.offsetY) + ")");

        return ErrorCode::Success;
    }

    ErrorCode CameraController::ValidateROI(const ROI& roi) {
        // 4의 배수로 정렬 검증
        if (roi.offsetX % 4 != 0 || roi.offsetY % 4 != 0 ||
            roi.width % 4 != 0 || roi.height % 4 != 0) {
            LOG_ERROR("ROI must be aligned to 4 pixels");
            return ErrorCode::InvalidParameter;
        }

        // 최소 크기 검증
        if (roi.width < 32 || roi.height < 32) {
            LOG_ERROR("ROI too small (minimum 32x32)");
            return ErrorCode::InvalidParameter;
        }

        // 범위 검증
        if (roi.offsetX + roi.width > PYTHON1300_WIDTH ||
            roi.offsetY + roi.height > PYTHON1300_HEIGHT) {
            LOG_ERROR("ROI out of bounds");
            return ErrorCode::InvalidParameter;
        }

        return ErrorCode::Success;
    }

    void CameraController::SetFrameCallback(IFrameCallback* callback) {
        std::lock_guard<std::mutex> lock(m_callbackMutex);
        m_frameCallback = callback;
        LOG_INFO(callback ? "Frame callback set" : "Frame callback cleared");
    }

    float CameraController::GetCurrentFps() const {
        if (!m_xiHandle) return 0.0f;

        float fps = 0.0f;
        xiGetParamFloat(m_xiHandle, XI_PRM_FRAMERATE, &fps);
        return fps;
    }

    void CameraController::GetPerformanceStats(uint64_t& totalFrames, uint64_t& droppedFrames,
        uint64_t& errorCount, float& avgFps) const {
        totalFrames = m_stats.totalFrames.load();
        droppedFrames = m_stats.droppedFrames.load();
        errorCount = m_stats.errorCount.load();

        auto elapsed = std::chrono::steady_clock::now() - m_stats.startTime;
        float seconds = std::chrono::duration<float>(elapsed).count();
        avgFps = seconds > 0 ? totalFrames / seconds : 0.0f;
    }

    void CameraController::ResetPerformanceStats() {
        m_stats.totalFrames = 0;
        m_stats.droppedFrames = 0;
        m_stats.errorCount = 0;
        m_stats.startTime = std::chrono::steady_clock::now();
    }

    std::string CameraController::GetXiErrorString(XI_RETURN status) {
        switch (status) {
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
        case XI_ISOCH_ATTACH_BUFFERS: return "Isochronous attach buffers error";
        case XI_GET_OVERLAPPED_RESULT: return "Get overlapped result error";
        case XI_MEMORY_ALLOCATION: return "Memory allocation error";
        case XI_DLLCONTEXTISNULL: return "DLL context is null";
        case XI_DLLCONTEXTISNONZERO: return "DLL context is non-zero";
        case XI_DLLCONTEXTEXIST: return "DLL context exists";
        case XI_TOOMANYDEVICES: return "Too many devices";
        case XI_ERRORCAMCONTEXT: return "Camera context error";
        case XI_UNKNOWN_HARDWARE: return "Unknown hardware";
        case XI_INVALID_TM_FILE: return "Invalid TM file";
        case XI_INVALID_TM_TAG: return "Invalid TM tag";
        case XI_INCOMPLETE_TM: return "Incomplete TM";
        case XI_BUS_RESET_FAILED: return "Bus reset failed";
        case XI_NOT_IMPLEMENTED: return "Not implemented";
        case XI_SHADING_TOOBRIGHT: return "Shading too bright";
        case XI_SHADING_TOODARK: return "Shading too dark";
        case XI_TOO_LOW_GAIN: return "Gain too low";
        case XI_INVALID_BPL: return "Invalid bad pixel list";
        case XI_BPL_REALLOC: return "Bad pixel list reallocation error";
        case XI_INVALID_PIXEL_LIST: return "Invalid pixel list";
        case XI_INVALID_FFS: return "Invalid flat field shading";
        case XI_INVALID_PROFILE: return "Invalid profile";
        case XI_INVALID_CALIBRATION: return "Invalid calibration";
        case XI_INVALID_BUFFER: return "Invalid buffer";
        case XI_INVALID_DATA: return "Invalid data";
        case XI_TGBUSY: return "Timing generator busy";
        case XI_IO_WRONG: return "Wrong I/O direction";
        case XI_ACQUISITION_ALREADY_UP: return "Acquisition already started";
        case XI_OLD_DRIVER_VERSION: return "Old driver version";
        case XI_GET_LAST_ERROR: return "Get last error";
        case XI_CANT_PROCESS: return "Cannot process";
        case XI_ACQUISITION_STOPED: return "Acquisition stopped";
        case XI_ACQUISITION_STOPED_WERR: return "Acquisition stopped with error";
        case XI_INVALID_INPUT_ICC_PROFILE: return "Invalid input ICC profile";
        case XI_INVALID_OUTPUT_ICC_PROFILE: return "Invalid output ICC profile";
        case XI_DEVICE_NOT_READY: return "Device not ready";
        case XI_SHADING_TOOCONTRAST: return "Shading too contrast";
        case XI_ALREADY_INITIALIZED: return "Already initialized";
        case XI_NOT_ENOUGH_PRIVILEGES: return "Not enough privileges";
        case XI_NOT_COMPATIBLE_DRIVER: return "Not compatible driver";
        case XI_TM_INVALID_RESOURCE: return "TM invalid resource";
        case XI_DEVICE_HAS_BEEN_RESETED: return "Device has been reset";
        case XI_NO_DEVICES_FOUND: return "No devices found";
        case XI_RESOURCE_OR_FUNCTION_LOCKED: return "Resource or function locked";
        case XI_BUFFER_SIZE_TOO_SMALL: return "Buffer size too small";
        case XI_COULDNT_INIT_PROCESSOR: return "Could not initialize processor";
        case XI_NOT_INITIALIZED: return "Not initialized";
        case XI_RESOURCE_NOT_FOUND: return "Resource not found";
        case XI_UNKNOWN_PARAM: return "Unknown parameter";
        case XI_WRONG_PARAM_VALUE: return "Wrong parameter value";
        case XI_WRONG_PARAM_TYPE: return "Wrong parameter type";
        case XI_WRONG_PARAM_SIZE: return "Wrong parameter size";
        case XI_BUFFER_TOO_SMALL: return "Buffer too small";
        case XI_NOT_SUPPORTED_PARAM: return "Parameter not supported";
        case XI_NOT_SUPPORTED_PARAM_INFO: return "Parameter info not supported";
        case XI_NOT_SUPPORTED_DATA_FORMAT: return "Data format not supported";
        case XI_READ_ONLY_PARAM: return "Read-only parameter";
        case XI_BANDWIDTH_NOT_SUPPORTED: return "Bandwidth not supported";
        //case XI_INVALID_TM_ENV: return "Invalid TM environment";
        //case XI_INVALID_TM_RES: return "Invalid TM result";
        default: return "Unknown error (" + std::to_string(status) + ")";
        }
    }

    ErrorCode CameraController::GetCameraInfo(CameraInfo& info) const {
        info = m_cameraInfo;
        return ErrorCode::Success;
    }

    ErrorCode CameraController::GetCurrentSettings(CameraSettings& settings) const {
        settings = m_settings;
        return ErrorCode::Success;
    }

    ErrorCode CameraController::EnumerateCameras(std::vector<CameraInfo>& cameras) {
        cameras.clear();

        DWORD deviceCount = 0;
        XI_RETURN status = xiGetNumberDevices(&deviceCount);
        if (status != XI_OK || deviceCount == 0) {
            LOG_WARNING("No cameras found");
            return ErrorCode::CameraNotFound;
        }

        for (DWORD i = 0; i < deviceCount; ++i) {
            CameraInfo info;
            memset(&info, 0, sizeof(info));

            // 임시로 열어서 정보 읽기
            HANDLE hDevice = nullptr;
            if (xiOpenDevice(i, &hDevice) == XI_OK) {
                xiGetParamString(hDevice, XI_PRM_DEVICE_SN, info.serialNumber, sizeof(info.serialNumber));
                xiGetParamString(hDevice, XI_PRM_DEVICE_NAME, info.modelName, sizeof(info.modelName));

                int width = 0, height = 0;
                xiGetParamInt(hDevice, XI_PRM_WIDTH XI_PRM_INFO_MAX, &width);
                xiGetParamInt(hDevice, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &height);
                info.maxWidth = width;
                info.maxHeight = height;

                xiCloseDevice(hDevice);
                cameras.push_back(info);
            }
        }

        LOG_INFO("Found " + std::to_string(cameras.size()) + " cameras");
        return ErrorCode::Success;
    }

} // namespace XimeaSensor