#include "pch.h"
#include "XIMEASensor.h"
#include "XIMEASensorCommon.h"
#include "CameraController.h"
#include "NetworkServer.h"
#include "Logger.h"
#include <memory>
#include <string>

using namespace XimeaSensor;

// 전역 변수
static std::unique_ptr<NetworkServer> g_networkServer;
static IFrameCallback* g_frameCallbackWrapper = nullptr;
static void* g_frameCallbackUserData = nullptr;
static XSensorFrameCallback g_frameCallback = nullptr;
static XSensorErrorCallback g_errorCallback = nullptr;
static XSensorDisconnectCallback g_disconnectCallback = nullptr;

// 버전 정보
static const char* g_version = "2.0.0";

// 콜백 래퍼 클래스
class CallbackWrapper : public IFrameCallback, public ILogCallback {
public:
    void OnFrameReady(const uint8_t* frameData, const FrameInfo& info) override {
        if (g_frameCallback) {
            XSensorFrameInfo cInfo;
            cInfo.frameNumber = info.frameNumber;
            cInfo.timestamp = info.timestamp;
            cInfo.width = info.width;
            cInfo.height = info.height;
            cInfo.stride = info.stride;
            cInfo.bytesPerPixel = info.bytesPerPixel;
            cInfo.exposureTime = info.exposureTime;
            cInfo.gain = info.gain;

            g_frameCallback(frameData, &cInfo, g_frameCallbackUserData);
        }
    }

    void OnError(ErrorCode error, const char* message) override {
        if (g_errorCallback) {
            g_errorCallback(static_cast<XSensorError>(error), message, g_frameCallbackUserData);
        }
    }

    void OnCameraDisconnected() override {
        if (g_disconnectCallback) {
            g_disconnectCallback(g_frameCallbackUserData);
        }
    }

    void OnLog(LogLevel level, const char* message, const char* file, int line) override {
        // 로그 콜백 구현
    }
};

static CallbackWrapper g_callbackWrapper;

// ============================================================================
// 시스템 초기화/종료
// ============================================================================
XSensorError XSensor_Initialize(const char* logFilePath, XSensorLogLevel minLogLevel) {
    try {
        Logger::GetInstance().Initialize(
            logFilePath ? logFilePath : "",
            static_cast<LogLevel>(minLogLevel)
        );

        LOG_INFO("XIMEASensor initialized, version " + std::string(g_version));
        return XSENSOR_SUCCESS;
    }
    catch (...) {
        return XSENSOR_ERROR_UNKNOWN;
    }
}

void XSensor_Shutdown() {
    LOG_INFO("XIMEASensor shutting down");

    // 네트워크 서버 정지
    if (g_networkServer) {
        g_networkServer->Stop();
        g_networkServer.reset();
    }

    // 카메라 정지
    CameraController::GetInstance().StopCapture();
    CameraController::GetInstance().CloseCamera();
}

const char* XSensor_GetVersion() {
    return g_version;
}

const char* XSensor_GetErrorString(XSensorError error) {
    switch (error) {
    case XSENSOR_SUCCESS: return "Success";
    case XSENSOR_ERROR_CAMERA_NOT_FOUND: return "Camera not found";
    case XSENSOR_ERROR_CAMERA_OPEN_FAILED: return "Failed to open camera";
    case XSENSOR_ERROR_CAMERA_ALREADY_OPEN: return "Camera already open";
    case XSENSOR_ERROR_CAMERA_NOT_OPEN: return "Camera not open";
    case XSENSOR_ERROR_ACQUISITION_START_FAILED: return "Failed to start acquisition";
    case XSENSOR_ERROR_ACQUISITION_STOP_FAILED: return "Failed to stop acquisition";
    case XSENSOR_ERROR_FRAME_CAPTURE_FAILED: return "Failed to capture frame";
    case XSENSOR_ERROR_INVALID_PARAMETER: return "Invalid parameter";
    case XSENSOR_ERROR_BUFFER_TOO_SMALL: return "Buffer too small";
    case XSENSOR_ERROR_MEMORY_ALLOCATION_FAILED: return "Memory allocation failed";
    case XSENSOR_ERROR_THREAD_CREATION_FAILED: return "Thread creation failed";
    case XSENSOR_ERROR_CALLBACK_NOT_SET: return "Callback not set";
    case XSENSOR_ERROR_NETWORK_ERROR: return "Network error";
    case XSENSOR_ERROR_UNKNOWN: return "Unknown error";
    default: return "Undefined error";
    }
}

// ============================================================================
// 로깅 시스템
// ============================================================================
void XSensor_SetLogLevel(XSensorLogLevel level) {
    Logger::GetInstance().SetLogLevel(static_cast<LogLevel>(level));
}

void XSensor_SetLogCallback(XSensorLogCallback callback, void* userData) {
    // TODO: 구현
}

void XSensor_EnableConsoleLog(int enable) {
    Logger::GetInstance().EnableConsoleOutput(enable != 0);
}

void XSensor_EnableFileLog(int enable) {
    Logger::GetInstance().EnableFileOutput(enable != 0);
}

// ============================================================================
// 카메라 제어
// ============================================================================
XSensorError XSensor_OpenCamera(int deviceIndex) {
    ErrorCode result = CameraController::GetInstance().OpenCamera(deviceIndex);
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_CloseCamera() {
    ErrorCode result = CameraController::GetInstance().CloseCamera();
    return static_cast<XSensorError>(result);
}

int XSensor_IsCameraOpen() {
    return CameraController::GetInstance().IsOpen() ? 1 : 0;
}

XSensorError XSensor_StartCapture() {
    // 콜백 설정
    if (!g_frameCallbackWrapper) {
        g_frameCallbackWrapper = &g_callbackWrapper;
        CameraController::GetInstance().SetFrameCallback(g_frameCallbackWrapper);
    }

    ErrorCode result = CameraController::GetInstance().StartCapture();
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_StopCapture() {
    ErrorCode result = CameraController::GetInstance().StopCapture();
    return static_cast<XSensorError>(result);
}

int XSensor_IsCapturing() {
    return CameraController::GetInstance().IsCapturing() ? 1 : 0;
}

// ============================================================================
// 카메라 설정
// ============================================================================
XSensorError XSensor_SetExposureTime(float microseconds) {
    ErrorCode result = CameraController::GetInstance().SetExposureTime(microseconds);
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_SetGain(float gainDb) {
    ErrorCode result = CameraController::GetInstance().SetGain(gainDb);
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_SetROI(const XSensorROI* roi) {
    if (!roi) return XSENSOR_ERROR_INVALID_PARAMETER;

    ROI internalRoi;
    internalRoi.offsetX = roi->offsetX;
    internalRoi.offsetY = roi->offsetY;
    internalRoi.width = roi->width;
    internalRoi.height = roi->height;

    ErrorCode result = CameraController::GetInstance().SetROI(internalRoi);
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_SetTargetFps(uint32_t fps) {
    ErrorCode result = CameraController::GetInstance().SetTargetFps(fps);
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_SetAutoExposure(int enable) {
    ErrorCode result = CameraController::GetInstance().SetAutoExposure(enable != 0);
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_SetAutoGain(int enable) {
    ErrorCode result = CameraController::GetInstance().SetAutoGain(enable != 0);
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_ApplySettings(const XSensorCameraSettings* settings) {
    if (!settings) return XSENSOR_ERROR_INVALID_PARAMETER;

    CameraSettings internalSettings;
    internalSettings.exposureTime = settings->exposureTime;
    internalSettings.gain = settings->gain;
    internalSettings.targetFps = settings->targetFps;
    internalSettings.roi.offsetX = settings->roi.offsetX;
    internalSettings.roi.offsetY = settings->roi.offsetY;
    internalSettings.roi.width = settings->roi.width;
    internalSettings.roi.height = settings->roi.height;
    internalSettings.autoExposure = settings->autoExposure != 0;
    internalSettings.autoGain = settings->autoGain != 0;

    ErrorCode result = CameraController::GetInstance().ApplySettings(internalSettings);
    return static_cast<XSensorError>(result);
}

// ============================================================================
// 카메라 조회
// ============================================================================
XSensorError XSensor_GetCameraInfo(XSensorCameraInfo* info) {
    if (!info) return XSENSOR_ERROR_INVALID_PARAMETER;

    CameraInfo internalInfo;
    ErrorCode result = CameraController::GetInstance().GetCameraInfo(internalInfo);

    if (result == ErrorCode::Success) {
        strcpy_s(info->serialNumber, sizeof(info->serialNumber), internalInfo.serialNumber);
        strcpy_s(info->modelName, sizeof(info->modelName), internalInfo.modelName);
        info->maxWidth = internalInfo.maxWidth;
        info->maxHeight = internalInfo.maxHeight;
        info->maxFps = internalInfo.maxFps;
        info->isColorCamera = internalInfo.isColorCamera ? 1 : 0;
    }

    return static_cast<XSensorError>(result);
}

XSensorError XSensor_GetCurrentSettings(XSensorCameraSettings* settings) {
    if (!settings) return XSENSOR_ERROR_INVALID_PARAMETER;

    CameraSettings internalSettings;
    ErrorCode result = CameraController::GetInstance().GetCurrentSettings(internalSettings);

    if (result == ErrorCode::Success) {
        settings->exposureTime = internalSettings.exposureTime;
        settings->gain = internalSettings.gain;
        settings->targetFps = internalSettings.targetFps;
        settings->roi.offsetX = internalSettings.roi.offsetX;
        settings->roi.offsetY = internalSettings.roi.offsetY;
        settings->roi.width = internalSettings.roi.width;
        settings->roi.height = internalSettings.roi.height;
        settings->autoExposure = internalSettings.autoExposure ? 1 : 0;
        settings->autoGain = internalSettings.autoGain ? 1 : 0;
    }

    return static_cast<XSensorError>(result);
}

XSensorError XSensor_GetExposureRange(float* minUs, float* maxUs) {
    if (!minUs || !maxUs) return XSENSOR_ERROR_INVALID_PARAMETER;

    ErrorCode result = CameraController::GetInstance().GetExposureRange(*minUs, *maxUs);
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_GetGainRange(float* minDb, float* maxDb) {
    if (!minDb || !maxDb) return XSENSOR_ERROR_INVALID_PARAMETER;

    ErrorCode result = CameraController::GetInstance().GetGainRange(*minDb, *maxDb);
    return static_cast<XSensorError>(result);
}

float XSensor_GetCurrentFps() {
    return CameraController::GetInstance().GetCurrentFps();
}

XSensorError XSensor_GetPerformanceStats(XSensorPerformanceStats* stats) {
    if (!stats) return XSENSOR_ERROR_INVALID_PARAMETER;

    CameraController::GetInstance().GetPerformanceStats(
        stats->totalFrames, stats->droppedFrames,
        stats->errorCount, stats->averageFps
    );

    return XSENSOR_SUCCESS;
}

void XSensor_ResetPerformanceStats() {
    CameraController::GetInstance().ResetPerformanceStats();
}

// ============================================================================
// 콜백 설정
// ============================================================================
void XSensor_SetFrameCallback(XSensorFrameCallback callback, void* userData) {
    g_frameCallback = callback;
    g_frameCallbackUserData = userData;
}

void XSensor_SetErrorCallback(XSensorErrorCallback callback, void* userData) {
    g_errorCallback = callback;
}

void XSensor_SetDisconnectCallback(XSensorDisconnectCallback callback, void* userData) {
    g_disconnectCallback = callback;
}

// ============================================================================
// 동기식 캡처
// ============================================================================
XSensorError XSensor_CaptureSnapshot(uint8_t* buffer, size_t bufferSize,
    XSensorFrameInfo* info, uint32_t timeoutMs) {
    if (!buffer || !info) return XSENSOR_ERROR_INVALID_PARAMETER;

    FrameInfo internalInfo;
    ErrorCode result = CameraController::GetInstance().CaptureSnapshot(
        buffer, bufferSize, internalInfo, timeoutMs
    );

    if (result == ErrorCode::Success) {
        info->frameNumber = internalInfo.frameNumber;
        info->timestamp = internalInfo.timestamp;
        info->width = internalInfo.width;
        info->height = internalInfo.height;
        info->stride = internalInfo.stride;
        info->bytesPerPixel = internalInfo.bytesPerPixel;
        info->exposureTime = internalInfo.exposureTime;
        info->gain = internalInfo.gain;
    }

    return static_cast<XSensorError>(result);
}

// ============================================================================
// 카메라 열거
// ============================================================================
XSensorError XSensor_EnumerateCameras(XSensorCameraInfo* cameras, int* count, int maxCount) {
    if (!cameras || !count || maxCount <= 0) return XSENSOR_ERROR_INVALID_PARAMETER;

    std::vector<CameraInfo> internalCameras;
    ErrorCode result = CameraController::EnumerateCameras(internalCameras);

    if (result == ErrorCode::Success) {
        *count = static_cast<int>(std::min(internalCameras.size(), static_cast<size_t>(maxCount)));

        for (int i = 0; i < *count; ++i) {
            strcpy_s(cameras[i].serialNumber, sizeof(cameras[i].serialNumber),
                internalCameras[i].serialNumber);
            strcpy_s(cameras[i].modelName, sizeof(cameras[i].modelName),
                internalCameras[i].modelName);
            cameras[i].maxWidth = internalCameras[i].maxWidth;
            cameras[i].maxHeight = internalCameras[i].maxHeight;
            cameras[i].maxFps = internalCameras[i].maxFps;
            cameras[i].isColorCamera = internalCameras[i].isColorCamera ? 1 : 0;
        }
    }

    return static_cast<XSensorError>(result);
}

// ============================================================================
// 네트워크 서버
// ============================================================================
XSensorError XSensor_StartNetworkServer(uint16_t port) {
    if (!g_networkServer) {
        g_networkServer = std::make_unique<NetworkServer>();
    }

    ErrorCode result = g_networkServer->Start(port);
    return static_cast<XSensorError>(result);
}

XSensorError XSensor_StopNetworkServer() {
    if (!g_networkServer) return XSENSOR_SUCCESS;

    ErrorCode result = g_networkServer->Stop();
    return static_cast<XSensorError>(result);
}

int XSensor_IsNetworkServerRunning() {
    return (g_networkServer && g_networkServer->IsRunning()) ? 1 : 0;
}

int XSensor_GetNetworkClientCount() {
    if (!g_networkServer) return 0;
    return static_cast<int>(g_networkServer->GetClientCount());
}

void XSensor_SetFrameCompression(int enable, int jpegQuality) {
    if (g_networkServer) {
        g_networkServer->SetFrameCompression(enable != 0, jpegQuality);
    }
}

// ============================================================================
// 레거시 호환성
// ============================================================================

// 레거시 프레임 버퍼 (단일 버퍼)
static struct {
    std::unique_ptr<uint8_t[]> buffer;
    int width;
    int height;
    std::mutex mutex;
    bool ready;
} g_legacyFrame;

// 레거시 콜백 래퍼
class LegacyCallbackWrapper : public IFrameCallback {
public:
    void OnFrameReady(const uint8_t* frameData, const FrameInfo& info) override {
        std::lock_guard<std::mutex> lock(g_legacyFrame.mutex);

        size_t frameSize = info.width * info.height * info.bytesPerPixel;
        if (!g_legacyFrame.buffer || frameSize > 1280 * 1024) {
            g_legacyFrame.buffer = std::make_unique<uint8_t[]>(frameSize);
        }

        memcpy(g_legacyFrame.buffer.get(), frameData, frameSize);
        g_legacyFrame.width = info.width;
        g_legacyFrame.height = info.height;
        g_legacyFrame.ready = true;
    }

    void OnError(ErrorCode error, const char* message) override {
        LOG_ERROR("Legacy callback error: " + std::string(message));
    }

    void OnCameraDisconnected() override {
        LOG_WARNING("Camera disconnected (legacy callback)");
    }
};

static LegacyCallbackWrapper g_legacyCallbackWrapper;

int Camera_Open(int deviceIndex) {
    // 로깅 시스템 초기화 (레거시 호환성)
    static bool initialized = false;
    if (!initialized) {
        XSensor_Initialize("", XSENSOR_LOG_INFO);
        initialized = true;
    }

    XSensorError result = XSensor_OpenCamera(deviceIndex);
    return (result == XSENSOR_SUCCESS) ? 1 : 0;
}

void Camera_Close() {
    XSensor_CloseCamera();
}

int Camera_Start() {
    // 레거시 콜백 설정
    CameraController::GetInstance().SetFrameCallback(&g_legacyCallbackWrapper);

    XSensorError result = XSensor_StartCapture();
    return (result == XSENSOR_SUCCESS) ? 1 : 0;
}

void Camera_Stop() {
    XSensor_StopCapture();
}

int Camera_GetFrame(unsigned char* buffer, int bufferSize, int* width, int* height) {
    if (!buffer || !width || !height) return 0;

    std::lock_guard<std::mutex> lock(g_legacyFrame.mutex);

    if (!g_legacyFrame.ready) return 0;

    int frameSize = g_legacyFrame.width * g_legacyFrame.height;
    if (bufferSize < frameSize) return 0;

    memcpy(buffer, g_legacyFrame.buffer.get(), frameSize);
    *width = g_legacyFrame.width;
    *height = g_legacyFrame.height;

    g_legacyFrame.ready = false;
    return 1;
}

int Camera_SetExposure(int microsec) {
    XSensorError result = XSensor_SetExposureTime(static_cast<float>(microsec));
    return (result == XSENSOR_SUCCESS) ? 1 : 0;
}

int Camera_SetROI(int offsetX, int offsetY, int width, int height) {
    XSensorROI roi;
    roi.offsetX = offsetX;
    roi.offsetY = offsetY;
    roi.width = width;
    roi.height = height;

    XSensorError result = XSensor_SetROI(&roi);
    return (result == XSENSOR_SUCCESS) ? 1 : 0;
}

int Camera_SetGain(float gain) {
    XSensorError result = XSensor_SetGain(gain);
    return (result == XSENSOR_SUCCESS) ? 1 : 0;
}