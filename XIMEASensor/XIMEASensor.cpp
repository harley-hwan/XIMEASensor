#include "pch.h"
#include "XIMEASensor.h"
#include "CameraController.h"
#include <string>
#include <cstring>

// 전역 콜백 컨텍스트
static void* g_frameCallbackContext = nullptr;
static void* g_errorCallbackContext = nullptr;
static void* g_logCallbackContext = nullptr;

// 마지막 에러 메시지
static std::string g_lastError;

// 내부 콜백 래퍼
static void InternalFrameCallback(const FrameInfo& info)
{
    if (auto callback = reinterpret_cast<FRAME_CALLBACK>(g_frameCallbackContext)) {
        callback(info.data, info.width, info.height,
            info.frameNumber, info.timestamp, g_frameCallbackContext);
    }
}

static void InternalErrorCallback(int errorCode, const std::string& errorMsg)
{
    if (auto callback = reinterpret_cast<ERROR_CALLBACK>(g_errorCallbackContext)) {
        callback(errorCode, errorMsg.c_str(), g_errorCallbackContext);
    }
    g_lastError = errorMsg;
}

static void InternalLogCallback(LogLevel level, const std::string& message)
{
    if (auto callback = reinterpret_cast<LOG_CALLBACK>(g_logCallbackContext)) {
        callback(static_cast<int>(level), message.c_str(), g_logCallbackContext);
    }
}

// 기본 카메라 제어
bool Camera_Initialize(int deviceIndex)
{
    try {
        return CameraController::GetInstance().Initialize(deviceIndex);
    }
    catch (const std::exception& e) {
        g_lastError = e.what();
        return false;
    }
}

bool Camera_Shutdown()
{
    try {
        CameraController::GetInstance().Shutdown();
        CameraController::DestroyInstance();
        return true;
    }
    catch (const std::exception& e) {
        g_lastError = e.what();
        return false;
    }
}

bool Camera_StartCapture()
{
    try {
        return CameraController::GetInstance().StartCapture();
    }
    catch (const std::exception& e) {
        g_lastError = e.what();
        return false;
    }
}

bool Camera_StopCapture()
{
    try {
        CameraController::GetInstance().StopCapture();
        return true;
    }
    catch (const std::exception& e) {
        g_lastError = e.what();
        return false;
    }
}

int Camera_GetState()
{
    return static_cast<int>(CameraController::GetInstance().GetState());
}

// 프레임 획득
bool Camera_GetLatestFrame(unsigned char* buffer, int bufferSize,
    int* width, int* height, unsigned long* frameNumber)
{
    if (!buffer || !width || !height) {
        g_lastError = "Invalid parameters";
        return false;
    }

    uint64_t frameNum = 0;
    bool result = CameraController::GetInstance().GetLatestFrame(
        buffer, bufferSize, *width, *height, frameNum);

    if (frameNumber) {
        *frameNumber = static_cast<unsigned long>(frameNum);
    }

    return result;
}

// 콜백 설정
void Camera_SetFrameCallback(FRAME_CALLBACK callback, void* userContext)
{
    g_frameCallbackContext = userContext;

    if (callback) {
        CameraController::GetInstance().SetFrameCallback(InternalFrameCallback);
    }
    else {
        CameraController::GetInstance().SetFrameCallback(nullptr);
    }
}

void Camera_SetErrorCallback(ERROR_CALLBACK callback, void* userContext)
{
    g_errorCallbackContext = userContext;

    if (callback) {
        CameraController::GetInstance().SetErrorCallback(InternalErrorCallback);
    }
    else {
        CameraController::GetInstance().SetErrorCallback(nullptr);
    }
}

void Camera_SetLogCallback(LOG_CALLBACK callback, void* userContext)
{
    g_logCallbackContext = userContext;

    if (callback) {
        CameraController::GetInstance().SetLogCallback(InternalLogCallback);
    }
    else {
        CameraController::GetInstance().SetLogCallback(nullptr);
    }
}

// 파라미터 설정
bool Camera_SetExposure(int microsec)
{
    return CameraController::GetInstance().SetExposure(microsec);
}

bool Camera_SetGain(float gain)
{
    return CameraController::GetInstance().SetGain(gain);
}

bool Camera_SetFPS(float fps)
{
    return CameraController::GetInstance().SetFPS(fps);
}

bool Camera_SetROI(int offsetX, int offsetY, int width, int height)
{
    return CameraController::GetInstance().SetROI(offsetX, offsetY, width, height);
}

bool Camera_SetBinning(int horizontal, int vertical)
{
    return CameraController::GetInstance().SetBinning(horizontal, vertical);
}

bool Camera_SetDecimation(int horizontal, int vertical)
{
    return CameraController::GetInstance().SetDecimation(horizontal, vertical);
}

// 파라미터 읽기
int Camera_GetExposure()
{
    auto params = CameraController::GetInstance().GetCurrentParameters();
    return params.exposureUs;
}

float Camera_GetGain()
{
    auto params = CameraController::GetInstance().GetCurrentParameters();
    return params.gain;
}

float Camera_GetFPS()
{
    return CameraController::GetInstance().GetCurrentFPS();
}

bool Camera_GetROI(int* offsetX, int* offsetY, int* width, int* height)
{
    if (!width || !height) return false;

    auto params = CameraController::GetInstance().GetCurrentParameters();
    *width = params.width;
    *height = params.height;

    // TODO: offset 값도 저장/반환하도록 수정 필요
    if (offsetX) *offsetX = 0;
    if (offsetY) *offsetY = 0;

    return true;
}

// 자동 모드
bool Camera_SetAutoExposure(bool enable, float targetLevel)
{
    return CameraController::GetInstance().SetAutoExposure(enable, targetLevel);
}

bool Camera_SetAutoGain(bool enable)
{
    return CameraController::GetInstance().SetAutoGain(enable);
}

bool Camera_SetAutoWhiteBalance(bool enable)
{
    return CameraController::GetInstance().SetAutoWhiteBalance(enable);
}

// 트리거 제어
bool Camera_SetTriggerMode(int mode)
{
    return CameraController::GetInstance().SetTriggerMode(
        static_cast<TriggerMode>(mode));
}

bool Camera_SoftwareTrigger()
{
    return CameraController::GetInstance().SoftwareTrigger();
}

// 이미지 포맷
bool Camera_SetImageFormat(int format)
{
    return CameraController::GetInstance().SetImageFormat(format);
}

int Camera_GetImageFormat()
{
    // TODO: 구현 필요
    return XIMEA_FORMAT_MONO8;
}

// 카메라 정보
bool Camera_GetInfo(XIMEA_CAMERA_INFO* info)
{
    if (!info) return false;

    auto& controller = CameraController::GetInstance();

    // 장치 이름
    std::string deviceName = controller.GetDeviceName();
    strncpy_s(info->deviceName, deviceName.c_str(), sizeof(info->deviceName) - 1);

    // 시리얼 번호
    std::string serialNumber = controller.GetSerialNumber();
    strncpy_s(info->serialNumber, serialNumber.c_str(), sizeof(info->serialNumber) - 1);

    // 센서 크기
    controller.GetSensorSize(info->sensorWidth, info->sensorHeight);

    // 온도
    controller.GetTemperature(info->temperature);

    // TODO: 추가 정보 구현
    info->pixelSize = 3.45f;  // PYTHON1300 기본값
    info->maxFPS = 210;
    info->apiVersion = 4;
    info->driverVersion = 4;

    return true;
}

bool Camera_GetPerformanceStats(XIMEA_PERFORMANCE_STATS* stats)
{
    if (!stats) return false;

    auto& controller = CameraController::GetInstance();

    stats->currentFPS = controller.GetCurrentFPS();
    stats->framesCaptured = controller.GetFramesCaptured();
    stats->framesDropped = controller.GetFramesDropped();

    // TODO: 추가 통계 구현
    stats->captureTime = 0.0;
    stats->cpuUsage = 0.0f;
    stats->bandwidthMbps = stats->currentFPS * 1280 * 1024 * 8 / 1000000.0f;

    return true;
}

// 로깅
void Camera_SetLogFile(const char* filepath)
{
    if (filepath) {
        CameraController::GetInstance().SetLogFile(std::string(filepath));
    }
}

void Camera_SetLogLevel(int level)
{
    CameraController::GetInstance().SetLogLevel(static_cast<LogLevel>(level));
}

// 설정 파일
bool Camera_SaveParameters(const char* filepath)
{
    if (!filepath) return false;
    return CameraController::GetInstance().SaveParametersToFile(std::string(filepath));
}

bool Camera_LoadParameters(const char* filepath)
{
    if (!filepath) return false;
    return CameraController::GetInstance().LoadParametersFromFile(std::string(filepath));
}

// 이미지 캡처
bool Camera_CaptureImage(const char* filepath, int format)
{
    if (!filepath) return false;
    return CameraController::GetInstance().CaptureImage(std::string(filepath), format);
}

// 고급 제어
bool Camera_SetParameter(const char* name, int value)
{
    if (!name) return false;
    return CameraController::GetInstance().SetParameter(std::string(name), value);
}

bool Camera_SetParameterFloat(const char* name, float value)
{
    if (!name) return false;
    return CameraController::GetInstance().SetParameter(std::string(name), value);
}

bool Camera_GetParameter(const char* name, int* value)
{
    if (!name || !value) return false;
    return CameraController::GetInstance().GetParameter(std::string(name), *value);
}

bool Camera_GetParameterFloat(const char* name, float* value)
{
    if (!name || !value) return false;
    return CameraController::GetInstance().GetParameter(std::string(name), *value);
}

// 유틸리티
const char* Camera_GetLastError()
{
    return g_lastError.c_str();
}

int Camera_GetVersion()
{
    return 0x010000;  // 1.0.0
}