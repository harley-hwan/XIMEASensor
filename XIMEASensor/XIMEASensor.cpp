#include "pch.h"
#include "XIMEASensor.h"
#include "CameraController.h"
#include "Logger.h"
#include <string>
#include <cstring>
#include <iostream>


bool Camera_Initialize(const char* logPath, int logLevel) {
    std::cout << "in XIMEASensor initialize" << std::endl;
    try {
        std::string path = logPath ? logPath : "XIMEASensor.log";
        Logger::GetInstance().Initialize(path, static_cast<LogLevel>(logLevel));
        LOG_INFO("XIMEASensor DLL initialized");
        return true;
    }
    catch (const std::exception& e) {
        return false;
    }
}

void Camera_Shutdown() {
    LOG_INFO("XIMEASensor DLL shutting down");

    try {
        // CameraController 인스턴스가 존재하는지 먼저 확인
        CameraController::Destroy();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception during CameraController::Destroy: " + std::string(e.what()));
    }
    catch (...) {
        LOG_ERROR("Unknown exception during CameraController::Destroy");
    }

    try {
        // Logger 종료
        Logger::Destroy();
    }
    catch (const std::exception& e) {
        // 로그 시스템이 이미 종료되었을 수 있으므로 stderr로 출력
        std::cerr << "Exception during Logger::Destroy: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown exception during Logger::Destroy" << std::endl;
    }
}


int Camera_GetDeviceCount() {
    try {
        return CameraController::GetInstance().GetConnectedDeviceCount();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetDeviceCount: " + std::string(e.what()));
        return 0;
    }
}

bool Camera_GetDeviceInfo(int index, char* name, int nameSize, char* serial, int serialSize) {
    try {
        std::string deviceName, deviceSerial;
        if (!CameraController::GetInstance().GetDeviceInfo(index, deviceName, deviceSerial)) {
            return false;
        }

        if (name && nameSize > 0) {
            strncpy_s(name, nameSize, deviceName.c_str(), _TRUNCATE);
        }

        if (serial && serialSize > 0) {
            strncpy_s(serial, serialSize, deviceSerial.c_str(), _TRUNCATE);
        }

        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetDeviceInfo: " + std::string(e.what()));
        return false;
    }
}

// 카메라 제어
bool Camera_Open(int deviceIndex) {
    try {
        return CameraController::GetInstance().OpenCamera(deviceIndex);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_Open: " + std::string(e.what()));
        return false;
    }
}

void Camera_Close() {
    try {
        CameraController::GetInstance().CloseCamera();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_Close: " + std::string(e.what()));
    }
}

bool Camera_Start() {
    try {
        return CameraController::GetInstance().StartCapture();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_Start: " + std::string(e.what()));
        return false;
    }
}

void Camera_Stop() {
    try {
        CameraController::GetInstance().StopCapture();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_Stop: " + std::string(e.what()));
    }
}

bool Camera_Pause(bool pause) {
    try {
        CameraController::GetInstance().PauseCapture(pause);
        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_Pause: " + std::string(e.what()));
        return false;
    }
}

// 프레임 획득
bool Camera_GetFrame(unsigned char* buffer, int bufferSize, int* width, int* height) {
    try {
        int w, h;
        bool result = CameraController::GetInstance().GetFrame(buffer, bufferSize, w, h);

        if (result) {
            if (width) *width = w;
            if (height) *height = h;
        }

        return result;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetFrame: " + std::string(e.what()));
        return false;
    }
}

// 카메라 설정
bool Camera_SetExposure(int microsec) {
    try {
        return CameraController::GetInstance().SetExposure(microsec);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetExposure: " + std::string(e.what()));
        return false;
    }
}

bool Camera_SetGain(float gain) {
    try {
        return CameraController::GetInstance().SetGain(gain);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetGain: " + std::string(e.what()));
        return false;
    }
}

bool Camera_SetROI(int offsetX, int offsetY, int width, int height) {
    try {
        return CameraController::GetInstance().SetROI(offsetX, offsetY, width, height);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetROI: " + std::string(e.what()));
        return false;
    }
}

bool Camera_SetFrameRate(float fps) {
    try {
        return CameraController::GetInstance().SetFrameRate(fps);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetFrameRate: " + std::string(e.what()));
        return false;
    }
}

bool Camera_SetTriggerMode(bool enabled) {
    try {
        return CameraController::GetInstance().SetTriggerMode(enabled);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetTriggerMode: " + std::string(e.what()));
        return false;
    }
}

// 카메라 속성 조회
int Camera_GetExposure() {
    try {
        return CameraController::GetInstance().GetExposure();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetExposure: " + std::string(e.what()));
        return 0;
    }
}

float Camera_GetGain() {
    try {
        return CameraController::GetInstance().GetGain();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetGain: " + std::string(e.what()));
        return 0.0f;
    }
}

bool Camera_GetROI(int* offsetX, int* offsetY, int* width, int* height) {
    try {
        auto& controller = CameraController::GetInstance();

        // 현재는 offset을 저장하지 않으므로 0으로 반환
        if (offsetX) *offsetX = 0;
        if (offsetY) *offsetY = 0;
        if (width) *width = controller.GetWidth();
        if (height) *height = controller.GetHeight();

        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetROI: " + std::string(e.what()));
        return false;
    }
}

float Camera_GetFrameRate() {
    try {
        return CameraController::GetInstance().GetFrameRate();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetFrameRate: " + std::string(e.what()));
        return 0.0f;
    }
}

int Camera_GetState() {
    try {
        return static_cast<int>(CameraController::GetInstance().GetState());
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetState: " + std::string(e.what()));
        return 0;
    }
}

// 통계 정보
bool Camera_GetStatistics(unsigned long* totalFrames, unsigned long* droppedFrames,
    double* averageFPS, double* minFPS, double* maxFPS) {
    try {
        auto stats = CameraController::GetInstance().GetStatistics();

        if (totalFrames) *totalFrames = stats.totalFrames;
        if (droppedFrames) *droppedFrames = stats.droppedFrames;
        if (averageFPS) *averageFPS = stats.averageFPS;
        if (minFPS) *minFPS = stats.minFPS;
        if (maxFPS) *maxFPS = stats.maxFPS;

        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetStatistics: " + std::string(e.what()));
        return false;
    }
}

void Camera_ResetStatistics() {
    try {
        CameraController::GetInstance().ResetStatistics();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_ResetStatistics: " + std::string(e.what()));
    }
}

// 콜백 관리
bool Camera_RegisterCallback(IXIMEACallback* callback) {
    try {
        CameraController::GetInstance().RegisterCallback(callback);
        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_RegisterCallback: " + std::string(e.what()));
        return false;
    }
}

bool Camera_UnregisterCallback(IXIMEACallback* callback) {
    try {
        CameraController::GetInstance().UnregisterCallback(callback);
        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_UnregisterCallback: " + std::string(e.what()));
        return false;
    }
}

void Camera_ClearCallbacks() {
    try {
        CameraController::GetInstance().ClearCallbacks();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_ClearCallbacks: " + std::string(e.what()));
    }
}

// 로그 설정
void Camera_SetLogLevel(int level) {
    try {
        Logger::GetInstance().SetLogLevel(static_cast<LogLevel>(level));
    }
    catch (const std::exception& e) {
        // 로그 시스템 자체의 에러는 출력하지 않음
    }
}

void Camera_FlushLog() {
    try {
        Logger::GetInstance().Flush();
    }
    catch (const std::exception& e) {
        // 로그 시스템 자체의 에러는 출력하지 않음
    }
}