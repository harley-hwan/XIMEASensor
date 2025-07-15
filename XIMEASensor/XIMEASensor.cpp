#include "pch.h"
#include "XIMEASensor.h"
#include "CameraController.h"
#include "Logger.h"
#include "ImageSaver.h"
#include <string>
#include <cstring>
#include <iostream>

static void(*g_continuousProgressCallback)(int, double, int) = nullptr; // 2025-07-15: continuous capture

namespace CameraDefaults {
    const int EXPOSURE_US = 4000;        // 4ms default exposure
    const float GAIN_DB = 0.0f;          // 0dB default gain
    const float FRAMERATE_FPS = 60.0f;   // 60 FPS default

    // Camera limits
    const int MIN_EXPOSURE_US = 10;
    const int MAX_EXPOSURE_US = 1000000; // 1 second
    const float MIN_GAIN_DB = 0.0f;
    const float MAX_GAIN_DB = 24.0f;
    const float MIN_FPS = 1.0f;
    const float MAX_FPS = 210.0f;       // PYTHON1300 max FPS
}

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
        CameraController::Destroy();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception during CameraController::Destroy: " + std::string(e.what()));
    }
    catch (...) {
        LOG_ERROR("Unknown exception during CameraController::Destroy");
    }

    try {
        Logger::Destroy();
    }
    catch (const std::exception& e) {
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

void Camera_SetLogLevel(int level) {
    try {
        Logger::GetInstance().SetLogLevel(static_cast<LogLevel>(level));
    }
    catch (const std::exception& e) {
        // 
    }
}

void Camera_FlushLog() {
    try {
        Logger::GetInstance().Flush();
    }
    catch (const std::exception& e) {
        //
    }
}


bool Camera_SaveSnapshot(const char* filename, int format, int quality) {
    try {
        auto& controller = CameraController::GetInstance();

        int width, height;
        width = controller.GetWidth();
        height = controller.GetHeight();

        if (width <= 0 || height <= 0) {
            LOG_ERROR("Invalid image dimensions");
            return false;
        }

        size_t bufferSize = width * height;
        unsigned char* buffer = new unsigned char[bufferSize];

        bool result = controller.GetFrame(buffer, bufferSize, width, height);

        if (result) {
            result = ImageSaver::SaveGrayscaleImage(buffer, width, height,
                filename,
                static_cast<ImageFormat>(format),
                quality);
        }

        delete[] buffer;
        return result;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SaveSnapshot: " + std::string(e.what()));
        return false;
    }
}

bool Camera_SaveCurrentFrame(unsigned char* buffer, int bufferSize,
    int* width, int* height, const char* filename,
    int format, int quality) {
    try {
        if (!buffer || !width || !height || !filename) {
            LOG_ERROR("Invalid parameters");
            return false;
        }

        return ImageSaver::SaveGrayscaleImage(buffer, *width, *height, filename, static_cast<ImageFormat>(format), quality);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SaveCurrentFrame: " + std::string(e.what()));
        return false;
    }
}




bool Camera_SetContinuousCaptureConfig(double duration, int format, int quality, bool asyncSave) {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            LOG_ERROR("Continuous capture manager not available");
            return false;
        }

        ContinuousCaptureConfig config;
        config.durationSeconds = duration;
        config.imageFormat = format;
        config.jpgQuality = quality;
        config.useAsyncSave = asyncSave;
        config.createMetadata = true;
        config.baseFolder = ".";  // 현재 위치

        captureManager->SetConfig(config);
        LOG_INFO("Continuous capture config set: duration=" + std::to_string(duration) +
            "s, format=" + std::to_string(format));
        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetContinuousCaptureConfig: " + std::string(e.what()));
        return false;
    }
}

bool Camera_StartContinuousCapture() {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            LOG_ERROR("Continuous capture manager not available");
            return false;
        }

        if (CameraController::GetInstance().GetState() != CameraState::CAPTURING) {
            LOG_ERROR("Camera must be capturing to start continuous capture");
            return false;
        }

        // Reset if completed or failed
        ContinuousCaptureState currentState = captureManager->GetState();
        if (currentState == ContinuousCaptureState::COMPLETED || 
            currentState == ContinuousCaptureState::kERROR) {
            LOG_INFO("Resetting continuous capture state from " + 
                     std::to_string(static_cast<int>(currentState)));
            captureManager->Reset();
        }

        return captureManager->StartCapture();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_StartContinuousCapture: " + std::string(e.what()));
        return false;
    }
}

void Camera_StopContinuousCapture() {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (captureManager) {
            captureManager->StopCapture();
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_StopContinuousCapture: " + std::string(e.what()));
    }
}

bool Camera_IsContinuousCapturing() {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        return captureManager ? captureManager->IsCapturing() : false;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_IsContinuousCapturing: " + std::string(e.what()));
        return false;
    }
}

int Camera_GetContinuousCaptureState() {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            return 0;  // IDLE
        }
        return static_cast<int>(captureManager->GetState());
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetContinuousCaptureState: " + std::string(e.what()));
        return 0;
    }
}

bool Camera_GetContinuousCaptureResult(int* totalFrames, int* savedFrames,
    int* droppedFrames, double* duration,
    char* folderPath, int pathSize) {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            return false;
        }

        auto result = captureManager->GetResult();

        if (totalFrames) *totalFrames = result.totalFrames;
        if (savedFrames) *savedFrames = result.savedFrames;
        if (droppedFrames) *droppedFrames = result.droppedFrames;
        if (duration) *duration = result.actualDuration;

        if (folderPath && pathSize > 0) {
            strncpy_s(folderPath, pathSize, result.folderPath.c_str(), _TRUNCATE);
        }

        return result.success;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetContinuousCaptureResult: " + std::string(e.what()));
        return false;
    }
}

void Camera_SetContinuousCaptureProgressCallback(void(*callback)(int currentFrame, double elapsedSeconds, int state)) {
    try {
        g_continuousProgressCallback = callback;

        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (captureManager) {
            if (callback) {
                captureManager->SetProgressCallback(
                    [](int frame, double elapsed, ContinuousCaptureState state) {
                        if (g_continuousProgressCallback) {
                            g_continuousProgressCallback(frame, elapsed, static_cast<int>(state));
                        }
                    });
            }
            else {
                captureManager->SetProgressCallback(nullptr);
            }
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetContinuousCaptureProgressCallback: " + std::string(e.what()));
    }
}

void Camera_GetDefaultSettings(int* exposureUs, float* gainDb, float* fps) {
    try {
        if (exposureUs) *exposureUs = CameraDefaults::EXPOSURE_US;
        if (gainDb) *gainDb = CameraDefaults::GAIN_DB;
        if (fps) *fps = CameraDefaults::FRAMERATE_FPS;

        LOG_DEBUG("Default settings requested: Exposure=" +
            std::to_string(CameraDefaults::EXPOSURE_US) + "us, Gain=" +
            std::to_string(CameraDefaults::GAIN_DB) + "dB, FPS=" +
            std::to_string(CameraDefaults::FRAMERATE_FPS));
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetDefaultSettings: " + std::string(e.what()));
    }
}