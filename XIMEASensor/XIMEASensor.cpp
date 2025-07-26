#include "pch.h"
#include "XIMEASensor.h"
#include "CameraController.h"
#include "Logger.h"
#include "ImageSaver.h"
#include <string>
#include <cstring>
#include <iostream>
#include "BallDetector.h"

static void(*g_continuousProgressCallback)(int, double, int) = nullptr;

// default settings
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

    // Continuous capture defaults
    const double CONTINUOUS_CAPTURE_DURATION = 1.0;    // 1 second
    const int CONTINUOUS_CAPTURE_FORMAT = 0;           // PNG
    const int CONTINUOUS_CAPTURE_QUALITY = 90;         // 90% quality
    const bool CONTINUOUS_CAPTURE_ASYNC_SAVE = true;   // Async save

    // Single snapshot defaults
    const int SNAPSHOT_FORMAT = 0;                     // PNG
    const int SNAPSHOT_QUALITY = 90;                   // 90% quality
}

// Implementation of ContinuousCaptureConfig methods
ContinuousCaptureConfig::ContinuousCaptureConfig() {
    version = 1;  // Current version
    durationSeconds = CameraDefaults::CONTINUOUS_CAPTURE_DURATION;
    imageFormat = CameraDefaults::CONTINUOUS_CAPTURE_FORMAT;
    jpgQuality = CameraDefaults::CONTINUOUS_CAPTURE_QUALITY;
    useAsyncSave = CameraDefaults::CONTINUOUS_CAPTURE_ASYNC_SAVE;
    enableBallDetection = true;
    saveOriginalImages = true;
    saveDetectionImages = true;
    saveBallDetectorDebugImages = false;  // Default OFF for production
    createMetadata = true;

    // Safe string initialization
    strncpy_s(baseFolder, sizeof(baseFolder), ".", _TRUNCATE);
    debugImagePath[0] = '\0';
    memset(reserved, 0, sizeof(reserved));
}

ContinuousCaptureConfig ContinuousCaptureConfig::GetDefault() {
    return ContinuousCaptureConfig();
}

bool ContinuousCaptureConfig::Validate() const {
    if (version != 1) {
        return false;  // Unsupported version
    }

    if (durationSeconds <= 0 || durationSeconds > 3600) {
        return false;
    }

    if (imageFormat < 0 || imageFormat > 1) {
        return false;
    }

    if (jpgQuality < 1 || jpgQuality > 100) {
        return false;
    }

    return true;
}

void ContinuousCaptureConfig::SetBaseFolder(const char* folder) {
    if (folder) {
        strncpy_s(baseFolder, sizeof(baseFolder), folder, _TRUNCATE);
    }
    else {
        strncpy_s(baseFolder, sizeof(baseFolder), ".", _TRUNCATE);
    }
}

void ContinuousCaptureConfig::SetDebugImagePath(const char* path) {
    if (path) {
        strncpy_s(debugImagePath, sizeof(debugImagePath), path, _TRUNCATE);
    }
    else {
        debugImagePath[0] = '\0';
    }
}

// Static configuration instances
static ContinuousCaptureConfig g_continuousCaptureConfig;
static SnapshotDefaults g_snapshotDefaults = {
    CameraDefaults::SNAPSHOT_FORMAT,
    CameraDefaults::SNAPSHOT_QUALITY
};

bool Camera_Initialize(const char* logPath, int logLevel) {
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
            result = ImageSaver::SaveGrayscaleImage(buffer, width, height, filename,
                static_cast<ImageFormat>(format), quality);
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

        return ImageSaver::SaveGrayscaleImage(buffer, *width, *height, filename,
            static_cast<ImageFormat>(format), quality);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SaveCurrentFrame: " + std::string(e.what()));
        return false;
    }
}

// New unified continuous capture API implementations
bool Camera_SetContinuousCaptureConfig(const ContinuousCaptureConfig* config) {
    if (!config) {
        LOG_ERROR("Invalid parameter: config is nullptr");
        return false;
    }

    if (!config->Validate()) {
        LOG_ERROR("Invalid configuration parameters");
        return false;
    }

    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            LOG_ERROR("Continuous capture manager not available");
            return false;
        }

        // Convert to internal format
        ContinuousCaptureConfigInternal internalConfig;
        internalConfig.durationSeconds = config->durationSeconds;
        internalConfig.imageFormat = config->imageFormat;
        internalConfig.jpgQuality = config->jpgQuality;
        internalConfig.useAsyncSave = config->useAsyncSave;
        internalConfig.enableBallDetection = config->enableBallDetection;
        internalConfig.saveOriginalImages = config->saveOriginalImages;
        internalConfig.saveDetectionImages = config->saveDetectionImages;
        internalConfig.saveBallDetectorDebugImages = config->saveBallDetectorDebugImages;
        internalConfig.createMetadata = config->createMetadata;
        internalConfig.baseFolder = config->baseFolder;
        internalConfig.debugImagePath = config->debugImagePath;

        captureManager->SetConfig(internalConfig);

        // Store in global config
        g_continuousCaptureConfig = *config;

        LOG_INFO("Continuous capture config set:");
        LOG_INFO("  Duration: " + std::to_string(config->durationSeconds) + "s");
        LOG_INFO("  Format: " + std::to_string(config->imageFormat) + " (0=PNG, 1=JPG)");
        LOG_INFO("  Quality: " + std::to_string(config->jpgQuality));
        LOG_INFO("  Async Save: " + std::string(config->useAsyncSave ? "Yes" : "No"));
        LOG_INFO("  Ball Detection: " + std::string(config->enableBallDetection ? "Yes" : "No"));
        LOG_INFO("  Save Original: " + std::string(config->saveOriginalImages ? "Yes" : "No"));
        LOG_INFO("  Save Detection: " + std::string(config->saveDetectionImages ? "Yes" : "No"));
        LOG_INFO("  Save Debug: " + std::string(config->saveBallDetectorDebugImages ? "Yes" : "No"));

        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetContinuousCaptureConfig: " + std::string(e.what()));
        return false;
    }
}

static bool Camera_GetContinuousCaptureConfig(ContinuousCaptureConfig* config) {
    if (!config) {
        LOG_ERROR("Invalid parameter: config is nullptr");
        return false;
    }

    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (captureManager) {
            auto internalConfig = captureManager->GetConfig();

            config->version = 1;
            config->durationSeconds = internalConfig.durationSeconds;
            config->imageFormat = internalConfig.imageFormat;
            config->jpgQuality = internalConfig.jpgQuality;
            config->useAsyncSave = internalConfig.useAsyncSave;
            config->enableBallDetection = internalConfig.enableBallDetection;
            config->saveOriginalImages = internalConfig.saveOriginalImages;
            config->saveDetectionImages = internalConfig.saveDetectionImages;
            config->saveBallDetectorDebugImages = internalConfig.saveBallDetectorDebugImages;
            config->createMetadata = internalConfig.createMetadata;
            config->SetBaseFolder(internalConfig.baseFolder.c_str());
            config->SetDebugImagePath(internalConfig.debugImagePath.c_str());
        }
        else {
            // Return stored global config
            *config = g_continuousCaptureConfig;
        }

        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetContinuousCaptureConfig: " + std::string(e.what()));
        return false;
    }
}

bool Camera_GetDefaultContinuousCaptureConfig(ContinuousCaptureConfig* config) {
    if (!config) {
        LOG_ERROR("Invalid parameter: config is nullptr");
        return false;
    }

    *config = ContinuousCaptureConfig::GetDefault();
    LOG_DEBUG("Retrieved default continuous capture config");
    return true;
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

void Camera_GetSnapshotDefaults(SnapshotDefaults* defaults) {
    if (!defaults) {
        LOG_ERROR("Invalid parameter: defaults is nullptr");
        return;
    }

    *defaults = g_snapshotDefaults;
    LOG_DEBUG("Retrieved snapshot defaults: format=" +
        std::to_string(defaults->format) + ", quality=" +
        std::to_string(defaults->quality));
}

void Camera_SetSnapshotDefaults(const SnapshotDefaults* defaults) {
    if (!defaults) {
        LOG_ERROR("Invalid parameter: defaults is nullptr");
        return;
    }

    if (defaults->format < 0 || defaults->format > 1) {
        LOG_ERROR("Invalid format: must be 0 (PNG) or 1 (JPG)");
        return;
    }

    if (defaults->quality < 1 || defaults->quality > 100) {
        LOG_ERROR("Invalid quality: must be between 1 and 100");
        return;
    }

    g_snapshotDefaults = *defaults;
    LOG_INFO("Updated snapshot defaults: format=" +
        std::to_string(defaults->format) + ", quality=" +
        std::to_string(defaults->quality));
}

bool Camera_SaveSnapshotWithDefaults(const char* filename) {
    try {
        if (!filename) {
            LOG_ERROR("Invalid parameter: filename is nullptr");
            return false;
        }

        return Camera_SaveSnapshot(filename,
            g_snapshotDefaults.format,
            g_snapshotDefaults.quality);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SaveSnapshotWithDefaults: " +
            std::string(e.what()));
        return false;
    }
}

bool Camera_GetContinuousCaptureDetectionResult(int* framesWithBalls, int* totalBallsDetected,
    float* averageConfidence, char* detectionFolder, int folderSize) {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            LOG_ERROR("Continuous capture manager not available");
            return false;
        }

        auto result = captureManager->GetDetectionResult();

        if (framesWithBalls) *framesWithBalls = result.framesWithBall;
        if (totalBallsDetected) *totalBallsDetected = result.totalBallsDetected;
        if (averageConfidence) *averageConfidence = result.averageConfidence;

        if (detectionFolder && folderSize > 0) {
            strncpy_s(detectionFolder, folderSize, result.detectionFolder.c_str(), _TRUNCATE);
        }

        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetContinuousCaptureDetectionResult: " + std::string(e.what()));
        return false;
    }
}

bool Camera_SetBallDetectorDebugImages(bool enable) {
    try {
#ifdef ENABLE_DEBUG_IMAGE_SAVING
        g_continuousCaptureConfig.saveBallDetectorDebugImages = enable;

        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (captureManager) {
            if (!captureManager->IsCapturing()) {
                auto config = captureManager->GetConfig();
                config.saveBallDetectorDebugImages = enable;
                captureManager->SetConfig(config);
            }

            auto* ballDetector = captureManager->GetBallDetector();
            if (ballDetector) {
                auto params = ballDetector->GetParameters();
                params.saveIntermediateImages = enable;
                ballDetector->SetParameters(params);

                LOG_INFO("Ball detector debug images globally " +
                    std::string(enable ? "enabled" : "disabled"));
            }
        }

        LOG_INFO("Ball detector debug images default set to: " +
            std::string(enable ? "enabled" : "disabled"));
        return true;
#else
        // Debug image saving is disabled at compile time
        if (enable) {
            LOG_WARNING("Ball detector debug images requested but ENABLE_DEBUG_IMAGE_SAVING is not defined");
            LOG_INFO("To enable debug image saving, define ENABLE_DEBUG_IMAGE_SAVING in BallDetector.h and recompile");
        }
        else {
            LOG_INFO("Ball detector debug images disabled (compile-time disabled)");
        }

        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (captureManager && !captureManager->IsCapturing()) {
            auto config = captureManager->GetConfig();
            config.saveBallDetectorDebugImages = true;
            captureManager->SetConfig(config);
        }

        return true;
#endif
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetBallDetectorDebugImages: " + std::string(e.what()));
        return false;
    }
}

bool Camera_GetBallDetectorDebugImages() {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (captureManager) {
            auto config = captureManager->GetConfig();
            return config.saveBallDetectorDebugImages;
        }

        return g_continuousCaptureConfig.saveBallDetectorDebugImages;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetBallDetectorDebugImages: " + std::string(e.what()));
        return false;
    }
}