#include "pch.h"
#include "XIMEASensor.h"
#include "CameraController.h"
#include "Logger.h"
#include "ImageSaver.h"
#include <string>
#include <cstring>
#include <iostream>
#include "BallDetector.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

#ifdef ENABLE_CONTINUOUS_CAPTURE
// Global callback for continuous capture progress updates
static void(*g_continuousProgressCallback)(int, double, int) = nullptr;
#endif

// Global callback for real-time ball detection
static RealtimeDetectionCallback g_realtimeDetectionCallback = nullptr;
static void* g_realtimeDetectionContext = nullptr;

// ============================================================================
// NAMESPACE: Camera Default Settings
// ============================================================================

namespace CameraDefaults {
    // Default operational parameters
    const int EXPOSURE_US = 4000;        // 4ms default exposure time
    const float GAIN_DB = 0.0f;          // 0dB default gain (no amplification)
    const float FRAMERATE_FPS = 60.0f;   // 60 FPS default frame rate

    // Hardware limits for PYTHON1300 sensor
    const int MIN_EXPOSURE_US = 10;      // 10 microseconds minimum
    const int MAX_EXPOSURE_US = 1000000; // 1 second maximum
    const float MIN_GAIN_DB = 0.0f;      // No negative gain
    const float MAX_GAIN_DB = 24.0f;     // Maximum 24dB gain
    const float MIN_FPS = 1.0f;          // 1 FPS minimum
    const float MAX_FPS = 210.0f;        // PYTHON1300 maximum frame rate

#ifdef ENABLE_CONTINUOUS_CAPTURE
    // Create default continuous capture configuration
    static ContinuousCaptureConfig CreateDefaultConfig() {
        return ContinuousCaptureConfig(
            1.0,    // 1 second capture duration
            0,      // PNG format
            90,     // 90% JPEG quality
            true,   // Create metadata file
            true,   // Use async saving for performance
            ".",    // Current directory as base
            true,   // Enable ball detection
            true,   // Save original images
            true,   // Save detection overlay images
            true,   // Save debug images
            ""      // No custom debug path
        );
    }
#endif

    // Snapshot default settings
    const int SNAPSHOT_FORMAT = 0;      // PNG format
    const int SNAPSHOT_QUALITY = 90;    // 90% quality for JPEG
}

// ============================================================================
// STATIC VARIABLES
// ============================================================================

#ifdef ENABLE_CONTINUOUS_CAPTURE
// Global default configuration for continuous capture
static ContinuousCaptureConfig g_continuousCaptureConfig = CameraDefaults::CreateDefaultConfig();
#endif

// Global default settings for snapshots
static SnapshotDefaults g_snapshotDefaults = {
    CameraDefaults::SNAPSHOT_FORMAT,
    CameraDefaults::SNAPSHOT_QUALITY
};

// ============================================================================
// SYSTEM INITIALIZATION FUNCTIONS
// ============================================================================

// Initialize camera system and logging
bool Camera_Initialize(const char* logPath, int logLevel) {
    try {
        // Use default log path if none provided
        std::string path = logPath ? logPath : "XIMEASensor.log";

        // Init logger
        Logger::GetInstance().Initialize(path, static_cast<LogLevel>(logLevel));
        LOG_INFO("XIMEASensor DLL initialized");

        return true;
    }
    catch (const std::exception& e) {
        // Silent failure - can't log if logger failed to initialize
        LOG_ERROR("Exception in Camera_Initialize: " + std::string(e.what()));
        return false;
    }
}

// Shutdown camera system
void Camera_Shutdown() {
    LOG_INFO("XIMEASensor DLL shutting down");

    // Destroy camera controller first
    try {
        CameraController::Destroy();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception during CameraController::Destroy: " + std::string(e.what()));
    }
    catch (...) {
        LOG_ERROR("Unknown exception during CameraController::Destroy");
    }

    // Then destroy logger
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

// ============================================================================
// DEVICE MANAGEMENT FUNCTIONS
// ============================================================================

// Get number of connected XIMEA cameras
int Camera_GetDeviceCount() {
    try {
        return CameraController::GetInstance().GetConnectedDeviceCount();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetDeviceCount: " + std::string(e.what()));
        return 0;
    }
}

// Get device information
bool Camera_GetDeviceInfo(int index, char* name, int nameSize, char* serial, int serialSize) {
    try {
        std::string deviceName, deviceSerial;

        // Get device info from controller
        if (!CameraController::GetInstance().GetDeviceInfo(index, deviceName, deviceSerial)) {
            return false;
        }

        // Copy name if buffer provided
        if (name && nameSize > 0) {
            strncpy_s(name, nameSize, deviceName.c_str(), _TRUNCATE);
        }

        // Copy serial if buffer provided
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

// ============================================================================
// CAMERA CONTROL FUNCTIONS
// ============================================================================

// Open a camera device by index
bool Camera_Open(int deviceIndex) {
    try {
        return CameraController::GetInstance().OpenCamera(deviceIndex);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_Open: " + std::string(e.what()));
        return false;
    }
}

// Close the currently open camera
void Camera_Close() {
    try {
        CameraController::GetInstance().CloseCamera();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_Close: " + std::string(e.what()));
    }
}

// Start image acquisition
bool Camera_Start() {
    try {
        return CameraController::GetInstance().StartCapture();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_Start: " + std::string(e.what()));
        return false;
    }
}

// Stop image acquisition
void Camera_Stop() {
    try {
        CameraController::GetInstance().StopCapture();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_Stop: " + std::string(e.what()));
    }
}

// Pause or resume image acquisition
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

// ============================================================================
// IMAGE ACQUISITION FUNCTIONS
// ============================================================================

// Get the latest captured frame
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

// ============================================================================
// CAMERA PARAMETER SETTERS
// ============================================================================

// Set camera exposure time
bool Camera_SetExposure(int microsec) {
    try {
        return CameraController::GetInstance().SetExposure(microsec);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetExposure: " + std::string(e.what()));
        return false;
    }
}

// Set camera gain
bool Camera_SetGain(float gain) {
    try {
        return CameraController::GetInstance().SetGain(gain);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetGain: " + std::string(e.what()));
        return false;
    }
}

// Set region of interest (ROI)
bool Camera_SetROI(int offsetX, int offsetY, int width, int height) {
    try {
        return CameraController::GetInstance().SetROI(offsetX, offsetY, width, height);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetROI: " + std::string(e.what()));
        return false;
    }
}

// Set camera frame rate
bool Camera_SetFrameRate(float fps) {
    try {
        return CameraController::GetInstance().SetFrameRate(fps);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetFrameRate: " + std::string(e.what()));
        return false;
    }
}

// Enable or disable trigger mode
bool Camera_SetTriggerMode(bool enabled) {
    try {
        return CameraController::GetInstance().SetTriggerMode(enabled);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetTriggerMode: " + std::string(e.what()));
        return false;
    }
}

// ============================================================================
// CAMERA PARAMETER GETTERS
// ============================================================================

// Get current exposure time
int Camera_GetExposure() {
    try {
        return CameraController::GetInstance().GetExposure();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetExposure: " + std::string(e.what()));
        return 0;
    }
}

// Get current gain value
float Camera_GetGain() {
    try {
        return CameraController::GetInstance().GetGain();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetGain: " + std::string(e.what()));
        return 0.0f;
    }
}

// Get current ROI settings
bool Camera_GetROI(int* offsetX, int* offsetY, int* width, int* height) {
    try {
        auto& controller = CameraController::GetInstance();

        // Return full frame dimensions (offsets always 0 in current implementation)
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

// Get current frame rate
float Camera_GetFrameRate() {
    try {
        return CameraController::GetInstance().GetFrameRate();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetFrameRate: " + std::string(e.what()));
        return 0.0f;
    }
}

// Get current camera state
int Camera_GetState() {
    try {
        return static_cast<int>(CameraController::GetInstance().GetState());
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetState: " + std::string(e.what()));
        return 0;
    }
}

// ============================================================================
// STATISTICS AND MONITORING
// ============================================================================

// Get capture statistics
bool Camera_GetStatistics(unsigned long* totalFrames, unsigned long* droppedFrames,
    double* averageFPS, double* minFPS, double* maxFPS) {
    try {
        auto stats = CameraController::GetInstance().GetStatistics();

        // Copy statistics to output parameters
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

// Reset capture statistics
void Camera_ResetStatistics() {
    try {
        CameraController::GetInstance().ResetStatistics();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_ResetStatistics: " + std::string(e.what()));
    }
}

// ============================================================================
// CALLBACK MANAGEMENT
// ============================================================================

// Register a callback for camera events
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

// Unregister a previously registered callback
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

// Clear all registered callbacks
void Camera_ClearCallbacks() {
    try {
        CameraController::GetInstance().ClearCallbacks();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_ClearCallbacks: " + std::string(e.what()));
    }
}

// ============================================================================
// LOGGING CONTROL
// ============================================================================

// Set the logging level
void Camera_SetLogLevel(int level) {
    try {
        Logger::GetInstance().SetLogLevel(static_cast<LogLevel>(level));
    }
    catch (const std::exception& e) {
        // Silent fail - can't log if logger is broken
        LOG_ERROR("Exception in Camera_SetLogLevel: " + std::string(e.what()));
    }
}

// Flush log buffer to file
void Camera_FlushLog() {
    try {
        Logger::GetInstance().Flush();
    }
    catch (const std::exception& e) {
        // Silent fail - can't log if logger is broken
        LOG_ERROR("Exception in Camera_FlushLog: " + std::string(e.what()));
    }
}

// ============================================================================
// IMAGE SAVING FUNCTIONS
// ============================================================================

// Save a snapshot of the current frame
bool Camera_SaveSnapshot(const char* filename, int format, int quality) {
    try {
        auto& controller = CameraController::GetInstance();

        // Get current frame dimensions
        int width, height;
        width = controller.GetWidth();
        height = controller.GetHeight();

        if (width <= 0 || height <= 0) {
            LOG_ERROR("Invalid image dimensions");
            return false;
        }

        // Allocate buffer for frame data
        size_t bufferSize = width * height;
        std::unique_ptr<unsigned char[]> buffer(new unsigned char[bufferSize]);

        // Get current frame
        bool result = controller.GetFrame(buffer.get(), static_cast<int>(bufferSize), width, height);

        // Save to file if frame was retrieved successfully
        if (result) {
            result = ImageSaver::SaveGrayscaleImage(buffer.get(), width, height, filename,
                static_cast<ImageFormat>(format), quality);
        }

        return result;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SaveSnapshot: " + std::string(e.what()));
        return false;
    }
}

// Save provided frame data to file
bool Camera_SaveCurrentFrame(unsigned char* buffer, int bufferSize,
    int* width, int* height, const char* filename,
    int format, int quality) {
    try {
        // Validate parameters
        if (!buffer || !width || !height || !filename) {
            LOG_ERROR("Invalid parameters");
            return false;
        }

        // Save image using ImageSaver utility
        return ImageSaver::SaveGrayscaleImage(buffer, *width, *height, filename,
            static_cast<ImageFormat>(format), quality);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SaveCurrentFrame: " + std::string(e.what()));
        return false;
    }
}

// ============================================================================
// DEFAULT SETTINGS MANAGEMENT
// ============================================================================

// Get default camera settings
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

// Get default snapshot settings
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

// Set default snapshot settings
void Camera_SetSnapshotDefaults(const SnapshotDefaults* defaults) {
    if (!defaults) {
        LOG_ERROR("Invalid parameter: defaults is nullptr");
        return;
    }

    // Validate format
    if (defaults->format < 0 || defaults->format > 1) {
        LOG_ERROR("Invalid format: must be 0 (PNG) or 1 (JPG)");
        return;
    }

    // Validate quality
    if (defaults->quality < 1 || defaults->quality > 100) {
        LOG_ERROR("Invalid quality: must be between 1 and 100");
        return;
    }

    g_snapshotDefaults = *defaults;
    LOG_INFO("Updated snapshot defaults: format=" +
        std::to_string(defaults->format) + ", quality=" +
        std::to_string(defaults->quality));
}

// Save snapshot using default settings
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

// ============================================================================
// REAL-TIME BALL DETECTION FUNCTIONS
// ============================================================================

// Enable/disable real-time ball detection
bool Camera_EnableRealtimeDetection(bool enable) {
    try {
        return CameraController::GetInstance().EnableRealtimeDetection(enable);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_EnableRealtimeDetection: " + std::string(e.what()));
        return false;
    }
}

// Check if real-time detection is enabled
bool Camera_IsRealtimeDetectionEnabled() {
    try {
        return CameraController::GetInstance().IsRealtimeDetectionEnabled();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_IsRealtimeDetectionEnabled: " + std::string(e.what()));
        return false;
    }
}

// Set callback for real-time detection results
void Camera_SetRealtimeDetectionCallback(RealtimeDetectionCallback callback, void* userContext) {
    try {
        // Store callback globally
        g_realtimeDetectionCallback = callback;
        g_realtimeDetectionContext = userContext;

        // Pass to controller
        CameraController::GetInstance().SetRealtimeDetectionCallback(callback, userContext);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetRealtimeDetectionCallback: " + std::string(e.what()));
    }
}

// Get the most recent detection result
bool Camera_GetLastDetectionResult(RealtimeDetectionResult* result) {
    try {
        if (!result) {
            LOG_ERROR("Invalid parameter: result is nullptr");
            return false;
        }

        return CameraController::GetInstance().GetLastDetectionResult(result);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetLastDetectionResult: " + std::string(e.what()));
        return false;
    }
}

// Set ROI scale for real-time detection
bool Camera_SetRealtimeDetectionROI(float roiScale) {
    try {
        // Validate ROI scale
        if (roiScale <= 0.0f || roiScale > 1.0f) {
            LOG_ERROR("Invalid ROI scale: " + std::to_string(roiScale));
            return false;
        }

        return CameraController::GetInstance().SetRealtimeDetectionROI(roiScale);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetRealtimeDetectionROI: " + std::string(e.what()));
        return false;
    }
}

// Set downscale factor for real-time detection
bool Camera_SetRealtimeDetectionDownscale(int factor) {
    try {
        // Validate downscale factor
        if (factor < 1 || factor > 4) {
            LOG_ERROR("Invalid downscale factor: " + std::to_string(factor));
            return false;
        }

        return CameraController::GetInstance().SetRealtimeDetectionDownscale(factor);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetRealtimeDetectionDownscale: " + std::string(e.what()));
        return false;
    }
}

// Set maximum candidates for real-time detection
bool Camera_SetRealtimeDetectionMaxCandidates(int maxCandidates) {
    try {
        // Validate candidate count
        if (maxCandidates < 1 || maxCandidates > 50) {
            LOG_ERROR("Invalid max candidates: " + std::to_string(maxCandidates));
            return false;
        }

        return CameraController::GetInstance().SetRealtimeDetectionMaxCandidates(maxCandidates);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetRealtimeDetectionMaxCandidates: " + std::string(e.what()));
        return false;
    }
}

// Get real-time detection statistics
void Camera_GetRealtimeDetectionStats(int* processedFrames, double* avgProcessingTimeMs, double* detectionFPS) {
    try {
        CameraController::GetInstance().GetRealtimeDetectionStats(processedFrames, avgProcessingTimeMs, detectionFPS);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetRealtimeDetectionStats: " + std::string(e.what()));
    }
}


// ============================================================================
// BALL STATE TRACKING FUNCTIONS
// ============================================================================

// Enable/disable ball state tracking
bool Camera_EnableBallStateTracking(bool enable) {
    try {
        return CameraController::GetInstance().EnableBallStateTracking(enable);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_EnableBallStateTracking: " + std::string(e.what()));
        return false;
    }
}

// Check if ball state tracking is enabled
bool Camera_IsBallStateTrackingEnabled() {
    try {
        return CameraController::GetInstance().IsBallStateTrackingEnabled();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_IsBallStateTrackingEnabled: " + std::string(e.what()));
        return false;
    }
}

// Get current ball state
BallState Camera_GetBallState() {
    try {
        return CameraController::GetInstance().GetBallState();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetBallState: " + std::string(e.what()));
        return BallState::NOT_DETECTED;
    }
}

// Get detailed ball state information
bool Camera_GetBallStateInfo(BallStateInfo* info) {
    try {
        if (!info) {
            LOG_ERROR("Invalid parameter: info is nullptr");
            return false;
        }

        return CameraController::GetInstance().GetBallStateInfo(info);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetBallStateInfo: " + std::string(e.what()));
        return false;
    }
}

// Set ball state tracking configuration
bool Camera_SetBallStateConfig(const BallStateConfig* config) {
    try {
        if (!config) {
            LOG_ERROR("Invalid parameter: config is nullptr");
            return false;
        }

        return CameraController::GetInstance().SetBallStateConfig(*config);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetBallStateConfig: " + std::string(e.what()));
        return false;
    }
}

// Get current ball state tracking configuration
bool Camera_GetBallStateConfig(BallStateConfig* config) {
    try {
        if (!config) {
            LOG_ERROR("Invalid parameter: config is nullptr");
            return false;
        }

        *config = CameraController::GetInstance().GetBallStateConfig();
        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetBallStateConfig: " + std::string(e.what()));
        return false;
    }
}

// Set callback for ball state changes
void Camera_SetBallStateChangeCallback(BallStateChangeCallback callback, void* userContext) {
    try {
        CameraController::GetInstance().SetBallStateChangeCallback(callback, userContext);

        if (callback) {
            LOG_INFO("Ball state change callback registered");
        }
        else {
            LOG_INFO("Ball state change callback cleared");
        }
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetBallStateChangeCallback: " + std::string(e.what()));
    }
}

// Reset ball state tracking
void Camera_ResetBallStateTracking() {
    try {
        //CameraController::GetInstance().ResetBallStateTracking();
        LOG_INFO("Ball state tracking reset");
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_ResetBallStateTracking: " + std::string(e.what()));
    }
}

// Get ball state as string
const char* Camera_GetBallStateString(BallState state) {
    switch (state) {
    case BallState::NOT_DETECTED:
        return "NOT_DETECTED";
    case BallState::MOVING:
        return "MOVING";
    case BallState::STABILIZING:
        return "STABILIZING";
    case BallState::READY:
        return "READY";
    case BallState::STOPPED:
        return "STOPPED";
    default:
        return "UNKNOWN";
    }
}

// Get time in current state (milliseconds)
int Camera_GetTimeInCurrentState() {
    try {
        return CameraController::GetInstance().GetTimeInCurrentState();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetTimeInCurrentState: " + std::string(e.what()));
        return 0;
    }
}

// Check if ball is in stable state (READY or STOPPED)
bool Camera_IsBallStable() {
    try {
        return CameraController::GetInstance().IsBallStable();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_IsBallStable: " + std::string(e.what()));
        return false;
    }
}


// ============================================================================
// CONTINUOUS CAPTURE FUNCTIONS (Conditional Compilation)
// ============================================================================

#ifdef ENABLE_CONTINUOUS_CAPTURE

// Set continuous capture configuration
bool Camera_SetContinuousCaptureConfig(const ContinuousCaptureConfig* config) {
    try {
        if (!config) {
            LOG_ERROR("Invalid parameter: config is nullptr");
            return false;
        }

        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            LOG_ERROR("Continuous capture manager not available");
            return false;
        }

        // Validate configuration parameters
        if (config->durationSeconds <= 0 || config->durationSeconds > 3600) {
            LOG_ERROR("Invalid duration: " + std::to_string(config->durationSeconds));
            return false;
        }

        if (config->imageFormat < 0 || config->imageFormat > 1) {
            LOG_ERROR("Invalid format: " + std::to_string(config->imageFormat));
            return false;
        }

        if (config->jpgQuality < 1 || config->jpgQuality > 100) {
            LOG_ERROR("Invalid quality: " + std::to_string(config->jpgQuality));
            return false;
        }

        // Apply configuration
        captureManager->SetConfig(*config);

        LOG_INFO("Continuous capture config set successfully");
        LOG_INFO("  Duration: " + std::to_string(config->durationSeconds) + "s");
        LOG_INFO("  Format: " + std::to_string(config->imageFormat) + " (0=PNG, 1=JPG)");
        LOG_INFO("  Ball Detection: " + std::string(config->enableBallDetection ? "Enabled" : "Disabled"));

        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetContinuousCaptureConfig: " + std::string(e.what()));
        return false;
    }
}

// Get current continuous capture configuration
bool Camera_GetContinuousCaptureConfig(ContinuousCaptureConfig* config) {
    try {
        if (!config) {
            LOG_ERROR("Invalid parameter: config is nullptr");
            return false;
        }

        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            LOG_ERROR("Continuous capture manager not available");
            return false;
        }

        *config = captureManager->GetConfig();
        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetContinuousCaptureConfig: " + std::string(e.what()));
        return false;
    }
}

// Start continuous capture session
bool Camera_StartContinuousCapture() {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            LOG_ERROR("Continuous capture manager not available");
            return false;
        }

        // Camera must be actively capturing
        if (CameraController::GetInstance().GetState() != CameraState::CAPTURING) {
            LOG_ERROR("Camera must be capturing to start continuous capture");
            return false;
        }

        // Reset if previous session completed or errored
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

// Stop ongoing continuous capture
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

// Check if continuous capture is active
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

// Get continuous capture state
int Camera_GetContinuousCaptureState() {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            return 0;  // IDLE state
        }
        return static_cast<int>(captureManager->GetState());
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetContinuousCaptureState: " + std::string(e.what()));
        return 0;
    }
}

// Get results from continuous capture session
bool Camera_GetContinuousCaptureResult(int* totalFrames, int* savedFrames,
    int* droppedFrames, double* duration,
    char* folderPath, int pathSize) {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            return false;
        }

        auto result = captureManager->GetResult();

        // Copy results to output parameters
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

// Set callback for continuous capture progress
void Camera_SetContinuousCaptureProgressCallback(void(*callback)(int currentFrame, double elapsedSeconds, int state)) {
    try {
        g_continuousProgressCallback = callback;

        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (captureManager) {
            if (callback) {
                // Wrap C callback in lambda for C++ interface
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

// Get default continuous capture configuration
void Camera_GetContinuousCaptureDefaults(ContinuousCaptureConfig* config) {
    if (!config) {
        LOG_ERROR("Invalid parameter: config is nullptr");
        return;
    }

    *config = g_continuousCaptureConfig;
    LOG_DEBUG("Retrieved continuous capture defaults: duration=" +
        std::to_string(config->durationSeconds) + "s, format=" +
        std::to_string(config->imageFormat) + ", enableBallDetection=" +
        std::to_string(config->enableBallDetection));
}

// Set default continuous capture configuration
void Camera_SetContinuousCaptureDefaults(const ContinuousCaptureConfig* config) {
    if (!config) {
        LOG_ERROR("Invalid parameter: config is nullptr");
        return;
    }

    // Validate configuration
    if (config->durationSeconds <= 0 || config->durationSeconds > 3600) {
        LOG_ERROR("Invalid duration: " + std::to_string(config->durationSeconds));
        return;
    }

    if (config->imageFormat < 0 || config->imageFormat > 1) {
        LOG_ERROR("Invalid format: " + std::to_string(config->imageFormat));
        return;
    }

    if (config->jpgQuality < 1 || config->jpgQuality > 100) {
        LOG_ERROR("Invalid quality: " + std::to_string(config->jpgQuality));
        return;
    }

    // Update global defaults
    g_continuousCaptureConfig = *config;

    LOG_INFO("Updated continuous capture defaults:");
    LOG_INFO("  Duration: " + std::to_string(g_continuousCaptureConfig.durationSeconds) + " seconds");
    LOG_INFO("  Format: " + std::to_string(g_continuousCaptureConfig.imageFormat) + " (0=PNG, 1=JPG)");
    LOG_INFO("  Quality: " + std::to_string(g_continuousCaptureConfig.jpgQuality));
    LOG_INFO("  Async Save: " + std::string(g_continuousCaptureConfig.useAsyncSave ? "Yes" : "No"));
    LOG_INFO("  Ball Detection: " + std::string(g_continuousCaptureConfig.enableBallDetection ? "Yes" : "No"));
    LOG_INFO("  Save Original: " + std::string(g_continuousCaptureConfig.saveOriginalImages ? "Yes" : "No"));
    LOG_INFO("  Save Detection: " + std::string(g_continuousCaptureConfig.saveDetectionImages ? "Yes" : "No"));
    LOG_INFO("  Save Debug Images: " + std::string(g_continuousCaptureConfig.saveBallDetectorDebugImages ? "Yes" : "No"));
}

// Start continuous capture with default settings
bool Camera_StartContinuousCaptureWithDefaults() {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            LOG_ERROR("Continuous capture manager not available");
            return false;
        }

        // Check and reset state if needed
        ContinuousCaptureState currentState = captureManager->GetState();
        LOG_INFO("Current capture state before start: " +
            std::to_string(static_cast<int>(currentState)));

        if (currentState == ContinuousCaptureState::COMPLETED ||
            currentState == ContinuousCaptureState::kERROR) {
            LOG_INFO("Resetting previous capture session");
            captureManager->Reset();
        }

        // Apply default configuration
        if (!Camera_SetContinuousCaptureConfig(&g_continuousCaptureConfig)) {
            LOG_ERROR("Failed to set continuous capture config with defaults");
            return false;
        }

        // Start capture
        bool result = captureManager->StartCapture();

        if (result) {
            LOG_INFO("Continuous capture started with defaults - Duration: " +
                std::to_string(g_continuousCaptureConfig.durationSeconds) + "s");
        }
        else {
            LOG_ERROR("Failed to start continuous capture");
        }

        return result;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_StartContinuousCaptureWithDefaults: " +
            std::string(e.what()));
        return false;
    }
}

// Get ball detection results from continuous capture
bool Camera_GetContinuousCaptureDetectionResult(int* framesWithBalls, int* totalBallsDetected,
    float* averageConfidence, char* detectionFolder, int folderSize) {
    try {
        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (!captureManager) {
            LOG_ERROR("Continuous capture manager not available");
            return false;
        }

        auto result = captureManager->GetDetectionResult();

        // Copy detection results
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

// Enable/disable ball detector debug images
bool Camera_SetBallDetectorDebugImages(bool enable) {
    try {
        // Update global default
        g_continuousCaptureConfig.saveBallDetectorDebugImages = enable;

        auto* captureManager = CameraController::GetInstance().GetContinuousCaptureManager();
        if (captureManager) {
            // Update current configuration if not capturing
            if (!captureManager->IsCapturing()) {
                auto config = captureManager->GetConfig();
                config.saveBallDetectorDebugImages = enable;
                captureManager->SetConfig(config);
            }

            // Update ball detector directly
            auto* ballDetector = captureManager->GetBallDetector();
            if (ballDetector) {
                auto params = ballDetector->GetParameters();
                params.saveIntermediateImages = enable;
                ballDetector->SetParameters(params);

                LOG_INFO("Ball detector debug images " +
                    std::string(enable ? "enabled" : "disabled"));
            }
        }

        // Also update real-time ball detector in CameraController
        auto& controller = CameraController::GetInstance();
        if (controller.IsRealtimeDetectionEnabled()) {
            // Get the realtime ball detector through the controller's internal mechanism
            // Since we can't directly access m_realtimeBallDetector from here,
            // we need to add a method to CameraController or use another approach
            LOG_INFO("Note: Real-time ball detector debug settings may need manual update");
        }

        LOG_INFO("Ball detector debug images default set to: " +
            std::string(enable ? "enabled" : "disabled"));
        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetBallDetectorDebugImages: " + std::string(e.what()));
        return false;
    }
}

// Check if debug image saving is enabled
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


// ============================================================================
// DYNAMIC ROI FUNCTIONS
// ============================================================================

bool Camera_EnableDynamicROI(bool enable) {
    try {
        return CameraController::GetInstance().EnableDynamicROI(enable);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_EnableDynamicROI: " + std::string(e.what()));
        return false;
    }
}

bool Camera_IsDynamicROIEnabled() {
    try {
        return CameraController::GetInstance().IsDynamicROIEnabled();
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_IsDynamicROIEnabled: " + std::string(e.what()));
        return false;
    }
}

bool Camera_SetDynamicROIConfig(const DynamicROIConfig* config) {
    try {
        if (!config) {
            LOG_ERROR("Invalid parameter: config is nullptr");
            return false;
        }

        // Validate configuration
        if (config->roiSizeMultiplier <= 0 || config->minROISize <= 0 ||
            config->maxROISize <= 0 || config->minROISize > config->maxROISize) {
            LOG_ERROR("Invalid dynamic ROI configuration");
            return false;
        }

        return CameraController::GetInstance().SetDynamicROIConfig(*config);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_SetDynamicROIConfig: " + std::string(e.what()));
        return false;
    }
}

bool Camera_GetDynamicROIConfig(DynamicROIConfig* config) {
    try {
        if (!config) {
            LOG_ERROR("Invalid parameter: config is nullptr");
            return false;
        }

        *config = CameraController::GetInstance().GetDynamicROIConfig();
        return true;
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetDynamicROIConfig: " + std::string(e.what()));
        return false;
    }
}

bool Camera_GetDynamicROIInfo(DynamicROIInfo* info) {
    try {
        if (!info) {
            LOG_ERROR("Invalid parameter: info is nullptr");
            return false;
        }

        return CameraController::GetInstance().GetDynamicROIInfo(info);
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_GetDynamicROIInfo: " + std::string(e.what()));
        return false;
    }
}

void Camera_ResetDynamicROI() {
    try {
        CameraController::GetInstance().ResetDynamicROI();
        LOG_INFO("Dynamic ROI reset");
    }
    catch (const std::exception& e) {
        LOG_ERROR("Exception in Camera_ResetDynamicROI: " + std::string(e.what()));
    }
}

#endif // ENABLE_CONTINUOUS_CAPTURE