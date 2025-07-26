#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif

#include "IXIMEACallback.h"

// Camera default settings
namespace CameraDefaults {
    XIMEASENSOR_API extern const int EXPOSURE_US;
    XIMEASENSOR_API extern const float GAIN_DB;
    XIMEASENSOR_API extern const float FRAMERATE_FPS;

    // Camera limits
    XIMEASENSOR_API extern const int MIN_EXPOSURE_US;
    XIMEASENSOR_API extern const int MAX_EXPOSURE_US;
    XIMEASENSOR_API extern const float MIN_GAIN_DB;
    XIMEASENSOR_API extern const float MAX_GAIN_DB;
    XIMEASENSOR_API extern const float MIN_FPS;
    XIMEASENSOR_API extern const float MAX_FPS;
}

// Unified continuous capture configuration structure
struct XIMEASENSOR_API ContinuousCaptureConfig {
    // Version for future compatibility
    unsigned int version;

    // Basic capture settings
    double durationSeconds;
    int imageFormat;        // 0: PNG, 1: JPG
    int jpgQuality;        // 1-100
    bool useAsyncSave;

    // Ball detection settings
    bool enableBallDetection;
    bool saveOriginalImages;
    bool saveDetectionImages;
    bool saveBallDetectorDebugImages;

    // Metadata and folder settings
    bool createMetadata;
    char baseFolder[256];
    char debugImagePath[256];

    // Reserved for future expansion
    char reserved[256];

    // Constructor with default values
    ContinuousCaptureConfig();

    // Static method to get default configuration
    static ContinuousCaptureConfig GetDefault();

    // Validation method
    bool Validate() const;

    // Copy utilities for safe string handling
    void SetBaseFolder(const char* folder);
    void SetDebugImagePath(const char* path);
};

struct SnapshotDefaults {
    int format;
    int quality;
};

extern "C" {
    XIMEASENSOR_API bool Camera_Initialize(const char* logPath = nullptr, int logLevel = 1);
    XIMEASENSOR_API void Camera_Shutdown();

    XIMEASENSOR_API int Camera_GetDeviceCount();
    XIMEASENSOR_API bool Camera_GetDeviceInfo(int index, char* name, int nameSize, char* serial, int serialSize);

    XIMEASENSOR_API bool Camera_Open(int deviceIndex);
    XIMEASENSOR_API void Camera_Close();
    XIMEASENSOR_API bool Camera_Start();
    XIMEASENSOR_API void Camera_Stop();
    XIMEASENSOR_API bool Camera_Pause(bool pause);

    XIMEASENSOR_API bool Camera_GetFrame(unsigned char* buffer, int bufferSize, int* width, int* height);

    XIMEASENSOR_API bool Camera_SetExposure(int microsec);
    XIMEASENSOR_API bool Camera_SetGain(float gain);
    XIMEASENSOR_API bool Camera_SetROI(int offsetX, int offsetY, int width, int height);
    XIMEASENSOR_API bool Camera_SetFrameRate(float fps);
    XIMEASENSOR_API bool Camera_SetTriggerMode(bool enabled);

    XIMEASENSOR_API int Camera_GetExposure();
    XIMEASENSOR_API float Camera_GetGain();
    XIMEASENSOR_API bool Camera_GetROI(int* offsetX, int* offsetY, int* width, int* height);
    XIMEASENSOR_API float Camera_GetFrameRate();
    XIMEASENSOR_API int Camera_GetState();

    XIMEASENSOR_API bool Camera_GetStatistics(unsigned long* totalFrames, unsigned long* droppedFrames, double* averageFPS, double* minFPS, double* maxFPS);
    XIMEASENSOR_API void Camera_ResetStatistics();

    XIMEASENSOR_API bool Camera_RegisterCallback(IXIMEACallback* callback);
    XIMEASENSOR_API bool Camera_UnregisterCallback(IXIMEACallback* callback);
    XIMEASENSOR_API void Camera_ClearCallbacks();

    XIMEASENSOR_API void Camera_SetLogLevel(int level);
    XIMEASENSOR_API void Camera_FlushLog();

    XIMEASENSOR_API bool Camera_SaveSnapshot(const char* filename, int format = 0, int quality = 90);
    XIMEASENSOR_API bool Camera_SaveCurrentFrame(unsigned char* buffer, int bufferSize, int* width, int* height, const char* filename, int format = 0, int quality = 90);

    // New unified continuous capture API
    XIMEASENSOR_API bool Camera_SetContinuousCaptureConfig(const ContinuousCaptureConfig* config);
    XIMEASENSOR_API bool Camera_GetContinuousCaptureConfig(ContinuousCaptureConfig* config);
    XIMEASENSOR_API bool Camera_GetDefaultContinuousCaptureConfig(ContinuousCaptureConfig* config);

    XIMEASENSOR_API bool Camera_StartContinuousCapture();
    XIMEASENSOR_API void Camera_StopContinuousCapture();
    XIMEASENSOR_API bool Camera_IsContinuousCapturing();
    XIMEASENSOR_API int Camera_GetContinuousCaptureState();
    XIMEASENSOR_API bool Camera_GetContinuousCaptureResult(int* totalFrames, int* savedFrames, int* droppedFrames, double* duration, char* folderPath, int pathSize);
    XIMEASENSOR_API void Camera_SetContinuousCaptureProgressCallback(void(*callback)(int currentFrame, double elapsedSeconds, int state));

    XIMEASENSOR_API void Camera_GetDefaultSettings(int* exposureUs, float* gainDb, float* fps);

    // Snapshot defaults (kept for backward compatibility)
    XIMEASENSOR_API void Camera_GetSnapshotDefaults(SnapshotDefaults* defaults);
    XIMEASENSOR_API void Camera_SetSnapshotDefaults(const SnapshotDefaults* defaults);
    XIMEASENSOR_API bool Camera_SaveSnapshotWithDefaults(const char* filename);

    // Ball detection results
    XIMEASENSOR_API bool Camera_GetContinuousCaptureDetectionResult(int* framesWithBalls, int* totalBallsDetected, float* averageConfidence, char* detectionFolder, int folderSize);

    // Debug control
    XIMEASENSOR_API bool Camera_SetBallDetectorDebugImages(bool enable);
    XIMEASENSOR_API bool Camera_GetBallDetectorDebugImages();
}