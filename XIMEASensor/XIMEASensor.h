#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif

#include "IXIMEACallback.h"

// Camera default settings
namespace CameraDefaults {
    XIMEASENSOR_API extern const int EXPOSURE_US;      // Default exposure time in microseconds
    XIMEASENSOR_API extern const float GAIN_DB;        // Default gain in dB
    XIMEASENSOR_API extern const float FRAMERATE_FPS;  // Default framerate in FPS

    // Camera limits
    XIMEASENSOR_API extern const int MIN_EXPOSURE_US;
    XIMEASENSOR_API extern const int MAX_EXPOSURE_US;
    XIMEASENSOR_API extern const float MIN_GAIN_DB;
    XIMEASENSOR_API extern const float MAX_GAIN_DB;
    XIMEASENSOR_API extern const float MIN_FPS;
    XIMEASENSOR_API extern const float MAX_FPS;

    // Continuous capture defaults
    XIMEASENSOR_API extern const double CONTINUOUS_CAPTURE_DURATION;
    XIMEASENSOR_API extern const int CONTINUOUS_CAPTURE_FORMAT;
    XIMEASENSOR_API extern const int CONTINUOUS_CAPTURE_QUALITY;
    XIMEASENSOR_API extern const bool CONTINUOUS_CAPTURE_ASYNC_SAVE;

    // Single snapshot defaults
    XIMEASENSOR_API extern const int SNAPSHOT_FORMAT;
    XIMEASENSOR_API extern const int SNAPSHOT_QUALITY;
}

struct ContinuousCaptureDefaults {
    double duration;
    int format;
    int quality;
    bool asyncSave;
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

    // Continuous capture API
    XIMEASENSOR_API bool Camera_SetContinuousCaptureConfig(double duration, int format, int quality, bool asyncSave);
    XIMEASENSOR_API bool Camera_StartContinuousCapture();
    XIMEASENSOR_API void Camera_StopContinuousCapture();
    XIMEASENSOR_API bool Camera_IsContinuousCapturing();
    XIMEASENSOR_API int Camera_GetContinuousCaptureState();
    XIMEASENSOR_API bool Camera_GetContinuousCaptureResult(int* totalFrames, int* savedFrames, int* droppedFrames, double* duration, char* folderPath, int pathSize);
    XIMEASENSOR_API void Camera_SetContinuousCaptureProgressCallback(void(*callback)(int currentFrame, double elapsedSeconds, int state));

    // Get default values functions
    XIMEASENSOR_API void Camera_GetDefaultSettings(int* exposureUs, float* gainDb, float* fps);

    XIMEASENSOR_API void Camera_GetContinuousCaptureDefaults(ContinuousCaptureDefaults* defaults);
    XIMEASENSOR_API void Camera_SetContinuousCaptureDefaults(const ContinuousCaptureDefaults* defaults);
    XIMEASENSOR_API void Camera_GetSnapshotDefaults(SnapshotDefaults* defaults);
    XIMEASENSOR_API void Camera_SetSnapshotDefaults(const SnapshotDefaults* defaults);

    // Simplified API
    XIMEASENSOR_API bool Camera_StartContinuousCaptureWithDefaults();
    XIMEASENSOR_API bool Camera_SaveSnapshotWithDefaults(const char* filename);
}