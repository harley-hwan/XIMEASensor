#pragma once

#ifdef XIMEASENSOR_EXPORTS
#define XIMEASENSOR_API __declspec(dllexport)
#else
#define XIMEASENSOR_API __declspec(dllimport)
#endif

#include "IXIMEACallback.h"
#include <string>

struct XIMEASENSOR_API ContinuousCaptureConfig {
    double durationSeconds;
    int imageFormat;            // 0: PNG, 1: JPG
    int jpgQuality;
    bool createMetadata;
    bool useAsyncSave;
    std::string baseFolder;

    bool enableBallDetection;
    bool saveOriginalImages;
    bool saveDetectionImages;
    bool saveBallDetectorDebugImages;
    std::string debugImagePath;

    ContinuousCaptureConfig()
        : durationSeconds(1.0)
        , imageFormat(0)
        , jpgQuality(90)
        , createMetadata(true)
        , useAsyncSave(true)
        , baseFolder(".")
        , enableBallDetection(true)
        , saveOriginalImages(true)
        , saveDetectionImages(true)
        , saveBallDetectorDebugImages(true)
        , debugImagePath("") {
    }

    ContinuousCaptureConfig(double duration, int format, int quality, bool metadata,
        bool asyncSave, const std::string& folder,
        bool ballDetection, bool saveOriginal,
        bool saveDetection, bool saveDebug,
        const std::string& debugPath = "")
        : durationSeconds(duration)
        , imageFormat(format)
        , jpgQuality(quality)
        , createMetadata(metadata)
        , useAsyncSave(asyncSave)
        , baseFolder(folder)
        , enableBallDetection(ballDetection)
        , saveOriginalImages(saveOriginal)
        , saveDetectionImages(saveDetection)
        , saveBallDetectorDebugImages(saveDebug)
        , debugImagePath(debugPath) {
    }

    static ContinuousCaptureConfig CreateDefault() {
        return ContinuousCaptureConfig(1.0, 0, 90, true, true, ".",
            true, true, true, true, "");
    }
};


namespace CameraDefaults {
    XIMEASENSOR_API extern const int EXPOSURE_US;      // microseconds
    XIMEASENSOR_API extern const float GAIN_DB;        // dB
    XIMEASENSOR_API extern const float FRAMERATE_FPS;  // FPS

    XIMEASENSOR_API extern const int MIN_EXPOSURE_US;
    XIMEASENSOR_API extern const int MAX_EXPOSURE_US;
    XIMEASENSOR_API extern const float MIN_GAIN_DB;
    XIMEASENSOR_API extern const float MAX_GAIN_DB;
    XIMEASENSOR_API extern const float MIN_FPS;
    XIMEASENSOR_API extern const float MAX_FPS;

    XIMEASENSOR_API extern const int SNAPSHOT_FORMAT;
    XIMEASENSOR_API extern const int SNAPSHOT_QUALITY;
}

struct SnapshotDefaults {
    int format;
    int quality;
};


// 2025-07-28: realtime ball detect result
struct XIMEASENSOR_API RealtimeBallInfo {
    float centerX;
    float centerY;
    float radius;
    float confidence;
    int frameIndex;
};

struct XIMEASENSOR_API RealtimeDetectionResult {
    bool ballFound;
    int ballCount;
    RealtimeBallInfo balls[5];  // 최대 5개까지
    double detectionTimeMs;
};

typedef void(*RealtimeDetectionCallback)(const RealtimeDetectionResult* result, void* userContext);

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

    // Continuous capture API - simplified with ContinuousCaptureConfig
    XIMEASENSOR_API bool Camera_SetContinuousCaptureConfig(const ContinuousCaptureConfig* config);
    XIMEASENSOR_API bool Camera_GetContinuousCaptureConfig(ContinuousCaptureConfig* config);
    XIMEASENSOR_API bool Camera_StartContinuousCapture();
    XIMEASENSOR_API void Camera_StopContinuousCapture();
    XIMEASENSOR_API bool Camera_IsContinuousCapturing();
    XIMEASENSOR_API int Camera_GetContinuousCaptureState();
    XIMEASENSOR_API bool Camera_GetContinuousCaptureResult(int* totalFrames, int* savedFrames, int* droppedFrames, double* duration, char* folderPath, int pathSize);
    XIMEASENSOR_API void Camera_SetContinuousCaptureProgressCallback(void(*callback)(int currentFrame, double elapsedSeconds, int state));

    // Default settings API
    XIMEASENSOR_API void Camera_GetDefaultSettings(int* exposureUs, float* gainDb, float* fps);
    XIMEASENSOR_API void Camera_GetContinuousCaptureDefaults(ContinuousCaptureConfig* config);
    XIMEASENSOR_API void Camera_SetContinuousCaptureDefaults(const ContinuousCaptureConfig* config);
    XIMEASENSOR_API void Camera_GetSnapshotDefaults(SnapshotDefaults* defaults);
    XIMEASENSOR_API void Camera_SetSnapshotDefaults(const SnapshotDefaults* defaults);

    // Simplified API using defaults
    XIMEASENSOR_API bool Camera_StartContinuousCaptureWithDefaults();
    XIMEASENSOR_API bool Camera_SaveSnapshotWithDefaults(const char* filename);

    // Ball detection results
    XIMEASENSOR_API bool Camera_GetContinuousCaptureDetectionResult(int* framesWithBalls, int* totalBallsDetected, float* averageConfidence, char* detectionFolder, int folderSize);

    // Debug settings
    XIMEASENSOR_API bool Camera_SetBallDetectorDebugImages(bool enable);
    XIMEASENSOR_API bool Camera_GetBallDetectorDebugImages();


    // 실시간 볼 검출 API 추가
    XIMEASENSOR_API bool Camera_EnableRealtimeDetection(bool enable);
    XIMEASENSOR_API bool Camera_IsRealtimeDetectionEnabled();
    XIMEASENSOR_API void Camera_SetRealtimeDetectionCallback(RealtimeDetectionCallback callback, void* userContext);
    XIMEASENSOR_API bool Camera_GetLastDetectionResult(RealtimeDetectionResult* result);

    // 실시간 검출 파라미터 설정
    XIMEASENSOR_API bool Camera_SetRealtimeDetectionROI(float roiScale);
    XIMEASENSOR_API bool Camera_SetRealtimeDetectionDownscale(int factor);
    XIMEASENSOR_API bool Camera_SetRealtimeDetectionMaxCandidates(int maxCandidates);
    XIMEASENSOR_API void Camera_GetRealtimeDetectionStats(int* processedFrames, double* avgProcessingTimeMs, double* detectionFPS);
}