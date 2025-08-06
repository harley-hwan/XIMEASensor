#pragma once
#include <xiApi.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <queue>
#include <condition_variable>
#include <array>
#include "XIMEASensor.h" 
#include "IXIMEACallback.h"
#include "Logger.h"
#include "BallDetector.h"
#ifdef ENABLE_CONTINUOUS_CAPTURE
#include "ContinuousCaptureManager.h"
#endif

#define PYTHON1300_WIDTH    1280
#define PYTHON1300_HEIGHT   960
#define PYTHON1300_MAX_FPS  210

struct CameraStatistics {
    unsigned long totalFrames;
    unsigned long droppedFrames;
    double averageFPS;
    double minFPS;
    double maxFPS;
    std::chrono::steady_clock::time_point startTime;

    void Reset() {
        totalFrames = 0;
        droppedFrames = 0;
        averageFPS = 0.0;
        minFPS = 9999.0;
        maxFPS = 0.0;
        startTime = std::chrono::steady_clock::now();
    }
};

class CameraController {
private:
    // singleton
    static std::unique_ptr<CameraController> instance;
    static std::mutex instanceMutex;

    // XIMEA handle
    HANDLE xiH;

    std::thread captureThread;
    std::atomic<bool> isRunning;
    std::atomic<bool> isPaused;

    // 더블 버퍼링을 위한 구조체
    struct ImageBuffer {
        std::unique_ptr<unsigned char[]> data;
        int width = 0;
        int height = 0;
        std::atomic<bool> ready{ false };
    };

    std::array<ImageBuffer, 2> buffers;
    std::atomic<int> writeIndex{ 0 };
    std::atomic<int> readIndex{ 1 };
    std::mutex bufferSwapMutex;

    // 기존 버퍼는 호환성을 위해 유지 (차후 제거)
    std::mutex frameMutex;
    unsigned char* frameBuffer;
    unsigned char* workingBuffer;

    // camera param
    int width;
    int height;
    int currentExposure;
    float currentGain;
    float currentFrameRate;

    // callbacks
    std::mutex callbackMutex;
    std::vector<IXIMEACallback*> callbacks;

    std::atomic<CameraState> currentState;
    CameraStatistics stats;
    std::chrono::steady_clock::time_point lastFrameTime;

#ifdef ENABLE_CONTINUOUS_CAPTURE
    std::unique_ptr<ContinuousCaptureManager> m_continuousCapture;
#endif

    std::atomic<int> deviceNotReadyCount;
    static const int MAX_DEVICE_NOT_READY_ERRORS = 5;

    // Real-time ball detection
    std::atomic<bool> m_realtimeDetectionEnabled;
    std::unique_ptr<BallDetector> m_realtimeBallDetector;
    RealtimeDetectionCallback m_realtimeCallback;
    void* m_realtimeCallbackContext;

    std::mutex m_detectionQueueMutex;
    std::queue<std::pair<std::vector<unsigned char>, FrameInfo>> m_detectionQueue;
    std::thread m_detectionThread;
    std::atomic<bool> m_detectionThreadRunning;
    std::condition_variable m_detectionCV;

    // Real-time detection statistics
    std::atomic<int> m_realtimeProcessedFrames;
    double m_realtimeTotalProcessingTime;
    std::chrono::steady_clock::time_point m_realtimeStartTime;
    RealtimeDetectionResult m_lastDetectionResult;
    std::mutex m_realtimeStatsMutex;
    std::mutex m_lastResultMutex;

    // Ball state tracking
    struct BallTrackingData {
        BallState currentState;
        BallState previousState;
        float lastPositionX;
        float lastPositionY;
        std::chrono::steady_clock::time_point lastDetectionTime;
        std::chrono::steady_clock::time_point stableStartTime;
        std::chrono::steady_clock::time_point lastStateChangeTime;
        int consecutiveDetections;
        bool isTracking;
        int missedDetectionCount;  // 탐지 실패 카운터
        std::chrono::steady_clock::time_point lastMissedTime;  // 마지막 실패 시간

        BallTrackingData()
            : currentState(BallState::NOT_DETECTED)
            , previousState(BallState::NOT_DETECTED)
            , lastPositionX(0.0f)
            , lastPositionY(0.0f)
            , consecutiveDetections(0)
            , isTracking(false)
            , missedDetectionCount(0) { 
        }
    };

    // Ball state tracking members
    std::atomic<bool> m_ballStateTrackingEnabled;
    BallTrackingData m_ballTracking;
    BallStateConfig m_ballStateConfig;
    BallStateChangeCallback m_ballStateCallback;
    void* m_ballStateCallbackContext;
    mutable std::mutex m_ballStateMutex;

    // Dynamic ROI members
    std::atomic<bool> m_dynamicROIEnabled;
    DynamicROIConfig m_dynamicROIConfig;
    DynamicROIInfo m_dynamicROIInfo;
    mutable std::mutex m_dynamicROIMutex;
    std::chrono::steady_clock::time_point m_lastROIUpdateTime;
    cv::Rect m_currentROI;
    bool m_usingDynamicROI;

    CameraController();

    void CaptureLoop();

    void NotifyFrameReceived(const FrameInfo& info);
    void NotifyStateChanged(CameraState newState);
    void NotifyError(CameraError error, const std::string& message);
    void NotifyPropertyChanged(const std::string& property, const std::string& value);

    void UpdateStatistics(bool frameReceived);
    std::string GetXiApiErrorString(XI_RETURN error);

    void RealtimeDetectionWorker();
    void ProcessRealtimeDetection(const unsigned char* data, int width, int height, int frameIndex);

    // Ball state tracking methods
    void UpdateBallState(const RealtimeDetectionResult* result);
    float CalculateDistance(float x1, float y1, float x2, float y2);
    void NotifyBallStateChanged(BallState newState, BallState oldState, const BallStateInfo& info);
    void ResetBallTracking();

    // Dynamic ROI methods
    void UpdateDynamicROI(const RealtimeDetectionResult* result);
    cv::Rect CalculateDynamicROI(float centerX, float centerY, float radius);
    void ApplyDynamicROI(const cv::Rect& roi);
    void ClearDynamicROI();

public:
    ~CameraController();

    // singleton
    static CameraController& GetInstance();
    static void Destroy();

    // camera control
    bool OpenCamera(int deviceIndex);
    void CloseCamera();
    bool StartCapture();
    void StopCapture();
    void PauseCapture(bool pause);

    bool GetFrame(unsigned char* buffer, int bufferSize, int& outWidth, int& outHeight);

    // camera settings
    bool SetExposure(int microsec);
    bool SetGain(float gain);
    bool SetROI(int offsetX, int offsetY, int width, int height);
    bool SetFrameRate(float fps);
    bool SetTriggerMode(bool enabled);

    // getters
    int GetExposure() const { return currentExposure; }
    float GetGain() const { return currentGain; }
    int GetWidth() const { return width; }
    int GetHeight() const { return height; }
    float GetFrameRate();
    CameraState GetState() const { return currentState.load(); }
    CameraStatistics GetStatistics() const { return stats; }
    void ResetStatistics() { stats.Reset(); }

#ifdef ENABLE_CONTINUOUS_CAPTURE
    ContinuousCaptureManager* GetContinuousCaptureManager() {
        return m_continuousCapture.get();
    }
#else
    void* GetContinuousCaptureManager() { return nullptr; }
#endif

    // callback
    void RegisterCallback(IXIMEACallback* callback);
    void UnregisterCallback(IXIMEACallback* callback);
    void ClearCallbacks();

    int GetConnectedDeviceCount();
    bool GetDeviceInfo(int index, std::string& name, std::string& serial);

    // Real-time ball detection
    bool EnableRealtimeDetection(bool enable);
    bool IsRealtimeDetectionEnabled() const { return m_realtimeDetectionEnabled.load(); }
    void SetRealtimeDetectionCallback(RealtimeDetectionCallback callback, void* context);
    bool GetLastDetectionResult(RealtimeDetectionResult* result);

    bool SetRealtimeDetectionROI(float roiScale);
    bool SetRealtimeDetectionDownscale(int factor);
    bool SetRealtimeDetectionMaxCandidates(int maxCandidates);
    void GetRealtimeDetectionStats(int* processedFrames, double* avgProcessingTimeMs, double* detectionFPS);

    // Ball state tracking
    bool EnableBallStateTracking(bool enable);
    bool IsBallStateTrackingEnabled() const { return m_ballStateTrackingEnabled.load(); }
    BallState GetBallState() const;
    bool GetBallStateInfo(BallStateInfo* info) const;
    void GetBallStateInfoInternal(BallStateInfo* info) const;
    bool SetBallStateConfig(const BallStateConfig& config);
    BallStateConfig GetBallStateConfig() const;
    void SetBallStateChangeCallback(BallStateChangeCallback callback, void* context);
    void ResetBallStateTracking();
    int GetTimeInCurrentState() const;
    bool IsBallStable() const;

    // Dynamic ROI
    bool EnableDynamicROI(bool enable);
    bool IsDynamicROIEnabled() const { return m_dynamicROIEnabled.load(); }
    bool SetDynamicROIConfig(const DynamicROIConfig& config);
    DynamicROIConfig GetDynamicROIConfig() const;
    bool GetDynamicROIInfo(DynamicROIInfo* info) const;
    void ResetDynamicROI();
    cv::Rect GetCurrentROI() const {
        std::lock_guard<std::mutex> lock(m_dynamicROIMutex);
        return m_currentROI;
    }
};