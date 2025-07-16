#pragma once
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>
#include <condition_variable>
#include <functional>
#include <memory>

enum class ContinuousCaptureState {
    IDLE = 0,
    CAPTURING,
    STOPPING,
    COMPLETED,
    kERROR
};

struct ContinuousCaptureConfig {
    double durationSeconds = 1.0;
    int imageFormat = 0;            // 0: PNG, 1: JPG
    int jpgQuality = 90;
    bool createMetadata = true;
    bool useAsyncSave = true;
    std::string baseFolder = ".";

    // Golf ball detection options
    bool enableGolfBallDetection = false;
    bool saveOriginalImages = true;
    bool saveDetectionImages = true;
};

struct ContinuousCaptureResult {
    bool success = false;
    int totalFrames = 0;
    int savedFrames = 0;
    int droppedFrames = 0;
    double actualDuration = 0.0;
    std::string folderPath;
    std::string errorMessage;
};

typedef std::function<void(int currentFrame, double elapsedSeconds, ContinuousCaptureState state)> ContinuousCaptureProgressCallback;

// Forward declaration
class GolfBallDetector;
struct GolfBallDetectionResult;

struct ContinuousCaptureDetectionResult {
    int framesWithGolfBall = 0;
    int totalBallsDetected = 0;
    float averageConfidence = 0.0f;
    std::string detectionFolder;
};

class ContinuousCaptureManager {
private:
    ContinuousCaptureConfig m_config;
    std::atomic<ContinuousCaptureState> m_state;
    std::atomic<bool> m_isCapturing;
    std::atomic<int> m_frameCount;
    std::atomic<int> m_savedCount;
    std::atomic<int> m_droppedCount;
    std::atomic<int> m_processingCount;  // 현재 처리 중인 작업 수
    mutable std::mutex m_detectionMutex;  // mutable 추가

    std::chrono::steady_clock::time_point m_startTime;
    std::string m_captureFolder;
    std::string m_originalFolder;    // 원본 이미지 폴더
    std::string m_detectionFolder;   // 검출 결과 이미지 폴더
    double m_actualDuration;

    // Golf ball detection
    std::unique_ptr<GolfBallDetector> m_golfBallDetector;
    ContinuousCaptureDetectionResult m_detectionResult;

    // Async save structure
    struct SaveItem {
        std::vector<unsigned char> data;
        std::string filename;
        int width;
        int height;
        int frameIndex;  // 검출을 위한 프레임 인덱스 추가
    };

    std::thread m_saveThread;
    std::queue<SaveItem> m_saveQueue;
    std::mutex m_queueMutex;
    std::condition_variable m_queueCV;
    std::condition_variable m_completionCV;  // 작업 완료 대기용
    std::atomic<bool> m_saveThreadRunning;

    // Buffer pool for performance
    std::queue<std::vector<unsigned char>> m_bufferPool;
    std::mutex m_poolMutex;

    ContinuousCaptureProgressCallback m_progressCallback;
    std::mutex m_callbackMutex;

    bool CreateCaptureFolder();
    void SaveThreadWorker();
    void SaveFrameAsync(const unsigned char* data, int width, int height);
    void SaveMetadata();
    std::vector<unsigned char> GetBufferFromPool(size_t size);
    void ReturnBufferToPool(std::vector<unsigned char>&& buffer);
    bool WaitForSaveCompletion(int timeoutSeconds = 30);  // 저장 완료 대기

    // Golf ball detection
    void ProcessGolfBallDetection(const SaveItem& item);
    void SaveDetectionMetadata();

public:
    ContinuousCaptureManager();
    ~ContinuousCaptureManager();

    void SetConfig(const ContinuousCaptureConfig& config);

    bool StartCapture();
    void StopCapture();
    bool IsCapturing() const { return m_isCapturing.load(); }
    ContinuousCaptureState GetState() const { return m_state.load(); }

    void ProcessFrame(const unsigned char* data, int width, int height);

    ContinuousCaptureResult GetResult() const;
    int GetCurrentFrameCount() const { return m_frameCount.load(); }
    double GetElapsedTime() const;
    ContinuousCaptureDetectionResult GetDetectionResult() const;

    void SetProgressCallback(ContinuousCaptureProgressCallback callback);
    void Reset();
};