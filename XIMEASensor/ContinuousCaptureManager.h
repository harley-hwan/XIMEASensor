#pragma once
#include "XIMEASensor.h"  // For ContinuousCaptureConfig
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

class BallDetector;
struct BallDetectionResult;

struct ContinuousCaptureDetectionResult {
    int framesWithBall = 0;
    int totalBallsDetected = 0;
    float averageConfidence = 0.0f;
    std::string detectionFolder;
};

class ContinuousCaptureManager {
private:
    // Performance report struct
    struct SessionPerformanceData {
        struct FrameTimingData {
            int frameIndex = 0;

            double totalFrameProcessingTime_ms = 0;     // ProcessBallDetection 전체 시간
            double imageLoadTime_ms = 0;                // 이미지 로드 시간
            double ballDetectionTime_ms = 0;            // BallDetector::DetectBall 전체 시간
            double saveDetectionImageTime_ms = 0;       // 검출 결과 이미지 저장 시간
            double otherOperationsTime_ms = 0;          // 기타 오버헤드

            double preprocessingTime_ms = 0;
            double houghDetectionTime_ms = 0;
            double adaptiveThresholdTime_ms = 0;
            double contourDetectionTime_ms = 0;
            double candidateEvaluationTime_ms = 0;
            double debugImagesSavingTime_ms = 0;

            int candidatesFound = 0;
            int candidatesEvaluated = 0;
            bool ballDetected = false;
        };

        std::vector<FrameTimingData> frameTimings;

        int totalFramesProcessed = 0;
        int framesWithBallDetected = 0;
        double totalProcessingTime_ms = 0.0;
        double minFrameTime_ms = std::numeric_limits<double>::max();
        double maxFrameTime_ms = 0.0;
        double avgFrameTime_ms = 0.0;

        // 알고리즘별 누적 시간
        double totalPreprocessingTime_ms = 0.0;
        double totalHoughDetectionTime_ms = 0.0;
        double totalContourDetectionTime_ms = 0.0;
        double totalCandidateEvaluationTime_ms = 0.0;
        double totalDebugImagesSavingTime_ms = 0.0;

        // I/O 누적 시간 추가
        double totalDetectionImageSavingTime_ms = 0.0;
        double totalOtherOperationsTime_ms = 0.0;

        void Reset() {
            frameTimings.clear();
            totalFramesProcessed = 0;
            framesWithBallDetected = 0;
            totalProcessingTime_ms = 0.0;
            minFrameTime_ms = std::numeric_limits<double>::max();
            maxFrameTime_ms = 0.0;
            avgFrameTime_ms = 0.0;
            totalPreprocessingTime_ms = 0.0;
            totalHoughDetectionTime_ms = 0.0;
            totalContourDetectionTime_ms = 0.0;
            totalCandidateEvaluationTime_ms = 0.0;
            totalDebugImagesSavingTime_ms = 0.0;
            totalDetectionImageSavingTime_ms = 0.0;
            totalOtherOperationsTime_ms = 0.0;
        }
    };

    SessionPerformanceData m_sessionPerformance;

    static constexpr int DEFAULT_WAIT_TIMEOUT_SECONDS = 300;
    static constexpr int BUFFER_POOL_SIZE = 20;
    static constexpr int MAX_QUEUE_SIZE = 100;
    static constexpr int PROGRESS_UPDATE_INTERVAL = 10;

    ContinuousCaptureConfig m_config;
    std::atomic<ContinuousCaptureState> m_state;
    std::atomic<bool> m_isCapturing;
    std::atomic<int> m_frameCount;
    std::atomic<int> m_savedCount;
    std::atomic<int> m_droppedCount;
    std::atomic<int> m_processingCount;
    std::atomic<int> m_ballDetectionPendingCount;
    std::atomic<int> m_ballDetectionCompletedCount;
    mutable std::mutex m_detectionMutex;

    std::chrono::steady_clock::time_point m_startTime;
    std::string m_captureFolder;
    std::string m_originalFolder;
    std::string m_detectionFolder;
    double m_actualDuration;

    // Ball detection
    std::unique_ptr<BallDetector> m_ballDetector;
    ContinuousCaptureDetectionResult m_detectionResult;

    struct PerformanceStats {
        std::atomic<int> totalProcessedFrames{ 0 };
        std::atomic<int> framesWithBallDetected{ 0 };
        std::atomic<double> totalProcessingTime{ 0.0 };
        std::chrono::steady_clock::time_point startTime;

        void Reset() {
            totalProcessedFrames = 0;
            framesWithBallDetected = 0;
            totalProcessingTime = 0.0;
            startTime = std::chrono::steady_clock::now();
        }
    } m_performanceStats;

    // Async save structure
    struct SaveItem {
        std::vector<unsigned char> data;
        std::string filename;
        int width;
        int height;
        int frameIndex;
    };

    std::thread m_saveThread;
    std::queue<SaveItem> m_saveQueue;
    std::mutex m_queueMutex;
    std::condition_variable m_queueCV;
    std::condition_variable m_completionCV;
    std::atomic<bool> m_saveThreadRunning;

    // Buffer pool for performance
    std::queue<std::vector<unsigned char>> m_bufferPool;
    std::mutex m_poolMutex;

    ContinuousCaptureProgressCallback m_progressCallback;
    std::mutex m_callbackMutex;

    // Private methods
    bool CreateCaptureFolder();
    void SaveThreadWorker();
    void SaveFrameAsync(const unsigned char* data, int width, int height);
    void SaveMetadata();
    std::vector<unsigned char> GetBufferFromPool(size_t size);
    void ReturnBufferToPool(std::vector<unsigned char>&& buffer);
    bool WaitForSaveCompletion(int timeoutSeconds = 0);

    // Ball detection
    void ProcessBallDetection(const SaveItem& item);
    void SaveDetectionMetadata();

    void ConfigureBallDetectorOptimal();

public:
    ContinuousCaptureManager();
    ~ContinuousCaptureManager();

    void SetConfig(const ContinuousCaptureConfig& config);
    bool StartCapture();
    void SaveSessionPerformanceReport();
    void StopCapture();
    bool IsCapturing() const { return m_isCapturing.load(); }

    ContinuousCaptureConfig GetConfig() const { return m_config; }
    BallDetector* GetBallDetector() { return m_ballDetector.get(); }
    ContinuousCaptureState GetState() const { return m_state.load(); }

    void ProcessFrame(const unsigned char* data, int width, int height);

    ContinuousCaptureResult GetResult() const;
    int GetCurrentFrameCount() const { return m_frameCount.load(); }
    double GetElapsedTime() const;
    ContinuousCaptureDetectionResult GetDetectionResult() const;

    void SetProgressCallback(ContinuousCaptureProgressCallback callback);

    void SetBallDetectorDebugOutput(bool enable, const std::string& customPath = "");

    void Reset();
};