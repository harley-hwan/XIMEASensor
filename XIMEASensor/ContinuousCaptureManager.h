#pragma once

#ifdef ENABLE_CONTINUOUS_CAPTURE

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
    // Performance report struct 내부에 추가
    struct SessionPerformanceData {
        struct FrameTimingData {
            int frameIndex = 0;

            // 전체 프레임 처리 시간
            double totalFrameProcessingTime_ms = 0;

            // I/O 관련 시간
            double imageLoadTime_ms = 0;
            double saveOriginalImageTime_ms = 0;
            double saveDetectionImageTime_ms = 0;
            double debugImagesSavingTime_ms = 0;

            // Ball Detection 전체 시간
            double ballDetectionTime_ms = 0;

            // Ball Detection 세부 시간 - 모든 메트릭 포함
            double roiExtractionTime_ms = 0;
            double downscaleTime_ms = 0;
            double preprocessingTime_ms = 0;

            // Preprocessing 세부 시간 추가
            double filterTime_ms = 0;
            double claheTime_ms = 0;
            double shadowEnhancementTime_ms = 0;
            double sharpenTime_ms = 0;
            double normalizationTime_ms = 0;

            double thresholdingTime_ms = 0;
            double morphologyTime_ms = 0;
            double contourDetectionTime_ms = 0;
            double houghDetectionTime_ms = 0;
            double templateMatchingTime_ms = 0;
            double candidateEvaluationTime_ms = 0;
            double trackingTime_ms = 0;
            double selectionTime_ms = 0;

            // 기타 작업 시간
            double otherOperationsTime_ms = 0;
            double queueOperationsTime_ms = 0;
            double memoryOperationsTime_ms = 0;

            // Detection 통계
            int candidatesFound = 0;
            int candidatesEvaluated = 0;
            int candidatesRejected = 0;
            bool ballDetected = false;
            float averageConfidence = 0.0f;
        };

        std::vector<FrameTimingData> frameTimings;

        // 전체 세션 통계
        int totalFramesProcessed = 0;
        int framesWithBallDetected = 0;
        double totalProcessingTime_ms = 0.0;
        double minFrameTime_ms = std::numeric_limits<double>::max();
        double maxFrameTime_ms = 0.0;
        double avgFrameTime_ms = 0.0;

        // 각 단계별 누적 시간
        double totalImageLoadTime_ms = 0.0;
        double totalROIExtractionTime_ms = 0.0;
        double totalDownscaleTime_ms = 0.0;
        double totalPreprocessingTime_ms = 0.0;

        // Preprocessing 세부 누적 시간 추가
        double totalFilterTime_ms = 0.0;
        double totalCLAHETime_ms = 0.0;
        double totalShadowEnhancementTime_ms = 0.0;
        double totalSharpenTime_ms = 0.0;
        double totalNormalizationTime_ms = 0.0;

        double totalThresholdingTime_ms = 0.0;
        double totalMorphologyTime_ms = 0.0;
        double totalContourDetectionTime_ms = 0.0;
        double totalHoughDetectionTime_ms = 0.0;
        double totalTemplateMatchingTime_ms = 0.0;
        double totalCandidateEvaluationTime_ms = 0.0;
        double totalTrackingTime_ms = 0.0;
        double totalSelectionTime_ms = 0.0;
        double totalDebugImagesSavingTime_ms = 0.0;
        double totalDetectionImageSavingTime_ms = 0.0;
        double totalOriginalImageSavingTime_ms = 0.0;
        double totalOtherOperationsTime_ms = 0.0;
        double totalQueueOperationsTime_ms = 0.0;
        double totalMemoryOperationsTime_ms = 0.0;

        // 새로 추가된 누적 시간 필드들
        double totalContextInitTime_ms = 0.0;
        double totalParameterCopyTime_ms = 0.0;
        double totalMatCreationTime_ms = 0.0;
        double totalEdgeDetectionTime_ms = 0.0;
        double totalResultFilteringTime_ms = 0.0;
        double totalConfidenceCalculationTime_ms = 0.0;
        double totalSynchronizationTime_ms = 0.0;

        void Reset() {
            frameTimings.clear();
            frameTimings.reserve(100); // Pre-allocate for performance

            totalFramesProcessed = 0;
            framesWithBallDetected = 0;
            totalProcessingTime_ms = 0.0;
            minFrameTime_ms = std::numeric_limits<double>::max();
            maxFrameTime_ms = 0.0;
            avgFrameTime_ms = 0.0;

            // Reset all accumulated times
            totalImageLoadTime_ms = 0.0;
            totalROIExtractionTime_ms = 0.0;
            totalDownscaleTime_ms = 0.0;
            totalPreprocessingTime_ms = 0.0;

            // Reset preprocessing detail times
            totalFilterTime_ms = 0.0;
            totalCLAHETime_ms = 0.0;
            totalShadowEnhancementTime_ms = 0.0;
            totalSharpenTime_ms = 0.0;
            totalNormalizationTime_ms = 0.0;

            totalThresholdingTime_ms = 0.0;
            totalMorphologyTime_ms = 0.0;
            totalContourDetectionTime_ms = 0.0;
            totalHoughDetectionTime_ms = 0.0;
            totalTemplateMatchingTime_ms = 0.0;
            totalCandidateEvaluationTime_ms = 0.0;
            totalTrackingTime_ms = 0.0;
            totalSelectionTime_ms = 0.0;
            totalDebugImagesSavingTime_ms = 0.0;
            totalDetectionImageSavingTime_ms = 0.0;
            totalOriginalImageSavingTime_ms = 0.0;
            totalOtherOperationsTime_ms = 0.0;
            totalQueueOperationsTime_ms = 0.0;
            totalMemoryOperationsTime_ms = 0.0;

            // Reset new fields
            totalContextInitTime_ms = 0.0;
            totalParameterCopyTime_ms = 0.0;
            totalMatCreationTime_ms = 0.0;
            totalEdgeDetectionTime_ms = 0.0;
            totalResultFilteringTime_ms = 0.0;
            totalConfidenceCalculationTime_ms = 0.0;
            totalSynchronizationTime_ms = 0.0;
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
        int width = 0;
        int height = 0;
        int frameIndex = 0;
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

#endif
