#pragma once
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <deque>
#include <chrono>
#include <algorithm>

#include "XIMEASensor.h"

#define ENABLE_PERFORMANCE_PROFILING

namespace cv {
    class Mat;
}

// Forward declaration
class CameraController;

// Ball information structure
struct BallInfo {
    cv::Point2f center;
    float radius;
    float confidence;
    int frameIndex;
    float circularity;
    float brightness;
    float edgeStrength;
    float motionScore;

    BallInfo() : radius(0.0f), confidence(0.0f), frameIndex(0),
        circularity(1.0f), brightness(0.0f),
        edgeStrength(0.0f), motionScore(0.0f) {
    }
};

// Detection result structure
struct BallDetectionResult {
    bool found;
    std::vector<BallInfo> balls;
    std::string errorMessage;

    BallDetectionResult() : found(false) {}
};

// Thread-safe ball detector
class BallDetector {
public:
    // Detection parameters
    struct DetectionParams {
        // Circle Detection Parameters
        int minRadius;
        int maxRadius;
        float minCircularity;

        // Hough Circle Parameters
        double dp;
        double minDist;
        double param1;
        double param2;

        // Image Thresholding Parameters
        int brightnessThreshold;
        bool useAdaptiveThreshold;
        int adaptiveBlockSize;
        double adaptiveConstant;

        // Validation Parameters
        bool useColorFilter;
        bool useCircularityCheck;
        float contrastThreshold;
        bool detectMultiple;
        float edgeThreshold;

        // Preprocessing Parameters
        bool skipPreprocessing;
        float shadowEnhanceFactor;
        bool useEnhanceShadows;
        bool useMorphology;
        bool useNormalization;
        bool useCLAHE;
        double claheClipLimit;

        // Detection Method Control
        bool useContourDetection;
        bool useHoughDetection;
        bool useThresholding;
        bool useTemplateMatching;

        // Performance Parameters
        bool fastMode;
        bool useROI;
        float roiScale;
        int downscaleFactor;
        bool useParallel;
        int maxCandidates;
        int processingThreads;
        bool useGPU;

        // Debug Parameters
        bool saveIntermediateImages;
        std::string debugOutputDir;
        bool enableProfiling;

        // Tracking Parameters
        bool enableTracking;
        float maxTrackingDistance;
        int trackingHistorySize;

        DetectionParams();
    };

    // Performance metrics
    struct PerformanceMetrics {
        double totalDetectionTime_ms;
        double contextInitTime_ms;
        double parameterCopyTime_ms;
        double matCreationTime_ms;
        double roiExtractionTime_ms;
        double downscaleTime_ms;
        double preprocessingTime_ms;
        double filterTime_ms;
        double claheTime_ms;
        double shadowEnhancementTime_ms;
        double sharpenTime_ms;
        double normalizationTime_ms;
        double edgeDetectionTime_ms;
        double thresholdingTime_ms;
        double morphologyTime_ms;
        double contourDetectionTime_ms;
        double houghDetectionTime_ms;
        double templateMatchingTime_ms;
        double candidateEvaluationTime_ms;
        double trackingTime_ms;
        double selectionTime_ms;
        double resultFilteringTime_ms;
        double confidenceCalculationTime_ms;
        double imagesSavingTime_ms;
        double synchronizationTime_ms;
        double metricsUpdateTime_ms;
        double memoryPoolTime_ms;
        int candidatesFound;
        int candidatesEvaluated;
        int candidatesRejected;
        bool ballDetected;
        float averageConfidence;

        // 메모리 풀 통계
        size_t poolSize;
        size_t poolInUse;
        float poolHitRate;

        void Reset();
    };

    // Tracking information
    struct TrackingInfo {
        std::vector<BallInfo> history;
        cv::Point2f predictedCenter;
        float predictedRadius;
        int consecutiveDetections;
        int trackId;

        TrackingInfo() : predictedRadius(0.0f), consecutiveDetections(0), trackId(-1) {}
    };

private:
    // Camera type for algorithm selection
    enum class CameraType {
        XIMEA = 0,
        HIKVISION = 1,
        UNKNOWN = -1
    };

    CameraType m_currentCameraType;
    mutable std::mutex m_cameraTypeMutex;
    CameraController* m_cameraController;  // 카메라 컨트롤러 참조

    // Thread safety
    mutable std::mutex m_paramsMutex;
    mutable std::mutex m_metricsMutex;
    DetectionParams m_params;
    mutable PerformanceMetrics m_lastMetrics;

    // Thread-local context for detection
    struct DetectionContext {
        PerformanceMetrics metrics;
        cv::Mat tempMat1, tempMat2, tempMat3;
        std::vector<cv::Vec3f> tempCandidates;

        DetectionContext() {
            metrics.Reset();
        }
    };

    static thread_local std::unique_ptr<DetectionContext> t_context;

    // Implementation with enhanced memory pool
    class Impl;
    std::unique_ptr<Impl> pImpl;
    bool m_performanceProfilingEnabled;

    // Tracking state
    std::unordered_map<int, TrackingInfo> m_tracks;
    int m_nextTrackId;

    // Template matching
    cv::Mat m_ballTemplate;
    bool m_templateInitialized;

    // Private methods
    void InitializeDefaultParams();
    void UpdateCameraType();  // 새로 추가
    BallDetectionResult DetectBallStandard(const unsigned char* imageData, int width, int height, int frameIndex);
    BallDetectionResult DetectBallIR_Internal(const unsigned char* imageData, int width, int height, int frameIndex);

    std::vector<cv::Vec3f> detectByContours(const cv::Mat& binary,
        const cv::Mat& grayImage,
        float downscaleFactor);
    std::vector<cv::Vec3f> detectByContoursOptimized(const cv::Mat& binary,
        const cv::Mat& grayImage,
        float downscaleFactor);
    std::vector<cv::Vec3f> detectByTemplate(const cv::Mat& image, float downscaleFactor);
    std::vector<cv::Vec3f> detectByTemplateOptimized(const cv::Mat& image,
        float downscaleFactor);
    void selectBestBalls(const std::vector<BallInfo>& validBalls,
        BallDetectionResult& result);
    void selectBestBallsOptimized(const std::vector<BallInfo>& validBalls,
        BallDetectionResult& result);
    void updateTracking(const BallDetectionResult& result);
    void updateTrackingOptimized(std::vector<BallInfo>& validBalls);
    void predictNextPosition(TrackingInfo& track);
    float calculateMotionConsistency(const BallInfo& ball, const TrackingInfo& track);
    void saveDebugImages(const unsigned char* imageData, int width, int height,
        int frameIndex, const BallDetectionResult& result);
    void saveDebugImagesAsync(const unsigned char* imageData, int width, int height,
        int frameIndex, const BallDetectionResult& result);


    // White ball detection mode
    bool m_whiteBallMode;
    WhiteBallDetectionConfig m_whiteBallConfig;
    mutable std::mutex m_whiteBallConfigMutex;
    
    // White ball detection methods
    BallDetectionResult DetectWhiteBallInternal(const unsigned char* imageData,
                                                int width, int height,
                                                const WhiteBallDetectionConfig& config);
    cv::Mat applyWhiteBallThreshold(const cv::Mat& grayImage, 
                                    const WhiteBallDetectionConfig& config);
    std::vector<cv::Vec3f> detectWhiteBallContours(const cv::Mat& binary,
                                                   const WhiteBallDetectionConfig& config);
    float calculateWhiteBallConfidence(const cv::Mat& image, 
                                      const cv::Vec3f& circle,
                                      const WhiteBallDetectionConfig& config);

public:
    BallDetector();
    ~BallDetector();

    // Prevent copying
    BallDetector(const BallDetector&) = delete;
    BallDetector& operator=(const BallDetector&) = delete;
    BallDetector(BallDetector&&) = delete;
    BallDetector& operator=(BallDetector&&) = delete;

    // Camera controller 설정
    void SetCameraController(CameraController* controller) {
        m_cameraController = controller;
        UpdateCameraType();
    }

    // Thread-safe parameter access
    void SetParameters(const DetectionParams& params) {
        std::lock_guard<std::mutex> lock(m_paramsMutex);
        m_params = params;
    }

    DetectionParams GetParameters() const {
        std::lock_guard<std::mutex> lock(m_paramsMutex);
        return m_params;
    }

    void ResetToDefaults();

    // Performance control
    void EnablePerformanceProfiling(bool enable);
    bool IsPerformanceProfilingEnabled() const { return m_performanceProfilingEnabled; }

    // 메모리 풀 관리 인터페이스
    void OptimizeMemoryPool();
    void GetMemoryPoolStatistics(size_t& poolSize, size_t& inUse,
        float& hitRate) const;

    // Thread-safe detection - 카메라 타입에 따라 자동 선택
    void SetCurrentCaptureFolder(const std::string& folder);
    BallDetectionResult DetectBall(const unsigned char* imageData, int width, int height, int frameIndex = 0);

    // Template management
    bool InitializeTemplate(const cv::Mat& templateImage);
    void ClearTemplate();

    // Tracking interface
    void ResetTracking();
    std::vector<TrackingInfo> GetActiveTracks() const;

    // Visualization
    bool SaveDetectionImage(const unsigned char* originalImage,
                            int width, int height,
                            const BallDetectionResult& result,
                            const std::string& outputPath,
                            bool saveAsColor = false);

    // IR camera specific
    void SetIRCameraOptimizedParams();
    std::vector<cv::Vec3f> detectByContoursForIR(const cv::Mat& binary, const cv::Mat& grayImage);
    BallDetectionResult DetectBallIR(const unsigned char* imageData, int width, int height, int frameIndex);

    // Thread-safe performance metrics
    PerformanceMetrics GetLastPerformanceMetrics() const {
        std::lock_guard<std::mutex> lock(m_metricsMutex);
        return m_lastMetrics;
    }

    // Calibration
    bool CalibrateForBallSize(const std::vector<cv::Mat>& sampleImages,
        float knownBallDiameter_mm);

    // White ball detection interface
    void EnableWhiteBallMode(bool enable) { m_whiteBallMode = enable; }
    bool IsWhiteBallMode() const { return m_whiteBallMode; }
    void SetWhiteBallConfig(const WhiteBallDetectionConfig& config);
    BallDetectionResult DetectWhiteBall(const unsigned char* imageData,
                                       int width, int height,
                                       const WhiteBallDetectionConfig& config);};