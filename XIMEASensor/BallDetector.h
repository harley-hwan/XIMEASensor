#pragma once
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>
#include <memory>

#define ENABLE_DEBUG_IMAGE_SAVING       // save_debug_images or not     // 이거 지우는 방법으로 구현해야됨. 이거 주석하면 디버그 이미지 저장 안됨
//#define ENABLE_DETECTION_IMAGE_SAVING
#define ENABLE_PERFORMANCE_PROFILING    // performance_report log or not

namespace cv {
    class Mat;
}

struct BallInfo {
    cv::Point2f center;
    float radius;
    float confidence;     // Detection confidence (0.0 ~ 1.0)
    int frameIndex;

    float circularity;    // 0.0 ~ 1.0
    float brightness;     // Average brightness (0 ~ 255)
    float contrast;

    BallInfo() : radius(0.0f), confidence(0.0f), frameIndex(0),
        circularity(1.0f), brightness(0.0f), contrast(0.0f) {
    }
};

struct BallDetectionResult {
    bool found;
    std::vector<BallInfo> balls;
    std::string errorMessage;

    BallDetectionResult() : found(false) {}
};

class BallDetector {
public:
    struct DetectionParams {
        // Hough Circle detection params
        double dp;             // Inverse ratio of accumulator resolution
        double minDist;        // Minimum distance between detected centers
        double param1;         // Higher threshold for Canny edge detection
        double param2;         // Circle detection threshold (lower = more permissive)
        int minRadius;         // pixels
        int maxRadius;         // pixels

        int brightnessThreshold;
        float minCircularity;      // 0.0 ~ 1.0
        float contrastThreshold;

        // Detection opt
        bool useColorFilter;
        bool useCircularityCheck;
        bool useHoughGradientAlt;
        bool detectMultiple;
        bool useMorphology;
        bool useAdaptiveThreshold;

        // Camera specific opt
        bool correctPerspective;
        bool enhanceShadows;
        float shadowEnhanceFactor;  // 0.0 ~ 1.0

        // Debug opt
        bool saveIntermediateImages;
        std::string debugOutputDir;

        bool fastMode;
        bool useROI;
        float roiScale;             // 0.5 ~ 1.0
        int downscaleFactor;        // 1 ~ 4
        bool useParallel;           // parallel with TBB
        int maxCandidates;

        bool skipPreprocessing;
        int edgeThreshold;
        bool useCache;
        int processingThreads;

        DetectionParams();
    };

    // Performance metrics struct
    struct PerformanceMetrics {
        double totalDetectionTime_ms;
        double preprocessingTime_ms;
        double houghDetectionTime_ms;
        double adaptiveThresholdTime_ms;
        double contourDetectionTime_ms;
        double candidateEvaluationTime_ms;
        double imagesSavingTime_ms;

        int candidatesFound;
        int candidatesEvaluated;
        bool ballDetected;

        void Reset() {
            totalDetectionTime_ms = 0;
            preprocessingTime_ms = 0;
            houghDetectionTime_ms = 0;
            adaptiveThresholdTime_ms = 0;
            contourDetectionTime_ms = 0;
            candidateEvaluationTime_ms = 0;
            imagesSavingTime_ms = 0;
            candidatesFound = 0;
            candidatesEvaluated = 0;
            ballDetected = false;
        }
    };

private:
    DetectionParams m_params;

    class Impl;
    std::unique_ptr<Impl> pImpl;

    mutable PerformanceMetrics m_lastMetrics;
    bool m_performanceProfilingEnabled;

    void InitializeDefaultParams();

public:
    BallDetector();
    ~BallDetector();

    BallDetector(const BallDetector&) = delete;
    BallDetector& operator=(const BallDetector&) = delete;
    BallDetector(BallDetector&&) = delete;
    BallDetector& operator=(BallDetector&&) = delete;

    void SetParameters(const DetectionParams& params) { m_params = params; }
    DetectionParams GetParameters() const { return m_params; }
    void ResetToDefaults();

    void EnablePerformanceProfiling(bool enable);
    bool IsPerformanceProfilingEnabled() const { return m_performanceProfilingEnabled; }

    void SetCurrentCaptureFolder(const std::string& folder);

    BallDetectionResult DetectBall(const unsigned char* imageData, int width, int height, int frameIndex = 0);

    bool SaveDetectionImage(const unsigned char* originalImage, int width, int height, const BallDetectionResult& result, const std::string& outputPath, bool saveAsColor = false);

    PerformanceMetrics GetLastPerformanceMetrics() const { return m_lastMetrics; }

    double EstimateProcessingTime(int width, int height) const;
};