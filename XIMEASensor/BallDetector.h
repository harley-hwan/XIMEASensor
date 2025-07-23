// BallDetector.h - Complete Optimized Version with TBB
#pragma once
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>
#include <memory>

namespace cv {
    class Mat;
}

// Structure to hold information about a detected ball
struct BallInfo {
    cv::Point2f center;   // Ball center coordinates (in image pixels)
    float radius;         // Ball radius (in pixels)
    float confidence;     // Detection confidence (0.0 ~ 1.0)
    int frameIndex;       // Frame index of detection

    // Additional diagnostic information
    float circularity;    // Circularity metric of the detected shape
    float brightness;     // Average brightness of the ball region
    float contrast;       // Contrast between ball and background
};

struct BallDetectionResult {
    bool found;                   // Whether a ball was detected
    std::vector<BallInfo> balls;  // Detected ball information
    std::string errorMessage;     // Error message if detection failed
};

class BallDetector {
public:
    struct DetectionParams {
        // Hough Circle detection parameters
        double dp;             // Inverse ratio of accumulator resolution
        double minDist;        // Minimum distance between detected centers
        double param1;         // Higher threshold for Canny
        double param2;         // Circle detection threshold
        int minRadius;         // Minimum circle radius (pixels)
        int maxRadius;         // Maximum circle radius

        // Filtering and validation
        int brightnessThreshold;   // Brightness threshold
        float minCircularity;      // Minimum acceptable circularity
        float contrastThreshold;   // Minimum contrast from background

        // Detection options
        bool useColorFilter;        // Use brightness/color filtering
        bool useCircularityCheck;   // Use circularity check
        bool useHoughGradientAlt;   // Use alternative Hough method
        bool detectMultiple;        // Detect multiple balls
        bool useMorphology;         // Use morphological operations
        bool useAdaptiveThreshold;  // Use adaptive threshold

        // Camera specific options
        bool correctPerspective;    // Apply lens distortion correction
        bool enhanceShadows;        // Enhance shadow regions
        float shadowEnhanceFactor;  // Shadow enhancement factor

        // Debug options
        bool saveIntermediateImages;
        std::string debugOutputDir;

        // === OPTIMIZATION PARAMETERS ===
        // Performance optimization
        bool fastMode;              // Enable all speed optimizations
        bool useROI;                // Process only center region
        float roiScale;             // ROI scale (0.5-1.0)
        int downscaleFactor;        // Image downscale factor (1-4)
        bool useParallel;           // Enable parallel processing
        int maxCandidates;          // Maximum candidates to evaluate

        // Advanced optimization
        bool skipPreprocessing;     // Skip preprocessing for max speed
        int edgeThreshold;          // Edge detection threshold
        bool useCache;              // Cache intermediate results
        int processingThreads;      // Number of processing threads

        // Constructor with optimized defaults
        DetectionParams();
    };

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

    // PIMPL idiom for implementation
    class Impl;
    std::unique_ptr<Impl> pImpl;

    mutable PerformanceMetrics m_lastMetrics;
    bool m_performanceProfilingEnabled;

    void InitializeDefaultParams();

public:
    BallDetector();
    ~BallDetector();

    // Parameter management
    void SetParameters(const DetectionParams& params) { m_params = params; }
    DetectionParams GetParameters() const { return m_params; }
    void ResetToDefaults();

    // Camera calibration
    void SetCalibrationData(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

    // Performance profiling
    void EnablePerformanceProfiling(bool enable);
    bool IsPerformanceProfilingEnabled() const;

    // === MAIN DETECTION FUNCTION ===
    BallDetectionResult DetectBall(const unsigned char* imageData, int width, int height, int frameIndex = 0);

    // === OPTIMIZATION METHODS ===
    // Quick configuration methods
    void EnableFastMode(bool enable);
    void SetDownscaleFactor(int factor);
    void SetROIScale(float scale);
    void SetMaxCandidates(int maxCandidates);
    void SetProcessingThreads(int threads);

    // Advanced optimization controls
    void EnableParallelProcessing(bool enable) { m_params.useParallel = enable; }
    void EnableCaching(bool enable) { m_params.useCache = enable; }
    void SkipPreprocessing(bool skip) { m_params.skipPreprocessing = skip; }

    // Preset configurations
    void ConfigureForRealtime();     // Fastest settings for 60+ FPS
    void ConfigureForAccuracy();     // Best accuracy, slower
    void ConfigureForBalance();      // Balanced speed/accuracy

    // Visualization utilities
    bool DrawDetectionResult(unsigned char* imageData, int width, int height,
        const BallDetectionResult& result,
        cv::Scalar color = cv::Scalar(0, 255, 0),
        int thickness = 2);

    bool SaveDetectionImage(const unsigned char* originalImage, int width, int height,
        const BallDetectionResult& result,
        const std::string& outputPath);

    // Auto-tuning
    void AutoTuneParameters(const std::vector<unsigned char*>& sampleImages, int width, int height);

    // Performance analysis
    PerformanceMetrics GetLastPerformanceMetrics() const { return m_lastMetrics; }
    std::string GeneratePerformanceReport() const;

    // === UTILITY METHODS ===
    // Get estimated processing time for given image size
    double EstimateProcessingTime(int width, int height) const;

    // Check if current settings can achieve target FPS
    bool CanAchieveTargetFPS(int targetFPS, int imageWidth, int imageHeight) const;

    // Get recommended settings for target performance
    DetectionParams GetRecommendedSettings(int targetFPS, int imageWidth, int imageHeight) const;
};

// Inline optimization preset methods
inline void BallDetector::ConfigureForRealtime() {
    m_params.fastMode = true;
    m_params.useROI = true;
    m_params.roiScale = 0.6f;
    m_params.downscaleFactor = 3;
    m_params.useParallel = true;
    m_params.maxCandidates = 5;
    m_params.skipPreprocessing = false;
    m_params.enhanceShadows = false;
    m_params.useMorphology = false;
    m_params.useAdaptiveThreshold = false;
    m_params.saveIntermediateImages = false;
    m_params.useCircularityCheck = false;
    m_params.dp = 2.5;
    m_params.param1 = 150.0;
    m_params.param2 = 0.9;
}

inline void BallDetector::ConfigureForAccuracy() {
    m_params.fastMode = false;
    m_params.useROI = false;
    m_params.roiScale = 1.0f;
    m_params.downscaleFactor = 1;
    m_params.useParallel = true;
    m_params.maxCandidates = 50;
    m_params.skipPreprocessing = false;
    m_params.enhanceShadows = true;
    m_params.useMorphology = true;
    m_params.useAdaptiveThreshold = true;
    m_params.useCircularityCheck = true;
    m_params.dp = 1.0;
    m_params.param1 = 100.0;
    m_params.param2 = 0.85;
}

inline void BallDetector::ConfigureForBalance() {
    m_params.fastMode = false;
    m_params.useROI = true;
    m_params.roiScale = 0.9f;
    m_params.downscaleFactor = 1;
    m_params.useParallel = true;
    m_params.maxCandidates = 20;
    m_params.skipPreprocessing = false;
    m_params.enhanceShadows = false;
    m_params.useMorphology = false;
    m_params.useAdaptiveThreshold = false;
    m_params.useCircularityCheck = true;
    m_params.dp = 1.5;
    m_params.param1 = 120.0;
    m_params.param2 = 0.85;
}