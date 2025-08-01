#pragma once
#include <vector>
#include <string>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <unordered_map>

#define ENABLE_PERFORMANCE_PROFILING

namespace cv {
    class Mat;
}

/**
 * @brief Ball information structure containing detection results
 */
struct BallInfo {
    cv::Point2f center;       // Ball center in image coordinates
    float radius;             // Ball radius in pixels
    float confidence;         // Detection confidence score (0.0 ~ 1.0)
    int frameIndex;          // Frame number where ball was detected
    float circularity;       // Shape circularity measure (0.0 ~ 1.0)
    float brightness;        // Average brightness inside ball region (0 ~ 255)
    float edgeStrength;      // Edge response strength (added)
    float motionScore;       // Motion consistency score for tracking (added)

    BallInfo() : radius(0.0f), confidence(0.0f), frameIndex(0),
        circularity(1.0f), brightness(0.0f), edgeStrength(0.0f), motionScore(0.0f) {
    }
};

/**
 * @brief Detection result containing all found balls and status
 */
struct BallDetectionResult {
    bool found;                      // Whether any ball was detected
    std::vector<BallInfo> balls;     // List of detected balls
    std::string errorMessage;        // Error description if detection failed

    BallDetectionResult() : found(false) {}
};

/**
 * @brief Main ball detection class using computer vision techniques
 */
class BallDetector {
public:
    /**
     * @brief Detection parameters for fine-tuning the algorithm
     */
    struct DetectionParams {
        // === Circle Detection Parameters ===
        int minRadius;                      // Minimum ball radius in pixels
        int maxRadius;                      // Maximum ball radius in pixels
        float minCircularity;               // Minimum circularity threshold (0.0 ~ 1.0)

        // === Hough Circle Parameters (Fallback Method) ===
        double dp;                          // Inverse accumulator resolution ratio
        double minDist;                     // Minimum distance between detected centers
        double param1;                      // Canny edge upper threshold
        double param2;                      // Circle detection threshold

        // === Image Thresholding Parameters ===
        int brightnessThreshold;            // Global threshold value (0 = Otsu's method)
        bool useAdaptiveThreshold;          // Enable adaptive thresholding
        int adaptiveBlockSize;              // Block size for adaptive threshold (added)
        double adaptiveConstant;            // Constant for adaptive threshold (added)

        // === Validation Parameters ===
        bool useColorFilter;                // Enable brightness-based filtering
        bool useCircularityCheck;           // Enable circularity validation
        float contrastThreshold;            // Minimum contrast threshold
        bool detectMultiple;                // Enable multi-ball detection
        float edgeThreshold;                // Minimum edge strength (added)

        // === Preprocessing Parameters ===
        bool skipPreprocessing;             // Skip all preprocessing steps
        float shadowEnhanceFactor;          // Shadow enhancement factor (0.0 ~ 1.0)
        bool useEnhanceShadows;             // Enable shadow region enhancement
        bool useMorphology;                 // Enable morphological operations
        bool useNormalization;              // Enable histogram normalization
        bool useCLAHE;                      // Enable CLAHE for contrast (added)
        double claheClipLimit;              // CLAHE clip limit (added)

        // === Detection Method Control ===
        bool useContourDetection;           // Enable contour-based detection
        bool useHoughDetection;             // Enable Hough circle detection
        bool useThresholding;               // Enable binary thresholding
        bool useTemplateMatching;           // Enable template matching (added)

        // === Performance Parameters ===
        bool fastMode;                      // Enable fast processing mode
        bool useROI;                        // Enable region of interest
        float roiScale;                     // ROI scale factor (0.5 ~ 1.0)
        int downscaleFactor;                // Image downscale factor (1 ~ 4)
        bool useParallel;                   // Enable TBB parallel processing
        int maxCandidates;                  // Maximum candidate circles
        int processingThreads;              // Number of processing threads
        bool useGPU;                        // Enable GPU acceleration if available (added)

        // === Debug Parameters ===
        bool saveIntermediateImages;        // Save intermediate processing images
        std::string debugOutputDir;         // Debug output directory path
        bool enableProfiling;               // Enable detailed profiling (added)

        // === Tracking Parameters (added) ===
        bool enableTracking;                // Enable temporal consistency
        float maxTrackingDistance;          // Maximum tracking distance
        int trackingHistorySize;            // Number of frames to track

        DetectionParams();
    };

    /**
     * @brief Performance metrics for profiling
     */
    struct PerformanceMetrics {
        // Overall timing
        double totalDetectionTime_ms;

        // Preprocessing stage timings
        double roiExtractionTime_ms;
        double downscaleTime_ms;
        double preprocessingTime_ms;
        double claheTime_ms;                // Added

        // Detection stage timings
        double thresholdingTime_ms;
        double morphologyTime_ms;
        double contourDetectionTime_ms;
        double houghDetectionTime_ms;
        double templateMatchingTime_ms;      // Added
        double candidateEvaluationTime_ms;

        // Post-processing timings
        double trackingTime_ms;              // Added
        double imagesSavingTime_ms;

        // Statistics
        int candidatesFound;
        int candidatesEvaluated;
        int candidatesRejected;              // Added
        bool ballDetected;
        float averageConfidence;             // Added

        void Reset();
    };

    /**
     * @brief Ball tracking information for temporal consistency
     */
    struct TrackingInfo {
        std::vector<BallInfo> history;       // Historical ball positions
        cv::Point2f predictedCenter;         // Predicted center for next frame
        float predictedRadius;               // Predicted radius for next frame
        int consecutiveDetections;           // Number of consecutive detections
        int trackId;                         // Unique track identifier

        TrackingInfo() : predictedRadius(0.0f), consecutiveDetections(0), trackId(-1) {}
    };

private:
    DetectionParams m_params;
    class Impl;
    std::unique_ptr<Impl> pImpl;
    mutable PerformanceMetrics m_lastMetrics;
    bool m_performanceProfilingEnabled;

    // Tracking state
    std::unordered_map<int, TrackingInfo> m_tracks;
    int m_nextTrackId;

    // Template matching cache
    cv::Mat m_ballTemplate;
    bool m_templateInitialized;

    void InitializeDefaultParams();

    // Core detection methods
    std::vector<cv::Vec3f> detectByContours(const cv::Mat& binary,
        const cv::Mat& grayImage,
        float downscaleFactor);
    std::vector<cv::Vec3f> detectByTemplate(const cv::Mat& image,
        float downscaleFactor);

    // Helper functions
    void selectBestBalls(const std::vector<BallInfo>& validBalls,
        BallDetectionResult& result);
    void updateTracking(const BallDetectionResult& result);
    void predictNextPosition(TrackingInfo& track);
    float calculateMotionConsistency(const BallInfo& ball, const TrackingInfo& track);

    // Debug and visualization
    void saveDebugImages(const unsigned char* imageData, int width, int height,
        int frameIndex, const BallDetectionResult& result);
    void logPerformanceMetrics(int frameIndex) const;

public:
    BallDetector();
    ~BallDetector();

    // Prevent copying and moving
    BallDetector(const BallDetector&) = delete;
    BallDetector& operator=(const BallDetector&) = delete;
    BallDetector(BallDetector&&) = delete;
    BallDetector& operator=(BallDetector&&) = delete;

    // === Configuration Methods ===
    void SetParameters(const DetectionParams& params) { m_params = params; }
    DetectionParams GetParameters() const { return m_params; }
    void ResetToDefaults();

    // === Performance Control ===
    void EnablePerformanceProfiling(bool enable);
    bool IsPerformanceProfilingEnabled() const { return m_performanceProfilingEnabled; }

    // === Detection Interface ===
    void SetCurrentCaptureFolder(const std::string& folder);
    BallDetectionResult DetectBall(const unsigned char* imageData,
        int width, int height,
        int frameIndex = 0);

    // === Template Management ===
    bool InitializeTemplate(const cv::Mat& templateImage);
    void ClearTemplate();

    // === Tracking Interface ===
    void ResetTracking();
    std::vector<TrackingInfo> GetActiveTracks() const;

    // === Visualization ===
    bool SaveDetectionImage(const unsigned char* originalImage,
        int width, int height,
        const BallDetectionResult& result,
        const std::string& outputPath,
        bool saveAsColor = false);

    // === Performance Analysis ===
    PerformanceMetrics GetLastPerformanceMetrics() const { return m_lastMetrics; }
    double EstimateProcessingTime(int width, int height) const;

    // === Calibration Support ===
    bool CalibrateForBallSize(const std::vector<cv::Mat>& sampleImages,
        float knownBallDiameter_mm);
};